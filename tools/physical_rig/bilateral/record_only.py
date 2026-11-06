#!/usr/bin/env python3
"""Read-only full-fidelity ROS JSONL recorder. No hardware publishers or clients."""
import argparse,json,os,time,sys
from pathlib import Path

def recording_duration(protocol):
 if protocol not in ('sweep','release','long_variable','long_variable_pilot'):raise ValueError('Unknown recording protocol')
 return 342.04 if protocol=='long_variable' else 40.

def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True)
 p.add_argument('--protocol',choices=['sweep','release','long_variable','long_variable_pilot'],default='sweep');a=p.parse_args()
 total_limit=recording_duration(a.protocol);long_mode=a.protocol in ('long_variable','long_variable_pilot')
 a.output=a.output.expanduser().resolve()
 names=['feedback','state','bilateral','targets','wire','bus']
 if not a.output.is_dir() or any((a.output/(k+'.jsonl')).exists() for k in names) or (a.output/'recorder-result.json').exists():p.error('Recorder requires an existing directory with no prior record files')
 import rclpy
 from rclpy.node import Node
 from rclpy.qos import QoSProfile,ReliabilityPolicy
 from dm_motor_interfaces.msg import MotorState,MotorFeedback,MotorSetpoint,BilateralState
 from can_bus.msg import Frame,Status
 from rosidl_runtime_py.convert import message_to_ordereddict
 rclpy.init();n=Node('rig_read_only_recorder',start_parameter_services=False)
 files={k:(a.output/(k+'.jsonl')).open('x') for k in ['feedback','state','bilateral','targets','wire','bus']};counts={k:0 for k in files};seen=set();subs=[]
 def emit(kind,data):files[kind].write(json.dumps(data,separators=(',',':'),allow_nan=False)+'\n');counts[kind]+=1
 def record(kind,msg,axis=None,topic=None):
  row=dict(host_time=time.monotonic(),**message_to_ordereddict(msg))
  if axis is not None:row['axis']=axis
  if long_mode and topic is not None:row['topic']=topic
  if long_mode and kind in ('bilateral','bus'):
   before=time.monotonic();ros_ns=n.get_clock().now().nanoseconds;after=time.monotonic()
   source_ns=msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec
   row.update(clock_before=before,clock_ros_ns=ros_ns,clock_after=after,
    source_time=(before+after)/2+(source_ns-ros_ns)*1e-9,
    source_time_uncertainty=(after-before)/2)
  emit(kind,row);seen.add((kind,axis))
 def wire(kind,m):
  row=dict(kind=kind,host_ns=time.monotonic_ns(),received_steady=m.received_steady,can_id=m.can_id,data_hex=bytes(m.data).hex(),length=m.length,fd=m.fd,extended=m.extended,rtr=m.rtr,error=m.error)
  if long_mode:row['source_stamp_ns']=m.header.stamp.sec*1000000000+m.header.stamp.nanosec
  emit('wire',row)
 q=QoSProfile(depth=1024,reliability=ReliabilityPolicy.BEST_EFFORT)
 reliable=QoSProfile(depth=2048,reliability=ReliabilityPolicy.RELIABLE)
 for axis in [1,2,3]:
  for kind,typ,suffix in [('feedback',MotorFeedback,'feedback'),('state',MotorState,'state'),('targets',MotorSetpoint,'target')]:
   topic=f'/motor_{axis}/{suffix}';quality=reliable if long_mode and axis==3 and kind=='targets' else q
   subs.append(n.create_subscription(typ,topic,lambda m,k=kind,a=axis,t=topic:record(k,m,a,t),quality))
  if long_mode and axis in (1,2):
   topic=f'/motor_{axis}/setpoint'
   subs.append(n.create_subscription(MotorSetpoint,topic,lambda m,a=axis,t=topic:record('targets',m,a,t),q))
 for kind,typ,topic in [('bilateral',BilateralState,'/bilateral_control/state'),('bus',Status,'/can/main/status')]:subs.append(n.create_subscription(typ,topic,lambda m,k=kind:record(k,m),q))
 for kind,topic in [('RX','/can/main/rx'),('TX','/can/main/tx_observed')]:subs.append(n.create_subscription(Frame,topic,lambda m,k=kind:wire(k,m),reliable if long_mode else q))
 os.set_blocking(sys.stdin.fileno(),False);boot=time.monotonic();ready=False;end=None;error='';request=b''
 try:
  while time.monotonic()-boot<total_limit:
   rclpy.spin_once(n,timeout_sec=.001)
   if not ready and all((k,a) in seen for k in ['state','feedback'] for a in [1,2,3]) and ('bilateral',None) in seen and ('bus',None) in seen:
    ready=True;print(json.dumps(dict(type='READY')),flush=True)
   try:
    data=os.read(sys.stdin.fileno(),128)
    if not data and end is None:end=time.monotonic()+.3
    elif data:
     request+=data
     if request==b'FINISH\n' and end is None:end=time.monotonic()+.3
     elif len(request)>128:raise RuntimeError('invalid input')
   except BlockingIOError:pass
   if end is not None and time.monotonic()>end:break
  else:error='Recorder duration exceeded'
 except BaseException as e:error=repr(e)
 finally:
  for f in files.values():f.close()
  result=dict(success=ready and not error,error=error,counts=counts,started=boot,finished=time.monotonic())
  if long_mode:result.update(protocol=a.protocol,max_total_duration_s=total_limit,wire_source_stamps=True,target_topics=['/motor_1/setpoint','/motor_2/setpoint','/motor_3/target'])
  with (a.output/'recorder-result.json').open('x') as out:out.write(json.dumps(result,indent=2)+'\n')
  print(json.dumps(dict(type='DONE',**result)),flush=True)
  n.destroy_node();rclpy.shutdown()
 return 0 if result['success'] else 1
if __name__=='__main__':raise SystemExit(main())
