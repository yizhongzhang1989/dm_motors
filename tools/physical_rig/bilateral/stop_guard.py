#!/usr/bin/env python3
"""Independent STOP-only monitor. Default is offline plan; never enables or publishes targets."""
import argparse,json,math,os,sys,time,uuid
from pathlib import Path

def validate(c):
 def require(ok,message):
  if not ok:raise ValueError(message)
 require(c['schema']==1 and [a['motor_id'] for a in c['axes']]==[1,2,3], 'Expected schema 1 and motor IDs 1/2/3')
 require([a['namespace'] for a in c['axes']]==['/motor_1','/motor_2','/motor_3'], 'Unexpected motor namespaces')
 require(c['bilateral_namespace']=='/bilateral_control', 'Unexpected bilateral namespace')
 for a in c['axes']:require(type(a['origin']) in (int,float) and math.isfinite(a['origin']), 'Finite origins required')
 require(set(c['guards'])=={'speed','torque','travel','temperature'}, 'All four guard bounds are required')
 for x in c['guards'].values():require(type(x) in (int,float) and math.isfinite(x) and x>0, 'Positive finite guard bounds required')
 require(c['guards']['speed']<=18 and c['guards']['torque']<=5 and c['guards']['travel']<=25 and c['guards']['temperature']<=48, 'Guard bound outside supported envelope')
 protocol=c.get('protocol','sweep')
 require(protocol in ('sweep','release','long_variable','long_variable_pilot'), 'Unknown guard protocol')
 if protocol=='long_variable':
  require(type(c['max_total_duration_s']) in (int,float) and math.isfinite(c['max_total_duration_s']) and abs(c['max_total_duration_s']-316.04)<1e-6, 'long_variable total limit must be 316.04 seconds')
 else:require(type(c['max_total_duration_s']) in (int,float) and 3<c['max_total_duration_s']<=40, 'Duration must be in (3,40] seconds')
 result=Path(c['result_path'])
 require(result.is_absolute() and result.parent.is_dir() and not result.exists(), 'New absolute result path with existing parent required')

def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--config',type=Path,required=True);p.add_argument('--execute',action='store_true');args=p.parse_args()
 c=json.loads(args.config.read_text());validate(c)
 if not args.execute:print(json.dumps(dict(type='PLAN',config=c)));return
 import rclpy
 from rclpy.node import Node
 from rclpy.qos import qos_profile_sensor_data
 from dm_motor_interfaces.msg import MotorState,MotorFeedback,MotorRequest,BilateralState
 from dm_motor_interfaces.srv import BilateralControl
 from collections import deque
 rclpy.init();n=Node('independent_three_axis_stop_'+uuid.uuid4().hex[:8],start_parameter_services=False)
 s={};f={};received={};positions={a:deque(maxlen=1500) for a in [1,2,3]};b=[None];sub=[];pub={}
 def state(a,m):s[a]=m;received['s'+str(a)]=time.monotonic()
 def feedback(a,m):f[a]=m;received['f'+str(a)]=time.monotonic();positions[a].append((m.receive_time,m.position))
 def follow(m):b[0]=m;received['b']=time.monotonic()
 for a in c['axes']:
  i=a['motor_id'];ns=a['namespace'];pub[i]=n.create_publisher(MotorRequest,ns+'/control',64)
  sub.append(n.create_subscription(MotorState,ns+'/state',lambda m,i=i:state(i,m),qos_profile_sensor_data))
  sub.append(n.create_subscription(MotorFeedback,ns+'/feedback',lambda m,i=i:feedback(i,m),qos_profile_sensor_data))
 sub.append(n.create_subscription(BilateralState,c['bilateral_namespace']+'/state',follow,qos_profile_sensor_data))
 control=n.create_client(BilateralControl,c['bilateral_namespace']+'/operator_control')
 client='stop_guard_'+uuid.uuid4().hex;counter=0;started=time.monotonic();armed=False;ready=False;seq=-1;ping=started;buf=b'';eof=False;reason='';failure=False;fence=None;stop_counts={};stop_positions={};last_stop=0.;bilateral_stop_sent=False;proof=False;stationary=False
 os.set_blocking(sys.stdin.fileno(),False)
 def emit(**kw):print(json.dumps(kw,allow_nan=False),flush=True)
 def fresh(now):
  return len(s)==3 and len(f)==3 and b[0] is not None and now-received['b']<.2 and all(s[i].motor_id==i and s[i].ready and not s[i].simulated and not s[i].host_fault and not s[i].device_fault and 0<=now-f[i].receive_time<.08 and 0<=now-s[i].feedback_receive_time<.08 and now-received['s'+str(i)]<.08 and now-received['f'+str(i)]<.08 for i in [1,2,3])
 def trip(message,bad=True):
  nonlocal fence,reason,failure,stop_counts,stop_positions
  if fence is not None:return
  fence=time.monotonic();reason=message;failure=bad;stop_counts={i:f[i].feedback_count for i in f};stop_positions={i:f[i].position for i in f};emit(type='STOPPING',reason=reason,failure=bad,time=fence)
 def stop(now):
  nonlocal counter,last_stop,bilateral_stop_sent
  if now-last_stop<.05:return
  for i in [1,2,3]:
   counter+=1;m=MotorRequest(client_id=client,request_id=counter,valid_for=.3,kind=MotorRequest.OPERATION,operation=MotorRequest.DISABLE);m.header.stamp=n.get_clock().now().to_msg();pub[i].publish(m)
  if not bilateral_stop_sent and control.service_is_ready():
   req=BilateralControl.Request(operation=BilateralControl.Request.STOP,session=client,valid_for=.3);req.header.stamp=n.get_clock().now().to_msg();control.call_async(req);bilateral_stop_sent=True
  last_stop=now
 try:
  while True:
   rclpy.spin_once(n,timeout_sec=.001);now=time.monotonic()
   if fence is None:
    try:
     while True:
      chunk=os.read(sys.stdin.fileno(),65536)
      if not chunk:eof=True;break
      buf+=chunk
      if len(buf)>65536:raise ValueError('Parent protocol overflow')
    except BlockingIOError:pass
    while b'\n' in buf:
     line,buf=buf.split(b'\n',1);msg=json.loads(line);kind=msg['type']
     if kind=='READY':continue
     if type(msg.get('seq')) is not int or msg['seq']<=seq:raise ValueError('Nonmonotonic parent sequence')
     seq=msg['seq']
     if kind=='FINISH':trip('Parent FINISH',False);break
     stamp=msg.get('sent_time')
     if type(stamp) not in [float,int] or not math.isfinite(stamp) or not -.02<=now-stamp<=.2:raise ValueError('Expired parent request')
     if kind=='ARM':
      if not ready or armed:raise ValueError('ARM requires stopped READY state')
      armed=True;ping=stamp;emit(type='ARMED',time=now)
     elif kind=='PING':
      if not armed:raise ValueError('PING before ARM')
      ping=stamp
     else:raise ValueError('Unknown parent command')
    if eof:trip('Parent pipe EOF')
    if now-started>c['max_total_duration_s']:trip('Total time limit')
    if not ready and fresh(now) and all(s[i].phase==0 and s[i].status==0 and not s[i].target_active and not s[i].active_source for i in [1,2,3]) and b[0].phase==0 and b[0].stop_confirmed:
     ready=True;emit(type='READY',time=now)
    if armed:
     if now-ping>.2:trip('Parent heartbeat timeout')
     elif not fresh(now):trip('State/feedback freshness or fault')
     else:
      for a in c['axes']:
       i=a['motor_id'];g=c['guards'];z=f[i]
       if max(abs(z.velocity),abs(s[i].velocity))>g['speed'] or abs(z.torque)>g['torque'] or abs(z.position-a['origin'])>g['travel'] or max(z.mos_temperature,z.rotor_temperature)>=g['temperature']:trip(f'Axis {i} motion bound')
   if fence is not None:
    stop(now)
    proof=len(s)==3 and len(f)==3 and b[0] is not None and received['b']>fence and now-received['b']<.2 and b[0].stop_confirmed and b[0].phase in (0,5) and all(s[i].status==0 and s[i].phase in (0,4) and not s[i].target_active and not s[i].task_active and not s[i].active_source and fence<f[i].receive_time<=now and fence<s[i].feedback_receive_time<=now and now-f[i].receive_time<.08 and now-s[i].feedback_receive_time<.08 and f[i].feedback_count>stop_counts.get(i,-1) for i in [1,2,3])
    windows={i:[q for t,q in positions[i] if now-.2<=t<=now] for i in [1,2,3]}
    stationary=all(len(v)>30 and max(v)-min(v)<.003 for v in windows.values())
    if (proof and stationary and now-fence>=.25) or now-fence>5:break
 except BaseException as e:
  trip('Guard exception: '+repr(e))
  until=time.monotonic()+5
  while time.monotonic()<until:stop(time.monotonic());rclpy.spin_once(n,timeout_sec=.005)
 result=dict(type='RESULT',success=proof and stationary and not failure,stop_confirmed=proof,stationary_observed=stationary,failure=failure,reason=reason,started=started,stop_requested=fence,finished=time.monotonic(),counts={i:f[i].feedback_count for i in f},post_stop_max_displacement={i:max([abs(q-stop_positions.get(i,q)) for t,q in positions[i] if fence is not None and t>=fence] or [0.]) for i in [1,2,3]})
 with Path(c['result_path']).open('x') as out:out.write(json.dumps(result,indent=2)+'\n')
 emit(**result);n.destroy_node();rclpy.shutdown()
 if not result['success']:raise SystemExit(1)

if __name__=='__main__':main()
