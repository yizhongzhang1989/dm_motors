#!/usr/bin/env python3
"""Finite real three-motor drag test; explicit execute, original timestamps, all-axis STOP."""
import argparse,hashlib,json,math,signal,sys,time,uuid,os,subprocess,struct,shutil
from collections import deque
from pathlib import Path
from long_protocol import Protocol as LongProtocol, BUFFER_BYTES as LONG_BUFFER_BYTES
from start_preparation import prepare_start
from startup_guard import (check_stage, discovery_deadline, native_deadline, native_ready, check_deadline,
 observations_after_fence, wait_fresh_observations)
from capability_handshake import read_capabilities, DEADLINE_ERROR as CAPABILITY_DEADLINE_ERROR
HERE = Path(__file__).resolve().parent
W = HERE.parents[2]
if not (W / 'src/dm_motor_control').is_dir():
 raise RuntimeError('Run this tool from its repository checkout')
FIELDS=('control_mode','coupling_mode','direction','stiffness','coupling_damping','local_damping','rate','required_velocity_bandwidth','operator_timeout','target_validity','max_torque','motion_limits_enabled','friction_compensation_enabled','friction_compensation_fraction','friction_torque_limit','extra_viscous_a','extra_viscous_b','extra_viscous_velocity_limit','system_parameters_file','config_file')

def condition_settings(condition,extra_viscous_a=None):
 if extra_viscous_a is not None and (type(extra_viscous_a) not in (int,float) or not math.isfinite(extra_viscous_a) or not 0<=extra_viscous_a<=.05):
  raise ValueError('extra-viscous-a must be finite and in [0,.05] Nm s/rad')
 if condition=='passive':
  if extra_viscous_a is not None:raise ValueError('extra-viscous-a requires a non-passive condition')
  return {}
 if condition not in ('bilateral_no_friction','bilateral_friction','unilateral_friction'):
  raise ValueError('Unknown coupling trial condition')
 result=dict(coupling_mode='a_leads_b' if condition=='unilateral_friction' else 'bilateral',
  friction_compensation_enabled=condition!='bilateral_no_friction')
 if extra_viscous_a is not None:result['extra_viscous_a']=float(extra_viscous_a)
 return result

def controller_stopped(phase, confirmed):
 # BilateralState.IDLE=0, FAULT=5. A completed fault stop is still a stop;
 # it must not turn a failed trajectory into a pass or permit automatic START.
 return phase in (0,5) and confirmed is True

def geometry(speed,protocol='sweep'):
 if protocol in ('long_variable','long_variable_pilot'):
  if speed!=13.2:raise ValueError('long_variable requires --speed 13.2; its six levels are fixed')
  plan=LongProtocol(pilot=protocol=='long_variable_pilot')
  return dict(protocol_version=1,duration=plan.duration,distance=plan.max_distance,
   motion_start=plan.motion_start,motion_end=plan.motion_end,motion_duration=plan.motion_duration,
   groups=plan.groups,speeds=[.6,1.5,3.,6.,9.,13.2],step=.6,windows=plan.windows())
 if protocol=='release':
  r=max(.25,speed/15.);p=.6;release=.3+r+p
  return dict(protocol_version=2,ramp=r,plateau=p,distance=speed*(r/2+p),release_time=release,
   observation=5.,neutral_tail=.05,duration=release+5.05,
   windows=[[release-.15,release-.03]])
 if protocol!='sweep':raise ValueError('Unknown protocol')
 r=max(.25,speed/15.);p=.6;leg=2*r+p;distance=speed*(r+p)
 return dict(ramp=r,plateau=p,leg=leg,distance=distance,duration=2*leg+1.05,
  windows=[[.3+r+.15,.3+r+p-.15],[.55+leg+r+.15,.55+leg+r+p-.15]])

def sweep(t,speed,r,p):
 if t<r:
  v=speed*(1-math.cos(math.pi*t/r))/2;d=speed*(t-r*math.sin(math.pi*t/r)/math.pi)/2
 elif t<r+p:v=speed;d=speed*r/2+speed*(t-r)
 else:
  u=min(r,t-r-p);v=speed*(1+math.cos(math.pi*u/r))/2
  d=speed*r/2+speed*p+speed*(u+r*math.sin(math.pi*u/r)/math.pi)/2
 return d,v

def reference(t,start,speed,protocol='sweep'):
 if protocol in ('long_variable','long_variable_pilot'):
  r=LongProtocol(pilot=protocol=='long_variable_pilot').reference(t,start);return r.q,r.v,r.phase
 g=geometry(speed,protocol)
 if protocol=='release':
  if t<.3:return start,0.,'baseline'
  if t<.3+g['ramp']:
   q,v=sweep(t-.3,speed,g['ramp'],g['plateau']);return start+q,v,'ramp'
  if t<g['release_time']:
   return start+speed*g['ramp']/2+speed*(t-.3-g['ramp']),speed,'release_steady'
  return 0.,0.,'release_neutral'  # Simultaneously kp=kd=t=0; q is an encodable unused field.
 if t<.3:return start,0.,'baseline'
 if t<.3+g['leg']:
  q,v=sweep(t-.3,speed,g['ramp'],g['plateau']);return start+q,v,'positive'
 if t<.55+g['leg']:return start+g['distance'],0.,'turnaround'
 if t<.55+2*g['leg']:
  q,v=sweep(t-.55-g['leg'],speed,g['ramp'],g['plateau']);return start+g['distance']-q,-v,'negative'
 return start,0.,'settle'

def required_driver_bandwidth(value):
 if not math.isfinite(value) or not 0 < value < 500:
  raise ValueError('driver-bandwidth must be finite and in (0,500) Hz')
 encoded=struct.unpack('<f',struct.pack('<f',value))[0]
 if not 0 < encoded < 500:
  raise ValueError('driver-bandwidth float32 must be in (0,500) Hz')
 return encoded


def decode_bandwidth_result(state,address,raw):
 """Only a matching independently confirmed address-32 read is a bandwidth."""
 if state==0:return None  # MotorReply.PENDING; queued is not confirmation.
 if state!=1 or address!=32:
  raise ValueError('V_BW register 32 read was not independently confirmed')
 value=struct.unpack('<f',struct.pack('<I',raw))[0]
 if not math.isfinite(value):raise ValueError('Non-finite V_BW readback')
 return value

def main():
 script_source=Path(__file__).read_bytes();script_sha256=hashlib.sha256(script_source).hexdigest()
 p=argparse.ArgumentParser(description=__doc__)
 p.add_argument('--execute',action='store_true');p.add_argument('--output',type=Path,required=True)
 p.add_argument('--stream-binary',type=Path,required=True,help='Explicit CMake-built rig_stream executable')
 p.add_argument('--driver-bandwidth',type=float,default=100.,help='Required C/id3 RAM V_BW readback in Hz; read only, default 100')
 p.add_argument('--driver-kp',type=float,default=20.,help='C/id3 native MIT position gain, (0,30], default 20')
 p.add_argument('--driver-kd',type=float,default=.6,help='C/id3 native MIT velocity gain, (0,1], default .6')
 p.add_argument('--extra-viscous-a',type=float,default=None,help='Explicit A viscosity assistance, [0,.05] Nm s/rad; restored after STOP')
 p.add_argument('--protocol',choices=['sweep','release','long_variable','long_variable_pilot'],default='sweep')
 p.add_argument('--condition',choices=['passive','bilateral_no_friction','bilateral_friction','unilateral_friction'],required=True)
 p.add_argument('--speed',type=float,required=True);p.add_argument('--speed-guard',type=float,required=True)
 p.add_argument('--torque-guard',type=float,required=True);p.add_argument('--travel-guard',type=float,required=True)
 args=p.parse_args()
 try:trial_settings=condition_settings(args.condition,args.extra_viscous_a)
 except ValueError as exc:p.error(str(exc))
 long_mode=args.protocol in ('long_variable','long_variable_pilot')
 args.output=args.output.expanduser().resolve();args.stream_binary=args.stream_binary.expanduser().resolve()
 try:required_driver_bandwidth(args.driver_bandwidth)
 except ValueError as exc:p.error(str(exc))
 vals=[args.speed,args.speed_guard,args.torque_guard,args.travel_guard,args.driver_kp,args.driver_kd]
 if not all(math.isfinite(x) and x>0 for x in vals):p.error('Positive finite bounds required')
 if args.driver_kp>30 or args.driver_kd>1:p.error('driver-kp must be in (0,30] and driver-kd in (0,1]')
 if not args.speed<args.speed_guard<=18 or not args.torque_guard<=5 or not args.travel_guard<=25:p.error('Unsupported test bounds')
 if (args.protocol=='release' or long_mode) and args.condition=='passive':p.error('Release/long_variable requires A/B bilateral control')
 if long_mode and args.speed!=13.2:p.error('long_variable requires --speed 13.2; its six levels are fixed')
 g=geometry(args.speed,args.protocol)
 if g['distance']+.1>args.travel_guard:p.error('Planned travel exceeds guard')
 if args.output.exists() or not args.output.parent.is_dir():p.error('New output path with existing parent required')
 if not args.execute:print(json.dumps(dict(plan=vars(args)|{'output':str(args.output),'stream_binary':str(args.stream_binary)},controller_settings=trial_settings,geometry=g)));return
 sources=[HERE/'run_trial.py',HERE/'start_preparation.py',HERE/'startup_guard.py',HERE/'capability_handshake.py',HERE/'record_only.py',HERE/'stop_guard.py',HERE/'native_stream/stream.cpp',HERE/'native_stream/CMakeLists.txt',HERE/'native_stream/build_info.hpp.in']
 if long_mode:sources.extend(HERE/name for name in ['long_protocol.py','long_metrics.py','long_audit_core.py','long_wire_audit.py','score_long.py','score_trials.py'])
 source_snapshots={source:(script_source if source==Path(__file__).resolve() else source.read_bytes()) for source in sources}
 if long_mode and shutil.disk_usage(args.output.parent).free<4*1024**3:p.error('long_variable recording requires at least 4 GiB free disk space')
 if not args.stream_binary.is_file() or not os.access(args.stream_binary,os.X_OK):p.error('stream-binary must be an executable file')
 # This query exits before ROS initialization in the supplied C++ program.
 build_query=subprocess.run([str(args.stream_binary),'--build-info'],capture_output=True,text=True,timeout=5,check=True)
 build_info=json.loads(build_query.stdout)
 expected_source=hashlib.sha256((HERE/'native_stream/stream.cpp').read_bytes()).hexdigest()
 expected_cmake=hashlib.sha256((HERE/'native_stream/CMakeLists.txt').read_bytes()).hexdigest()
 if build_info.get('source_sha256')!=expected_source or build_info.get('cmake_sha256')!=expected_cmake:p.error('Native sender build does not match these source/CMake files; rebuild it')
 stream_binary_hash=hashlib.sha256(args.stream_binary.read_bytes()).hexdigest()
 sys.path[:0]=[str(W/'src/dm_motor_control')]
 import rclpy
 from rclpy.node import Node
 from rclpy.parameter import Parameter
 from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
 from dm_motor_interfaces.msg import MotorState,MotorFeedback,MotorRequest,BilateralState
 from dm_motor_interfaces.srv import BilateralControl
 from dm_motor_control.topic_client import motor_topics
 from rcl_interfaces.srv import GetParameters,SetParametersAtomically
 from rcl_interfaces.msg import Parameter as ParameterMessage
 from can_bus.msg import Status
 from rosidl_runtime_py.convert import message_to_ordereddict as convert
 args.output.mkdir();rclpy.init();n=Node('physical_three_motor_drag',start_parameter_services=False);parent_launched=time.monotonic()
 s={};fb={};arrival={};origins={};topics={};caps={};bus=[None];bilateral=[None];sub=[]
 logs={k:[] for k in ['events']}
 hist={a:deque(maxlen=64) for a in [1,2,3]}
 class RowLog:
  def __init__(self,name):self.stream=(args.output/(name+'.jsonl')).open('x')
  def append(self,row):self.stream.write(json.dumps(row,allow_nan=False)+'\n')
  def close(self):self.stream.close()
 logs={name:RowLog(name) for name in logs}
 q=QoSProfile(depth=4,reliability=ReliabilityPolicy.BEST_EFFORT,durability=DurabilityPolicy.VOLATILE)
 def state(a,m):
  s[a]=m;arrival['s'+str(a)]=time.monotonic()
 def feedback(a,m):
  fb[a]=m;arrival['f'+str(a)]=time.monotonic();hist[a].append((m.receive_time,m.position))
 def follow(m):
  bilateral[0]=m;arrival['bilateral']=time.monotonic()
 def bus_state(m):bus[0]=m;arrival['bus']=time.monotonic()
 for a in [1,2,3]:
  topics[a]=motor_topics(n,f'/motor_{a}')
  sub.append(n.create_subscription(MotorState,f'/motor_{a}/state',lambda m,a=a:state(a,m),q))
  sub.append(n.create_subscription(MotorFeedback,f'/motor_{a}/feedback',lambda m,a=a:feedback(a,m),q))

 sub.append(n.create_subscription(BilateralState,'/bilateral_control/state',follow,q));sub.append(n.create_subscription(Status,'/can/main/status',bus_state,q))

 operator=n.create_client(BilateralControl,'/bilateral_control/operator_control')
 get=n.create_client(GetParameters,'/bilateral_control/get_parameters');setp=n.create_client(SetParametersAtomically,'/bilateral_control/set_parameters_atomically')
 recorder=None;recorder_ready=False;recorder_buffer=b'';recorder_log=None;recorder_result=None
 stream=None;stream_ready=False;stream_started=None;stream_done=None;stream_buffer=b'';stream_log=None;stream_source='rig_stream_'+uuid.uuid4().hex
 session='three_rig_'+uuid.uuid4().hex;heartbeat=False;pending=None;last_hb=0.;last_hb_send=0.
 monitor=None;monitor_buffer=b'';monitor_ready=False;monitor_armed=False;monitor_result=None;monitor_stopping=False;monitor_sequence=0;monitor_ping=0.;monitor_log=None
 startup_stage='prestart';guard_enabled=False;controller_source=None;idle_command_count=None;stream_launched=None;start_deadline=None;changed_names=[]
 def monitor_send(kind):
  nonlocal monitor_sequence,monitor_ping
  monitor_sequence+=1;stamp=time.monotonic()
  os.write(monitor.stdin.fileno(),(json.dumps(dict(type=kind,seq=monitor_sequence,sent_time=stamp))+'\n').encode());monitor_ping=stamp
 def monitor_poll(enforce=True):
  nonlocal monitor_buffer,monitor_ready,monitor_armed,monitor_result,monitor_stopping
  if monitor is None:return
  try:
   while True:
    chunk=os.read(monitor.stdout.fileno(),65536)
    if not chunk:break
    monitor_buffer+=chunk
  except BlockingIOError:pass
  while b'\n' in monitor_buffer:
   line,monitor_buffer=monitor_buffer.split(b'\n',1)
   item=json.loads(line);logs['events'].append(dict(event='GUARD',time=time.monotonic(),message=item))
   kind=item.get('type')
   if kind=='READY':monitor_ready=True
   elif kind=='ARMED':monitor_armed=True
   elif kind=='STOPPING':monitor_stopping=True
   elif kind=='RESULT':monitor_result=item
  if enforce:
   if monitor_stopping or monitor_result is not None:raise RuntimeError('Independent guard requested STOP')
   if monitor.poll() is not None:raise RuntimeError('Independent guard exited before FINISH')
   if monitor_armed and time.monotonic()-monitor_ping>.04:monitor_send('PING')
 def stream_poll(enforce=True):
  nonlocal stream_buffer,stream_ready,stream_started,stream_done
  if stream is None:return
  try:
   while True:
    chunk=os.read(stream.stdout.fileno(),65536)
    if not chunk:break
    stream_buffer+=chunk
  except BlockingIOError:pass
  while b'\n' in stream_buffer:
   line,stream_buffer=stream_buffer.split(b'\n',1);item=json.loads(line);logs['events'].append(dict(event='NATIVE_STREAM',time=time.monotonic(),message=item))
   if item.get('type')=='READY':stream_ready=True
   elif item.get('type')=='START':stream_started=item['started']
   elif item.get('type')=='DONE':stream_done=item
  if enforce and ((stream_done is not None and not stream_done['success']) or (stream.poll() is not None and stream_done is None)):
   raise RuntimeError('Native target stream failed')
 def recorder_poll(enforce=True):
  nonlocal recorder_buffer,recorder_ready
  if recorder is None:return
  try:
   while True:
    chunk=os.read(recorder.stdout.fileno(),65536)
    if not chunk:break
    recorder_buffer+=chunk
  except BlockingIOError:pass
  while b'\n' in recorder_buffer:
   line,recorder_buffer=recorder_buffer.split(b'\n',1);item=json.loads(line)
   if item.get('type')=='READY':recorder_ready=True
  if enforce and recorder.poll() is not None:raise RuntimeError('Read-only recorder exited unexpectedly')
 def request(operation):
  req=BilateralControl.Request(operation=operation,session=session,valid_for=.3);req.header.stamp=n.get_clock().now().to_msg();return req
 def pump():
  nonlocal pending,last_hb,last_hb_send
  rclpy.spin_once(n,timeout_sec=.0005);now=time.monotonic();monitor_poll();stream_poll();recorder_poll()
  if heartbeat:
   if pending is not None:
    if pending.done():
     reply=pending.result()
     if not reply.success:raise RuntimeError('Heartbeat rejected: '+reply.message)
     pending=None;last_hb=now
    elif now-last_hb_send>.3:raise RuntimeError('Heartbeat response timeout')
   if pending is None and now-last_hb_send>.1:
    pending=operator.call_async(request(BilateralControl.Request.HEARTBEAT));last_hb_send=now
  if start_deadline is not None and stream_started is None:check_deadline(start_deadline,time.monotonic())
  if guard_enabled:guard()
 def wait(pred,timeout=2.):
  until=time.monotonic()+timeout
  while time.monotonic()<until:
   pump()
   if pred():return
  raise TimeoutError('ROS response/discovery timeout')
 def future(client,req,timeout=2.):
  wait(client.service_is_ready,3.);f=client.call_async(req);wait(f.done,timeout);return f.result()
 def call(a,kind,req,before_send=None,deadline=None):
  def available(limit):
   if deadline is None:return limit
   remaining=deadline-time.monotonic()
   if remaining<=0:raise TimeoutError(CAPABILITY_DEADLINE_ERROR)
   return min(limit,remaining)
  ep=topics[a].endpoint(kind);wait(ep.is_ready,available(8.))
  if before_send is not None:before_send()
  available(2.)
  f=ep.call_async(req);wait(f.done,available(2.))
  available(2.)
  reply=f.result()
  if not reply.success:raise RuntimeError(reply.message)
  return reply
 def read_config():
  reply=future(get,GetParameters.Request(names=list(FIELDS)))
  if len(reply.values)!=len(FIELDS):raise RuntimeError('Incomplete parameter response')
  return {name:Parameter.from_parameter_msg(ParameterMessage(name=name,value=value)).value for name,value in zip(FIELDS,reply.values)}
 def set_trial_settings(values):
  reply=future(setp,SetParametersAtomically.Request(parameters=[Parameter(name,value=value).to_parameter_msg() for name,value in values.items()]))
  if not reply.result.successful:raise RuntimeError('Parameter rejected: '+reply.result.reason)
 def guard(active=False,stage=None):
  now=time.monotonic();b=bilateral[0]
  if not bus[0].ready or now-arrival['bus']>.2 or now-arrival['bilateral']>.2:raise RuntimeError('Stale bus/controller status')
  check_stage(b,s,stage=stage or ('active' if active else startup_stage),passive=args.condition=='passive',
   controller_source=controller_source,driver_source=stream_source,enable_source=topics[3].client_id,idle_command_count=idle_command_count)
  for a in [1,2,3]:
   z,f=s[a],fb[a]
   if not (z.ready and z.absolute_position_valid and not z.simulated and z.motor_id==a and z.mode==1 and not z.host_fault and not z.device_fault and not z.task_active and 0<=now-f.receive_time<.08 and 0<=now-z.feedback_receive_time<.08 and now-arrival['s'+str(a)]<.08 and now-arrival['f'+str(a)]<.08):raise RuntimeError(f'Axis {a} not fresh/ready: '+z.error)
   if abs(f.velocity)>args.speed_guard or abs(z.velocity)>args.speed_guard or abs(f.torque)>args.torque_guard or max(f.mos_temperature,f.rotor_temperature)>=48 or abs(f.position-origins[a])>args.travel_guard:raise RuntimeError(f'Axis {a} motion guard: v={f.velocity:.6f}, state_v={z.velocity:.6f}, tau={f.torque:.6f}, travel={abs(f.position-origins[a]):.6f}')
   if z.rejected_commands!=rejected_baseline[a]:raise RuntimeError(f'Axis {a} command rejection')
  common=min(fb[1].receive_time,fb[3].receive_time)
  def aligned(a):
   rr=list(hist[a]);pairs=[(x,y) for x,y in zip(rr,rr[1:]) if x[0]<=common<=y[0] and y[0]>x[0]]
   if not pairs:
    if rr and abs(rr[-1][0]-common)<1e-8:return rr[-1][1]
    raise RuntimeError('No common-time mechanical observation')
   x,y=pairs[-1];return x[1]+(y[1]-x[1])*(common-x[0])/(y[0]-x[0])
  if now-common>.02:raise RuntimeError('Mechanical pair age >20ms')
  if abs((aligned(1)-origins[1])+(aligned(3)-origins[3]))>.08:raise RuntimeError('Time-aligned mechanical mapping deviates >.08 rad')
 def read_driver_bandwidth():
  guard()
  queued=call(3,'registers',MotorRequest(write=False,addresses=[32],raw_values=[]))
  logs['events'].append(dict(event='READ_ONLY_C_V_BW_QUEUED',time=time.monotonic(),reply=convert(queued)))
  if len(queued.operation_ids)!=1 or queued.operation_ids[0]<=0:raise RuntimeError('V_BW read requires exactly one new operation ID')
  operation_id=queued.operation_ids[0];driver_bandwidth['operation_id']=operation_id
  deadline=time.monotonic()+2.
  while time.monotonic()<deadline:
   guard()
   reply=call(3,'get_operation',MotorRequest(operation_id=operation_id))
   logs['events'].append(dict(event='READ_ONLY_C_V_BW_RESULT',time=time.monotonic(),operation_id=operation_id,reply=convert(reply)))
   # TopicEndpoint matches this request/reply by client_id and request_id.
   # OPERATION_RESULT does not echo operation_id in the current driver.
   actual=decode_bandwidth_result(reply.state,reply.address,reply.raw_value)
   if actual is not None:
    driver_bandwidth.update(actual=actual,raw_value=reply.raw_value,confirmed_time=time.monotonic())
    if actual!=required_driver_bandwidth(args.driver_bandwidth):raise RuntimeError(f'Actual C V_BW={actual} differs from required {args.driver_bandwidth} Hz')
    driver_bandwidth['verified']=True
    return
   pause_until=time.monotonic()+.03;wait(lambda:time.monotonic()>=pause_until,.1)
  raise TimeoutError('C V_BW readback confirmation timeout')
 def stationary_start_observation(after):
  guard(stage='prestart')  # A confirmed stationary rejection must still be all-disabled.
  if after is not None and not (arrival['bilateral']>after and all(
   arrival['s'+str(a)]>after and arrival['f'+str(a)]>after and
   s[a].feedback_receive_time>after and fb[a].receive_time>after for a in [1,2])):
   return None
  return tuple(value for a in [1,2] for value in (s[a].velocity,fb[a].velocity))
 def submit_prepared_start(deadline):
  if not operator.service_is_ready():raise RuntimeError('Bilateral START service unavailable')
  if time.monotonic()>=deadline:raise TimeoutError('Pre-enable START deadline reached before request')
  req=request(BilateralControl.Request.START)
  if time.monotonic()>=deadline:raise TimeoutError('Pre-enable START deadline reached before publish')
  return operator.call_async(req)
 def submit_native_prepared_start(deadline):
  nonlocal startup_stage
  deadline=min(deadline,start_deadline) if start_deadline is not None else deadline
  startup_stage='ab_starting'
  return submit_prepared_start(deadline)
 def startup_clock():
  now=time.monotonic();check_deadline(start_deadline,now);return now
 def require_native_window():
  if not native_ready(args.protocol,stream_launched,time.monotonic()):raise RuntimeError('Native discovery interval is incomplete')
 seq=0
 driver_bandwidth=dict(required=args.driver_bandwidth,actual=None,verified=False,register_address=32,operation_id=None,read_only=True)
 error='';stop_ok=False;restored=False;changed=False;config={};effective={};started=None;rejected_baseline={};stop_elapsed=None;input_hashes={}
 previous={sig:signal.signal(sig,lambda *_:(_ for _ in ()).throw(KeyboardInterrupt())) for sig in [signal.SIGINT,signal.SIGTERM]}
 try:
  source_hashes={}
  for source in sources:
   data=source_snapshots[source];relative=source.relative_to(HERE);destination=args.output/'sources'/relative
   destination.parent.mkdir(exist_ok=True,parents=True)
   with destination.open('xb') as copied:copied.write(data)
   source_hashes[str(relative)]=hashlib.sha256(data).hexdigest()
  with (args.output/'provenance.json').open('x') as out:
   json.dump(dict(source_hashes=source_hashes,stream_binary=str(args.stream_binary),stream_binary_sha256=stream_binary_hash,embedded_build_info=build_info),out,indent=2);out.write('\n')
  wait(lambda:len(s)==3 and len(fb)==3 and bilateral[0] is not None and bus[0] is not None,8.)
  wait(lambda:all(n.count_publishers(f'/motor_{a}/control_result')>0 and topics[a].commands.get_subscription_count()>0 for a in [1,2,3]),8.)
  # Discovery is asynchronous in both directions. Only this read-only request may retry.
  discovery_settle=discovery_deadline(parent_launched,time.monotonic())
  wait(lambda:time.monotonic()>=discovery_settle,discovery_settle-time.monotonic()+.5)
  caps={str(a):reply for a,reply in read_capabilities((1,2,3),
   lambda axis,deadline:convert(call(axis,'get_capabilities',MotorRequest(),deadline=deadline)),
   pump,time.monotonic,logs['events'].append).items()}

  config=read_config()
  for key in ['system_parameters_file','config_file']:
   source=Path(config[key]).expanduser().resolve();data=source.read_bytes()
   input_hashes[key]=dict(path=str(source),sha256=hashlib.sha256(data).hexdigest())
   with (args.output/(key+'.yaml')).open('xb') as copied:copied.write(data)
  if config['control_mode']!='differential_torque' or config['rate']!=500. or config['required_velocity_bandwidth']!=100.:raise RuntimeError('Unexpected controller mode/rate/bandwidth')
  if config['coupling_mode'] not in ('bilateral','a_leads_b'):raise RuntimeError('Controller coupling_mode unavailable or invalid')
  c=caps['3']['configuration']
  if c['watchdog_timeout']>.1+1e-9 or c['command_timeout']>.12+1e-9 or not c['host_protection_enabled'] or c['measured_velocity_limit']<args.speed_guard-1e-9 or c['measured_torque_limit']<args.torque_guard-1e-9:raise RuntimeError('Driver3 limits/watchdogs insufficient for requested test')
  # Parameter responses and hashing/copying files can leave valid callback
  # caches older than the unchanged 20 ms mechanical observation guard.
  wait_fresh_observations(lambda fence,now:observations_after_fence(s,fb,arrival,bilateral[0],bus[0],
   fence=fence,now=now),pump,time.monotonic,logs['events'].append)
  origins={a:fb[a].position for a in [1,2,3]};rejected_baseline={a:s[a].rejected_commands for a in [1,2,3]}
  controller_source=bilateral[0].controller_source;idle_command_count=bilateral[0].command_count;guard_enabled=True;guard()
  if s[3].direction!=1. or s[3].software_offset!=0. or s[3].pmax!=12.5 or s[3].vmax!=50. or s[3].tmax!=10.:raise RuntimeError('Third-axis identity/mapping differs')
  if not (-12.4<origins[3]<origins[3]+g['distance']<12.4):raise RuntimeError('Third-axis MIT position envelope exceeded; park first')
  if long_mode:LongProtocol(pilot=args.protocol=='long_variable_pilot').validate_origin(origins[3])
  read_driver_bandwidth()
  if args.condition!='passive':
   changed_names=[name for name,value in trial_settings.items() if config[name]!=value]
   if changed_names:
    changed=True;set_trial_settings(trial_settings)
   effective=read_config()
   if any(effective[name]!=value for name,value in trial_settings.items()):raise RuntimeError('Effective trial settings not confirmed')
  else:effective=dict(config)
  # Slow graph/capability/configuration work finishes before the bounded recorder.
  recorder_log=(args.output/'recorder.log').open('x')
  recorder_args=[sys.executable,str(Path(__file__).with_name('record_only.py')),'--output',str(args.output)]
  if long_mode:recorder_args+=['--protocol',args.protocol]
  recorder=subprocess.Popen(recorder_args,stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=recorder_log,start_new_session=True)
  os.set_blocking(recorder.stdout.fileno(),False)
  (args.output/'recorder-process.json').write_text(json.dumps(dict(pid=recorder.pid,start_ticks=Path(f'/proc/{recorder.pid}/stat').read_text().split()[21]))+'\n')
  wait(lambda:recorder_ready,8.)
  guard_config=dict(schema=1,axes=[dict(namespace=f'/motor_{a}',motor_id=a,origin=origins[a]) for a in [1,2,3]],bilateral_namespace='/bilateral_control',guards=dict(speed=args.speed_guard,torque=args.torque_guard,temperature=48.,travel=args.travel_guard),max_total_duration_s=g['duration']+14.,result_path=str((args.output/'guard-result.json').resolve()))
  if long_mode:guard_config['protocol']=args.protocol
  guard_path=args.output/'guard-config.json';guard_path.write_text(json.dumps(guard_config,indent=2)+'\n')
  monitor_log=(args.output/'guard.log').open('x')
  monitor=subprocess.Popen([sys.executable,str(Path(__file__).with_name('stop_guard.py')),'--execute','--config',str(guard_path)],stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=monitor_log,start_new_session=True)
  os.set_blocking(monitor.stdout.fileno(),False);os.set_blocking(monitor.stdin.fileno(),False)
  (args.output/'guard-process.json').write_text(json.dumps(dict(pid=monitor.pid,start_ticks=Path(f'/proc/{monitor.pid}/stat').read_text().split()[21]))+'\n')
  wait(lambda:monitor_ready,8.);monitor_send('ARM');wait(lambda:monitor_armed,1.)
  stream_program=args.stream_binary
  if hashlib.sha256(stream_program.read_bytes()).hexdigest()!=stream_binary_hash:raise RuntimeError('Native sender changed after build verification')
  stream_log=(args.output/'native-stream.log').open('x')
  stream_args=[str(stream_program),str(origins[3]),str(args.speed),str(args.driver_kp),str(args.driver_kd),stream_source,str(args.output/'commands.jsonl'),str(args.output/'stream-result.json'),'--protocol',args.protocol]
  if long_mode:stream_args+=['--log-buffer-bytes',str(LONG_BUFFER_BYTES)]
  stream_launched=time.monotonic();start_deadline=native_deadline(args.protocol,stream_launched)
  stream=subprocess.Popen(stream_args,stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=stream_log,start_new_session=True)
  os.set_blocking(stream.stdout.fileno(),False)
  (args.output/'stream-process.json').write_text(json.dumps(dict(pid=stream.pid,launched=stream_launched,start_ticks=Path(f'/proc/{stream.pid}/stat').read_text().split()[21],executable_sha256=hashlib.sha256(stream_program.read_bytes()).hexdigest()))+'\n')
  wait(lambda:stream_ready,8.)
  if args.protocol=='sweep':wait(lambda:native_ready(args.protocol,stream_launched,time.monotonic()),max(.001,start_deadline-time.monotonic()))
  require_native_window();logs['events'].append(dict(event='NATIVE_DISCOVERY_SETTLED',time=time.monotonic(),launched=stream_launched,deadline=start_deadline))
  if args.condition!='passive':
   desired=trial_settings['friction_compensation_enabled']
   prepare_start(stationary_start_observation,submit_native_prepared_start,pump,startup_clock,logs['events'].append)
   heartbeat=True;last_hb_send=time.monotonic()
   wait(lambda:bilateral[0].phase==BilateralState.ACTIVE and bilateral[0].quality==BilateralState.NORMAL and bilateral[0].recovery_scale==1. and all(s[a].phase==2 and s[a].status==1 and s[a].active_source==controller_source and s[a].target_active for a in [1,2]),4.)
   startup_stage='ab_active'
   if not bilateral[0].velocity_bandwidth_verified:raise RuntimeError('Required bandwidth not freshly verified')
   wait(lambda:bilateral[0].compensation_active==desired,.5)
  require_native_window();startup_stage='c_enabling'
  logs['events'].append(dict(event='C_ENABLE_REQUEST',time=time.monotonic(),client_id=topics[3].client_id))
  call(3,'operation',MotorRequest(operation=MotorRequest.ENABLE),before_send=require_native_window);wait(lambda:s[3].phase==2,.5)
  require_native_window()
  stream.stdin.write(b'GO\n');stream.stdin.flush();wait(lambda:stream_started is not None,1.);started=stream_started
  wait(lambda:s[3].active_source==stream_source and s[3].target_active and s[3].feedback_receive_time>started,.08)
  startup_stage='active'
  while stream_done is None:
   pump();seq=int((time.monotonic()-started)/.002)
   if time.monotonic()-started>g['duration']+.3:raise RuntimeError('Native trajectory completion timeout')
   if stream_done is None:guard(True)
  guard(True)
 except BaseException as exc:error=repr(exc)
 finally:
  heartbeat=False;guard_enabled=False;start_deadline=None;fence=time.monotonic();stop_counts={a:fb[a].feedback_count for a in fb};until=fence+5;last=0.;bilateral_stop_sent=True
  logs['events'].append(dict(event='STOP_REQUESTED',time=fence,feedback_counts=stop_counts))
  operator.call_async(request(BilateralControl.Request.STOP))
  for a in [1,2,3]:topics[a].endpoint('operation').call_async(MotorRequest(operation=MotorRequest.DISABLE))
  if monitor is not None and monitor.poll() is None:
   try:monitor_send('FINISH')
   except (OSError,BrokenPipeError):pass
  if stream is not None:
   if stream.poll() is None:stream.send_signal(signal.SIGINT)
   try:stream.wait(timeout=1.)
   except subprocess.TimeoutExpired:error+='; native target stream failed to exit'
   stream_poll(False);stream.stdin.close();stream.stdout.close();stream_log.close();stream=None
  while time.monotonic()<until:
   now=time.monotonic()
   if now-last>=.05:
    if not bilateral_stop_sent:
     operator.call_async(request(BilateralControl.Request.STOP));bilateral_stop_sent=True
    for a in [1,2,3]:topics[a].endpoint('operation').call_async(MotorRequest(operation=MotorRequest.DISABLE))
    last=now
   rclpy.spin_once(n,timeout_sec=.001);monitor_poll(False)
   if len(s)==3 and len(fb)==3 and bilateral[0] and controller_stopped(bilateral[0].phase,bilateral[0].stop_confirmed) and arrival['bilateral']>fence and all(s[a].status==0 and s[a].phase in (0,4) and not s[a].active_source and not s[a].target_active and not s[a].task_active and fence<s[a].feedback_receive_time<=now and fence<fb[a].receive_time<=now and fb[a].feedback_count>stop_counts.get(a,-1) and now-s[a].feedback_receive_time<.08 and now-fb[a].receive_time<.08 for a in [1,2,3]):stop_ok=True;stop_elapsed=now-fence
   if stop_ok and (monitor is None or monitor_result is not None):break
  if monitor is not None:
   try:monitor.wait(timeout=1.)
   except subprocess.TimeoutExpired:error+='; independent guard has not exited'
   monitor_poll(False)
   if monitor_result is None or not monitor_result.get('stop_confirmed',False):error+='; independent STOP unconfirmed'
   elif not monitor_result.get('success',False):error+='; independent guard failed: '+str(monitor_result.get('reason'))
   monitor.stdin.close();monitor.stdout.close();monitor_log.close()
   monitor=None
  try:
   if changed:
    if not stop_ok:raise RuntimeError('Cannot restore parameter without STOP confirmation')
    wait(lambda:controller_stopped(bilateral[0].phase,bilateral[0].stop_confirmed) and all(s[a].phase==0 and s[a].status==0 and time.monotonic()-s[a].feedback_receive_time<.02 for a in [1,2]),2.)
    set_trial_settings({name:config[name] for name in changed_names})
   if config and read_config()!=config:raise RuntimeError('Controller configuration restore readback differs')
   restored=True
  except BaseException as exc:error=error+'; restore: '+repr(exc)
  if recorder is not None:
   try:
    recorder.stdin.write(b'FINISH\n');recorder.stdin.flush();recorder.wait(timeout=3.)
    recorder_result=json.loads((args.output/'recorder-result.json').read_text())
    if recorder.returncode or not recorder_result['success']:raise RuntimeError('Recording failed')
   except BaseException as exc:error+='; recorder: '+repr(exc)
   recorder.stdin.close();recorder.stdout.close();recorder_log.close();recorder=None
  result=dict(protocol=args.protocol,protocol_version=g.get('protocol_version',1),driver_source=stream_source,controller_source=bilateral[0].controller_source if bilateral[0] is not None else None,driver_kp=args.driver_kp,driver_kd=args.driver_kd,driver_bandwidth=driver_bandwidth,recorder=recorder_result,success=not error and stop_ok and restored,error=error,stop_confirmed=stop_ok,configuration_restored=restored,condition=args.condition,requested_speed=args.speed,guards=dict(speed=args.speed_guard,torque=args.torque_guard,travel=args.travel_guard,temperature=48,feedback_age=.08),geometry=g,origins=origins,started=started,configuration=config,effective_configuration=effective,capabilities=caps,nominal_driver_refresh_hz=500,native_stream=stream_done,independent_guard=monitor_result,stop_request_time=fence,stop_confirmation_seconds=stop_elapsed,script_sha256=script_sha256,registry_sha256=input_hashes.get('system_parameters_file',{}).get('sha256'),input_hashes=input_hashes,stream_binary_sha256=stream_binary_hash)
  result.update(requested_extra_viscous_a=args.extra_viscous_a,changed_parameter_names=changed_names,
   native_launch_time=stream_launched,native_prestart_deadline=native_deadline(args.protocol,stream_launched) if stream_launched is not None else None,
   initial_controller_command_count=idle_command_count,enable_source=topics[3].client_id,
   final_observations=dict(states={str(a):convert(z) for a,z in s.items()},feedback={str(a):convert(z) for a,z in fb.items()},bilateral=convert(bilateral[0]) if bilateral[0] is not None else None))
  result.update(final_states=result['final_observations']['states'],final_feedback=result['final_observations']['feedback'],
   final_bilateral=result['final_observations']['bilateral'],stop_counts=stop_counts)
  for rows in logs.values():rows.close()
  with (args.output/'result.json').open('x') as out:out.write(json.dumps(result,indent=2,allow_nan=False)+'\n')
  for sig,handler in previous.items():signal.signal(sig,handler)
  n.destroy_node();rclpy.shutdown();print(json.dumps({k:v for k,v in result.items() if k!='capabilities'}),flush=True)
 if not result['success']:raise SystemExit(1)

if __name__=='__main__':main()
