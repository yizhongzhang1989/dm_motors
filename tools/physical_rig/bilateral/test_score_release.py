"""Synthetic full-artifact release gates; no ROS/hardware."""
import importlib.util,json,hashlib,math,copy
from pathlib import Path
import pytest
SPEC=importlib.util.spec_from_file_location('release_score',Path(__file__).with_name('score_release.py'))
score=importlib.util.module_from_spec(SPEC);SPEC.loader.exec_module(score)
TRIAL_SPEC=importlib.util.spec_from_file_location('release_runner',Path(__file__).with_name('run_trial.py'))
trial=importlib.util.module_from_spec(TRIAL_SPEC);TRIAL_SPEC.loader.exec_module(trial)

def dump(path,data):path.write_text(json.dumps(data))
def rows(path,data):path.write_text(''.join(json.dumps(x,separators=(',',':'))+'\n' for x in data))
def digest(data):return hashlib.sha256(data).hexdigest()
def payload(q,v,kp,kd):
 p=int((q+12.5)/25*65535);v=int((v+50)/100*4095);kp=int(kp/500*4095);kd=int(kd/5*4095);t=2047
 return bytes([p>>8,p&255,v>>4,((v&15)<<4)|(kp>>8),kp&255,kd>>4,((kd&15)<<4)|(t>>8),t&255]).hex()

def make_trial(path,speed=13.2,actual_speed=None,growing=False):
 path.mkdir();start=100.;g=trial.geometry(speed,'release');release=start+g['release_time']+.001;stop=start+g['duration']+.005
 actual=speed if actual_speed is None else actual_speed;origin={1:20.,2:-40.,3:-10.5};commands=[];targets=[];wire=[];feedback=[];states=[];bilateral=[]
 drive_source='rig_release_synthetic';source='bilateral_synthetic';last_target=None
 count=int(g['duration']/.002-1e-8)
 for i in range(1,count+1):
  elapsed=i*.002;t=start+elapsed;q,v,phase=trial.reference(elapsed,origin[3],speed,'release');neutral=phase=='release_neutral';kp,kd=(0.,0.) if neutral else (10.,.6)
  command=dict(time=t,elapsed=elapsed,axis=3,sequence=i,q=q,v=v,kp=kp,kd=kd,torque=0.,phase=phase);commands.append(command)
  target=dict(mode=1,position=q,velocity=v,kp=kp,kd=kd,torque=0.)
  targets.append(dict(host_time=t+.0002,axis=3,client_id=drive_source,sequence=i,target=target))
  wire.append(dict(kind='TX',received_steady=round((t+.001)*1e9),host_ns=round((t+.0012)*1e9),can_id=3,length=8,data_hex=payload(q,v,kp,kd)))
 for i in range(int((g['duration']+.3)/.002)+1):
  t=start+i*.002;u=t-release;is_active=t<stop
  if u<0:
   q,v,_=trial.reference(max(0,t-start-.001),0.,speed,'release');q*=actual/speed;v*=actual/speed
  elif growing:q=g['distance']*actual/speed+actual*(u+.1*u*u);v=actual*(1+.2*u)
  else:q=g['distance']*actual/speed+actual*.2*(1-math.exp(-u/.2));v=actual*math.exp(-u/.2)
  if not is_active:v=0.
  for a in (1,2,3):
   sign=1 if a==3 else -1;qa=origin[a]+sign*q;velocity=sign*v;neutral=t>=release;src=drive_source if a==3 else source
   target=dict(mode=1,position=0. if neutral else -10.,velocity=0. if neutral else speed,kp=0. if neutral else 10.,kd=0. if neutral else .6,torque=0.)
   feedback.append(dict(axis=a,receive_time=t,host_time=t+.0005,feedback_count=10000*a+i,position=qa,velocity=velocity,torque=.01,absolute_position_valid=True,target_active=is_active))
   states.append(dict(axis=a,host_time=t+.0006,feedback_receive_time=t,feedback_age=0.,ready=True,absolute_position_valid=True,phase=2 if is_active else 0,status=int(is_active),target_active=is_active,active_source=src if is_active else '',task_active=False,host_fault=False,device_fault=False,error='',pmax=12.5,vmax=50.,tmax=10.,direction=1.,software_offset=0.,active_target=target))
   wire.append(dict(kind='RX',received_steady=round(t*1e9),host_ns=round((t+.0004)*1e9),can_id=0,length=8,data_hex=f'{a:02x}'+'00'*7))
  if i%10==0:bilateral.append(dict(host_time=t+.0005,phase=3 if is_active else 0,quality=0,operator_guarded=is_active,recovery_scale=1.,controller_source=source,protocol_saturated_a=False,protocol_saturated_b=False,disturbance_active=False,angle_offset_active=False,position_error=0.))
 wire.append(dict(kind='TX',received_steady=round((stop+.0001)*1e9),host_ns=round((stop+.0002)*1e9),can_id=3,length=8,data_hex='fffffffffffffffd'))
 wire.sort(key=lambda x:x['received_steady']);sources={}
 for name in ('run_trial.py','native_stream/stream.cpp','native_stream/CMakeLists.txt','stop_guard.py','record_only.py','native_stream/build_info.hpp.in'):
  content=('fixture source '+name).encode();dst=path/'sources'/name;dst.parent.mkdir(parents=True,exist_ok=True);dst.write_bytes(content);sources[name]=digest(content)
 inputs={}
 for key in ('config_file','system_parameters_file'):
  content=(key+' fixture').encode();(path/(key+'.yaml')).write_bytes(content);inputs[key]={'sha256':digest(content)}
 result=dict(protocol='release',protocol_version=2,success=True,error='',configuration_restored=True,stop_confirmed=True,started=start,stop_request_time=stop,geometry=g,requested_speed=speed,driver_kp=10.,driver_kd=.6,driver_source=drive_source,controller_source=source,condition='bilateral_friction',driver_bandwidth={'verified':True,'actual':40.},registry_sha256=inputs['system_parameters_file']['sha256'],input_hashes=inputs,script_sha256=sources['run_trial.py'],stream_binary_sha256='fixture-binary',native_stream={'success':True,'commands':len(commands)},effective_configuration={'stiffness':50.,'friction_compensation_enabled':True},origins={str(a):v for a,v in origin.items()})
 dump(path/'result.json',result);dump(path/'guard-result.json',dict(success=True,stop_confirmed=True,stationary_observed=True,stop_requested=stop+.0002,post_stop_max_displacement={'1':0.,'2':0.,'3':0.}));dump(path/'stream-result.json',dict(success=True,protocol='release',protocol_version=2,commands=len(commands)));dump(path/'recorder-result.json',dict(success=True));dump(path/'provenance.json',dict(source_hashes=sources,stream_binary_sha256='fixture-binary',embedded_build_info=dict(source_sha256=sources['native_stream/stream.cpp'],cmake_sha256=sources['native_stream/CMakeLists.txt'])))
 for name,data in [('commands',commands),('targets',targets),('wire',wire),('feedback',feedback),('state',states),('bilateral',bilateral)]:rows(path/(name+'.jsonl'),data)
 return path

def test_actual_wire_anchor_full_neutral_and_multiturn_position_pass(tmp_path):
 p=make_trial(tmp_path/'complete');r=score.score_trial(p)
 assert r['formal_eligible'],r['reasons'];assert r['passed'],r
 assert r['release_elapsed']==pytest.approx(1.781)
 assert r['neutral_decoded_torque_nm']==pytest.approx(-10/4095)
 assert r['axes']['1']['pre_release_signed_ols']==pytest.approx(-13.2)
 assert r['axes']['2']['settled_confirmed_by_seconds']<5.
 assert r['caveat'].startswith('C remains mechanically connected')

def test_early_stop_never_uses_disabled_tail_and_failure_never_formally_scores(tmp_path):
 p=make_trial(tmp_path/'early');result=json.loads((p/'result.json').read_text());result.update(success=False,stop_request_time=result['started']+2.);dump(p/'result.json',result)
 r=score.score_trial(p);assert not r['formal_eligible'];assert not r['passed'];assert r['axes']=={}
 assert 'STOP before full5s actual-wire observation' in r['reasons']

def test_delayed_wire_transition_shortens_real_observation(tmp_path):
 p=make_trial(tmp_path/'late');data=score.common.rows(p/'wire.jsonl')
 # A command-phase marker is insufficient: make the first real neutral TX late.
 for x in data:
  if score.mit_codes(x)==score.ZERO:x['received_steady']+=100_000_000
 rows(p/'wire.jsonl',data);r=score.score_trial(p)
 assert not r['formal_eligible'];assert 'STOP before full5s actual-wire observation' in r['reasons']

def test_reengaged_C_or_source_loss_rejects_complete_looking_tail(tmp_path):
 p=make_trial(tmp_path/'reengage');data=score.common.rows(p/'wire.jsonl');zero=[i for i,x in enumerate(data) if score.mit_codes(x)==score.ZERO];data[zero[100]]['data_hex']=payload(0.,0.,10.,.6);rows(p/'wire.jsonl',data)
 r=score.score_trial(p);assert not r['formal_eligible'];assert any('lost neutral' in x for x in r['reasons'])
 p=make_trial(tmp_path/'source');data=score.common.rows(p/'state.jsonl');data[len(data)//2]['active_source']='another_controller';rows(p/'state.jsonl',data)
 r=score.score_trial(p);assert not r['formal_eligible'];assert any('source/active' in x for x in r['reasons'])

def test_feedback_missing_raw_mismatch_and_input_tampering_are_explicit(tmp_path):
 p=make_trial(tmp_path/'missing');data=score.common.rows(p/'feedback.jsonl');del data[len(data)//2];rows(p/'feedback.jsonl',data)
 r=score.score_trial(p);assert not r['formal_eligible'];assert any('count discontinuity' in x for x in r['reasons'])
 p=make_trial(tmp_path/'raw');data=score.common.rows(p/'wire.jsonl');rx=[i for i,x in enumerate(data) if x['kind']=='RX'];del data[rx[len(rx)//2]];rows(p/'wire.jsonl',data)
 r=score.score_trial(p);assert not r['formal_eligible'];assert any('unmatched raw RX' in x for x in r['reasons'])
 (p/'system_parameters_file.yaml').write_text('changed')
 assert any('hash mismatch' in x for x in score.score_trial(p)['reasons'])

def test_unreached_four_pi_and_growing_motion_fail_metrics(tmp_path):
 r=score.score_trial(make_trial(tmp_path/'slow',actual_speed=12.55));assert r['formal_eligible'],r['reasons'];assert not r['passed'];assert not r['axes']['1']['reached']
 r=score.score_trial(make_trial(tmp_path/'growing',growing=True));assert r['formal_eligible'],r['reasons'];assert not r['passed'];assert r['axes']['2']['sustained_growth'];assert r['axes']['2']['settled_confirmed_by_seconds'] is None


def test_protocol_versions_do_not_mix_or_relabel_windows(tmp_path):
 p=make_trial(tmp_path/'version');result=json.loads((p/'result.json').read_text())
 result['protocol_version']=1;dump(p/'result.json',result)
 r=score.score_trial(p);assert not r['formal_eligible'];assert 'Native protocol declaration differs' in r['reasons'];assert 'Release geometry mismatch' in r['reasons']
 result['protocol_version']=3;dump(p/'result.json',result)
 assert 'Unknown release protocol version' in score.score_trial(p)['reasons']


def test_release_delayed_state_delivery_passes_but_driver_stale_or_gap_fails(tmp_path):
 p=make_trial(tmp_path/'delayed-state');data=score.common.rows(p/'state.jsonl')
 for x in data:x['host_time']+=.04
 rows(p/'state.jsonl',data);r=score.score_trial(p)
 assert r['formal_eligible'],r['reasons'];assert r['passed']
 assert r['recording_audit']['state_timing']['3']['callback_delay_s']['max']==pytest.approx(.0406)
 # Preserve the snapshot time, but make the underlying driver feedback stale.
 for x in data:x['feedback_receive_time']-=.021;x['feedback_age']=.021
 rows(p/'state.jsonl',data);r=score.score_trial(p)
 assert not r['formal_eligible'];assert any('freshness invalid' in s for s in r['reasons'])
 p=make_trial(tmp_path/'source-gap');data=score.common.rows(p/'state.jsonl')
 data=[x for x in data if not 102.<x['feedback_receive_time']<102.04]
 rows(p/'state.jsonl',data);r=score.score_trial(p)
 assert not r['formal_eligible'];assert any('source-time coverage gap' in s for s in r['reasons'])


def boundary_samples():
 # The final ordinary4.7ms receive interval straddles the exact100ms boundary.
 times=[-.001]+[.001+i*.002 for i in range(48)]+[.0956,.1003]
 return [dict(receive_time=t,position=5*t,velocity=999. if t<0 or t>.1 else 4.,torque=888.) for t in times]


def test_fixed_window_uses_bounded_position_endpoints_without_fake_speed_samples():
 data=boundary_samples();w=score.window(data,0.,.1)
 assert w is not None
 assert w[0]['receive_time']==0. and w[-1]['receive_time']==.1
 assert w[0]['position']==pytest.approx(0.) and w[-1]['position']==pytest.approx(.5)
 for x in (w[0],w[-1]):
  assert x['interpolated_boundary'];assert x['bracket_gap']<=.020
  assert 'velocity' not in x and 'torque' not in x
 assert score.ols(w)==pytest.approx(5.)
 assert score.motion(w)['reported_abs_max']==4.  # Outside999 is never interpolated in.
 assert score.motion(w)['position_range']==pytest.approx(.5)


def test_fixed_window_rejects_missing_bracket_and_real_internal_or_edge_hole():
 data=boundary_samples()
 assert score.window(data[1:],0.,.1) is None  # No left bracket; no extrapolation.
 assert score.window(data[:-1],0.,.1) is None  # No right bracket.
 assert score.window([x for x in data if not .03<x['receive_time']<.06],0.,.1) is None
 edge=[dict(x) for x in data];edge[-1]['receive_time']=.12  # Edge gap24.4ms.
 assert score.window(edge,0.,.1) is None
