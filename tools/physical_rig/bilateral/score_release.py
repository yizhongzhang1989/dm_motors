#!/usr/bin/env python3
"""Offline artifact-aware C-neutral release scoring; no ROS or hardware access.

C remains mechanically connected. STOP tails never complete an observation.
"""
import bisect,math,statistics
SCORING_REVISION='driver-source-time-and-bounded-position-boundaries-v2'
WINDOW_METHOD='Exact fixed time boundaries bracketed by original feedback, all gaps<=20ms; linear position-only endpoints, no extrapolation; reported velocity/torque statistics use original samples only'
CRITERIA=dict(observation=5.,window=.5,ols_stop=.02,position_range_stop=.01,
              reported_speed_p95_stop=.05,reported_speed_max_stop=.1,
              growth_fraction=.10,growth_floor=.2,acquisition_tolerance=.05)
def ols(rows):
 ts=[x['receive_time'] for x in rows];qs=[x['position'] for x in rows];mt=statistics.fmean(ts);mq=statistics.fmean(qs)
 return sum((t-mt)*(q-mq) for t,q in zip(ts,qs))/sum((t-mt)**2 for t in ts)
def pct(xs,p):
 xs=sorted(xs);return xs[min(len(xs)-1,int((len(xs)-1)*p))]
def window(rows,lo,hi):
 # The fixed boundary can lie inside an ordinary receive interval. Require
 # actual samples on both sides, including the complete edge-bracketing gaps.
 # Interpolate position only; never manufacture a velocity or torque sample.
 times=[x['receive_time'] for x in rows]
 if not times or not times[0]<=lo<hi<=times[-1]:return None
 left=bisect.bisect_right(times,lo)-1;right=bisect.bisect_left(times,hi)
 bracketed=rows[left:right+1]
 if any(not 0<=b['receive_time']-a['receive_time']<=.020 for a,b in zip(bracketed,bracketed[1:])):return None
 result=rows[bisect.bisect_left(times,lo):bisect.bisect_right(times,hi)]
 if len(result)<3:return None
 def endpoint(t,a,b):
  dt=b['receive_time']-a['receive_time']
  u=(t-a['receive_time'])/dt
  return dict(receive_time=t,position=a['position']+(b['position']-a['position'])*u,
              interpolated_boundary=True,bracket_gap=dt)
 if result[0]['receive_time']>lo:result.insert(0,endpoint(lo,rows[left],rows[left+1]))
 if result[-1]['receive_time']<hi:result.append(endpoint(hi,rows[right-1],rows[right]))
 return result
def motion(rows):
 q=[x['position'] for x in rows];v=[abs(x['velocity']) for x in rows if not x.get('interpolated_boundary',False)]
 return dict(ols=ols(rows),position_range=max(q)-min(q),reported_abs_max=max(v),reported_abs_p95=pct(v,.95))
def stationary(m):
 return abs(m['ols'])<=CRITERIA['ols_stop'] and m['position_range']<=CRITERIA['position_range_stop'] and m['reported_abs_p95']<=CRITERIA['reported_speed_p95_stop'] and m['reported_abs_max']<=CRITERIA['reported_speed_max_stop']
def score_motion(axes,release,stop,requested_speed,*,protocol_verified=False):
 end=release+CRITERIA['observation'];reasons=[]
 if not protocol_verified:reasons.append('Artifact/source/neutral-wire/protocol gates not verified')
 if stop<end:reasons.append('STOP before full5s observation: never use disabled tail')
 out=dict(formal_eligible=False,passed=False,reasons=reasons,criteria=CRITERIA,axes={},C_mechanically_connected=True)
 if reasons:return out
 for a in [1,2,3]:
  rows=sorted(axes[a],key=lambda x:x['receive_time']);pre=window(rows,release-.15,release-.03);obs=window(rows,release,end);tail=window(rows,end-.5,end)
  if pre is None or obs is None or tail is None:reasons.append(f'Axis{a} coverage missing');continue
  pre_speed=ols(pre);expected=1 if a==3 else -1;achieved=expected*pre_speed
  floor=(1-CRITERIA['acquisition_tolerance'])*requested_speed
  if requested_speed>=4*math.pi:floor=max(floor,4*math.pi)
  reached=floor<=achieved<=(1+CRITERIA['acquisition_tolerance'])*requested_speed
  # Position-based100ms envelopes: the raw reported peak remains separately visible.
  bins=[window(rows,release+i*.05,release+i*.05+.1) for i in range(99)]
  if any(x is None for x in bins):reasons.append(f'Axis{a} envelope coverage missing');continue
  envelope=max(abs(ols(x)) for x in bins);tol=max(CRITERIA['growth_floor'],CRITERIA['growth_fraction']*abs(pre_speed))
  tails=[window(rows,release+i*.05-.5,release+i*.05) for i in range(10,101)]
  if any(x is None for x in tails):reasons.append(f'Axis{a} rolling-tail coverage missing');continue
  checks=[stationary(motion(x)) for x in tails]
  settled=None
  for i in range(len(checks)):
   if all(checks[i:]):settled=(i+10)*.05;break
  out['axes'][str(a)]=dict(pre_release_signed_ols=pre_speed,reached=reached,
    post_release_100ms_envelope_abs_peak=envelope,reported_abs_peak=max(abs(x['velocity']) for x in obs if not x.get('interpolated_boundary',False)),
    growth_velocity_tolerance=tol,
    sustained_growth=envelope>abs(pre_speed)+tol,settled_confirmed_by_seconds=settled,
    final_half_second=motion(tail),coast_displacement=obs[-1]['position']-obs[0]['position'],
    max_coast_excursion=max(abs(x['position']-obs[0]['position']) for x in obs))
 out['formal_eligible']=not reasons and len(out['axes'])==3
 out['passed']=out['formal_eligible'] and all(x['reached'] and not x['sustained_growth'] and x['settled_confirmed_by_seconds'] is not None for x in out['axes'].values())
 return out

# Artifact-aware release scorer. No ROS imports or live binary reads.
import argparse,hashlib,importlib.util,json
from pathlib import Path
_spec=importlib.util.spec_from_file_location('release_common_score',Path(__file__).with_name('score_trials.py'))
common=importlib.util.module_from_spec(_spec);_spec.loader.exec_module(common)
FILES=('result.json','guard-result.json','recorder-result.json','stream-result.json',
       'provenance.json','commands.jsonl','targets.jsonl','feedback.jsonl','state.jsonl','bilateral.jsonl','wire.jsonl')
CAVEAT='C remains mechanically connected and enabled with zero requested PD/torque. Its friction, inertia, internal losses and MIT zero-torque quantization bias remain. This is not a disconnected-C release or an individual-A friction measurement.'

def finite_number(v):return type(v) in (int,float) and math.isfinite(v)
def mit_codes(row):
 if row.get('kind')!='TX' or row.get('can_id')!=3 or row.get('length')!=8:return None
 try:d=bytes.fromhex(row['data_hex'])
 except (ValueError,TypeError):return None
 if len(d)!=8 or d[:7]==b'\xff'*7:return None
 return ((d[0]<<8)|d[1],(d[2]<<4)|(d[3]>>4),((d[3]&15)<<8)|d[4],(d[5]<<4)|(d[6]>>4),((d[6]&15)<<8)|d[7])
ZERO=(32767,2047,0,0,2047)
def requested_zero(c):return all(c.get(k)==0 for k in ('q','v','kp','kd','torque'))
def target_zero(t):return t.get('mode')==1 and all(t.get(k)==0 for k in ('position','velocity','kp','kd','torque'))
def encoded_gain(value,limit):return int(value/limit*4095)

def score_trial(path):
 path=Path(path);reasons=[];out=dict(trial=path.name,path=str(path),formal_eligible=False,passed=False,
   reasons=reasons,criteria=CRITERIA,caveat=CAVEAT,C_mechanically_connected=True,axes={},scoring_revision=SCORING_REVISION,window_method=WINDOW_METHOD)
 missing=[f for f in FILES if not (path/f).is_file()]
 if missing:reasons.append('Missing artifacts: '+', '.join(missing));return out
 try:
  r=json.loads((path/'result.json').read_text());guard=json.loads((path/'guard-result.json').read_text());rec=json.loads((path/'recorder-result.json').read_text());native=json.loads((path/'stream-result.json').read_text());prov=json.loads((path/'provenance.json').read_text())
  logs={f:common.rows(path/(f+'.jsonl')) for f in ('commands','targets','feedback','state','bilateral','wire')}
  start=r.get('started');speed=r.get('requested_speed');kp=r.get('driver_kp');kd=r.get('driver_kd')
  if r.get('protocol')!='release':reasons.append('Not a release protocol');return out
  version=r.get('protocol_version',1)
  if type(version)is not int or version not in (1,2):reasons.append('Unknown release protocol version');return out
  plateau=.2 if version==1 else .6
  out['protocol_version']=version
  if not all(finite_number(x) for x in (start,speed,kp,kd)) or not 0<speed<=13.3 or not 0<kp<=30 or not 0<kd<=1:reasons.append('Invalid release metadata');return out
  for label,valid in [('runner',r.get('success') is True),('restore',r.get('configuration_restored') is True),('parent STOP',r.get('stop_confirmed') is True),('independent guard',guard.get('success') is True and guard.get('stop_confirmed') is True and guard.get('stationary_observed') is True),('native sender',native.get('success') is True and (r.get('native_stream') or {}).get('success') is True),('recorder',rec.get('success') is True)]:
   if not valid:reasons.append(label+' failed')
  if r.get('condition') not in ('bilateral_friction','bilateral_no_friction'):reasons.append('Release requires A/B bilateral condition')
  source=r.get('controller_source');driver_source=r.get('driver_source')
  if not isinstance(source,str) or not source or not isinstance(driver_source,str) or not driver_source:reasons.append('Missing declared A/B or C command source')
  cohort=common.bandwidth_cohort(r,path.name)
  if not cohort['verified']:reasons.append('C bandwidth not independently verified')
  cohort['bandwidth_verified']=cohort['verified'];cohort['verified']=False;cohort['key']=None
  cohort.update(protocol='release',protocol_version=version,plateau_seconds=plateau,driver_gains={'kp':kp,'kd':kd},controller_source=source,
                registry_sha256=r.get('registry_sha256'),effective_configuration=r.get('effective_configuration'))
  out['cohort']=cohort
  if not isinstance(r.get('registry_sha256'),str) or not r['registry_sha256']:reasons.append('Missing registry SHA')
  # The copied inputs make a later offline review independent of the current checkout.
  for key,entry in (r.get('input_hashes') or {}).items():
   copy=path/(key+'.yaml')
   if not copy.is_file() or hashlib.sha256(copy.read_bytes()).hexdigest()!=entry.get('sha256'):reasons.append('Input hash mismatch: '+key)
  if not all(k in (r.get('input_hashes') or {}) for k in ('config_file','system_parameters_file')):reasons.append('Missing copied config/registry hash evidence')
  if (r.get('input_hashes') or {}).get('system_parameters_file',{}).get('sha256')!=r.get('registry_sha256'):reasons.append('Registry SHA declaration differs')
  sources=prov.get('source_hashes') or {}
  for name,expected in sources.items():
   if Path(name).is_absolute() or '..' in Path(name).parts:reasons.append('Invalid copied-source path');continue
   copy=path/'sources'/name
   if not copy.is_file() or hashlib.sha256(copy.read_bytes()).hexdigest()!=expected:reasons.append('Copied source hash mismatch: '+name)
  for name in ('run_trial.py','native_stream/stream.cpp','native_stream/CMakeLists.txt','stop_guard.py','record_only.py','native_stream/build_info.hpp.in'):
   if name not in sources:reasons.append('Missing source provenance: '+name)
  if sources.get('run_trial.py')!=r.get('script_sha256'):reasons.append('Runner hash declaration differs')
  if prov.get('stream_binary_sha256')!=r.get('stream_binary_sha256') or not r.get('stream_binary_sha256'):reasons.append('Native binary provenance differs')
  embedded=prov.get('embedded_build_info') or {}
  if embedded.get('source_sha256')!=sources.get('native_stream/stream.cpp') or embedded.get('cmake_sha256')!=sources.get('native_stream/CMakeLists.txt'):reasons.append('Native build/source provenance differs')
  commands=logs['commands'];neutral=[i for i,c in enumerate(commands) if c.get('phase')=='release_neutral']
  if not neutral or neutral[0]==0:reasons.append('Missing drive-to-neutral transition');return out
  first=neutral[0];release_publish=commands[first]['time'];planned=.3+max(.25,speed/15)+plateau
  if not finite_number(release_publish) or not start+planned<=release_publish<=start+planned+.010:reasons.append('First neutral publish outside planned transition')
  if any(c.get('axis')!=3 or not finite_number(c.get('time')) or not finite_number(c.get('elapsed')) or abs(c['time']-start-c['elapsed'])>1e-6 for c in commands):reasons.append('Invalid command clock/axis')
  sequence=[c.get('sequence') for c in commands]
  if not all(type(v)is int for v in sequence) or sequence!=list(range(1,len(commands)+1)):reasons.append('Native command sequence missing/replayed')
  if native.get('commands')!=len(commands) or (r.get('native_stream') or {}).get('commands')!=len(commands):reasons.append('Native command count mismatch')
  if native.get('protocol')!='release' or native.get('protocol_version',1)!=version:reasons.append('Native protocol declaration differs')
  if any(c.get('kp')!=kp or c.get('kd')!=kd or c.get('torque')!=0 or c.get('phase') not in ('baseline','ramp','release_steady') for c in commands[:first]):reasons.append('Drive gains/phase mismatch')
  if any(c.get('phase')!='release_neutral' or not requested_zero(c) for c in commands[first:]):reasons.append('Non-neutral command after release')
  if any(not 0<b['time']-a['time']<=.010+1e-9 for a,b in zip(commands,commands[1:])):reasons.append('Command time regression/gap>10ms')
  geometry=r.get('geometry') or {}
  if geometry.get('protocol_version',1)!=version or geometry.get('plateau')!=plateau or geometry.get('observation')!=5. or abs(geometry.get('release_time',-1)-planned)>1e-9 or geometry.get('duration',0)<planned+5.05-1e-9:reasons.append('Release geometry mismatch')
  if commands[-1]['elapsed']<geometry.get('duration',0)-.010-1e-9:reasons.append('Native protocol ended early')
  stops=[v for v in (r.get('stop_request_time'),guard.get('stop_requested')) if finite_number(v)]
  disables=[x['received_steady']*1e-9 for x in logs['wire'] if x.get('kind')=='TX' and x.get('can_id') in (1,2,3) and x.get('data_hex')=='fffffffffffffffd' and x['received_steady']*1e-9>start]
  if len(stops)!=2:reasons.append('Missing STOP timestamp');return out
  stop=min(stops+disables)
  wire=sorted([dict(x,time=x['received_steady']*1e-9,codes=mit_codes(x)) for x in logs['wire'] if mit_codes(x) is not None],key=lambda x:x['time'])
  zero=[x for x in wire if x['time']>=release_publish-1e-6 and x['codes']==ZERO]
  if not zero:reasons.append('No actual C zero-PD wire transition');return out
  release=zero[0]['time'];end=release+5.;lo=release-.15
  out.update(release_time=release,release_elapsed=release-start,release_publish_elapsed=release_publish-start,
             stop_time=stop,pre_stop_observation_seconds=stop-release,neutral_decoded_torque_nm=-10/4095.)
  if release-release_publish>.04:reasons.append('Neutral wire arrived after target validity')
  if end>stop:reasons.append('STOP before full5s actual-wire observation')
  after=[x for x in wire if release<=x['time']<stop]
  if not common.coverage(wire,'time',release,end,.020) or not after or any(x['codes']!=ZERO for x in after):reasons.append('C wire lost neutral PD/torque or coverage')
  driven=[x for x in wire if lo<=x['time']<=release-.03]
  if not driven or any(x['codes'][2:4]!=(encoded_gain(kp,500.),encoded_gain(kd,5.)) for x in driven):reasons.append('Acquisition window lacks actual drive gains')
  target_rows=[x for x in logs['targets'] if x.get('axis')==3 and x.get('client_id')==driver_source]
  if not target_rows:reasons.append('C source lacks setpoint record evidence')
  by_sequence={c['sequence']:c for c in commands}
  neutral_targets=0
  for x in target_rows:
   c=by_sequence.get(x.get('sequence'))
   if c is None:reasons.append('Unknown C setpoint sequence');continue
   t=x.get('target') or {}
   if t.get('mode')!=1 or any(t.get(k)!=c.get(v) for k,v in [('position','q'),('velocity','v'),('kp','kp'),('kd','kd'),('torque','torque')]):reasons.append('Setpoint/command mismatch')
   if c['phase']=='release_neutral' and target_zero(t):neutral_targets+=1
  if not neutral_targets:reasons.append('No recorded neutral C setpoint')
  feedback=logs['feedback'];states=logs['state'];bilateral=logs['bilateral']
  selected=[x for x in feedback if lo<=x.get('receive_time',-math.inf)<=end]
  if any(not all(finite_number(x.get(k)) for k in ('receive_time','host_time','position','velocity','torque')) or not x.get('absolute_position_valid',False) or not x.get('target_active',False) for x in selected):reasons.append('Invalid/nonactive feedback')
  axes={a:common.unique_feedback(feedback,a) for a in (1,2,3)}
  faudit=common.feedback_audit(feedback,lo,end);waudit=common.wire_audit(path,lo,end,feedback)
  out['recording_audit']={'feedback':faudit,'wire_rx':waudit,'recorded_C_targets':len(target_rows),'native_commands':len(commands),'state_timing':{}}
  for a in (1,2,3):
   aa=faudit[str(a)]
   if not aa.get('rows') or any(aa.get(k) for k in ('missing_count_values_between_endpoints','duplicate_count_transitions','count_regressions')):reasons.append(f'Axis{a} feedback count discontinuity')
   if waudit[str(a)]['feedback_rows_with_no_matching_raw_rx_timestamp_within_0_5us']:reasons.append(f'Axis{a} unmatched raw RX timestamp')
   ss,state_covered,state_audit=common.motor_state_window(states,a,lo,end)
   out['recording_audit']['state_timing'][str(a)]=state_audit
   expected_source=driver_source if a==3 else source
   if not common.coverage(axes[a],'receive_time',lo,end,.020) or not state_covered:reasons.append(f'Axis{a} state/feedback source-time coverage gap or invalid timestamp')
   if not ss or not all(x.get('ready') and x.get('absolute_position_valid') and x.get('phase')==2 and x.get('status')==1 and x.get('target_active') and x.get('active_source')==expected_source and not x.get('task_active') and not x.get('host_fault') and not x.get('device_fault') and not x.get('error') and 0<=x['feedback_age']<.020 for x in ss):reasons.append(f'Axis{a} source/active/freshness invalid')
   if a==3 and any(x.get('pmax')!=12.5 or x.get('vmax')!=50. or x.get('tmax')!=10. or x.get('direction')!=1. or x.get('software_offset')!=0. for x in ss):reasons.append('C mapping/coordinates changed')
   if a==3 and any(not target_zero(x.get('active_target') or {}) for x in ss if x['state_source_time']>=release+.04):reasons.append('C active target not neutral')
  bs=[x for x in bilateral if lo<=x.get('host_time',-math.inf)<=end]
  if not common.coverage(bilateral,'host_time',lo,end,.05) or not bs or not all(x.get('phase')==3 and x.get('quality')==0 and x.get('operator_guarded') and x.get('recovery_scale')==1 and x.get('controller_source')==source and not x.get('protocol_saturated_a') and not x.get('protocol_saturated_b') and not x.get('disturbance_active') and not x.get('angle_offset_active') for x in bs):reasons.append('A/B bilateral not continuously ACTIVE/NORMAL under same source')
  out['reasons']=list(dict.fromkeys(reasons));out['protocol_result_success']=r.get('success');out['post_stop_max_displacement']=guard.get('post_stop_max_displacement')
  out['sha256']={f:hashlib.sha256((path/f).read_bytes()).hexdigest() for f in FILES}
  if out['reasons']:return out
  cohort['verified']=True
  settings_hash=hashlib.sha256(json.dumps(r.get('effective_configuration'),sort_keys=True).encode()).hexdigest()
  cohort['key']=f"protocol=release-v{version};V_BW={cohort['motor_3_velocity_bandwidth']!r};kp={kp!r};kd={kd!r};controller={source};registry={r['registry_sha256']};settings={settings_hash}"
  metrics=score_motion(axes,release,stop,speed,protocol_verified=True)
  out.update(formal_eligible=metrics['formal_eligible'],passed=metrics['passed'],axes=metrics['axes'],reasons=metrics['reasons'])
  out['paired_state_error_diagnostic']=common.describe([x['position_error'] for x in bs])
  offsets=r.get('origins') or {};pair=[]
  if '1' in offsets and '3' in offsets:
   for x in axes[1]:
    if release<=x['receive_time']<=end:
     q=common.interp(axes[3],x['receive_time'])
     if q is not None:pair.append(x['position']+q-offsets['1']-offsets['3'])
  out['qA_plus_qC_change_diagnostic']=common.describe(pair)
  out['measured_torque_abs_peak']={str(a):max(abs(x['torque']) for x in axes[a] if release<=x['receive_time']<=end) for a in (1,2,3)}
  return out
 except (KeyError,TypeError,ValueError,IndexError,ZeroDivisionError,OverflowError) as error:
  reasons.append('Malformed/inconsistent release artifacts: '+str(error));out['reasons']=list(dict.fromkeys(reasons));return out

def main():
 p=argparse.ArgumentParser(description='Offline C-neutral release scoring; no ROS/hardware.')
 p.add_argument('--root',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('trials',nargs='+');a=p.parse_args();root=a.root.expanduser().resolve()
 if any(Path(n).is_absolute() or len(Path(n).parts)!=1 or n in ('.','..') for n in a.trials):p.error('Trial names must be direct root children')
 result={'schema':3,'protocol':'release','scoring_revision':SCORING_REVISION,'window_method':WINDOW_METHOD,'state_timebase':common.STATE_TIMEBASE,'criteria':CRITERIA,'caveat':CAVEAT,'trials':[score_trial(root/n) for n in a.trials]}
 a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
 print(json.dumps({'trials':len(result['trials']),'eligible':sum(x['formal_eligible'] for x in result['trials']),'passed':sum(x['passed'] for x in result['trials'])}))
if __name__=='__main__':main()
