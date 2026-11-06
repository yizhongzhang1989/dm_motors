"""Synthetic full-artifact pilot and failure mutations. Never ROS/hardware."""
import bisect
import hashlib
import json
from pathlib import Path
import shutil
import struct

import pytest
from long_protocol import Protocol,command_row
from run_trial import geometry
from score_long import score_trial

HERE=Path(__file__).resolve().parent

def dump(path,value):path.write_text(json.dumps(value,separators=(',',':'))+'\n')
def rows(path,values):
    with path.open('w') as stream:
        for value in values:stream.write(json.dumps(value,separators=(',',':'))+'\n')
def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def stamp(stamp_ns):return dict(sec=stamp_ns//1000000000,nanosec=stamp_ns%1000000000)
def mapped_clock(t):
    source_ns=round((t+1000)*1e9);host=t+.03;before=host+.0001;after=before+.00002
    ros_ns=round(((before+after)/2+1000)*1e9)
    return dict(header=dict(stamp=stamp(source_ns)),host_time=host,clock_before=before,clock_after=after,
                clock_ros_ns=ros_ns,source_time=(before+after)/2+(source_ns-ros_ns)*1e-9,
                source_time_uncertainty=(after-before)/2)
def payload(q,v,kp,kd):
    p=int((q+12.5)/25*65535);v=int((v+50)/100*4095);kp=int(kp/500*4095);kd=int(kd/5*4095);tau=2047
    return bytes([p>>8,p&255,v>>4,((v&15)<<4)|(kp>>8),kp&255,kd>>4,((kd&15)<<4)|(tau>>8),tau&255]).hex()


def make_trial(path):
    path.mkdir();plan=Protocol(pilot=True);start=1000.;stop=start+plan.duration+.005
    names=('commands','targets','wire','feedback','state','bilateral','bus','events')
    files={name:(path/(name+'.jsonl')).open('w') for name in names};counts={name:0 for name in names}
    source='synthetic_bilateral';driver='synthetic_driver';q0=-10.;n=round(plan.duration/.002)
    source_stamps={'RX':[],'TX':[]};maxgap=0.;last=None
    def emit(name,value):files[name].write(json.dumps(value,separators=(',',':'))+'\n');counts[name]+=1
    def wire(kind,t,axis,data):
        source_ns=round((t+1000+(.00005 if kind=='RX' else -.0000001))*1e9)
        source_stamps[kind].append(source_ns)
        emit('wire',dict(kind=kind,received_steady=round(t*1e9),host_ns=round((t+.01)*1e9),source_stamp_ns=source_ns,
                         can_id=0 if kind=='RX' else axis,length=8,data_hex=data,fd=False,extended=False,rtr=False,error=False))
    for i in range(-10,n+300):
        elapsed=i*.002+.0005;t=start+elapsed
        r=plan.reference(min(plan.duration,max(0.,elapsed)),q0)
        if 0<=i<n:
            command=command_row(elapsed,i+1,r,start_clock=start)
            if last is not None:maxgap=max(maxgap,command['time']-last)
            last=command['time'];emit('commands',command)
            for axis in (1,2,3):
                target=dict(mode=1,position=r.q if axis==3 else 0.,velocity=r.v if axis==3 else 0.,kp=10. if axis==3 else 0.,kd=.6 if axis==3 else 1.,torque=0.)
                emit('targets',dict(axis=axis,host_time=t+.0001,topic=f'/motor_{axis}/'+('target' if axis==3 else 'setpoint'),client_id=driver if axis==3 else source,sequence=i+1,target=target))
        for axis in (1,2,3):
            wire('TX',t+.0002,axis,payload(r.q if axis==3 else 0.,r.v if axis==3 else 0.,10. if axis==3 else 0.,.6 if axis==3 else 1.))
        for axis in (1,2,3):
            receive=t+.0004;active=receive<stop;position=r.q if axis==3 else -(r.q-q0);velocity=r.v if axis==3 else -r.v
            feedback=dict(axis=axis,receive_time=receive,host_time=receive+.01,feedback_count=100000*axis+i,
                          position=position,velocity=velocity,torque=0.,absolute_position_valid=True,mos_temperature=25,rotor_temperature=25)
            emit('feedback',feedback)
            wire('RX',receive,axis,bytes([(16 if active else 0)|axis,0,0,0,0,0,25,25]).hex())
            snapshot=receive+.0001
            emit('state',dict(axis=axis,host_time=snapshot+.03,feedback_receive_time=receive,feedback_age=.0001,
                feedback_count=feedback['feedback_count'],phase=2 if active else 0,status=int(active),mode=1,
                ready=True,target_active=active,active_source=(driver if axis==3 else source) if active else '',
                task_active=False,simulated=False,host_fault=False,device_fault=False,error='',absolute_position_valid=True,
                rejected_commands=0,applied_target_sequence=max(1,i+1) if active else 0,velocity=velocity))
        if i%10==0:
            source_time=t+.0006;active=source_time<stop
            emit('bilateral',dict(**mapped_clock(source_time),phase=3 if active else 0,quality=0,operator_guarded=active,
                recovery_scale=1.,controller_source=source,protocol_saturated_a=False,protocol_saturated_b=False,
                disturbance_active=False,angle_offset_active=False,angle_offset_ramping=False,angle_offset_applied=0.,
                position_error=0.,stop_confirmed=not active))
    for i in range(-1,int((plan.duration+.59)/.02)+1):
        t=start+i*.02+.0003;source_ns=round((t+1000)*1e9)
        emit('bus',dict(**mapped_clock(t),epoch=1,ready=True,endpoint_ready=True,backend='serial',error='',
            group_faults=[],client_faults=[False]*3,observer_dropped=0,observer_queue_depth=0,rejected=0,
            rx_count=1000+bisect.bisect_left(source_stamps['RX'],source_ns),tx_count=2000+bisect.bisect_left(source_stamps['TX'],source_ns)))
    emit('events',dict(event='NATIVE_STREAM',message=dict(type='START',started=start)))
    emit('events',dict(event='NATIVE_STREAM',message=dict(type='DONE',success=True)))
    emit('events',dict(event='STOP_REQUESTED',time=stop))
    for stream in files.values():stream.close()
    hashes={}
    for name in ('run_trial.py','record_only.py','stop_guard.py','long_protocol.py','long_metrics.py','long_audit_core.py',
                 'long_wire_audit.py','score_long.py','score_trials.py','native_stream/stream.cpp','native_stream/CMakeLists.txt'):
        target=path/'sources'/name;target.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(HERE/name,target);hashes[name]=digest(target)
    inputs={}
    for name in ('config_file','system_parameters_file'):
        p=path/(name+'.yaml');p.write_text('synthetic offline test only\n');inputs[name]=dict(sha256=digest(p))
    bandwidth=dict(actual=80.,required=80.,verified=True,register_address=32,read_only=True,operation_id=1,
                   confirmed_time=start-.1,raw_value=struct.unpack('<I',struct.pack('<f',80.))[0])
    result=dict(synthetic_offline=True,protocol='long_variable_pilot',protocol_version=1,success=True,error='',
        started=start,stop_request_time=stop,geometry=geometry(13.2,'long_variable_pilot'),condition='bilateral_friction',
        stop_confirmed=True,configuration_restored=True,driver_source=driver,controller_source=source,driver_kp=10.,driver_kd=.6,
        driver_bandwidth=bandwidth,origins={'1':0.,'2':0.,'3':q0},guards=dict(speed=15.9,torque=3.8,travel=20.1,temperature=48),
        script_sha256=hashes['run_trial.py'],stream_binary_sha256='synthetic-not-an-executable',input_hashes=inputs,
        registry_sha256=inputs['system_parameters_file']['sha256'])
    dump(path/'result.json',result)
    dump(path/'stream-result.json',dict(success=True,commands=n,max_gap=maxgap,protocol='long_variable_pilot',log_buffer_bytes=64*1024*1024))
    dump(path/'guard-result.json',dict(success=True,stop_confirmed=True,stationary_observed=True,reason='Parent FINISH'))
    dump(path/'recorder-result.json',dict(success=True,error='',wire_source_stamps=True,counts=counts))
    dump(path/'provenance.json',dict(source_hashes=hashes,stream_binary_sha256='synthetic-not-an-executable',
        embedded_build_info=dict(source_sha256=hashes['native_stream/stream.cpp'],cmake_sha256=hashes['native_stream/CMakeLists.txt'])))
    return path


@pytest.fixture(scope='module')
def pilot(tmp_path_factory):return make_trial(tmp_path_factory.mktemp('long_artifact_parent')/'pilot')


def test_complete_raw_pilot_passes_but_never_final_acceptance(pilot):
    result=score_trial(pilot)
    assert result['passed'],result['audit']
    assert result['pilot_passed'] and not result['final_acceptance']
    assert len(result['wire']['steps'])==24
    assert len(result['feedback']['speed_windows']['windows'])==36
    assert result['states']['callback_delay'][1]['mean']==pytest.approx(.03)


def test_raw_disable_and_truncated_feedback_cannot_pass(pilot,tmp_path):
    for mode in ('disable','missing_feedback'):
        target=tmp_path/mode;shutil.copytree(pilot,target)
        name='wire.jsonl' if mode=='disable' else 'feedback.jsonl'
        data=[json.loads(line) for line in (target/name).read_text().splitlines()]
        if mode=='disable':
            item=next(row for row in data if row.get('kind')=='TX' and row['can_id']==3 and row['received_steady']>1002*1e9)
            item['data_hex']='fffffffffffffffd'
        else:data.pop(len(data)//2)
        rows(target/name,data);result=score_trial(target)
        assert not result['passed'] and not result['final_acceptance'] and not result['pilot_passed']
        expected='non_mit_CAN_control_during_motion' if mode=='disable' else 'feedback_count_discontinuity'
        assert expected in result['audit']['failures'],result['audit']


def test_pilot_relabel_or_command_file_truncation_cannot_pass(pilot,tmp_path):
    for mode in ('relabel','truncate'):
        target=tmp_path/mode;shutil.copytree(pilot,target)
        if mode=='relabel':
            result=json.loads((target/'result.json').read_text());result['protocol']='long_variable';dump(target/'result.json',result)
        else:
            p=target/'commands.jsonl';p.write_bytes(p.read_bytes()[:-2])
        result=score_trial(target)
        assert not result['passed'] and not result['final_acceptance']
