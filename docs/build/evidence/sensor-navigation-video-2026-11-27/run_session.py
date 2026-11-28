"""Run one sensor-only validation; own and clean up its ROS launch processes.

Run inside the sourced Humble container. Recording is performed separately
on the host. Touch OUTPUT/recording-ready only after capture has started.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

p=argparse.ArgumentParser();p.add_argument('--root',default='/root/mrider-team');p.add_argument('--output',required=True)
a=p.parse_args();root=Path(a.root);out=Path(a.output);out.mkdir(parents=True,exist_ok=False)
env=os.environ.copy();env.update(ROS_DOMAIN_ID='192',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MRIDER_SIM_REPORT_DIR=str(out))
command=['ros2','launch','mrider_navigation_sim','sensor_only_demo.launch.py','scenario:=large_office',
         'vehicle:=1','goal_x:=24.0','auto_resume:=true','actor_time_offset_s:=227.0','diagnostics:=true','rviz:=false']

def default_signals():
    # Avoid the SIGINT-ignore disposition inherited by shell background jobs.
    signal.signal(signal.SIGINT,signal.SIG_DFL)
    signal.signal(signal.SIGTERM,signal.SIG_DFL)


def members(group):
    found=[]
    for path in Path('/proc').glob('[0-9]*/stat'):
        try:
            fields=path.read_text().rsplit(')',1)[1].split()
            if fields[0]!='Z' and int(fields[2])==group:found.append(int(path.parent.name))
        except (OSError,ValueError,IndexError):continue
    return found


def stop_owned(process):
    forced=[]
    if process.poll() is None:
        process.send_signal(signal.SIGINT)
        try:process.wait(timeout=12.)
        except subprocess.TimeoutExpired:pass
    for sig,delay in [(signal.SIGTERM,5.),(signal.SIGKILL,2.)]:
        if not members(process.pid):break
        forced.append(sig.name)
        try:os.killpg(process.pid,sig)
        except ProcessLookupError:break
        end=time.monotonic()+delay
        while members(process.pid) and time.monotonic()<end:time.sleep(.1)
    try:process.wait(timeout=1.)
    except subprocess.TimeoutExpired:pass
    return dict(returncode=process.returncode,forced_signals=forced,remaining=members(process.pid))


metadata=dict(started_epoch=time.time(),ros_domain_id=192,vehicle=1,actor_time_offset_s=227.,launch=command)
code=1;backend=None;validator=None;cleanup={}
try:
    with (out/'backend.log').open('w') as log:
        backend=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,env=env,
                                 start_new_session=True,preexec_fn=default_signals)
        metadata['backend_pid']=backend.pid
        (out/'environment.json').write_text(json.dumps(metadata,indent=2)+'\n')
        end=time.monotonic()+60.
        while not (out/'recording-ready').exists():
            if backend.poll() is not None:raise RuntimeError('backend exited before recording')
            if time.monotonic()>end:raise TimeoutError('recording was not ready')
            time.sleep(.1)
        metadata['validation_started_epoch']=time.time()
        (out/'environment.json').write_text(json.dumps(metadata,indent=2)+'\n')
        validation=root/'docs/build/evidence/registration-recovery-2026-11-27/run_live_validation.py'
        with (out/'validation.log').open('w') as result:
            validator=subprocess.Popen([sys.executable,str(validation),str(root)],stdout=result,
                stderr=subprocess.STDOUT,env=env,start_new_session=True,preexec_fn=default_signals)
            code=validator.wait(timeout=650.)
        (out/'validation-exit-code.txt').write_text(str(code)+'\n')
except BaseException as error:
    (out/'runner-error.txt').write_text(repr(error)+'\n')
    raise
finally:
    if validator is not None and validator.poll() is None:cleanup['validator']=stop_owned(validator)
    if backend is not None:cleanup['backend']=stop_owned(backend)
    metadata['finished_epoch']=time.time();metadata['validation_exit_code']=code
    (out/'environment.json').write_text(json.dumps(metadata,indent=2)+'\n')
    (out/'cleanup.json').write_text(json.dumps(cleanup,indent=2)+'\n')
    (out/'finished').write_text(str(code)+'\n')
    print(json.dumps(dict(validation_exit_code=code,cleanup=cleanup)),flush=True)
sys.exit(code if not cleanup.get('backend',{}).get('remaining') else 2)
