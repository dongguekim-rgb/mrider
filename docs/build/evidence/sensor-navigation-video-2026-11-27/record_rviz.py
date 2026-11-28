"""Capture the actual existing RViz X11 window, with a bounded CPU encoder.

No simulator geometry, GT, or reconstructed animation is used. Fixed-rate
capture duplicates the last captured frame across scheduling delays; the
elapsed duration and duplicate count are recorded for verification.
"""
import argparse
import json
from pathlib import Path
import signal
import subprocess
import time
from Xlib import display,X,protocol

p=argparse.ArgumentParser();p.add_argument('--display',default=':13.0');p.add_argument('--window',default='0x04c00013')
p.add_argument('--output',required=True);p.add_argument('--fps',type=int,default=8)
a=p.parse_args();out=Path(a.output);out.mkdir(parents=True,exist_ok=True)
c=display.Display(a.display);w=c.create_resource_object('window',int(a.window,0));root=c.screen().root
geometry=w.get_geometry();width,height=geometry.width,geometry.height
assert width%2==0 and height%2==0
pid=int(w.get_full_property(c.intern_atom('_NET_WM_PID'),X.AnyPropertyType).value[0])
above=c.intern_atom('_NET_WM_STATE_ABOVE');state=c.intern_atom('_NET_WM_STATE')
previous=w.get_full_property(state,X.AnyPropertyType)
restore_above=previous is None or above not in previous.value
def keep_above(enabled):
    event=protocol.event.ClientMessage(window=w,client_type=state,data=(32,[int(enabled),above,0,1,0]))
    root.send_event(event,event_mask=X.SubstructureRedirectMask|X.SubstructureNotifyMask);c.flush()
if restore_above:keep_above(True)
msg=protocol.event.ClientMessage(window=w,client_type=c.intern_atom('_NET_ACTIVE_WINDOW'),data=(32,[1,X.CurrentTime,0,0,0]))
root.send_event(msg,event_mask=X.SubstructureRedirectMask|X.SubstructureNotifyMask);c.flush();time.sleep(1.)
command=['ffmpeg','-hide_banner','-loglevel','warning','-nostats','-filter_threads','1','-f','rawvideo','-pixel_format','bgr0',
         '-video_size',f'{width}x{height}','-framerate',str(a.fps),'-i','pipe:0','-an',
         '-c:v','libx264','-preset','ultrafast','-crf','23','-threads','1','-pix_fmt','yuv420p',
         '-movflags','+faststart',str(out/'rviz-navigation.mp4')]
stop=False

def request_stop(*unused):
    global stop
    stop=True

signal.signal(signal.SIGINT,request_stop);signal.signal(signal.SIGTERM,request_stop)
frames=0;duplicates=0;last=None;started=time.monotonic();epoch=time.time();failure=None
with (out/'ffmpeg.log').open('w') as log:
    encoder=subprocess.Popen(command,stdin=subprocess.PIPE,stdout=log,stderr=subprocess.STDOUT)
    try:
        while not stop and not (out/'stop-recording').exists():
            elapsed=time.monotonic()-started
            if elapsed>660.:raise TimeoutError('recording exceeded 660 seconds')
            target=int(elapsed*a.fps)
            if last is not None:
                while frames<target:
                    encoder.stdin.write(last);frames+=1;duplicates+=1
            raw=w.get_image(0,0,width,height,X.ZPixmap,0xffffffff)
            last=raw.data
            if len(last)!=width*height*4:raise RuntimeError('unexpected X11 pixel format')
            encoder.stdin.write(last);frames+=1
            if frames==1:(out/'recording-started').write_text(str(epoch)+'\n')
            remaining=started+frames/a.fps-time.monotonic()
            if remaining>0:time.sleep(min(remaining,1./a.fps))
    except BaseException as error:
        failure=repr(error)
        raise
    finally:
        ended=time.monotonic();encoder.stdin.close();status=encoder.wait(timeout=30.)
        info=dict(started_epoch=epoch,ended_epoch=time.time(),wall_duration_s=ended-started,
                  fps=a.fps,frames=frames,duplicated_frames=duplicates,width=width,height=height,
                  rviz_pid=pid,window=a.window,display=a.display,encoder_exit_code=status,error=failure,
                  content='Actual RViz window capture, real time; synthetic sensor simulation, not Gazebo')
        (out/'capture.json').write_text(json.dumps(info,indent=2)+'\n');print(json.dumps(info),flush=True)
        if restore_above:keep_above(False)
        c.close()
if status:raise SystemExit(status)
