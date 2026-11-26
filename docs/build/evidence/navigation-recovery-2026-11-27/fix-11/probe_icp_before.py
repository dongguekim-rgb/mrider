import numpy as np,math,inspect
import mrider_navigation_sim.sensor_core as core
from mrider_navigation_sim.geometry import ray_distances,obstacles
from mrider_navigation_sim.large_office import triangle
from mrider_navigation_sim.indoor import box
source=inspect.getsource(core.register_scan)
old="if rmse>.08 or np.linalg.norm(translation)>max_translation or abs(angle)>.2:raise ValueError('scan_registration_failed')"
new="""if rmse>.08 or np.linalg.norm(translation)>max_translation or abs(angle)>.2:
        print('rejected', {'rmse':rmse,'translation':translation.tolist(),'angle':angle,'points':len(current),'below_8cm':float(np.mean(distances<.08)),'quantiles':np.quantile(distances,[.5,.6,.7,.8,.9]).tolist()})
        raise ValueError('scan_registration_failed')"""
assert old in source
exec(source.replace(old,new),core.__dict__)
angles=np.linspace(-math.pi,math.pi,360,endpoint=False)
rng=np.random.default_rng(42);est=core.KeyframeOdometry()
for t in np.arange(109.,114.,.1):
 actors=[box(24.,triangle(t,32.,.5)),box(triangle(t+25.,52.,.6),16.)]
 ranges=ray_distances(24.77,7.846,angles+1.802,obstacles('large_office')+actors,15.)
 valid=ranges<14.95;ranges+=rng.normal(0.,.002,len(ranges))
 points=np.column_stack([ranges[valid]*np.cos(angles[valid]),ranges[valid]*np.sin(angles[valid])])
 try:pose,rmse=est.update(points)
 except ValueError as e: print('time',t)
