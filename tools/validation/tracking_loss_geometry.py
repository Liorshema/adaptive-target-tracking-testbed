"""Read-only diagnostic geometry and causal source-time contracts."""
import numpy as np
from scipy.spatial.transform import Rotation

class History:
    def __init__(self, rows):
        self.rows=[r for r in rows if r['source_time'] is not None]
        self.times=np.array([r['source_time'] for r in self.rows])
        if np.any(np.diff(self.times)<0):raise ValueError('split clock reset epochs before analysis')
    def at(self,t,max_age):
        i=np.searchsorted(self.times,t,side='right')-1
        if i<0 or t-self.times[i]>max_age+1e-9:return None
        return self.rows[i]

def pose(position, quaternion):
    out=np.eye(4);out[:3,:3]=Rotation.from_quat(quaternion).as_matrix();out[:3,3]=position
    return out

def planar(x,y,yaw):
    return pose([x,y,0],Rotation.from_euler('z',yaw).as_quat())

def wrap(angle):return np.arctan2(np.sin(angle),np.cos(angle))

def task(camera,target,axis,radius):
    delta=np.asarray(target)-camera[:3,3];distance=np.linalg.norm(delta)
    if distance<=1e-9:raise ValueError('undefined optical direction at target origin')
    forward=camera[:3,:3]@np.asarray(axis);forward/=np.linalg.norm(forward)
    return distance-radius,float(np.arccos(np.clip(forward@delta/distance,-1,1)))

def oracle_mean(mean,physical,spawn,linear,angular):
    result=np.asarray(mean,dtype=float).copy();local=np.linalg.inv(spawn)@physical
    result[:5]=[local[0,3],local[1,3],np.arctan2(local[1,0],local[0,0]),linear,angular]
    return result

def interval_fraction(times,values,start,end,max_gap):
    observed=accepted=0.
    for i in range(len(times)-1):
        a,b=times[i:i+2];dt=b-a
        if a>=start and b<=end and 0<dt<=max_gap:
            observed+=dt;accepted+=dt*bool(values[i])
    return {'observed_s':observed,'compliant_s':accepted,
            'valid_time_percent':100*accepted/observed if observed else None,
            'coverage_percent':100*observed/(end-start)}
