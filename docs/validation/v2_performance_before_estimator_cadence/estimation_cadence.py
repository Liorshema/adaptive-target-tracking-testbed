import runpy, sys
from robot_estimation.estimation_node import EstimationNode as C
original=C._joint_state_callback
def gated(self,msg):
    timestamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
    previous=getattr(self,'_benchmark_joint_time',None)
    if previous is not None and timestamp>=previous and timestamp-previous<.01-1e-9:
        return
    original(self,msg)
    if self._last_joint_time==timestamp:
        self._benchmark_joint_time=timestamp
C._joint_state_callback=gated
sys.argv=sys.argv[1:]
runpy.run_path(sys.argv[0],run_name='__main__')
