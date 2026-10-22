import atexit, collections, json, runpy, sys
from pathlib import Path
from robot_simulation.actuation.velocity_actuation import VelocityActuationNode as C
counts=collections.Counter()
command=C._command_callback
halt=C._halt
def observe_command(self,msg):
    counts['incoming']+=1
    command(self,msg)
    counts['accepted' if self._command is not None else ('inactive' if not msg.active else 'rejected')]+=1
def observe_halt(self,reason):
    counts['halt:'+reason]+=1
    halt(self,reason)
C._command_callback=observe_command
C._halt=observe_halt
atexit.register(lambda:Path(__file__).with_name('actuation_counts.json').write_text(json.dumps(dict(counts),indent=2)))
sys.argv=sys.argv[1:]
runpy.run_path(sys.argv[0],run_name='__main__')
