"""Capture actual owned RViz/Gazebo X11 windows, never mockups or desktop contents."""
import argparse
import json
from pathlib import Path
import re
import subprocess
from PIL import Image

from PyQt5.QtWidgets import QApplication

parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
args=parser.parse_args();app=QApplication([])
tree=subprocess.check_output(['xwininfo','-root','-tree'],text=True)
found=[]
for line in tree.splitlines():
    if not re.search(r'RViz|Gazebo|rviz',line):continue
    match=re.search(r'(0x[0-9a-fA-F]+)\s+"([^"]+)"',line)
    if not match:continue
    window=int(match.group(1),16);label='rviz' if 'rviz' in line.lower() else 'gazebo'
    pixmap=app.primaryScreen().grabWindow(window)
    if pixmap.isNull():continue
    path=args.output/(label+'_actual_'+match.group(1)+'.png')
    if pixmap.save(str(path)):
        extrema=Image.open(path).convert('RGB').getextrema()
        usable=pixmap.width()>=100 and pixmap.height()>=100 and any(high>low for low,high in extrema)
        found.append({'title':match.group(2),'file':path.name,'width':pixmap.width(),'height':pixmap.height(),'nonblank_capture':usable})
print(json.dumps({'captures':found}));app.quit()
