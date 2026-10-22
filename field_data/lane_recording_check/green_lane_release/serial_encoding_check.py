"""Compile the actual driver encoder with a memory serial sink, no hardware."""
from __future__ import print_function
import json
import os
import shutil
import subprocess
import tempfile

root='/home/nano/robocup_ws'
with open(root+'/src/drivers/base/src/base_controller.cpp') as stream:
    source=stream.read()
encoder=(source[source.index('unsigned char data_header='):source.index('void callback(')]+
         source[source.index('int writeSpeed('):source.index('void publishStatus(')])
prefix='''#include <cstdio>
struct MemorySerial {
    int write(const unsigned char* bytes, int count) {
        for (int i=0;i<count;++i) printf("%s%u", i ? " " : "", bytes[i]);
        printf("\\n"); return count;
    }
} ser;
'''
cases=[(12,-3),(12,-8),(12,-15),(12,-22),(0,22),(0,0)]
main='int main() {\n'+''.join('writeSpeed(%d,%d);\n'%case for case in cases)+'}\n'
folder=tempfile.mkdtemp(prefix='steering_encoder_')
try:
    filename=folder+'/check.cpp'
    with open(filename,'w') as stream:stream.write(prefix+encoder+main)
    subprocess.check_call(['g++',filename,'-o',folder+'/check'])
    lines=subprocess.check_output([folder+'/check']).splitlines()
    rows=[]
    for case,line in zip(cases,lines):
        speed,steer=case
        packet=list(map(int,line.split()))
        assert len(packet)==11 and packet[0]==165 and packet[-1]==90
        assert packet[4]==(2 if speed>0 else 0) and packet[5]==abs(speed)
        assert packet[6]==(1 if steer>0 else 2 if steer<0 else 0)
        assert packet[7]==abs(steer)
        rows.append(dict(speed=speed,steer=steer,packet=packet))
    with open(root+'/field_data/lane_recording_check/green_lane_release/serial_packets.json','w') as stream:
        json.dump(dict(scope='Actual C++ writeSpeed encoder with memory sink; not STM32 acceptance or measured wheel angle.',rows=rows),stream,indent=2)
    print('PASS: %d actual encoder cases; nonzero steering bytes retained'%len(rows))
finally:
    shutil.rmtree(folder)
