# Copyright 2024 Google LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# This script takes in a stereo input (ideally of a live concert with
# 90 degree microphone setup) and computes a wave field representation
# from it.

import os
import sys

def run(str):
    print("Executing '%s'" % str)
    assert 0 == os.system(str)
    print("Completed '%s'\n" % str)

# The commands here are needed for installing the right environment for linux.
# run("sudo apt-get install libsndfile1-dev libfftw3-dev libeigen3-dev")
# run("git submodule update --init")

run("(cd build; cmake -DCMAKE_BUILD_TYPE=release ..)")
run("(cd build; make -j 55)")

file = sys.argv[1]
print("playing", file)
run('cp "' + file + '" /tmp/input.wav')

run("sox /tmp/input.wav -b 24 /tmp/down-dry.wav gain -10 rate 48k trim 0 30")

run("./build/revolve /tmp/down-dry.wav /tmp/down-dry-angular.wav")

run("./build/virtual_speakers --input_file /tmp/down-dry-angular.wav --num_speakers=16 --output_file=/tmp/16speakers.wav --speaker_separation=0.1 --virtual_speaker_positions '" + ";".join(["%g,-9" % ((i - 60)*0.1 + 0.05) for i in range(120)]) + "'")

run("sox /tmp/16speakers.wav -e float -b 32 /tmp/down3-20.wav remix 0 0 1v1 2v1 3v1 4v1 5v1 6v1 7v1 8v1 0 0 9v1.3 10v1.3 11v1.3 12v1.3 13v1.3 14v1.3 15v1.3 16v1.3")

run("./build/driver_model /tmp/down3-20.wav /tmp/down3-mod.wav")

run("sox /tmp/down3-mod.wav -b 24 /tmp/down3-norm.wav treble -2 10000 norm -32");

run("amixer --card 3 cset numid=3,iface=MIXER,name='UMC1820 Output Playback Volume' 127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127")
run("amixer --card 3 cset numid=1,iface=MIXER,name='UMC1820 Output Playback Switch' on,on,on,on,on,on,on,on,on,on,on,on,on,on,on,on")

run("aplay -D 'hw:CARD=UMC1820,DEV=0' /tmp/down3-norm.wav");

import sys
sys.exit(0)
