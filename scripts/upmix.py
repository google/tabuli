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
#run("sox /tmp/input.wav /tmp/down.wav loudness -0 65 gain -40 treble 3 13000 treble 3 10000 treble 3 7000 bass 7 75 bass 12 50 bass 12 37 bass 12 20 rate 48k")
run("sox /tmp/input.wav -b 24 /tmp/pre-down.wav gain -10 rate 48k trim 0 30")
run("./build/emphasizer /tmp/pre-down.wav /tmp/down.wav")

run("sox /tmp/down.wav /tmp/down-dry.wav remix 1 2 gain 2 bass 6 300 treble 4 5500")
run("sox /tmp/down.wav /tmp/down-wet.wav remix 3 4 gain -4")
run("sox /tmp/down.wav /tmp/down-wetter.wav remix 5 6 gain -4")

#run("sox /tmp/down.wav /tmp/down-wet.wav remix 3 4 gain -40")
#run("sox /tmp/down.wav /tmp/down-wetter.wav remix 5 6 gain -40")

"""
run("sox /tmp/down.wav -b 32 /tmp/down_reverb1.wav reverb -w 80 55 170 70 75 gain 23 &\
 sox /tmp/down.wav -b 32 /tmp/down_reverb2.wav reverb -w 75 40 280 80 95 gain 23 &\
 sox /tmp/down.wav -b 32 /tmp/down_reverb3.wav reverb -w 85 40 180 60 140 gain 23 &\
 sox /tmp/down.wav -b 32 /tmp/down_reverb4.wav reverb -w 82 40 380 70 120 gain 23 &\
 wait")"""

#run("./build/two_to_three /tmp/down.wav /tmp/down3.wav")

run("./build/angular /tmp/down-dry.wav /tmp/down-dry-angular.wav")

# early reflections assume this bandpass filtering and reflection_volume
# attenuation
run("sox -v 0.85 /tmp/down-dry-angular.wav -b 24 /tmp/down-dry-angular-sinc.wav sinc 300-5500")
run("sox -v 0.7225 /tmp/down-dry-angular.wav -b 24 /tmp/down-dry-angular-sinc2.wav sinc 300-4500")

run("sox --combine merge /tmp/down-dry-angular.wav /tmp/down-dry-angular-sinc.wav  /tmp/down-dry-angular-sinc2.wav /tmp/down-dry-angular-mirrored.wav remix " + " ".join(["%d" % i for i in range(241,361)]) + " " + " ".join(["%d" % i for i in range(240,120,-1)]) + " " + " ".join(["%d" % i for i in range(1, 121)]) + " " + " ".join(["%d" % i for i in range(240,120,-1)]) + " " + " ".join(["%d" % i for i in range(241,361)]))


mixer = "0 " * 240 + "1 " * 50 + "1v0.5,2v0.5 " * 20 + "2 " * 50 + "0 " * 240
run("sox /tmp/down-wet.wav /tmp/down-wet-angular.wav remix " + mixer)

run("sox /tmp/down-wetter.wav /tmp/down-wetter-angular.wav remix " + mixer)

run("sox -m /tmp/down-wetter-angular.wav /tmp/down-wet-angular.wav /tmp/down-dry-angular-mirrored.wav /tmp/down36.wav")


run("./build/virtual_speakers --input_file /tmp/down36.wav --num_speakers=16 --output_file=/tmp/16speakers.wav --speaker_separation=0.1 --virtual_speaker_positions '" + ";".join(["%g,-9" % ((i - 300)*0.1 + 0.05) for i in range(600)]) + "'")

run("sox /tmp/16speakers.wav -e float -b 32 /tmp/down3-20.wav remix 0 0 1v3 2v3 3v1 4v1 5v1 6v1 7v1 8v1 0 0 9v1.3 10v1.3 11v1.3 12v1.3 13v1.3 14v1.3 15v3.9 16v3.9")

run("./build/driver_model /tmp/down3-20.wav /tmp/down3-mod.wav")

run("sox /tmp/down3-mod.wav -b 24 /tmp/down3-norm.wav norm -40");

run("amixer --card 2 cset numid=3,iface=MIXER,name='UMC1820 Output Playback Volume' 127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127")
run("amixer --card 2 cset numid=1,iface=MIXER,name='UMC1820 Output Playback Switch' on,on,on,on,on,on,on,on,on,on,on,on,on,on,on,on")

run("aplay -D 'hw:CARD=UMC1820,DEV=0' /tmp/down3-norm.wav");

import sys
sys.exit(0)
