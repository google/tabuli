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
run("sox /tmp/input.wav -b 24 /tmp/pre-down.wav gain -10 rate 48k trim 0 120")
run("./build/emphasizer /tmp/pre-down.wav /tmp/down.wav")

run("sox /tmp/down.wav /tmp/down-dry.wav remix 1 2 gain 2 bass 3 500 treble 3 5500")
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
run("sox /tmp/down-dry-angular.wav -b 24 /tmp/down-dry-angular-sinc.wav sinc 500-5500")

reflection_volume=0.85
run("sox --combine merge /tmp/down-dry-angular.wav /tmp/down-dry-angular-sinc.wav /tmp/down-dry-angular-mirrored.wav remix " + " ".join("%dv%g" % (i, reflection_volume) for i in range(240,120,-1)) + " " + " ".join("%d" % i for i in range(1, 121)) + " " + " ".join("%dv%g" % (i,reflection_volume) for i in range(240,120,-1)))


mixer = "0 " * 120 + "1 " * 50 + "1v0.5,2v0.5 " * 20 + "2 " * 50 + "0 " * 120
run("sox /tmp/down-wet.wav /tmp/down-wet-angular.wav remix " + mixer)

run("sox /tmp/down-wetter.wav /tmp/down-wetter-angular.wav remix " + mixer)

run("sox -m /tmp/down-wetter-angular.wav /tmp/down-wet-angular.wav /tmp/down-dry-angular-mirrored.wav /tmp/down36.wav")

# ";".join(["%0.2f,-0.6" % (i * 0.1 - 1.75) for i in range(36)])
# >>> ";".join(["%0.2f,-6" % (i * 1 - 17.5) for i in range(36)])


run("./build/virtual_speakers --input_file /tmp/down36.wav --num_speakers=16 --output_file=/tmp/12speakers.wav --speaker_separation=1.0 --virtual_speaker_positions '-17.95,-12;-17.85,-12;-17.75,-12;-17.65,-12;-17.55,-12;-17.45,-12;-17.35,-12;-17.25,-12;-17.15,-12;-17.05,-12;-16.95,-12;-16.85,-12;-16.75,-12;-16.65,-12;-16.55,-12;-16.45,-12;-16.35,-12;-16.25,-12;-16.15,-12;-16.05,-12;-15.95,-12;-15.85,-12;-15.75,-12;-15.65,-12;-15.55,-12;-15.45,-12;-15.35,-12;-15.25,-12;-15.15,-12;-15.05,-12;-14.95,-12;-14.85,-12;-14.75,-12;-14.65,-12;-14.55,-12;-14.45,-12;-14.35,-12;-14.25,-12;-14.15,-12;-14.05,-12;-13.95,-12;-13.85,-12;-13.75,-12;-13.65,-12;-13.55,-12;-13.45,-12;-13.35,-12;-13.25,-12;-13.15,-12;-13.05,-12;-12.95,-12;-12.85,-12;-12.75,-12;-12.65,-12;-12.55,-12;-12.45,-12;-12.35,-12;-12.25,-12;-12.15,-12;-12.05,-12;-11.95,-12;-11.85,-12;-11.75,-12;-11.65,-12;-11.55,-12;-11.45,-12;-11.35,-12;-11.25,-12;-11.15,-12;-11.05,-12;-10.95,-12;-10.85,-12;-10.75,-12;-10.65,-12;-10.55,-12;-10.45,-12;-10.35,-12;-10.25,-12;-10.15,-12;-10.05,-12;-9.95,-12;-9.85,-12;-9.75,-12;-9.65,-12;-9.55,-12;-9.45,-12;-9.35,-12;-9.25,-12;-9.15,-12;-9.05,-12;-8.95,-12;-8.85,-12;-8.75,-12;-8.65,-12;-8.55,-12;-8.45,-12;-8.35,-12;-8.25,-12;-8.15,-12;-8.05,-12;-7.95,-12;-7.85,-12;-7.75,-12;-7.65,-12;-7.55,-12;-7.45,-12;-7.35,-12;-7.25,-12;-7.15,-12;-7.05,-12;-6.95,-12;-6.85,-12;-6.75,-12;-6.65,-12;-6.55,-12;-6.45,-12;-6.35,-12;-6.25,-12;-6.15,-12;-6.05,-12;-5.95,-12;-5.85,-12;-5.75,-12;-5.65,-12;-5.55,-12;-5.45,-12;-5.35,-12;-5.25,-12;-5.15,-12;-5.05,-12;-4.95,-12;-4.85,-12;-4.75,-12;-4.65,-12;-4.55,-12;-4.45,-12;-4.35,-12;-4.25,-12;-4.15,-12;-4.05,-12;-3.95,-12;-3.85,-12;-3.75,-12;-3.65,-12;-3.55,-12;-3.45,-12;-3.35,-12;-3.25,-12;-3.15,-12;-3.05,-12;-2.95,-12;-2.85,-12;-2.75,-12;-2.65,-12;-2.55,-12;-2.45,-12;-2.35,-12;-2.25,-12;-2.15,-12;-2.05,-12;-1.95,-12;-1.85,-12;-1.75,-12;-1.65,-12;-1.55,-12;-1.45,-12;-1.35,-12;-1.25,-12;-1.15,-12;-1.05,-12;-0.95,-12;-0.85,-12;-0.75,-12;-0.65,-12;-0.55,-12;-0.45,-12;-0.35,-12;-0.25,-12;-0.15,-12;-0.05,-12;0.05,-12;0.15,-12;0.25,-12;0.35,-12;0.45,-12;0.55,-12;0.65,-12;0.75,-12;0.85,-12;0.95,-12;1.05,-12;1.15,-12;1.25,-12;1.35,-12;1.45,-12;1.55,-12;1.65,-12;1.75,-12;1.85,-12;1.95,-12;2.05,-12;2.15,-12;2.25,-12;2.35,-12;2.45,-12;2.55,-12;2.65,-12;2.75,-12;2.85,-12;2.95,-12;3.05,-12;3.15,-12;3.25,-12;3.35,-12;3.45,-12;3.55,-12;3.65,-12;3.75,-12;3.85,-12;3.95,-12;4.05,-12;4.15,-12;4.25,-12;4.35,-12;4.45,-12;4.55,-12;4.65,-12;4.75,-12;4.85,-12;4.95,-12;5.05,-12;5.15,-12;5.25,-12;5.35,-12;5.45,-12;5.55,-12;5.65,-12;5.75,-12;5.85,-12;5.95,-12;6.05,-12;6.15,-12;6.25,-12;6.35,-12;6.45,-12;6.55,-12;6.65,-12;6.75,-12;6.85,-12;6.95,-12;7.05,-12;7.15,-12;7.25,-12;7.35,-12;7.45,-12;7.55,-12;7.65,-12;7.75,-12;7.85,-12;7.95,-12;8.05,-12;8.15,-12;8.25,-12;8.35,-12;8.45,-12;8.55,-12;8.65,-12;8.75,-12;8.85,-12;8.95,-12;9.05,-12;9.15,-12;9.25,-12;9.35,-12;9.45,-12;9.55,-12;9.65,-12;9.75,-12;9.85,-12;9.95,-12;10.05,-12;10.15,-12;10.25,-12;10.35,-12;10.45,-12;10.55,-12;10.65,-12;10.75,-12;10.85,-12;10.95,-12;11.05,-12;11.15,-12;11.25,-12;11.35,-12;11.45,-12;11.55,-12;11.65,-12;11.75,-12;11.85,-12;11.95,-12;12.05,-12;12.15,-12;12.25,-12;12.35,-12;12.45,-12;12.55,-12;12.65,-12;12.75,-12;12.85,-12;12.95,-12;13.05,-12;13.15,-12;13.25,-12;13.35,-12;13.45,-12;13.55,-12;13.65,-12;13.75,-12;13.85,-12;13.95,-12;14.05,-12;14.15,-12;14.25,-12;14.35,-12;14.45,-12;14.55,-12;14.65,-12;14.75,-12;14.85,-12;14.95,-12;15.05,-12;15.15,-12;15.25,-12;15.35,-12;15.45,-12;15.55,-12;15.65,-12;15.75,-12;15.85,-12;15.95,-12;16.05,-12;16.15,-12;16.25,-12;16.35,-12;16.45,-12;16.55,-12;16.65,-12;16.75,-12;16.85,-12;16.95,-12;17.05,-12;17.15,-12;17.25,-12;17.35,-12;17.45,-12;17.55,-12;17.65,-12;17.75,-12;17.85,-12;17.95,-12'")

#run("sox /tmp/12speakers.wav -e float -b 32 /tmp/down3-20.wav remix 0 0 0 0 1v1 2v1 3v1 4v1 5v1 6v1 0 0 7v1.1 8v1.1 9v1.1 10v1.1 11v1.1 12v1.1 0 0")
run("sox /tmp/12speakers.wav -e float -b 32 /tmp/down3-20.wav remix 0 0 1v3 2v3 3v1 4v1 5v1 6v1 7v1 8v1 0 0 9v1.3 10v1.3 11v1.3 12v1.3 13v1.3 14v1.3 15v3.9 16v3.9")

run("./build/driver_model /tmp/down3-20.wav /tmp/down3-mod.wav")

run("sox /tmp/down3-mod.wav -b 24 /tmp/down3-norm.wav norm -28");

run("amixer --card 2 cset numid=3,iface=MIXER,name='UMC1820 Output Playback Volume' 127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127")
run("amixer --card 2 cset numid=1,iface=MIXER,name='UMC1820 Output Playback Switch' on,on,on,on,on,on,on,on,on,on,on,on,on,on,on,on")

run("aplay -D 'hw:CARD=UMC1820,DEV=0' /tmp/down3-norm.wav");

import sys
sys.exit(0)
