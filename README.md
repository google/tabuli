Tabuli is a project for [Wave field synthesis](https://en.wikipedia.org/wiki/Wave_field_synthesis).

This project is a study about upmixing 2-channel stereo input audio streams into
multi-channel (e.g. 16 channel) wave field speaker use.

We produce wave the field synthesis audio using an multi-stage process. The
states are reverbration separation, dry sound spatial location, rereverbration,
rendering, and speaker reverse physics modeling. The different stages produce
wav-files that are compatible with sox as an auxilliary processing tool that can
be applied for input, output or intermediate results of the stages. 

The first step makes an attempt to separate the input stereo audio into three
stereo audio streams that together sum back into the input stereo. The first of
them attempts to capture the 'dry' audio of the recording, without reverbration
and with minimal in-instrument resonances. The second stream will contain some
early room reverbration and most of the in-instrument resonances. 
The third stream will contain the sound that has longer reverb within the room
or concert hall, i.e., late reverb. The separation happens in a module called
'emphasizer'.

In the dry sound spatial location, we use a process that reverses the predicted
amplitudes of a spatialized source and tries to finds an optimal single source
for each frequency band to explain a microphone sensitivity pattern that we
define. In the current practical application we use this process to upmix the
audio from two tracks to twelwe tracks, but we could use a larger or smaller
number of tracks in upmixing. We call this model 'angular' as it relates to
computing the angle of the sound source.

In the rereverbration we attempt to produce a multi-dimensional model of the
reverbration based on the position of the dry sounds, while maintaining the
volume of the reverbration within the two measured stages of reverbration -- the
early and late reverbration.

In the rendering phase of the computation, a 3d-geometric virtual placement of
sound sources (often a linear array) is rendered to actual speakers, such as a
wave field speaker. Often it is practical to keep the virtual speakers a bit
further away from the listener than the actual linear array, as that will allow
the speakers to collaborate rather than trying to reproduce complex
interferences.

The last phase of computation, that we call 'driver model', will apply the
inverse physics of speaker drivers. This may allow us to reduce the impact of
various non-linearities and oscillations within the speaker driver and
contribute to the overall experience positively.

Contact Jyrki Alakuijala (jyrki@google.com) for more information about this
project.
