# CPX Glove Synth

This is a synthesizer for the Circuit Playground Express, designed to be played using a specially-constructed glove. Features:

* Paraphonic subtractive synthesizer
* Oscillator and amp controlled by finger position
* Filter controlled by hand tilt
* Antialiased saw oscillators
* Delay and reverb modules - 2+ seconds of total delay time are possible
* DMA audio playback and neopixel control
* Digital pad input
* Smoothed accelerometer input
* Various ugly fixed-point hacks

To construct the glove, you'll want to get some conductive thread and sew pads onto the fingers and the tip of the thumb. Connect the thumb pad to ground, and the finger pads to A1-A7. Then you can touch your thumb to the finger pads to play notes! Watch out for shorts. 

The code's a mess and contains a bunch of debugging / profiling gunk that I haven't cleaned out -- but I haven't worked on the project in months, so I'm declaring it done for now.
