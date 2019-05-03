# MIDI-Theremin-with-TIVA-C
Digital interface with the Theremin (musical instrument) with MIDI output 

Created in the 80’s, the MIDI (Musical Instrument Digital Interface) technology still remains a reference in the sector of the music 
industry. This technology describes a digital interface protocol and has innovated the way of producing music and is currently 
present in most recording studios and stages around the world. This monograph describes the development of an interface capable of 
converting the audio signal generated by a Theremin (an analogue electronic musical instrument) to MIDI. The most relevant output 
information from the Theremin is the frequency of the musical note generated, which is monitored and processed by a microcontroller
(Tiva C – Texas Instruments), converted in MIDI data and sent to a virtual instrument installed in a computer (VSTi). 
The interface provides a visual feedback to the musician through a RGB LED display, which returns the musical note played in colours, 
facilitating the musical performance of Theremin. There is also a manual controller which allows control to the MIDI parameter velocity 
through a force sensitive resistor and a button capable of controlling the moment when a musical note is played, in addition to the 
traditional Theremin behavior. The interface was designed to be used in conjunction with virtual instruments software and audio interfaces
that have MIDI communication. Thus the musician can choose several sound parameters, expanding the musical possibilities of a Theremin.
The project resulted in a functional prototype, in accordance with the proposed objectives and compatible with a low-cost Theremin.