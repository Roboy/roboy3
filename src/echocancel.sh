#!/bin/bash

#pacmd set-default-sink 0
pacmd set-default-source 1
pacmd load-module module-combine-sink sink_name=combined sink_properties=device.description=CombinedSink slaves=alsa_output.usb-0b0e_Jabra_SPEAK_510_USB_1C48F9E9DBE1020A00-00.analog-stereo,alsa_output.usb-0d8c_Generic_USB_Audio_Device-00.analog-stereo

pacmd set-default-sink combined
#pactl unload-module module-echo-cancel
##pactl load-module module-echo-cancel source_master=alsa_input.usb-Solid_State_System_Co._Ltd._iTalk-02_000000000000-00.analog-mono sink_master=alsa_output.usb-0d8c_Generic_USB_Audio_Device-00.analog-stereo aec_method=webrtc source_name=echocancel sink_name=echocancel1
#pactl load-module module-echo-cancel source_master=alsa_input.usb-0b0e_Jabra_SPEAK_510_USB_1C48F9E9DBE1020A00-00.analog-mono sink_master=alsa_output.usb-0b0e_Jabra_SPEAK_510_USB_1C48F9E9DBE1020A00-00.analog-stereo aec_method=webrtc source_name=echocancel sink_name=echocancel1
#pacmd set-default-source echocancel
#pacmd set-default-sink echocancel1
#pactl set-sink-volume @DEFAULT_SINK@ 100%
#pactl set-source-volume @DEFAULT_SOURCE@ 120%
