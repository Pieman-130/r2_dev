from pulsectl import Pulse

with Pulse() as pulse:
    # Get the default sink (output device)
    default_sink = pulse.get_sink_by_name(pulse.server_info().default_sink_name)

    # Get current volume (as a list of values, usually one for stereo)
    current_volume_percent = int(default_sink.volume.value_flat * 100)
    print(f"Current volume: {current_volume_percent}%")

    # Set volume to 60%
    pulse.volume_set_all_chans(default_sink, 1.0) # 0.0 to 1.0
    print("Volume set to 60%")

    # Mute the sink
    pulse.mute(default_sink, False)
    #print("Sink muted")
