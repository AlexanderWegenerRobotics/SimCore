try:
    from simcore.streaming.gstreamer_udp_streamer import GStreamerUDPStreamer
except ImportError:
    GStreamerUDPStreamer = None