from typing import Dict, List

from simcore.streaming.base_streamer import BaseStreamer
from simcore.streaming.gstreamer_udp_streamer import GStreamerUDPStreamer

# Registry of available streamer backends
STREAMER_BACKENDS = {
    'gstreamer_udp': GStreamerUDPStreamer,
    # Add future backends here, e.g.:
    # 'ffmpeg_rtmp': FFmpegRTMPStreamer,
    # 'webrtc': WebRTCStreamer,
}


class StreamerManager:
    """Manages streaming pipelines for multiple cameras.
    
    Reads the 'streaming' section from the scene config and creates
    the appropriate streamer instances. Acts as the single interface
    that FrameDistributor talks to.

    Config example:
        streaming:
          enabled: True
          streams:
            - camera: workspace_overview
              backend: gstreamer_udp
              host: 127.0.0.1
              port: 5000
              bitrate: 2000
            - camera: eye_in_hand
              backend: gstreamer_udp
              host: 127.0.0.1
              port: 5001
    """

    def __init__(self, config: dict):
        self.config = config.get('streaming', {})
        self.enabled = self.config.get('enabled', False)
        
        # camera_name -> list of streamers (one camera could have multiple streams)
        self._streamers: Dict[str, List[BaseStreamer]] = {}

        if self.enabled:
            self._build_from_config()

    def _build_from_config(self):
        """Instantiate streamers based on config."""
        for stream_cfg in self.config.get('streams', []):
            camera_name = stream_cfg.get('camera')
            backend_name = stream_cfg.get('backend', 'gstreamer_udp')

            if backend_name not in STREAMER_BACKENDS:
                print(f"Warning: Unknown streaming backend '{backend_name}'. "
                      f"Available: {list(STREAMER_BACKENDS.keys())}")
                continue

            streamer_cls = STREAMER_BACKENDS[backend_name]
            streamer = streamer_cls(stream_cfg)

            if camera_name not in self._streamers:
                self._streamers[camera_name] = []
            self._streamers[camera_name].append(streamer)

            print(f"Registered streamer: {backend_name} for camera '{camera_name}'")

    def initialize_camera(self, camera_name: str, width: int, height: int, fps: int = 30):
        """Initialize all streamers for a given camera with its frame dimensions."""
        if camera_name not in self._streamers:
            return
        for streamer in self._streamers[camera_name]:
            streamer.initialize(width, height, fps)

    def send_frame(self, camera_name: str, frame):
        """Send a frame to all streamers registered for this camera."""
        if camera_name not in self._streamers:
            return
        for streamer in self._streamers[camera_name]:
            if streamer.is_initialized():
                streamer.send_frame(frame)

    def get_cameras(self) -> List[str]:
        """Return list of camera names that have streaming configured."""
        return list(self._streamers.keys())

    def stop(self):
        """Stop all streaming pipelines."""
        for camera_name, streamers in self._streamers.items():
            for streamer in streamers:
                streamer.stop()
        self._streamers.clear()
        print("StreamerManager: all streamers stopped")