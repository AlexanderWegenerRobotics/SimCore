import numpy as np
import subprocess
import signal
from typing import Optional
import shutil
import time

from simcore.streaming.base_streamer import BaseStreamer


class GStreamerUDPStreamer(BaseStreamer):
    """Low-latency video streamer using GStreamer with RTP over UDP.
    
    Launches a GStreamer pipeline via subprocess that reads raw video from stdin,
    encodes it, and streams via RTP/UDP. This avoids the need for Python GStreamer
    bindings (gi) and keeps things simple and portable.

    Default pipeline:
        rawvideo -> x264enc (zerolatency) -> rtph264pay -> udpsink

    Receive side example (on the consumer machine):
        gst-launch-1.0 udpsrc port=5000 caps="application/x-rtp,encoding-name=H264,payload=96" \
            ! rtph264depay ! avdec_h264 ! videoconvert ! autovideosink sync=false
    """

    def __init__(self, config: dict):
        super().__init__(config)
        self._process: Optional[subprocess.Popen] = None
        
        # Streaming parameters from config
        self.host = config.get('host', '127.0.0.1')
        self.port = config.get('port', 5000)
        self.bitrate = config.get('bitrate', 2000)  # kbit/s
        self.tune = config.get('tune', 'zerolatency')
        self.speed_preset = config.get('speed_preset', 'ultrafast')

    def initialize(self, width: int, height: int, fps: int = 30):
        """Launch the GStreamer pipeline subprocess."""
        self._width = width
        self._height = height
        self._fps = fps

        pipeline = (
            f"fdsrc fd=0 "
            f"! rawvideoparse width={width} height={height} format=bgr framerate={fps}/1 "
            f"! videoconvert "
            f"! x264enc tune={self.tune} speed-preset={self.speed_preset} "
            f"  bitrate={self.bitrate} key-int-max={fps} "
            f"! rtph264pay config-interval=1 pt=96 "
            f"! udpsink host={self.host} port={self.port} sync=false"
        )

        try:
            gst_binary = shutil.which('gst-launch-1.0') or shutil.which('gst-launch-1.0.exe')
            if not gst_binary:
                print("Error: gst-launch-1.0 not found. Install GStreamer to use UDP streaming.")
                self._initialized = False
                return
            self._process = subprocess.Popen(
                [gst_binary] + pipeline.split(),
                stdin=subprocess.PIPE,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE,
            )
            self._initialized = True
            print(f"GStreamer UDP streamer started -> {self.host}:{self.port} "
                  f"({width}x{height} @ {fps}fps, {self.bitrate}kbps)")
        except FileNotFoundError:
            print("Error: gst-launch-1.0 not found. Install GStreamer to use UDP streaming.")
            self._initialized = False
        except Exception as e:
            print(f"Error starting GStreamer pipeline: {e}")
            self._initialized = False

    def send_frame(self, frame: np.ndarray):
        """Write raw frame bytes to the GStreamer pipeline's stdin."""
        if not self._initialized or self._process is None:
            return

        if self._process.poll() is not None:
            print("Warning: GStreamer pipeline has exited unexpectedly")
            self._initialized = False
            return

        try:
            self._process.stdin.write(frame.tobytes())
            self._process.stdin.flush()
        except BrokenPipeError:
            print("Warning: GStreamer pipeline broken pipe - stopping streamer")
            self._initialized = False

    def stop(self):
        """Terminate the GStreamer pipeline."""
        if self._process is not None:
            try:
                self._process.stdin.close()
                self._process.send_signal(signal.SIGINT)
                self._process.wait(timeout=3)
            except Exception:
                self._process.kill()
            finally:
                self._process = None
                self._initialized = False
                print("GStreamer UDP streamer stopped")