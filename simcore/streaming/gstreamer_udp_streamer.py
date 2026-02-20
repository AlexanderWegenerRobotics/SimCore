import numpy as np
import time
from typing import Optional

import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst

from simcore.streaming.base_streamer import BaseStreamer

Gst.init(None)


class GStreamerUDPStreamer(BaseStreamer):

    def __init__(self, config: dict):
        super().__init__(config)
        self._pipeline = None
        self._appsrc = None
        self._frame_count = 0
        self._frame_duration = 0  # nanoseconds, set in initialize()

        self.host = config.get('host', '127.0.0.1')
        self.port = config.get('port', 5000)
        self.bitrate = config.get('bitrate', 2000)
        self.tune = config.get('tune', 'zerolatency')
        self.speed_preset = config.get('speed_preset', 'ultrafast')

    def initialize(self, width: int, height: int, fps: int = 30):
        """Build and start the GStreamer pipeline."""
        self._width = width
        self._height = height
        self._fps = fps
        self._frame_count = 0
        self._frame_duration = Gst.SECOND // fps  # nanoseconds per frame

        pipeline_str = (
            f'appsrc name=src is-live=true format=time '
            f'! video/x-raw,format=BGR,width={width},height={height},framerate={fps}/1 '
            f'! videoconvert '
            f'! x264enc tune={self.tune} speed-preset={self.speed_preset} '
            f'  bitrate={self.bitrate} key-int-max={fps} '
            f'! rtph264pay config-interval=1 pt=96 '
            f'! udpsink host={self.host} port={self.port} sync=false'
        )

        try:
            self._pipeline = Gst.parse_launch(pipeline_str)
            self._appsrc = self._pipeline.get_by_name('src')

            self._pipeline.set_state(Gst.State.PLAYING)

            # Push a dummy frame immediately to unblock the ASYNC state transition
            dummy = np.zeros((height, width, 3), dtype=np.uint8)
            buf = Gst.Buffer.new_wrapped(dummy.tobytes())
            buf.pts = 0
            buf.dts = 0
            buf.duration = self._frame_duration
            self._appsrc.emit('push-buffer', buf)
            self._frame_count = 1

            # Now wait for pipeline to reach PLAYING
            ret = self._pipeline.get_state(5 * Gst.SECOND)
            if ret[1] not in (Gst.State.PLAYING, Gst.State.PAUSED):
                print(f"GStreamer pipeline failed to reach PLAYING state: {ret}")
                self._initialized = False
                return

            time.sleep(0.2)

            self._initialized = True
            print(f"GStreamer UDP streamer started -> {self.host}:{self.port} "
                  f"({width}x{height} @ {fps}fps, {self.bitrate}kbps)")

        except Exception as e:
            print(f"Error starting GStreamer pipeline: {e}")
            self._initialized = False

    def send_frame(self, frame: np.ndarray):
        """Push a timestamped BGR frame into the GStreamer pipeline."""
        if not self._initialized or self._appsrc is None:
            return

        try:
            buf = Gst.Buffer.new_wrapped(frame.tobytes())
            buf.pts = self._frame_count * self._frame_duration
            buf.dts = self._frame_count * self._frame_duration
            buf.duration = self._frame_duration
            self._frame_count += 1

            ret = self._appsrc.emit('push-buffer', buf)
            if ret != Gst.FlowReturn.OK:
                print(f"Warning: push-buffer returned {ret}")
                self._initialized = False

        except Exception as e:
            print(f"GStreamer send_frame error: {e}")
            self._initialized = False

    def stop(self):
        """Tear down the GStreamer pipeline."""
        if self._pipeline is not None:
            try:
                self._appsrc.emit('end-of-stream')
                time.sleep(0.1)
                self._pipeline.set_state(Gst.State.NULL)
            except Exception:
                pass
            finally:
                self._pipeline = None
                self._appsrc = None
                self._frame_count = 0
                self._initialized = False
                print("GStreamer UDP streamer stopped")