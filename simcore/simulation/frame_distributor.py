import cv2
import numpy as np
import time
import threading
from typing import Dict, List, Optional, TYPE_CHECKING

if TYPE_CHECKING:
    from simcore.simulation.sim_model import SimulationModel
    from simcore.streaming.streamer_manager import StreamerManager
    from simcore.common.video_logger import VideoLogger


class FrameDistributor:
    """Pulls frames from simulation renderers and distributes to all consumers.
    
    This decouples frame production (rendering) from consumption (display,
    video logging, streaming). All consumers receive the same frames so
    we only render once per camera per update cycle.

    Consumers:
        - Display (CV2 windows)          -> optional
        - VideoLogger (mp4 recording)    -> optional  
        - StreamerManager (RTP/UDP etc)  -> optional

    Usage:
        distributor = FrameDistributor(sim, config)
        distributor.set_display_enabled(True)
        distributor.set_video_logger(video_logger)
        distributor.set_streamer_manager(streamer_manager)
        distributor.run()  # blocking, call on main thread (CV2 needs main thread)
    """

    def __init__(self, sim: 'SimulationModel', config: dict):
        self.sim = sim
        self.config = config
        self.running = False

        self._fps = config.get('render_fps', 30)
        self._frame_interval = 1.0 / self._fps

        # Consumers - all optional
        self._display_enabled = False
        self._video_logger: Optional['VideoLogger'] = None
        self._streamer_manager: Optional['StreamerManager'] = None

    # ── Consumer registration ──────────────────────────────────────

    def set_display_enabled(self, enabled: bool):
        self._display_enabled = enabled

    def set_video_logger(self, video_logger: 'VideoLogger'):
        """Attach video logger and register cameras with it."""
        self._video_logger = video_logger
        video_cfg = self.config.get('video_logging', {})
        cameras_to_log = video_cfg.get('cameras', [])

        for cam_name in cameras_to_log:
            if cam_name in self.sim.renderers:
                renderer = self.sim.renderers[cam_name]
                self._video_logger.add_camera(cam_name, renderer.width, renderer.height)
            else:
                print(f"Warning: Camera '{cam_name}' not found for video logging")

    def set_streamer_manager(self, streamer_manager: 'StreamerManager'):
        """Attach streamer manager and initialize pipelines with frame dimensions."""
        self._streamer_manager = streamer_manager

        for cam_name in streamer_manager.get_cameras():
            if cam_name in self.sim.renderers:
                renderer = self.sim.renderers[cam_name]
                streamer_manager.initialize_camera(
                    cam_name, renderer.width, renderer.height, self._fps
                )
            else:
                print(f"Warning: Camera '{cam_name}' not found for streaming")

    # ── Main loop ──────────────────────────────────────────────────

    def run(self):
        """Blocking frame distribution loop. Run on main thread (CV2 requirement)."""
        self.running = True

        if not self.sim.renderers:
            print("No renderers configured. FrameDistributor exiting.")
            return

        while self.running and self.sim.running:
            t0 = time.time()
            self._tick()

            elapsed = time.time() - t0
            sleep_time = self._frame_interval - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

        if self._display_enabled:
            cv2.destroyAllWindows()

    def stop(self):
        self.running = False
        if self._display_enabled:
            cv2.destroyAllWindows()

    # ── Per-frame logic ────────────────────────────────────────────

    def _tick(self):
        """Render all cameras once and distribute to consumers."""
        frames: Dict[str, np.ndarray] = {}

        # Render all configured cameras (under lock for scene update)
        with self.sim._lock:
            for cam_name, renderer in self.sim.renderers.items():
                renderer.update_scene(self.sim.mj_data, camera=renderer._cam_id)

        for cam_name, renderer in self.sim.renderers.items():
            frame_rgb = renderer.render()
            frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
            frames[cam_name] = frame_bgr

        # ── Distribute to consumers ──

        # 1. Video logging
        if self._video_logger is not None:
            video_cameras = self.config.get('video_logging', {}).get('cameras', [])
            for cam_name in video_cameras:
                if cam_name in frames:
                    self._video_logger.log_frame(cam_name, frames[cam_name])

        # 2. Streaming
        if self._streamer_manager is not None:
            for cam_name, frame in frames.items():
                self._streamer_manager.send_frame(cam_name, frame)

        # 3. Display (CV2 windows)
        if self._display_enabled:
            self._update_display(frames)

    def _update_display(self, frames: Dict[str, np.ndarray]):
        """Compose and show frames in CV2 window."""
        display_frames = []
        for cam_name, frame in frames.items():
            annotated = frame.copy()
            cv2.putText(
                annotated, cam_name, (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2, cv2.LINE_AA
            )
            display_frames.append(annotated)

        if not display_frames:
            return

        display_cfg = self.config.get('display', {})
        layout = display_cfg.get('layout', 'horizontal')

        if len(display_frames) == 1:
            combined = display_frames[0]
        elif layout == 'horizontal':
            combined = np.hstack(display_frames)
        elif layout == 'vertical':
            combined = np.vstack(display_frames)
        else:
            cols = display_cfg.get('grid_cols', 2)
            rows = (len(display_frames) + cols - 1) // cols
            while len(display_frames) < rows * cols:
                display_frames.append(np.zeros_like(display_frames[0]))
            grid_rows = [np.hstack(display_frames[i * cols:(i + 1) * cols]) for i in range(rows)]
            combined = np.vstack(grid_rows)

        window_name = display_cfg.get('window_name', 'Camera Views')
        cv2.imshow(window_name, combined)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.running = False
            print("User requested to stop simulation")