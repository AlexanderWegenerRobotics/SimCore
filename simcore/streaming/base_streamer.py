from abc import ABC, abstractmethod
import numpy as np
from typing import Optional, Tuple


class BaseStreamer(ABC):
    """Abstract base class for video streamers.
    
    Follows the same pattern as BaseController - subclass and implement
    the abstract methods for different streaming backends (GStreamer, FFmpeg, WebRTC, etc.)
    """

    def __init__(self, config: dict):
        self.config = config
        self._initialized = False
        self._width: int = 0
        self._height: int = 0
        self._fps: int = 30

    @abstractmethod
    def initialize(self, width: int, height: int, fps: int = 30):
        """Initialize the streaming pipeline with frame dimensions and target fps."""
        pass

    @abstractmethod
    def send_frame(self, frame: np.ndarray):
        """Push a single BGR frame into the streaming pipeline.
        
        Args:
            frame: BGR image as numpy array (H, W, 3), uint8
        """
        pass

    @abstractmethod
    def stop(self):
        """Tear down the streaming pipeline and release resources."""
        pass

    def is_initialized(self) -> bool:
        return self._initialized