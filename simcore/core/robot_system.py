from pathlib import Path
import numpy as np
import time, threading
from typing import Dict

from simcore.simulation.sim_model import SimulationModel
from simcore.simulation.frame_distributor import FrameDistributor
from simcore.controller.controller_manager import ControllerManager
from simcore.common.robot_kinematics import RobotKinematics
from simcore.common.data_logger import DataLogger
from simcore.common.video_logger import VideoLogger
from simcore.common.utils import load_yaml, get_asset_path
from simcore.common.pose import Pose
from simcore.streaming.streamer_manager import StreamerManager

class RobotSystem:
    def __init__(self, config: Dict):
        self.config = config
        self.sim_cfg = load_yaml(self.config.get("scene_config"))
        self.sim_cfg = self._resolve_asset_paths(self.sim_cfg)
        self.headless = self.sim_cfg.get("headless", False)

        log_dir = Path(self.config.get("logging_path", "log/"))
        trial_name = self.config.get("trial_name", f"trial_{int(time.time())}")
        trial_dir = log_dir / trial_name
        trial_dir.mkdir(parents=True, exist_ok=True)
        
        self.logger = DataLogger(trial_name, str(log_dir))

        # ── Video logger (optional, config-driven) ────────────────
        self.video_logger = None
        if not self.headless and self.sim_cfg.get('video_logging', {}).get('enabled', False):
            fps = self.sim_cfg['video_logging'].get('frequency', 10)
            self.video_logger = VideoLogger(trial_dir, fps=fps)
            print(f"Video logging enabled at {fps} fps")
        
        # ── Simulation model ──────────────────────────────────────
        self.sim = SimulationModel(config=self.sim_cfg, logger=self.logger)

        # ── Frame distributor (replaces SimulationDisplay) ────────
        #    Renders once, distributes to: display + video logger + streamers
        #    Skipped entirely in headless mode
        self.distributor = None
        self.streamer_manager = None
        if not self.headless:
            self.distributor = FrameDistributor(sim=self.sim, config=self.sim_cfg)

            display_enabled = self.sim_cfg.get('display', {}).get('enabled', True)
            self.distributor.set_display_enabled(display_enabled)

            if self.video_logger is not None:
                self.distributor.set_video_logger(self.video_logger)

            # ── Streamer manager (optional, config-driven) ────────────
            self.streamer_manager = StreamerManager(self.sim_cfg)
            if self.streamer_manager.enabled:
                self.distributor.set_streamer_manager(self.streamer_manager)

        # ── Controllers & kinematics ──────────────────────────────
        self.ctrl, kin_model = {}, {}
        self._target = {}
        for device in self.sim_cfg["devices"]:
            device_name = device["name"]
            base_pose = device["base_pose"]
            dof = device["dof"]
            q0 = device.get("q0")
            if q0 and len(q0) >= dof:
                q_init = np.array(q0[:dof])
            else:
                q_init = np.zeros(dof)

            self._target[device_name] = {
                'q': q_init,
                'x': Pose(position=np.zeros(3), quaternion=np.array([1,0,0,0])),
                'xd': np.zeros(6)
            }
            urdf_path = device.get("urdf_path", None)
            urdf_ee_name = device.get("urdf_ee_name", None)
            robot_kin = None
            if urdf_path and urdf_ee_name:
                if urdf_path not in kin_model:
                    robot_kin = RobotKinematics(urdf_path=urdf_path, ee_frame_name=urdf_ee_name)
                    kin_model[urdf_path] = robot_kin
                else:
                    robot_kin = kin_model[urdf_path]
            param_path = device.get("ctrl_param")
            if not param_path:
                print(f"Warning! For {device_name} there is no control parameter file! Cant initialize model like this")
                continue

            ctrl_param = load_yaml(param_path)
            cfg = {
                "name":device_name,
                "base_pose":base_pose,
                "kinematic_model":robot_kin,
                "control_param":ctrl_param
            }

            self.ctrl[device_name] = ControllerManager(config=cfg, logger=self.logger)

        self.control_rate = self.sim_cfg.get("control_rate", 200.0)
        self.dt = 1.0 / self.control_rate
        self.n_substeps = max(1, round(1.0 / (self.control_rate * self.sim.dt)))
        self.running = False
        
        self._lock = threading.Lock()
    
    def run(self):
        """Start all subsystems and block on frame distributor (main thread)."""
        if self.headless:
            self.running = True
            self.sim.running = True
            print(f"System running in headless mode (n_substeps={self.n_substeps})")
            return

        self.running = True
        
        self.sim.start()
        
        self.control_thread = threading.Thread(target=self._loop, daemon=False)
        self.control_thread.start()
        
        try:
            self.distributor.run()
        except KeyboardInterrupt:
            print("\nShutting down...")
        finally:
            self.stop()
    
    def stop(self):
        """Shutdown all subsystems"""
        if not self.running:
            return
        
        print("Shutting down robot system...")
        self.running = False

        if self.headless:
            self.sim.running = False
            self.logger.save()
            print("Robot system stopped (headless)")
            return
        
        if hasattr(self, 'control_thread') and self.control_thread.is_alive():
            self.control_thread.join(timeout=2.0)
        
        self.distributor.stop()
        self.streamer_manager.stop()
        self.sim.stop()
        if self.video_logger is not None:
            self.video_logger.close()
        self.logger.save()
        
        print("Robot system stopped")

    def step(self):
        """Single synchronous step: read state -> compute control -> physics step.
        
        For headless mode. One call = one control cycle = n_substeps physics steps.
        Runs as fast as the CPU allows — no sleeps.
        """
        states = self.sim.get_state()
        with self._lock:
            target = self._target.copy()
        for name, ctrl in self.ctrl.items():
            ctrl_vec = ctrl.compute_control(states[name], target[name])
            self.sim.set_command(tau=ctrl_vec, device_name=name)
        for _ in range(self.n_substeps):
            self.sim.step()
    
    def _loop(self):
        """Main control loop (threaded, real-time mode only)"""
        print(f"Control loop started (n_substeps={self.n_substeps})")
        
        while self.running:
            start_time = time.time()
            
            states = self.sim.get_state()
            
            with self._lock:
                target = self._target.copy()
            
            for name, ctrl in self.ctrl.items():
                ctrl_vec = ctrl.compute_control(states[name], target[name])
                self.sim.set_command(tau=ctrl_vec, device_name=name)
            
            for _ in range(self.n_substeps):
                self.sim.signal_step()
                while self.sim._command_event.is_set():
                    time.sleep(0.00005)
            
            elapsed = time.time() - start_time
            sleep_time = self.dt - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)
        
        print("Control loop stopped")
    
    def set_target(self, device_name: str, target: Dict, verbose=False):
        """Set control target for a specific device"""
        with self._lock:
            if device_name not in self._target:
                raise ValueError(f"Unknown device: {device_name}")
            
            for key in ['q', 'x', 'xd', 'Fff']:
                if key in target:
                    self._target[device_name][key] = target[key]

            if verbose and 'x' in target:
                self.sim.set_target_pose(target['x'][:3])
    
    def get_state(self) -> Dict:
        """Get current state"""
        return self.sim.get_state()
    
    def get_object_states(self) -> Dict:
        """Get current object state"""
        return self.sim.get_object_states()
    
    def set_controller_mode(self, device_name: str, mode: str):
        """Switch controller mode"""
        if device_name in self.ctrl:
            self.ctrl[device_name].set_mode(mode)

    def get_timestep(self):
        return self.sim.dt

    def _resolve_asset_paths(self, config):
        """Replace relative assets/ paths with absolute SimCore asset paths."""
        if isinstance(config, dict):
            return {k: self._resolve_asset_paths(v) for k, v in config.items()}
        elif isinstance(config, list):
            return [self._resolve_asset_paths(item) for item in config]
        elif isinstance(config, str) and config.startswith("assets/"):
            return get_asset_path(config.removeprefix("assets/"))
        return config
    

if __name__ == "__main__":
    cfg = load_yaml("configs/global_config.yaml")
    system = RobotSystem(cfg)

    try:
        system.run()
    except KeyboardInterrupt:
        print("\nInterrupted")
    finally:
        system.stop()