import numpy as np
from scipy.spatial.transform import Rotation as R

from simcore.controller.joint_position import JointPositionController
from simcore.controller.impedance import ImpedanceController
from simcore.controller.dynamic_impedance_controller import DynamicImpedanceController
from simcore.common.pose import Pose


class ControllerManager:
    def __init__(self, config, logger=None):
        self.device_name = config["name"]
        self.base_pose   = config["base_pose"]
        self.kin_model   = config.get("kinematic_model", None)
        self.logger      = logger

        # Base frame transforms
        pos      = np.array(self.base_pose["position"])
        quat     = self.base_pose["orientation"]          # [w,x,y,z]
        quat_scipy       = [quat[1], quat[2], quat[3], quat[0]]
        self.R_world_base = R.from_quat(quat_scipy)
        self.R_base_world = self.R_world_base.inv()
        self.p_world_base = pos

        ctrl_params = config["control_param"]
        self.mode   = ctrl_params.get("default_mode", "idle")

        ctrl_names  = list(ctrl_params.keys())
        self.controllers = {"idle": None}

        if "position" in ctrl_names:
            self.controllers["position"] = JointPositionController(
                ctrl_params["position"], self.kin_model)

        if "impedance" in ctrl_names:
            self.controllers["impedance"] = ImpedanceController(
                ctrl_params["impedance"], self.kin_model)

        if "dynamic_impedance" in ctrl_names:
            self.controllers["dynamic_impedance"] = DynamicImpedanceController(
                ctrl_params["dynamic_impedance"], self.kin_model)

        if self.mode not in self.controllers:
            print(f"Warning: Default mode '{self.mode}' not available for "
                  f"{self.device_name}. Using 'idle'.")
            self.mode = "idle"

        self.active_controller = self.controllers[self.mode]
        self._current_state    = None
        self._current_target   = None
        self._current_output   = None

    def set_mode(self, mode):
        if mode not in self.controllers:
            raise ValueError(f"Mode '{mode}' not available for {self.device_name}. "
                             f"Available: {list(self.controllers.keys())}")
        if self.active_controller is not None:
            self.active_controller.reset()

        self.mode             = mode
        self.active_controller = self.controllers[mode]

    def get_current_mode(self) -> str:
        return self.mode

    def compute_control(self, state, target) -> np.ndarray:
        self._current_state  = state
        self._current_target = target

        if self.mode == "idle" or self.active_controller is None:
            return np.zeros(len(state.q))

        # Transform cartesian targets from world to base frame
        if 'x' in target and self.mode in ('impedance', 'dynamic_impedance'):
            target_base = target.copy()
            target_base['x'] = self.transform_world_to_base_frame(target['x'])

            if 'xd' in target:
                xd_world = target['xd']
                target_base['xd'] = np.concatenate([
                    self.R_base_world.apply(xd_world[:3]),
                    self.R_base_world.apply(xd_world[3:])
                ])

            if 'Fff' in target:
                Fff_world = target['Fff']
                target_base['Fff'] = np.concatenate([
                    self.R_base_world.apply(Fff_world[:3]),
                    self.R_base_world.apply(Fff_world[3:])
                ])

            target = target_base

        ctrl_vec             = self.active_controller.compute_control(state, target)
        self._current_output = ctrl_vec

        if self.logger is not None:
            self._log_step()

        return ctrl_vec

    def transform_world_to_base_frame(self, pose_world):
        """Transform Pose from world to base frame"""
        pos_base  = self.R_base_world.apply(pose_world.position - self.p_world_base)
        R_base_ee = self.R_base_world.as_matrix() @ pose_world.rotation_matrix
        return Pose.from_matrix(pos_base, R_base_ee)

    def transform_base_to_world_frame(self, pose_base):
        """Transform Pose from base to world frame"""
        pos_world  = self.R_world_base.apply(pose_base.position) + self.p_world_base
        R_world_ee = self.R_world_base.as_matrix() @ pose_base.rotation_matrix
        return Pose.from_matrix(pos_world, R_world_ee)

    def _log_step(self):
        if self._current_state is None or self._current_output is None:
            return

        bundle_name = f"ctrl_{self.device_name}"
        log_data = {
            'q':        self._current_state.q,
            'qd':       self._current_state.qd,
            'q_target': self._current_target.get('q', np.zeros_like(self._current_state.q)),
            'x_target': self._current_target['x'].as_7d() if isinstance(self._current_target.get('x'), Pose) else np.zeros(7),
            'xd_target': self._current_target.get('xd', np.zeros(6)),
            'tau':       self._current_output,
            'mode':      np.array([self._mode_to_int(self.mode)])
        }

        if self.kin_model is not None:
            x_current_base = self.kin_model.forward_kinematics(self._current_state.q)
            xd_current     = self.kin_model.get_ee_velocity(self._current_state.q, self._current_state.qd)
            log_data['x_current'] = self.transform_base_to_world_frame(x_current_base).as_7d()
            log_data['xd_current'] = xd_current
            log_data['f_internal'] = self.kin_model.get_internal_wrench(self._current_state.q, self._current_state.qd, self._current_output)
            log_data['tau_gravity'] = self.kin_model.get_gravity_torques(self._current_state.q)
            log_data['cartesian_mass']  = self.kin_model.get_cartesian_mass_matrix(self._current_state.q).flatten()
            log_data['Fff']        = self._current_target.get('Fff', np.zeros(6))   # feedforward command sent
            log_data['f_ext']      = self._current_target.get('f_ext', np.zeros(6)) # if you log sensor data here

        self.logger.log_bundle(bundle_name, log_data)

    def _mode_to_int(self, mode: str) -> int:
        mode_map = {
            'idle':              0,
            'position':          1,
            'impedance':         2,
            'dynamic_impedance': 3
        }
        return mode_map.get(mode, -1)

    def get_ee_pose_world(self, state) -> Pose:
        """Returns current end-effector pose in world frame given joint state."""
        pose_base = self.kin_model.forward_kinematics(state.q)
        return self.transform_base_to_world_frame(pose_base)
    
    def get_internal_wrench(self, q, qd, tau) -> np.ndarray:
        return self.kin_model.get_internal_wrench(q, qd, tau)