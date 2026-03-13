import numpy as np
import pinocchio as pin
from simcore.controller.base_controller import BaseController


class DynamicImpedanceController(BaseController):
    def __init__(self, config, robot_kinematics=None):
        super().__init__(config)

        if robot_kinematics is None:
            raise ValueError("ComputedTorqueController requires a kinematic model")

        self.robot_kin = robot_kinematics

        # Cartesian impedance gains
        self.K_cart = np.diag(config['K_cart'])
        self.D_cart = np.diag(config['D_cart'])

        # Nullspace configuration
        self.K_null    = config['K_null']
        self.q_nominal = np.array(config.get('q_nominal', np.zeros(7)))

        # Torque limits
        self.tau_max = np.array(config['tau_max'])

        # Gravity compensation flag
        self.gravity_comp = config.get('gravity_compensation', True)

        self._initialized = True

    def compute_control(self, state, target):
        q  = state.q
        qd = state.qd

        # Target cartesian state
        x_desired   = target['x']                        # Pose object
        xd_desired  = target.get('xd',  np.zeros(6))    # desired EE velocity
        xdd_desired = target.get('xdd', np.zeros(6))    # desired EE acceleration
        F_ff        = target.get('Fff', np.zeros(6))    # feedforward wrench

        # Current cartesian state
        x_current  = self.robot_kin.forward_kinematics(q)
        xd_current = self.robot_kin.get_ee_velocity(q, qd)

        # Jacobian
        J = self.robot_kin.get_jacobian(q)

        # Dynamic model terms (compute M once, reuse across all terms)
        M      = self.robot_kin.get_mass_matrix(q)           # joint space mass matrix (7x7)
        C      = self.robot_kin.get_coriolis_matrix(q, qd)   # coriolis matrix (7x7)
        Lambda = self._cartesian_mass_matrix(J, M)           # cartesian mass matrix (6x6)

        # Position error (base frame)
        e_pos = x_desired.position - x_current.position

        # Orientation error via SO(3) logarithmic map
        R_current = x_current.rotation_matrix
        R_desired = x_desired.rotation_matrix
        R_error   = R_desired @ R_current.T
        e_rot     = pin.log3(R_error)

        # Full cartesian error
        e  = np.concatenate([e_pos, e_rot])
        de = xd_desired - xd_current

        # Cartesian force: impedance + inertia feedforward + coriolis feedforward + feedforward wrench
        F  = self.K_cart @ e                            # stiffness
        F += self.D_cart @ de                           # damping
        F += Lambda @ xdd_desired                       # inertia feedforward
        F += Lambda @ (J @ np.linalg.inv(M) @ C @ qd)   # coriolis feedforward
        F += F_ff                                        # feedforward wrench

        # Map to joint torques via Jacobian transpose
        tau_task = J.T @ F

        # Nullspace torque to keep robot near nominal configuration
        J_pinv         = np.linalg.pinv(J)
        null_projector = np.eye(len(q)) - J_pinv @ J
        tau_null       = self.K_null * (self.q_nominal - q)

        # Combine task and nullspace torques
        tau = tau_task + null_projector @ tau_null

        # Gravity compensation
        if self.gravity_comp:
            tau += self.robot_kin.get_gravity_torques(q)

        return self.validate_torques(tau, self.tau_max)

    def _cartesian_mass_matrix(self, J, M):
        """Cartesian mass matrix: Lambda = (J M^-1 J^T)^-1"""
        M_inv      = np.linalg.inv(M)       # joint space mass matrix inverse
        J_M_inv_JT = J @ M_inv @ J.T        # (6x6) intermediate term
        return np.linalg.inv(J_M_inv_JT)    # (6x6) cartesian mass matrix

    def reset(self):
        pass