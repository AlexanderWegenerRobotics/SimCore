import numpy as np
import pinocchio as pin
from simcore.common.pose import Pose


class RobotKinematics:
    def __init__(self, urdf_path: str, ee_frame_name: str):
        self.model = pin.buildModelFromUrdf(urdf_path)
        self.data = self.model.createData()
        
        try:
            self.ee_frame_id = self.model.getFrameId(ee_frame_name)
        except:
            print(f"Available frames: {[self.model.frames[i].name for i in range(self.model.nframes)]}")
            raise ValueError(f"Frame '{ee_frame_name}' not found in URDF")
        
    def forward_kinematics(self, q):
        pin.framesForwardKinematics(self.model, self.data, q)
        pose = self.data.oMf[self.ee_frame_id]
        return Pose.from_matrix(pose.translation.copy(), pose.rotation.copy())
    
    def get_jacobian(self, q):
        pin.computeJointJacobians(self.model, self.data, q)
        J = pin.getFrameJacobian(self.model, self.data, self.ee_frame_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
        return J
    
    def get_ee_velocity(self, q, qd):
        J = self.get_jacobian(q)
        return J @ qd
    
    def get_gravity_torques(self, q):
        pin.forwardKinematics(self.model, self.data, q)
        tau_g = pin.computeGeneralizedGravity(self.model, self.data, q)
        return tau_g[:7]
    
    def get_mass_matrix(self, q):
        pin.crba(self.model, self.data, q)
        return self.data.M[:7, :7]

    def get_coriolis_matrix(self, q, qd):
        pin.computeCoriolisMatrix(self.model, self.data, q, qd)
        return self.data.C[:7, :7]
    
    def get_internal_wrench(self, q, qd, tau_m):
        """ Internal wrench F_in = J_body†T (tau_m - C*qd - g)"""
        C = self.get_coriolis_matrix(q, qd)
        g = self.get_gravity_torques(q)
        pin.computeJointJacobians(self.model, self.data, q)
        J_body = pin.getFrameJacobian(
            self.model, self.data, 
            self.ee_frame_id, 
            pin.ReferenceFrame.LOCAL
        )
        J_body_pinv = np.linalg.pinv(J_body)
        return J_body_pinv.T @ (tau_m - C @ qd - g)
    
    def get_cartesian_mass_matrix(self, q):
        """Cartesian mass matrix: Lambda = (J M^-1 J^T)^-1"""
        M = self.get_mass_matrix(q)         # joint space mass matrix (7x7)
        J = self.get_jacobian(q)            # Jacobian (6x7)

        # Compute M^-1 J^T first for efficiency
        M_inv       = np.linalg.inv(M)      # joint space mass matrix inverse
        J_M_inv_JT  = J @ M_inv @ J.T      # (6x6) intermediate term

        # Cartesian mass matrix via inverse of J M^-1 J^T
        Lambda = np.linalg.inv(J_M_inv_JT)  # (6x6) cartesian mass matrix

        return Lambda