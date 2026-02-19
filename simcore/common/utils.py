import yaml
from pathlib import Path
import numpy as np

def load_yaml(path: str):
    with open(path, "r") as f:
        data = yaml.safe_load(f)
    return data

def get_asset_path(relative_path=None):
    """Returns absolute path to SimCore's assets directory."""
    assets_dir = Path(__file__).parent.parent.parent / "assets"
    if relative_path:
        return str(assets_dir / relative_path)
    return str(assets_dir)

def look_at_quaternion(eye: np.ndarray, target: np.ndarray, up=np.array([0,0,1])):
    # MuJoCo camera looks along -Z, Y is up in camera frame
    forward = target - eye
    forward /= np.linalg.norm(forward)
    
    # Camera -Z should align with forward
    # So we need R such that R @ [0,0,-1] = forward
    z_cam = -forward
    x_cam = np.cross(up, z_cam)
    x_cam /= np.linalg.norm(x_cam)
    y_cam = np.cross(z_cam, x_cam)
    
    R = np.column_stack([x_cam, y_cam, z_cam])
    # Convert rotation matrix to quaternion (w,x,y,z)
    from scipy.spatial.transform import Rotation
    return Rotation.from_matrix(R).as_quat()[[3,0,1,2]]  # scipy gives xyzw, convert to wxyz