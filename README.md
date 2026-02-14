# SimCore

Modular simulation and control framework for robotic manipulation. Built on [MuJoCo](https://mujoco.org/) and [Pinocchio](https://github.com/stack-of-tasks/pinocchio), designed to be used as a shared core across multiple robotics research projects.

## Features

- **Configurable scene composition** — assemble multi-device scenes (robots, sensors, objects) from individual MuJoCo XML models via YAML config files
- **Cartesian and joint-space control** — impedance control with proper SO(3) orientation error, joint position control, and extensible controller architecture
- **Kinematic modeling** — forward kinematics, Jacobians, gravity compensation, and dynamics via Pinocchio
- **Structured data logging** — time-synchronized signal logging to HDF5 with automatic schema validation
- **Video recording** — configurable camera rendering and video capture from simulation
- **Multi-robot support** — tested with Franka Panda, Boston Dynamics Spot, and Unitree H1

## Installation

```bash
git clone https://github.com/yourname/SimCore.git
cd SimCore
pip install -e .
```

## Usage

SimCore is intended as a dependency for task-specific projects (e.g., vision-based MPC, teleoperation, data collection). A minimal example:

```python
from simcore import RobotSystem, Pose, load_yaml

cfg = load_yaml("configs/global_config.yaml")
system = RobotSystem(cfg)

# Set a Cartesian target
target_pose = Pose(position=[0.5, 0.0, 0.4], quaternion=[0, 1, 0, 0])
system.set_target("arm", {"x": target_pose})
system.set_controller_mode("arm", "impedance")

system.run()
```

## Project Structure

```
SimCore/
├── simcore/
│   ├── common/          # Pose, RobotState, kinematics, logging, utilities
│   ├── controller/      # Impedance, joint position, controller manager
│   ├── simulation/      # MuJoCo simulation model and display
│   └── core/            # RobotSystem orchestrator
├── assets/
│   ├── mujoco/          # Robot XMLs, scene files, props
│   └── urdf/            # URDF models for kinematic computation
└── pyproject.toml
```

## Configuration

Scenes are defined in YAML config files specifying devices, objects, cameras, and control parameters. See `assets/` for included robot models and example configurations in downstream projects.

## License

MIT
