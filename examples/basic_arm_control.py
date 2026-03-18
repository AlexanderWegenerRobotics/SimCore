"""
Basic arm control example.

Moves a Franka FR3 through three Cartesian waypoints using impedance control,
then holds the final pose. Shows the minimal setup needed to get a robot moving
with SimCore.
"""
import time
import numpy as np
from simcore import RobotSystem, Pose, load_yaml


WAYPOINTS = [
    Pose(position=[0.5, 0.0, 0.6], quaternion=[0, 1, 0, 0]),  # above table
    Pose(position=[0.5, 0.2, 0.6], quaternion=[0, 1, 0, 0]),  # side
    Pose(position=[0.5, 0.0, 0.5], quaternion=[0, 1, 0, 0]),  # lower
]
DWELL_TIME = 2.0  # seconds to hold each waypoint


def main():
    config = load_yaml("examples/configs/global_config.yaml")
    system = RobotSystem(config)
    system.set_controller_mode("arm", "impedance")

    if system.headless:
        system.run()
        _run_waypoints(system)
        system.stop()
    else:
        import threading
        t = threading.Thread(target=_run_waypoints, args=(system,), daemon=True)
        t.start()
        system.run()  # blocks on display; Ctrl-C to quit


def _run_waypoints(system: RobotSystem):
    # Wait for sim to come up
    while not system.sim.running:
        time.sleep(0.05)

    print("Starting waypoint sequence")

    for i, pose in enumerate(WAYPOINTS):
        print(f"  Moving to waypoint {i + 1}/{len(WAYPOINTS)}: {np.round(pose.position, 3)}")
        system.set_target("arm", {"x": pose})

        if system.headless:
            # Step the sim manually for DWELL_TIME seconds
            steps = int(DWELL_TIME / system.get_timestep())
            for _ in range(steps):
                system.step()
        else:
            time.sleep(DWELL_TIME)

    print("Sequence complete — holding final pose")

    if not system.headless:
        # In display mode, keep running until the window is closed
        while system.running:
            time.sleep(0.1)
    else:
        system.stop()


if __name__ == "__main__":
    main()
