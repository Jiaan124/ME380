#!/usr/bin/env python3
"""
Standalone IKPy playground (no ROS2).

Edit the configuration values below, then run:
  python3 ik_test.py
"""

from __future__ import annotations

import math
from pathlib import Path

import ikpy.chain
import numpy as np
from scipy.spatial.transform import Rotation as R
import ikpy.utils.plot as plot_utils
import matplotlib.pyplot as plt

# -----------------------------
# Edit these values to test IKPy
# -----------------------------
HERE = Path(__file__).resolve().parent
URDF_PATH = HERE / "me380_robot.urdf"

# Target position in meters:
TARGET_XYZ = np.array([0.4583, 0, 0.2391], dtype=float)

# Target orientation (Euler XYZ degrees). Set to None to ignore orientation.
TARGET_RPY_DEG: list[float] | None = [0.0, -90.0, 0.0]

# One entry per IKPy chain link, same order as chain.links:
#   False = fixed (not solved; angle stays at initial_position[i])
#   True  = active revolute joint (solved by IK)
# For this URDF: base link + 6 joints + ee_link → 8 links; first & last are not DOFs.
ACTIVE_LINKS_MASK = [False, True, True, True, True, True, True, False]

def main() -> None:
    urdf_path = URDF_PATH.resolve()
    if not urdf_path.exists():
        raise FileNotFoundError(f"URDF not found: {urdf_path}")

    chain = ikpy.chain.Chain.from_urdf_file(
        str(urdf_path),
        active_links_mask=ACTIVE_LINKS_MASK,
    )

    target_orientation = R.from_euler("xyz", TARGET_RPY_DEG, degrees=True).as_matrix()

    initial_position = [0, 0, 2.64, 1.571, 0.0, 1.0, 0, 0] #needed to have correct arm orientation when solving 2.64 rad = 151 deg, 1.57 rad = 90
    initial_position = [0, 0.0, 0.1, -0.1, 0.0, 0.0, 0, 0] #needed to have correct arm orientation when solving 2.64 rad = 151 deg, 1.57 rad = 90


    ik = chain.inverse_kinematics(
        target_position=TARGET_XYZ,
        target_orientation=target_orientation,
        orientation_mode="all",
        initial_position=initial_position,
    )

    print("IK result (radians)") #elements 0 and 7 are dummy elements, will always be 0
    print(ik[1:7].tolist())
    print("IK result (degrees)")
    print([math.degrees(float(a)) for a in ik[1:7]])
    print()

    forward_xyz = chain.forward_kinematics(ik)

    print("forward orientation\n",forward_xyz[:3, :3], "\ntarget orientation\n", R.from_euler("xyz", TARGET_RPY_DEG, degrees=True).as_matrix())


    fig, ax = plot_utils.init_3d_figure()
    fig.set_figheight(8)
    fig.set_figwidth(12)
    chain.plot(ik, ax, target=TARGET_XYZ)
    ax.set_title("IKPy result")
    plt.xlim(-0.4, 0.4)
    plt.ylim(-0.4, 0.4)
    ax.set_zlim(0.0, 0.4)
    plt.show()


if __name__ == "__main__":
    main()

