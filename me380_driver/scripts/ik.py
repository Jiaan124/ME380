#!/usr/bin/env python3
"""
ME380 inverse kinematics using ikpy.
Loads the robot chain from a URDF and computes joint angles for a target pose.

Install: pip install ikpy numpy
Optional (for --plot): pip install matplotlib

URDF: Use a processed URDF (e.g. from xacro) so kinematics match your robot:
  xacro urdf/me380_robot.urdf.xacro > scripts/me380_robot.urdf

Usage:
  # Use URDF next to this script (e.g. scripts/me380_robot.urdf)
  python ik.py

  # Or pass URDF path and target x y z
  python ik.py /path/to/me380_robot.urdf 0.3 0.1 0.4

  # From code:
  from ik import get_chain, inverse_kinematics
  chain = get_chain()
  q = inverse_kinematics(chain, target_position=[0.3, 0.1, 0.4])
"""

import argparse
import os
import sys

import numpy as np

# Optional: only needed for plotting
try:
    import matplotlib.pyplot as plt
    HAS_MATPLOTLIB = True
except ImportError:
    HAS_MATPLOTLIB = False

import ikpy.chain


def _default_urdf_path():
    """Path to me380_robot.urdf: same dir as this script or ../urdf/ (after xacro)."""
    script_dir = os.path.dirname(os.path.abspath(__file__))
    candidates = [
        os.path.join(script_dir, "me380_robot.urdf"),
        os.path.join(script_dir, "..", "urdf", "me380_robot.urdf"),
    ]
    for p in candidates:
        if os.path.isfile(p):
            return p
    return os.path.join(script_dir, "me380_robot.urdf")


def get_chain(
    urdf_path=None,
    base_link="base_link",
    active_links_mask=None,
):
    """
    Build an ikpy Chain from the ME380 URDF.

    Args:
        urdf_path: Path to the URDF file. If None, uses default (see _default_urdf_path).
        base_link: Name of the base link for the chain.
        active_links_mask: Which links are active (revolute). If None, all non-fixed joints are active.
            For ME380 with base_link first: [False, True, True, True, True, True, True].

    Returns:
        ikpy.chain.Chain
    """
    if urdf_path is None:
        urdf_path = _default_urdf_path()
    if not os.path.isfile(urdf_path):
        raise FileNotFoundError(
            f"URDF not found: {urdf_path}. "
            "Generate it with: xacro urdf/me380_robot.urdf.xacro > scripts/me380_robot.urdf"
        )

    base_elements = [base_link] if base_link else None
    kwargs = {
        "urdf_file": urdf_path,
        "name": "me380",
    }
    if base_elements is not None:
        kwargs["base_elements"] = base_elements
    if active_links_mask is not None:
        kwargs["active_links_mask"] = active_links_mask

    chain = ikpy.chain.Chain.from_urdf_file(**kwargs)
    return chain


def inverse_kinematics(
    chain,
    target_position,
    target_orientation=None,
    orientation_mode="Z",
    initial_position=None,
):
    """
    Compute inverse kinematics for a target pose.

    Args:
        chain: ikpy.chain.Chain (from get_chain).
        target_position: [x, y, z] in meters.
        target_orientation: Optional (3,) axis for orientation (e.g. Z axis of end-effector).
        orientation_mode: "X", "Y", "Z", or None (position only).
        initial_position: Optional initial joint array for the optimizer (same length as chain).

    Returns:
        np.ndarray: Joint angles (full length, including inactive). Use active_from_full if you need only the 6 joint values.
    """
    target_position = np.array(target_position, dtype=float)
    if target_position.shape != (3,):
        raise ValueError("target_position must be length 3 (x, y, z)")

    kwargs = {"target_position": target_position}
    if target_orientation is not None:
        kwargs["target_orientation"] = np.array(target_orientation, dtype=float)
        kwargs["orientation_mode"] = orientation_mode or "Z"
    if initial_position is not None:
        kwargs["initial_position"] = np.array(initial_position, dtype=float)

    return chain.inverse_kinematics(**kwargs)


def inverse_kinematics_frame(chain, target_matrix, initial_position=None):
    """
    Compute IK for a full 4x4 target frame (position + orientation).

    Args:
        chain: ikpy.chain.Chain.
        target_matrix: 4x4 transformation matrix (rotation + translation) in meters.
        initial_position: Optional initial joint array.

    Returns:
        np.ndarray: Joint angles (full length).
    """
    kwargs = {"target": np.array(target_matrix, dtype=float)}
    if initial_position is not None:
        kwargs["initial_position"] = np.array(initial_position, dtype=float)
    return chain.inverse_kinematics_frame(**kwargs)


def joint_names(chain):
    """Return list of joint/link names in the chain (for reference)."""
    return [link.name for link in chain.links]


def plot_chain(chain, joint_angles, target_position=None, show=True):
    """Plot the robot and optional target. Requires matplotlib."""
    if not HAS_MATPLOTLIB:
        raise ImportError("matplotlib is required for plotting. pip install matplotlib")
    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")
    target = np.array(target_position) if target_position is not None else None
    chain.plot(joint_angles, ax, target=target, show=False)
    if show:
        plt.show()


def main():
    parser = argparse.ArgumentParser(description="ME380 inverse kinematics (ikpy)")
    parser.add_argument(
        "urdf",
        nargs="?",
        default=None,
        help="Path to me380_robot.urdf (default: next to script or urdf/me380_robot.urdf)",
    )
    parser.add_argument("x", nargs="?", type=float, default=0.3, help="Target X (m)")
    parser.add_argument("y", nargs="?", type=float, default=0.1, help="Target Y (m)")
    parser.add_argument("z", nargs="?", type=float, default=0.4, help="Target Z (m)")
    parser.add_argument("--plot", action="store_true", help="Plot the chain and target")
    parser.add_argument("--no-mask", action="store_true", help="Do not set active_links_mask (use ikpy default)")
    args = parser.parse_args()

    urdf_path = args.urdf or _default_urdf_path()
    active_mask = None if args.no_mask else None  # let ikpy use default (all revolute active)

    chain = get_chain(urdf_path=urdf_path, active_links_mask=active_mask)
    target = [args.x, args.y, args.z]

    print(f"Chain from {urdf_path} (links: {[l.name for l in chain.links]})")
    print(f"Target position: {target} m")

    try:
        q = inverse_kinematics(chain, target_position=target)
    except Exception as e:
        print(f"IK failed: {e}", file=sys.stderr)
        sys.exit(1)

    # Full array includes base; for a 6R arm, q[1:7] are typically the joint angles
    n_links = len(chain.links)
    active = chain.active_from_full(q) if hasattr(chain, "active_from_full") else q
    print(f"Joint angles (rad): {np.array(active).tolist()}")
    print(f"Joint angles (deg): {np.rad2deg(active).tolist()}")

    # Verify with FK
    fk = chain.forward_kinematics(q)
    tip_pos = fk[:3, 3]
    print(f"FK check (tip position): {tip_pos.tolist()} (error: {np.linalg.norm(tip_pos - np.array(target)):.6f} m)")

    if args.plot and HAS_MATPLOTLIB:
        plot_chain(chain, q, target_position=target)


if __name__ == "__main__":
    main()
