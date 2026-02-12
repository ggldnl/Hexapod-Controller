"""
Quick and dirty code to plot the current hexapod configuration with matplotlib.
"""

import matplotlib.pyplot as plt
import numpy as np


def rotation_matrix(angles):
    """
    Compute a rotation matrix from roll, pitch, and yaw angles.
    """
    roll, pitch, yaw = angles

    R_x = np.array([
        [1, 0, 0],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll), np.cos(roll)]
    ])

    R_y = np.array([
        [np.cos(pitch), 0, np.sin(pitch)],
        [0, 1, 0],
        [-np.sin(pitch), 0, np.cos(pitch)]
    ])

    R_z = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw), np.cos(yaw), 0],
        [0, 0, 1]
    ])

    return R_z @ R_y @ R_x


def transformation_matrix(rotation, translation):
    """
    Compute a 4x4 transformation matrix from rotation and translation.

    Args:
        rotation: Either a 3x3 rotation matrix or a (3,) array of [roll, pitch, yaw]
        translation: A (3,) array of [x, y, z]
    """
    rotation = np.atleast_1d(rotation)

    if rotation.shape == (3,):  # We have orientation angles
        rotation = rotation_matrix(rotation)

    matrix = np.eye(4)
    matrix[:3, :3] = rotation
    matrix[:3, 3] = translation
    return matrix


def plot_hexapod(config, joint_values, body_position=None, body_orientation=None, foot_positions=None):
    """Self-contained method to plot the current hexapod configuration."""

    fig = plt.figure(figsize=(12, 10))
    ax = fig.add_subplot(111, projection='3d')

    lims = (-200, 200)
    ax.set_xlim(*lims)
    ax.set_ylim(*lims)
    ax.set_zlim(*lims)
    ax.set_box_aspect([1, 1, 1])

    # Plot target feet positions if given
    if foot_positions:
        for leg_name, foot_position in foot_positions.items():
            ax.scatter(foot_position[0], foot_position[1], foot_position[2], color='red')

    legs_config = config["kinematics"]["legs"]
    leg_names = [
        "front_right",
        "middle_right",
        "rear_right",
        "rear_left",
        "middle_left",
        "front_left"
    ]

    body_pos = body_position if body_position is not None else np.zeros(3)
    body_ori = np.radians(body_orientation) if body_orientation is not None else np.zeros(3)
    body_transform = transformation_matrix(body_ori, body_pos)

    # Store leg mount positions and orientations (relative to body center)
    leg_frames = {}
    for leg_name in leg_names:
        leg_config = legs_config[leg_name]
        leg_position = np.array(leg_config["position"])
        leg_orientation = np.radians(np.array(leg_config["orientation"]))  # orientation in config is in degrees for simplicity
        leg_frames[leg_name] = transformation_matrix(leg_orientation, leg_position)  # transformation from body frame to leg frame

    body_points = np.array(
        [(body_transform @ leg_frames[leg_name])[:3, 3] for leg_name in leg_names] +
        [(body_transform @ leg_frames[leg_names[0]])[:3, 3]]
    )

    # Plot body
    ax.plot(body_points[:, 0], body_points[:, 1], body_points[:, 2], color='black')

    coxa_length = config['kinematics']['legs']['coxa']
    femur_length = config['kinematics']['legs']['femur']
    tibia_length = config['kinematics']['legs']['tibia']

    # Plot legs
    for leg_name in leg_names:

        angles = joint_values[leg_name]
        angles = angles if angles is not None else np.zeros(3)
        coxa_angle = np.radians(angles[0])
        femur_angle = np.radians(angles[1])
        tibia_angle = np.radians(angles[2])

        coxa_end_hom = np.array([
            coxa_length * np.cos(coxa_angle),
            coxa_length * np.sin(coxa_angle),
            0,
            1
        ])
        femur_end_hom = np.array([
            (coxa_length + femur_length * np.cos(femur_angle)) * np.cos(coxa_angle),
            (coxa_length + femur_length * np.cos(femur_angle)) * np.sin(coxa_angle),
            femur_length * np.sin(femur_angle),
            1
        ])
        tibia_end_hom = np.array([
            (coxa_length + femur_length * np.cos(femur_angle) + tibia_length * np.cos(femur_angle + tibia_angle)) * np.cos(coxa_angle),
            (coxa_length + femur_length * np.cos(femur_angle) + tibia_length * np.cos(femur_angle + tibia_angle)) * np.sin(coxa_angle),
            (femur_length * np.sin(femur_angle) + tibia_length * np.sin(femur_angle + tibia_angle)),
            1
        ])

        coxa_end_world = (body_transform @ leg_frames[leg_name] @ coxa_end_hom)[:3]
        femur_end_world = (body_transform @ leg_frames[leg_name] @ femur_end_hom)[:3]
        tibia_end_world = (body_transform @ leg_frames[leg_name] @ tibia_end_hom)[:3]

        leg_points = np.stack([
            (body_transform @ leg_frames[leg_name])[:3, 3],
            coxa_end_world,
            femur_end_world,
            tibia_end_world
        ])
        ax.scatter(leg_points[:, 0], leg_points[:, 1], leg_points[:, 2], color='black')
        ax.plot(leg_points[:, 0], leg_points[:, 1], leg_points[:, 2], color='black')

    plt.show()
