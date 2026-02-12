"""
Mock interface that visualizes the robot on Viser instead of sending commands to the real one.
"""

import viser.transforms as vtf
import numpy as np
import trimesh
import viser


class Interface:
    """Viser hexapod kinematic visualization"""

    def __init__(self, config: dict, port: int = 8080):
        self.config = config

        # Enable joints to accept new values
        self.enabled = True

        # Viser config
        self.server = viser.ViserServer(port=port)

        # Extract parameters
        self.coxa_length = config['kinematics']['legs']['coxa'] / 1000.0  # Convert mm to meters
        self.femur_length = config['kinematics']['legs']['femur'] / 1000.0
        self.tibia_length = config['kinematics']['legs']['tibia'] / 1000.0

        self.leg_names = [
            "front_right",
            "middle_right",
            "rear_right",
            "rear_left",
            "middle_left",
            "front_left"
        ]  # should be ordered

        # Storage for scene graph elements
        self.body_frame = None
        self.leg_frames = {}  # mount frames for each leg
        self.coxa_joints = {}
        self.femur_joints = {}
        self.tibia_joints = {}
        self.foot_spheres = {}
        self.target_foot_spheres = {}

        # Current joint values (necessary for partial joint update)
        self.current_joint_values = {}

        self.init()

    def init(self):
        """Initialize the hexapod skeleton in the viser scene."""

        # Create body frame at origin
        self.body_frame = self.server.scene.add_frame("/body", show_axes=False)

        # Get leg configurations
        legs_config = self.config["kinematics"]["legs"]

        # Create body outline by connecting leg mount points
        body_points = []
        for leg_name in self.leg_names:
            leg_config = legs_config[leg_name]
            pos = np.array(leg_config["position"]) / 1000.0  # Convert to meters
            body_points.append(pos)
        body_points.append(body_points[0])  # Close the loop

        # Draw body as a mesh (hexagon outline)
        for i in range(len(body_points) - 1):
            p1 = body_points[i]
            p2 = body_points[i + 1]

            # Create a thin cylinder between points
            direction = p2 - p1
            length = np.linalg.norm(direction)
            if length > 0:
                midpoint = (p1 + p2) / 2

                # Create cylinder mesh
                cylinder = trimesh.creation.cylinder(radius=0.005, height=length)

                # Compute rotation to align cylinder with direction
                z_axis = np.array([0, 0, 1])
                direction_norm = direction / length

                # Rotation from z-axis to direction
                rotation_axis = np.cross(z_axis, direction_norm)
                rotation_axis_norm = np.linalg.norm(rotation_axis)

                if rotation_axis_norm > 1e-6:
                    rotation_axis = rotation_axis / rotation_axis_norm
                    angle = np.arccos(np.clip(np.dot(z_axis, direction_norm), -1, 1))

                    tangent = rotation_axis * angle
                    wxyz = vtf.SO3.exp(tangent).wxyz

                else:
                    # Parallel or anti-parallel
                    if np.dot(z_axis, direction_norm) > 0:
                        wxyz = np.array([1.0, 0, 0, 0])
                    else:
                        wxyz = np.array([0.0, 1, 0, 0])

                self.server.scene.add_mesh_simple(
                    f"/body/edge_{i}",
                    vertices=cylinder.vertices,
                    faces=cylinder.faces,
                    color=(0, 0, 0),
                    position=midpoint,
                    wxyz=wxyz
                )

        # Create legs
        for leg_name in self.leg_names:
            leg_config = legs_config[leg_name]
            leg_position = np.array(leg_config["position"]) / 1000.0  # mm to meters
            leg_orientation_deg = np.array(leg_config["orientation"])
            leg_orientation_rad = np.radians(leg_orientation_deg)

            self.current_joint_values[leg_name] = [0, 0, 0]

            # Create leg mount frame (relative to body)
            leg_mount_frame = self.server.scene.add_frame(
                f"/body/leg_{leg_name}_mount",
                position=leg_position,
                wxyz=vtf.SO3.from_z_radians(leg_orientation_rad[2]).wxyz,
                show_axes=False
            )
            self.leg_frames[leg_name] = leg_mount_frame

            # Create coxa joint (rotates around Z-axis in leg mount frame)
            coxa_joint = self.server.scene.add_frame(
                f"/body/leg_{leg_name}_mount/coxa_joint",
                show_axes=False
            )
            self.coxa_joints[leg_name] = coxa_joint

            # Add coxa link (cylinder from origin to coxa_length along X)
            coxa_cylinder = trimesh.creation.cylinder(radius=0.005, height=self.coxa_length)
            # Rotate cylinder to align with X-axis (default is Z-axis)
            wxyz_x = vtf.SO3.from_y_radians(np.pi / 2).wxyz
            self.server.scene.add_mesh_simple(
                f"/body/leg_{leg_name}_mount/coxa_joint/coxa_link",
                vertices=coxa_cylinder.vertices,
                faces=coxa_cylinder.faces,
                color=(100, 100, 100),
                position=(self.coxa_length / 2, 0, 0),
                wxyz=wxyz_x
            )
            self.server.scene.add_icosphere(
                f"/body/leg_{leg_name}_mount/coxa_joint/coxa_link_tip",
                radius=0.005,
                color=(100, 100, 100),
                position=(self.coxa_length, 0, 0),
                wxyz=wxyz_x
            )

            # Create femur joint at end of coxa (rotates around Y-axis)
            femur_joint = self.server.scene.add_frame(
                f"/body/leg_{leg_name}_mount/coxa_joint/femur_joint",
                position=(self.coxa_length, 0, 0),
                show_axes=False
            )
            self.femur_joints[leg_name] = femur_joint

            # Add femur link
            femur_cylinder = trimesh.creation.cylinder(radius=0.005, height=self.femur_length)
            wxyz_x = vtf.SO3.from_y_radians(np.pi / 2).wxyz
            self.server.scene.add_mesh_simple(
                f"/body/leg_{leg_name}_mount/coxa_joint/femur_joint/femur_link",
                vertices=femur_cylinder.vertices,
                faces=femur_cylinder.faces,
                color=(150, 150, 150),
                position=(self.femur_length / 2, 0, 0),
                wxyz=wxyz_x
            )
            self.server.scene.add_icosphere(
                f"/body/leg_{leg_name}_mount/coxa_joint/femur_joint/femur_link_tip",
                radius=0.005,
                color=(150, 150, 150),
                position=(self.femur_length, 0, 0),
                wxyz=wxyz_x
            )

            # Create tibia joint at end of femur (rotates around Y-axis)
            tibia_joint = self.server.scene.add_frame(
                f"/body/leg_{leg_name}_mount/coxa_joint/femur_joint/tibia_joint",
                position=(self.femur_length, 0, 0),
                show_axes=False
            )
            self.tibia_joints[leg_name] = tibia_joint

            # Add tibia link
            tibia_cylinder = trimesh.creation.cylinder(radius=0.005, height=self.tibia_length)
            wxyz_x = vtf.SO3.from_y_radians(np.pi / 2).wxyz
            self.server.scene.add_mesh_simple(
                f"/body/leg_{leg_name}_mount/coxa_joint/femur_joint/tibia_joint/tibia_link",
                vertices=tibia_cylinder.vertices,
                faces=tibia_cylinder.faces,
                color=(200, 200, 200),
                position=(self.tibia_length / 2, 0, 0),
                wxyz=wxyz_x
            )

            # Add foot sphere at end of tibia
            foot_sphere = self.server.scene.add_icosphere(
                f"/body/leg_{leg_name}_mount/coxa_joint/femur_joint/tibia_joint/foot",
                radius=0.005,
                color=(200, 200, 200),
                position=(self.tibia_length, 0, 0)
            )
            self.foot_spheres[leg_name] = foot_sphere

            # Create target foot position sphere (initially hidden/at origin)
            target_sphere = self.server.scene.add_icosphere(
                f"/target_foot_{leg_name}",
                radius=0.01,
                color=(255, 0, 0),
                # position=(0, 0, -1000)  # Hide initially far below
            )
            self.target_foot_spheres[leg_name] = target_sphere

    def update(self, joint_values, body_position=None, body_orientation=None, foot_positions=None):
        """
        Update the hexapod visualization.

        Args:
            joint_values: Dictionary mapping leg names to [coxa, femur, tibia] angles in degrees
            body_position: [x, y, z] position in mm (optional)
            body_orientation: [roll, pitch, yaw] in degrees (optional)
            foot_positions: Dictionary mapping leg names to [x, y, z] target positions in mm (optional)
        """

        if not self.enabled:
            return

        # Update body frame position and orientation
        if body_position is not None:
            body_pos = np.array(body_position) / 1000.0  # mm to meters
            self.body_frame.position = body_pos

        if body_orientation is not None:
            body_ori = np.radians(body_orientation)
            roll, pitch, yaw = body_ori

            # Compute rotation as ZYX Euler angles
            R = (vtf.SO3.from_z_radians(yaw) @
                 vtf.SO3.from_y_radians(pitch) @
                 vtf.SO3.from_x_radians(roll))
            self.body_frame.wxyz = R.wxyz

        # Update leg joint angles
        for leg_name in self.leg_names:
            if leg_name in joint_values and joint_values[leg_name] is not None:
                angles = joint_values[leg_name]
                coxa_angle = np.radians(angles[0])
                femur_angle = np.radians(angles[1])
                tibia_angle = np.radians(angles[2])

                self.current_joint_values[leg_name] = angles

                # Update joint rotations
                # Coxa rotates around Z-axis
                self.coxa_joints[leg_name].wxyz = vtf.SO3.from_z_radians(coxa_angle).wxyz

                # Femur rotates around Y-axis
                self.femur_joints[leg_name].wxyz = vtf.SO3.from_y_radians(-femur_angle).wxyz

                # Tibia rotates around Y-axis
                self.tibia_joints[leg_name].wxyz = vtf.SO3.from_y_radians(-tibia_angle).wxyz

        # Update target foot positions if provided
        if foot_positions is not None:
            for leg_name, foot_pos in foot_positions.items():
                if leg_name in self.target_foot_spheres:
                    pos_meters = np.array(foot_pos) / 1000.0  # mm to meters
                    self.target_foot_spheres[leg_name].position = pos_meters
        else:
            # Hide target spheres if not provided
            for leg_name in self.target_foot_spheres:
                self.target_foot_spheres[leg_name].position = (0, 0, -1000)

    # Hardware interface

    def enable(self):
        self.enabled = True

    def disable(self):
        self.enabled = False

    def convert_angle(self, leg, joint, angle, **kargs):
        """
        Convert angle from kinematic space to servo space.
        In this mock interface the two sapces coincide, so nothing will be done.
        """
        return angle

    def set_leg_angle(self, leg, joint, angle, **kargs):
        new_angle = self.convert_angle(leg, joint, angle)
        current_joint_values = self.current_joint_values
        new_angles = current_joint_values[leg]
        new_angles[['coxa', 'femur', 'tibia'].index(joint)] = new_angle
        current_joint_values[leg] = new_angles
        self.update(
            current_joint_values,
            **kargs
        )

    def set_all_legs(self, leg_angles):

        joint_names = ['coxa', 'femur', 'tibia']
        for leg_name in self.leg_names:
            angles = leg_angles[leg_name]
            mapped_angles = [self.convert_angle(leg_name, joint_names[i], angles[i]) for i in range(len(angles))]
            leg_angles[leg_name] = mapped_angles  # same as before

        self.update(leg_angles)

    def get_voltage(self):
        return 6.1  # mock value

    def get_current(self):
        return 10.4  # mock value

    def check(self):
        return True

    def set_led(pin, r, g, b):
        return True
