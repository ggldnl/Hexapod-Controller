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


class LegKinematics:
    """Leg kinematics."""

    def __init__(self, coxa, femur, tibia):
        self.coxa = coxa
        self.femur = femur
        self.tibia = tibia

    def forward(self, coxa_angle, femur_angle, tibia_angle):
        """Compute forward kinematics: from joint angles to end effector position (in leg frame)."""
                
        # Compute end effector position in femur-tibia plane
        r = self.femur * np.cos(femur_angle) + self.tibia * np.cos(femur_angle + tibia_angle)
        z = self.femur * np.sin(femur_angle) + self.tibia * np.sin(femur_angle + tibia_angle)
        
        # Add coxa offset
        r += self.coxa
        
        # Project to 3D
        x = r * np.cos(coxa_angle)
        y = r * np.sin(coxa_angle)
        
        return np.array([x, y, z])
    
    def inverse(self, x, y, z):
        """Compute inverse kinematics: from end effector position (in leg frame) to joint angles."""
        
        # Coxa angle
        alpha = np.arctan2(y, x)
        r = np.sqrt(x ** 2 + y ** 2) - self.coxa

        # Compute 2D planar IK for femur and tibia
        d = np.sqrt(r ** 2 + z ** 2)  # distance to the target
        if d > (self.femur + self.tibia) or d < abs(self.femur - self.tibia):
            return None

        cos_angle2 = (self.femur ** 2 + self.tibia ** 2 - d ** 2) / (2 * self.femur * self.tibia)
        gamma = np.arccos(np.clip(cos_angle2, -1, 1)) - np.pi

        cos_angle1 = (self.femur ** 2 + d ** 2 - self.tibia ** 2) / (2 * self.femur * d)

        beta = np.arctan2(z, r) + np.arccos(np.clip(cos_angle1, -1, 1))

        return alpha, beta, gamma
    

class HexapodKinematics:
    """Hexapod kinematics with support for body orientation and position."""

    def __init__(self, config):

        self.config = config['kinematics']
        
        # Create LegKinematics instances
        legs_config = self.config['legs']
        self.leg_kinematics = LegKinematics(
            coxa=legs_config['coxa'],
            femur=legs_config['femur'],
            tibia=legs_config['tibia']
        )
        
        # Store leg mount positions and orientations (relative to body center)
        self.leg_frames = {}
        self.leg_frames_inv = {}
        for leg_name, leg_config in legs_config.items():
            if leg_name in ['coxa', 'femur', 'tibia']: continue
            leg_position = np.array(leg_config['position'])
            leg_orientation = np.radians(np.array(leg_config['orientation']))   # orientation in config is in degrees for simplicity
            self.leg_frames[leg_name] = transformation_matrix(leg_orientation, leg_position)    # transformation from body frame to leg frame
            self.leg_frames_inv[leg_name] = np.linalg.inv(self.leg_frames[leg_name])            # transformation from leg frame to body frame

    def forward_leg(self, leg_name, coxa_angle, femur_angle, tibia_angle,
                        body_position=None, body_orientation=None):
        """
        Compute forward kinematics for a single leg.
        
        Args:
            leg_name: Name of the leg
            coxa_angle, femur_angle, tibia_angle: Joint angles in radians
            body_position: Optional [x, y, z] position of body center
            body_orientation: Optional [roll, pitch, yaw] orientation of body
            
        Returns:
            End effector position in world frame as [x, y, z]
        """
        # Get position in leg frame
        leg_pos = self.leg_kinematics.forward(coxa_angle, femur_angle, tibia_angle)
        
        # Transform to body frame
        leg_frame = self.leg_frames[leg_name]
        leg_pos_homogeneous = np.append(leg_pos, 1)
        body_pos_homogeneous = leg_frame @ leg_pos_homogeneous
        
        # Apply body transformation if provided
        if body_position is not None or body_orientation is not None:
            body_pos = body_position if body_position is not None else np.zeros(3)
            body_ori = body_orientation if body_orientation is not None else np.zeros(3)
            body_transform = transformation_matrix(body_ori, body_pos)
            world_pos_homogeneous = body_transform @ body_pos_homogeneous
            return world_pos_homogeneous[:3]
        
        return body_pos_homogeneous[:3]
    
    def inverse_leg(self, leg_name, x, y, z, 
                        body_position=None, body_orientation=None):
        """
        Compute inverse kinematics for a single leg.
        
        Args:
            leg_name: Name of the leg
            x, y, z: Target position in world frame
            body_position: Optional [x, y, z] position of body center
            body_orientation: Optional [roll, pitch, yaw] orientation of body
            
        Returns:
            Joint angles [coxa, femur, tibia] in radians, or None if unreachable
        """
        target_position = np.array([x, y, z])
        
        # Transform target from world frame to body frame
        if body_position is not None or body_orientation is not None:
            body_pos = body_position if body_position is not None else np.zeros(3)
            body_ori = body_orientation if body_orientation is not None else np.zeros(3)
            body_transform = transformation_matrix(body_ori, body_pos)
            body_transform_inv = np.linalg.inv(body_transform)
            target_homogeneous = np.append(target_position, 1)
            target_body = body_transform_inv @ target_homogeneous
            target_position = target_body[:3]
        
        # Transform from body frame to leg frame
        target_homogeneous = np.append(target_position, 1)
        target_leg = self.leg_frames_inv[leg_name] @ target_homogeneous
        
        # Compute IK in leg frame
        return self.leg_kinematics.inverse(target_leg[0], target_leg[1], target_leg[2])
    
    def forward(self, joint_angles, body_position=None, body_orientation=None):
        """
        Compute forward kinematics for all legs.
        
        Args:
            joint_angles: Dict mapping leg names to [coxa, femur, tibia] angles
            body_position: Optional [x, y, z] position of body center
            body_orientation: Optional [roll, pitch, yaw] orientation of body
            
        Returns:
            Dict mapping leg names to [x, y, z] positions
        """
        positions = {}
        for leg_name, angles in joint_angles.items():
            positions[leg_name] = self.forward_leg(
                leg_name, angles[0], angles[1], angles[2],
                body_position, body_orientation
            )
        return positions
    
    def inverse(self, target_positions, body_position=None, body_orientation=None):
        """
        Compute inverse kinematics for all legs.
        
        Args:
            target_positions: Dict mapping leg names to [x, y, z] target positions
            body_position: Optional [x, y, z] position of body center
            body_orientation: Optional [roll, pitch, yaw] orientation of body
            
        Returns:
            Dict mapping leg names to [coxa, femur, tibia] angles, or None for unreachable targets
        """
        angles = {}
        for leg_name, position in target_positions.items():
            angles[leg_name] = self.inverse_leg(
                leg_name, position[0], position[1], position[2],
                body_position, body_orientation
            )
        return angles