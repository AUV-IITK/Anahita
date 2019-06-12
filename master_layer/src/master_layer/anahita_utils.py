import tf
from geometry_msgs.msg import Pose, Point

def TO_DEGREE(angle_in_rad):
    return angle_in_rad*180/3.145

def TO_RADIAN(angle_in_deg):
    return angle_in_deg*3.145/180

def calc_dist (pose1, pose2):
    x1 = pose1.x
    y1 = pose1.y
    z1 = pose1.z

    x2 = pose2.x
    y2 = pose2.y
    z2 = pose2.z

    dist = (x1-x2)*(x1-x2) + (y1-y2)*(y1-y2) + (z1-z2)*(z1-z2)
    return dist

def fill_pose_data_pq(pose_object, pos, quaternion):
    pose_object.position.x = pos.x
    pose_object.position.y = pos.y
    pose_object.position.z = pos.z
    pose_object.orientation.x = quaternion[0]
    pose_object.orientation.y = quaternion[1]
    pose_object.orientation.z = quaternion[2]
    pose_object.orientation.w = quaternion[3]

def fill_pose_data_xyzq(pose_object, pos_x, pos_y, pos_z, quaternion):
    pose_object.position.x = pos_x
    pose_object.position.y = pos_y
    pose_object.position.z = pos_z
    pose_object.orientation.x = quaternion.x
    pose_object.orientation.y = quaternion.y
    pose_object.orientation.z = quaternion.z
    pose_object.orientation.w = quaternion.w

def fill_pose_data(pose_object, pos_x, pos_y, pos_z, orient_x, orient_y, orient_z, orient_w):
    pose_object.position.x = pos_x
    pose_object.position.y = pos_y
    pose_object.position.z = pos_z
    pose_object.orientation.x = orient_x
    pose_object.orientation.y = orient_y
    pose_object.orientation.z = orient_z
    pose_object.orientation.w = orient_w

def quaternion_to_eulerRPY(orientation_in_quat):

    # HACK ON GITHUB, ELSE WEIRD ERROR COMES
    explicit_orient_in_quat = [orientation_in_quat.x, orientation_in_quat.y, orientation_in_quat.z, orientation_in_quat.w]
    orientation_in_euler = tf.transformations.euler_from_quaternion(explicit_orient_in_quat)
    #print("Euler: R:" + str(orientation_in_euler[0]) + ", P: " + str(orientation_in_euler[1]) + ", Y:" + str(orientation_in_euler[2]))
    return orientation_in_euler[0], orientation_in_euler[1], orientation_in_euler[2]

def eulerRPY_to_quaternion(euler_roll, euler_pitch, euler_yaw):
    orientation_in_quaternion = tf.transformations.quaternion_from_euler(euler_roll, euler_pitch, euler_yaw)
    return orientation_in_quaternion

def fill_pose (p_x, p_y, p_z, q_x, q_y, q_z, q_w):
    
    pose = Pose()
    pose.position.x = p_x
    pose.position.y = p_y
    pose.position.z = p_z
    pose.orientation.x = q_x
    pose.orientation.y = q_y
    pose.orientation.z = q_z
    pose.orientation.w = q_w
    return pose 

def fill_point (x, y, z):

    point = Point()
    point.x = x
    point.y = y
    point.z = z
    return point
