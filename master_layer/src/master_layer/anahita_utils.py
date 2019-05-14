def calc_dist (pose1, pose2):
    x1 = pose1.x
    y1 = pose1.y
    z1 = pose1.z

    x2 = pose2.x
    y2 = pose2.y
    z2 = pose2.z

    dist = (x1-x2)*(x1-x2) + (y1-y2)*(y1-y2) + (z1-z2)*(z1-z2)
    return dist


def fill_pose_data(pose_object, pos_x, pos_y, pos_z, orient_x, orient_y, orient_z, orient_w):
    pose_object.position.x = pos_x
    pose_object.position.y = pos_y
    pose_object.position.z = pos_z
    pose_object.orientation.x = orient_x
    pose_object.orientation.y = orient_y
    pose_object.orientation.z = orient_z
    pose_object.orientation.w = orient_w