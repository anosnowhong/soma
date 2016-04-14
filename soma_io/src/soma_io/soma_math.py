from geometry_msgs.msg import Pose, Quaternion


def qua_to_mat(quaternion_msg):
    a = []
    a11 = 1 - 2 * pow(quaternion_msg.y, 2) - 2 * pow(quaternion_msg.z, 2)
    a12 = 2 * quaternion_msg.x * quaternion_msg.y - 2 * quaternion_msg.z * quaternion_msg.w
    a13 = 2 * quaternion_msg.x * quaternion_msg.z + 2 * quaternion_msg.y * quaternion_msg.w
    a21 = 2 * quaternion_msg.x * quaternion_msg.y + 2 * quaternion_msg.z * quaternion_msg.w
    a22 = 1 - 2 * pow(quaternion_msg.x, 2) - 2 * pow(quaternion_msg.z, 2)
    a23 = 2 * quaternion_msg.y * quaternion_msg.z - 2 * quaternion_msg.x * quaternion_msg.w
    a31 = 2 * quaternion_msg.x * quaternion_msg.z - 2 * quaternion_msg.y * quaternion_msg.w
    a32 = 2 * quaternion_msg.y * quaternion_msg.z + 2 * quaternion_msg.x * quaternion_msg.w
    a33 = 1 - 2 * pow(quaternion_msg.x, 2) - 2 * pow(quaternion_msg.y, 2)
    a.append([a11, a12, a13])
    a.append([a21, a22, a23])
    a.append([a31, a32, a33])
    return a


def quaternion_to_matrix(*args):
    """
    Convert ROS quaternion_msg to 3 by 3 rotation matrix.
    :param quaternion_msg:
    :return:
    """
    if len(args) == 1:
        quaternion_msg = args[0]
        if isinstance(quaternion_msg, Quaternion):
            rot = qua_to_mat(quaternion_msg)
        else:
            raise Exception("Illegal input argument, expect: ", Quaternion, "but got: ", type(quaternion_msg))
    elif len(args) == 4:
        quaternion_msg = Quaternion()
        quaternion_msg.x = args[0]
        quaternion_msg.y = args[1]
        quaternion_msg.z = args[2]
        quaternion_msg.w = args[3]
        rot = qua_to_mat(quaternion_msg)
    return rot

