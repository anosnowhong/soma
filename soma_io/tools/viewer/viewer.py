import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud2
from octomap_msgs.msg import Octomap
from soma_io import octree
from soma_io import soma_io
import sys
import os, glob

from soma_io import msg_io

"""
The input parameter only need the pcd file, it'll automatically find the bt file and segmented object. This node publish all the sensor data inlucding , point cloud, octomap, and bounding box to ros topics. A rviz config is avaiable in the same folder.
Usage:
python viewer.py /path/to/pcdfile/eg.intermediate0007.pcd 
"""

def bbx_points(bbx_info):

    point_list = [Point() for i in range(8)]

    point_list[0].x = bbx_info['min'][0]
    point_list[0].y = bbx_info['min'][1]
    point_list[0].z = bbx_info['min'][2]

    point_list[1].x = bbx_info['min'][0]
    point_list[1].y = bbx_info['max'][1]
    point_list[1].z = bbx_info['min'][2]

    point_list[2].x = bbx_info['max'][0]
    point_list[2].y = bbx_info['max'][1]
    point_list[2].z = bbx_info['min'][2]

    point_list[3].x = bbx_info['max'][0]
    point_list[3].y = bbx_info['min'][1]
    point_list[3].z = bbx_info['min'][2]

    point_list[4].x = bbx_info['min'][0]
    point_list[4].y = bbx_info['min'][1]
    point_list[4].z = bbx_info['max'][2]

    point_list[5].x = bbx_info['min'][0]
    point_list[5].y = bbx_info['max'][1]
    point_list[5].z = bbx_info['max'][2]

    point_list[6].x = bbx_info['max'][0]
    point_list[6].y = bbx_info['max'][1]
    point_list[6].z = bbx_info['max'][2]

    point_list[7].x = bbx_info['max'][0]
    point_list[7].y = bbx_info['min'][1]
    point_list[7].z = bbx_info['max'][2]

    return point_list


def bbx_lines(point_list):
    bbx.points.append(point_list[0])
    bbx.points.append(point_list[1])
    pub.publish(bbx)

    bbx.points.append(point_list[1])
    bbx.points.append(point_list[2])
    pub.publish(bbx)

    bbx.points.append(point_list[2])
    bbx.points.append(point_list[3])
    pub.publish(bbx)

    bbx.points.append(point_list[0])
    bbx.points.append(point_list[3])
    pub.publish(bbx)

    bbx.points.append(point_list[0])
    bbx.points.append(point_list[4])
    pub.publish(bbx)

    bbx.points.append(point_list[1])
    bbx.points.append(point_list[5])
    pub.publish(bbx)

    bbx.points.append(point_list[2])
    bbx.points.append(point_list[6])
    pub.publish(bbx)

    bbx.points.append(point_list[3])
    bbx.points.append(point_list[7])
    pub.publish(bbx)

    bbx.points.append(point_list[4])
    bbx.points.append(point_list[5])
    pub.publish(bbx)

    bbx.points.append(point_list[5])
    bbx.points.append(point_list[6])
    pub.publish(bbx)

    bbx.points.append(point_list[6])
    bbx.points.append(point_list[7])
    pub.publish(bbx)

    bbx.points.append(point_list[4])
    bbx.points.append(point_list[7])
    pub.publish(bbx)

if __name__ == '__main__':
    if len(sys.argv) == 2:
        print "loading data and drawing bouding box..."
        cloud_file = str(sys.argv[1])
    else:
        print "please input file name"
        exit()

    rospy.init_node('viewer')
    pub = rospy.Publisher('bbx', Marker, queue_size=10)
    pub_cloud = rospy.Publisher('cloud', PointCloud2, queue_size=10)
    pub_oct = rospy.Publisher('oct', Octomap, queue_size=10)
    rate = rospy.Rate(10)

    print 'start publishing topics ...'
    while not rospy.is_shutdown():
        # Prepare bounding box common info
        bbx = Marker()
        bbx.header.frame_id = '/viewer'
        bbx.header.stamp = rospy.get_rostime()
        bbx.type = Marker.LINE_LIST
        # bbx.lifetime.secs = 1.0
        bbx.scale.x = 0.01
        bbx.color.r = 1.0
        bbx.color.a = 1.0

        # Publish point cloud msg
        cloud = msg_io.read_pcd(cloud_file, get_tf=False)
        cloud.header.stamp = rospy.get_rostime()
        cloud.header.frame_id = '/viewer'
        pub_cloud.publish(cloud)

        # Publish octree msg (object octree and overall octree)
        bt_file = cloud_file[:-3] + 'bt'
        oct = msg_io.read_bt(bt_file, get_tf=False)
        oct.header.stamp = rospy.get_rostime()
        oct.header.frame_id = '/viewer'
        pub_oct.publish(oct)

        # Publish bounding box msg for observations
        octr = octree.SOMAOctree()
        octr.load_tree(bt_file)
        bbx_info = octr.bbx_info
        bbx_vertices = bbx_points(bbx_info)
        bbx_lines(bbx_vertices)

        # Publish bounding box msg for segmented objects
        rubbish, n_file = bt_file.rsplit('/', 1)
        num = soma_io.get_number(n_file)
        folder_dir, rubbish = bt_file.rsplit('/', 1)
        pattern = 'rgb_' + str(num) + '_label_*.bt'
        owd = os.getcwd()
        os.chdir(folder_dir)
        for btfile in glob.glob(pattern):
            octr = octree.SOMAOctree()
            octr.load_tree(btfile)
            bbx_info = octr.bbx_info
            bbx_vertices = bbx_points(bbx_info)
            bbx_lines(bbx_vertices)

        os.chdir(owd)

        rate.sleep()



