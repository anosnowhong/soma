#!/usr/bin/env python
# This tool is used to transform point cloud read form file to a global coordinate system by using the transformation
# provided by the room.xml file in the dataset folder of each room.

import sys
from soma_io.soma_io import FileIO
from soma_io import soma_math
import numpy as np
import copy
import pcl

if __name__ == '__main__':
    if len(sys.argv) is 2:
        print "search pcd file under given path: ", str(sys.argv[1])
        ROMBUS_DB = str(sys.argv[1])
    else:
        ROMBUS_DB = "/media/psf/strands_data_backup/20150505/patrol_run_10/room_6"
        print "Path not given, use default path instead: "
        print "======>", ROMBUS_DB

    pcd_list = FileIO.scan_file(ROMBUS_DB, ".pcd")
    print "Found ", len(pcd_list), "target files"
    pcd_list.sort()

    time = 0

    for pcd in pcd_list:
        # Prepare needed data

        print "Transform to global coordinate system, " + pcd

        dir_name, turd = pcd.rsplit('/', 1)
        xml_file = dir_name + '/room.xml'
        turd, pcd_file = pcd.rsplit('/', 1)
        if not pcd_file.startswith('intermediate'):
            print "skip."
            continue
        gl_pose = FileIO.get_xml_pose(xml_file, pcd_file)
        rot_mat = soma_math.quaternion_to_matrix(gl_pose.pose.orientation)

        cloud_data_nan = FileIO.load_pcd(pcd)
        cloud_data = pcl.PointCloud()
        FileIO.remove_nan(cloud_data_nan, cloud_data)

        cloud_array = np.asarray(cloud_data)

        # Transformation
        #cloud_array_new = np.empty([cloud_array.shape[0], cloud_array.shape[1]], dtype=np.float32)
        count = 0
        for p in cloud_array:
            r_point = np.dot(rot_mat, [p[0], p[1], p[2]])
            r_point[0] += float(gl_pose.pose.position.x)
            r_point[1] += float(gl_pose.pose.position.y)
            r_point[2] += float(gl_pose.pose.position.z)
            cloud_array[count] = r_point
            count += 1

        cloud_save = pcl.PointCloud()
        cloud_save.from_array(cloud_array)
        # Save pcd to current folder
        save_name = '/home/anokk' + '/transformed' + pcd_file

        FileIO.save_pcd(cloud_save, save_name)
        time += 1

