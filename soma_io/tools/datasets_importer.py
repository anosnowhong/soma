#!/usr/bin/env python
import rospy
import argparse
import sys
from soma_io.soma_io import FileIO
from soma_io.state import World

"""
Inport all the data to mongodb, works for G4S dataset.
The information that are imported are listed below:
/tf
/pcl

"""
#ROMBUS_DB = "/media/psf/strands_data_backup/20150505/patrol_run_10/room_6"

if __name__ == '__main__':
    parser = argparse.ArgumentParser(prog='datasets_importer.py')
    parser.add_argument("-m", metavar='mode', help="'local' for saved dataset,'online' for ros topics ")
    parser.add_argument("-p", metavar='path', help="when use 'local' mode, need specify the path to dataset")

    args = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])

    if args.m == 'local':
        rospy.init_node("local_data_importer", anonymous=True)

        # load local dataset info index
        if args.p is None:
            print "error: need specify path parameter"
            exit()
        ROMBUS_DB = args.p
        object_classes = FileIO.scan_objects(ROMBUS_DB)
        rooms = FileIO.scan_xml(ROMBUS_DB, "room.xml")

        # init collection and start inserting data
        world = World()
        rooms.sort(key=lambda x: x[-1])
        for i, r in enumerate(rooms):
            print "-" * 20
            print "[%d/%d]" % (i, len(rooms)), r[0], "\t\t", r[-1]
            print "\n" * 5
            FileIO.parse_room(world, r[0], r[2], object_classes)

        print "Done!"
        exit()
    elif args.m == 'online':
	print args.u
        rospy.init_node("online_data_importer", anonymous=True)
        pass
    else:
         print("Unknown options detected, use -h for usage info")
