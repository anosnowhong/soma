#!/usr/bin/env python
import rospy
from soma_io.soma_io import FileIO
from soma_io.state import World

if __name__ == '__main__':
    #ROMBUS_DB = "/home/hongru/20140820/patrol_run_2/room_0"
    ROMBUS_DB = "/media/psf/strands_data_backup/20150505/patrol_run_10/room_6"
    object_classes = FileIO.scan_objects(ROMBUS_DB)

    rospy.init_node("data_importer", anonymous=True)

    world = World()

    rooms = FileIO.scan_xml(ROMBUS_DB, "room.xml")

    rooms.sort(key=lambda x: x[-1])
    for i, r in enumerate(rooms):
        print "-" * 20
        print "[%d/%d]" % (i, len(rooms)), r[0], "\t\t", r[-1]
        print "\n" * 5
        FileIO.parse_room(world, r[0], r[2], object_classes)
