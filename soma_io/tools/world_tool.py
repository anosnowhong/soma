#!/usr/bin/env python
import rospy
import argparse
import argparse
import sys
from soma_io.soma_io import FileIO
from soma_io.state import World
from mongodb_store.message_store import MessageStoreProxy

"""
A Query tool that can handle the MessageStore message and normal DB data.
usage example:
```
#Query 'room' Object instance
$world_tool.py -q key:WayPoint42
#Query how much observations in a room that covers the point
$world_tool.py -r WayPoint42 -p 28,22,1
```
"""

if __name__ == '__main__':
    # Prepare parameters
    parser = argparse.ArgumentParser(prog='world_tool.py')
    parser.add_argument("-dbpath", metavar='database address and port number',
                        help="format: database_address:port_number")
    parser.add_argument("-q", metavar='query content',
                        help="same as mongodb query input: field:field_content")
    parser.add_argument("-r", metavar='room string',
                        help="query options to specify a room")
    parser.add_argument("-p", metavar='a query point to check which observations covers this point',
                        help="example: -p 1.0,2.0,3.0 the order is x,y,z")
    args = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])

    # Setup Connection
    rospy.init_node("query_tools")
    if args.dbpath is None:
        print "No database specified, trying to connection to default database..."
        # handle world_state database
        world = World()
        msg_store = MessageStoreProxy(database='message_store', collection='ws_observations')
    else:
        addr, port = args.d.split(':')
        # use default database name 'world_state', but change the database address
        world = World(server_host=addr, server_port=port)
        msg_store = MessageStoreProxy(database='message_store', collection='ws_observations')

    # Query
    if args.q is not None:
        if args.r is None:
            room_str = "room not specified"
        else:
            room_str = args.r
        print "Receive query task: " + args.q + ", " + room_str
        field, field_content = args.q.split(':')
        result = world.query_object(field, field_content)
        print "Got "+ str(result.count()) + "  results"

    # Query Point
    if args.p is not None:
        if args.r is None:
            room_str = "room not specified"
        else:
            room_str = args.r

        _x, _y, _z = args.p.split(',')
        _x=float(_x)
        _y=float(_y)
        _z=float(_z)
        result = world.query_object("key", room_str)

        # As a room is unique in the database, so result can only be 0 or 1
        if result.count() == 1:
            print "Got " + str(len(result[0]._bounding_box)) + " bounding box in room " + room_str
            bbx_list = result[0]._bounding_box
            obs_list = result[0]._observations
        else:
            print "No result found!"

        print "Checking coverd observations..."
        reflection = 0
        for th in bbx_list:
            if _x < th[0][0] and _y < th[0][1] and _z < th[0][2] and _x > th[1][0] and _y > th[1][1] and _z > th[1][2]:
                print "Observation at time " + str(obs_list[reflection].stamp) + " covered query point!"
            else:
                print "not in Observation at time " + str(obs_list[reflection].stamp)
            reflection += 1
