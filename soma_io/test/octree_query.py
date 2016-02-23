#!/usr/bin/env python
import rospy
import octomap

from mongodb_store.message_store import MessageStoreProxy

if __name__ == '__main__':
    #connect to mongodb
    rospy.init_node("query_example")
    msg_store = MessageStoreProxy(database='message_store',collection='observations')

    #messge_store api
    oct_msg_list = msg_store.query_named('octomap_msgs/Octomap')

    #deserialise_message

    #load octree from binary octree
    oct_head = '# Octomap OcTree binary file\n# (feel free to add / change comments, but leave the first line as it ' \
               'is!)\n#\nid OcTree\nsize '+ oct_msg_list[][].+'\nres 0.01\ndata\n'
    oct_data = octomap.OcTree(0.01)
    oct_data.read()
    oct_msg.data


    #query octree

    #load octree form query result

    #check if point is covered

    #return result
