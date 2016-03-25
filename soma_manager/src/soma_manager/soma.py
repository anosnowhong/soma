#!/usr/bin/env python

import roslib
from soma_io.state import Observation
from soma_io.state import World

roslib.load_manifest("soma_manager")
import rospy
from rospkg import RosPack
import json
import argparse
import random
import copy
import sys
from soma_io.mongo import MongoConnection, MongoTransformable, MongoDocument

from threading import Timer

from mongodb_store.message_store import MessageStoreProxy
from soma_geospatial_store.geospatial_store import GeoSpatialStoreProxy
from visualization_msgs.msg import Marker, InteractiveMarkerControl
from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from geometry_msgs.msg import Pose

from soma_msgs.msg import SOMAObject
from bson.objectid import ObjectId

def trapezoidal_shaped_func(a, b, c, d, x):
    min_val = min(min((x - a)/(b - a), float(1.0)), (d - x)/(d - c))
    return max(min_val, float(0.0))


def r_func(x):
    a = -0.125
    b =  0.125
    c =  0.375
    d =  0.625

    x = 1.0 - x
  
    value = trapezoidal_shaped_func(a,b,c,d,x)
    return value

def g_func(x):
    a =  0.125
    b =  0.375
    c =  0.625
    d =  0.875
    
    x = 1.0 - x
    
    value = trapezoidal_shaped_func(a,b,c,d,x)
    return value


def b_func(x):

    a =  0.375
    b =  0.625
    c =  0.875
    d =  1.125
  
    x = 1.0 - x
  
    value = trapezoidal_shaped_func(a,b,c,d,x)
    return value


class Object(MongoDocument):
    def __init__(self, mongo=None):
        super(Object, self).__init__()
        if mongo is not None:
            # this will create the document live..
            self._connect(mongo)
        self._children = []
        self._parent = None
        self._bounding_box = None #BBoxArray(bbox)
        self._observations = None

        self._life_start = rospy.Time.now().to_time()
        self._life_end = None

        self.identifications = {}
        self.identification = ObjectIdentification()

        self._msg_store_objects =  [] # a list of object IDs (strings)

        self._observations =  [] # a list of observation objects

        self._poses = []
        self._point_cloud = None # will be a MessageStoreObject or None

        self._spans = [] # for storing life spans as tuples (start,end)

    @property
    def pose(self):
        if len(self._poses) < 1:
            raise StateException("NOPOSE")
        return copy.deepcopy(self._poses[-1])

    @property
    def position(self):
        if len(self._poses) < 1:
            raise StateException("NOPOSE")
        return copy.deepcopy(self._poses[-1].position)

    @property
    def quaternion(self):
        if len(self._poses) < 1:
            raise StateException("NOPOSE")
        return copy.deepcopy(self._poses[-1].quaternion)

    @property
    def pose_homog_transform(self):
        if len(self._poses) < 1:
            raise StateException("NOPOSE")
        return self._poses[-1].as_homog_matrix

    def cut(self, stamp=None):
        """
        Marks the end of the life of this object, adding an entry for its life
        span.
        """
        if self._life_end is not None:
            return
        if stamp is not None:
            self._life_end = stamp
        else:
            self._life_end = rospy.Time.now().to_time()
        self._spans.append((self._life_start, self._life_end))
        self._spans = self._spans

    def cut_all_children(self):
        world =  World()
        children = world.get_children(self.name, {'_life_end': None,})
        for i in children:
            i.cut()

    def set_life_start(self, start):
        self._life_start = start
        self._life_end = None

    @property
    def name(self):
        return self.key

    @name.setter
    def name(self, name):
        # TODO: check doesn't already exist
        self.key = name


    def add_identification(self, classifier_id, identification):
        if not self.identifications.has_key(classifier_id):
            self.identifications[classifier_id] = []
        # TODO check time consistency
        self.identifications[classifier_id].append(identification)
        self.identifications = self.identifications #force mongo update
        # TODO: don't duplicate? include classifier_id?
        self.identification = identification

    def add_pose(self, pose):
        # TODO check time consistency
        p = copy.deepcopy(pose)
        self._poses.append(p) #[str(p)] = p
        self._poses = self._poses #force mongo update

    def add_observation(self, observation):
        assert isinstance(observation,  Observation)
        self._observations.append(observation)
        self._observations =  self._observations

    def add_msg_store(self, message):
        assert isinstance(message, MessageStoreObject)
        self._msg_store_objects.append(message)
        self._msg_store_objects = self._msg_store_objects

    def get_identification(self, classifier_id=None):
        if classifier_id is None:
            return self.identification
        return self.identifications[classifier_id][-1]

    ##the value of _parent need to be decided
    def get_parent(self):
        world =  World()
        ##FIX the error in unittest that can't find the object
        ##self._parent = self.key

        parent = world.get_object(self._parent)
        return parent

    ##usually, when remove a object we need find the parent a call remove child
    def remove_child(self, child_name):
        try:
            self._children.remove(child_name)
        except:
            raise Exception("Trying to remove child from object, but"
                            "parent object is not parent of this child.")

    def get_children_names(self):
        return copy.copy(self._children)

    def add_child(self, child_object):
        #self._children.append(child_object.get_name)
        # have to recreate to catch in setattr
        assert child_object._parent is None # otherwise dual parentage is ok?
        child_object._parent = self.name
        self._children+=[child_object.name]

    def get_message_store_messages(self, typ=None):
        msgs = []
        proxy = MessageStoreProxy()
        for msg in self._msg_store_objects:
            if typ != msg.typ and typ is not None:
                continue
            proxy.database =  msg.database
            proxy.collection =  msg.collection
            msgs.append(proxy.query_id(msg.obj_id, msg.typ)[0])
        return msgs

    @classmethod
    def _mongo_encode(cls, class_object):
        doc = {}
        doc.update(class_object.__dict__)
        try:
            doc.pop("_MongoDocument__mongo")
            doc.pop("_MongoDocument__connected")
        except KeyError:
            print "Warning: no no no"
        doc["__pyobject_class_type"] = class_object.get_pyoboject_class_string()
        doc = copy.deepcopy(doc)
        return doc



class SOMAManager():

    def __init__(self, soma_map, soma_conf, config_file=None):

        self.soma_map = soma_map
        self.soma_conf = soma_conf
        if config_file:
            self._config_file = config_file
        else:
            # default file
            rp = RosPack()
            path = rp.get_path('soma_objects') + '/config/'
            filename = 'default.json'
            self._config_file=path+filename
        self._soma_obj_ids = dict()
        self._soma_obj_msg = dict()
                
        self._interactive = True

        self._msg_store=MessageStoreProxy(collection="soma")

        self._gs_store=GeoSpatialStoreProxy(db="geospatial_store", collection="soma")
        
        self._server = InteractiveMarkerServer("soma")

        self._init_types()

        self._init_menu()
        
        self.load_objects()

        rospy.spin()


    def _init_types(self):
        # read from config in soma_objects 
        
        with open(self._config_file) as config_file:
            config = json.load(config_file)

            self.mesh = dict()
            self.marker = dict()
            if '2D' in config:
                for k, v in config['2D'].iteritems():
                    self.mesh[k] = v
                    self.marker[k] = '2D'

            if '3D' in config:
                for k, v in config['3D'].iteritems():
                    self.mesh[k] = v
                    self.marker[k] = '3D'

    def _init_menu(self):

        self.menu_handler = MenuHandler()
        add_entry = self.menu_handler.insert( "Add object" )

        self.menu_item = dict()
        for k in sorted(self.mesh.keys()):
            entry =  self.menu_handler.insert(k, parent=add_entry, callback=self._add_cb)
            self.menu_item[entry] = k
            
        del_entry =  self.menu_handler.insert( "Delete object", callback=self._del_cb)

        enable_entry = self.menu_handler.insert( "Movement control", callback=self._enable_cb )

        self.menu_handler.setCheckState( enable_entry, MenuHandler.CHECKED )

    def _add_cb(self, feedback):
        rospy.loginfo("Add marker: %s", self.menu_item[feedback.menu_entry_id])
        self.add_object(self.menu_item[feedback.menu_entry_id], feedback.pose)

    def _del_cb(self, feedback):
        rospy.loginfo("Delete marker: %s", feedback.marker_name)
        self.delete_object(feedback.marker_name)        

    def _update_cb(self, feedback):
        p = feedback.pose.position
        print "Marker " + feedback.marker_name + " position: " + str(round(p.x,2)) + ", " + str(round(p.y,2)) +  ", " + str(round(p.z,2))
        
        if hasattr(self, "vp_timer_"+feedback.marker_name):
            getattr(self, "vp_timer_"+feedback.marker_name).cancel()        
        setattr(self, "vp_timer_"+feedback.marker_name,
                Timer(3, self.update_object, [feedback]))
        getattr(self, "vp_timer_"+feedback.marker_name).start()        
        
    def _enable_cb(self, feedback):
        handle = feedback.menu_entry_id
        state = self.menu_handler.getCheckState( handle )

        if state == MenuHandler.CHECKED:
            self.menu_handler.setCheckState( handle, MenuHandler.UNCHECKED )
            self._interactive = False
        else:
            self.menu_handler.setCheckState( handle, MenuHandler.CHECKED )
            self._interactive = True

        self.menu_handler.reApply( self._server )

        self.load_objects()
        
        self._server.applyChanges()
        
    def _next_id(self):
        self._soma_id += 1
        return self._soma_id

    def _retrieve_objects(self):

        objs = self._msg_store.query(SOMAObject._type, message_query={"map": self.soma_map,
                                                                      "config": self.soma_conf})

        max_id = 0
        for o,om in objs:
            if int(o.id) > max_id:
                max_id = int(o.id)
        self._soma_id = max_id
        
        return objs
    
    def load_objects(self):

        objs = self._retrieve_objects()

        # if collection is empty insert initial object
        if not objs:
            pose = Pose()
            self.add_object('Table', pose)
            return

        # otherwise, load all object from collection
        for o, om  in objs:
            self._soma_obj_ids[o.id] = om['_id']
            self._soma_obj_msg[o.id] = o
            self.load_object(o.id, o.type, o.pose)


    def load_object(self, soma_id, soma_type, pose):

        int_marker = self.create_object_marker(soma_id, soma_type, pose)
        
        self._server.insert(int_marker, self._update_cb)

        self.menu_handler.apply( self._server, soma_id )

        self._server.applyChanges()

    def add_object(self, soma_type, pose):
        soma_id = self._next_id()

        soma_obj = SOMAObject()
        soma_obj.id = str(soma_id)
        soma_obj.map = str(self.soma_map)
        soma_obj.config = str(self.soma_conf)
        soma_obj.type = soma_type
        soma_obj.pose = pose
        soma_obj.pose.position.z = 0.0
        soma_obj.frame = 'map'
        soma_obj.mesh = self.mesh[soma_type]

        _id = self._msg_store.insert(soma_obj)
        self._soma_obj_ids[soma_obj.id] = _id
        self._soma_obj_msg[soma_obj.id] = soma_obj

        # add object to geo_spatial store
        self._gs_store.insert(self.geo_json_from_soma_obj(soma_obj))
        print "GS Store: added obj"
        
        self.load_object(str(soma_id), soma_type, soma_obj.pose)

    def geo_json_from_soma_obj(self, soma_obj):

        geo_json = {}
        geo_json['soma_id'] = soma_obj.id
        geo_json['soma_map'] = soma_obj.map
        geo_json['soma_config'] = soma_obj.config
        geo_json['type'] = soma_obj.type
        geo_json['loc'] = {'type': 'Point',
                           'coordinates': self._gs_store.coords_to_lnglat(soma_obj.pose.position.x,
                                                                          soma_obj.pose.position.y)}
        return geo_json

    def delete_object(self, soma_id):

        # geospatial store
        res = self._gs_store.find_one({'soma_id': soma_id,
                                       'soma_map': self.soma_map,
                                       'soma_config': self.soma_conf})
        if res:
            _gs_id = res['_id']
            self._gs_store.remove(_gs_id)
            print "GS Store: deleted obj"
                        

        # message store
        _id = self._soma_obj_ids[str(soma_id)]
        self._msg_store.delete(str(_id))
        
        self._server.erase(soma_id)
        self._server.applyChanges()
        
    def update_object(self, feedback):
        print "Updated marker " + feedback.marker_name

        _id = self._soma_obj_ids[feedback.marker_name]
        msg = self._soma_obj_msg[feedback.marker_name]

        new_msg = copy.deepcopy(msg)
        new_msg.pose = feedback.pose

        self._msg_store.update_id(_id, new_msg)

        # geospatial store
        # delete old message
        res = self._gs_store.find_one({'soma_id': new_msg.id,
                                       'soma_map': self.soma_map,
                                       'soma_config': self.soma_conf})
        if res:
            _gs_id = res['_id']
            self._gs_store.remove(_gs_id)
            print "GS Store: deleted obj"            

        # add new object to geospatial store
        self._gs_store.insert(self.geo_json_from_soma_obj(new_msg))
        print "GS Store: added obj"

    def create_object_marker(self, soma_obj, soma_type, pose):
        # create an interactive marker for our server
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "map"
        int_marker.name = soma_obj
        int_marker.description = "id" + soma_obj
        int_marker.pose = pose
        int_marker.pose.position.z = 0.01 
        
        mesh_marker = Marker()
        mesh_marker.type = Marker.MESH_RESOURCE
        mesh_marker.scale.x = 1
        mesh_marker.scale.y = 1
        mesh_marker.scale.z = 1

        random.seed(soma_type)
        val = random.random()
        mesh_marker.color.r = r_func(val)
        mesh_marker.color.g = g_func(val)
        mesh_marker.color.b = b_func(val)
        mesh_marker.color.a = 1.0
        #mesh_marker.pose = pose
        mesh_marker.mesh_resource = self.mesh[soma_type]

        # create a control which will move the box
        # this control does not contain any markers,
        # which will cause RViz to insert two arrows
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.interaction_mode = InteractiveMarkerControl.MOVE_ROTATE

        
        if self._interactive:
            int_marker.controls.append(copy.deepcopy(control))
            # add the control to the interactive marker
            if self.marker[soma_type] == '3D':
                control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
                int_marker.controls.append(control)

        # add menu control
        menu_control = InteractiveMarkerControl()

        menu_control.interaction_mode = InteractiveMarkerControl.BUTTON
        menu_control.always_visible = True
        
        menu_control.markers.append( mesh_marker) #makeBox(int_marker) )
        int_marker.controls.append(menu_control)

        return int_marker
        

if __name__=="__main__":

    # TODO: add list command
    
    parser = argparse.ArgumentParser(prog='soma.py')
    parser.add_argument("map", nargs=1, help='Name of the used 2D map')
    parser.add_argument("conf", nargs=1, help='Name of the object configuration')
    parser.add_argument('-t', metavar='config-file')
                    
    args = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])
    
    rospy.init_node("soma_obj")
    rospy.loginfo("Running SOMA (map: %s, conf: %s, types: %s)", args.map[0], args.conf[0], args.t)
    SOMAManager(args.map[0], args.conf[0],args.t)
    


