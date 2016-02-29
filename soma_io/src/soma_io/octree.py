import octomap
from geometry_msgs.msg import Pose

class soma_octree(object):

    def __init__(self, oct, res=0.01):
        # init octree and load data
        self.octree = octomap.OcTree(res)
        print self.octree.readBinary(oct)

    def load_tree(self, input_path):
        self.octree.readBinary(input_path)

    def tree_from_db(self,db_query_result):
        i=0
        self.oct_template = '# Octomap OcTree binary file\n'
        self.oct_template += 'id ' + db_query_result[i][0].id + '\n'

        self.oct_template += 'res ' + str(db_query_result[i][0].resolution) + '\n'
        self.oct_template += 'data\n'

    def find_bbx(octree):
        pass

    def get_bbx_info(self, octree):
        print self.octree.getResolution()
        print self.octree.getMetricMax()
        print self.octree.getMetricMin()
        print self.octree.getMetricSize()
        print '===================='

    def point_in_observation(self, observe_tree, *args):
        _min = observe_tree.getMetricMax()
        _max = observe_tree.getMetricMin()
        if len(args) is 3:
            _x = args[0]
            _y = args[1]
            _z = args[2]
            if _min[0] < _x < _max[0] and \
               _min[1] < _y < _max[1] and \
               _min[2] < _z < _max[2]:
                return True
            else:
                return False

        if isinstance(args[0],Pose):
            _x = args[0].position.x
            _y = args[0].position.y
            _z = args[0].position.z
            if _min[0] < _x < _max[0] and \
               _min[1] < _y < _max[1] and \
               _min[2] < _z < _max[2]:
                return True
            else:
                return False
        raise Exception("unknown args type or amount of args.")
