import octomap

class soma_octree(octomap):


    def __init__(self, res=0.01):
        self.octree = octomap.OcTree(res)

    def load_tree(self, input_path):
        self.octree.readBinary(input_path)
