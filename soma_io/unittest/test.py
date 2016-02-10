import unittest
from soma_io.


class TestSomaIO(unittest.TestCase):

    def test_remove_nan(self):
        pc_in = pcl.load("example.pcd")
        assert pc_in.size
        pc_out = pcl.PointCloud()
        FileIO.remove_nan(None, pc_in, pc_out)
        #log.info("output point cloud size: %d", pc_out.size)

if __name__ == "__main__":
    unittest.main()
