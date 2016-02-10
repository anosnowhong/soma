from world_state_importer.src import soma_octree

if __name__ == '__main__':
    ROMBUS_DB = "/home/hongru/20140820/patrol_run_2"

    eg = soma_octree.FileIO()
    pcd_list = eg.scan_file(ROMBUS_DB, "pcd")
    pcd_list.sort()
    for pcd in pcd_list:
        pc2 = eg.load_pcd(pcd)
        # convert pcd to octree
        print "Start converting " + pcd
        octree = eg.pcd_to_octree(pc2, 0.01)
        # save octree to bt format
        for i in range(len(pcd)):
            x = len(pcd) - (i + 1)
            if pcd[x] == '.':
                bt_name = pcd[:x] + ".bt"
                eg.save_oct(bt_name,octree)
                break


