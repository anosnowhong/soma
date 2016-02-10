from world_state_importer.src.soma_octree import FileIO

if __name__ == '__main__':
    ROMBUS_DB = "/Volumes/60G/strands_data_backup"

    eg = FileIO()
    pcd_list = eg.scan_file(ROMBUS_DB, "pcd")
    pcd_list.sort()
    for pcd in pcd_list:
        pc2 = eg.load_pcd(pcd)
        print "start transformation" + pcd
        octree = eg.pc2_to_octree(pc2, 0.01)
        for i in range(len(pcd)):
            x = len(pcd) - (i + 1)
            if pcd[x] == '.':
                bt_name = pcd[:x] + ".bt"
                eg.save_oct(bt_name,octree)
                break


