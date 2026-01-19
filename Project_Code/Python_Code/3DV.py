#Taimur Ahmed 400514463
#2DX3

import numpy as np
import open3d as o3d

if __name__ == "__main__":
    scans = int(input("How many scans are taken"))
    spins = 16
    pcd = o3d.io.read_point_cloud("point_array.xyz", format="xyz")

    print(np.asarray(pcd.points))
      
    o3d.visualization.draw_geometries([pcd])

    yzx = []
    for x in range(0,scans*spins):
        yzx.append([x])
       
    l = []  
    for x in range(0,scans*spins,spins):
        for i in range(spins):
            if i==spins-1:
                l.append([yzx[x+i], yzx[x]])
            else:
                l.append([yzx[x+i], yzx[x+i+1]])
        
       
    for x in range(0,scans*spins-spins-1,spins):
        for i in range(spins):
            l.append([yzx[x+i], yzx[x+i+spins]])

    line_set = o3d.geometry.LineSet(points=o3d.utility.Vector3dVector(np.asarray(pcd.points)),lines=o3d.utility.Vector2iVector(l))

    o3d.visualization.draw_geometries([line_set])
                                    
    
 
#test
