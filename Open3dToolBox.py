"""
tool box to display with open3d

"""
import unittest
#http://www.open3d.org/docs/latest/tutorial/Advanced/voxelization.html
#https://stackoverflow.com/questions/60104854/how-to-your-load-3d-model-obj-in-open3d
#sudo apt-get install software-properties-common
#sudo add-apt-repository ppa:kisak/kisak-mesa
#sudo apt update
#sudo apt upgrade
#glxinfo | grep version
import numpy as np

try :
    import open3d as o3d #pip install open3d
except :
    print ("please try pip install open3d or conda install -c open3d-admin open3d")
    raise (1)
from scipy.spatial.transform import Rotation as R
#from matplotlib import cm
#import matplotlib


class Open3dToolBox():
    def __init__(self,vis=None):
        if vis is not None:
            self.vis = vis#
            
        else:
            self.vis =None
        self.line_set={}#dictionnaire avec les elements graphiques a actualiser

    def initVisualiser(self):
        self.vis=o3d.visualization.Visualizer()
        self.vis.create_window()
        originFrame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0, origin=[0, 0, 0])
        self.vis.add_geometry(originFrame)

    def updateRenderer(self):
        if self.vis is not None:
            self.vis.poll_events()
            self.vis.update_renderer()
            
    

    def addPointCloud(self,pointCLoudNumpy, voxel_size=None, objectName="dummywithoutcolor"):
        """ rajoute un nuage de point, le sous echantilonne si voxel_size est defini"""
        if self.vis is not None :
            if  objectName not in self.line_set :
                self.line_set[objectName]=o3d.geometry.PointCloud()
                self.line_set[objectName].points = o3d.utility.Vector3dVector(pointCLoudNumpy)  # set pcd_np as the point cloud points
                if voxel_size is not None :
                    self.line_set[objectName] =  self.line_set[objectName].voxel_down_sample(voxel_size=0.05)
                    self.vis.add_geometry(self.line_set[objectName])
                else:
                    self.vis.add_geometry(self.line_set[objectName])
            else:
                dummy = o3d.geometry.PointCloud()
                dummy.points = o3d.utility.Vector3dVector(pointCLoudNumpy) 
                if voxel_size is not None :
                    dummy=dummy.voxel_down_sample(voxel_size=0.05)
                self.line_set[objectName].points = dummy.points 

                self.vis.update_geometry(self.line_set[objectName])
            return True
            """
            pcd_o3d = o3d.geometry.PointCloud()  # create point cloud object
            pcd_o3d.points = o3d.utility.Vector3dVector(pointCLoudNumpy) 
            if voxel_size is not None :
                voxel_down_pcd = pcd_o3d.voxel_down_sample(voxel_size=0.05)
                #subsample
                self.vis.add_geometry(voxel_down_pcd)
                return True
            else:
                self.vis.add_geometry(pcd_o3d)
                return True
            """

    def addPointCloudColorizeed(self,pointCLoudNumpy, colorsNumpy, objectName="dummywithcolor",voxel_size=None, SOR_NeigbordAndradiusList=None):
        """ rajoute un nuage de point colorize"""
        if self.vis is not None :
            if  objectName not in self.line_set :
                self.line_set[objectName]=o3d.geometry.PointCloud()
                self.line_set[objectName].points = o3d.utility.Vector3dVector(pointCLoudNumpy)  # set pcd_np as the point cloud points
                self.line_set[objectName].colors = o3d.utility.Vector3dVector(colorsNumpy)#pcd_o3d.colors =
                
                if SOR_NeigbordAndradiusList is not None :
                    cl, ind =  self.line_set[objectName].remove_radius_outlier(nb_points=SOR_NeigbordAndradiusList[0],radius=SOR_NeigbordAndradiusList[1])
                    self.line_set[objectName]= self.line_set[objectName].select_by_index(ind)
                if voxel_size is not None :
                    self.line_set[objectName] =  self.line_set[objectName].voxel_down_sample(voxel_size=voxel_size)
                

                self.vis.add_geometry(self.line_set[objectName])
            else:
                dummy = o3d.geometry.PointCloud()
                dummy.points = o3d.utility.Vector3dVector(pointCLoudNumpy)
                dummy.colors = o3d.utility.Vector3dVector(colorsNumpy)

                #self.line_set[objectName].points = o3d.utility.Vector3dVector(pointCLoudNumpy)  # set pcd_np as the point cloud points
                #self.line_set[objectName].colors = o3d.utility.Vector3dVector(colorsNumpy)#pcd_o3d.colors =
                if SOR_NeigbordAndradiusList is not None :
                    cl, ind =  dummy.remove_radius_outlier(nb_points=SOR_NeigbordAndradiusList[0],radius=SOR_NeigbordAndradiusList[1])
                    dummy = dummy.select_by_index(ind)
                if voxel_size is not None :
                    dummy=dummy.voxel_down_sample(voxel_size=voxel_size)
                self.line_set[objectName].points = dummy.points
                self.line_set[objectName].colors = dummy.colors

                #self.vis.update_geometry(self.line_set[objectName])
            print ("self.line_set[objectName]"+str(self.line_set[objectName]))
                    
            
                    

    def displayTriedre(self,HList,objectNameList,triedreAxeSize=0.07):
        """affiche un triedre et retourne le nombe de triedre rajoutes"""
        if self.vis is not None :
           
            #viridis =  matplotlib.colormaps.get_cmap('viridis')#, len(HList))
            #cm.get_cmap('viridis', len(HList))
            
            triedreIndex=0
            for H in HList:
                #listofPose=arucoDict[arucoKeys]
                
                #triedreAxeSize=0.2
                pointToMove=np.array([0,0,0,1.0,  triedreAxeSize,0.0,0.0,1.0,  0.00,triedreAxeSize,0.0,1.0,  0.00,0.00,triedreAxeSize,1.0   ]).reshape((-1,4))#pointx x y z 1 
                
                
                listofPointTriedre=[]
                indexOfPoint=0
                listofPointTriedreColor=[]
                listofPointTriedreLine=[]
                
                lisotOfCOlorPointCentral=[]
                lisotOfCOlorPointCentralColor=[]
               
                        
                origin= np.dot(H,pointToMove[0,:])
                xaxis= np.dot(H,pointToMove[1,:])
                yaxis= np.dot(H,pointToMove[2,:])
                zaxis= np.dot(H,pointToMove[3,:])
                    
                #origin
                listofPointTriedre.append( [ origin[0],origin[1],origin[2]])
                #listofPointTriedreColor.append( [ 0,0,0])
                
                listofPointTriedre.append( [ xaxis[0],xaxis[1],xaxis[2]])
                listofPointTriedreColor.append( [ 255.0,0,0])
                    
                listofPointTriedreLine.append([indexOfPoint,indexOfPoint+1])#ligne origin to x
                    
                    
                    
                listofPointTriedre.append( [ yaxis[0],yaxis[1],yaxis[2]])
                listofPointTriedreColor.append( [ 0.0,255.0,0])
                listofPointTriedreLine.append([indexOfPoint,indexOfPoint+2])#ligne orogin to y
                    
                listofPointTriedre.append( [ zaxis[0],zaxis[1],zaxis[2]])
                listofPointTriedreColor.append( [ 0.0,0,255.0])
                listofPointTriedreLine.append([indexOfPoint,indexOfPoint+3])#ligne origin toz
                    
            
                    
                indexOfPoint+=4
            

                if  objectNameList[triedreIndex] not in self.line_set :
                    self.line_set[objectNameList[triedreIndex]] = o3d.geometry.LineSet()
                    self.line_set[objectNameList[triedreIndex]].points = o3d.utility.Vector3dVector(listofPointTriedre)
                    self.line_set[objectNameList[triedreIndex]].lines = o3d.utility.Vector2iVector(listofPointTriedreLine)
                    self.line_set[objectNameList[triedreIndex]].colors = o3d.utility.Vector3dVector(listofPointTriedreColor)
                    self.vis.add_geometry(self.line_set[objectNameList[triedreIndex]])#triedres
                    
                else:
                    self.line_set[objectNameList[triedreIndex]].points = o3d.utility.Vector3dVector(listofPointTriedre)
                    self.line_set[objectNameList[triedreIndex]].lines = o3d.utility.Vector2iVector(listofPointTriedreLine)
                    self.line_set[objectNameList[triedreIndex]].colors = o3d.utility.Vector3dVector(listofPointTriedreColor)
                    self.vis.update_geometry(self.line_set[objectNameList[triedreIndex]])
                
                
                triedreIndex+=1
            return triedreIndex


 
class Open3dToolBoxTest(unittest.TestCase):
    def test_instanciate(self):
        """simple test pour verifier l'instanciation"""
        vis=o3d.visualization.Visualizer()
        vis.create_window()
        originFrame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0, origin=[0, 0, 0])
        vis.add_geometry(originFrame)


        open3dToolBox  = Open3dToolBox(vis)

    def test_addtriedre(self):
        """test de rajout de triedre"""
        
        vis=o3d.visualization.Visualizer()
        vis.create_window()
        originFrame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0, origin=[0, 0, 0])
        vis.add_geometry(originFrame)


        open3dToolBox  = Open3dToolBox(vis)
        object1=np.eye(4)
        object1[0,3]=1.0
        object1[1,3]=1.0

        object2=np.eye(4)
        object2[0,3]=2.0
        object2[1,3]=2.0
        
        numberDisplayed =open3dToolBox.displayTriedre([object1,object2],["object1","object2"], 0.1)
        print ("numberDisplayed="+str(numberDisplayed))
        open3dToolBox.updateRenderer()
        self.assertEqual(numberDisplayed, 2.0)#check position
        
    def test_addPointCloud(self):

        vis=o3d.visualization.Visualizer()
        vis.create_window()
        originFrame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0, origin=[0, 0, 0])
        vis.add_geometry(originFrame)


        open3dToolBox  = Open3dToolBox(vis)

        #point cloud

        xMax=100

        pointCloud=np.zeros((xMax*xMax,3))
        pointIndex=0
        for i in range(xMax):
            for j in range(xMax):
                pointCloud[pointIndex,0]= i
                pointCloud[pointIndex,1]= j
                pointCloud[pointIndex,2]= j*i

                pointIndex+=1

        status=open3dToolBox.addPointCloud(pointCloud)#no subsampling
        status=open3dToolBox.addPointCloud(pointCloud)#no subsampling
        open3dToolBox.updateRenderer()
        print ("addPointCloud status "+str(status))
        self.assertEqual(status, True)#check status

        pointCloud[:,2]+=1000
        status=open3dToolBox.addPointCloud(pointCloud, voxel_size=0.05)#no subsampling
        open3dToolBox.updateRenderer()
        print ("addPointCloud status "+str(status))
        self.assertEqual(status, True)#check status

if __name__ =='__main__':
    import time
    #creation d'un visualisateur
    vis=o3d.visualization.Visualizer()
    vis.create_window()
    originFrame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0, origin=[0, 0, 0])
    vis.add_geometry(originFrame)


    open3dToolBox  = Open3dToolBox(vis)
    object1=np.eye(4)
    object1[0,3]=1.0
    object1[1,3]=1.0

    object2=np.eye(4)
    object2[0,3]=2.0
    object2[1,3]=2.0
    
    numberDisplayed =open3dToolBox.displayTriedre([object1,object2],["object1","object2"], 0.1)
    print ("numberDisplayed="+str(numberDisplayed))

    open3dToolBox.updateRenderer()

    #point cloud
    xMax=100

    pointCloud=np.zeros((xMax*xMax,3))
    pointIndex=0
    for i in range(xMax):
        for j in range(xMax):
            pointCloud[pointIndex,0]= i
            pointCloud[pointIndex,1]= j
            pointCloud[pointIndex,2]= j*i

            pointIndex+=1

    status=open3dToolBox.addPointCloud(pointCloud)#no subsampling
    open3dToolBox.updateRenderer()
    print ("addPointCloud status "+str(status))


    pointCloud[:,2]+=1000
    status=open3dToolBox.addPointCloud(pointCloud, voxel_size=0.05)#no subsampling
    status=open3dToolBox.addPointCloud(pointCloud, voxel_size=0.05)#no subsampling
    
    print ("addPointCloud status "+str(status))
    while True:
        open3dToolBox.updateRenderer()