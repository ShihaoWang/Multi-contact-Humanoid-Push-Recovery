import sys, os, time
from klampt import *
from klampt import vis
from klampt.vis.glrobotprogram import GLSimulationPlugin
from klampt.model.trajectory import Trajectory, RobotTrajectory
import ipdb
import copy
from scipy.spatial import ConvexHull
import draw_hull
from OpenGL.GL import *
import math
import numpy as np
import random

class MyGLPlugin(vis.GLPluginInterface):
    def __init__(self, world):
        vis.GLPluginInterface.__init__(self)
        self.world = world
        self.quit = False
        self.starp = False

    def mousefunc(self, button, state, x, y):
        print("mouse",button,state,x,y)
        if button==2:
            if state==0:
                print("Click list...",[o.getName() for o in self.click_world(x,y)])
            return True
        return False

    def motionfunc(self, x, y, dx, dy):
        return False

    def keyboardfunc(self, c, x, y):
        print("Pressed", c)
        return True

    def click_world(self, x, y):
        """Helper: returns a list of world objects sorted in order of
        increasing distance."""
        #get the viewport ray
        (s, d) = self.click_ray(x, y)

        #run the collision tests
        collided = []
        for g in self.collider.geomList:
            (hit, pt) = g[1].rayCast(s, d)
            if hit:
                dist = vectorops.dot(vectorops.sub(pt, s), d)
                collided.append((dist,g[0]))
        return [g[1] for g in sorted(collided)]

def my_draw_hull(h):
    glEnable(GL_LIGHTING)
    glEnable(GL_BLEND)
    glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA)
    glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT_AND_DIFFUSE,[1.0,0.25,0.5,0.5])
    draw_hull.draw_hull(h)

def ContactDataUnplot(vis, ReachableContacts_data):
    RowNo, ColumnNo = ReachableContacts_data.shape
    RowStart = 0
    RowEnd = RowNo
    for i in range(RowStart, RowEnd):
        vis.hide("Point_" + str(i))

def ContactDataPlot(vis, ReachableContacts_data):
    RowNo, ColumnNo = ReachableContacts_data.shape
    RowStart = 0
    # RowEnd = min(RowNo, 99)
    RowEnd = RowNo

    for i in range(RowStart, RowEnd):
        point_start = [0.0, 0.0, 0.0]
        ReachableContact_i = ReachableContacts_data[i]
        point_start[0] = ReachableContact_i[0]
        point_start[1] = ReachableContact_i[1]
        point_start[2] = ReachableContact_i[2]

        vis.add("Point_" + str(i), point_start)
        vis.hideLabel("Point_" + str(i), True)
        vis.setColor("Point_" + str(i),65.0/255.0, 199.0/255.0, 244.0/255.0, 1.0)

def ContactDataRefine(ContactPts, ContactWeights_array):
    # This function is used to address the Klampt visualization problem
    ContactPointSize = ContactWeights_array.size/3
    if ContactPointSize>100:
        ContactWeightLists = []
        for i in range(ContactPointSize):
            ContactWeight_i = ContactWeights_array[i]
            ContactWeight_i_value = ContactWeight_i[0]**2 + ContactWeight_i[1]**2 + ContactWeight_i[2]**2
            ContactWeightLists.append(ContactWeight_i_value)
        sorted_indices = sorted(range(len(ContactWeightLists)), key=lambda k: ContactWeightLists[k])
        sorted_indices.reverse()
        ContactPts_ = []
        ContactWeights_ = []
        for i in range(0, 50):
            sorted_index = sorted_indices[i]
            ContactPt_i = ContactPts[sorted_index]
            ContactWeight_i = ContactWeights_array[sorted_index]
            ContactPts_.append(ContactPt_i.tolist())
            ContactWeights_.append(ContactWeight_i.tolist())
        return np.array(ContactPts_), np.array(ContactWeights_)
    else:
        return ContactPts, ContactWeights_array

def WeightedContactDataPlot(vis, OptimalContact_data, OptimalContactWeights_data):
    scale = 1.0
    for i in range(OptimalContact_data.size/3):
        point_start = [0.0, 0.0, 0.0]
        ReachableContact_i = OptimalContact_data[i]
        point_start[0] = ReachableContact_i[0]
        point_start[1] = ReachableContact_i[1]
        point_start[2] = ReachableContact_i[2]

        point_end = [0.0, 0.0, 0.0]
        ReachableContactWeight_i = OptimalContactWeights_data[i]
        point_end[0] = point_start[0] + scale * ReachableContactWeight_i[0]
        point_end[1] = point_start[1] + scale * ReachableContactWeight_i[1]
        point_end[2] = point_start[2] + scale * ReachableContactWeight_i[2]
        print i
        vis.add("PointWeights_" + str(i), Trajectory([0, 1], [point_start, point_end]))
        vis.hideLabel("PointWeights_" + str(i), True)
        vis.setColor("PointWeights_" + str(i), 0.0, 204.0/255.0, 0.0, 1.0)
        vis.setAttribute("PointWeights_" + str(i), 'width', 5.0)

def WeightedContactDataUnPlot(vis, OptimalContact_data):
    for i in range(OptimalContact_data.size/3):
        vis.hide("PointWeights_" + str(i))

def Robot_Config_Plot(world, DOF, config_init):
    robot_viewer = MyGLPlugin(world)
    vis.pushPlugin(robot_viewer)
    vis.add("world", world)
    vis.show()

   # Here we would like to read point cloud for visualization of planning.
    # 1. All Reachable Points
    # IdealReachableContacts_data = ContactDataLoader("IdealReachableContact")
    # # 2. Active Reachable Points
    # ReachableContacts_data = ContactDataLoader("ReachableContacts")
    # # 3. Contact Free Points
    # CollisionFreeContacts_data = ContactDataLoader("CollisionFreeContacts")
    # # # 4. Supportive Points
    # SupportiveContacts_data = ContactDataLoader("SupportiveContacts")

    # OptimalContact_data = ContactDataLoader("OptimalContact")

    # OptimalContactWeights_data = ContactDataLoader("OptimalContactWeights")

    # OptimalContact_data, OptimalContactWeights_data = ContactDataRefine(OptimalContact_data, OptimalContactWeights_data)
    # # 6.
    # TransitionPoints_data = ContactDataLoader("TransitionPoints")
    # import ipdb; ipdb.set_trace()
    # 7.
    # InitialTransitionPoints_data = ContactDataLoader("InitialPathWayPoints")
    # AdjusterWayPoints_data = ContactDataLoader("AdjusterWayPoints")
    # SegmentWayPoints_data = ContactDataLoader("SegmentWayPoints")
    # # 8.
    # ShiftedTransitionPoints_data = ContactDataLoader("ShiftedPathWayPoints")
    #
    # TwoPointLine_data = ContactDataLoader("TwoPointLine")

    TestPathWayPoints_data = ContactDataLoader("FineShiftedPathWayPoints")
    # 9.
    # FineShiftedPathWayPoints_data = ContactDataLoader("FineShiftedPathWayPoints")
    #
    # ReducedOptimalContact_data = ContactDataLoader("ReducedOptimalContact")

    ContactChoice = TestPathWayPoints_data
    # ContactChoice = TwoPointLine_data
    SimRobot = world.robot(0)
    SimRobot.setConfig(config_init)
    while vis.shown():
        # This is the main plot program
        vis.lock()
        SimRobot.setConfig(config_init)
        # WeightedContactDataPlot(vis, OptimalContact_data, OptimalContactWeights_data)
        ContactDataPlot(vis, ContactChoice)
        # for i in range(0, 31):
        #     BBName = "BB" + str(i)
        #     ConvexHull_data = ContactDataLoader(BBName)
        #     h = ConvexHull(ConvexHull_data)
        #     hrender = draw_hull.PrettyHullRenderer(h)
        #     vis.add("ContactPolytope" + str(i), h)
        #     vis.setDrawFunc("ContactPolytope" + str(i), my_draw_hull)

        vis.unlock()
        ipdb.set_trace()
        time.sleep(0.1)
        WeightedContactDataUnPlot(vis, OptimalContact_data)
        ContactDataUnplot(vis, ContactChoice)

def RobotCOMPlot(SimRobot, vis):
    COMPos_start = SimRobot.getCom()
    COMPos_end = COMPos_start[:]
    COMPos_end[2] = COMPos_end[2] - 7.50
    vis.add("COM", Trajectory([0, 1], [COMPos_start, COMPos_end]))
    vis.hideLabel("COM",True)
    vis.setColor("COM", 0.0, 204.0/255.0, 0.0, 1.0)
    vis.setAttribute("COM",'width', 5.0)

def ContactDataGene(ReachableContacts_data):
    RowNo, ColumnNo = ReachableContacts_data.shape
    RowStart = 0
    RowEnd = RowNo

    point = []
    for i in range(RowStart, RowEnd):
        point_start = [0.0, 0.0, 0.0]
        ReachableContact_i = ReachableContacts_data[i]
        point_start[0] = ReachableContact_i[0]
        point_start[1] = ReachableContact_i[1]
        point_start[2] = ReachableContact_i[2]
        point.append(point_start)
    return ContactDataGene



def Configuration_Loader_fn(Config_Name):
    # This function is only used to load in the initial configuraiton
    # The initial file will be in the .config format
    with open(Config_Name,'r') as robot_angle_file:
        robotstate_angle_i = robot_angle_file.readlines()
    config_temp = [x.replace('\t',' ') for x in robotstate_angle_i]
    config_temp = [x.replace('\n','') for x in config_temp]
    config_temp = [float(i) for i in config_temp[0].split()]

    DOF = int(config_temp[0])
    # Config_Init = np.array(config_temp[1:])
    Config_Init = config_temp[1:]
    return DOF, Config_Init

def ContactDataLoader(IdealReachableContact):
    IdealReachableContacts = "../build/" + IdealReachableContact + ".bin"
    f_IdealReachableContacts = open(IdealReachableContacts, 'rb')
    IdealReachableContacts_data = np.fromfile(f_IdealReachableContacts, dtype=np.double)
    IdealReachableContacts_data = IdealReachableContacts_data.reshape((IdealReachableContacts_data.size/3, 3))
    return IdealReachableContacts_data

def main(*arg):
    Robot_Option = "../user/"
    world = WorldModel()                    	# WorldModel is a pre-defined class
    EnviName = "/home/motion/Desktop/Whole-Body-Planning-for-Push-Recovery-Data/flat_1Contact/"
    # The next step is to load in robot's XML file
    XML_path = EnviName + "Envi.xml"
    result = world.readFile(XML_path)         	# Here result is a boolean variable indicating the result of this loading operation
    if not result:
        raise RuntimeError("Unable to load model " + XML_path)
    # In this case, what we have is a config
    CtrlStateTraj = Trajectory(world.robot(0))
    CaseNo = 1
    TestNo = 5
    SpecificPath = EnviName + "/" + str(CaseNo) + "/" + str(TestNo)
    CtrlStateTraj.load(SpecificPath + "/CtrlStateTraj.path")
    import ipdb; ipdb.set_trace()

    Config_Init = CtrlStateTraj.milestones[-1]

    Robot_Config_Plot(world, len(Config_Init), Config_Init)
if __name__ == "__main__":
    main("Config")
