#ifndef COMMON_HEADER_H
#define COMMON_HEADER_H
#include <iostream>
#include <limits.h>
#include <string>
#include <sys/stat.h>
#include <unistd.h>
#include <string>
#include <ctime>
#include <KrisLibrary/geometry/Conversions.h>
#include <KrisLibrary/geometry/MultiVolumeGrid.h>
#include <KrisLibrary/geometry/CollisionMesh.h>
#include <KrisLibrary/geometry/AnyGeometry.h>
#include <KrisLibrary/meshing/VolumeGrid.h>
#include <KrisLibrary/meshing/PointCloud.h>
#include <KrisLibrary/math3d/rotation.h>
#include "RobotInfo.h"
#include "Spline.h"
#include <Eigen/QR>    
#include <Eigen/Geometry> 


/* 0. Robot Info Initiaization */
std::vector<LinkInfo>           LinkInfoLoader(string ContactLinkFile);
std::vector<ContactStatusInfo>  ContactStatusInfoLoader(const string & ContactStatusFile);
std::vector<int>                LinkIndicesLoader(const string & UserFilePath, const string & FileName);
SDFInfo                         SDFInfoObjInit(const string & ExperimentFolderPath, const RobotWorld & worldObj);
ReachabilityMap                 ReachabilityMapInit(const Robot& SimRobot, const std::vector<LinkInfo> & LinkInfoObj, const std::vector<int> & TorsoLinkIndices);
AnyCollisionGeometry3D          TerrColGeomObjInit(const RobotWorld & worldObj);
std::vector<double>             getSimParaVec();

/* 1. Main MainDriver */
void                            MainDriver(const bool & SpecifiedFlag, const string & ExperimentFolderPath, int & FileIndex, const SelfCollisionInfo & SelfCollisionInfoObj);
void                            MainInner(string ExperimentFolderPath, int FileIndex, int ExpIndex, SelfCollisionInfo & SelfCollisionInfoObj, SimPara & SimParaObj);

/* 2. Robot Utilities */
int                             FileIndexFinder(bool UpdateFlag, int WriteInt);

/*  */

#endif
