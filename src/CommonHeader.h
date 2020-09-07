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
std::vector<double>             RobotConfigLoader(const string & user_path, const string & file_name);

/* 1. Main MainDriver */
void                            MainDriver(const bool & SpecifiedFlag, const string & ExperimentFolderPath, int & FileIndex, const SelfCollisionInfo & SelfCollisionInfoObj);
void                            MainInner(string ExperimentFolderPath, int FileIndex, int ExpIndex, SelfCollisionInfo & SelfCollisionInfoObj, SimPara & SimParaObj);

/* 2. Robot Utilities */
int                             FileIndexFinder(bool UpdateFlag, int WriteInt);
void                            PathFileManager(const string & CurrentCasePath);
void                            Vector3Appender(const char *File_Name, double Time_t, const Vector3 & Vector3Obj);
void                            StateTrajAppender(const char *stateTrajFile_Name, const double & Time_t, const std::vector<double> & Configuration);
void                            getCentroidalState(const Robot & SimRobot, Vector3 & COMPos, Vector3 & COMVel);
void                            PushInfoFileAppender(double SimTime, double Fx_t, double Fy_t, double Fz_t, const string & SpecificPath);
std::vector<Vector3>            ActiveContactFinder(const Robot & SimRobot, const std::vector<ContactStatusInfo> & RobotContactInfo);
void                            ContactPolytopeWriter(const std::vector<Vector3> & ActiveContact, const std::vector<PIPInfo> & PIPTotal, const SimPara & SimParaObj);
bool                            FailureChecker(const Robot & SimRobot);
bool                            PenetrationTester(const Robot & SimRobotObj, int SwingLinkInfoIndex);
double                          FailureMetricEval(const std::vector<PIPInfo> & PIPTotal);
void                            StateLogger(WorldSimulation & Sim, LinearPath & CtrlStateTraj, LinearPath & PlanStateTraj, std::vector<double> & qDes, const SimPara & SimParaObj);
bool                            OneHandAlreadyChecker(const ContactForm & ContactFormObj);
void                            PlanTimeRecorder(double PlanTimeVal, const string & CurrentCasePath);
void                            PlanningInfoFileAppender(int PlanStageIndex, int TotalLinkNo, const string & CurrentCasePath, double CurTime);
Vector3                         getEndEffectorTranslationalVelocity(const Robot & SimRobot, int SwingLinkInfoIndex, const Config & JointVelocity);
Vector3                         getEndEffectorAngularVelocity(const Robot & SimRobot, int SwingLinkInfoIndex, const Config & JointVelocity);
Vector3                         getPostionFromCubicCoeffs(double s, const std::vector<Vector3> & CubicCoeffs);
std::vector<double>             LinearSpace(double a, double b, std::size_t N);
void                            Vector3Writer(const std::vector<Vector3> & ContactPoints, const std::string & ContactPointFileName);



/* 3. Simulation */
int                             SimulationWithMethod(WorldSimulation & Sim, const std::vector<ContactStatusInfo> & InitContactInfo, SelfCollisionInfo & SelfCollisionInfoObj, SimPara & SimParaObj);

/* 4. Simulation Related */
LinearPath                      InitialSimulation(WorldSimulation & Sim, const SimPara & SimParaObj);
void                            PushImposer(WorldSimulation & Sim, double CurTime, const SimPara & SimParaObj, bool FailureFlag);

/* 5. Convex Polytope */
std::vector<PIPInfo>            PIPGenerator( const Vector3 & COMPos, const Vector3 & COMVel, const std::vector<Vector3> & ContactPoints);
PIPInfo                         TipOverPIPGenerator(const Vector3 & COMPos, const Vector3 & COMVel, const std::vector<Vector3> & ActiveContacts, bool & ValidFlag);

std::vector<ContactForm>        getCandidateContactStatus(const Robot & SimRobot, const std::vector<ContactStatusInfo> & RobotContactInfo);
std::vector<Vector3>            OptimalContactSearcher(const Robot & SimRobot, const PIPInfo & PIPObj, const ContactForm & ContactFormObj, SimPara & SimParaObj);

/* 6. Recovery Reference */

RecoveryReferenceInfo           RecoveryReferenceComputation(   const Robot & SimRobot,
                                                                const std::vector<ContactStatusInfo> & curRobotContactInfo,
                                                                SelfCollisionInfo & SelfCollisionInfoObj,
                                                                SimPara & SimParaObj);
RecoveryReferenceInfo           RecoveryReferenceComputationInner(  const Robot & SimRobot,                           const PIPInfo & TipOverPIPObj, 
                                                                    SelfCollisionInfo & SelfCollisionInfoObj,   const ContactForm & ContactFormObj, 
                                                                    SimPara & SimParaObj);                                                    
/* 7. Path Computation */
CubicSplineInfo                 EndEffectorPathComputation(const Robot & SimRobot, const SelfCollisionInfo & SelfCollisionInfoObj, SimPara & SimParaObj);
void                            EndEffectorPathSlopeComputation(const Robot & SimRobot, int SwingLinkInfoIndex, Vector3 & PathInitSlope, Vector3 & PathEndSlope, const SimPara & SimParaObj);
void                            InitialWayPointsComputation(const Robot & SimRobot, int SwingLinkInfoIndex, const SimPara & SimParaObj, std::vector<Vector3> & InitWayPoints, std::vector<double> & sVec);
std::vector<Vector3>            InitialWayPointsShifter(const std::vector<Vector3> & WayPoints, const int & SwingLinkInfoIndex, const SelfCollisionInfo & SelfCollisionInfoObj, bool & ShifterFlag);
CubicSplineInfo                 CubicSplineInfoObjComputation(const Robot & SimRobot, const int & SwingLinkInfoIndex, const SelfCollisionInfo & SelfCollisionInfoObj, const SimPara & SimParaObj);

#endif
