#include "CommonHeader.h"
#include "NonlinearOptimizerInfo.h"

/* Based on
    "Closed-Loop Manipulator Control Using Quaternion Feedback" Joseph Yuan 1987
 */
static double Kp = 0.0;        // Position Gain
static double Ko = 5.0;        // Orientation Gain

std::vector<double> CartesianController(const Robot & SimRobot, const ControlReferenceInfo  & ControlReference, double InnerTime, double TimeStep){
    int SwingLinkInfoIndex = ControlReference.getSwingLinkInfoIndex();
    int RobotLinkIndex = NonlinearOptimizerInfo::RobotLinkInfo[SwingLinkInfoIndex].LinkIndex;
    
    Vector3 EndEffectorLocalPos = NonlinearOptimizerInfo::RobotLinkInfo[SwingLinkInfoIndex].AvgLocalContact;
    Vector3 EndEffectorGlobalPos;
    SimRobot.GetWorldPosition(EndEffectorLocalPos, RobotLinkIndex, EndEffectorGlobalPos);
    Matrix FullJac;
    SimRobot.GetFullJacobian(EndEffectorLocalPos, RobotLinkIndex, FullJac);

    Eigen::MatrixXd FullJacMat(FullJac.numRows(), FullJac.numCols());
    for (int i = 0; i < FullJac.numRows(); i++){
        for (int j = 0; j < FullJac.numCols(); j++){
            FullJacMat(i,j) = FullJac(i,j);
        }        
    }
    Eigen::MatrixXd FullJacMatInv = FullJacMat.completeOrthogonalDecomposition().pseudoInverse();
    
    // For position
    Vector EndEffectorGlobalPosRefVec, EndEffectorGlobalVelocityRefVec;
    ControlReference.EndEffectorTraj.Eval(InnerTime, EndEffectorGlobalPosRefVec);
    ControlReference.EndEffectorTraj.Deriv(InnerTime, EndEffectorGlobalVelocityRefVec);

    Vector3 EndEffectorGlobalPosRef(1.0, 1.0, 1.0);
    Vector3 EndEffectorGlobalVelocityRef(EndEffectorGlobalVelocityRefVec);
    
    // Vector3 PosOffset = EndEffectorGlobalVelocityRef - Kp * (EndEffectorGlobalPos - EndEffectorGlobalPosRef);
    Vector3 PosOffset = -Kp * (EndEffectorGlobalPos - EndEffectorGlobalPosRef);
// 
    // For orientation
    QuaternionRotation EndEffectorQuat = getEndEffectorQuaternion(SimRobot, SwingLinkInfoIndex);
    QuaternionRotation EndEffectorQuatRef = QuaternionRotationReference(InnerTime, ControlReference);

    // q1 desired
    double eta1 = EndEffectorQuatRef.data[0];
    double q1_x = EndEffectorQuatRef.data[1];
    double q1_y = EndEffectorQuatRef.data[2];
    double q1_z = EndEffectorQuatRef.data[3];

    // q2 actual
    double eta2 = EndEffectorQuat.data[0];
    double q2_x = EndEffectorQuat.data[1];
    double q2_y = EndEffectorQuat.data[2];
    double q2_z = EndEffectorQuat.data[3];

    Vector3 q1(q1_x, q1_y, q1_z);
    Vector3 q2(q2_x, q2_y, q2_z);

    Vector3 delta_q = eta1 * q2 - eta2 * q1 - cross(q1, q2);
    std::cout<<delta_q<<endl;
    Vector3 OriOffset = -Ko * delta_q;

    Eigen::VectorXd RHS(6);
    for (int i = 0; i < 3; i++){
        RHS[i] = OriOffset[i];
        RHS[i+3] = PosOffset[i];
    }

    Eigen::VectorXd qdotDes = FullJacMatInv * RHS;
    std::vector<double> qDes(SimRobot.q.size());
    for (int i = 0; i < SimRobot.q.size(); i++){
        qDes[i] = SimRobot.q[i] + qdotDes[i] * TimeStep;
        if(i>5){
            if(qDes[i]<=SimRobot.qMin(i)) qDes[i] = SimRobot.qMin(i);
            if(qDes[i]>=SimRobot.qMax(i)) qDes[i] = SimRobot.qMax(i);
        }
    }
    return qDes;
}

std::vector<double> SimpleCartesianControllerTest(const Robot & SimRobot, const std::vector<double> & InitConfig, const int & SwingLinkInfoIndex, double InnerTime, double TimeStep){
    int RobotLinkIndex = NonlinearOptimizerInfo::RobotLinkInfo[SwingLinkInfoIndex].LinkIndex;
    
    Vector3 EndEffectorLocalPos = NonlinearOptimizerInfo::RobotLinkInfo[SwingLinkInfoIndex].AvgLocalContact;
    Vector3 EndEffectorGlobalPos;
    SimRobot.GetWorldPosition(EndEffectorLocalPos, RobotLinkIndex, EndEffectorGlobalPos);
    Matrix FullJac;
    SimRobot.GetFullJacobian(EndEffectorLocalPos, RobotLinkIndex, FullJac);

    Matrix *Htheta[3];
    Matrix *Hp[3];
    SimRobot.GetJacobianDeriv(EndEffectorLocalPos, RobotLinkIndex, Htheta, Hp);

    Eigen::MatrixXd FullJacMat(FullJac.numRows(), FullJac.numCols());
    for (int i = 0; i < FullJac.numRows(); i++){
        for (int j = 0; j < FullJac.numCols(); j++){
            FullJacMat(i,j) = FullJac(i,j);
        }        
    }
    Eigen::MatrixXd FullJacMatInv = FullJacMat.completeOrthogonalDecomposition().pseudoInverse();
    
    // For position
    // Vector3 EndEffectorGlobalPosRef(1.0, 1.0, 1.0);
    Vector3 EndEffectorGlobalPosRef(EndEffectorGlobalPos);
    // 1.052793552448519, 0.67004973564675863, 0.50351747714962491
    Vector3 PosOffset = -Kp * (EndEffectorGlobalPos - EndEffectorGlobalPosRef);
    // For orientation
    std::cout<<"PosOffset: "<<PosOffset<<endl;
    QuaternionRotation EndEffectorQuat = getEndEffectorQuaternion(SimRobot, SwingLinkInfoIndex);
    QuaternionRotation EndEffectorQuatRef(0.7071, 0.7071, 0.0, 0.0);

    // q1 desired
    double eta1 = EndEffectorQuatRef.data[0];
    double q1_x = EndEffectorQuatRef.data[1];
    double q1_y = EndEffectorQuatRef.data[2];
    double q1_z = EndEffectorQuatRef.data[3];

    // q2 actual
    double eta2 = EndEffectorQuat.data[0];
    double q2_x = EndEffectorQuat.data[1];
    double q2_y = EndEffectorQuat.data[2];
    double q2_z = EndEffectorQuat.data[3];

    Vector3 q1(q1_x, q1_y, q1_z);
    Vector3 q2(q2_x, q2_y, q2_z);

    Vector3 delta_q = eta1 * q2 - eta2 * q1 - cross(q1, q2);
    std::cout<<"delta_q: "<<delta_q<<endl;
    std::cout<<"EndEffectorGlobalPosRef: "<<EndEffectorGlobalPosRef<<endl;
    Vector3 OriOffset = -Ko * delta_q;

    Eigen::VectorXd RHS(6);
    for (int i = 0; i < 3; i++){
        RHS[i] = OriOffset[i];
        RHS[i+3] = PosOffset[i];
    }

    Eigen::VectorXd qdotDes = FullJacMatInv * RHS;
    std::vector<double> qDes = SimRobot.q;
    double offsetTol = 1e-6;
    for (int i = 0; i < SimRobot.q.size(); i++){
        // std::cout<<"Link: "<<i<<" velocity: "<<qdotDes[i]<<endl;
        double qDesOffset = qdotDes[i] * TimeStep;
        qDes[i] = SimRobot.q[i] + qDesOffset;
        if(abs(qDesOffset)<=offsetTol) qDes[i] = InitConfig[i];
        if(qDes[i]<=SimRobot.qMin(i)) qDes[i] = SimRobot.qMin(i);
        if(qDes[i]>=SimRobot.qMax(i)) qDes[i] = SimRobot.qMax(i);
    }
    return qDes;
}


