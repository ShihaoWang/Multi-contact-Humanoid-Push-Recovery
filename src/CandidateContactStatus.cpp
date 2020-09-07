#include "CommonHeader.h"
#include "NonlinearOptimizerInfo.h"

std::vector<ContactForm> getCandidateContactStatus(const Robot & SimRobot, const std::vector<ContactStatusInfo> & RobotContactInfo){
  // First part is for contact modification.
  // Second part is for contact addition.
  Vector3 COMPos(0.0, 0.0, 0.0), COMVel(0.0, 0.0, 0.0);
  getCentroidalState(SimRobot, COMPos, COMVel);

  std::vector<ContactForm> ContactFormVec;
  int LegActNo = RobotContactInfo[0].LocalContactStatus[0] + RobotContactInfo[1].LocalContactStatus[0];
  int AllActNo = LegActNo + RobotContactInfo[2].LocalContactStatus[0] + RobotContactInfo[3].LocalContactStatus[0];

  // Contact Modification
  switch (AllActNo){
    case 0:{
      std::cerr<<"No Active Contact!"<<endl;
      exit(1);
    }
    break;
    case 1:
    // No contact modification
    break;
    default:{
      // Algorithms for contact modification
      switch (LegActNo){
        case 0:{
          std::cerr<<"No Active Foot Contact!"<<endl;
          exit(1);
        }
        break;
        case 1:{
          // In this case, the contact modification can only be conducted for hand contact if there exists.
          switch (RobotContactInfo[2].LocalContactStatus[0]){
            case 1:{
              std::vector<ContactStatusInfo> RobotContactInfoModi = RobotContactInfo;
              RobotContactInfoModi[2].StatusSwitch(0);
              ContactFormVec.push_back(ContactForm(RobotContactInfoModi, 2, 0));
            }
            break;
            default:
            break;
          }
          switch (RobotContactInfo[3].LocalContactStatus[0]){
            case 1:{
              std::vector<ContactStatusInfo> RobotContactInfoModi = RobotContactInfo;
              RobotContactInfoModi[3].StatusSwitch(0);
              ContactFormVec.push_back(ContactForm(RobotContactInfoModi, 3, 0));
            }
            break;
            default:
            break;
          }
        }
        break;
        default:{
          // More general case where two feet contact is shown while hands may be involved.
          for (int i = 0; i < 4; i++){
            switch (RobotContactInfo[i].LocalContactStatus[0]){
              case 1:{
                std::vector<ContactStatusInfo> RobotContactInfoModi = RobotContactInfo;
                RobotContactInfoModi[i].StatusSwitch(0);
                ContactFormVec.push_back(ContactForm(RobotContactInfoModi, i, 0));
              }
              break;
              default:
              break;
            }
          }
        }
        break;
      }
    }
    break;
  }

  // Contact Addition
  for (int i = 0; i < RobotContactInfo.size(); i++){
    if(!RobotContactInfo[i].LocalContactStatus[0]){
      std::vector<ContactStatusInfo> RobotContactInfoTemp = RobotContactInfo;
      ContactFormVec.push_back(ContactForm(RobotContactInfoTemp, i, 1));
    }
  }
  return ContactFormVec;
}