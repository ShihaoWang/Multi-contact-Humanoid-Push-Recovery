// This function is used to load in the robot's specification
#include <iostream>
#include <fstream>
#include <sstream>
#include "CommonHeader.h"

std::vector<LinkInfo> LinkInfoLoader(string ContactLinkFile){
  string str_line, str_keyword;
  str_keyword = "Link";
  ContactLinkFile+="LinkInfo.txt";
  ifstream linkinfofile (ContactLinkFile);
  std::vector<LinkInfo> LinkInfoVec;
  LinkInfo contact_link_i;
  int LinkIndex = -1;
  if (linkinfofile.is_open()){
    while (getline (linkinfofile,str_line) ){
      if (str_line.find(str_keyword) != string::npos){
        str_line.erase(str_line.begin(), str_line.begin()+4);
        int link_index = stoi(str_line);
        LinkInfo contact_link_point_i(link_index);
        contact_link_i = contact_link_point_i;
        LinkInfoVec.push_back(contact_link_i);
        LinkIndex = LinkIndex + 1;
      } else {
        // Each row will be converted into a Vector3 and push into LinkInfo
        istringstream ss(str_line);     // Here the string row can be separated by the comma
        string link_coordinate_i;
        vector <double> link_coordinates;
        while (ss >> link_coordinate_i){
          link_coordinate_i.erase(link_coordinate_i.end()-1, link_coordinate_i.end());
          link_coordinates.push_back(stod(link_coordinate_i));
        }
        Vector3 link_coordinates_Vect3(link_coordinates[0], link_coordinates[1], link_coordinates[2]);
        LinkInfoVec[LinkIndex].AddLocalConact(link_coordinates_Vect3);
      }
    }
    for (int i = 0; i < LinkInfoVec.size(); i++)
      LinkInfoVec[i].AvgContactUpdate();
    linkinfofile.close();
  }
  else std::cerr << "Unable to open file";

  if (!LinkInfoVec.size())
    std::cerr<<"Robot Contact Link Info failed to be loaded!"<<"\n";
  
  return LinkInfoVec;
}

std::vector<ContactStatusInfo> ContactStatusInfoLoader(const string & ContactStatusFile){
  string str_line, str_keyword;
  str_keyword = "Link";

  ifstream contactstatusinfofile (ContactStatusFile);
  std::vector<ContactStatusInfo> ContactStatusInfoVec;
  ContactStatusInfo contact_link_i;
  int LinkIndex = -1;
  if (contactstatusinfofile.is_open()){
    while (getline (contactstatusinfofile,str_line)){
      if (str_line.find(str_keyword) != string::npos){
        str_line.erase (str_line.begin(), str_line.begin()+4);
        int link_index = stoi(str_line);
        ContactStatusInfo contact_link_point_i(link_index);
        contact_link_i = contact_link_point_i;
        ContactStatusInfoVec.push_back(contact_link_i);
        LinkIndex = LinkIndex + 1;
      } else ContactStatusInfoVec[LinkIndex].AddLocalConactStatus(std::stoi(str_line));
    }
    contactstatusinfofile.close();
  }
  else std::cerr << "\nUnable to open file";

  if (!ContactStatusInfoVec.size())
    std::cerr<<"\nRobot Contact Status Info failed to be loaded!"<<"\n";
  
  return ContactStatusInfoVec;
}

std::vector<int> LinkIndicesLoader(const string & UserFilePath, const string & FileName){
  string TorsoLinkFilePath = UserFilePath + FileName;
  ifstream TorsoLinkFile (TorsoLinkFilePath);
  std::vector<int> TorsoLinkVec;
  int LinkIndex = -1;
  if (TorsoLinkFile.is_open()){
    string str_line;
    while (getline (TorsoLinkFile, str_line) ){
      int link_index = stoi(str_line);
      TorsoLinkVec.push_back(link_index);
    }
    TorsoLinkFile.close();
  }
  else std::cerr << "Unable to open file " <<TorsoLinkFilePath<<" does not exist!\n";
  if (!TorsoLinkVec.size()) 
    std::cerr<<TorsoLinkFilePath<<" failed to be loaded!"<<"\n";
  return TorsoLinkVec;
}
