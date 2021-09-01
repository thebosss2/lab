//
// Created by ge on 14/11/19.
//
/**
 *
 * This file is a modification of ORB-SLAM2.
 *
 *
 * Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
 * For more information see <https://github.com/raulmur/ORB_SLAM2>
 *
 * ORB-SLAM2 is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ORB-SLAM2 is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include <unistd.h>
#include<System.h>
#include<Converter.h>
#include<sys/socket.h>	//socket
#include<arpa/inet.h>	//inet_addr
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <csignal>
#include <math.h>
#include <string>
#include <thread>
#include <vector>
/************* PATHS *************/
const std::string slam_to_tello_file = "/tmp/slam_to_tello.txt";
const std::string slam_to_tello_file_current_map = "/tmp/slam_to_tello_file_current_map.txt";
const char RequsetSavePoints= '1';
const char PointsSaved = '2';
const char CheckSlamStatus = '3';
const char Exit = '5';
char Localized = '0';
int closeProgram = 0;
/************* SIGNAL *************/
#define ROWS 720
#define COLS 960
#define COLORS 3
#define NotLocalized 6
#define Localized 7
#define AMOUNT_OF_FRAMES_WE_BUFF 100
#define AMOUNT_OF_LOST_LOCALIZATIONS 35
void send_signal(int signal);
int get_signal(std::string fileName);
void save_points();
void save_curr_points();
void saveCurrentPosition(cv::Mat Tcw){
    cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
    cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);
    std::vector<float> q = ORB_SLAM2::Converter::toQuaternion(Rwc);
    std::ofstream currentPostionFile;
    currentPostionFile.open("/tmp/tello_last_location.csv",std::ofstream::out | std::ofstream::trunc);
    currentPostionFile << twc.at<float>(0)  << "," << twc.at<float>(1)  << "," << twc.at<float>(2) << "," << q[0]
    << "," << q[1]<< "," << q[2]<< "," << q[3] << std::endl;
    currentPostionFile.close();   
}
std::vector<ORB_SLAM2::MapPoint*> allMapPoints;
std::vector<ORB_SLAM2::MapPoint*> currMapPoints;
void saveCurrentMap(){
    std::ofstream currentPointData;
    currentPointData.open("/tmp/currentPointData.csv");
    for(auto p : currMapPoints) {
        Eigen::Matrix<double, 3, 1> v = ORB_SLAM2::Converter::toVector3d(p->GetWorldPos());
        currentPointData << v.x() << "," << v.y() << "," << v.z() << std::endl;
    }

    currentPointData.close();
}
void saveMap( int fileNumber ) {   
    std::ofstream pointData;
    pointData.open("/tmp/pointData"+ std::to_string(fileNumber) + ".csv");
        std::cout << "allMapPoints size" << allMapPoints.size() << std::endl;

    for(auto p : allMapPoints) {
        auto point = p->GetWorldPos();
        auto frame = p->GetReferenceKeyFrame();
        auto pose = frame->GetPoseInverse();
        auto distance = frame->ComputeSceneMedianDepth(2);
        auto Rwc = pose.rowRange(0,3).colRange(0,3);
        auto q = ORB_SLAM2::Converter::toQuaternion(Rwc);
        Eigen::Matrix<double, 3, 1> v = ORB_SLAM2::Converter::toVector3d(point);
        pointData << v.x() << "," << v.y() << "," << v.z()<< "," << q[0]
        << "," << q[1]<< "," << q[2]<< "," << q[3] << "," << distance << std::endl;
    }
    pointData.close();
    std::cout << "saved map" << std::endl;
}
int connect_to_socket(){
	struct sockaddr_in server;
	int sock;
	//Create socket
	sock = socket(AF_INET , SOCK_STREAM , 0);
	if (sock == -1)
	{
		printf("Could not create socket");
	}
	puts("Socket created");
	
	server.sin_addr.s_addr = inet_addr("127.0.0.1");
	server.sin_family = AF_INET;
	server.sin_port = htons(4444);
    while (1)
    {
        if (connect(sock , (struct sockaddr *)&server , sizeof(server)) >= 0)
        {
            std::cout <<"connection successed";
            return sock;
        }
    }
    
	//Connect to remote server
	
    return sock;
}
void comunicate_with_drone(ORB_SLAM2::System* SLAM){
    int sock = connect_to_socket();
    char server_reply[2000];
    char message[1];
    std::string command = "";
    int amountOfMaps = 0;
    while (1)
    {
        if(recv(sock , server_reply , 2000 , 0) >= 0)
		{
			
            command = std::string(server_reply);
            if (command.find(Exit) != std::string::npos)
            {
                close(sock);
                std::cout << "we closed socket"<< std::endl;
                closeProgram = 1;
                break;
            }
            else if (command.find(RequsetSavePoints) != std::string::npos){
                
                allMapPoints = SLAM->GetMap()->GetAllMapPoints();
                saveMap(amountOfMaps++);
                message[0] = PointsSaved;
                if( send(sock , message , strlen(message) , 0) < 0)
                {
                    puts("Send failed");
                }
            }
            
            
		}
        
    }
    
}

int main(int argc, char **argv)
{
    if(argc != 3)
    {
        std::cerr << std::endl << "Usage: ./mono_tello path_to_vocabulary path_to_settings"
                        " (/dev/videox -> x, rdp stream -> full url)" << std::endl;
        std::cout << "Example command: ./mono_tello ../../Vocabulary/ORBvoc.txt tello.yaml" << std::endl;
        return 1;
    }
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);
    cv::VideoCapture capture("/tmp/outpy.avi");
    if(!capture.isOpened()){
        std::cout << "Error opening video stream or file" << std::endl;
        return 0;
    }else
    {
        std::cout << "Success opening video stream or file" << std::endl;
    }
    
    cv::Mat frame;
    double tframe = 0.2;
    capture >> frame;
    for(; ; )
    {
        SLAM.TrackMonocular(frame,capture.get(CV_CAP_PROP_POS_MSEC));
             
        capture >> frame;       
        if (frame.empty())
        {
            break;
        }
    }
    capture.release();
    allMapPoints = SLAM.GetMap()->GetAllMapPoints();
    if (allMapPoints.size() > 0)
    {
        saveMap(0);
    }
    SLAM.Shutdown();

    cvDestroyAllWindows();

    return 0;
}

int get_signal(std::string fileName)
{
    int signal = -100;
    std::string line = "";
    std::ifstream myfile(fileName);
    if (myfile.is_open()) {
        try {
            getline(myfile,line);
            signal = stoi(line);
            myfile.close();
        }
        catch(...) {
            return signal;
        }
    }
    return signal;
    
}

void send_signal(int signal)
{
    std::ofstream myfile;
    myfile.open(slam_to_tello_file, std::ios::trunc);
    myfile << std::to_string(signal);
    myfile.close();
}

