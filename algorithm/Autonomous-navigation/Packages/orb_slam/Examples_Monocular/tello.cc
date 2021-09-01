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
// #include<algorithm>
#include<fstream>
// #include<chrono>
#include <unistd.h>
#include<System.h>
#include <Tracking.h>
#include<Converter.h>
#include<sys/socket.h>	//socket
#include<arpa/inet.h>	//inet_addr
#include <opencv2/core.hpp>
// #include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
// #include <csignal>
// #include <math.h>
#include <string>
#include <thread> 
/************* PATHS *************/
const std::string slam_to_tello_file = "/tmp/slam_to_tello.txt";
//const std::string currentPointFileFile = "/tmp/currentPointFile.csv";
const char RequsetSavePoints= '1';
const char Done = '2';
//const char CheckSlamStatus = '3';
const char RequsetSaveCurrentPoints = '4';
const char Exit = '5';
char Localized = '0';
int closeProgram = 0;
/************* SIGNAL *************/
#define ROWS 720
#define COLS 960
#define COLORS 3
#define NotLocalized 6
#define Localized 7
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
cv::Mat pose;


void saveCurrentMap(ORB_SLAM2::Frame* frame){
	ORB_SLAM2::Frame fr = *frame;
	std::ofstream currentPointFile("/tmp/currentPointData.csv",std::ofstream::out | std::ofstream::trunc);
	//fr.UpdatePoseMatrices();
    auto Rwc = fr.mRwc.clone();
    if(Rwc.empty()){
	    currentPointFile.close();
	    send_signal(NotLocalized);
	    return;	
    }
    auto q = ORB_SLAM2::Converter::toQuaternion(Rwc);
    auto mOw = fr.mOw.clone();
    auto pt = fr.mvpMapPoints[0];
    for(auto p : fr.mvpMapPoints) {
        if (p != nullptr) {
        	pt = p;
        	break;
        }
    }
    if(pt == nullptr){
	    currentPointFile.close();
	    send_signal(NotLocalized);
	    return;	
    }
    volatile auto keyframe = pt->GetReferenceKeyFrame();
    cv::Mat Tcw = keyframe->GetPose();
    cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);
    for(auto p : fr.mvpMapPoints) {
        if (p != nullptr)
        {
            Eigen::Matrix<double, 3, 1> v = ORB_SLAM2::Converter::toVector3d(p->GetWorldPos());
            currentPointFile << v.x() << "," << v.y() << "," << v.z()<< "," << q[0]
            << "," << q[1]<< "," << q[2]<< "," << q[3] << "," << fr.mnId << 
            ","<< twc.at<float>(0)  << "," << twc.at<float>(1)  << "," << twc.at<float>(2) << 
            ","<< mOw.at<float>(0)  << "," << mOw.at<float>(1)  << "," << mOw.at<float>(2) << std::endl; 
        }
    }
    
    currentPointFile.close();
}
void saveMap( int fileNumber ) {
    std::ofstream pointData;
    pointData.open("/tmp/pointData"+ std::to_string(fileNumber) + ".csv");
    for(auto p : allMapPoints) {
        if (p != nullptr)
        {
            auto frame = p->GetReferenceKeyFrame();
            int frameId = frame->mnFrameId;
            cv::Mat Tcw = frame->GetPose();
            auto point = p->GetWorldPos();
            cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
            cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);
            auto q = ORB_SLAM2::Converter::toQuaternion(Rwc);
            Eigen::Matrix<double, 3, 1> v = ORB_SLAM2::Converter::toVector3d(point);
            pointData << v.x() << "," << v.y() << "," << v.z()<< "," << q[0]
            << "," << q[1]<< "," << q[2]<< "," << q[3] << "," << frameId << 
            ","<< twc.at<float>(0)  << "," << twc.at<float>(1)  << "," << twc.at<float>(2) << std::endl; 
        }
    }
    pointData.close();
    std::cout << "saved map" << std::endl;
  
}
int connect_to_socket(int port){
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
	server.sin_port = htons(port);
    while (1)
    {
        if (connect(sock , (struct sockaddr *)&server , sizeof(server)) >= 0)
        {
            std::cout <<"connection successed" << std::endl;
            return sock;
        }
    }
}
void alert_of_big_change(ORB_SLAM2::System* SLAM){
    int sock = connect_to_socket(5555);
    int amountOfChanges = SLAM->GetMap()->GetLastBigChangeIdx();
    char message[1];
    while (1)
    {
        message[0]='1';
        int numberOfChanges = SLAM->GetMap()->GetLastBigChangeIdx();
        if (amountOfChanges != numberOfChanges)
        {
            if (send(sock,message,1,0))
            {
                std::cout << "we sent an alert for big change: "<<message<< std::endl;
            }
            amountOfChanges = numberOfChanges;

        }
        
        
    }


}
void comunicate_with_drone(ORB_SLAM2::System* SLAM){
    int sock = connect_to_socket(4444);
    char server_reply[2000];
    char message[1];
    std::string command = "";
    int amountOfMaps = 0;
    while (1)
    {
        message[0]='n';
        if(recv(sock , server_reply , 2000 , 0) >= 0)
		{
			
            command = std::string(server_reply);
            std::cout << "command from python is: " << command << std::endl;
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
                command = "";
                server_reply[0] = 0;
                message[0]=Done;
            }
            else if (command.find(RequsetSaveCurrentPoints) != std::string::npos){
            	ORB_SLAM2::Tracking* tracker= SLAM->GetTracker(); 
            	ORB_SLAM2::Frame fr = tracker->mCurrentFrame;
            	saveCurrentMap(&fr);
            	
                message[0]=Done;
            }
		}
        if(message[0] != 'n'){
            if (send(sock,message,1,0))
            {
                std::cout << "we sent a replay: " << message[0] << std::endl;
            }else
            {
                std::cout << "coudlnt send a replay"<< std::endl;
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

    const auto sizeOfData = ROWS * COLS * COLORS;            // TODO: move magic numbers to config file

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);

    const char *fifo_name = "/tmp/images.fifo";
    std::ifstream fifo;
    char buf[sizeOfData];
    //cv::namedWindow( "read from pipe", CV_WINDOW_AUTOSIZE );
    std::thread thread_comunicate_with_drone(comunicate_with_drone,&SLAM);
    std::thread thread_alert_of_big_change(alert_of_big_change,&SLAM);
    std::cout << "Start processing video from named pipe ..." << std::endl;
    for(; ; )
    {
        
        fifo.open(fifo_name, std::ios_base::in);

        // Read image from pipe
        fifo.read(buf, sizeOfData);
        if (!fifo.good()) {
            
            std::cerr << "Read failed" << std::endl;
            fifo.close();
            continue;
        }

        // Read image from buf to cvmat
        cv::Mat imageWithData2 = cv::Mat(ROWS, COLS, CV_8UC3, buf);//.clone();
        fifo.close();

                // TODO: magic number fix according to original mono
        
        double tframe = 0.2;
        // Pass the image to the SLAM system
        pose = SLAM.TrackMonocular(imageWithData2,tframe);
        if (pose.empty()) {
            send_signal(NotLocalized);
            continue;
        }
        send_signal(Localized);
        saveCurrentPosition(pose);
        if (closeProgram)
        {
            break;
        }
    }    
        
    // Stop all threads
    SLAM.Shutdown();

    // cv::destroyAllWindows();

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
