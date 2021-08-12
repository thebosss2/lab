#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include <unistd.h>	

#include<opencv2/core/core.hpp>
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgcodecs.hpp"
#include <opencv2/videoio.hpp>
#include<System.h>
#include <Converter.h>	
#include<pthread.h>
#include "ctello.h"

const char* const TELLO_STREAM_URL{"udp://0.0.0.0:11111"};

using namespace::std;
using ctello::Tello;
using cv::CAP_FFMPEG;
using cv::imshow;
using cv::VideoCapture;
using cv::waitKey;

VideoCapture capture{TELLO_STREAM_URL, cv::CAP_FFMPEG};


void saveMap(ORB_SLAM2::System &SLAM,string&);

bool ret=false;

Tello tello;

void* scan(void *arg);



int main(int argc, char **arg)
{
    if(argc != 5)
    {
        cerr << endl << "Usage: ./mono_tum path_to_vocabulary path_to_settings path_to_sequence path_to_point_pointData" << endl;
        return 1;
    }


    //check connection 
    if (!tello.Bind())
    {
        return 0;
    }

    pthread_t thread;
    pthread_attr_t attr;

    //create a new thread

    pthread_attr_init(&attr);
    pthread_create(&thread, NULL, scan, arg);

    tello.SendCommand("streamon");
     while(!(tello.ReceiveResponse()));

    tello.SendCommand("takeoff");
     while (!(tello.ReceiveResponse()));

     
     tello.SendCommand("up 50");
    while (!(tello.ReceiveResponse()));

    for (size_t i = 0; i < 2; i++) //change to 18 iterations for 360deg
    {
        tello.SendCommand("cw 20");
         while (!(tello.ReceiveResponse()));
        tello.SendCommand("up 50");
         while (!(tello.ReceiveResponse()));
        tello.SendCommand("down 100");
          while (!(tello.ReceiveResponse()));
        tello.SendCommand("up 50");
         while (!(tello.ReceiveResponse()));
    }

    sleep(10);

    ret = true;
    //pthread_join(thread, NULL);








    tello.SendCommand("land");
    while (!(tello.ReceiveResponse()));

    return 0;
}


void* scan(void* arg)
{

    int i=0;

    char** argv=(char**)arg;

    string strPointData=string(argv[4])+"pointData.csv";

    // Create SLAM system. It initializes all system threads and gets ready to process ims.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);

    cout << endl << "-------" << endl;
    cout << "Start processing ..." << endl;

    // Main loop
    cv::Mat im;
    
    int index{0};
    bool busy{false};
    while(!ret)
    {
        //############################
        

        
        capture >> im;

        // Listen response

        //see tellos camera stream
        

        
        std::cout<<"Just read an image"<<std::endl;

        if (!im.empty())
        {
            cout<<"Image is empty!"<<endl;
        }
        else
        {
        // Pass the image to the SLAM system
        imshow("CTello Stream", im);
        double t = capture.get(CV_CAP_PROP_POS_MSEC);
        SLAM.TrackMonocular(im, t);
        
        }

        //double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
        sleep(0.1);
        i++;
    }

    // Stop all threads
    SLAM.Shutdown();
    // saving the map

    saveMap(SLAM,strPointData);

    return nullptr;

}


void saveMap(ORB_SLAM2::System &SLAM,string &strPointData)
{
    std::cout<<"saving the map"<<std::endl;

    std::vector<ORB_SLAM2::MapPoint*> mapPoints = SLAM.GetMap()->GetAllMapPoints();
    std::ofstream pointData;
    pointData.open(strPointData); //Choose location
    for(auto p : mapPoints)
    {
        if (p != NULL)
        {
            auto point = p->GetWorldPos();
            Eigen::Matrix<double, 3, 1> v = ORB_SLAM2::Converter::toVector3d(point);
            pointData << v.x() << "," << v.y() << "," << v.z()<<  std::endl;
        }
    }
    pointData.close();
}

