




#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include <unistd.h>	

#include<opencv2/core/core.hpp>
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgcodecs.hpp"
#include <System.h>
#include <Converter.h>	
#include <pthread.h>
#include "ctello.h"

const char* const TELLO_STREAM_URL{"udp://0.0.0.0:11111"};

using namespace::std;
using ctello::Tello;
using cv::CAP_FFMPEG;
using cv::imshow;
using cv::VideoCapture;
using cv::waitKey;

//function from moodle
void saveMap(ORB_SLAM2::System &SLAM,string&);

bool ret=false;
bool finish=false;

VideoCapture capture;
    cv::Mat im;

pthread_mutex_t my_mutex;


Tello tello;

//our function to use ORB_SLAM for saving map. modified by mono_tum.cc
void* scan(void *arg);


//multithreaded implementation to analize pictures faster (our code)
void* takePicture(void*);


//main was written by ourselves
int main(int argc, char **arg)
{
    //check parameters
    if(argc != 4)
    {
        cerr << endl << "Usage: ./mono_tum path_to_vocabulary path_to_settings path_to_point_pointData" << endl;
        return 1;
    }

    //check connection 
    if (!tello.Bind())
    {
        return 0;
    }

    pthread_mutex_init(&my_mutex,NULL);

    pthread_t thread1;
    pthread_attr_t attr1;

    tello.SendCommand("streamon");
     while(!(tello.ReceiveResponse()));
    tello.SendCommand("takeoff");
     while (!(tello.ReceiveResponse()));

    tello.SendCommand("up 50");
    while (!(tello.ReceiveResponse()));

    //start creating map
    pthread_attr_init(&attr1);
    pthread_create(&thread1, &attr1, scan, arg);

    for (size_t i = 0; i < 4; i++) //change to 18 iterations for 360deg
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

    //let scan to finish
    sleep(100);

    //save map
    finish=true;
    ret = true;
    pthread_join(thread1, NULL);
    
    //call algorythm here
    //mave twards the door...

    tello.SendCommand("land");
    while (!(tello.ReceiveResponse()));

    return 0;
}


void* scan(void* arg)
{

    int i=0;
    char** argv=(char**)arg;

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);

    string strPointData=string(argv[3])+"/PointData.csv";

    cout << endl << "-------" << endl;
    cout << "Start processing ..." << endl;

    // Main loop
    pthread_t thread2;
    pthread_attr_t attr2;
    pthread_attr_init(&attr2);
    pthread_create(&thread2, &attr2, takePicture, nullptr);
    

    while (!ret); //wait for videoCapture to be initiallized by thread2

    while(!finish) //checking main thread request
    {   
        //see tellos camera stream
    
        pthread_mutex_lock(&my_mutex); //mutex on image object

        if (im.empty())
            cout<<"Image is empty"<<endl;

        else
        {
            //initializing time by tutorial found on opencv.org
            double t=capture.get(CV_CAP_PROP_POS_MSEC );
            SLAM.TrackMonocular(im,t);
        }
        pthread_mutex_unlock(&my_mutex);

        //double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
        i++;  
    }

    // Stop second thread
    SLAM.Shutdown();
    // saving the map

    saveMap(SLAM,strPointData);
    pthread_join(thread2, NULL);
    return nullptr;

}


void* takePicture(void* ptr)
{
    //currently using leptop camera, update on Monday
    int i=0;
    int deviceID = 0;             // 0 = open default camera
    int apiID = cv::CAP_ANY;      // 0 = autodetect default API
    // open selected camera using selected API
    capture.open(deviceID, apiID);
    if (!capture.isOpened()) {
        cerr << "ERROR: Unable to open camera\n";
        return nullptr;
    }
    ret=true;//let thread1 continue
    while (!finish) //communication with main thread
    {
        pthread_mutex_lock(&my_mutex);
        capture.read(im);

        if (im.empty())
        {
            cerr << "ERROR: blank frame grabbed\n";
            break;
        }

        //camera stream to screen:
        imshow("ctello Stream", im);
        pthread_mutex_unlock(&my_mutex);
        
        sleep(0.1); //let thread1 to pass image to orb_slam
        i++;
    }
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

