




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

cv::Mat im;
double t;

pthread_mutex_t im_lock;


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
    cout<<"connected to tello"<<endl;
    pthread_mutex_init(&im_lock,NULL);

    pthread_t thread1;
    pthread_attr_t attr1;

    tello.SendCommand("streamon");
     while(!(tello.ReceiveResponse()));

    
    tello.SendCommand("takeoff");
     while (!(tello.ReceiveResponse()));

     cout<<"took off"<<endl;

    tello.SendCommand("up 50");
    while (!(tello.ReceiveResponse()));


    //start creating map
    pthread_attr_init(&attr1);
    pthread_create(&thread1, &attr1, scan, arg);

    sleep(10);

    for (size_t i = 0; i < 3; i++) //change to 18 iterations for 360deg
    {
        tello.SendCommand("cw 20");
         while (!(tello.ReceiveResponse()));
        tello.SendCommand("up 20");
         while (!(tello.ReceiveResponse()));
        tello.SendCommand("down 20");
          while (!(tello.ReceiveResponse()));
        tello.SendCommand("up 10");
         while (!(tello.ReceiveResponse()));
    }

    //let scan to finish
    sleep(4);
    

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
    cv::Mat frame;
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
    
    // cout<<"no problem1"<<endl;
    while (!ret)
        cout<<"waiting"<<endl; //wait for videoCapture to be initiallized by thread2
// cout<<"no problem2"<<endl;
    while(!finish) //checking main thread request
    {   
        cout<<"no problem3"<<endl;
        //see tellos camera stream
    
        pthread_mutex_lock(&im_lock);
        sleep(0.2);
        // cout << "II "<< im.empty() << endl;
        if (im.empty())
            cout<<"image is empty"<<endl;
        else
        {
            cout<<"passing image to slam"<<endl;
            SLAM.TrackMonocular(im,t);
        }
        
        pthread_mutex_unlock(&im_lock);        

        //double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
        i++;  
    }

    // Stop second thread
    SLAM.Shutdown();
    // saving the map

    //saveMap(SLAM,strPointData);
    pthread_join(thread2, NULL);
    return nullptr;

}


void* takePicture(void* ptr)
{
    int i=0;

    VideoCapture capture{TELLO_STREAM_URL, cv::CAP_FFMPEG};
    ret=true;//let thread1 continue
    while (!finish) //communication with main thread
    {
        pthread_mutex_lock(&im_lock);
        capture>>im;
        t=capture.get(CV_CAP_PROP_POS_MSEC);
        if (im.empty())
        {
            cerr << "ERROR: blank frame grabbed\n";
            break;
        }
        // cout<<"Just read an image"<<endl;

        //camera stream to screen:
        /*imshow("ctello Stream", im);
        if (waitKey(5) >= 0)
                break;*/
        // sleep(0.1);
        pthread_mutex_unlock(&im_lock);
        sleep(0.1);
        // cout<<"here"<<endl;
    
        i++;
    }

        finish=true;
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

