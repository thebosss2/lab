
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
#include <thread>
#include "ctello.h"

const char* const TELLO_STREAM_URL{"udp://0.0.0.0:11111?fifo_size=100000"};

using ctello::Tello;
using cv::CAP_FFMPEG;
using cv::imshow;
using cv::VideoCapture;
using cv::waitKey;
using std::count;

//function from moodle
void saveMap(ORB_SLAM2::System &SLAM,string&);

bool ret=false;
bool finish=false;
bool orbSlamRunning = false;
cv::Mat im;
double t;



Tello tello;

//our function to use ORB_SLAM for saving map. modified by mono_tum.cc
void scan(void *arg);


//multithreaded implementation to analize pictures faster (our code)
void takePicture();


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


    tello.SendCommand("streamon");
     while(!(tello.ReceiveResponse()));
    
    std::thread ScanThread(scan,arg);

    while(!orbSlamRunning){
        usleep(20000);
    }
    tello.SendCommand("takeoff");
    sleep(3);

    tello.SendCommand("up 50");
    sleep(2);

    cout<<"start to rotate"<<endl;

    //start creating map
    
    for (size_t i = 0; i < 4; i++) //change to 18 iterations for 360deg
    {
        tello.SendCommand("cw 20");
        sleep(2);
         
        tello.SendCommand("forward 20");
        sleep(2);

        tello.SendCommand("back 20");
        sleep(2); 
        
        //cout<<"rotation no. "<<i<<endl;
    }

    //cout<<"finished rotating"<<endl;
    finish=true;
    ScanThread.join();
    //let scan to finish

    //save map
    //call algorythm here
    //mave twards the door...

    tello.SendCommand("land");

    return 0;
}


void scan(void* arg)
{

    cv::Mat frame;
    int i=0;

    char** argv=(char**)arg;
 

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);

    string strPointData=string(argv[3])+"/PointData.csv";

    cout << endl << "-------" << endl;
    cout << "Start processing ..." << endl;

    std::thread takePictureThread(takePicture);
    // cout<<"no problem1"<<endl;
    while(!ret){
        usleep(20000);
    }
    orbSlamRunning=true;
        // cout<<"waiting"<<endl; //wait for videoCapture to be initiallized by thread2
// cout<<"no problem2"<<endl;
    while(!finish) //checking main thread request
    {   
        // cout<<"no problem3"<<endl;
        //see tellos camera stream
    
        // pthread_mutex_lock(&my_mutex);
        // sleep(0.2);
        // cout << "II "<< im.empty() << endl;
        if (im.empty())
            cout<<"image is empty"<<endl;
        else if (i%10==0)
        {
            // cout<<"passing image to slam"<<endl;
            SLAM.TrackMonocular(im,t);
        }
        
        // pthread_mutex_unlock(&my_mutex);        

        //double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
        i++;  
    }

    // Stop second thread
    SLAM.Shutdown();
    // saving the map

    saveMap(SLAM,strPointData);
    takePictureThread.join();

}


void takePicture()
{
    //currently using leptop camera, update on Monday
    int i=0;

    VideoCapture capture{TELLO_STREAM_URL, cv::CAP_FFMPEG};
    ret=true;//let thread1 continue
    while (!finish) //communication with main thread
    {
        // pthread_mutex_lock(&my_mutex);
        capture>>im;
        // t=capture.get(CV_CAP_PROP_POS_MSEC);
        
        // cout<<"Just read an image"<<endl;

        //camera stream to screen:
        /*imshow("ctello Stream", im);
        if (waitKey(5) >= 0)
                break;*/
        // sleep(0.1);
        // pthread_mutex_unlock(&my_mutex);
        // sleep(0.1);
        // cout<<"here"<<endl;
    
        i++;
    }

        finish=true;
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


