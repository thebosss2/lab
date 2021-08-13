




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
//#include "ctello.h"

//const char* const TELLO_STREAM_URL{"udp://0.0.0.0:11111"};

using namespace::std;
//using ctello::Tello;
using cv::CAP_FFMPEG;
using cv::imshow;
using cv::VideoCapture;
using cv::waitKey;

void saveMap(ORB_SLAM2::System &SLAM,string&);

bool ret=false;
bool finish=false;

VideoCapture capture;
    cv::Mat im;

pthread_mutex_t my_mutex;


//Tello tello;

void* scan(void *arg);

void* takePicture(void*);



int main(int argc, char **arg)
{
    if(argc != 5)
    {
        cerr << endl << "Usage: ./mono_tum path_to_vocabulary path_to_settings path_to_sequence path_to_point_pointData" << endl;
        return 1;
    }


    //check connection 
    /*if (!tello.Bind())
    {
        return 0;
    }*/

    pthread_mutex_init(&my_mutex,NULL);

    pthread_t thread1;
    pthread_attr_t attr1;

    //create a new thread

    /*tello.SendCommand("streamon");
     while(!(tello.ReceiveResponse()));
    tello.SendCommand("takeoff");
     while (!(tello.ReceiveResponse()));*/


     

     pthread_attr_init(&attr1);
    pthread_create(&thread1, &attr1, scan, arg);
   
    /*tello.SendCommand("up 50");
    while (!(tello.ReceiveResponse()));*/

    /*for (size_t i = 0; i < 4; i++) //change to 18 iterations for 360deg
    {
        tello.SendCommand("cw 20");
         while (!(tello.ReceiveResponse()));
        tello.SendCommand("up 50");
         while (!(tello.ReceiveResponse()));
        tello.SendCommand("down 100");
          while (!(tello.ReceiveResponse()));
        tello.SendCommand("up 50");
         while (!(tello.ReceiveResponse()));
    }*/

    

    sleep(100);

    finish=true;

    ret = true;
    pthread_join(thread1, NULL);
    






    cout<<"dane all"<<endl;

    /*tello.SendCommand("land");
    while (!(tello.ReceiveResponse()));*/

    return 0;
}


void* scan(void* arg)
{

    int i=0;

    char** argv=(char**)arg;

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);

    string strPointData=string(argv[4])+"/PointData.csv";

    cout << endl << "-------" << endl;
    cout << "Start processing ..." << endl;

    // Main loop
    pthread_t thread2;
    pthread_attr_t attr2;
    pthread_attr_init(&attr2);
    pthread_create(&thread2, &attr2, takePicture, nullptr);
    

    while (!ret)
        cout<<"main: error"<<endl;

    while(!finish)
    {
        //############################

        
        //see tellos camera stream
        

        
        //cout<<"In the main loop"<<i<<endl;
        cout<<"main: waiting for mutex"<<i<<endl;
        pthread_mutex_lock(&my_mutex);
        cout<<"main: mutex aquired"<<i<<endl;
        if (im.empty())
        {
            cout<<"main: Image is empty"<<endl;
        }
        else
        {
            double t=capture.get(CV_CAP_PROP_POS_MSEC );
            
            SLAM.TrackMonocular(im,t);
        }
        pthread_mutex_unlock(&my_mutex);
        cout<<"main: mutex released"<<i<<endl;

        //double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        i++;
        
    }

    // Stop all threads
    SLAM.Shutdown();
    // saving the map

    //saveMap(SLAM,strPointData);
pthread_join(thread2, NULL);
    return nullptr;

}


void* takePicture(void* ptr)
{
    int i=0;
    int deviceID = 0;             // 0 = open default camera
    int apiID = cv::CAP_ANY;      // 0 = autodetect default API
    // open selected camera using selected API
    capture.open(deviceID, apiID);
    if (!capture.isOpened()) {
        cerr << "ERROR! Unable to open camera\n";
        return nullptr;
    }
    ret=true;
    while (!finish)
    {
        cout<<"thread: waiting for mutex"<<i<<endl;
        cout<<"thread: mutex aquired"<<i<<endl;
        pthread_mutex_lock(&my_mutex);
        capture.read(im);
        cout<<"thread: array elements: "<<im.total()<<endl;
                if (im.empty()) {
            cerr << "ERROR! blank frame grabbed\n";
            break;
                }
        std::cout<<"thread: Just read an image"<<i<<std::endl;
        //imshow("CTello Stream", im);
        pthread_mutex_unlock(&my_mutex);
        cout<<"thread: mutex released"<<i<<endl;
        
        sleep(0.1);
        i++;
    }
    cout<<"thread: done!"<<endl;
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

