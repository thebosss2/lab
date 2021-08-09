/**
* This file is part of ORB-SLAM2.
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



/* To Do:

Change main loop to an infinite loop and busy wait until a new frame loads into rgb.txt.
Then load this single frame (similiar to LoadImages()).(copy from Load)
Proccess the frame the same way is already written.

*/

/* Daniel's notes

arg[0] = Examples/Monocular/TUMX.yaml or build own executable from similar source.
argv[1] = Vocabulary/ORBvoc.txt always (static vocabulary)
argv[2] = camera parameters. try: Examples/Monocular/TUMX.yaml
argv[3] = sequence, must be updated always somehow

*/




#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include <unistd.h>	
#include<opencv2/core/core.hpp>

#include<System.h>
#include <Converter.h>	
using namespace std;

void LoadImage(const string &strFile, vector<string> &vstrImageFilenames,
                vector<double> &vTimestamps);

//our function for saving map
void saveMap(ORB_SLAM2::System &SLAM,string&);


int main(int argc, char **argv)
{

int i=0;

    if(argc != 5)
    {
        cerr << endl << "Usage: ./mono_tum path_to_vocabulary path_to_settings path_to_sequence path_to_point_pointData" << endl;
        return 1;
    }

    // Retrieve paths to images
    vector<string> vstrImageFilenames;
    vector<double> vTimestamps;
    string strFile = string(argv[3])+"/rgb.txt";


    string strPointData=string(argv[4])+"/pointData.csv";

    std::ofstream pointData;
    pointData.open(strPointData); //Choose location

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);

    // Vector for tracking time statistics
    //vector<float> vTimesTrack;
    //vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing ..." << endl;

    // Main loop
    cv::Mat im;
    //int nImages = 0;


    //
    //
    // create a new thread copying the points to pointData.csv
    // passed parameters: reference to argv[4], reference to SLAM
    // use the code from save_map()
    //
    //

    ifstream f;
    f.open(strFile.c_str());
    string s;


    std::cout<<"Here!"<<std::endl;

getline(f,s);
getline(f,s);
getline(f,s);

    std::cout<<"Here!"<<std::endl;


    while(i<10000)
    {
        
        // recieve a new image path from the rgb.txt ()


        // lock mutual mutex- shared with code which saving the information into rgb.txt


        do
        {
            getline(f,s);
            //std::cout<<s<<std::endl;
        } while (s.empty());
        

        std::cout<<s<<std::endl;


        if (s=="q")
        {
            std::cout<<"and now here!!"<<std::endl;        
            break;
        }

        std::cout<<"Here!"<<std::endl;


        //while (s.empty());  we need to check whether the f_pos pointer goes further if line is empty

        stringstream ss;
        ss << s;
        double t;
        string sRGB;
        ss >> t;
        ss >> sRGB;

        //std::cout<<t<<std::endl<<sRGB<<std::endl;

        im = cv::imread(string(argv[3])+"/"+sRGB,CV_LOAD_IMAGE_UNCHANGED);


        if(im.empty())
        {
            cerr << endl << "Failed to load image at: " << string(argv[3]) << "/" << sRGB << endl;
            return 1;
        }

        std::cout<<"Just read an image"<<std::endl;

        /*if (!im.empty())
        {
            break;
        }*/

        

        // Pass the image to the SLAM system
        SLAM.TrackMonocular(im,t);


        //double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
std::cout<<"Here!"<<std::endl;


        i++;
    }

    // Stop all threads
    SLAM.Shutdown();



    // saving the map

    saveMap(SLAM,strPointData);

    // Tracking time statistics
    /*sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    return 0;*/




}

/*void LoadImage(const string &strFile, vector<string> &vstrImageFilenames, vector<double> &vTimestamps)
{
    ifstream f;
    f.open(strFile.c_str());

    // skip first three lines
    string s0;
    getline(f,s0);
    getline(f,s0);
    getline(f,s0);

    
        while(f.eof());
        
            string s;
            getline(f,s);
            if(!s.empty())
            {
                stringstream ss;
                ss << s;
                double t;
                string sRGB;
                ss >> t;
                vTimestamps.push_back(t);
                ss >> sRGB;
                vstrImageFilenames.push_back(sRGB);
            }
}*/

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



