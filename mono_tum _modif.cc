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

Find better way to skip lines when getting new frame,
(maybe executing the function inline to save file pointer?)

*/

/* Daniel's smart notes

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

void LoadImage(const string &strFile, vector<string> &vstrImageFilenames,vector<double> &vTimestamps, const int nLines);

//our function for saving map
void saveMap(ORB_SLAM2::System &SLAM);


int main(int argc, char **argv)
{
    if(argc != 4)
    {
        cerr << endl << "Usage: ./mono_tum path_to_vocabulary path_to_settings path_to_sequence" << endl;
        return 1;
    }

    // Retrieve paths to images
    vector<string> vstrImageFilenames;
    vector<double> vTimestamps;
    string strFile = string(argv[3])+"/rgb.txt";

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing ..." << endl;

    // Main loop
    cv::Mat im;
    int nImages = 0;

    while(1)
    {
        LoadImage(strFile, vstrImageFilenames, vTimestamps, nImages); //includes busy wait
        nImages += 1;

        // Read image from file
        im = cv::imread(string(argv[3])+"/"+vstrImageFilenames[nImages-1],CV_LOAD_IMAGE_UNCHANGED);
        double tframe = vTimestamps[nImages-1];

        if (im[0][0] == 'q') //need to stop somehow
        {
            cerr << endl << "Quitting..." <<endl;
            break;
        }
            
        

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the image to the SLAM system
        SLAM.TrackMonocular(im,tframe);

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack.push_back(ttrack);

    }

    

    // Stop all threads
    SLAM.Shutdown();

    //save map(MY)
    saveMap(SLAM);

    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
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

    return 0;
}

//must find faster way for skipping lines; maybe needed 2 
void LoadImage(const string &strFile, vector<string> &vstrImageFilenames, vector<double> &vTimestamps, const int nLines)
{
    ifstream f;
    f.open(strFile.c_str());

    // skip first lines, as num of already processed frames + 3
    string s0;

    for (size_t i = 0; i < nlines+3; i++)
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
}

void saveMap(ORB_SLAM2::System &SLAM)
{
    std::vector<ORB_SLAM2::MapPoint*> mapPoints = SLAM.GetMap()->GetAllMapPoints();
    std::ofstream pointData;
    pointData.open("/tmp/pointData.csv"); //Choose location
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



