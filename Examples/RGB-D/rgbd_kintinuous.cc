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


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include <unistd.h>

#include<opencv2/core/core.hpp>
#include<opencv2/imgcodecs/legacy/constants_c.h>

#include<System.h>

using namespace std;

void LoadImages(const string &strPathToSequence,
                vector<string> &vstrFilenamesRGB,
                vector<string> &vstrFilenamesDEP, 
                vector<double> &vTimestamps);

int main(int argc, char **argv)
{
    if(argc != 4)
    {
        cerr << endl << "Usage: ./rgbd_neural path_to_vocabulary path_to_settings path_to_sequence" << endl;
        return 1;
    }

    // Retrieve paths to images
    vector<string> vstrFilenamesRGB;
    vector<string> vstrFilenamesDEP;
    vector<double> vTimestamps;
    string strDataPath = string(argv[3]);
    LoadImages(strDataPath, vstrFilenamesRGB, vstrFilenamesDEP, vTimestamps);

    // Check consistency in the number of images and depthmaps
    int nImages = vstrFilenamesRGB.size();
    if(vstrFilenamesRGB.empty())
    {
        cerr << endl << "No images found in provided path." << endl;
        return 1;
    }
    else if(vstrFilenamesDEP.size()!=vstrFilenamesRGB.size())
    {
        cerr << endl << "Different number of images for rgb and depth." << endl;
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::RGBD,true);

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    // save log data
    ofstream output_file1, output_file2, output_file3, output_file4, output_file_all;
    output_file1.open("AllTrajectoryKFs-1.yml");
    output_file2.open("AllTrajectoryKFs-2.yml");
    output_file3.open("AllTrajectoryKFs-3.yml");
    output_file4.open("AllTrajectoryKFs-4.yml");
    // ofstream output_file, output_file_all;
    // output_file.open("AllTrajectoryKFs.yml");
    output_file_all.open("AllTrajectoryAllFrames.yml");

    // Main loop
    cv::Mat imRGB, imD;
    for(int ni=0; ni<nImages; ni++)
    {
        // Read image and depthmap from file
        imRGB = cv::imread(vstrFilenamesRGB[ni],CV_LOAD_IMAGE_UNCHANGED);
        imD   = cv::imread(vstrFilenamesDEP[ni],CV_LOAD_IMAGE_UNCHANGED);
        double tframe = vTimestamps[ni];

        if(imRGB.empty())
        {
            cerr << endl << "Failed to load image at: "
                 << string(argv[3]) << "/" << vstrFilenamesRGB[ni] << endl;
            return 1;
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the image to the SLAM system
        SLAM.TrackRGBD(imRGB,imD,tframe);


#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[ni]=ttrack;

        // Wait to load the next frame
        double T=0;
        if(ni<nImages-1)
            T = vTimestamps[ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimestamps[ni-1];

        if(ttrack<T)
            usleep((T-ttrack)*1e6);

        usleep(0.1*1e6);
        if (ni<2000)        
            SLAM.SaveTrajectoryAllKFs(output_file1, ni);
        else if (ni<2800)
            SLAM.SaveTrajectoryAllKFs(output_file2, ni);
        else if (ni<3600)
            SLAM.SaveTrajectoryAllKFs(output_file3, ni);
        else
            SLAM.SaveTrajectoryAllKFs(output_file4, ni);
        // SLAM.SaveTrajectoryAllKFs(output_file, ni);
        // SLAM.SaveTrajectoryAllFrames(output_file_all, ni);
        SLAM.SaveTrajectoryAllFramesNew(output_file_all, ni);

    }

    // Stop all threads
    SLAM.Shutdown();

    output_file1.close();
    output_file2.close();
    output_file3.close();
    output_file4.close();
    // output_file.close();
    output_file_all.close();

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
    SLAM.SaveTrajectoryTUM("AllCameraTrajectory.txt");
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");  

    return 0;
}

void LoadImages(const string &strPathToSequence,
                vector<string> &vstrFilenamesRGB,
                vector<string> &vstrFilenamesDEP, 
                vector<double> &vTimestamps)
{
    // +++ timestamps +++
    ifstream fTimes;
    string strPathTimeFile = strPathToSequence + "/times.txt";
    fTimes.open(strPathTimeFile.c_str());
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            ss >> t;
            vTimestamps.push_back(t);
        }
    }
    fTimes.close();

    string strPrefixImage = strPathToSequence + "/images/";
    string strPrefixDepth = strPathToSequence + "/depth/";   

    const int nTimes = vTimestamps.size();
    vstrFilenamesRGB.resize(nTimes);
    vstrFilenamesDEP.resize(nTimes);    

    for(int i=0; i<nTimes; i++)
    {
        stringstream ss;
        ss << i+1;
        vstrFilenamesRGB[i] = strPrefixImage + "img" + ss.str() + ".png";
        vstrFilenamesDEP[i] = strPrefixDepth + "depth" + ss.str() + ".png";
    }



}
