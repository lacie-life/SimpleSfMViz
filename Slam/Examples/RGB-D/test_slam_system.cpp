/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <string>

#include <opencv2/core/core.hpp>

#include <System.h>

// Dataset
std::string SEQ_PATH = "/home/lacie/Github/Dataset/rgbd_dataset_freiburg2_large_no_loop";
std::string RGB_IMAGE_PATH = "/home/lacie/Github/Dataset/rgbd_dataset_freiburg2_large_no_loop/rgb";
std::string DEPTH_IMAGE_PATH = "/home/lacie/Github/Dataset/rgbd_dataset_freiburg2_large_no_loop/depth";
std::string ASSO_DEPTH_PATH = "/home/lacie/Github/Dataset/rgbd_dataset_freiburg2_large_no_loop/depth.txt";
std::string ASSO_RGB_PATH = "/home/lacie/Github/Dataset/rgbd_dataset_freiburg2_large_no_loop/rgb.txt";

// SLAM System
std::string SETTING_PATH = "/home/lacie/Github/GreenHouseAR/Engine/data/config/rgbd.yaml";
std::string VOCABULARY_PATH = "/home/lacie/Github/GreenHouseAR/Slam/Vocabulary/ORBvoc.txt";

#define COMPILEDWITHC11 1

using namespace std;

void LoadImages(const string &strAssociationDFilename, const string &strAssociationRGBFilename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps);

int main(int argc, char **argv)
{
    vector<string> vstrImageFilenamesRGB;
    vector<string> vstrImageFilenamesD;
    vector<double> vTimestamps;

    LoadImages(ASSO_DEPTH_PATH, ASSO_RGB_PATH, vstrImageFilenamesRGB, vstrImageFilenamesD, vTimestamps);

    // Check consistency in the number of images and depthmaps
    int nImages = vstrImageFilenamesRGB.size();
    if(vstrImageFilenamesRGB.empty())
    {
        cerr << endl << "No images found in provided path." << endl;
        return 1;
    }
    else if(vstrImageFilenamesD.size()!=vstrImageFilenamesRGB.size())
    {
        cerr << endl << "Different number of images for rgb and depth." << endl;
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(VOCABULARY_PATH, SETTING_PATH,ORB_SLAM3::System::RGBD,true);
    float imageScale = SLAM.GetImageScale();

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    // Main loop
    cv::Mat imRGB, imD;
    for(int ni=0; ni<nImages; ni++)
    {
        // Read image and depthmap from file
        imRGB = cv::imread(SEQ_PATH +"/" +vstrImageFilenamesRGB[ni],cv::IMREAD_UNCHANGED); //,cv::IMREAD_UNCHANGED);
        imD = cv::imread(SEQ_PATH +"/" + vstrImageFilenamesD[ni],cv::IMREAD_UNCHANGED); //,cv::IMREAD_UNCHANGED);
        double tframe = vTimestamps[ni];

        if(imRGB.empty())
        {
            cerr << endl << "Failed to load image at: "
                 << string(argv[3]) << "/" << vstrImageFilenamesRGB[ni] << endl;
            return 1;
        }

        if(imageScale != 1.f)
        {
            int width = imRGB.cols * imageScale;
            int height = imRGB.rows * imageScale;
            cv::resize(imRGB, imRGB, cv::Size(width, height));
            cv::resize(imD, imD, cv::Size(width, height));
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
    }

    // Stop all threads
    SLAM.Shutdown();

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
    SLAM.SaveTrajectoryTUM("CameraTrajectory.txt");
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");   

    return 0;
}

void LoadImages(const string &strAssociationDFilename, const string &strAssociationRGBFilename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps)
{

    ifstream fAssociationD;
    fAssociationD.open(strAssociationDFilename.c_str());

    ifstream fAssociationRGB;
    fAssociationRGB.open(strAssociationRGBFilename.c_str());

    while (!fAssociationD.eof()) {

        std::string s_d;
        getline(fAssociationD, s_d);
        if (!s_d.empty()) {
            std::stringstream ss;
            ss << s_d;
            double t;
            std::string sD;
            ss >> t;
            ss >> sD;
            vstrImageFilenamesD.push_back(sD);
            std::cout << sD << std::endl;

            std::string s;
            getline(fAssociationRGB, s);
            if (!s.empty()) {
                std::stringstream ss;
                ss << s;
                double t;
                std::string sRGB;
                ss >> t;
                vTimestamps.push_back(t);
                ss >> sRGB;
                vstrImageFilenamesRGB.push_back(sRGB);
                std::cout << sRGB << std::endl;
            }
        }
    }

    std::cout << "Depth: " << vstrImageFilenamesD.size() << std::endl;
    std::cout << "RGB: " << vstrImageFilenamesRGB.size() << std::endl;
}
