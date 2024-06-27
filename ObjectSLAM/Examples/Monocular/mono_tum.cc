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

#include<opencv2/core/core.hpp>

#include<System.h>

using namespace std;

void LoadImages(const string &strFile, vector<string> &vstrImageFilenames, vector<string> &vstrBoundingBoxNames, vector<string> &vstrLineNames, 
                vector<double> &vTimestamps);

void LoadBBOrLines(const string &strBBTxt, vector< vector<double> > &vBoundingBoxes);

int main(int argc, char **argv)
{
    if(argc != 4)
    {
        cerr << endl << "Usage: ./mono_tum path_to_vocabulary path_to_settings path_to_sequence" << endl;
        return 1;
    }

    // Retrieve paths to images
    vector<string> vstrImageFilenames;
    vector<string> vstrBoundingBoxNames;
    vector<string> vstrLineNames;
    vector<double> vTimestamps;
    string strFile = string(argv[3])+"/rgb.txt";
    LoadImages(strFile, vstrImageFilenames, vstrBoundingBoxNames, vstrLineNames, vTimestamps);

    int nImages = vstrImageFilenames.size();

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    // Main loop
    cv::Mat im;
    vector<vector<double>> vBoundingBoxes;
    vector<vector<double>> vLines;
    for(int ni=0; ni<nImages; ni++)//114 //827
    {
        if(ni%5 != 0 && ni > 3)
        {
            continue;
        }
        cout <<endl << endl << vstrImageFilenames[ni] << endl;
        // Read image from file
        im = cv::imread(string(argv[3])+"/"+vstrImageFilenames[ni],CV_LOAD_IMAGE_UNCHANGED);
        double tframe = vTimestamps[ni];
        // string strBBName = string(argv[3])+"/"+vstrBoundingBoxNames[ni];
        // string strLinesName = string(argv[3])+"/"+vstrLineNames[ni];
        string strBBName = vstrBoundingBoxNames[ni];
        string strLinesName = vstrLineNames[ni];
        // 清空vBoundingBoxes和vLines
        for (int vBBi=0; vBBi<(int)vBoundingBoxes.size(); vBBi++)
        {
            vBoundingBoxes[vBBi].clear();
        }
        vBoundingBoxes.clear();
        for (int vLi=0; vLi<(int)vLines.size(); vLi++)
        {
            vLines[vLi].clear();
        }
        vLines.clear();

        LoadBBOrLines(strBBName, vBoundingBoxes);
        LoadBBOrLines(strLinesName, vLines);

        if(im.empty())
        {
            cerr << endl << "Failed to load image at: "
                 << string(argv[3]) << "/" << vstrImageFilenames[ni] << endl;
            return 1;
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the image to the SLAM system
        // cout << "Tracking"<< endl;
        SLAM.TrackMonocular(im,tframe, vBoundingBoxes, vLines);

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
        usleep(100000);

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
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    return 0;
}

void LoadImages(const string &strFile, vector<string> &vstrImageFilenames, vector<string> &vstrBoundingBoxNames, vector<string> &vstrLineNames, vector<double> &vTimestamps)
{
    ifstream f;
    f.open(strFile.c_str());

    // skip first three lines
    string s0;
    getline(f,s0);
    getline(f,s0);
    getline(f,s0);

    while(!f.eof())
    {
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
            vstrBoundingBoxNames.push_back("./data/TUM-box/"+sRGB.substr(4,sRGB.size()-8)+".txt");
            vstrLineNames.push_back("./data/TUM-lines/"+sRGB.substr(4,sRGB.size()-8)+".txt");
        }
    }
}

void LoadBBOrLines(const string &strBBTxt, vector< vector<double> > &vBoundingBoxes)
{
    ifstream f;
    f.open(strBBTxt.c_str());
    while(!f.eof())
    {
        string s;
        getline(f, s);
        if(!s.empty())
        {
            vector<double> vTemp;
            stringstream ss;
            ss << s;
            double t0, t1, t2, t3, t4, t5;
            ss>>t0;
            vTemp.push_back(t0);
            ss>>t1;
            vTemp.push_back(t1);
            ss>>t2;
            vTemp.push_back(t2);
            ss>>t3;
            vTemp.push_back(t3);
            ss>>t4;
            vTemp.push_back(t4);
            ss>>t5;
            vTemp.push_back(t5);
            vBoundingBoxes.push_back(vTemp);
        }
    }
}