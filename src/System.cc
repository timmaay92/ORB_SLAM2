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



#include "System.h"
#include "Converter.h"
#include <thread>
#include <iomanip>

static bool has_suffix(const std::string &str, const std::string &suffix)
{
    std::size_t index = str.find(suffix, str.size() - suffix.size());
    return (index != std::string::npos);
}

namespace ORB_SLAM2
{

System::System(const string &strVocFile, const string &strSettingsFile, const eSensor sensor,
              bool is_save_map_):mSensor(sensor), is_save_map(is_save_map_), mbReset(false)
{
    //Check settings file
    cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
       cerr << "Failed to open settings file at: " << strSettingsFile << endl;
       exit(-1);
    }
    useOdometry = fsSettings["Initializer.UseOdometry"];


    // Output welcome message
    cout << endl <<
    "ORB-SLAM2 Copyright (C) 2014-2016 Raul Mur-Artal, University of Zaragoza." << endl <<
    "This program comes with ABSOLUTELY NO WARRANTY;" << endl  <<
    "This is free software, and you are welcome to redistribute it" << endl <<
    "under certain conditions. See LICENSE.txt." << endl << endl;

    cout << "Input sensor was set to: ";

    if(mSensor==MONOCULAR)
    {
        if(useOdometry)
            cout << "Monocular with Odometry" << endl;
        else
            cout << "Monocular" << endl;
    }
    else if(mSensor==STEREO)
        cout << "Stereo" << endl;
    else if(mSensor==RGBD)
        cout << "RGB-D" << endl;




    cv::FileNode mapfilen = fsSettings["Map.mapfile"];
    bool bReuseMap = false;
    if (!mapfilen.empty())
    {
        mapfile = (string)mapfilen;
    }

    //Load ORB Vocabulary
    cout << endl << "Loading ORB Vocabulary. This could take a while..." << endl;

    mpVocabulary = new ORBVocabulary();
    bool bVocLoad = false; // chose loading method based on file extension
    if (has_suffix(strVocFile, ".txt"))
        bVocLoad = mpVocabulary->loadFromTextFile(strVocFile);
    else if(has_suffix(strVocFile, ".bin"))
        bVocLoad = mpVocabulary->loadFromBinaryFile(strVocFile);
    else
        bVocLoad = false;
    if(!bVocLoad)
    {
        cerr << "Wrong path to vocabulary. " << endl;
        cerr << "Falied to open at: " << strVocFile << endl;
        exit(-1);
    }
    cout << "Vocabulary loaded!" << endl << endl;


    //Create KeyFrame Database
    //Create the Map
    if (!mapfile.empty() && LoadMap(mapfile))
    {
        bReuseMap = true;
    }
    else
    {
        mpKeyFrameDatabase = new KeyFrameDatabase(mpVocabulary);
        mpMap = new Map();
    }

//    //Create Drawers. These are used by the Viewer
//    mpFrameDrawer = new FrameDrawer(mpMap, bReuseMap);
//    mpMapDrawer = new MapDrawer(mpMap, strSettingsFile);

    //Initialize the Tracking thread
    //(it will live in the main thread of execution, the one that called this constructor)
    mpTracker = new Tracking(this, mpVocabulary,
                             mpMap, mpKeyFrameDatabase, strSettingsFile, mSensor, bReuseMap);
    if(bReuseMap)
        mpTracker->MapReloaded= true;


    //Initialize the Local Mapping thread and launch
//    mpLocalMapper = new LocalMapping(mpMap, strSettingsFile, mSensor==MONOCULAR);
//    mptLocalMapping = new thread(&ORB_SLAM2::LocalMapping::Run,mpLocalMapper);

    //Initialize the Loop Closing thread and launch
//    mpLoopCloser = new LoopClosing(mpMap, mpKeyFrameDatabase, mpVocabulary, true);
//    mptLoopClosing = new thread(&ORB_SLAM2::LoopClosing::Run, mpLoopCloser);


    //Set pointers between threads
//    mpTracker->SetLocalMapper(mpLocalMapper);
//    mpTracker->SetLoopClosing(mpLoopCloser);

//    mpLocalMapper->SetTracker(mpTracker);
//    mpLocalMapper->SetLoopCloser(mpLoopCloser);

//    mpLoopCloser->SetTracker(mpTracker);
//    mpLoopCloser->SetLocalMapper(mpLocalMapper);
}

cv::Mat System::TrackStereo(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timestamp)
{

    if(mSensor!=STEREO)
    {
        cerr << "ERROR: you called TrackStereo but input sensor was not set to STEREO." << endl;
        exit(-1);
    }



    // Check reset
    {
    unique_lock<mutex> lock(mMutexReset);
    if(mbReset)
    {
        mpTracker->Reset();
        mbReset = false;
    }
    }

    cv::Mat Tcw = mpTracker->GrabImageStereo(imLeft,imRight,timestamp);

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;
    return Tcw;

}

cv::Mat System::TrackRGBD(const cv::Mat &im, const cv::Mat &depthmap, const double &timestamp)
{

    if(mSensor!=RGBD)
    {
        cerr << "ERROR: you called TrackRGBD but input sensor was not set to RGBD." << endl;
        exit(-1);
    }



    // Check reset
    {
    unique_lock<mutex> lock(mMutexReset);
    if(mbReset)
    {
        mpTracker->Reset();
        mbReset = false;
    }
    }

    cv::Mat Tcw = mpTracker->GrabImageRGBD(im,depthmap,timestamp);

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;
    return Tcw;

}

cv::Mat System::TrackMonocular(const cv::Mat &im, const double &timestamp)
{
    if(mSensor!=MONOCULAR)
    {
        cerr << "ERROR: you called TrackMonocular but input sensor was not set to Monocular or Odometry." << endl;
        exit(-1);
    }

    mpTracker->InformOnlyTracking(true);


    // Check reset
    {
    unique_lock<mutex> lock(mMutexReset);
    if(mbReset)
    {
        mpTracker->Reset();
        mbReset = false;
    }
    }


    cv::Mat Tcw;
    if(useOdometry)
        Tcw = mpTracker->GrabImageMonocular(im,timestamp, mTF_w_c);
    else
        Tcw = mpTracker->GrabImageMonocular(im,timestamp);

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;

    if(!Tcw.empty())
    {
        // convert from orb world frame to maqui world frame
        g2o::SE3Quat O_w_c = mpMap->GetInitialPose();
        g2o::SE3Quat mT_c_w = Converter::toSE3Quat(Tcw.clone());

        cv::Mat mT_w_c = Converter::toCvMat(O_w_c.inverse() * mT_c_w.inverse());
        return mT_w_c;
    }
    else
    {
        return Tcw;
    }
}


bool System::MapChanged()
{
    static int n=0;
    int curn = mpMap->GetLastBigChangeIdx();
    if(n<curn)
    {
        n=curn;
        return true;
    }
    else
        return false;
}

void System::Reset()
{
    unique_lock<mutex> lock(mMutexReset);
    mbReset = true;
}

void System::Shutdown()
{
    if (is_save_map)
        SaveMap(mapfile);
}

int System::GetTrackingState()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackingState;
}

vector<MapPoint*> System::GetTrackedMapPoints()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackedMapPoints;
}

vector<cv::KeyPoint> System::GetTrackedKeyPointsUn()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackedKeyPointsUn;
}
void System::SetOdomPose(const cv::Mat& T_w_c)
{
    cv::Mat mTf_w_c = T_w_c.clone();
    mTF_w_c = Converter::toSE3Quat(mTf_w_c);
}

void System::SaveMap(const string &filename)
{
    std::ofstream out(filename, std::ios_base::binary);
    if (!out)
    {
        cerr << "Cannot Write to Mapfile: " << mapfile << std::endl;
        exit(-1);
    }
    cout << "Saving Mapfile: " << mapfile << std::flush;
    boost::archive::binary_oarchive oa(out, boost::archive::no_header);
    oa << mpMap;
    oa << mpKeyFrameDatabase;
    cout << " ...done" << std::endl;
    out.close();
}
bool System::LoadMap(const string &filename)
{
    std::ifstream in(filename, std::ios_base::binary);
    if (!in)
    {
        cerr << "Cannot Open Mapfile: " << mapfile << " , Create a new one" << std::endl;
        return false;
    }
    cout << "Loading Mapfile: " << mapfile << std::flush;
    boost::archive::binary_iarchive ia(in, boost::archive::no_header);
    ia >> mpMap;
    ia >> mpKeyFrameDatabase;
    mpKeyFrameDatabase->SetORBvocabulary(mpVocabulary);
    cout << " ...done" << std::endl;
    cout << "Map Reconstructing" << flush;
    vector<ORB_SLAM2::KeyFrame*> vpKFS = mpMap->GetAllKeyFrames();
    unsigned long mnFrameId = 0;
    for (auto it:vpKFS) {
        it->SetORBvocabulary(mpVocabulary);
        it->ComputeBoW();
        if (it->mnFrameId > mnFrameId)
            mnFrameId = it->mnFrameId;
    }
    Frame::nNextId = mnFrameId;
    cout << " ...done" << endl;
    in.close();
    return true;
}

} //namespace ORB_SLAM
