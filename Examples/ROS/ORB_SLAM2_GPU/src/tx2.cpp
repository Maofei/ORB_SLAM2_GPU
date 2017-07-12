#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <iomanip>
#include <ros/ros.h>

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

//#include <geometry_msgs/PoseStamped.h>
#include "geometry_msgs/PoseStamped.h"
#include <tf/transform_broadcaster.h>

#include "../../../include/System.h"
#include "../../../include/Converter.h"
#include "../../../include/Utils.hpp"

using namespace std;

#ifdef COMPILEDWITHC11
#define SET_CLOCK(t0) \
    std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
#else
#define SET_CLOCK(t0) \
    std::chrono::monotonic_clock::time_point t0 = std::chrono::monotonic_clock::now();
#endif

#define TIME_DIFF(t1, t0) \
    (std::chrono::duration_cast<std::chrono::duration<double>>((t1) - (t0)).count())

void PublishPose(ros::Publisher* pub, const cv::Mat& Tcw)
{
    geometry_msgs::PoseStamped poseMSG;
    if(!Tcw.empty())
    {

        cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
        cv::Mat twc = -Rwc * Tcw.rowRange(0,3).col(3);

        vector<float> q = ORB_SLAM2::Converter::toQuaternion(Rwc);

        /*
            cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
            cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);
            tf::Matrix3x3 M(Rwc.at<float>(0,0),Rwc.at<float>(0,1),Rwc.at<float>(0,2),
                            Rwc.at<float>(1,0),Rwc.at<float>(1,1),Rwc.at<float>(1,2),
                            Rwc.at<float>(2,0),Rwc.at<float>(2,1),Rwc.at<float>(2,2));
            tf::Vector3 V(twc.at<float>(0), twc.at<float>(1), twc.at<float>(2));

            tf::Transform tfTcw(M,V);

            //mTfBr.sendTransform(tf::StampedTransform(tfTcw,ros::Time::now(), "ORB_SLAM/World", "ORB_SLAM/Camera"));
        */
        poseMSG.pose.position.x = twc.at<float>(0);
        poseMSG.pose.position.y = twc.at<float>(2);
        poseMSG.pose.position.z = twc.at<float>(1);

        poseMSG.pose.orientation.x = q[0];
        poseMSG.pose.orientation.y = q[1];
        poseMSG.pose.orientation.z = q[2];
        poseMSG.pose.orientation.w = q[3];

        poseMSG.header.frame_id = "VSLAM";
        poseMSG.header.stamp = ros::Time::now();
        cout << "PublishPose position.x = " << poseMSG.pose.position.x << endl;

        pub->publish(poseMSG);

        //mlbLost.push_back(mState==LOST);
    }
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "tx2");
    //ros::start();
    ros::NodeHandle nh_;
    ros::Publisher pos_pub = nh_.advertise<geometry_msgs::PoseStamped>("ORB_SLAM/pose", 5);

    if(argc != 4) {
        cerr << endl
             << "Usage: rosrun ORB_SLAM2_GPU tx2 [path to vocabulary] [path to settings] [whether use Map]" << endl;
        ros::shutdown();
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    bool bReuseMap = false, bUseViewer = true;
    string MapName = "/home/nvidia/quemaofei/ORB_SLAM2_GPU/Examples/ROS/ORB_SLAM2_GPU/launch/default_map.bin";
    if(strcmp(argv[3], "false") != 0)
    {
        bReuseMap = true;
        MapName = argv[3];
    }

    ORB_SLAM2::System SLAM(
        argv[1], argv[2], ORB_SLAM2::System::MONOCULAR, bUseViewer, bReuseMap, MapName);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;

    // const char * gst = "nvcamerasrc ! video/x-raw(memory:NVMM), width=(int)1280, height=(int)720, format=(string)I420, framerate=(fraction)24/1 ! nvvidconv flip-method=2 ! videoconvert ! appsink";
    const char * gst = "nvcamerasrc ! video/x-raw(memory:NVMM), width=(int)960, height=(int)540, format=(string)I420, framerate=(fraction)30/1 ! nvvidconv flip-method=2 ! videoconvert ! appsink";
    cv::VideoCapture cap(gst);
    if (!cap.isOpened()) {
        printf("can not open camera or video file\n%s", gst);
        ros::shutdown();
        return -1;
    }


    // Main loop
    double trackTimeSum = 0.0;
    cv::Mat im;
    SET_CLOCK(t0);
    int frameNumber = 0;

    ros::Rate r(10);
    while (nh_.ok()) {
        cap >> im;
        if (im.empty()) continue;
        SET_CLOCK(t1);
        double tframe = TIME_DIFF(t1, t0);

        PUSH_RANGE("Track image", 4);
        // Pass the image to the SLAM system
        cv::Mat Tcw = SLAM.TrackMonocular(im, tframe);
        PublishPose(&pos_pub, Tcw);
        POP_RANGE;
        SET_CLOCK(t2);

        double trackTime = TIME_DIFF(t2, t1);
        trackTimeSum += trackTime;
        //cout << "Frame Number" << frameNumber << " : Frame Time: " << tframe
             //<< " Track Time: " << trackTime << "\n";
        ++frameNumber;
        ros::spinOnce();
        r.sleep();
    }
    // Stop all threads
    SLAM.Shutdown();
    SLAM.SaveMap(MapName);

    cout << "Mean track time: " << trackTimeSum / frameNumber <<"\n";
    ros::shutdown();

    return 0;
}
