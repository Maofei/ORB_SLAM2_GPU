#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <iomanip>
#include <ros/ros.h>

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "../../../include/System.h"
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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tx1");
    //ros::start();
    ros::NodeHandle nh_;

    if(argc != 3) {
        cerr << endl << "Usage: rosrun ORB_SLAM2_GPU tx1 [path to vocabulary] [path to settings]" << endl;
        ros::shutdown();
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);

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

    //double ttl = atof(argv[3]);

    double trackTimeSum = 0.0;
    // Main loop
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
      SLAM.TrackMonocular(im, tframe);
      POP_RANGE;
      SET_CLOCK(t2);

      double trackTime = TIME_DIFF(t2, t1);
      trackTimeSum += trackTime;
      cout << "Frame Number" << frameNumber << " : Frame Time: " << tframe
           << " Track Time: " << trackTime << "\n";
      ++frameNumber;
      ros::spinOnce();
      r.sleep();
    }
    // Stop all threads
    SLAM.Shutdown();

    cerr << "Mean track time: " << trackTimeSum / frameNumber <<"\n";

    return 0;
}
