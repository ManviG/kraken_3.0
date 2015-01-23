#ifndef BIN_TASK_H
#define BIN_TASK


#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <cmath>

///!! blob libraries
///
#include <blob/blob.h>
#include <blob/BlobContour.h>
#include <blob/BlobLibraryConfiguration.h>
#include <blob/BlobOperators.h>
#include <blob/BlobProperties.h>
#include <blob/BlobResult.h>
#include <blob/ComponentLabeling.h>


///! libraries for opencv
///
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


///!action server and client
///
#include <actionlib/server/simple_action_server.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <sensor_msgs/image_encodings.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <binTask/binResult.h>
#include <ip_msgs/binAction.h>
#include <binTask/binGoal.h>
#include <resources/topicHeader.h>


using namespace std;
using namespace cv;
using namespace ros;

typedef actionlib::SimpleActionServer<ip_msgs::binAction> Server;

std_msgs::Header _h;

class bin_task{
private:
    NodeHandle _n;

    ip_msgs::binFeedback _feed;
    ip_msgs::binResult res;

    Point medianPoint;
    int medianx, mediany;
    ip_msgs::binFeedback _feed;
    ip_msgs::binResult res;
    Point imgCenter;
    float radius;
    Scalar low, high;
    image_transport::ImageTransport _it;
    image_transport::Publisher _pub;
    Server serv;

    image_transport::Subscriber _imgsub;
    vector<vector<Point> > contours;
    Mat image, hsv,_fimg,drawing, trackimg,thresh,yuv,dilate_img,gray,image2,gray2;
    Mat img_1,_imageHSV,_green, _red, _merge, _median,_kernel, I2;
    CBlobResult _blobs1,_blobs2;
    CBlob * _currentBlob1,*_currentBlob2;
    CBlob _cb;
    vector<double> _contourarea;
    vector<Point> _fcontour;
    RotatedRect _frect;
    double max=0;
    Point2f _fpoints[4], median, imgCenter, _error;
    bool bin_detect_status, _status;

    vector<vector<Point> > contours;

    Mat image, hsv,_fimg,drawing, trackimg,thresh,yuv, _kernel,dilate_img,gray,image2,gray2;



public:

    bin_task();
    bool detectbin();
    void imageCallback(const sensor_msgs::ImageConstPtr &ptr);
    void alignbin();
    void serverCallback(const ip_msgs::binGoalConstPtr &goal);
    ~bin_task();

};


#endif bin_TASK_h
