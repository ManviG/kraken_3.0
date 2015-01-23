#ifndef HEART_TASK_h
#define HEART_TASK


#include <iostream>
#include <fstream>
#include <cmath>
#include <blob/blob.h>
#include <blob/BlobContour.h>
#include <blob/BlobLibraryConfiguration.h>
#include <blob/BlobOperators.h>
#include <blob/BlobProperties.h>
#include <blob/BlobResult.h>
#include <blob/ComponentLabeling.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <heartTask/heartGoal.h>
#include <heartTask/heartResult.h>
#include <ip_msgs/heartAction.h>
#include <resources/topicHeader.h>


using namespace std;
using namespace cv;
using namespace ros;



typedef actionlib::SimpleActionServer<ip_msgs::heartAction> Server;

std_msgs::Header _h;
Mat image, hsv, trackimg;
int slider_max = 255;
int slider = 0;



class heart_task{
private:
    NodeHandle _n;
    Server serv;
    //    int _errorx, mediany;
    ip_msgs::heartFeedback _feed;
    ip_msgs::heartResult res;
    float radius;
    Scalar low, high;
    image_transport::ImageTransport _it;
    image_transport::Publisher _pub;
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
    bool heart_detect_status, _status;


public:
    heart_task();
    bool detectHeart();
    void imageCallback(const sensor_msgs::ImageConstPtr &ptr);
    void alignCenter();
    void serverCallback(const ip_msgs::heartGoalConstPtr &goal);
    //    void imageCallback(const sensor_msgs::ImageConstPtr& ptr);

    ~heart_task();

};


#endif HEART_TASK_h
