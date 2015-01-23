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
#include <ip_msgs/markerAction.h>
#include <ip_msgs/heartAction.h>


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
    Point medianPoint;
    int medianx, mediany;
    ip_msgs::heartFeedback _feed;
    ip_msgs::heartResult res;
    Point imgCenter;
    float radius;
    Scalar low, high;
    image_transport::ImageTransport _it;
    image_transport::Publisher _pub;
    vector<vector<Point> > contours;

    Mat image, hsv,_fimg,drawing, trackimg,thresh,yuv, _kernel,dilate_img,gray,image2,gray2;


public:
    heart_task();
    void detectHeart();
    void on_trackbar(int, void*);
    void alignCenter();
    void serverCallback(const ip_msgs::heartGoalConstPtr &goal);
//    void imageCallback(const sensor_msgs::ImageConstPtr& ptr);

    ~heart_task();

};


#endif HEART_TASK_h
