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
#include <opencv2/imgproc/imgproc.hpp>
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>

using namespace std;
using namespace cv;
int slider = 100;
const int max_slider = 800;


Mat img_1, img_2;
void on_trackbar(int, void*)
{
    SurfFeatureDetector detector( slider );
      std::vector<KeyPoint> keypoints_1, keypoints_2;
      detector.detect( img_1, keypoints_1 );
      detector.detect( img_2, keypoints_2 );

      Mat img_keypoints_1; Mat img_keypoints_2;
      drawKeypoints( img_1, keypoints_1, img_keypoints_1, Scalar::all(-1), DrawMatchesFlags::DEFAULT );
      drawKeypoints( img_2, keypoints_2, img_keypoints_2, Scalar::all(-1), DrawMatchesFlags::DEFAULT );

      imshow("Keypoints 1", img_keypoints_1 );
      imshow("Keypoints 2", img_keypoints_2 );
}



int main(int argc, char** argv){
      img_1 = imread( "/home/manvi/ros_ws/heartTask/bin/heart.jpg", CV_LOAD_IMAGE_COLOR );
      img_2 = imread( "/home/manvi/Desktop/heart3.jpg", CV_LOAD_IMAGE_COLOR );
      if( !img_1.data || !img_2.data )
      { std::cout<< " --(!) Error reading images " << std::endl; return -1; }


      namedWindow("Keypoints 1",1);
      createTrackbar("minhessian","Keypoints 1",&slider, max_slider, on_trackbar);
      on_trackbar(slider,0);
      waitKey(0);
      return 0;
}
