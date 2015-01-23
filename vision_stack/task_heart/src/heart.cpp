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

Mat img_1;



int main(int argc, char** argv){
    Mat _imageHSV,_green, _red, _merge, _median,_kernel;
    img_1 = imread( argv[1],1 );


    CBlobResult _blobs1,_blobs2;
    CBlob * _currentBlob1,*_currentBlob2;
    CBlob _cb;
    //              Scalar lowval(0,0,0), highval(20,255,255);
    _kernel = getStructuringElement( MORPH_RECT, Size( 3, 3 ) ,Point(-1,-1));
    cvtColor(img_1, _imageHSV, CV_BGR2HSV);
    imshow("img_1", _imageHSV);
    inRange(_imageHSV,Scalar(0,75,75),Scalar(40,255,255), _red);
    inRange(_imageHSV,Scalar(35,0,0),Scalar(75,255,255), _green);
    add(_green, _red, _merge);

    vector<vector<Point> > contours;
    adaptiveThreshold(_merge,_median,200,ADAPTIVE_THRESH_MEAN_C,THRESH_BINARY,13,0);
    erode(_median,_median,_kernel);
    dilate(_median,_median,_kernel);

    IplImage fimage = _median;
    _blobs1 = CBlobResult(&fimage,NULL,0);
    _blobs2 = CBlobResult(&fimage,NULL,0);
    //    cout << "blobs.. \n";
    _blobs1.Filter(_blobs1, B_EXCLUDE, CBlobGetArea(), B_GREATER, 100);
    _blobs2.Filter(_blobs2,B_INCLUDE,CBlobGetArea(), B_LESS, 40);

    //    cout << "1";
    for(int i=0; i< _blobs1.GetNumBlobs();i++){
        _currentBlob1 = _blobs1.GetBlob(i);
        _currentBlob1->FillBlob(&fimage,Scalar(255));
    }

    for (int i = 0; i < _blobs2.GetNumBlobs(); i++)
    {
        _currentBlob2=_blobs2.GetBlob(i);
        _currentBlob2->FillBlob(&fimage,Scalar(0));
    }

    Mat fmat(&fimage);

    imshow("after blob detection",fmat);



    findContours(_merge, contours,CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
    vector<vector<Point> > polygon(contours.size());
    vector<RotatedRect> _rect(contours.size());

    vector<double> _contourarea;
    vector<Point> _fcontour;
    double max=0;
    for (int i = 0; i < contours.size(); ++i) {
        if(contourArea(contours[i]) > 200)
        {
            approxPolyDP(contours[i],polygon[i],3,true);
            _rect[i] = minAreaRect(Mat(polygon[i]));
            Point2f points[4];
            _rect[i].points(points);
            rectangle(_merge,points[0],points[3],Scalar(255,0,0),2);
            if(contourArea(contours[i]) > max){
                max = contourArea(contours[i]);
                cout << "hello ";
                rectangle(_merge,points[0],points[3],Scalar(255,0,0),3);
            }
        }
    }

    imshow("red", _red);
    imshow("green",_green);
    imshow("final", _merge);
    waitKey(0);
    return 0;
}
