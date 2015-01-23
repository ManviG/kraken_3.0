#include<iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv/cv.h>

using namespace std;
using namespace cv;

Mat thresh;

Point2f computeIntersect(Vec4i a, Vec4i b)
{
    int x1 = a[0], y1 = a[1], x2 = a[2], y2 = a[3];
    int x3 = b[0], y3 = b[1], x4 = b[2], y4 = b[3];

    if (float d = ((float)(x1-x2) * (y3-y4)) - ((y1-y2) * (x3-x4)))
    {
        Point2f pt;
        pt.x = ((x1*y2 - y1*x2) * (x3-x4) - (x1-x2) * (x3*y4 - y3*x4)) / d;
        pt.y = ((x1*y2 - y1*x2) * (y3-y4) - (y1-y2) * (x3*y4 - y3*x4)) / d;
        return pt;
    }
    else
        return Point2f(-1, -1);
}


struct bunch{
    vector<Vec4i> v;
    Point pt;
};


vector<Vec3f> circles;


void detectCircle(){
    HoughCircles(thresh,circles,CV_HOUGH_GRADIENT,1,thresh.rows/16,150,25,0,0);
    if(circles.size() == 0)
        cout << "nothing circular " << endl;

    for (size_t i = 0; i < circles.size(); ++i) {
        Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
        int radius = cvRound(circles[i][2]);
        // circle center
        circle( thresh, center, 3, Scalar(0, 255, 0), 3, 8, 0 );
        // circle outline
        circle( thresh, center, radius, Scalar(0, 0, 255), 1, 8, 0 );
    }
}




int main(int argc, char** argv)
{

    Mat orgimg, finalimg, hsv,edges;
    double slope;
    vector<Vec4i> _linesx, _linesy;

    Scalar lower(0,127,127), upper(255,255,255);
    orgimg = imread(argv[1],1);

    if(!orgimg.data)
    {
        cout << "no image loaded..\n";
        return 0;
    }
    Mat newimg(orgimg.rows, orgimg.cols, CV_8UC3, Scalar(0,0,255));
    vector<vector<Point> > contours;
    vector<vector<Point> > polygon;
    cvtColor(orgimg,hsv,CV_BGR2HSV);
    vector<Vec4i> lines;
    inRange(hsv,lower, upper,thresh);

    imshow("inrange thresh",thresh);
    if(!hsv.data && !thresh.data)
        return 0;



    findContours(thresh,contours,CV_RETR_CCOMP,CV_CHAIN_APPROX_SIMPLE,Point(0,0));

    vector<RotatedRect> _rect(contours.size());
    RotatedRect _rr;
    _rr = minAreaRect(contours[0]);
    Point2f pp[4];
    _rr.points(pp);
    rectangle(orgimg,pp[0],pp[3],Scalar(255,255,0),4);
    Point2f points[4];
    RotatedRect _rrt;
    for(size_t i=0; i<contours.size(); i++){
        cout << "rectangle.. ";
        _rect[i] = minAreaRect(contours[i]);
        _rect[i].points(points);
        rectangle(orgimg,points[0],points[3],Scalar(255,0,0),2);
        imshow("orgimg", orgimg);
        waitKey(0);
    }



    polygon.resize(contours.size());

    cout << contours.size() << " = contours size " << endl;
    for (int i = 0; i < contours.size(); ++i) {
        approxPolyDP(Mat(contours[i]),polygon[i],3,true);
        drawContours(thresh,polygon,i,Scalar(255,0,0),2);
    }


    ///! checking number of corners in the contour
    ///! number of corners  should be 12 ...

    cout << polygon.size() << endl;
    for (int i = 0; i < polygon.size(); ++i) {
        if(!isContourConvex(polygon[i]))
            cout << "polygon[" << i<<"] is not convex.." <<endl;
    }
    cout << "polygon 0 ka size = " << polygon[0].size() << endl;
    vector<Point2f> corners;


    Canny(thresh,thresh,20,200,3);
    HoughLinesP(thresh,lines,1,CV_PI/180,20,20,10);

    cout << lines.size() << " = number of lines " << endl;

    double m1, m2, angle;
    int count=0;
    for (int i = 0; i < lines.size() - 1; ++i) {

        //        line(newimg,Point(lines[i][0],lines[i][1]),Point(lines[i][2],lines[i][3]),Scalar(255,255,0),1);
        if(lines[i][2]-lines[i][0] !=0){
            slope = atan((double)(lines[i][3]-lines[i][1])/(double)(lines[i][2]-lines[i][0]));
            //            cout << "slope for line[" << i<<"] = " << slope << endl;
            //            line(newimg,Point(lines[i][0],lines[i][1]),Point(lines[i][2],lines[i][3]),Scalar(255,0,255),1);
        }
        else{
            slope = 90*CV_PI/180;
            //            line(newimg,Point(lines[i][0],lines[i][1]),Point(lines[i][2],lines[i][3]),Scalar(255,0,0),1);
        }



        if(slope<30*CV_PI/180  || slope>150*CV_PI/180){
            //            _linesx.push_back(lines[i]);
            cout << "slope of line drawn in x = " << slope << endl;
            line(newimg,Point(lines[i][0],lines[i][1]),Point(lines[i][2],lines[i][3]),Scalar(255,0,0),2);
        }
        else if(slope>70*CV_PI/180 && slope<120*CV_PI/180){
            //            _linesy.push_back(lines[i]);
            cout << "slope of line drawn in y = " << slope << endl;
            line(orgimg,Point(lines[i][0],lines[i][1]),Point(lines[i][2],lines[i][3]),Scalar(255,255,0),2);
        }
    }




    ///! find intersection points of lines
    for (int i = 0; i < lines.size(); i++)
    {
        for (int j = i+1; j < lines.size(); j++)
        {
            Point2f pt = computeIntersect(lines[i], lines[j]);
            if (pt.x >= 0 && pt.y >= 0)
                corners.push_back(pt);
            circle(newimg,corners[i],5,Scalar(255,0,255),2);
        }
    }

    cout << "corners size = " << corners.size() << endl;

    imshow("win", newimg);
    imshow("orgimg", orgimg);
    imshow("thresh", thresh);
    waitKey(0);
    return 0;
}

