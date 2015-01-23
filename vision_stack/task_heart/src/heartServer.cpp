#ifndef HEART_TASK_cpp
#define HEART_TASK_cpp

#include <heartTask/heartHeader.h>

#include <heartTask/heartGoal.h>
#include <heartTask/heartResult.h>
// #include <heartTask/heartAction.h>

heart_task::heart_task()
    :serv(_n,"heart_server",boost::bind(&heart_task::serverCallback,this,_1),false), _it(_n)
{
    image = imread("/home/manvi/ros_ws/heartTask/bin/heart.jpg" , CV_LOAD_IMAGE_COLOR);
    if(!image.data){
        cout << "image canot be opened";
        shutdown();
    }
    medianx= 0;
    mediany = 0;

    imgCenter.x = image.rows/2;
    imgCenter.y = image.cols/2;

    ifstream _ifo("/home/manvi/ros_ws/heartTask/threshold.th",ios::in);
        if(_ifo.is_open())
            {
                for(int i=0; i<3;i++)
                    _ifo >> low[i];
                for(int i=0;i<3;i++)
                    _ifo >> high[i];
            }
        else
            {
                cout << "threshold file could not be opened.." << endl;
                shutdown();
            }
    _kernel = getStructuringElement(MORPH_RECT, Size(3,3),Point(-1,-1));
    _pub = _it.advertise("heart_server",1);
    serv.start();
    ROS_INFO("waiting for clients");
    imshow("win", image);

//    if(waitKey(33)==27)
//        cout << "end process";
}


void heart_task::detectHeart(){
    _fimg = Mat::zeros(image.size(),CV_8UC3);
    drawing = Mat::zeros(image.size(),CV_8UC3);

    cvtColor(image, hsv, CV_BGR2HSV_FULL);
//    createTrackbar("cvtcolor","hsv image",&slider,slider_max,on_trackbar);

    cvtColor(image,gray,CV_BGR2GRAY);
    inRange(hsv,low,high,thresh);
    medianBlur(thresh,thresh,3);
    dilate(thresh,dilate_img,_kernel);

    findContours(dilate_img,contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE,Point(0,0));
    cout << "contours size" <<contours.size() << endl;
    vector<vector<Point> > temp(contours.size());
    vector<vector<Point> > hull(contours.size());
    vector<int> _medianx(contours.size(),0), _mediany(contours.size(),0);
    cout << _medianx.size();
//    vector<Vec3f> circles;
        drawContours(_fimg,contours,-1,Scalar(255,255,255),1,8);
        for(int i=0; i<contours.size(); i++){
            cout << "contours.size = " << i << endl;
            convexHull(Mat(contours[i]),hull[i],false);
        }
//        drawContours( drawing, hull, -1, Scalar(255,255,255), 1, 8);
//        cout <<"hull size"<< hull.size() << endl;
        for(int i=0; i<hull.size(); i++){
            cout << "size of hull [" << i << "]" << hull[i].size() << endl;
        }
//        cout << "hull_size = " << hull[0].size() << endl;
        for(int i=0; i<hull.size(); i++){
            for(int j=0; j<hull[i].size(); j++){
                cout << "hull["<<i<<"]["<< j << "].x = " << hull[i][j].x << endl;
                _medianx[i] += hull[i][j].x;
                _mediany[i] += hull[i][j].y;
            }
            _medianx[i] = _medianx[i]/hull[i].size();
            _mediany[i] = _mediany[i]/hull[i].size();
            cout << "_medianx["<<i <<"] = " << _medianx[i] << endl;
            cout << "_medianx["<<i <<"] = " << _medianx[i] << endl;
        }


        for(int i=0; i<hull[0].size(); i++){
            cout << hull[0][i].x << endl;
            medianx += hull[0][i].x ;
            mediany += hull[0][i].y ;
        }


        medianx = medianx/hull[0].size();
        mediany = mediany/hull[0].size();
        cout << "\n median x of heart = " << medianx << endl;
        cout << "\n median y of heart = " << mediany << endl;
        circle(image,Point(medianx,mediany),5,Scalar(255,255,255),2,8,0);

    for(int i=0; i<contours.size(); i++){
        approxPolyDP(contours[i],temp[i],7, true);
        cout << contourArea(temp[i]) <<endl;
    }

   // drawContours(thresh,temp,0,Scalar(127,150,0));
 //   HoughCircles(gray2,circles,CV_HOUGH_GRADIENT,1,2,200,100);
//    cout << circles.size() <<endl;

//    for( size_t i=0; i<circles.size(); i++){
//       // cout << "circles.Size" << circles.size();
//        Point center(cvRound(circles[i][0]),cvRound(circles[i][1]));
//        float radius = cvRound(circles[i][2]);
//        circle(gray2,center,3,Scalar(127,100,100));
//        circle(gray2,center,radius,Scalar(127,100,100));
//    }
//    minEnclosingCircle(temp[0],center[0],radius[0]);
//    circle(thresh,center[0],radius[0],Scalar(127,0,0));
    imshow("original" , image);
    imshow("hsv", hsv);
//    imshow("")
    imshow("win", thresh);
//    imshow("win2", gray2);
    imshow("dilate", dilate_img);
    imshow("contours",_fimg);
    imshow("drawing", drawing);

    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(),"bgr8",image).toImageMsg();
//   sensor_msgs::ImagePtr msg=object.toImageMsg();
           _pub.publish(msg);
           imshow("_fimg",_fimg);
           if(waitKey(33)==27)
             return;
}

void heart_task::alignCenter(){
//    ROS_INFO("center.x and center.y are : %f %f ", medianx,mediany);
    _feed.errorx = medianx-(image.rows/2);
    _feed.errory = mediany-(image.cols/2);

    radius = sqrt(pow((medianx-imgCenter.x),2)+pow((mediany-imgCenter.y),2));
    if(radius<10){
        if(medianx - imgCenter.x<0 && mediany - imgCenter.y>0)
            ROS_INFO("GO RIGHT AND THEN DOWNWARDS");
        else if(medianx- imgCenter.x >0 && mediany -imgCenter.y >0)
            ROS_INFO("GO LEFT AND THEN DOWNWARDS");
        else if (medianx -imgCenter.x <0 && mediany-imgCenter.y <0) {
            ROS_INFO("GO right and the upwards");
        }
        else if(medianx -imgCenter.x >0 && mediany-imgCenter.y <0)
            ROS_INFO("go left and then upwards");
    }

//   ROS_INFO("error x and error y : %f %f", _feed.errorx, _feed.errory);
}


void heart_task::serverCallback(const ip_msgs::heartGoalConstPtr &goal)
{
    Rate looprate(10);
    switch (goal->heart) {
    case DETECT_CENTER:
    {
        ROS_INFO("start detecting validation gate...");
        while(ok()){
            if(serv.isPreemptRequested())
            {
                res.centerx = medianx;
                res.centery = mediany;
                serv.setPreempted(res);
                break;
            }
            detectHeart();

            if((medianx - (image.rows/2))< 10 && (mediany - (image.cols/2))< 10 ){
                _feed.hfeed = GO_STRAIGHT;
                res.hresult = CENTER_DETECTED;
                serv.setSucceeded(res);
            }
            serv.publishFeedback(_feed);
            if(medianx == 0 && mediany == 0)
                res.hresult = NOT_DETECTED;
            else{
                res.centerx = medianx;
                res.centery = mediany;
                res.hresult = CENTER_DETECTED;
                serv.setSucceeded(res);
            }
            looprate.sleep();
             medianx =0;
             mediany =0;
        }
       break;
    }


    case ALIGN_CENTER:
    {
        ROS_INFO("start aligning..!!!");
        while(ok()){
            if(serv.isPreemptRequested()){
                res.centerx = medianx;
                res.centery = mediany;
                serv.setPreempted(res);
                break;
            }

            detectHeart();

            if(abs(imgCenter.x-medianx) < 10 && abs(imgCenter.y-mediany) < 10){
                _feed.hfeed = GO_STRAIGHT;
                res.hresult = CENTER_DETECTED;
                serv.setSucceeded(res);
            }

            else{
                alignCenter();
                if(_feed.errorx <= 10 && _feed.errory <=10)
                   {
                       res.centerx = medianx;
                       res.centery = mediany;
                       res.hresult = CENTER_ALIGNED;
                       serv.setSucceeded(res);
                       break;
                   }

            }
            serv.publishFeedback(_feed);
            looprate.sleep();
        }
        break;
    }
    }
}


//void heartTask::imageCallback(const sensor_msgs::ImageConstPtr &ptr){
//    cv_bridge::CvImagePtr bridge_ptr;
//            try
//            {
//                 bridge_ptr = cv_bridge::toCvCopy(ptr,"8UC3");
//            }
//            catch (cv_bridge::Exception& e)
//            {
//                 ROS_ERROR("cv_bridge exception: %s", e.what());
//                return;
//            }

//            I2=bridge_ptr->image;
//            imshow("subscribed img",bridge_ptr->image);
//            waitKey(40);
//}

//void heart_task::on_trackbar(int, void * ){
//        Scalar thresh1(0,slider,50), thresh2(255,slider_max,255);
//        inRange(hsv,thresh1,thresh2,trackimg);
//        imshow("hsv image" , trackimg);
//}

heart_task::~heart_task(){

}




int main(int argc, char** argv){
    ros::init(argc, argv, "talker");
    // if(argc <2){
    //     ROS_INFO("heart server");
    //     ros::shutdown();
    // }
    heart_task ht;
    ros::spin();
    return 0;
}
#endif HEART_TASK_cpp
