#ifndef bin_TASK_cpp
#define bin_TASK_cpp

#include <binTask/binHeader.h>
#include <binTask/binGoal.h>
#include <binTask/binResult.h>
//#include <binTask/binAction.h>


bin_task::bin_task()
    :serv(_n,"bin_server",boost::bind(&bin_task::serverCallback,this,_1),false), _it(_n)
{

    _imgsub = _it.subscribe(topics::CAMERA_FRONT_RAW_IMAGE,10,&bin_task::imageCallback,this);

    if(!image.data){
        cout << "image canot be opened";
        shutdown();
    }

    median.x= 0;
    median.y = 0;
    imgCenter.x = image.rows/2;
    imgCenter.y = image.cols/2;

    ifstream _ifo("/home/manvi/ros_ws/binTask/threshold.th",ios::in);
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
    _pub = _it.advertise("bin_server",1);
    serv.start();
    ROS_INFO("waiting for clients");;
}


bool bin_task::detectbin(){

    cvtColor(img_1, _imageHSV, CV_BGR2HSV);
    imshow("img_1", _imageHSV);
    inRange(_imageHSV,Scalar(0,75,75),Scalar(40,255,255), _red);
    inRange(_imageHSV,Scalar(35,0,0),Scalar(75,255,255), _green);
    add(_green, _red, _merge);

    adaptiveThreshold(_merge,_median,200,ADAPTIVE_THRESH_MEAN_C,THRESH_BINARY,13,0);
    erode(_median,_median,_kernel);
    dilate(_median,_median,_kernel);

    IplImage fimage = _median;
    _blobs1 = CBlobResult(&fimage,NULL,0);
    _blobs2 = CBlobResult(&fimage,NULL,0);
    _blobs1.Filter(_blobs1, B_EXCLUDE, CBlobGetArea(), B_GREATER, 100);
    _blobs2.Filter(_blobs2,B_INCLUDE,CBlobGetArea(), B_LESS, 40);

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
    imshow("fmat",fmat);


    findContours(_merge, contours,CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
    vector<vector<Point> > polygon(contours.size());
    vector<RotatedRect> _rect(contours.size());

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

                _fcontour = contours[i];
                _frect = minAreaRect(Mat(_fcontour));
                _frect.points(_fpoints);
                cout << "hello ";
                rectangle(_merge,_fpoints[0],_fpoints[3],Scalar(255,0,0),3);
                bin_detect_status = true;
            }
        }
    }


    for(int i=0; i<4; i++){
        median.x += _fpoints[i].x;
        median.y += _fpoints[i].y;
    }

    median.x = median.x/4;
    median.y = median.y/4;

    circle(img_1,median,5,Scalar(255,255,255),2,8,0);

    if(median.x ==0 || median.y == 0){
        bin_detect_status = false;
    }
    else
        bin_detect_status = true;


    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(),"bgr8",image).toImageMsg();
    _pub.publish(msg);

    if(waitKey(33)==27)
        return true;

    return bin_detect_status;
}

void bin_task::alignCenter(){

    _error.x = median.x-imgCenter.x;
    _error.y = median.y-imgCenter.y;

    //    if(_error.x<0 && _error.y>0)
    //        ROS_INFO("GO RIGHT AND THEN DOWNWARDS");
    //    else if(_error.x >0 && _error.y >0)
    //        ROS_INFO("GO LEFT AND THEN DOWNWARDS");
    //    else if (_error.x <0 && _error.y <0)
    //        ROS_INFO("GO right and the upwards");
    //    else if(_error.x >0 && _error.y<0)
    //        ROS_INFO("go left and then upwards");
}



void bin_task::serverCallback(const ip_msgs::binGoalConstPtr &goal)
{
    Rate looprate(10);
    switch (goal->bin) {
    case DETECT_bin:
    {
        ROS_INFO("detect bin..");
        while(ok()){
            if(serv.isPreemptRequested())
            {
                res.centerx = _error.x;
                res.centery = _error.y;
                serv.setPreempted(res);
                break;
            }
            _status = detectbin();

            if(_status){
                ROS_INFO("bin has been detected..\n");
                res.centerx = _error.x;
                res.centery = _error.y;
                res.fresult = bin_DETECTED;
                serv.setSucceeded(res);
            }else{
                ROS_INFO("bin has not been detected..\n continue search");
                res.fresult = NOT_DETECTED;
                serv.setAborted(res);
            }


            serv.publishFeedback(_feed);
            looprate.sleep();
            _error.x =0;
            _error.y =0;
        }
        break;
    }


    case ALIGN_bin:
    {
        ROS_INFO("start aligning..!!!");
        while(ok()){
            if(serv.isPreemptRequested()){
                res.centerx = _error.x;
                res.centery = _error.y;
                serv.setPreempted(res);
                break;
            }
            _status = detectbin();
            if(_status){
                alignCenter();
                if(fabs(_error.x) < 10 && fabs(_error.y) < 10){
                    _feed.errorx = _error.x;
                    _feed.errory =bin_ALIGNED;
                    serv.setSucceeded(res);
                }
                else{
                    ROS_INFO("detect gate first.. and then align..");
                    _feed.errorx = _error.x;
                    _feed.errory = _error.y;
                    res.fresult = NOT_DETECTED;
                    serv.setAborted(res);
                }
            }
            serv.publishFeedback(_feed);
            looprate.sleep();
        }
        break;
    }
    }
}



void bin_task::imageCallback(const sensor_msgs::ImageConstPtr &ptr){
    cv_bridge::CvImagePtr bridge_ptr;
    try
    {
        bridge_ptr = cv_bridge::toCvCopy(ptr,"8UC3");
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    I2=bridge_ptr->image;
    image = I2;
    imshow("subscribed img",image);
    waitKey(40);
}


bin_task::~bin_task(){

}

int main(int argc, char** argv){
    ros::init(argc, argv, "talker");
    bin_task bt;
    ros::spin();
    return 0;
}
#endif bin_TASK_cpp
