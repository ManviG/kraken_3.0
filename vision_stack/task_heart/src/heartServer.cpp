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
    RotatedRect _frect;
    double max=0;
    bool heart_detect_status;
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
                cout << "hello ";
                rectangle(_merge,points[0],points[3],Scalar(255,0,0),3);
                heart_detect_status = true;
            }
        }
    }

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


    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(),"bgr8",image).toImageMsg();
    //   sensor_msgs::ImagePtr msg=object.toImageMsg();
    _pub.publish(msg);
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


void heartTask::imageCallback(const sensor_msgs::ImageConstPtr &ptr){
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
            imshow("subscribed img",bridge_ptr->image);
            waitKey(40);
}


heart_task::~heart_task(){

}




int main(int argc, char** argv){
    ros::init(argc, argv, "talker");
    heart_task ht;
    ros::spin();
    return 0;
}
#endif HEART_TASK_cpp
