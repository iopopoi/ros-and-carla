#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <vector>
#include <iostream>

using namespace std;
using namespace cv;

struct Subscribe_And_Publish{
    ros::NodeHandle n;
    image_transport::ImageTransport it;
    image_transport::Subscriber getImage;
    image_transport::Publisher pubLaneImage;
    bool isRun;

    Subscribe_And_Publish(bool isrun): isRun(isrun), it(n){
        getImage = it.subscribe("/carla/ego_vehicle/camera/rgb/front/image_color", 1, &Subscribe_And_Publish::callback, this);
        pubLaneImage = it.advertise("/laneImage",1);
    }

    void callback(const sensor_msgs::ImageConstPtr& img){
        // sensor_msgs/Image -> opencv Mat type images
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8); // img를 인코딩하여 opencv Image타입으로 변환
        cv_bridge::CvImagePtr cv_ptr2 = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
        
        // image size : 800 * 600 , channel: 1
        Mat origin;
        cv_ptr->image.copyTo(origin);

        Mat img_canny;
        Canny(origin, img_canny, 100, 200);
        
        // region of interest
        Mat mask;
        threshold(img_canny, mask, 150,255, THRESH_MASK);

        vector<Point> points;
        points.push_back(Point(250,450));
        points.push_back(Point(550,450));
        points.push_back(Point(700,600));
        points.push_back(Point(100,600));

        fillConvexPoly(mask, points, Scalar(255), 0);

        bitwise_and(img_canny, mask, img_canny);

        // [ hough edge ] -------------------------------------
        vector<Vec4i> linesP;
        HoughLinesP(img_canny, linesP, 1,(CV_PI/180), 20,10,20);

        // [ candidate lines ] --------------------------------
        Point ini, fini;
        vector<Vec4i> selected_lines;

        vector<Vec4i> Llines, Rlines; 
        vector<double> slopes;
        int Rdetect=0,Ldetect=0;
        double slope_thresh = 0.3;

        // check slope
        for(int i=0;i<linesP.size();i++){
            Vec4i line = linesP[i];
            ini = Point(line[0], line(1));
            fini = Point(line[2], line[3]);

            double slope = (static_cast<double>(fini.y) - static_cast<double>(ini.y)) / (static_cast<double>(fini.x) - static_cast<double>(ini.x)+0.00001);

            if(abs(slope) > slope_thresh){
                slopes.push_back(slope);
                selected_lines.push_back(line);
            } 
        }

        // seperate left lines with right lines
        int center = static_cast<double>((img_canny.cols)/2);

        for(int i=0;i<selected_lines.size();i++){
            ini = Point(selected_lines[i][0],selected_lines[i][1]);
            fini = Point(selected_lines[i][2], selected_lines[i][3]);

            if(slopes[i] > 0 && fini.x > center+80 && ini.x > center){
                Llines.push_back(selected_lines[i]);
                Ldetect = 1;
            }
            else if(slopes[i] < 0 && fini.x < center-80 && ini.x < center){
                Rlines.push_back(selected_lines[i]);
                Rdetect = 1;
            }
        }
        // ------------------------------------- [ end candidate lines ] 


        // [ regression ] -------- select left, right line --------------------------

        Point ini2, fini2;
        Vec4d left_line, right_line;
        vector<Point> left_points, right_points;

        double m[2]; // 0: left, 1: right
        Point b[2];

        if(Rdetect){
            for(auto i:Rlines){
                right_points.push_back(Point(i[0],i[1]));
                right_points.push_back(Point(i[2],i[3]));
            }
            if(right_points.size()>0){
                fitLine(right_points, right_line, DIST_L2, 0, 0.01, 0.01);
                m[1] = right_line[1] / right_line[0]; // slope of right line
                b[1] = Point(right_line[2], right_line[3]);
            }
        }

        if(Ldetect){
            for(auto i:Llines){
                left_points.push_back(Point(i[0],i[1]));
                left_points.push_back(Point(i[2],i[3]));
            }
            if(left_points.size()>0){
                fitLine(left_points, left_line, DIST_L2, 0, 0.01, 0.01);
                m[0] = left_line[1] / left_line[0]; // slope of left line
                b[0] = Point(left_line[2], left_line[3]);
            }
        }

        int ini_y = img_canny.rows, fin_y = 450;
        Mat img_lane(img_canny.size(), CV_8UC1, Scalar(0)), RM(origin.size(), CV_8U, Scalar(0));
        
        double Lx[2], Rx[2];
        // compute left line
        if(left_points.size()>0){
            Lx[0] = ((ini_y - b[0].y) / m[0] )+b[0].x;
            Lx[1] = ((fin_y - b[0].y) / m[0] )+b[0].x;
        }
        // compute right line
        if(right_points.size()>0){
            Rx[0] = ((ini_y - b[1].y) / m[1] )+b[1].x;
            Rx[1] = ((fin_y - b[1].y) / m[1] )+b[1].x;

            line(cv_ptr->image, Point(cvRound(Rx[0]), ini_y), Point(cvRound(Rx[1]), fin_y), Scalar(0,0,255), 4,8);
            line(img_lane, Point(cvRound(Rx[0]), ini_y), Point(cvRound(Rx[1]), fin_y), Scalar(255),2,8);
        }

        // fill between lines
        if(left_points.size()>0&& right_points.size()>0){
            vector<Point> box;
            box.push_back(Point(cvRound(Lx[0]), ini_y));
            box.push_back(Point(cvRound(Lx[1]), fin_y));
            box.push_back(Point(cvRound(Rx[1]), fin_y));
            box.push_back(Point(cvRound(Rx[0]), ini_y));
            fillConvexPoly(cv_ptr->image, box, Scalar(204,255,255), LINE_AA,0);
        }

        // draw left line
        if(left_points.size()>0){
            line(cv_ptr->image, Point(cvRound(Lx[0]), ini_y), Point(cvRound(Lx[1]), fin_y), Scalar(51,255,0), 5,8);
            line(img_lane, Point(cvRound(Lx[0]), ini_y), Point(cvRound(Lx[1]), fin_y), Scalar(255),2,8);
        }

        // draw right line
        if(right_points.size()>0){
            line(cv_ptr->image, Point(cvRound(Rx[0]), ini_y), Point(cvRound(Rx[1]), fin_y), Scalar(51,255,0), 5,8);
            line(img_lane, Point(cvRound(Rx[0]), ini_y), Point(cvRound(Rx[1]), fin_y), Scalar(255),2,8);
        }
        cv_ptr2->image = img_lane;
				// image
        // cv_ptr->image
        pubLaneImage.publish(cv_ptr->toImageMsg()); // opencv Mat type -> sensor_msgs/Image type
    }
};

int main(int argc, char **argv){
    cout << "Start laneDetect node!\n";
    ros::init(argc, argv, "laneDetect");

    Subscribe_And_Publish laneDetect(1);
    ros::spin();
}