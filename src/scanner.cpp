#include "../include/scanner.h"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include <iostream>

using namespace std;
using namespace cv;

ros::NodeHandle * nh;
Mat image;
int rl = 110;
int gl = 30;
int bl = 30;

int rh = 150;
int gh = 95;
int bh = 95;

double alpha;
double beta;

Mat src1;
Mat src2;
string window_name = "katt";

//Wht does line classifiers need
int min_num_pixels;
int max_num_pixels;

cv::Scalar lower_color_range(0,255,255);
cv::Scalar upper_color_range(0,255,255);

// a distance to pixel number sanity check. This must be calibrated
// Surrounding color compensation, for pixels to match
// light intensity compensation
// tracing to find planes and surfaces
//


Mat src, src_gray;
Mat dst, detected_edges, dst2;




pcl::PointCloud<pcl::PointXYZRGB>::Ptr img_to_cloud(
        const cv::Mat& image,
        const cv::Mat &coords)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());

    for (int y=0;y<image.rows;y++)
    {
        for (int x=0;x<image.cols;x++)
        {
            pcl::PointXYZRGB point;


            cv::Vec3b color = image.at<cv::Vec3b>(cv::Point(x,y));
            int r = (color[2]);
            int g = (color[1]);
            int b = (color[0]);

            if(g == 0) {
                point.x = coords.at<double>(0,y*image.cols+x);
                point.y = coords.at<double>(1,y*image.cols+x);
                point.z = coords.at<double>(2,y*image.cols+x) + 1;
            }else {
                point.x = coords.at<double>(0,y*image.cols+x);
                point.y = coords.at<double>(1,y*image.cols+x);
                point.z = coords.at<double>(2,y*image.cols+x);
            }

            int32_t rgb = (r << 16) | (g << 8) | b;
            point.rgb = *reinterpret_cast<float*>(&rgb);
            if((r+g+b)!= 0){
                cloud->points.push_back(point);
            }

        }
    }
    return cloud;
}




int main(int argc, char **argv) {

    ros::init(argc, argv, "nscan");
    nh = new ros::NodeHandle("nscan");

    namedWindow( window_name, WINDOW_NORMAL );
    cv::resizeWindow(window_name, 1000, 1000);

    src = imread(argv[1], CV_LOAD_IMAGE_COLOR);   // Read the file
    cv::resize(src, src, cv::Size(src.rows,src.rows), 0, 0, CV_INTER_LINEAR);

    image = src;
    image.convertTo(image, -1, 0.5, 0);
    if(! image.data )                              // Check for invalid input
    {
        cout <<  "Could not open or find the image" << std::endl ;
        return -1;
    }

    /*

    vector<Mat> planes;
    split(image,planes);
    Mat red = planes[2];
    Mat temp = planes[2] - planes[0];
    threshold(temp,temp,0,255,THRESH_OTSU);
    //imwrite("reslut.jpg",temp);

    image = temp;

    */





    cv::Mat hsv_image;
    cv::cvtColor(image, hsv_image, cv::COLOR_BGR2HSV);

    //imshow(window_name, image);


    // Threshold the HSV image, keep only the red pixels
    cv::Mat lower_red_hue_range;
    cv::Mat upper_red_hue_range;
    cv::inRange(image, cv::Scalar(bl, gl, rl), cv::Scalar(bh, gh, rh), lower_red_hue_range);







    Mat img;
    cv::cvtColor(lower_red_hue_range, img, cv::COLOR_GRAY2BGR);
    int center_x = img.cols/2;
    int center_y = img.rows/2;
    int angle = 500;
    int bar_length = 600;
    line(img, Point(center_x+angle, center_y+bar_length),
         Point(center_x-angle, center_y-bar_length),
         Scalar(255, 0, 255), 5);

    line(img, Point(center_x+bar_length, center_y-angle),
         Point(center_x-bar_length, center_y+angle),
         Scalar(255, 0, 255), 5);

    Mat lines_image = img;
    img.setTo(cv::Scalar(0,255,255), img);

    src -= img;
    int filter_size = 30;
    int dividers = 10;

    vector<float> filter_mask;

    for (int i = 0;i < filter_size; i++){
        if(i < filter_size/2)
            filter_mask.push_back(i/10);
        else
            filter_mask.push_back(10-i/10);
    }


    for(int r = 0; r < lines_image.rows; r++){
        for(int c = 0; c < lines_image.cols; c++) {
            lines_image.at<cv::Vec3b>(r,c)[1];

        }

    }


    imshow(window_name, src);



    cv::waitKey(0);



    cv::Mat cloud_image = img;

    cv::Mat coords(3, cloud_image.cols * cloud_image.rows, CV_64FC1);
    for (int col = 0; col < coords.cols; ++col)
    {
        coords.at<double>(0, col) = ((col % cloud_image.cols))/1000.0;
        coords.at<double>(1, col) = ((col / cloud_image.cols))/1000.0;
        coords.at<double>(2, col) = 2;
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = img_to_cloud(cloud_image, coords);


    sensor_msgs::PointCloud2 trump;
    pcl::toROSMsg(*cloud, trump);
    trump.header.frame_id = "trump";


    ros::Publisher cloudpup = nh->advertise<sensor_msgs::PointCloud2>("trump", 2);


    while (ros::ok()){
        cloudpup.publish(trump);

        ros::Duration(5).sleep();
        ros::spinOnce();
    }


    delete nh;




    return 0;
}
