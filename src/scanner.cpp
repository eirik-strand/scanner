#include "../include/scanner.h"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include <iostream>

using namespace std;
using namespace cv;

ros::NodeHandle * nh;


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
            point.x = coords.at<double>(0,y*image.cols+x);
            point.y = coords.at<double>(1,y*image.cols+x);
            point.z = coords.at<double>(2,y*image.cols+x);

            cv::Vec3b color = image.at<cv::Vec3b>(cv::Point(x,y));
            uint8_t r = (color[2]);
            uint8_t g = (color[1]);
            uint8_t b = (color[0]);

            int32_t rgb = (r << 16) | (g << 8) | b;
            point.rgb = *reinterpret_cast<float*>(&rgb);

            cloud->points.push_back(point);
        }
    }
    return cloud;
}


int main(int argc, char **argv) {

    ros::init(argc, argv, "nscan");
    nh = new ros::NodeHandle("nscan");

    Mat image;
    image = imread(argv[1], CV_LOAD_IMAGE_COLOR);   // Read the file

    if(! image.data )                              // Check for invalid input
    {
        cout <<  "Could not open or find the image" << std::endl ;
        return -1;
    }

    //namedWindow( "Display window", WINDOW_AUTOSIZE );// Create a window for display.
    //imshow( "Display window", image );                   // Show our image inside it.
    //cv::waitKey();

    cv::Mat coords(3, image.cols * image.rows, CV_64FC1);
    for (int col = 0; col < coords.cols; ++col)
    {
        coords.at<double>(0, col) = col % image.cols;
        coords.at<double>(1, col) = col / image.cols;
        coords.at<double>(2, col) = 10;
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = img_to_cloud(image, coords);


    sensor_msgs::PointCloud2 trump;
    pcl::toROSMsg(*cloud, trump);
    trump.header.frame_id = "trump";


    ros::Publisher cloudpup = nh->advertise<sensor_msgs::PointCloud2>("trump", 2);


    while (ros::ok()){
        cloudpup.publish(trump);

        ros::Duration(1).sleep();
        ros::spinOnce();
    }


    delete nh;
    ros::spin();




}
