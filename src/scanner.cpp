#include "../include/scanner.h"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include <iostream>

using namespace std;
using namespace cv;

ros::NodeHandle * nh;
Mat image;
int alpha_slider = 0;
int alpha_slider2 = 255;

int alpha_slider3 = 100;
int alpha_slider4 = 170;
int alpha_slider5 = 255;
int alpha_slider6 = 255;

double alpha;
double beta;

Mat src1;
Mat src2;

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

int edgeThresh = 1;
int lowThreshold;
int const max_lowThreshold = 100;
int ratio = 3;
int kernel_size = 3;
char* window_name = "Edge Map";


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



static void on_trackbar( int, void* )
{
    alpha = (double) alpha_slider/255 ;
    beta = ( 1.0 - alpha );
    cv::inRange(image, cv::Scalar(alpha_slider, alpha_slider3, alpha_slider4), cv::Scalar(alpha_slider2, alpha_slider5, alpha_slider6), dst);

    imshow( "Display window2", dst );
}


static void on_trackbar2( int, void* )
{
    alpha = (double) alpha_slider/255 ;
    beta = ( 1.0 - alpha );
    cv::inRange(image, cv::Scalar(alpha_slider, alpha_slider3, alpha_slider4), cv::Scalar(alpha_slider2, alpha_slider5, alpha_slider6), dst);

    imshow( "Display window2", dst );
}


static void on_trackbar3( int, void* )
{
    alpha = (double) alpha_slider/255 ;
    beta = ( 1.0 - alpha );
    cv::inRange(image, cv::Scalar(alpha_slider, alpha_slider3, alpha_slider4), cv::Scalar(alpha_slider2, alpha_slider5, alpha_slider6), dst);

    imshow( "Display window2", dst );
}


static void on_trackbar4( int, void* )
{
    alpha = (double) alpha_slider/255 ;
    beta = ( 1.0 - alpha );
    cv::inRange(image, cv::Scalar(alpha_slider, alpha_slider3, alpha_slider4), cv::Scalar(alpha_slider2, alpha_slider5, alpha_slider6), dst);

    imshow( "Display window2", dst );
}


static void on_trackbar5( int, void* )
{
    alpha = (double) alpha_slider/255 ;
    beta = ( 1.0 - alpha );
    cv::inRange(image, cv::Scalar(alpha_slider, alpha_slider3, alpha_slider4), cv::Scalar(alpha_slider2, alpha_slider5, alpha_slider6), dst);

    imshow( "Display window2", dst );
}


static void on_trackbar6( int, void* )
{
    alpha = (double) alpha_slider/255 ;
    beta = ( 1.0 - alpha );
    cv::inRange(image, cv::Scalar(alpha_slider, alpha_slider3, alpha_slider4), cv::Scalar(alpha_slider2, alpha_slider5, alpha_slider6), dst);

    imshow( "Display window2", dst );
}

void CannyThreshold(int, void*)
{
    /// Reduce noise with a kernel 3x3
    //cvtColor( dst, src_gray, CV_BGR2GRAY );

    blur( src_gray, detected_edges, Size(3,3) );

    /// Canny detector
    Canny( detected_edges, detected_edges, lowThreshold, lowThreshold*ratio, kernel_size );

    /// Using Canny's output as a mask, we display our result
    dst2.create( src.size(), src.type() );

    dst2 = Scalar::all(0);

    src.copyTo( dst2, detected_edges);
    imshow( window_name, dst2 );
}


int main(int argc, char **argv) {

    ros::init(argc, argv, "nscan");
    nh = new ros::NodeHandle("nscan");

    src = imread(argv[1], CV_LOAD_IMAGE_COLOR);   // Read the file
    cv::resize(src, src, cv::Size(src.rows,src.rows), 0, 0, CV_INTER_LINEAR);

    image = src;
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

    // Threshold the HSV image, keep only the red pixels
    cv::Mat lower_red_hue_range;
    cv::Mat upper_red_hue_range;
    cv::inRange(hsv_image, cv::Scalar(alpha_slider, alpha_slider3, alpha_slider4), cv::Scalar(alpha_slider2, alpha_slider5, alpha_slider6), lower_red_hue_range);

    //image = lower_red_hue_range;
    namedWindow( window_name, WINDOW_NORMAL );
    cv::resizeWindow(window_name, 1000, 1000);




    string text = "Funny text inside the box";
    int fontFace = FONT_HERSHEY_SCRIPT_SIMPLEX;
    double fontScale = 2;
    int thickness = 8;

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


















    /*
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


    */

    return 0;
}
