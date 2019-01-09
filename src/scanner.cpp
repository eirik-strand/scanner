#include "../include/scanner.h"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include <iostream>


using namespace std;
using namespace cv;

#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <cstdio>
#include <tf2/LinearMath/Quaternion.h>



double laser_angle;

ros::NodeHandle * nh;
Mat image;
int rl = 180;
int gl = 0;
int bl = 20;

int rh = 256;
int gh = 95;
int bh = 130;

double alpha;
double beta;


std::vector<double > depths;
std::vector<double > pixels;
Mat filtered;

Mat src1;
Mat src2;
string window_name = "katt";

//Wht does line classifiers need
int min_num_pixels;
int max_num_pixels;

cv::Scalar lower_color_range(0,255,255);
cv::Scalar upper_color_range(0,255,255);


ros::Publisher cloudpup;
// a distance to pixel number sanity check. This must be calibrated
// Surrounding color compensation, for pixels to match
// light intensity compensation
// tracing to find planes and surfaces
//


Mat src, src_gray;
Mat dst, detected_edges, dst2;

double start;


int row_filter(std::vector<double > pixels){
    int sum = 0;
    if(pixels.size() < 5 || pixels.size() > 40){
        return -1;
    }
    ROS_INFO_STREAM("Starting filert");
    for(int i = 0; i< pixels.size(); i++){
        sum+=pixels[i];
    }


    return (int)sum/pixels.size();
}



pcl::PointCloud<pcl::PointXYZRGB>::Ptr img_to_cloud(
        const cv::Mat& image,
        const cv::Mat &coords)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    //std::cout << "size: " << depths.size() << std::endl;
    //std::cout << "sizer: " << image.rows << std::endl;
    //std::cout << "sizec: " << image.cols << std::endl;

    for (int y=0;y<image.rows;y++){
        for (int x=0;x<image.cols;x++){
            pcl::PointXYZRGB point;


            cv::Vec3b color = image.at<cv::Vec3b>(cv::Point(x,y));
            int r = (color[2]);
            int g = (color[1]);
            int b = (color[0]);


            point.x = coords.at<double>(0,y*image.cols+x);
            point.y = coords.at<double>(1,y*image.cols+x);
            point.z = 1;


            int32_t rgb = (r << 16) | (g << 8) | b;
            point.rgb = *reinterpret_cast<float*>(&rgb);


            cloud->points.push_back(point);






        }
    }
    return cloud;
}

std::vector<cv::Vec4i> extractAllLines(const cv::Mat& image, int threshold, double minLength = 100)
{
    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(image, lines, 1, CV_PI / 180, threshold, minLength);
    return lines;
}


// Define a pixel
typedef Point3_<uint8_t> Pixel;


struct Operator
{
    void operator ()(Pixel &pixel, const int * position) const
    {

    }
};


void video_test(){
    VideoCapture cap(0);

  // Check if camera opened successfully
  if(!cap.isOpened()){
    cout << "Error opening video stream or file" << endl;
    return ;
  }

  double expo = 0.25;
  std::cout << "MJPG: " << cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
  cap.set(CV_CAP_PROP_FRAME_WIDTH,640);
  cap.set(CV_CAP_PROP_FRAME_HEIGHT,480);
  cap.set(CV_CAP_PROP_FPS, 10);
  cap.set(CV_CAP_PROP_AUTO_EXPOSURE, false);
  cap.set(CV_CAP_PROP_EXPOSURE, 500);
  cap.set(CV_CAP_PROP_GAIN, 0);
  double last_time = ros::Time::now().toSec();
  double now = last_time;
  Mat frame;
  while(ros::ok()){

      cap >> frame;
      if (frame.empty())
          break;


      now = ros::Time::now().toSec();
      double time_duratinon = 1.0/(now-last_time);
      //double num_pixels = frame.rows*frame.cols;
      //ROS_INFO("FPS: %f", time_duratinon);

      last_time = now;
      int ro= frame.rows/2;
      int co = frame.cols/2;
      cv::Vec3b color = frame.at<cv::Vec3b>(cv::Point(co,ro));
      int katt = color[0]+color[1]+color[2];
      ROS_INFO_STREAM(katt);

    // Capture frame-by-frame


    // If the frame is empty, break immediately

    // Display the resulting frame

    // Press  ESC on keyboard to exit

    int *i = 0;

    //frame.forEach<Pixel>(Operator());
    imshow(window_name, frame);
    if (waitKey(5) >= 0)
        break;


      cv::Mat cloud_image = frame;
      //cv::cvtColor(frame, cloud_image, cv::COLOR_GRAY2BGR);

      cv::Mat coords(3, cloud_image.cols * cloud_image.rows, CV_64FC1);
      for (int col = 0; col < coords.cols; ++col) {
          coords.at<double>(0, col) = ((col % cloud_image.cols)) / 1000.0;
          coords.at<double>(1, col) = ((col / cloud_image.cols)) / 1000.0;
          coords.at<double>(2, col) = 2;
      }

      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = img_to_cloud(cloud_image, coords);


      sensor_msgs::PointCloud2 trump;
      pcl::toROSMsg(*cloud, trump);
      trump.header.frame_id = "laser";


      cloudpup.publish(trump);

    }





  // When everything done, release the video capture object
  cap.release();

  // Closes all the frames
  destroyAllWindows();

}
void init_transforms(){

    static tf2_ros::StaticTransformBroadcaster static_broadcaster;
    geometry_msgs::TransformStamped world_camera;

    world_camera.header.stamp = ros::Time(0);
    world_camera.header.frame_id = "world";
    world_camera.child_frame_id = "camera";
    world_camera.transform.translation.x = 0;
    world_camera.transform.translation.y = 0;
    world_camera.transform.translation.z = 1;
    tf2::Quaternion quat;
    quat.setRPY(0, 0, 0);
    world_camera.transform.rotation.x = quat.x();
    world_camera.transform.rotation.y = quat.y();
    world_camera.transform.rotation.z = quat.z();
    world_camera.transform.rotation.w = quat.w();



    geometry_msgs::TransformStamped camera_laser;

    camera_laser.header.stamp = ros::Time(0);
    camera_laser.header.frame_id = "camera";
    camera_laser.child_frame_id = "laser";
    camera_laser.transform.translation.x = 0.6;
    camera_laser.transform.translation.y = 0;
    camera_laser.transform.translation.z = 0;
    quat.setRPY(-1.4, 0, 0.3);
    camera_laser.transform.rotation.x = quat.x();
    camera_laser.transform.rotation.y = quat.y();
    camera_laser.transform.rotation.z = quat.z();
    camera_laser.transform.rotation.w = quat.w();
    static_broadcaster.sendTransform(camera_laser);


    static_broadcaster.sendTransform(world_camera);

}

int main(int argc, char **argv) {

    ros::init(argc, argv, "nscan");
    nh = new ros::NodeHandle("nscan");
    init_transforms();
    namedWindow( window_name, WINDOW_NORMAL );
    cv::resizeWindow(window_name, 1000, 1000);

    start = ros::Time::now().toSec();
    cloudpup = nh->advertise<sensor_msgs::PointCloud2>("scan", 0);

    video_test();
    return 0;


    src = imread(argv[1], CV_LOAD_IMAGE_COLOR);   // Read the file

    image = src;

    if (!image.data)                              // Check for invalid input
    {
        cout << "Could not open or find the image" << std::endl;
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
    int pixel;

    for (int x = 0; x < lower_red_hue_range.rows; x++) {
        pixels.clear();
        for (int y = 0; y < lower_red_hue_range.cols; y++) {
            pixel = (int) lower_red_hue_range.at<uchar>(x, y);
            if (pixel) {

                pixels.push_back(y);

            }

        }
        int avrg_pixel = row_filter(pixels);
        double angle = 75.0*M_PI/180.0;
        double distance = 0.32;
        double zero_depth = tan(angle)*distance;
        double half_pixel = lower_red_hue_range.cols/2;
        double depth = (avrg_pixel/half_pixel)*0.32*tan(angle);
        std::cout << "Found at col: " <<depth  << std::endl;
        depths.push_back(depth);
    }


        Mat img;
        Mat cloud_depth = lower_red_hue_range;
        cv::cvtColor(lower_red_hue_range, img, cv::COLOR_GRAY2BGR);
        int center_x = img.cols / 2;
        int center_y = img.rows / 2;
        int angle = 0;
        int bar_length = img.rows;
        line(img, Point(center_x + angle, center_y + bar_length),
             Point(center_x - angle, center_y - bar_length),
             Scalar(50, 0, 100), 5);


        Mat lines_image = img;
        img.setTo(cv::Scalar(255, 50, 0), img);


        src += img;


        for (int r = 0; r < lines_image.rows; r++) {
            for (int c = 0; c < lines_image.cols; c++) {
                lines_image.at<cv::Vec3b>(r, c)[1];

            }

        }


        //imshow(window_name, src);



        //cv::waitKey(0);



        cv::Mat cloud_image;
        cv::cvtColor(lower_red_hue_range, cloud_image, cv::COLOR_GRAY2BGR);

        cv::Mat coords(3, cloud_image.cols * cloud_image.rows, CV_64FC1);
        for (int col = 0; col < coords.cols; ++col) {
            coords.at<double>(0, col) = ((col % cloud_image.cols)) / 5000.0;
            coords.at<double>(1, col) = ((col / cloud_image.cols)) / 5000.0;
            coords.at<double>(2, col) = 2;
        }

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = img_to_cloud(cloud_image, coords);


        sensor_msgs::PointCloud2 trump;
        pcl::toROSMsg(*cloud, trump);
        trump.header.frame_id = "laser";


        ros::Publisher cloudpup = nh->advertise<sensor_msgs::PointCloud2>("scan", 2);
        cloudpup.publish(trump);

        while (ros::ok()) {
            cloudpup.publish(trump);

            ros::Duration(5).sleep();
            ros::spinOnce();
        }


        delete nh;


        return 0;
    }
