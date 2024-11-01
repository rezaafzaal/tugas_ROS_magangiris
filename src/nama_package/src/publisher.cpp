#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <iostream>

using namespace cv;

int main(int argc, char** argv) {
    ros::init(argc, argv, "position_publisher");
    ros::NodeHandle nh;

    ros::Publisher robot_pos_pub = nh.advertise<geometry_msgs::Pose>("robot_position", 10);
    ros::Publisher ball_pos_pub = nh.advertise<geometry_msgs::Pose>("ball_position", 10);

    VideoCapture cap("/home/rezaafzaal/Downloads/robotkeren.mp4");
    if (!cap.isOpened()) return -1;

    Mat frame, hsvFrame, mask;
    double x_robot = 0.0, y_robot = 0.0;
    double prev_ballX_cm = 0.0, prev_ballY_cm = 0.0;
    const float px_to_cm = 0.1;
    const float cm_to_lapangan = 10.0;
    bool is_first_frame = true;
    const double min_area_ball = 30.0;
    const double max_area_ball = 5000.0;

    // Static ball position
    // const double staticBallX_cm = 5.5; // Static X position in cm
    // const double staticBallY_cm = 5.5; // Static Y position in cm

    // Set center field point (where the ball is assumed to be static)
    Point2f centerField(cap.get(CAP_PROP_FRAME_WIDTH) / 2, cap.get(CAP_PROP_FRAME_HEIGHT) / 2);
    int exclusionRadius = 100;

    ros::Rate loop_rate(30);
    while (ros::ok()) {
        cap >> frame;
        if (frame.empty()) break;

        // Convert to HSV and apply mask for detecting the robot
        cvtColor(frame, hsvFrame, COLOR_BGR2HSV);
        Scalar lower_orange(5, 150, 150);
        Scalar upper_orange(15, 255, 255);
        inRange(hsvFrame, lower_orange, upper_orange, mask);

        // Exclude the central region where the ball is located
        circle(mask, centerField, exclusionRadius, Scalar(0), -1);

        // Find contours to detect the robot
        std::vector<std::vector<Point>> contours;
        findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        if (!contours.empty()) {
            double maxArea = 0;
            int largestContourIndex = -1;

            for (int i = 0; i < contours.size(); i++) {
                double area = contourArea(contours[i]);
                if (area > maxArea && area >= min_area_ball && area <= max_area_ball) {
                    maxArea = area;
                    largestContourIndex = i;
                }
            }

            if (largestContourIndex != -1) {
                RotatedRect rect = minAreaRect(contours[largestContourIndex]);
                Point2f center = rect.center;

                double object_x_cm = center.x * px_to_cm;
                double object_y_cm = center.y * px_to_cm;

                if (!is_first_frame) {
                    x_robot -= object_x_cm - prev_ballX_cm;
                    y_robot += object_y_cm - prev_ballY_cm;
                }

                prev_ballX_cm = object_x_cm;
                prev_ballY_cm = object_y_cm;

                if (x_robot < 0){
                    x_robot -= x_robot - 1;
                }
                
                if(y_robot < 0){
                    y_robot -= y_robot - 1;
                }

                
                is_first_frame = false;

                // Publish robot position
                geometry_msgs::Pose robot_position;
                robot_position.position.x = x_robot / cm_to_lapangan + 2.5; // Convert cm to TurtleSim units
                robot_position.position.y = y_robot / cm_to_lapangan + 2.5;

                robot_pos_pub.publish(robot_position);

                // Publish static ball position (staticBallX_cm, staticBallY_cm)
                geometry_msgs::Pose ball_position;
                ball_position.position.x = 5.5; 
                ball_position.position.y = 5.5;
                ball_pos_pub.publish(ball_position);
            }
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}