#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/TeleportAbsolute.h>
#include <turtlesim/SetPen.h>
#include <std_srvs/Empty.h>
#include <math.h>
#define _USE_MATH_DEFINES

ros::Publisher turtle1_pub;
ros::ServiceClient turtle1_set_pen_client;
ros::ServiceClient turtle1_teleport_client;
turtlesim::SetPen pen;
turtlesim::TeleportAbsolute tele;
geometry_msgs::Twist move;

void moveTurtle(double speed) {
    move.linear.x = speed;
    turtle1_pub.publish(move);
    ros::Duration(1).sleep();
    move.linear.x = 0;
    turtle1_pub.publish(move);
}

void rotate(double degree) {
    move.angular.z = degree * M_PI / 180;
    turtle1_pub.publish(move);
    ros::Duration(1.5).sleep();
    move.angular.z = 0;
    turtle1_pub.publish(move);
}

void setpen(int red, int green, int blue) {
    pen.request.r = red;
    pen.request.g = green;
    pen.request.b = blue;
    pen.request.width = 2;
    pen.request.off = 0;

    if (!turtle1_set_pen_client.call(pen)) {
        ROS_ERROR("Failed to call set_pen service");
    }
}

void teleport(float x, float y, float theta = 0) {
    tele.request.x = x;
    tele.request.y = y;
    tele.request.theta = theta;
    ros::Duration(1).sleep();

    if (!turtle1_teleport_client.call(tele)) {
        ROS_ERROR("Failed to call teleport_absolute service");
    }
}

void write_A(int r, int g, int b, double x, double y, double x1, double y1) {
    setpen(255, 255, 255);
    teleport(x, y);
    setpen(r, g, b);

    rotate(75);
    moveTurtle(2);

    rotate(-150);
    moveTurtle(2);

    setpen(255, 255, 255);
    teleport(x1, y1);
    setpen(r, g, b);

    moveTurtle(0.6);
}

void write_F(int r, int g, int b, double x, double y) {

    setpen(255, 255, 255);
    teleport(x, y);
    setpen(r, g, b);

    
    moveTurtle(1.0);

    rotate(180.0);
    moveTurtle(1.0);

    rotate(90.0);
    moveTurtle(1.0);

    rotate(90);
    moveTurtle(1.0);

    rotate(180);
    moveTurtle(1.0);

    rotate(90);
    moveTurtle(1.0);
}

void write_Z(int r, int g, int b, double x, double y) {
    setpen(255, 255, 255);
    teleport(x, y);
    setpen(r, g, b);

    moveTurtle(1.5);
    rotate(-126);
    moveTurtle(2.3);
    rotate(126);
    moveTurtle(1.5);
}

void write_L(int r, int g, int b, double x, double y) {
    setpen(255, 255, 255);
    teleport(x, y);
    setpen(r, g, b);

    rotate(-90);
    moveTurtle(2);

    rotate(90);
    moveTurtle(1);
}

void roket(int r, int g, int b, double x, double y, int x1) {
    setpen(255, 255, 255);
    teleport(x, y);
    setpen(r, g, b);
    rotate(x1);
    moveTurtle(2);
    rotate(-30);
    moveTurtle(1.5);
    rotate(-120);
    moveTurtle(1.5);
    rotate(-30);
    moveTurtle(2);
    rotate(-90);
    moveTurtle(1.4);


}

int main(int argc, char** argv) {
    ros::init(argc, argv, "control_turtle1_AFZAAL");
    ros::NodeHandle nh;

    nh.setParam("/turtlesim/background_r", 255);
    nh.setParam("/turtlesim/background_g", 255);
    nh.setParam("/turtlesim/background_b", 255);


    


    turtle1_pub = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
    turtle1_set_pen_client = nh.serviceClient<turtlesim::SetPen>("/turtle1/set_pen");
    turtle1_teleport_client = nh.serviceClient<turtlesim::TeleportAbsolute>("/turtle1/teleport_absolute");
    ros::ServiceClient clear_client = nh.serviceClient<std_srvs::Empty>("/clear");

    std_srvs::Empty srv;
    ros::Rate rate(0.1);
    ros::Duration(0.5).sleep();

    // while (ros::ok()) {

    
        setpen(255, 255, 255);
        write_A(0, 0, 0, 1.0, 5, 1.2, 5.5);
        write_F(0, 0, 0, 2.5, 7);
        write_Z(0, 0, 0, 4, 7);
        write_A(0, 0, 0, 6.0, 5, 6.2, 5.5);
        write_A(0, 0, 0, 7.5, 5, 7.7, 5.5);
        write_L(0, 0, 0, 9.0, 7);

        roket(0, 0, 255, 0.5, 3, 45);
        roket(255, 0, 0, 0.5, 10, -45);
        roket(0, 255, 0, 8.8, 10, -135);
        roket(255, 255, 0, 8.8, 1.5, 135);

        roket(255, 255, 255, 8.8, 1.5, 135);
        roket(255, 255, 255, 8.8, 10, -135);
        roket(255, 255, 255, 0.5, 10, -45);
        roket(255, 255, 255, 0.5, 3, 45);

        write_L(255, 255, 255, 9.0, 7);
        write_A(255, 255, 255, 7.5, 5, 7.7, 5.5);
        write_A(255, 255, 255, 6.0, 5, 6.2, 5.5);
        write_Z(255, 255, 255, 4, 7);
        write_F(255, 255, 255, 2.5, 7);
        write_A(255, 255, 255, 1.0, 5, 1.2, 5.5);
        

        ros::spinOnce();
        rate.sleep();
    // }

    return 0;
}