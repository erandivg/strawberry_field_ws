#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>

class TurtleMover {
public:
    TurtleMover() {
        // Suscribirse al tópico /odom
        sub_ = nh_.subscribe("/odom", 10, &TurtleMover::odomCallback, this);
        // Publicar en el tópico /cmd_vel
        pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

        // Inicializar posición objetivo
        initial_x_ = 0;
        target_x_ = initial_x_ + 1.2;
        reached_target_ = false;
        last_pause_time_ = ros::Time::now();
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        double current_x = msg->pose.pose.position.x;
        double current_y = msg->pose.pose.position.y;
        ros::Time current_time = ros::Time::now();

        // Verificar si se ha alcanzado la posición objetivo
        if (!reached_target_ && current_x >= target_x_) {
            ROS_INFO("Reached target x: %f", target_x_);
            reached_target_ = true;
            last_pause_time_ = current_time;
        }

        // Publicar velocidad para mover la tortuga hacia la posición objetivo
        geometry_msgs::Twist cmd;
        if (reached_target_) {
            if ((current_time - last_pause_time_).toSec() >= 10.0) {
                // Actualizar la siguiente posición objetivo
                target_x_ += 1.2;
                reached_target_ = false;
            } else {
                // Detener la tortuga
                cmd.linear.x = 0.0;
            }
        } else {
            cmd.linear.x = 0.1; // Velocidad constante

            // Corrección en yaw para mantener y = 0
            if (current_y > 0.09) {
                cmd.angular.z = -0.02;
            } else if (current_y < 0.01) {
                cmd.angular.z = 0.02;
            } else {
                cmd.angular.z = 0.0;
            }
        }
        pub_.publish(cmd);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;
    double initial_x_;
    double target_x_;
    bool reached_target_;
    ros::Time last_pause_time_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "move_turtle");

    TurtleMover turtle_mover;

    ros::spin();

    return 0;
}
