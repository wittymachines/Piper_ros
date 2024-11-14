#include <ros/ros.h>

#include <std_msgs/Header.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

// #include <pcl/common/common.h>
// #include <pcl/common/transforms.h>

#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

class Publisher
{
public:
    ros::NodeHandle nh;

    ros::Publisher pubJointState;

    KDL::Chain chain;
    KDL::ChainFkSolverPos_recursive *fkSolver;
    KDL::ChainIkSolverPos_LMA *ikSolver;

    Publisher()
    {
        pubJointState = nh.advertise<sensor_msgs::JointState>("/joint_states", 1);
        KDL::Tree my_tree;
        kdl_parser::treeFromFile("/home/agilex/piper_ws/src/piper_description/urdf/piper_description.urdf",my_tree);
        my_tree.getChain("base_link","link6",chain);
        fkSolver = new KDL::ChainFkSolverPos_recursive(chain);
        ikSolver = new KDL::ChainIkSolverPos_LMA(chain);
    }

    void process(){
        cv::Mat image = cv::Mat::ones(2, 200, CV_8UC3);
        cv::namedWindow("xyzrpy");
        cv::imshow("xyzrpy", image);
        int x = 70;
        int y = 50;
        int z = 70;
        int roll = 50;
        int pitch = 50;
        int yaw = 50;
        cv::createTrackbar("x", "xyzrpy", &x, 100);
        cv::createTrackbar("y", "xyzrpy", &y, 100);
        cv::createTrackbar("z", "xyzrpy", &z, 100);
        cv::createTrackbar("roll", "xyzrpy", &roll, 100);
        cv::createTrackbar("pitch", "xyzrpy", &pitch, 100);
        cv::createTrackbar("yaw", "xyzrpy", &yaw, 100);
        KDL::Frame pos_end(KDL::Rotation::RPY(1.57, 0, 1.57), KDL::Vector(0, 0, 0));
        while (ros::ok())
        {
            std::cout<<"processing"<<std::endl;
            sensor_msgs::JointState jointState;
            KDL::Rotation rot = KDL::Rotation::RPY(((float)pitch - 50) / 50 * 3.14, ((float)yaw - 50) / 50 * 3.14, ((float)roll - 50) / 50 * 3.14);
            KDL::Vector vec(((float)y - 50) / 100, ((float)z - 50) / 100, ((float)x - 50) / 100);
            KDL::Frame pos_goal = pos_end * KDL::Frame(rot, vec);
            unsigned int n = chain.getNrOfJoints();
            KDL::JntArray q_init(n);
            KDL::JntArray q_sol(n);
            int retval;
            retval = ikSolver->CartToJnt(q_init, pos_goal, q_sol);
            jointState.header.frame_id = "map";
            jointState.header.stamp = ros::Time().now();
            jointState.position.resize(8);
            jointState.name.resize(8);
            jointState.name[0] = "joint1";
            jointState.name[1] = "joint2";
            jointState.name[2] = "joint3";
            jointState.name[3] = "joint4";
            jointState.name[4] = "joint5";
            jointState.name[5] = "joint6";
            jointState.name[6] = "joint7";
            jointState.name[7] = "joint8";
            jointState.position[0] = q_sol(0);
            jointState.position[1] = q_sol(1);
            jointState.position[2] = q_sol(2);
            jointState.position[3] = q_sol(3);
            jointState.position[4] = q_sol(4);
            jointState.position[5] = q_sol(5);
            jointState.position[6] = 0;
            jointState.position[7] = 0;
            pubJointState.publish(jointState);
            std::cout<<q_sol(0)<<"  "<<q_sol(1)<<"  "<<q_sol(2)<<"  "<<q_sol(3)<<"  "<<q_sol(4)<<"  "<<q_sol(5)<<"  "<<std::endl;
            cv::imshow("xyzrpy", image);
            char key = cv::waitKey(10);
            if (key == 27 || key == 'q')
                break;
        }
        cv::destroyAllWindows();
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "joint_publisher");

    Publisher publisher;

    ROS_INFO("\033[1;32m----> Publisher Started.\033[0m");

    publisher.process();
    return 0;
}
