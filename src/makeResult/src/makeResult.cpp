#define _USE_MATH_DEFINES

#include <cmath>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/ISAM2.h>

#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

#include <ros/ros.h>
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Path.h"

class mapOptimizer{
    
    private:
        ros::NodeHandle nh;
        std::string pose_topic;

        ros::Publisher pubPath;
        ros::Publisher pubOptimizedPath;
        ros::Subscriber subNdtPose;
        ros::Subscriber subLanding;
        
        gtsam::NonlinearFactorGraph gtSAMgraph;
        gtsam::Values initialEstimate;
        gtsam::Values optimizedEstimate;
        gtsam::ISAM2 *isam;
        gtsam::Values isamCurrentEstimate;

        gtsam::noiseModel::Diagonal::shared_ptr priorNoise;
        gtsam::noiseModel::Diagonal::shared_ptr odometryNoise;
        gtsam::noiseModel::Diagonal::shared_ptr constraintNoise;
        gtsam::noiseModel::Base::shared_ptr robusNoiseModel;
        gtsam::ISAM2Params parameters;
    
        nav_msgs::Path path;
        nav_msgs::Path optimizedPath;

        float lastPose[3];
        bool loopDetected;
        bool isInit = false;

        float startPoint[3] = {0.2, 0.2, 3*M_PI/4};
        

    public:
        mapOptimizer():
            nh("~")
            {
                nh.param<std::string>("poseTopic", pose_topic, "/ndtpso_slam_node/pose_front");
                pubPath = nh.advertise<nav_msgs::Path>("/resultPath",5);
                pubOptimizedPath = nh.advertise<nav_msgs::Path>("/resultOptimizedPath",5);
                subNdtPose = nh.subscribe<geometry_msgs::PoseStamped>(pose_topic, 5, &mapOptimizer::poseHandler,this);
                subLanding = nh.subscribe<std_msgs::Bool<("/Landing",1, &mapOptimized::loopclosing,this);
                init();
            }

        void init()
        {   
            parameters.relinearizeThreshold = 0.01;
            parameters.relinearizeSkip = 1;
            isam = new gtsam::ISAM2(parameters);

            gtsam::Vector Vector3(3);
            Vector3 << 1e-6, 1e-6, 1e-6;
            priorNoise = gtsam::noiseModel::Diagonal::Variances(Vector3);
            odometryNoise = gtsam::noiseModel::Diagonal::Variances(Vector3);

            gtSAMgraph.add(gtsam::PriorFactor<gtsam::Pose2>(0, gtsam::Pose2(startPoint[0],startPoint[1],startPoint[2]),priorNoise));
            initialEstimate.insert(0,gtsam::Pose2(startPoint[0],startPoint[1],startPoint[2]));

            isam->update(gtSAMgraph, initialEstimate);
            isam->update();
            gtSAMgraph.resize(0);
            initialEstimate.clear();
            isam->calculateEstimate();

            for (int i = 0; i < 3; i++)
            {
                lastPose[i] = startPoint[i];
            }

            loopDetected = false;
            isInit = true;
        }

        void addNode(const geometry_msgs::PoseStampedConstPtr& ndt_pose)
        {
            geometry_msgs::PoseStamped pose;
            path.header = ndt_pose->header;
            pose.header = ndt_pose->header;
            pose.pose = ndt_pose->pose;
            float theta = std::atan2(2 * (pose.pose.orientation.w*pose.pose.orientation.z + pose.pose.orientation.x*pose.pose.orientation.y),(1 - 2 * (pose.pose.orientation.y * pose.pose.orientation.y + pose.pose.orientation.z * pose.pose.orientation.z)));
            //need to align coordinate between map and base_link
            if(isInit)
            {

                path.poses.push_back(pose);
                gtsam::Pose2 poseFrom = gtsam::Pose2(lastPose[0], lastPose[1], lastPose[2]);
                gtsam::Pose2 poseTo = gtsam::Pose2(pose.pose.position.x, pose.pose.position.y, theta);
                gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose2>(path.poses.size()-1, path.poses.size(), poseFrom.between(poseTo), odometryNoise));
                initialEstimate.insert(path.poses.size(),poseTo);

                isam->update(gtSAMgraph, initialEstimate);
                isam->update();
                gtSAMgraph.resize(0);
                initialEstimate.clear();
                isam->calculateEstimate();
                pubPath.publish(path);
            }
            //Need update last pose
            lastPose[0] = pose.pose.position.x;
            lastPose[1] = pose.pose.position.y;
            lastPose[2] = theta;
        }

        void loopclosing(const std_msgs::BoolConstPtr& landing)
        {   
            if (landing->data)
            {
            float noiseScore = 0.5;
            gtsam::Vector Vector3(3);
            Vector3 << noiseScore, noiseScore, noiseScore;
            constraintNoise = gtsam::noiseModel::Diagonal::Variances(Vector3);
            robusNoiseModel = gtsam::noiseModel::Robust::Create(
                gtsam::noiseModel::mEstimator::Cauchy::Create(1),
                gtsam::noiseModel::Diagonal::Variances(Vector3)
            );
            
            
            // this code assume that we should be on the "H" marker, not only starting time but also ending time. 
            // May need a check that it works well, there will be too many node in the starting point...
            /*
            pcl::IterativeClosestPoint<pcl::PointXY,pcl::PointXY> icp;
            icp.setMaxCorrespondenceDistance(100);
            icp.setMaximumIterations(100);
            icp.setTransformationEpsilon(1e-6);
            icp.setEuclideanFitnessEpsilon(1e-6);
            icp.setRANSACIterations(0);

            icp.setInputSource();
            icp.setInputTarget();
            pcl::PointCloud<pcl::PointXY>::Ptr result_pc(new pcl::PointCloud<pcl::PointXY>());
            icp.align(*result_pc);

            if (icp.hasConverged())
            {
                std::cout << "Loop closing succesfull" << std::endl;
            }

            //Maybe we need to check whether appropriate type was used.
            Eigen::MatrixXf correctionFrame = icp.getFinalTransformation();
            //How can we change under code for case of 2D
            pcl::getTranslationAndEulerAngles(correctionFrame, x, y, z, roll, pitch, yaw);
            */

            gtsam::Pose2 poseFrom = gtsam::Pose2(0.2 ,0.2, -M_PI/4); // Let's assume that we land at the very close position.
            gtsam::Pose2 poseTo = gtsam::Pose2(0, 0, 0); // From Scan Context Loop Closing Usage...

            gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose2>(0,path.poses.size(),poseFrom.between(poseTo),robusNoiseModel));
            isam->update(gtSAMgraph);
            isam->update();
            gtSAMgraph.resize(0);
            isamCurrentEstimate = isam->calculateEstimate();
            updatePath(isamCurrentEstimate);
            }
        }

        void updatePath(gtsam::Values optimized2DPoses)
        {
            optimizedPath.header = path.header;
            for (int i=0; i<path.poses.size(); i++)
            {
                geometry_msgs::PoseStamped tmpPose;
                tmpPose= path.poses[i];
                gtsam::Pose2 optimized2DPose = optimized2DPoses.at<gtsam::Pose2>(i);
                tmpPose.pose.position.x = optimized2DPose.translation().x();
                tmpPose.pose.position.y = optimized2DPose.translation().y();
                optimizedPath.poses.push_back(tmpPose);
            }
            pubOptimizedPath.publish(optimizedPath);
        }

        void poseHandler(const geometry_msgs::PoseStampedConstPtr& ndt_pose)
        {
        // TODO 
        // 1. Transformation Matrix : we need a transformation matrix between 'lidar and camera' or other matrix that can tell us about tf between lidar and camera
        // 2. Keyframe Selection : Now we use all poses from ndt_pso to build a pose graph, but later maybe we need to select the key poses to reduce the number of the pose_nodes.

        // build a pose graph. try to use same way the SC_LEGO_LOAM use.
            addNode(ndt_pose);
        }

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "result_maker");
    ROS_INFO("<-------------------Result Maker Ready------------------->");
    mapOptimizer MO;
    ros::Rate rate(200);
    
    while (ros::ok())
    {   
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

//TODO
//1. Get the corresponding frame detecting object
//2. Classify with class name
//3. filtering its poses
//4. make a text file