#include "aurora/auroratracker.h"


#include <real_time_tools/spinner.hpp>


#include <algorithm>
#include <numeric>

#include <yaml-cpp/yaml.h>

#include <fstream>

int main()
{


    const unsigned int weight = 2500;

    const unsigned int number_of_sensors = 3;
    unsigned int baud_rate;
    baud_rate = 9600;   //  works
//    baud_rate =  14400;   //  ERROR: vtkNDITracker (0x55e85c483e30): Host not capable of given communications parameters
//    baud_rate =  19200;   //  stuck forever
//    baud_rate =  38400;   //  works
//    baud_rate =  57600;   //  works
//    baud_rate = 115200;   //  works
//    baud_rate = 230400;   //  ERROR: vtkNDITracker (0x55c526506e30): Incorrect number of command parameters
//    baud_rate = 921600;   //  ERROR: vtkNDITracker (0x55e85c483e30): Host not capable of given communications parameters
//    baud_rate = 1228739;  //  ERROR: vtkNDITracker (0x563356aafe30): Unable to set up new communication parameters


    AuroraTracker aurora_tracker(number_of_sensors, baud_rate);


    std::vector<Eigen::Matrix4d> poses(number_of_sensors);
    std::vector<std::vector<Eigen::Matrix4d>> stack_relative_poses(number_of_sensors);




    real_time_tools::Spinner spinner;


    const double dt = 0.01;
    spinner.set_period(dt);

    


    double time = 0.0;
    const double end_time = 10.0;






    while( time <= end_time ){

        aurora_tracker.updateFrame();

        for(unsigned int i=0; i < number_of_sensors; i++){
            poses[i] = aurora_tracker.getFrames()[i];
        }
        

        auto it = std::find_if(poses.begin(), poses.end(), [](Eigen::Matrix4d pose){ return pose.isIdentity(); });

        if(it == poses.end())/*
            std::cout << "Sensor " << std::distance(poses.begin(), it) << " is not visible" << std::endl;
        else*/{
            //  All sensors are visible

            //  Log time
            if(static_cast<int>(time*100) % 100 == 0)
                std::cout << "time: " << time << std::endl;

            //  Compute the relative poses
            for(unsigned int i=0; i < number_of_sensors; i++){
                const auto relative_pose = poses[0].inverse() * poses[i];
                stack_relative_poses[i].push_back( relative_pose );
            }

            //  Update time
            time += dt;
        }

        spinner.spin();
        
    }


    const unsigned int number_of_snapshots = stack_relative_poses[0].size();


    //  Data processing
    YAML::Node node;
    node["weight [g]"] = weight;

    for(unsigned int sensor_numb=0; sensor_numb < number_of_sensors; sensor_numb++){
        Eigen::MatrixXd stack_relative_position    = Eigen::MatrixXd::Zero(3, stack_relative_poses[sensor_numb].size());
        Eigen::MatrixXd stack_relative_orientation = Eigen::MatrixXd::Zero(3, stack_relative_poses[sensor_numb].size());



        // Positions stacks
        for(unsigned int snapshot=0; snapshot < number_of_snapshots; snapshot++){

            stack_relative_position.col(snapshot) = stack_relative_poses[sensor_numb][snapshot].block<3,1>(0,3);
            
            Eigen::Matrix3d relative_Rotation_matrix = stack_relative_poses[sensor_numb][snapshot].block<3,3>(0,0);
            Eigen::Vector3d euler_angles = relative_Rotation_matrix.eulerAngles(0,1,2);

            stack_relative_orientation.col(snapshot) = euler_angles;

        }


        //  Compute the mean
        Eigen::Vector3d position_mean = stack_relative_position.rowwise().mean();
        Eigen::Vector3d euler_angles_mean = stack_relative_orientation.rowwise().mean();


        //  Center the data
        Eigen::MatrixXd position_centered = stack_relative_position.colwise() - position_mean;
        Eigen::MatrixXd euler_angles_centered = stack_relative_orientation.colwise() - euler_angles_mean;


        //  Compute the standard devitation
        Eigen::VectorXd position_std = position_centered.rowwise().norm()/std::sqrt(stack_relative_position.size());
        Eigen::VectorXd euler_angles_std = euler_angles_centered.rowwise().norm()/std::sqrt(stack_relative_orientation.size());




        


        YAML::Node sensor;
        sensor["position"]["mean"]["x"] = position_mean.x();
        sensor["position"]["mean"]["y"] = position_mean.y();
        sensor["position"]["mean"]["z"] = position_mean.z();
        sensor["position"]["std"]["x"] = position_std.x();
        sensor["position"]["std"]["y"] = position_std.y();
        sensor["position"]["std"]["z"] = position_std.z();
        sensor["orientation"]["mean"]["x"] = euler_angles_mean.x();
        sensor["orientation"]["mean"]["y"] = euler_angles_mean.y();
        sensor["orientation"]["mean"]["z"] = euler_angles_mean.z();
        sensor["orientation"]["std"]["x"] = euler_angles_std.x();
        sensor["orientation"]["std"]["y"] = euler_angles_std.y();
        sensor["orientation"]["std"]["z"] = euler_angles_std.z();




        node["sensors"]["sensor_"+std::to_string(sensor_numb)] = sensor;

    }


    

    const std::string name = "calibration" + std::to_string(weight) + "g.yaml";
    const std::string path = "../../data/";
    std::ofstream fout( path + name);
    fout << node;
    fout.close();

    return 0;
}
