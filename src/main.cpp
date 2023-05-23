#include "aurora/auroratracker.h"


#include <real_time_tools/spinner.hpp>

#include "utilities/Eigen/eigen_io.hpp"


#include <algorithm>
#include <numeric>

#include <yaml-cpp/yaml.h>

#include <fstream>

int main()
{


    const unsigned int weight = 1000;

    AuroraTracker aurora_tracker(3);


    Eigen::Matrix4d g_0;
    Eigen::Matrix4d g_1;
    Eigen::Matrix4d g_2;

    Eigen::Matrix4d _0g_0;
    Eigen::Matrix4d _0g_1;
    Eigen::Matrix4d _0g_2;



    std::vector<Eigen::Matrix4d> stack_0g_0;
    std::vector<Eigen::Matrix4d> stack_0g_1;
    std::vector<Eigen::Matrix4d> stack_0g_2;



    real_time_tools::Spinner spinner;


    const double dt = 0.01;
    spinner.set_period(dt);

    


    double time = 0.0;
    const double end_time = 10.0;






    while( time <= end_time ){
        std::cout << "time: " << time << std::endl;

        aurora_tracker.updateFrame();

        g_0 = aurora_tracker.getFrames()[0];
        g_1 = aurora_tracker.getFrames()[1];
        g_2 = aurora_tracker.getFrames()[2];

        if(!g_0.isIdentity() || !g_1.isIdentity() || !g_2.isIdentity()){
            _0g_0 = g_0.inverse() * g_0;
            _0g_1 = g_0.inverse() * g_1;
            _0g_2 = g_0.inverse() * g_2;


            stack_0g_0.push_back( _0g_0 );
            stack_0g_1.push_back( _0g_1 );
            stack_0g_2.push_back( _0g_2 );
        }

        




        spinner.spin();
        time += dt;
    }


    Eigen::MatrixXd stack_0p_0(3, stack_0g_0.size());
    Eigen::MatrixXd stack_0p_1(3, stack_0g_1.size());
    Eigen::MatrixXd stack_0p_2(3, stack_0g_2.size());


    Eigen::MatrixXd stack_0R_0(3, stack_0g_0.size());
    Eigen::MatrixXd stack_0R_1(3, stack_0g_1.size());
    Eigen::MatrixXd stack_0R_2(3, stack_0g_2.size());



    // Positions stacks
    for(unsigned int i=0; i < stack_0g_0.size(); i++){

        stack_0p_0.col(i) = stack_0g_0[i].block<3,1>(0,3);
        stack_0p_1.col(i) = stack_0g_1[i].block<3,1>(0,3);
        stack_0p_2.col(i) = stack_0g_2[i].block<3,1>(0,3);


        Eigen::Matrix3d _0R_0 = stack_0g_0[i].block<3,3>(0,0);
        Eigen::Matrix3d _0R_1 = stack_0g_1[i].block<3,3>(0,0);
        Eigen::Matrix3d _0R_2 = stack_0g_2[i].block<3,3>(0,0);


        Eigen::Vector3d euler_angles_0 = _0R_0.eulerAngles(0,1,2);
        Eigen::Vector3d euler_angles_1 = _0R_1.eulerAngles(0,1,2);
        Eigen::Vector3d euler_angles_2 = _0R_2.eulerAngles(0,1,2);


        stack_0R_0.col(i) = euler_angles_0;
        stack_0R_1.col(i) = euler_angles_1;
        stack_0R_2.col(i) = euler_angles_2;

    }



    //  Compute the mean
    Eigen::Vector3d p0_mean = stack_0p_0.rowwise().mean();
    Eigen::Vector3d p1_mean = stack_0p_1.rowwise().mean();
    Eigen::Vector3d p2_mean = stack_0p_2.rowwise().mean();

    Eigen::Vector3d R0_mean = stack_0R_0.rowwise().mean();
    Eigen::Vector3d R1_mean = stack_0R_1.rowwise().mean();
    Eigen::Vector3d R2_mean = stack_0R_2.rowwise().mean();


    //  Center the data
    Eigen::MatrixXd p0_centered = stack_0p_0.colwise() - p0_mean;
    Eigen::MatrixXd p1_centered = stack_0p_1.colwise() - p1_mean;
    Eigen::MatrixXd p2_centered = stack_0p_2.colwise() - p2_mean;

    Eigen::MatrixXd R0_centered = stack_0R_0.colwise() - R0_mean;
    Eigen::MatrixXd R1_centered = stack_0R_1.colwise() - R1_mean;
    Eigen::MatrixXd R2_centered = stack_0R_2.colwise() - R2_mean;


    //  Compute the standard devitation
    Eigen::VectorXd p0_std = p0_centered.rowwise().norm()/std::sqrt(stack_0p_0.size());
    Eigen::VectorXd p1_std = p1_centered.rowwise().norm()/std::sqrt(stack_0p_1.size());
    Eigen::VectorXd p2_std = p2_centered.rowwise().norm()/std::sqrt(stack_0p_2.size());

    Eigen::VectorXd R0_std = R0_centered.rowwise().norm()/std::sqrt(stack_0R_0.size());
    Eigen::VectorXd R1_std = R1_centered.rowwise().norm()/std::sqrt(stack_0R_1.size());
    Eigen::VectorXd R2_std = R2_centered.rowwise().norm()/std::sqrt(stack_0R_2.size());



    

    YAML::Node node;

    node["weight [g]"] = weight;
    for(unsigned int i=0; i<3; i++){

        node["sensors"]["0"]["position"]["mean"].push_back( p0_mean[i] );
        node["sensors"]["0"]["position"]["std"].push_back( p0_std[i] );

        node["sensors"]["0"]["orientation"]["mean"].push_back( R0_mean[i] );
        node["sensors"]["0"]["orientation"]["std"].push_back( R0_std[i] );


        node["sensors"]["1"]["position"]["mean"].push_back( p1_mean[i] );
        node["sensors"]["1"]["position"]["std"].push_back( p1_std[i] );

        node["sensors"]["1"]["orientation"]["mean"].push_back( R1_mean[i] );
        node["sensors"]["1"]["orientation"]["std"].push_back( R1_std[i] );


        node["sensors"]["2"]["position"]["mean"].push_back( p2_mean[i] );
        node["sensors"]["2"]["position"]["std"].push_back( p2_std[i] );

        node["sensors"]["2"]["orientation"]["mean"].push_back( R2_mean[i] );
        node["sensors"]["2"]["orientation"]["std"].push_back( R2_std[i] );


    }
    

    const std::string name = "calibration" + std::to_string(weight) + "g.yaml";
    const std::string path = "../../data/";
    std::ofstream fout( path + name);
    fout << node;
    fout.close();

    return 0;
}
