#include "aurora/auroratracker.h"


#include <real_time_tools/spinner.hpp>

#include "utilities/Eigen/eigen_io.hpp"


#include <algorithm>
#include <numeric>

#include <yaml-cpp/yaml.h>

#include <fstream>

int main()
{


    const unsigned int weight = 0;

    AuroraTracker aurora_tracker(3);

    Eigen::Matrix4d frame0;
    Eigen::Matrix4d frame1;
    Eigen::Matrix4d frame2;

    Eigen::Matrix4d g_1wrt0;
    Eigen::Matrix4d g_2wrt0;

    Eigen::Vector3d relative_position;

    real_time_tools::Spinner spinner;
    const double dt = 0.01;
    spinner.set_period(dt);

    std::vector<Eigen::Vector3d> p1_stack;
    std::vector<Eigen::Vector3d> p2_stack;


    double time = 0.0;
    const double end_time = 10.0;

    while( time <= end_time ){
        std::cout << "time: " << time << std::endl;

        aurora_tracker.updateFrame();

        frame0 = aurora_tracker.getFrames()[0];
        frame1 = aurora_tracker.getFrames()[1];
        frame2 = aurora_tracker.getFrames()[2];

        g_1wrt0 = frame0.inverse()*frame1;
        g_2wrt0 = frame0.inverse()*frame2;

        p1_stack.push_back( g_1wrt0.block<3, 1>(0, 3) );
        p2_stack.push_back( g_2wrt0.block<3, 1>(0, 3) );


        spinner.spin();
        time += dt;
    }

    double x_sum = 0.0;
    for(const auto p : p1_stack){
        x_sum += p.x();
    }

    double x_mean = x_sum/p1_stack.size();
    double x_std = 0.0;
    for(const auto p : p1_stack){
        x_std += std::pow(p.x() - x_mean, 2);
    }
    x_std = std::sqrt(x_std/p1_stack.size());

    std::cout << "x_mean: " << x_mean << std::endl;
    std::cout << "x_std: " << x_std << std::endl;


    Eigen::MatrixXd p1_matrix(3, p1_stack.size());
    Eigen::MatrixXd p2_matrix(3, p2_stack.size());

    for(unsigned int i = 0; i < p1_stack.size(); i++ ){
        p1_matrix.col(i) = p1_stack[i];
        p2_matrix.col(i) = p2_stack[i];
    }

    Eigen::Vector3d p1_mean = p1_matrix.rowwise().mean();
    Eigen::Vector3d p2_mean = p2_matrix.rowwise().mean();

    Eigen::MatrixXd p1_centered = p1_matrix.colwise() - p1_mean;
    Eigen::MatrixXd p2_centered = p2_matrix.colwise() - p2_mean;

    //  Compute the standard devitation
    Eigen::VectorXd p1_std = p1_centered.rowwise().norm()/std::sqrt(p1_stack.size());
    Eigen::VectorXd p2_std = p2_centered.rowwise().norm()/std::sqrt(p2_stack.size());



    //  Log the results
    std::cout << "p1_mean: " << p1_mean.transpose() << std::endl;
    std::cout << "p2_mean: " << p2_mean.transpose() << std::endl;

    std::cout << "p1_std: " << p1_std.transpose() << std::endl;
    std::cout << "p2_std: " << p2_std.transpose() << std::endl;




    YAML::Emitter out;
    out << YAML::BeginMap;
    out << YAML::Key << "weight [g]";
    out << YAML::Value << weight;
    out << YAML::Key << "p1_mean";
    out << YAML::BeginMap;
    out << YAML::Key << "x";
    out << YAML::Value << p1_mean.x();
    out << YAML::Key << "y";
    out << YAML::Value << p1_mean.y();
    out << YAML::Key << "z";
    out << YAML::Value << p1_mean.z();
    out << YAML::EndMap;
    out << YAML::Key << "p1_std";
    out << YAML::BeginMap;
    out << YAML::Key << "x";
    out << YAML::Value << p1_std.x();
    out << YAML::Key << "y";
    out << YAML::Value << p1_std.y();
    out << YAML::Key << "z";
    out << YAML::Value << p1_std.z();
    out << YAML::EndMap;
    out << YAML::Key << "p2_mean";
    out << YAML::BeginMap;
    out << YAML::Key << "x";
    out << YAML::Value << p2_mean.x();
    out << YAML::Key << "y";
    out << YAML::Value << p2_mean.y();
    out << YAML::Key << "z";
    out << YAML::Value << p2_mean.z();
    out << YAML::EndMap;
    out << YAML::Key << "p2_std";
    out << YAML::BeginMap;
    out << YAML::Key << "x";
    out << YAML::Value << p2_std.x();
    out << YAML::Key << "y";
    out << YAML::Value << p2_std.y();
    out << YAML::Key << "z";
    out << YAML::Value << p2_std.z();
    out << YAML::EndMap;
    out << YAML::EndMap;

    const std::string name = "calibration" + std::to_string(weight) + "g.yaml";
    std::ofstream file(name.c_str());
    file << out.c_str();
    file.close();


    return 0;
}
