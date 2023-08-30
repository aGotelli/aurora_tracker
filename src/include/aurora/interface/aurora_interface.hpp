/**
 * @file sine_torque_control.hpp
 * @copyright Copyright (c) 2018-2020, New York University and Max Planck
 * Gesellschaft, License BSD-3-Clause
 */

#include <real_time_tools/timer.hpp>
#include <real_time_tools/thread.hpp>
#include <real_time_tools/spinner.hpp>

#include "aurora/auroratracker.h"


#include <yaml-cpp/yaml.h>

namespace aurora_interface
{

class AuroraInterface
{
public:
    /**
     * @brief Construct a new SineTorqueControl object.
     *
     *
     *
     *
     * @param motor_slider_pairs
     */
    AuroraInterface(const unsigned int t_number_of_sensors,
                    std::shared_ptr<bool> t_StopDemos,
                    std::shared_ptr<bool> t_start_recording,
                    unsigned int t_baud_rate=NDIBaudRates::baud_9600,
                    const double t_dt = 0.05);

    /**
     * @brief Destroy the SineTorqueControl object
     */
    ~AuroraInterface();

    /**
     * @brief This method is a helper to start the thread loop.
     */
    void start_loop();
    /**
     * @brief Stop the control and dump the data
     */
    void stop_loop();

    bool m_sensors_are_ready {false};


    std::pair<YAML::Node, Eigen::MatrixXd> getSamplesData()const;


private:

    /**
     * @brief This is the real time thread object.
     */
    real_time_tools::RealTimeThread rt_thread_;

    /**
     * @brief this function is just a wrapper around the actual loop function,
     * such that it can be spawned as a posix thread.
     */
    static THREAD_FUNCTION_RETURN_TYPE loop(void* instance_pointer);

    /**
     * @brief this is a simple control loop which runs at a kilohertz.
     *
     * it reads the measurement from the analog sensor, in this case the
     * slider. then it scales it and sends it as the current target to
     * the motor.
     */
    void loop();

//    /**
//     * @brief managing the stopping of the loop
//     */
//    bool stop_loop_;


    real_time_tools::Spinner m_spinner;



    /**
     * @brief time
     */
    std::vector<double> m_time;

    double m_dt { 0.05 };



    unsigned int m_number_of_sensors { 3 };

    unsigned int m_baud_rate { NDIBaudRates::baud_9600 };

    AuroraTracker m_aurora_tracker { AuroraTracker(m_number_of_sensors, m_baud_rate) };
    std::vector<Eigen::Matrix4d> m_poses {
        std::vector<Eigen::Matrix4d>(m_number_of_sensors)
    };
    std::vector<std::vector<Eigen::Matrix4d>> m_stack_poses { std::vector<std::vector<Eigen::Matrix4d>>(m_number_of_sensors) };


    std::shared_ptr<bool> m_StopDemos;
    std::shared_ptr<bool> m_start_recording;
    unsigned int m_number_of_observations { 0 };

};  // end class SineTorqueControl definition

}  // namespace blmc_drivers
