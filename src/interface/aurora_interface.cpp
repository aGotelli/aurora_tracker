#include "aurora/interface/aurora_interface.hpp"

#include "utilities/Eigen/eigen_io.hpp"


namespace aurora_interface
{


AuroraInterface::AuroraInterface(const unsigned int t_number_of_sensors,
                                 std::shared_ptr<bool> t_StopDemos,
                                 std::shared_ptr<bool> t_start_recording,
                                 unsigned int t_baud_rate,
                                 const double t_dt)
    :   m_dt(t_dt),
        m_number_of_sensors(t_number_of_sensors),
        m_baud_rate(t_baud_rate),
        m_StopDemos(t_StopDemos),
        m_start_recording(t_start_recording)
{


    std::cout << "Setting for " << m_number_of_sensors << " sensors..." << std::endl;
    m_time.clear();



    m_spinner.set_period( 0.01 );

//    stop_loop_ = false;

    //  Wait for the sensors to be ready
    m_sensors_are_ready = false;
    while (!m_sensors_are_ready
           and !*m_StopDemos)
    {
        m_aurora_tracker.updateFrame();

        for(unsigned int i=0; i < m_number_of_sensors; i++){
            m_poses[i] = m_aurora_tracker.getFrames()[i];
        }

        const auto it = std::find_if(m_poses.begin(), m_poses.end(), [](Eigen::Matrix4d pose){ return pose.isIdentity(); });

        if(it == m_poses.end()){
            m_sensors_are_ready = true;
        }else{
            std::cout << "Sensor :" << std::distance(m_poses.begin(), it) << " is not ready" << std::endl;
        }

        real_time_tools::Timer::sleep_sec(0.01);
    }

    std::cout << "Sensors are ready" << std::endl;

    
}


AuroraInterface::~AuroraInterface()
{
    rt_printf("\n\n\n           Destructor\n\n\n");
//    stop_loop_ = true;
    rt_thread_.join();
}


void AuroraInterface::start_loop()
{
    rt_thread_.create_realtime_thread(&AuroraInterface::loop, this);
}



THREAD_FUNCTION_RETURN_TYPE AuroraInterface::loop(void* instance_pointer)
{
    ((AuroraInterface*)(instance_pointer))->loop();
    return THREAD_FUNCTION_RETURN_VALUE;
}


void AuroraInterface::loop()
{


    double current_time = 0.0;
//    double previus_iteration_end_time = 0.0;
//    double delta_time = 0.0;
//    unsigned int snapshot_number=0;
    m_sensors_are_ready = true;

    std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();
    std::chrono::high_resolution_clock::time_point current;

    std::chrono::high_resolution_clock::duration time_elapsed;
    while (!*m_StopDemos)
    {

        if(*m_start_recording){
            current = std::chrono::high_resolution_clock::now();
            time_elapsed = current - start;

            current_time = time_elapsed.count() / 1e9;
            m_time.push_back(current_time);

            m_aurora_tracker.updateFrame();

            for(unsigned int i=0; i < m_number_of_sensors; i++){
                m_stack_poses[i].push_back(m_aurora_tracker.getFrames()[i]);
            }

            m_number_of_observations++;
        }


        m_spinner.spin();

        


//        snapshot_number++;

        
//        delta_time = current_time - previus_iteration_end_time;
//        // local_time += m_dt;
//        while(!stop_loop_ and
//              delta_time < m_dt*1000){
//            m_spinner.spin();


//            current = std::chrono::high_resolution_clock::now();
//            elapsed = current - start;
//            current_time = std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count();

//            delta_time = current_time - previus_iteration_end_time;
//        }

//        previus_iteration_end_time = current_time;

    }      // endwhile
}

void AuroraInterface::stop_loop()
{
//    stop_loop_ = true;

}

std::pair<YAML::Node, Eigen::MatrixXd> AuroraInterface::getSamplesData()const
{



    //  Data is stacked in column so there will be a column for every timestep
    //  The the data in the row is the following :
    //  time
    //  quaternion
    //  position
    //  quaternion
    //  position
    //  ...


    //  The matrix has 1 + Ns*(4+3) rows
    //  and Nsteps cols


    const unsigned int number_of_rows = m_number_of_sensors*(4 + 3) + 1;

    Eigen::MatrixXd poses_as_matrix = Eigen::MatrixXd(number_of_rows, m_number_of_observations);


    //  Copy all the timestamps
    for(unsigned int i=0; i<m_time.size(); i++)
        poses_as_matrix(0, i) = m_time[i];


    //  First define how ma
    unsigned int start_row  = 0;

    Eigen::Quaterniond q;
    Eigen::Vector3d p;
    for(unsigned int sensor=0; const auto &poses : m_stack_poses){

        std::cout << "Sensor : " << sensor << std::endl;
        start_row = (sensor++)*(4 + 3) + 1;
        for(unsigned int index=0; const auto &pose : poses){


            q = Eigen::Quaterniond( pose.block<3, 3>(0, 0) );
            p = pose.block<3, 1>(0, 3);

            poses_as_matrix.block<4 + 3, 1>(start_row, index++) << q.w(), q.x(), q.y(), q.z(),
                                                                    p.x(), p.y(), p.z();
        }

    }


    YAML::Node node;
    node["baud_rate"] = m_baud_rate;

    node["number_of_sensors"] = m_number_of_sensors;
    node["m_number_of_observations"] = m_number_of_observations;

    node["data_order"].push_back("time");

    node["data_order"].push_back("Qw");
    node["data_order"].push_back("Qx");
    node["data_order"].push_back("Qy");
    node["data_order"].push_back("Qz");

    node["data_order"].push_back("px");
    node["data_order"].push_back("py");
    node["data_order"].push_back("pz");


    return {node, poses_as_matrix};
}




}  // namespace blmc_drivers
