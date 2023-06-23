#pragma once


#include <queue>
#include <math.h>
#include "vtkNDITracker.h"
#include "vtkTrackerTool.h"
#include <vtkTransform.h>
#include <vtkSmartPointer.h>
#include <vtkMatrix4x4.h>
//#include "Eigen/Core"
//#include "Eigen/Geometry"


#ifdef USE_MATH_TOOLS
#include "math_tools/LieAlgebra/lie_algebra_utilities.hpp"
#endif


enum NDIBaudRates {
    baud_9600    =    9600, //  works
    baud_14400   =   14400, //  ERROR: vtkNDITracker (0x55e85c483e30): Host not capable of given communications parameters
    baud_19200   =   19200, //  stuck forever
    baud_38400   =   38400, //  works
    baud_57600   =   57600, //  works
    baud_115200  =  115200, //  works
    baud_230400  =  230400, //  ERROR: vtkNDITracker (0x55c526506e30): Incorrect number of command parameters
    baud_921600  =  921600, //  ERROR: vtkNDITracker (0x55e85c483e30): Host not capable of given communications parameters
    baud_1228739 = 1228739  //  ERROR: vtkNDITracker (0x563356aafe30): Unable to set up new communication parameters
};


class AuroraTracker
{

public:
    AuroraTracker(const int t_num_of_sensors, const int t_baud_rate=9600);
    ~AuroraTracker();

    std::vector<Eigen::Matrix4d> getFrames();

#ifdef USE_MATH_TOOLS
    std::vector<::LieAlgebra::SE3Pose> getFramesInSE3();
#endif


    void updateFrame();

    bool  IsTracking;

protected:
    // Flags



private:
    int baud_rate { 9600 };

    int m_iNumOfSensors { 0 };

    vtkSmartPointer<vtkNDITracker> m_pTracker;

    std::vector<vtkSmartPointer<vtkTrackerTool>> m_pSensors;
    std::vector<Eigen::Matrix4d> m_vSensorFrames;

#ifdef USE_MATH_TOOLS
    std::vector<::LieAlgebra::SE3Pose> m_sensor_poses_stack;
#endif
    

    int initTrackerAndTool();


    Eigen::Matrix4d vtkToEigen(vtkMatrix4x4* vtk_matrix);

};


