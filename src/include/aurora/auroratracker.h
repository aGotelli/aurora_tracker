#pragma once


#include <queue>
#include <math.h>
#include "vtkNDITracker.h"
#include "vtkTrackerTool.h"
#include <vtkTransform.h>
#include <vtkSmartPointer.h>
#include <vtkMatrix4x4.h>
#include "Eigen/Core"
#include "Eigen/Geometry"


#ifdef USE_MATH_TOOLS
#include "math_tools/LieAlgebra/lie_algebra_utilities.hpp"
#endif



class AuroraTracker
{

public:
    AuroraTracker(int num_of_sensors);
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
    vtkSmartPointer<vtkNDITracker> m_pTracker;

    std::vector<vtkSmartPointer<vtkTrackerTool>> m_pSensors;
    std::vector<Eigen::Matrix4d> m_vSensorFrames;

#ifdef USE_MATH_TOOLS
    std::vector<::LieAlgebra::SE3Pose> m_sensor_poses_stack;
#endif
    

    int initTrackerAndTool();
    int m_iNumOfSensors;

    Eigen::Matrix4d vtkToEigen(vtkMatrix4x4* vtk_matrix);

};


