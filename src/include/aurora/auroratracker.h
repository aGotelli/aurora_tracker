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


class AuroraTracker
{

public:
    AuroraTracker(int num_of_sensors);
    ~AuroraTracker();

    std::vector<Eigen::Matrix4d> getFrames();



    void updateFrame();

    bool  IsTracking;

protected:
    // Flags



private:
    vtkSmartPointer<vtkNDITracker> m_pTracker;

    std::vector<vtkSmartPointer<vtkTrackerTool>> m_pSensors;
    std::vector<Eigen::Matrix4d> m_vSensorFrames;

    int initTrackerAndTool();
    int m_iNumOfSensors;

    Eigen::Matrix4d vtkToEigen(vtkMatrix4x4* vtk_matrix);

};


