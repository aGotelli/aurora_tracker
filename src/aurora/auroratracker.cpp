#include "blmc_drivers/aurora_tracking/auroratracker.h"
#include <iostream>


AuroraTracker::AuroraTracker(int num_of_sensors)
{
    IsTracking = false;

    m_iNumOfSensors = num_of_sensors;

    for(int i = 0 ; i < m_iNumOfSensors ; i++)
    {
        m_vSensorFrames.push_back(Eigen::Matrix4d::Identity());
    }

    if(initTrackerAndTool() == 1)
    {
        std::cout << "Aurora Tracker initialized!" << std::endl;
        IsTracking = true;
    }
    else
    {
        std::cout << "Could not initialize Aurora Tracker!" << std::endl;
    }
}

AuroraTracker::~AuroraTracker()
{
    if(m_pTracker->IsTracking())// check if the tracker still running
        m_pTracker->StopTracking();
}

std::vector<Eigen::Matrix4d> AuroraTracker::getFrames()
{
    return m_vSensorFrames;
}


void AuroraTracker::updateFrame()
{
    for(int i = 0 ; i < m_iNumOfSensors; i++)
    {
        m_pSensors.at(i)->Update();
        if (m_pSensors.at(i)->IsOutOfView())
        {
          //std::cout << "Tool " << i << " missing" << std::endl;
          continue;
        }
        else
        {
          vtkSmartPointer<vtkTransform> toolTransform = vtkSmartPointer<vtkTransform>::New();
          toolTransform = m_pSensors.at(i)->GetTransform(); // get the transform of the tool with respect to the tracker reference frame
          vtkSmartPointer<vtkMatrix4x4> matrix = vtkSmartPointer<vtkMatrix4x4>::New();
          toolTransform->GetMatrix(matrix);

          Eigen::Matrix4d frame = vtkToEigen(matrix);

          //From mm to m
          Eigen::Vector3d pos = frame.block(0,3,3,1);
          frame.block(0,3,3,1) = (1e-3)*pos;
          m_vSensorFrames.at(i) = frame;
        }
    }
}


int AuroraTracker::initTrackerAndTool()
{
    //std::cout << "init" << std::endl;
    m_pTracker = vtkSmartPointer<vtkNDITracker>::New();
    m_pTracker->SetSerialDevice("/dev/ttyUSB0");
    m_pTracker->Probe();

    // beep once
    m_pTracker->Command("BEEP:1");
    // start tracking and activate the tool handles
    m_pTracker->StartTracking(); // tracker starts .
    m_pTracker->Update();

    for(int i = 0; i < m_iNumOfSensors; i++)
    {
        m_pSensors.push_back(m_pTracker->GetTool(i));
    }

    return m_pTracker->IsTracking();
}

Eigen::Matrix4d AuroraTracker::vtkToEigen(vtkMatrix4x4* vtk_matrix)
{
  Eigen::Matrix4d eigen_matrix = Eigen::Matrix4d::Identity ();
  for (int i=0; i < 4; i++)
  {
    for (int j=0; j < 4; j++)
    {
      // VTK
      eigen_matrix (i, j) = vtk_matrix->GetElement (i, j);
    }
  }
  return eigen_matrix;
}


