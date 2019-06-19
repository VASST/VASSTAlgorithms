/*=auto=========================================================================

  Portions (c) Copyright 2005 Brigham and Women's Hospital (BWH) All Rights Reserved.

  See COPYRIGHT.txt
  or http://www.slicer.org/copyright/copyright.txt for details.

  Program:   3D Slicer

=========================================================================auto=*/

// Logic includes
#include "vtkSlicerLinesIntersectionLogic.h"

// MRML includes
#include "vtkMRMLScene.h"

// VTK includes
#include <vtkMatrix4x4.h>

// STL includes
#include <fstream>
#include <limits>

// OS includes
#include <math.h>

//-----------------------------------------------------------------------------
bool AreSame(double a, double b)
{
  return fabs(a - b) < std::numeric_limits<double>::epsilon();
}

//-----------------------------------------------------------------------------
int vtkSlicerLinesIntersectionLogicTest1(int, char* [])
{
  vtkNew<vtkMRMLScene> scene;

  vtkNew<vtkSlicerLinesIntersectionLogic> moduleLogic;
  moduleLogic->SetMRMLScene(scene);
  if (moduleLogic->GetMRMLScene() != scene)
  {
    std::cerr << "A MRML Scene must be set to go further." << std::endl;
    return EXIT_FAILURE;
  }

  static double goldStandard[3] = { 0.5, 0.5, 0.5 };

  {
    double origin[3] = { 1.0, 0.0, 0.0 };
    double direction[3] = { 0.0, 1.0, 0.0 };
    moduleLogic->AddLine(origin, direction);
  }
  {
    double origin[3] = { 0.0, 1.0, 0.0 };
    double direction[3] = { 0.0, 0.0, 1.0 };
    moduleLogic->AddLine(origin, direction);
  }
  {
    double origin[3] = { 0.0, 0.0, 1.0 };
    double direction[3] = { 1.0, 0.0, 0.0 };
    moduleLogic->AddLine(origin, direction);
  }

  double* result = moduleLogic->Update();
  if (result == nullptr)
  {
    std::cerr << "Test failed." << std::endl;
    return EXIT_FAILURE;
  }
  // double error = moduleLogic->GetError();

  for (int i = 0; i < 3; ++i)
  {
    if (!AreSame(result[i], goldStandard[i]))
    {
      std::cerr << "Points do not match." << std::endl << std::endl;
      std::cerr << "Calculated: " << result[0] << "," << result[1] << "," << result[2] << std::endl;
      std::cerr << "Reference: " << goldStandard[0] << "," << goldStandard[1] << "," << goldStandard[2] << std::endl;
      return EXIT_FAILURE;
    }
  }

  return EXIT_SUCCESS;
}
