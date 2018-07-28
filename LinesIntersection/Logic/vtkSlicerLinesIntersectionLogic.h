/*==============================================================================

  Program: 3D Slicer

  Portions (c) Copyright Brigham and Women's Hospital (BWH) All Rights Reserved.

  See COPYRIGHT.txt
  or http://www.slicer.org/copyright/copyright.txt for details.

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.

==============================================================================*/

// .NAME vtkSlicerLinesIntersectionLogic - slicer logic class for volumes manipulation
// .SECTION Description
// This class manages the logic associated with reading, saving,
// and changing propertied of the volumes


#ifndef __vtkSlicerLinesIntersectionLogic_h
#define __vtkSlicerLinesIntersectionLogic_h

// Slicer includes
#include "vtkSlicerModuleLogic.h"
#include "vtkSlicerLinesIntersectionModuleLogicExport.h"

// VNL includes
#include <vnl/vnl_double_3x1.h>
#include <vnl/vnl_double_3x3.h>

/// \ingroup Slicer_QtModules_ExtensionTemplate
class VTK_SLICER_LINESINTERSECTION_MODULE_LOGIC_EXPORT vtkSlicerLinesIntersectionLogic :
  public vtkSlicerModuleLogic
{
  typedef std::vector<vnl_double_3x1> VectorType;

public:
  static vtkSlicerLinesIntersectionLogic* New();
  vtkTypeMacro(vtkSlicerLinesIntersectionLogic, vtkSlicerModuleLogic);
  void PrintSelf(ostream& os, vtkIndent indent);

  bool AddLine(double origin[3], double direction[3]);
  void Reset();

  double* Update() VTK_SIZEHINT(3);
  double* GetResult() VTK_SIZEHINT(3);
  double GetError();

protected:
  vtkSlicerLinesIntersectionLogic();
  virtual ~vtkSlicerLinesIntersectionLogic();

  // perpendicular distance from a point p to a line (a,n)
  double PointToLineDistance(const vnl_double_3x1& point, const vnl_double_3x1& origin, const vnl_double_3x1& direction);

  virtual void SetMRMLSceneInternal(vtkMRMLScene* newScene);
  /// Register MRML Node classes to Scene. Gets called automatically when the MRMLScene is attached to this logic class.
  virtual void RegisterNodes();
  virtual void UpdateFromMRMLScene();

protected:
  VectorType      LineOrigins;
  VectorType      LineDirections;

  vnl_double_3x1  LastResult;
  double          LastError;

private:
  vtkSlicerLinesIntersectionLogic(const vtkSlicerLinesIntersectionLogic&); // Not implemented
  void operator=(const vtkSlicerLinesIntersectionLogic&); // Not implemented
};

#endif
