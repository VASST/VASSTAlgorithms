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

// PointToLineRegistration Logic includes
#include "vtkSlicerPointToLineRegistrationLogic.h"
#include "vtkPointToLineRegistration.h"

// MRML includes
#include <vtkMRMLScene.h>

// VTK includes
#include <vtkIntArray.h>
#include <vtkNew.h>
#include <vtkObjectFactory.h>

// STD includes
#include <cassert>

//----------------------------------------------------------------------------
vtkStandardNewMacro(vtkSlicerPointToLineRegistrationLogic);

//----------------------------------------------------------------------------
vtkSlicerPointToLineRegistrationLogic::vtkSlicerPointToLineRegistrationLogic()
  : PointToLineRegistration(vtkSmartPointer<vtkPointToLineRegistration>::New())
{
}

//----------------------------------------------------------------------------
vtkSlicerPointToLineRegistrationLogic::~vtkSlicerPointToLineRegistrationLogic()
{
}

//----------------------------------------------------------------------------
void vtkSlicerPointToLineRegistrationLogic::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
}

//----------------------------------------------------------------------------
void vtkSlicerPointToLineRegistrationLogic::AddPointAndLine(double point[3], double lineOrigin[3], double lineDirection[3])
{
  this->PointToLineRegistration->AddPoint(point[0], point[1], point[2]);
  PointToLineRegistration->AddLine(lineOrigin[0], lineOrigin[1], lineOrigin[2], lineDirection[0], lineDirection[1], lineDirection[2]);
}

//----------------------------------------------------------------------------
void vtkSlicerPointToLineRegistrationLogic::Reset()
{
  this->PointToLineRegistration->Reset();
}

//----------------------------------------------------------------------------
vtkMatrix4x4* vtkSlicerPointToLineRegistrationLogic::CalculateRegistration()
{
  return this->PointToLineRegistration->Compute();
}

//----------------------------------------------------------------------------
void vtkSlicerPointToLineRegistrationLogic::SetTolerance(double arg)
{
  this->PointToLineRegistration->SetTolerance(arg);
}

//----------------------------------------------------------------------------
double vtkSlicerPointToLineRegistrationLogic::GetTolerance() const
{
  return this->PointToLineRegistration->GetTolerance();
}

//----------------------------------------------------------------------------
unsigned int vtkSlicerPointToLineRegistrationLogic::GetCount() const
{
  return this->PointToLineRegistration->GetCount();
}

//----------------------------------------------------------------------------
double vtkSlicerPointToLineRegistrationLogic::GetError() const
{
  return this->PointToLineRegistration->GetError();
}

//----------------------------------------------------------------------------
void vtkSlicerPointToLineRegistrationLogic::SetLandmarkRegistrationMode(int arg)
{
  this->PointToLineRegistration->SetLandmarkRegistrationMode(arg);
}

//----------------------------------------------------------------------------
void vtkSlicerPointToLineRegistrationLogic::SetLandmarkRegistrationModeToRigidBody()
{
  this->PointToLineRegistration->SetLandmarkRegistrationModeToRigidBody();
}

//----------------------------------------------------------------------------
void vtkSlicerPointToLineRegistrationLogic::SetLandmarkRegistrationModeToAffine()
{
  this->PointToLineRegistration->SetLandmarkRegistrationModeToAffine();
}

//----------------------------------------------------------------------------
void vtkSlicerPointToLineRegistrationLogic::SetLandmarkRegistrationModeToSimilarity()
{
  this->PointToLineRegistration->SetLandmarkRegistrationModeToSimilarity();
}

//---------------------------------------------------------------------------
void vtkSlicerPointToLineRegistrationLogic::SetMRMLSceneInternal(vtkMRMLScene* newScene)
{
  vtkNew<vtkIntArray> events;
  events->InsertNextValue(vtkMRMLScene::NodeAddedEvent);
  events->InsertNextValue(vtkMRMLScene::NodeRemovedEvent);
  events->InsertNextValue(vtkMRMLScene::EndBatchProcessEvent);
  this->SetAndObserveMRMLSceneEventsInternal(newScene, events.GetPointer());
}

//-----------------------------------------------------------------------------
void vtkSlicerPointToLineRegistrationLogic::RegisterNodes()
{
  assert(this->GetMRMLScene() != 0);
}

//---------------------------------------------------------------------------
void vtkSlicerPointToLineRegistrationLogic::UpdateFromMRMLScene()
{
  assert(this->GetMRMLScene() != 0);
}

//---------------------------------------------------------------------------
void vtkSlicerPointToLineRegistrationLogic
::OnMRMLSceneNodeAdded(vtkMRMLNode* vtkNotUsed(node))
{
}

//---------------------------------------------------------------------------
void vtkSlicerPointToLineRegistrationLogic
::OnMRMLSceneNodeRemoved(vtkMRMLNode* vtkNotUsed(node))
{
}
