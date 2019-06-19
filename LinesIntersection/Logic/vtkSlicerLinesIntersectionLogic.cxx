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

// LinesIntersection Logic includes
#include "vtkSlicerLinesIntersectionLogic.h"

// MRML includes
#include <vtkMRMLScene.h>

// VTK includes
#include <vtkIntArray.h>
#include <vtkNew.h>
#include <vtkMathUtilities.h>
#include <vtkMatrix3x3.h>
#include <vtkObjectFactory.h>
#include <vtkVector.h>

// VNL includes
#include <vnl/vnl_inverse.h>
#include <vnl/vnl_double_3x3.h>

// STL includes
#include <limits>

// OS includes
#include <math.h>

//----------------------------------------------------------------------------
vtkStandardNewMacro(vtkSlicerLinesIntersectionLogic);

//----------------------------------------------------------------------------
vtkSlicerLinesIntersectionLogic::vtkSlicerLinesIntersectionLogic()
{
}

//----------------------------------------------------------------------------
vtkSlicerLinesIntersectionLogic::~vtkSlicerLinesIntersectionLogic()
{
}

//----------------------------------------------------------------------------
double vtkSlicerLinesIntersectionLogic::PointToLineDistance(const vnl_double_3x1& point, const vnl_double_3x1& origin, const vnl_double_3x1& direction)
{
  /*
  based on the following doc by Johannes Traa (UIUC 2013)

  Least-Squares Intersection of Lines
  http://cal.cs.illinois.edu/~johannes/research/LS_line_intersect.pdf
  */

  vnl_double_3x3 I;
  I.set_identity();

  // equation 8
  vnl_matrix<double> temp = (origin - point).transpose() * (I - direction * direction.transpose()) * (origin - point); // for efficiency, assuming n is normalized already

  return (sqrt(temp[0][0]));
}

//----------------------------------------------------------------------------
bool vtkSlicerLinesIntersectionLogic::AddLine(double origin[3], double direction[3])
{
  if (origin == nullptr || direction == nullptr)
  {
    return false;
  }

  LineOrigins.push_back(vnl_matrix<double>(3, 1, 3, origin));
  LineDirections.push_back(vnl_matrix<double>(3, 1, 3, direction));

  return true;
}

//----------------------------------------------------------------------------
void vtkSlicerLinesIntersectionLogic::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
}

//----------------------------------------------------------------------------
void vtkSlicerLinesIntersectionLogic::Reset()
{
  LineOrigins.clear();
  LineDirections.clear();
}

//----------------------------------------------------------------------------
unsigned int vtkSlicerLinesIntersectionLogic::Count()
{
  return this->LineOrigins.size();
}

//----------------------------------------------------------------------------
double* vtkSlicerLinesIntersectionLogic::Update()
{
  // compute the common line intersection among n lines

  // INPUTS: a - 3xn line origin
  //         N - 3xn line direction, normalized
  // OUTPUT: p - 3x1 point
  //         err - residual error

  if (this->LineDirections.size() < 3)
  {
    vtkErrorMacro("Insufficient lines to calculate intersection. Must have at least 3.");
    return nullptr;
  }

  // Check for parallel lines
  for (VectorType::const_iterator first = this->LineDirections.begin(); first != this->LineDirections.end(); ++first)
  {
    for (VectorType::const_iterator next = std::next(first); next != this->LineDirections.end(); ++next)
    {
      // Compare directions
      if (vtkMathUtilities::FuzzyCompare(*first[0][0], *next[0][0]) &&
          vtkMathUtilities::FuzzyCompare(*first[1][0], *next[1][0]) &&
          vtkMathUtilities::FuzzyCompare(*first[2][0], *next[2][0]))
      {
        vtkErrorMacro("Parallel lines found. Algorithm cannot solve for parallel lines.");
        return nullptr;
      }
    }
  }

  /* Based on the following doc by Johannes Traa (UIUC 2013)

    Least-Squares Intersection of Lines
    http://cal.cs.illinois.edu/~johannes/research/LS_line_intersect.pdf
  */

  // Construct R and q based on Equation 14
  vnl_double_3x3 I;
  I.set_identity();
  vnl_double_3x3 R(0.0);
  vnl_double_3x1 q(0.0);
  for (VectorType::size_type i = 0; i < LineOrigins.size(); i++)
  {
    R += I - this->LineDirections[i] * this->LineDirections[i].transpose();
    q += (I - this->LineDirections[i] * this->LineDirections[i].transpose()) * this->LineOrigins[i];
  }

  // Equation 15
  this->LastResult = vnl_inverse<double>(R) * q;

  this->LastError = 0.0;
  for (VectorType::size_type i = 0; i < this->LineOrigins.size(); i++)
  {
    this->LastError += PointToLineDistance(this->LastResult, this->LineOrigins[i], this->LineDirections[i]);
  }
  this->LastError /= (double)this->LineOrigins.size();

  return this->LastResult.data_block();
}

//----------------------------------------------------------------------------
double* vtkSlicerLinesIntersectionLogic::GetResult()
{
  return this->LastResult.data_block();
}

//----------------------------------------------------------------------------
double vtkSlicerLinesIntersectionLogic::GetError()
{
  return this->LastError;
}

//---------------------------------------------------------------------------
void vtkSlicerLinesIntersectionLogic::SetMRMLSceneInternal(vtkMRMLScene* newScene)
{
  this->SetAndObserveMRMLScene(newScene);
}

//-----------------------------------------------------------------------------
void vtkSlicerLinesIntersectionLogic::RegisterNodes()
{
  assert(this->GetMRMLScene() != 0);
}

//---------------------------------------------------------------------------
void vtkSlicerLinesIntersectionLogic::UpdateFromMRMLScene()
{
  assert(this->GetMRMLScene() != 0);
}
