/*====================================================================
Copyright(c) 2018 Adam Rankin


Permission is hereby granted, free of charge, to any person obtaining a
copy of this software and associated documentation files(the "Software"),
to deal in the Software without restriction, including without limitation
the rights to use, copy, modify, merge, publish, distribute, sublicense,
and / or sell copies of the Software, and to permit persons to whom the
Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included
in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL
THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR
OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
OTHER DEALINGS IN THE SOFTWARE.
====================================================================*/

// Local includes
#include "vtkPointToLineRegistration.h"

// VTK includes
#include <vtkLandmarkTransform.h>
#include <vtkMatrix4x4.h>
#include <vtkObjectFactory.h>
#include <vtkPoints.h>
#include <vtkVector.h>

// VNL includes
#include <vnl/vnl_matrix.h>

// Eigen includes
#include <vtkeigen/eigen/Dense>
#include <vtkeigen/eigen/SVD>

//----------------------------------------------------------------------------

namespace
{
  void Matrix4x4ToVNL(vtkMatrix4x4* matrix, vnl_matrix<double>& R, vnl_matrix<double>& t)
  {
    for (int i = 0; i < 3; ++i)
    {
      for (int j = 0; j < 3; ++j)
      {
        R[i][j] = matrix->GetElement(i, j);
      }
      t[i][0] = matrix->GetElement(i, 3);
    }
  }

  void VNLToMatrix4x4(vnl_matrix<double>& R, vnl_matrix<double>& t, vtkMatrix4x4* matrix)
  {
    for (int i = 0; i < 3; ++i)
    {
      for (int j = 0; j < 3; ++j)
      {
        matrix->SetElement(i, j, R[i][j]);
      }
      matrix->SetElement(i, 3, t[i][0]);
    }
  }

  void EigenToMatrix4x4(vtkeigen::Matrix3d& R, vtkeigen::Matrix3d& S, vtkeigen::Vector3d& t, vtkMatrix4x4* matrix)
  {
    matrix->Identity();

    vtkeigen::Matrix3d RS = R * S;
    for (int i = 0; i < 3; i++)
    {
      for (int j = 0; j < 3; j++)
      {
        matrix->SetElement(i, j, RS(i, j));
      }
      matrix->SetElement(i, 3, t(i));
    }
  }
}

//----------------------------------------------------------------------------

vtkStandardNewMacro(vtkPointToLineRegistration);

//----------------------------------------------------------------------------
vtkPointToLineRegistration::vtkPointToLineRegistration()
: LandmarkRegistrationMode(VTK_LANDMARK_RIGIDBODY)
, Tolerance(1e-4)
, Error(0.0)
{
}

//----------------------------------------------------------------------------
vtkPointToLineRegistration::~vtkPointToLineRegistration()
{
}

//----------------------------------------------------------------------------
void vtkPointToLineRegistration::PrintSelf(ostream& os, vtkIndent indent)
{
  os << indent << "Tolerance: " << this->Tolerance << std::endl;
  os << indent << "Error: " << this->Error << std::endl;
}

//----------------------------------------------------------------------------
void vtkPointToLineRegistration::AddPoint(double x, double y, double z)
{
  vtkVector3d p;
  p.SetX(x);
  p.SetY(y);
  p.SetZ(z);
  this->Points.push_back(p);
}

//----------------------------------------------------------------------------
void vtkPointToLineRegistration::AddLine(double originX, double originY, double originZ, double directionI, double directionJ, double directionK)
{
  vtkVector3d origin;
  origin.SetX(originX);
  origin.SetY(originY);
  origin.SetZ(originZ);
  vtkVector3d dir;
  dir.SetX(directionI);
  dir.SetY(directionJ);
  dir.SetZ(directionK);
  this->Lines.push_back(Line(origin, dir));
}

//----------------------------------------------------------------------------
void vtkPointToLineRegistration::Reset()
{
  this->Points.clear();
  this->Lines.clear();
}

//----------------------------------------------------------------------------
void vtkPointToLineRegistration::SetTolerance(double arg)
{
  this->Tolerance = arg;
}

//----------------------------------------------------------------------------
double vtkPointToLineRegistration::GetTolerance() const
{
  return this->Tolerance;
}

//----------------------------------------------------------------------------
unsigned int vtkPointToLineRegistration::GetCount() const
{
  return this->Points.size();
}

//----------------------------------------------------------------------------
/*!
* Point to line registration
*
* use ICP to solve the following registration problem:
*
* O + a * D = R * X + t
*
* INPUTS: X - copied from this->Points
*         O - copied from this->Lines.first (origin)
*         D - copied from this->Lines.second (direction)
*
* OUTPUTS: 4x4 rotation + translation
*/
vtkMatrix4x4* vtkPointToLineRegistration::Compute()
{
  vtkMatrix4x4* matrix = vtkMatrix4x4::New();
  matrix->Identity();

  if (this->Points.size() != this->Lines.size() || this->Points.size() < 3)
  {
    return matrix;
  }

  // local typedef to clean up code
  typedef std::vector<vtkVector3d>::size_type sizeType;

  // assume column vector
  const sizeType n = this->Points.size();
  double outError = std::numeric_limits<double>::infinity();

  if (this->LandmarkRegistrationMode == VTK_POINTTOLINE_ANISOTROPIC)
  {
    vtkeigen::MatrixXd O(3, n);
    vtkeigen::MatrixXd X(3, n);
    vtkeigen::MatrixXd D(3, n);
    for (sizeType i = 0; i < this->Points.size(); ++i)
    {
      X(0, i) = this->Points[i].GetX();
      X(1, i) = this->Points[i].GetY();
      X(2, i) = this->Points[i].GetZ();

      O(0, i) = this->Lines[i].first.GetX();
      O(1, i) = this->Lines[i].first.GetY();
      O(2, i) = this->Lines[i].first.GetZ();

      D(0, i) = this->Lines[i].second.GetX();
      D(1, i) = this->Lines[i].second.GetY();
      D(2, i) = this->Lines[i].second.GetZ();

    }
    vtkeigen::Matrix3d S = vtkeigen::MatrixXd::Identity(3, 3);
    vtkeigen::Matrix3d R = vtkeigen::MatrixXd::Identity(3, 3);
    vtkeigen::Vector3d t(3, 1, 0.0);

    outError = AnisotropicPoint2Line(X, O, D, Tolerance, R, S, t);
    this->Error = outError;

    EigenToMatrix4x4(R, S, t, matrix);

    return matrix;
  }
  else
  {
    vnl_matrix<double> e(1, n, 1.0);
    vnl_matrix<double> E_old(3, n, 1.0);
    E_old = E_old * 1000.0;
    vnl_matrix<double> E(3, n, 0.0);

    vnl_matrix<double> O(3, n);
    vnl_matrix<double> X(3, n);
    vnl_matrix<double> Y(3, n);
    vnl_matrix<double> D(3, n);
    for (sizeType i = 0; i < this->Points.size(); ++i)
    {
      X[0][i] = this->Points[i].GetX();
      X[1][i] = this->Points[i].GetY();
      X[2][i] = this->Points[i].GetZ();

      O[0][i] = this->Lines[i].first.GetX();
      O[1][i] = this->Lines[i].first.GetY();
      O[2][i] = this->Lines[i].first.GetZ();

      D[0][i] = this->Lines[i].second.GetX();
      D[1][i] = this->Lines[i].second.GetY();
      D[2][i] = this->Lines[i].second.GetZ();

      Y[0][i] = this->Lines[i].first.GetX() + this->Lines[i].second.GetX();
      Y[1][i] = this->Lines[i].first.GetY() + this->Lines[i].second.GetY();
      Y[2][i] = this->Lines[i].first.GetZ() + this->Lines[i].second.GetZ();
    }

    vnl_matrix<double> d(1, n);
    vnl_matrix<double> tempM;
    vnl_matrix<double> R(3, 3);
    R.set_identity();
    vnl_matrix<double> t(3, 1, 0.0);


    vtkNew<vtkLandmarkTransform> landmarkRegistration;
    landmarkRegistration->SetMode(this->LandmarkRegistrationMode);

    vtkNew<vtkPoints> source;
    vtkNew<vtkPoints> target;
    vtkNew<vtkMatrix4x4> result;
    while (outError > this->Tolerance)
    {
      source->Reset();
      target->Reset();
      for (unsigned int i = 0; i < X.columns(); ++i)
      {
        source->InsertNextPoint(X[0][i], X[1][i], X[2][i]);
        target->InsertNextPoint(Y[0][i], Y[1][i], Y[2][i]);
      }
      landmarkRegistration->SetSourceLandmarks(source);
      landmarkRegistration->SetTargetLandmarks(target);
      landmarkRegistration->Update();
      landmarkRegistration->GetMatrix(result);
      Matrix4x4ToVNL(result, R, t);

      tempM = (R * X) + (t * e) - O;
      for (std::vector<vtkVector3d>::size_type i = 0; i < this->Points.size(); ++i)
      {
        d[0][i] = tempM[0][i] * D[0][i] + tempM[1][i] * D[1][i] + tempM[2][i] * D[2][i];
      }

      for (unsigned int i = 0; i < n; i++)
      {
        for (unsigned int j = 0; j < 3; j++)
        {
          Y[j][i] = O[j][i] + d[0][i] * D[j][i];
        }
      }

      E = Y - (R * X) - (t * e);
      outError = (E - E_old).frobenius_norm();
      E_old = E;
    }


    // compute the Euclidean distance between points and lines
    outError = 0.0;
    for (unsigned int i = 0; i < E.columns(); i++)
    {
      outError += std::sqrt(E[0][i] * E[0][i] + E[1][i] * E[1][i] + E[2][i] * E[2][i]);
    }
    this->Error = outError / E.columns();

    VNLToMatrix4x4(R, t, matrix);

    return matrix;
  }
}

//----------------------------------------------------------------------------
double vtkPointToLineRegistration::GetError() const
{
  return this->Error;
}

//----------------------------------------------------------------------------
void vtkPointToLineRegistration::SetLandmarkRegistrationMode(int arg)
{
  this->LandmarkRegistrationMode = arg;
}

//----------------------------------------------------------------------------
void vtkPointToLineRegistration::SetLandmarkRegistrationModeToRigidBody()
{
  this->LandmarkRegistrationMode = VTK_LANDMARK_RIGIDBODY;
}

//----------------------------------------------------------------------------
void vtkPointToLineRegistration::SetLandmarkRegistrationModeToAffine()
{
  this->LandmarkRegistrationMode = VTK_LANDMARK_AFFINE;
}

//----------------------------------------------------------------------------
void vtkPointToLineRegistration::SetLandmarkRegistrationModeToSimilarity()
{
  this->LandmarkRegistrationMode = VTK_LANDMARK_SIMILARITY;
}

//----------------------------------------------------------------------------
void vtkPointToLineRegistration::SetLandmarkRegistrationModeToAnisotropic()
{
  this->LandmarkRegistrationMode = VTK_POINTTOLINE_ANISOTROPIC;
}

//----------------------------------------------------------------------------
void vtkPointToLineRegistration::ClosestPointOnLine(const vtkeigen::Vector3d& x, const vtkeigen::Vector3d& o, const vtkeigen::Vector3d& n, vtkeigen::Vector3d& y)
{
  y = o - (o - x).dot(n) / n.squaredNorm() * n;
}

//----------------------------------------------------------------------------
double vtkPointToLineRegistration::AnisotropicPoint2Line(const vtkeigen::MatrixXd& X, const vtkeigen::MatrixXd& O, const vtkeigen::MatrixXd& D, const double tol, vtkeigen::Matrix3d& R, vtkeigen::Matrix3d& S, vtkeigen::Vector3d& t)
{
  // make sure we are working with 3xn matrices (column vectors in 3D)
  assert(X.rows() == 3);
  assert(O.rows() == 3);
  assert(D.rows() == 3);
  assert(X.cols() == O.cols());
  assert(X.cols() == D.cols());

  int n = X.cols();
  vtkeigen::MatrixXd e = vtkeigen::MatrixXd::Ones(n, 1);
  vtkeigen::MatrixXd II = vtkeigen::MatrixXd::Identity(n, n) - ((1.0 / double(n)) * (e * e.transpose()));
  vtkeigen::Matrix3d d = vtkeigen::Matrix3d::Identity();
  d(2, 2) = -1.0;

  // initialization, but not needed
  R = S = vtkeigen::Matrix3d::Identity();
  t = vtkeigen::Vector3d::Zero();

  vtkeigen::MatrixXd Y(3, n); // initialization
  vtkeigen::Vector3d y;
  for (int i = 0; i < n; i++)
  {
    ClosestPointOnLine(X.col(i), O.col(i), D.col(i), y);
    Y.col(i) = y;
  }

  double err = std::numeric_limits<double>::infinity();
  vtkeigen::MatrixXd E_old_outside = vtkeigen::MatrixXd::Constant(3, n, err), E_outside;
  while (err > tol)
  {
    // AOPA_Major
    vtkeigen::MatrixXd mX = (X * II).colwise().normalized();
    vtkeigen::MatrixXd mY = (Y * II);
    vtkeigen::MatrixXd B = mY * mX.transpose();

    vtkeigen::JacobiSVD<vtkeigen::MatrixXd> svd(B, vtkeigen::ComputeThinU | vtkeigen::ComputeThinV);
    R = svd.matrixU() * svd.matrixV().transpose();
    if (R.determinant() < 0.0)
    {
      R = svd.matrixU() * d * svd.matrixV().transpose();
    }
    double err2 = std::numeric_limits<double>::infinity();
    vtkeigen::MatrixXd E_old = vtkeigen::MatrixXd::Constant(3, n, err), E;

    while (err2 > tol)
    {
      vtkeigen::JacobiSVD<vtkeigen::MatrixXd> svd1(B * ((R.transpose()*B).diagonal()).asDiagonal(), vtkeigen::ComputeThinU | vtkeigen::ComputeThinV);
      R = svd1.matrixU() * svd1.matrixV().transpose();
      if (R.determinant() < 0.0)
      {
        R = svd1.matrixU() * d * svd1.matrixV().transpose();
      }
      E = R * mX - mY;
      err2 = (E - E_old).norm();
      E_old = E;
    }
    B = Y * II * X.transpose();
    S = ((B.transpose() * R).diagonal().array() / (X * II * X.transpose()).diagonal().array()).matrix().asDiagonal();

    // in case of 2D to 3D registration, it may be necessary to fix the scale.
    for (int i = 0; i < S.rows(); i++)
      for (int j = 0; j < S.cols(); j++)
        if (S.array().isNaN()(i, j))
        {
          S(i, j) = 1.0;
        }
    S(2, 2) = (S(0, 0) + S(1, 1)) * 0.5;
    t = (Y - R * S * X).rowwise().mean();
    // end of AOPA_Major

    E = (R * S * X + t * vtkeigen::MatrixXd::Ones(1, n) - O);
    for (int i = 0; i < n; i++)
    {
      Y.col(i) = O.col(i) + D.col(i).dot(E.col(i)) * D.col(i);
    }

    E_outside = Y - R * S * X - t * vtkeigen::MatrixXd::Ones(1, n);
    err = (E_outside - E_old_outside).norm();
    E_old_outside = E_outside;
  }

  return (E_outside.norm());
}
