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
#include <vtkSlicerPointToLineRegistrationLogic.h>

// PointToLineRegistration includes
#include "qSlicerPointToLineRegistrationModule.h"
#include "qSlicerPointToLineRegistrationModuleWidget.h"

//-----------------------------------------------------------------------------
#if (QT_VERSION < QT_VERSION_CHECK(5, 0, 0))
  #include <QtPlugin>
  Q_EXPORT_PLUGIN2(qSlicerPointToLineRegistrationModule, qSlicerPointToLineRegistrationModule);
#endif

//-----------------------------------------------------------------------------
/// \ingroup Slicer_QtModules_ExtensionTemplate
class qSlicerPointToLineRegistrationModulePrivate
{
public:
  qSlicerPointToLineRegistrationModulePrivate();
};

//-----------------------------------------------------------------------------
// qSlicerPointToLineRegistrationModulePrivate methods

//-----------------------------------------------------------------------------
qSlicerPointToLineRegistrationModulePrivate::qSlicerPointToLineRegistrationModulePrivate()
{
}

//-----------------------------------------------------------------------------
// qSlicerPointToLineRegistrationModule methods

//-----------------------------------------------------------------------------
qSlicerPointToLineRegistrationModule::qSlicerPointToLineRegistrationModule(QObject* _parent)
  : Superclass(_parent)
  , d_ptr(new qSlicerPointToLineRegistrationModulePrivate)
{
  this->setWidgetRepresentationCreationEnabled(false);
}

//-----------------------------------------------------------------------------
qSlicerPointToLineRegistrationModule::~qSlicerPointToLineRegistrationModule()
{
}

//-----------------------------------------------------------------------------
QString qSlicerPointToLineRegistrationModule::helpText() const
{
  return "This is a hidden loadable module that contains a point to line registration algorithm";
}

//-----------------------------------------------------------------------------
QString qSlicerPointToLineRegistrationModule::acknowledgementText() const
{
  return "The algorithm was written by Elvis Chen (Robarts Research Institute). This extention was created by Leah Groves (Robarts Research Institute), with help from Adam Rankin (Robarts Research Institute)";
}

//-----------------------------------------------------------------------------
QStringList qSlicerPointToLineRegistrationModule::contributors() const
{
  QStringList moduleContributors;
  moduleContributors << QString("Elvis Chen (Robarts Research Intitute)")
                     << QString("Leah Groves (Robarts Research Institute)")
                     << QString("Adam Rankin (Robarts Research Institute)")
                     << QString("Funding through OGS");
  return moduleContributors;
}

//-----------------------------------------------------------------------------
QIcon qSlicerPointToLineRegistrationModule::icon() const
{
  return QIcon(":/Icons/PointToLineRegistration.png");
}

//-----------------------------------------------------------------------------
QStringList qSlicerPointToLineRegistrationModule::categories() const
{
  return QStringList() << "IGT";
}

//-----------------------------------------------------------------------------
QStringList qSlicerPointToLineRegistrationModule::dependencies() const
{
  return QStringList();
}

//-----------------------------------------------------------------------------
void qSlicerPointToLineRegistrationModule::setup()
{
  this->Superclass::setup();
}

//-----------------------------------------------------------------------------
qSlicerAbstractModuleRepresentation* qSlicerPointToLineRegistrationModule
::createWidgetRepresentation()
{
  return new qSlicerPointToLineRegistrationModuleWidget;
}

//-----------------------------------------------------------------------------
vtkMRMLAbstractLogic* qSlicerPointToLineRegistrationModule::createLogic()
{
  return vtkSlicerPointToLineRegistrationLogic::New();
}
