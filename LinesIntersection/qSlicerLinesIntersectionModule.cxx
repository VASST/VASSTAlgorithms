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
#include <vtkSlicerLinesIntersectionLogic.h>

// LinesIntersection includes
#include "qSlicerLinesIntersectionModule.h"
#include "qSlicerLinesIntersectionModuleWidget.h"

//-----------------------------------------------------------------------------
#if (QT_VERSION < QT_VERSION_CHECK(5, 0, 0))
#include <QtPlugin>
Q_EXPORT_PLUGIN2(qSlicerLinesIntersectionModule, qSlicerLinesIntersectionModule);
#endif

//-----------------------------------------------------------------------------
/// \ingroup Slicer_QtModules_ExtensionTemplate
class qSlicerLinesIntersectionModulePrivate
{
public:
  qSlicerLinesIntersectionModulePrivate();
};

//-----------------------------------------------------------------------------
// qSlicerLinesIntersectionModulePrivate methods

//-----------------------------------------------------------------------------
qSlicerLinesIntersectionModulePrivate::qSlicerLinesIntersectionModulePrivate()
{
}

//-----------------------------------------------------------------------------
// qSlicerLinesIntersectionModule methods

//-----------------------------------------------------------------------------
qSlicerLinesIntersectionModule::qSlicerLinesIntersectionModule(QObject* _parent)
  : Superclass(_parent)
  , d_ptr(new qSlicerLinesIntersectionModulePrivate)
{
  this->setWidgetRepresentationCreationEnabled(false);
}

//-----------------------------------------------------------------------------
qSlicerLinesIntersectionModule::~qSlicerLinesIntersectionModule()
{
}

//-----------------------------------------------------------------------------
QString qSlicerLinesIntersectionModule::helpText() const
{
  return "This is a hidden loadable module that contains a lines intersection algorithm";
}

//-----------------------------------------------------------------------------
QString qSlicerLinesIntersectionModule::acknowledgementText() const
{
  return "The algorithm was implemented by Elvis Chen (Robarts Research Institute). This extension was created by Adam Rankin (Robarts Research Institute)";
}

//-----------------------------------------------------------------------------
QStringList qSlicerLinesIntersectionModule::contributors() const
{
  QStringList moduleContributors;
  moduleContributors << QString("Adam Rankin (Robarts Research Institute)")
                     << QString("Elvis Chen (Robarts Research Institute)");
  return moduleContributors;
}

//-----------------------------------------------------------------------------
QIcon qSlicerLinesIntersectionModule::icon() const
{
  return QIcon(":/Icons/LinesIntersection.png");
}

//-----------------------------------------------------------------------------
QStringList qSlicerLinesIntersectionModule::categories() const
{
  return QStringList() << "Geometry";
}

//-----------------------------------------------------------------------------
QStringList qSlicerLinesIntersectionModule::dependencies() const
{
  return QStringList();
}

//-----------------------------------------------------------------------------
void qSlicerLinesIntersectionModule::setup()
{
  this->Superclass::setup();
}

//-----------------------------------------------------------------------------
qSlicerAbstractModuleRepresentation* qSlicerLinesIntersectionModule
::createWidgetRepresentation()
{
  return new qSlicerLinesIntersectionModuleWidget;
}

//-----------------------------------------------------------------------------
vtkMRMLAbstractLogic* qSlicerLinesIntersectionModule::createLogic()
{
  return vtkSlicerLinesIntersectionLogic::New();
}
