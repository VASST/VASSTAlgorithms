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

#ifndef __qSlicerPointToLineRegistrationModuleWidget_h
#define __qSlicerPointToLineRegistrationModuleWidget_h

// SlicerQt includes
#include "qSlicerAbstractModuleWidget.h"

#include "qSlicerPointToLineRegistrationModuleExport.h"

class qSlicerPointToLineRegistrationModuleWidgetPrivate;
class vtkMRMLNode;

/// \ingroup Slicer_QtModules_ExtensionTemplate
class Q_SLICER_QTMODULES_POINTTOLINEREGISTRATION_EXPORT qSlicerPointToLineRegistrationModuleWidget :
  public qSlicerAbstractModuleWidget
{
  Q_OBJECT

public:

  typedef qSlicerAbstractModuleWidget Superclass;
  qSlicerPointToLineRegistrationModuleWidget(QWidget* parent = 0);
  virtual ~qSlicerPointToLineRegistrationModuleWidget();

public slots:


protected:
  QScopedPointer<qSlicerPointToLineRegistrationModuleWidgetPrivate> d_ptr;

  virtual void setup();

private:
  Q_DECLARE_PRIVATE(qSlicerPointToLineRegistrationModuleWidget);
  Q_DISABLE_COPY(qSlicerPointToLineRegistrationModuleWidget);
};

#endif
