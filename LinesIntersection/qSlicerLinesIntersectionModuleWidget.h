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

#ifndef __qSlicerLinesIntersectionModuleWidget_h
#define __qSlicerLinesIntersectionModuleWidget_h

// SlicerQt includes
#include "qSlicerAbstractModuleWidget.h"

#include "qSlicerLinesIntersectionModuleExport.h"

class qSlicerLinesIntersectionModuleWidgetPrivate;
class vtkMRMLNode;

/// \ingroup Slicer_QtModules_ExtensionTemplate
class Q_SLICER_QTMODULES_LINESINTERSECTION_EXPORT qSlicerLinesIntersectionModuleWidget :
  public qSlicerAbstractModuleWidget
{
  Q_OBJECT

public:

  typedef qSlicerAbstractModuleWidget Superclass;
  qSlicerLinesIntersectionModuleWidget(QWidget *parent=0);
  virtual ~qSlicerLinesIntersectionModuleWidget();

public slots:


protected:
  QScopedPointer<qSlicerLinesIntersectionModuleWidgetPrivate> d_ptr;

  virtual void setup();

private:
  Q_DECLARE_PRIVATE(qSlicerLinesIntersectionModuleWidget);
  Q_DISABLE_COPY(qSlicerLinesIntersectionModuleWidget);
};

#endif
