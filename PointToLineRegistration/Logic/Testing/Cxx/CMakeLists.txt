set(KIT ${PROJECT_NAME})

set(DATA_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Data)

#-----------------------------------------------------------------------------
set(KIT_TEST_SRCS
  vtkSlicerPointToLineRegistrationLogicTest1.cxx
  )

#-----------------------------------------------------------------------------
slicerMacroConfigureModuleCxxTestDriver(
  NAME ${KIT}
  SOURCES ${KIT_TEST_SRCS}
  WITH_VTK_DEBUG_LEAKS_CHECK
  WITH_VTK_ERROR_OUTPUT_CHECK
  )

#-----------------------------------------------------------------------------
simple_test( vtkSlicerPointToLineRegistrationLogicTest1 ${DATA_DIR}/point_to_line_sample.txt)
