cmake_minimum_required(VERSION 3.5)

project(VASSTAlgorithms)

#-----------------------------------------------------------------------------
# Extension meta-information
set(EXTENSION_HOMEPAGE "https://github.com/VASST/VASSTAlgorithms")
set(EXTENSION_CATEGORY "Registration")
set(EXTENSION_CONTRIBUTORS "Adam Rankin (Robarts Research Institute), Elvis Chen (Robarts Research Institute)")
set(EXTENSION_DESCRIPTION "This extension contains logic modules for registration algorithms.")
set(EXTENSION_ICONURL "https://github.com/VASST/VASSTAlgorithms/raw/master/VASSTAlgorithms.png") 
set(EXTENSION_SCREENSHOTURLS "https://github.com/VASST/VASSTAlgorithms/blob/master/VASSTAlgorithms.png")
set(EXTENSION_DEPENDS "NA") # Specified as a space separated string, a list or 'NA' if any

#-----------------------------------------------------------------------------
# Extension dependencies
find_package(Slicer REQUIRED)
include(${Slicer_USE_FILE})

#-----------------------------------------------------------------------------
# Extension modules
add_subdirectory(PointToLineRegistration)
add_subdirectory(LinesIntersection)
## NEXT_MODULE

#-----------------------------------------------------------------------------
include(${Slicer_EXTENSION_GENERATE_CONFIG})
include(${Slicer_EXTENSION_CPACK})
