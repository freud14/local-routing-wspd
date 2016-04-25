# Created by the script cgal_create_CMakeLists
# This is the CMake script for compiling a set of CGAL applications.

project( main )


cmake_minimum_required(VERSION 2.8.11)
if(POLICY CMP0043)
  cmake_policy(SET CMP0043 OLD)
endif()

# CGAL and its components
find_package( CGAL QUIET COMPONENTS Qt5  )


if ( NOT CGAL_FOUND )

  message(STATUS "This project requires the CGAL library, and will not be compiled.")
  return()

endif()

# include helper file
include( ${CGAL_USE_FILE} )


# Boost and its components
find_package( Boost REQUIRED )

find_package(Qt5 QUIET COMPONENTS Xml Script OpenGL Svg)


if ( NOT Boost_FOUND )

  message(STATUS "This project requires the Boost library, and will not be compiled.")

  return()

endif()

# include for local directory

# include for local package

# Creating entries for target: main
# ############################
if ( CGAL_FOUND AND CGAL_Qt5_FOUND AND Qt5_FOUND )

qt5_wrap_ui( DT_UI_FILES mainwindow.ui )

# use the Qt MOC preprocessor on classes that derives from QObject
qt5_generate_moc( "mainwindow.h" "${CMAKE_CURRENT_BINARY_DIR}/mainwindow.moc" )

# The executable itself.
add_executable  ( main main.cpp mainwindow.cpp mainwindow.moc util.cpp ${DT_UI_FILES} ${DT_RESOURCE_FILES} )

qt5_use_modules(main Xml Script OpenGL Svg)

add_to_cached_list( CGAL_EXECUTABLE_TARGETS main )

# Link with Qt libraries
target_link_libraries( main ${QT_LIBRARIES} )
# Link with CGAL
target_link_libraries( main ${CGAL_LIBRARIES} ${CGAL_3RD_PARTY_LIBRARIES})

else()

  message(STATUS "NOTICE: This demo requires CGAL and Qt5, and will not be compiled.")

endif()
