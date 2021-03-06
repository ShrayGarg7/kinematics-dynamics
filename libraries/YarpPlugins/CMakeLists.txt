# Copyright: (C) 2013 Universidad Carlos III de Madrid
# Authors: Juan G. Victores

# Create device interface target.
add_library(KinematicsDynamicsInterfaces INTERFACE)

# Configure usage requirements.
target_include_directories(KinematicsDynamicsInterfaces INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
                                                                  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

# Install interface headers.
install(FILES ICartesianControl.h
              ICartesianSolver.h
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

# Register export set.
install(TARGETS KinematicsDynamicsInterfaces
        EXPORT ROBOTICSLAB_KINEMATICS_DYNAMICS)

# Create devices.
add_subdirectory(AmorCartesianControl)
add_subdirectory(AsibotSolver)
add_subdirectory(BasicCartesianControl)
add_subdirectory(CartesianControlClient)
add_subdirectory(CartesianControlServer)
add_subdirectory(KdlSolver)
