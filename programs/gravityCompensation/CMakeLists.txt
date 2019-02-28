option(gravityCompensation "Enable/disable gravityCompensation program" ON)

if(ENABLE_gravityCompensation)

    add_executable(gravityCompensation main.cpp
                                       GravityCompensation.cpp
                                       GravityCompensation.hpp)

    target_link_libraries(gravityCompensation YARP::YARP_OS
                                              YARP::YARP_init
                                              YARP::YARP_dev
                                              ROBOTICSLAB::ColorDebug
                                              KinematicsDynamicsInterfaces)

    install(TARGETS gravityCompensation
            DESTINATION ${CMAKE_INSTALL_BINDIR})

else()

    set(ENABLE_gravityCompensation OFF CACHE BOOL "Enable/disable gravityCompensation program" FORCE)

endif()
