


add_library(${PROJECT_NAME}
    include/${PROJECT_NAME}/auroratracker.h
    include/${PROJECT_NAME}/vtkFrameToTimeConverter.h
    include/${PROJECT_NAME}/vtkNDITracker.h
    include/${PROJECT_NAME}/vtkTracker.h
    include/${PROJECT_NAME}/vtkTrackerBuffer.h
    include/${PROJECT_NAME}/vtkTrackerTool.h
    include/${PROJECT_NAME}/interface/aurora_interface.hpp

    ${PROJECT_NAME}/auroratracker.cpp
    ${PROJECT_NAME}/vtkFrameToTimeConverter.cxx
    ${PROJECT_NAME}/vtkNDITracker.cxx
    ${PROJECT_NAME}/vtkTracker.cxx
    ${PROJECT_NAME}/vtkTrackerBuffer.cxx
    ${PROJECT_NAME}/vtkTrackerTool.cxx
    interface/aurora_interface.cpp
)

target_link_libraries(${PROJECT_NAME}
    PUBLIC
        Eigen3::Eigen
        ${VTK_LIBRARIES}
        ndicapi
        real_time_tools::real_time_tools
)
#target_include_directories(${PROJECT_NAME}
#  PUBLIC
#    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
#    $<INSTALL_INTERFACE:include>
#)

target_compile_options(${PROJECT_NAME}
    PRIVATE
        -Wall
        -Wextra
)


#install(TARGETS ${PROJECT_NAME}
#    EXPORT ${PROJECT_NAME}Exports
#    DESTINATION ${CMAKE_INSTALL_LIBDIR}
#)
#install(DIRECTORY include/${PROJECT_NAME}
#    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
#)
#install(EXPORT ${PROJECT_NAME}Exports
#        NAMESPACE ${PROJECT_NAME}::
#        DESTINATION lib/cmake/${PROJECT_NAME}
#)
install(TARGETS ${PROJECT_NAME}
    DESTINATION ${CMAKE_INSTALL_LIBDIR}
)
install(DIRECTORY include/${PROJECT_NAME}
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)






find_package(yaml-cpp REQUIRED)
include_directories(include)

add_executable(main main.cpp)
target_link_libraries(main
    PUBLIC
        ${PROJECT_NAME}
        yaml-cpp
        math_tools
)
target_compile_definitions(main
    PUBLIC
        USE_MATH_TOOLS
)

