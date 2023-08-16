


add_library(${PROJECT_NAME}
    include/${PROJECT_NAME}/auroratracker.h
    include/${PROJECT_NAME}/vtkFrameToTimeConverter.h
    include/${PROJECT_NAME}/vtkNDITracker.h
    include/${PROJECT_NAME}/vtkTracker.h
    include/${PROJECT_NAME}/vtkTrackerBuffer.h
    include/${PROJECT_NAME}/vtkTrackerTool.h

    ${PROJECT_NAME}/auroratracker.cpp
    ${PROJECT_NAME}/vtkFrameToTimeConverter.cxx
    ${PROJECT_NAME}/vtkNDITracker.cxx
    ${PROJECT_NAME}/vtkTracker.cxx
    ${PROJECT_NAME}/vtkTrackerBuffer.cxx
    ${PROJECT_NAME}/vtkTrackerTool.cxx
)
if(math_tools_FOUND)
    target_link_libraries(${PROJECT_NAME}
        PUBLIC
            Eigen3::Eigen
            ${VTK_LIBRARIES}
            ndicapi
            real_time_tools::real_time_tools
            math_tools
    )
    target_compile_definitions(${PROJECT_NAME}
        PUBLIC
            USE_MATH_TOOLS
    )
else()
    target_link_libraries(${PROJECT_NAME}
        PUBLIC
            Eigen3::Eigen
            ${VTK_LIBRARIES}
            ndicapi
            real_time_tools::real_time_tools
    )
endif()

target_compile_options(${PROJECT_NAME}
    PRIVATE
        -Wall
        -Wextra
)


install(TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}Exports
    DESTINATION ${CMAKE_INSTALL_LIBDIR}
)
install(DIRECTORY include/${PROJECT_NAME}
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
install(EXPORT ${PROJECT_NAME}Exports
        NAMESPACE ${PROJECT_NAME}::
        DESTINATION lib/cmake/${PROJECT_NAME}
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

