add_executable(gripper_render
                    main.cpp
                    read_poly.cpp)
target_link_libraries(gripper_render
                    ${urdfdom_LIBRARIES}
                    Eigen3::Eigen
                    ${VTK_LIBRARIES})