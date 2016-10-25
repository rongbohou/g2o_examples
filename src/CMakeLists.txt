# CMake 工程，读者应该熟悉了，我就不一一注释了
cmake_minimum_required( VERSION 2.8 )

set( CMAKE_BUILD_TYPE Release )
set( CMAKE_CXX_FLAGS "-std=c++11 -Wall -O2 -march=native" )

list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )
SET( G2O_ROOT /usr/local/include/g2o )

find_package( G2O REQUIRED )
find_package( OpenCV REQUIRED )
find_package( Eigen3 REQUIRED )
find_package( Cholmod )
#FIND_PACKAGE(GLFW3)
#FIND_PACKAGE(OpenGL)
FIND_PACKAGE( CSparse REQUIRED)
# 增加PCL库的依赖
FIND_PACKAGE( PCL REQUIRED COMPONENTS common io visualization filters)

include_directories( ${EIGEN3_INCLUDE_DIR} 
                     ${CHOLMOD_INCLUDE_DIR}
                     ${CSPARSE_INCLUDE_DIR} 
                     ${G2O_INCLUDE_DIR}
                    ${PCL_INCLUDE_DIRS} 
                     )
SET(G2O_LIBS g2o_cli g2o_ext_freeglut_minimal g2o_simulator g2o_solver_slam2d_linear g2o_types_icp g2o_types_slam2d g2o_core g2o_interface g2o_solver_csparse g2o_solver_structure_only g2o_types_sba g2o_types_slam3d g2o_csparse_extension g2o_opengl_helper g2o_solver_dense g2o_stuff g2o_types_sclam2d g2o_parser g2o_solver_pcg g2o_types_data g2o_types_sim3 cxsparse )

add_executable( ba_example ba.cpp )
target_link_libraries( ba_example 
    ${OpenCV_LIBS}
    g2o_core g2o_types_slam3d g2o_solver_csparse g2o_stuff g2o_csparse_extension g2o_types_sba
    ${CHOLMOD_LIBRARIES}
    )

add_executable( simple_optimize simple_optimize.cpp )
target_link_libraries( simple_optimize 
   ${OpenCV_LIBS}
#    g2o_core g2o_types_slam3d g2o_solver_csparse g2o_stuff g2o_csparse_extension g2o_types_sba
    ${G2O_LIBS}
    ${CHOLMOD_LIBRARIES}
    ${CSPARSE_LIBRARY})

#ADD_EXECUTABLE(sba_demo sba_demo.cpp)
#TARGET_LINK_LIBRARIES(sba_demo 
#   ${CHOLMOD_LIBRARIES}
#   ${G2O_LIBS}
#    g2o_solver_cholmod g2o_core g2o_types_icp g2o_types_sba g2o_solver_csparse
#c    )

ADD_EXECUTABLE( gicp_demo gicp_demo.cpp)
TARGET_LINK_LIBRARIES( gicp_demo 
          ${CHOLMOD_LIBRARIES}
          ${CSPARSE_LIBRARY}
          ${OpenCV_LIBS}
          g2o_core g2o_types_slam3d g2o_solver_csparse g2o_stuff g2o_csparse_extension g2o_types_sba g2o_types_icp g2o_opengl_helper
#          ${G2O_LIBS}
          )

ADD_EXECUTABLE( icp_example icp.cpp)
TARGET_LINK_LIBRARIES( icp_example
         ${CHOLMOD_LIBRARIES}
         ${CSPARSE_LIBRARY}
          ${OpenCV_LIBS}
         ${PCL_LIBRARIES}
         ${G2O_LIBS}
#          g2o_core g2o_types_slam3d g2o_solver_csparse g2o_stuff g2o_csparse_extension
         )

#ADD_EXECUTABLE(gicp_sba_demo gicp_sba_demo.cpp)
#TARGET_LINK_LIBRARIES(gicp_sba_demo
#     ${G2O_LIBS}
#    g2o_core g2o_types_sba g2o_types_slam3d g2o_types_icp g2o_solver_csparse g2o_stuff g2o_csparse_extension
#)
