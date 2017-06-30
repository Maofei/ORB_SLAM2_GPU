cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

IF(NOT ROS_BUILD_TYPE)
  SET(ROS_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${ROS_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 -march=native")

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/../../../cmake_modules)

#find_package(OpenCV 3.1.0 REQUIRED)
find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)

include_directories(
/opt/ros/kinetic/include/opencv-3.2.0-dev
/usr/local/cuda-8.0/targets/aarch64-linux/include/
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/../../../
${PROJECT_SOURCE_DIR}/../../../include
${Pangolin_INCLUDE_DIRS}
)

set(LIBS 
/opt/ros/kinetic/lib/libopencv_aruco3.so.3.2.0
/opt/ros/kinetic/lib/libopencv_bgsegm3.so.3.2.0
/opt/ros/kinetic/lib/libopencv_bioinspired3.so.3.2.0
/opt/ros/kinetic/lib/libopencv_calib3d.so.3.2.0
/opt/ros/kinetic/lib/libopencv_calib3d3.so.3.2.0
/opt/ros/kinetic/lib/libopencv_ccalib3.so.3.2.0
/opt/ros/kinetic/lib/libopencv_core.so.3.2.0
/opt/ros/kinetic/lib/libopencv_cudaarithm.so.3.2.0
/opt/ros/kinetic/lib/libopencv_cudabgsegm.so.3.2.0
/opt/ros/kinetic/lib/libopencv_cudacodec.so.3.2.0
/opt/ros/kinetic/lib/libopencv_cudafeatures2d.so.3.2.0
/opt/ros/kinetic/lib/libopencv_cudafilters.so.3.2.0
/opt/ros/kinetic/lib/libopencv_cudaimgproc.so.3.2.0
/opt/ros/kinetic/lib/libopencv_cudalegacy.so.3.2.0
/opt/ros/kinetic/lib/libopencv_cudaobjdetect.so.3.2.0
/opt/ros/kinetic/lib/libopencv_cudaoptflow.so.3.2.0
/opt/ros/kinetic/lib/libopencv_cudastereo.so.3.2.0
/opt/ros/kinetic/lib/libopencv_cudawarping.so.3.2.0
/opt/ros/kinetic/lib/libopencv_cudev.so.3.2.0
/opt/ros/kinetic/lib/libopencv_cvv3.so.3.2.0
/opt/ros/kinetic/lib/libopencv_datasets3.so.3.2.0
/opt/ros/kinetic/lib/libopencv_dpm3.so.3.2.0
/opt/ros/kinetic/lib/libopencv_face3.so.3.2.0
/opt/ros/kinetic/lib/libopencv_features2d.so.3.2.0
/opt/ros/kinetic/lib/libopencv_features2d3.so.3.2.0
/opt/ros/kinetic/lib/libopencv_flann.so.3.2.0
/opt/ros/kinetic/lib/libopencv_flann3.so.3.2.0
/opt/ros/kinetic/lib/libopencv_fuzzy3.so.3.2.0
/opt/ros/kinetic/lib/libopencv_hdf3.so.3.2.0
/opt/ros/kinetic/lib/libopencv_highgui.so.3.2.0
/opt/ros/kinetic/lib/libopencv_highgui3.so.3.2.0
/opt/ros/kinetic/lib/libopencv_imgcodecs.so.3.2.0
/opt/ros/kinetic/lib/libopencv_imgcodecs3.so.3.2.0
/opt/ros/kinetic/lib/libopencv_imgproc.so.3.2.0
/opt/ros/kinetic/lib/libopencv_imgproc3.so.3.2.0
/opt/ros/kinetic/lib/libopencv_line_descriptor3.so.3.2.0
/opt/ros/kinetic/lib/libopencv_ml.so.3.2.0
/opt/ros/kinetic/lib/libopencv_ml3.so.3.2.0
/opt/ros/kinetic/lib/libopencv_objdetect.so.3.2.0
/opt/ros/kinetic/lib/libopencv_objdetect3.so.3.2.0
/opt/ros/kinetic/lib/libopencv_optflow3.so.3.2.0
/opt/ros/kinetic/lib/libopencv_phase_unwrapping3.so.3.2.0
/opt/ros/kinetic/lib/libopencv_photo.so.3.2.0
/opt/ros/kinetic/lib/libopencv_photo3.so.3.2.0
/opt/ros/kinetic/lib/libopencv_plot3.so.3.2.0
/opt/ros/kinetic/lib/libopencv_reg3.so.3.2.0
/opt/ros/kinetic/lib/libopencv_rgbd3.so.3.2.0
/opt/ros/kinetic/lib/libopencv_saliency3.so.3.2.0
/opt/ros/kinetic/lib/libopencv_shape.so.3.2.0
/opt/ros/kinetic/lib/libopencv_shape3.so.3.2.0
/opt/ros/kinetic/lib/libopencv_stereo3.so.3.2.0
/opt/ros/kinetic/lib/libopencv_stitching.so.3.2.0
/opt/ros/kinetic/lib/libopencv_stitching3.so.3.2.0
/opt/ros/kinetic/lib/libopencv_structured_light3.so.3.2.0
/opt/ros/kinetic/lib/libopencv_superres.so.3.2.0
/opt/ros/kinetic/lib/libopencv_superres3.so.3.2.0
/opt/ros/kinetic/lib/libopencv_surface_matching3.so.3.2.0
/opt/ros/kinetic/lib/libopencv_text3.so.3.2.0
/opt/ros/kinetic/lib/libopencv_video.so.3.2.0
/opt/ros/kinetic/lib/libopencv_video3.so.3.2.0
/opt/ros/kinetic/lib/libopencv_videoio.so.3.2.0
/opt/ros/kinetic/lib/libopencv_videostab.so.3.2.0
/opt/ros/kinetic/lib/libopencv_videostab3.so.3.2.0
/opt/ros/kinetic/lib/libopencv_viz.so.3.2.0
/opt/ros/kinetic/lib/libopencv_viz3.so.3.2.0
/opt/ros/kinetic/lib/libopencv_xfeatures2d3.so.3.2.0
/opt/ros/kinetic/lib/libopencv_ximgproc3.so.3.2.0
/opt/ros/kinetic/lib/libopencv_xobjdetect3.so.3.2.0
/opt/ros/kinetic/lib/libopencv_xphoto3.so.3.2.0
    #${OpenCV3_LIBS} 
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
${PROJECT_SOURCE_DIR}/../../../Thirdparty/DBoW2/lib/libDBoW2.so
${PROJECT_SOURCE_DIR}/../../../Thirdparty/g2o/lib/libg2o.so
${PROJECT_SOURCE_DIR}/../../../lib/libORB_SLAM2.so
)

# Node for monocular camera
rosbuild_add_executable(Mono
src/ros_mono.cc
)

target_link_libraries(Mono
${LIBS}
)

# Node for stereo camera
rosbuild_add_executable(Stereo
src/ros_stereo.cc
)

target_link_libraries(Stereo
${LIBS}
)

# Node for RGB-D camera
rosbuild_add_executable(RGBD
src/ros_rgbd.cc
)

target_link_libraries(RGBD
${LIBS}
)

########## 

# Node for usb camera
rosbuild_add_executable(usb_mono
src/ros_mono_usb.cc
)

target_link_libraries(usb_mono
${LIBS}
)

# Node for zed stereo camera
rosbuild_add_executable(zed_stereo
src/ros_stereo_zed.cc
)

target_link_libraries(zed_stereo
${LIBS}
)


# Node for tx1
rosbuild_add_executable(tx1
src/tx1.cpp
)

target_link_libraries(tx1
${LIBS}
)
