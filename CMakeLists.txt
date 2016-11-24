set(execName Track)

cmake_minimum_required(VERSION 2.4)

if (COMMAND cmake_policy)
	cmake_policy(SET CMP0003 OLD)
	cmake_policy(SET CMP0015 OLD)
endif (COMMAND cmake_policy)

set(EXECUTABLE_OUTPUT_PATH ".")

if (WIN32) # Windows
	set(ZED_INCLUDE_DIRS $ENV{ZED_INCLUDE_DIRS})
	if (CMAKE_CL_64) # 64 bits
		set(ZED_LIBRARIES $ENV{ZED_LIBRARIES_64})
	else (CMAKE_CL_64) # 32 bits
		message("32bits compilation is no more available with CUDA7.0")
	endif (CMAKE_CL_64)
	set(ZED_LIBRARY_DIR $ENV{ZED_LIBRARY_DIR})
	set(OPENCV_DIR $ENV{OPENCV_DIR})
	set(Eigen_INCLUDE_DIRS $ENV{EIGEN_DIR})
	find_package(OpenCV 3.1)

else () # Linux
	find_package(ZED 1.0 REQUIRED)
	set(Eigen_INCLUDE_DIRS "/usr/include/eigen3") ##depends if installed by packet or by own build install
	set(Eigen_INCLUDE_DIRS_2 "/usr/local/include/eigen3") ##depends if installed by packet or by own build install
	include_directories(${Eigen_INCLUDE_DIRS_2})
	
	##For Jetson, OpenCV4Tegra is based on OpenCV2.4
	exec_program(uname ARGS -p OUTPUT_VARIABLE CMAKE_SYSTEM_NAME2)
	if (CMAKE_SYSTEM_NAME2 MATCHES "aarch64" OR CMAKE_SYSTEM_NAME2 MATCHES "armv7l")
		find_package(OpenCV 2.4)
	else ()
		find_package(OpenCV 3.1)
	endif ()

endif (WIN32)

find_package(CUDA REQUIRED)

include_directories(${CUDA_INCLUDE_DIRS})
include_directories(${CUDA_INCLUDE_DIRS})
include_directories(${ZED_INCLUDE_DIRS})
include_directories(${Eigen_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

link_directories(${ZED_LIBRARY_DIR})
link_directories(${OpenCV_LIBRARY_DIRS})
link_directories(${CUDA_LIBRARY_DIRS})

set(SRC_FOLDER src)
file(GLOB_RECURSE SRC_FILES "${SRC_FOLDER}/*.cpp")

add_executable(${execName} ${SRC_FILES})
set_property(TARGET ${execName} PROPERTY OUTPUT_NAME "Track")

add_definitions(-std=c++0x)# -m64)

target_link_libraries(${execName}
                      ${ZED_LIBRARIES}
                      ${OpenCV_LIBRARIES}
                      ${CUDA_CUDA_LIBRARY} ${CUDA_CUDART_LIBRARY} ${CUDA_npp_LIBRARY}
                      )

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O3") # Release Perf mode
