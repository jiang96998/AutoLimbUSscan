cmake_minimum_required(VERSION 3.0)
project(Fast_RNRR)
set( CMAKE_CXX_STANDARD 14)

set(FastRNRR_Dir ${CMAKE_SOURCE_DIR}/examples/user_code/vessel)
#--- CMake configuration
set(CMAKE_MODULE_PATH ${FastRNRR_Dir}/cmake)

message(STATUS ${FastRNRR_Dir})

#--- Compiler configuration
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

#--- OpenMP (cannot work in clang)
find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
    message(STATUS "openmp is found")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp")
    include_directories(/home/y/Downloads/OpenMesh-8.1/src)
endif()

#--- Eigen3
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

# enable Intel-MKL (only for Intel compilers)
# NB requires correct specification of BLA_VENDOR, e.g.
#   export BLA_VENDOR=Intel10_64lp_seq
if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Intel")
  find_package(BLAS)
  include_directories(${BLAS_INCLUDE_DIRS})
  set(PROJECT_LIBS ${BLAS_LIBRARIES})
  add_definitions(-DEIGEN_USE_MKL_ALL)
endif()

#--- Nanoflann
find_package(NanoFlann REQUIRED)
include_directories(${NANOFLANN_INCLUDE_DIR})

#--- pcl
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#---k4a
find_package(k4a REQUIRED)
if(k4a_FOUND)
message(STATUS "Kinect SDK found")
endif()


#--- Build example
include_directories("./src/")
file(GLOB SOURCES "./src/*.cpp" "./src/geodesic/*.cpp" "./src/tools/*.cpp" "./src/lib_json/*.*")
file(GLOB HEADERS "./src/*.h" "./src/geodesic/*.h" "./src/tools/*.h" "./src/json/*.h")

link_directories(/home/y/Downloads/OpenMesh-8.1/build/Build/lib)

# 指定libTorch位置
set(Torch_DIR /home/y/libtorch/share/cmake/Torch)
find_package(Torch REQUIRED)
message(STATUS "Torch library status:")
message(STATUS "    version: ${TORCH_VERSION}")
message(STATUS "    libraries: ${TORCH_LIBS}")
message(STATUS "    include path: ${TORCH_INCLUDE_DIRS}")
message(STATUS "    torch lib : ${TORCH_LIBRARIES} ")
include_directories(${TORCH_INCLUDE_DIRS})

find_package(VTK REQUIRED)
include( ${VTK_USE_FILE} )

set(USER_CODE_FILES
     ./src/main.cpp)
#get_filename_component(SOURCE_NAME ${USER_CODE_FILE} NAME_WE)


add_executable(${PROJECT_NAME} ${SOURCES} ${HEADERS})

#--- openmesh libs
target_link_libraries(${PROJECT_NAME}
    OpenMeshCore OpenMeshTools
    openpose 
    k4a::k4a
    ${TORCH_LIBRARIES} 
    ${PCL_LIBRARIES}   
    ${VTK_LIBRARIES}
    ${examples_3rdparty_libraries})


if(APPLE OR UNIX)
    #--- Deploy data folder link
    execute_process(COMMAND ln -f -s ${FastRNRR_Dir}/data WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
    #--- "Dummy" target to have data appear in QtCreator
    add_custom_target(data SOURCES ${FastRNRR_Dir}/data)
else()
    file(COPY ${FastRNRR_Dir}/data DESTINATION ${PROJECT_BINARY_DIR})
endif()