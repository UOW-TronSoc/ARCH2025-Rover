cmake_minimum_required(VERSION 3.10)
project(ChassisSim)

set(CMAKE_CXX_STANDARD 14)

# Find packages
find_package(Eigen3 REQUIRED)
find_package(raisim CONFIG REQUIRED)
find_package(PythonLibs 3.10 REQUIRED)  # Change Python version as needed


# Include directories
include_directories(
	${EIGEN3_INCLUDE_DIRS}
	${PYTHON_INCLUDE_DIRS}
	"/home/hunter/Documents/github/importedRepos/matplotlib-cpp"
	"/home/hunter/raisim_ws/repos/matplotlib-cpp"
)

# Add executables
add_executable(chassisSim
	chassisSim.cpp
	../modules/localController/getController.cpp
	../modules/utilities/utils.cpp
)

# Link libraries to executables
target_link_libraries(chassisSim PUBLIC 
	raisim::raisim
	${PYTHON_LIBRARIES}
)

if(NOT WIN32)
	target_link_libraries(chassisSim PUBLIC pthread)
else()
	target_link_libraries(chassisSim PUBLIC Ws2_32 Winmm)
endif()

target_include_directories(chassisSim PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

file(COPY "../rsc" DESTINATION ${CMAKE_BINARY_DIR})



# cmake .. -DCMAKE_PREFIX_PATH=/home/hunter/raisim_ws/repos/raisimlib/raisim/linux