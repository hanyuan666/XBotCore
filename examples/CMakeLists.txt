#
#  Copyright (C) 2016 IIT-ADVR
#  Author: Luca Muratore
#  email: luca.muratore@iit.it
#
#  This program is free software: you can redistribute it and/or modify
#  it under the terms of the GNU Lesser General Public License as published by
#  the Free Software Foundation, either version 3 of the License, or
#  (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
#  GNU Lesser General Public License for more details.
#
#  You should have received a copy of the GNU Lesser General Public License
#  along with this program. If not, see <http://www.gnu.org/licenses/>
#

#minimum cmake required version
cmake_minimum_required(VERSION 3.0)
# generic_tutorial project
project(XCM_examples)

###################
## Configuration ##
###################

find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs sensor_msgs)

#enable C++ 11 : try with two different flags
include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++11" COMPILER_SUPPORTS_CXX11)
check_cxx_compiler_flag("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

# find the required packages

#############
## Include ##
#############
include_directories(include)

###########
## Build ##
###########
add_library(IkExample SHARED src/ik_example.cpp)
add_library(HomingExample SHARED src/homing_example.cpp)
add_library(IdExample SHARED src/id_example.cpp)
add_executable(IpcExampleRT src/example_ipc_rt.cpp)
add_executable(IpcExampleNRT src/example_ipc_nonrt.cpp)
add_executable(IpcExampleIDDPMain src/example_iddp_main.cpp ../src/signal_handler.cpp)
add_executable(IpcExampleIDDPMain1 src/example_iddp_main1.cpp ../src/signal_handler.cpp)
add_executable(IpcExampleIDDPMain2 src/example_iddp_main2.cpp ../src/signal_handler.cpp)
add_executable(IpcExampleXBOTIDDPMain src/example_xbot_iddp_main.cpp ../src/signal_handler.cpp)
add_library(IpcExampleIDDP1 src/example_iddp1.cpp)
add_library(IpcExampleIDDP2 src/example_iddp2.cpp)
add_library(IpcExampleXBOTIDDP src/example_xbot_iddp.cpp)

if ( Xenomai_FOUND )
    set_xeno_flags(HomingExample)
    set_xeno_flags(IpcExampleIDDP1)
    set_xeno_flags(IpcExampleIDDP2)
    set_xeno_flags(IpcExampleXBOTIDDP)
    set_xeno_flags(IpcExampleIDDPMain)
    set_xeno_flags(IpcExampleIDDPMain1)
    set_xeno_flags(IpcExampleIDDPMain2)
    set_xeno_flags(IpcExampleXBOTIDDPMain)
endif()


##########
## Link ##
##########
target_link_libraries(IkExample XCM)
target_link_libraries(IdExample XCM)
target_link_libraries(HomingExample XCM)
target_link_libraries(IpcExampleNRT XCM ${RT_LIBRARY})
target_link_libraries(IpcExampleRT XCM ${RT_LIBRARY})
target_link_libraries(IpcExampleIDDP1 XCM ${RT_LIBRARY})
target_link_libraries(IpcExampleIDDP2 XCM ${RT_LIBRARY})
target_link_libraries(IpcExampleXBOTIDDP XCM ${RT_LIBRARY})
target_link_libraries(IpcExampleIDDPMain IpcExampleIDDP1 IpcExampleIDDP2)
target_link_libraries(IpcExampleIDDPMain1 IpcExampleIDDP1 IpcExampleIDDP2)
target_link_libraries(IpcExampleIDDPMain2 IpcExampleIDDP1 IpcExampleIDDP2)
target_link_libraries(IpcExampleXBOTIDDPMain IpcExampleXBOTIDDP)



#############
## Install ##
#############
ycm_library_install(IdExample 1 0 0)
ycm_library_install(IkExample 1 0 0)
ycm_library_install(HomingExample 1 0 0)

include(AddUninstallTarget)
