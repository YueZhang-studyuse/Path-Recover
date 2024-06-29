project(pr)

cmake_minimum_required (VERSION 3.16)
set (CMAKE_CXX_STANDARD 14)

IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE "RELEASE")
ENDIF()

# include_directories("inc" "inc/CBS" "inc/PIBT" "inc/lacam")
# file(GLOB SOURCES "src/*.cpp" "src/CBS/*.cpp" "src/PIBT/*.cpp" "src/lacam/*.cpp")
include_directories("inc")
file(GLOB SOURCES "src/*.cpp")
add_executable(pr ${SOURCES})

# Find Boost
find_package(Boost REQUIRED COMPONENTS program_options system filesystem)

# Find Eigen3 for PIBT
find_package (Eigen3 3.3 REQUIRED NO_MODULE)


include_directories( ${Boost_INCLUDE_DIRS} )
target_link_libraries(pr ${Boost_LIBRARIES} Eigen3::Eigen)
