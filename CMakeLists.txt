cmake_minimum_required(VERSION 3.0.0)
project(INS VERSION 0.1.0)

include(CTest)
enable_testing()

find_package(Eigen3 3.1.0 REQUIRED)

include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(${PROJECT_NAME}_test
  src/mechanization_test.cpp
  src/INSMechanization.cpp
)

target_link_libraries(${PROJECT_NAME}_test
  ${EIGEN3_LIBS}
)

add_executable(GINS_test
  src/GINS_test.cpp
  src/INSMechanization.cpp
  src/GINS.cpp
)

target_link_libraries(GINS_test
  ${EIGEN3_LIBS}
)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
