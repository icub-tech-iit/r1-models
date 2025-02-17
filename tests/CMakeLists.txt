# Tests for generated models
# We use iDynTree & YARP for the tests
find_package(iDynTree REQUIRED)
find_package(YARP     REQUIRED)

add_executable(r1-model-test r1-model-test.cpp)
target_link_libraries(r1-model-test ${iDynTree_LIBRARIES} YARP::YARP_os)

macro(add_r1_model_test yarpRobotName)
    if (COMPILE_R1Mk3Models)
        add_test(NAME ${yarpRobotName}ConsistencyCheck
             COMMAND r1-model-test --model ${CMAKE_BINARY_DIR}/${BUILD_PREFIX}/robots/${yarpRobotName}/model.urdf)
    endif()
endmacro()