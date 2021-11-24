option(OMPL_BUILD_DEMOS "Build OMPL final project" ON)
add_feature_info(OMPL_BUILD_FINALPROJ "${OMPL_BUILD_FINALPROJ}" "Whether to build the OMPL final project.")

if (OMPL_BUILD_FINALPROJ)
    file(TO_NATIVE_PATH "${CMAKE_SOURCE_DIR}/tests/resources" TEST_RESOURCES_DIR)
    string(REPLACE "\\" "\\\\" TEST_RESOURCES_DIR "${TEST_RESOURCES_DIR}")
    add_definitions(-DTEST_RESOURCES_DIR=\"${TEST_RESOURCES_DIR}\")

    macro(add_ompl_finalproj demo_name)
        add_executable(${ARGV})
        target_link_libraries(${demo_name}
            ompl
            ${Boost_FILESYSTEM_LIBRARY}
            ${Boost_SYSTEM_LIBRARY}
            ${Boost_PROGRAM_OPTIONS_LIBRARY})
    endmacro(add_ompl_finalproj)

    ##### Add files compile here ######
    add_ompl_finalproj(demo_TestingGeometric3D TestingGeometric3D.cpp)
    ###################################

endif(OMPL_BUILD_FINALPROJ)

file(GLOB OMPL_FINALPROJ_PY_FILES "*.py")
install_python(PROGRAMS ${OMPL_DEMO_PY_FILES}
    DESTINATION "${OMPL_FINALPROJ_INSTALL_DIR}"
    COMPONENT ompl)
file(GLOB OMPL_FINALPROJ_CXX_FILES "*.cpp")
install(FILES ${OMPL_FINALPROJ_CXX_FILES}
    DESTINATION "${OMPL_FINALPROJ_INSTALL_DIR}"
    COMPONENT ompl)
install(DIRECTORY Koules
    DESTINATION "${OMPL_FINALPROJ_INSTALL_DIR}"
    COMPONENT ompl)
install(DIRECTORY VFRRT
    DESTINATION "${OMPL_FINALPROJ_INSTALL_DIR}"
    COMPONENT ompl)
