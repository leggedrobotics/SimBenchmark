add_executable(yamlTest testYamlCpp.cpp)
target_link_libraries(yamlTest ${YAML_CPP_LIBRARIES})

add_executable(EigenTest EigenTest.cpp)
target_link_libraries(EigenTest ${RAI_COMMON_LINK})