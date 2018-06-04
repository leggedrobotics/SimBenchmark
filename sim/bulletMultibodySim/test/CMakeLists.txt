add_executable(BtMbMultibodyTest MultiBodyTest.cpp)
target_link_libraries(BtMbMultibodyTest ${BtMbSimLink})

add_executable(BtMbSinglebodyTest SingleBodyTest.cpp)
target_link_libraries(BtMbSinglebodyTest ${BtMbSimLink})
