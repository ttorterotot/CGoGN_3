project(cgogn_geometry_test
	LANGUAGES CXX
)

find_package(cgogn_geometry REQUIRED)

add_executable(${PROJECT_NAME} main.cpp)

target_sources(${PROJECT_NAME}
	PRIVATE
		"${CMAKE_CURRENT_LIST_DIR}/types/dual_quaternion_test.cpp"
		"${CMAKE_CURRENT_LIST_DIR}/types/rigid_transformation_test.cpp"
		"${CMAKE_CURRENT_LIST_DIR}/types/keyframed_animation_test.cpp"
)

target_link_libraries(cgogn_geometry_test gtest gmock cgogn::geometry)

add_test(NAME ${PROJECT_NAME} WORKING_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY} COMMAND $<TARGET_FILE:${PROJECT_NAME}>)

set_target_properties (${PROJECT_NAME} PROPERTIES FOLDER tests)