project(cgogn_core_test
	LANGUAGES CXX
)

find_package(cgogn_geometry REQUIRED)

add_executable(${PROJECT_NAME} main.cpp)

target_sources(${PROJECT_NAME}
	PRIVATE
		"${CMAKE_CURRENT_LIST_DIR}/types/animation_skeleton.cpp"
)

target_link_libraries(cgogn_core_test gtest gmock cgogn::core)

add_test(NAME ${PROJECT_NAME} WORKING_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY} COMMAND $<TARGET_FILE:${PROJECT_NAME}>)

set_target_properties (${PROJECT_NAME} PROPERTIES FOLDER tests)
