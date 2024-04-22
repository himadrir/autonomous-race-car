find_package(Doxygen)
if(DOXYGEN_FOUND)
	set(DOXYGEN_IN ${CMAKE_CURRENT_SOURCE_DIR}/config/Doxyfile.in)
	set(DOXYGEN_OUT ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)
	configure_file(${DOXYGEN_IN} ${DOXYGEN_OUT} @ONLY)
	add_custom_target(doc-${PROJECT_NAME}
						COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT}
						WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
						COMMENT "Generating documentation with Doxygen"
						VERBATIM)
	set(${PROJECT_NAME}_DOCS_EXISTS 1)
	#message(STATUS "Found Doxygen. Use makefile target 'doc-${PROJECT_NAME}' to build documentation for ${PROJECT_NAME}")
	option(DOXYGEN_DOCS_BUILD ON)
else(DOXYGEN_FOUND)
	message(STATUS "Unable to found Doxygen. The documentation for ${PROJECT_NAME} will not be built")
endif(DOXYGEN_FOUND)

