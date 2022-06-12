###################################################################################
# Useful macros to work with the ImFusion SDK in CMake
###################################################################################

# provide instructions on how to launch the ImFusion Suite with the built custom plugin
function(imfusion_provide_ide_instructions)
	get_target_property(ImFusionSuiteCommand ImFusionSuite LOCATION)

	message(STATUS "")
	message(STATUS "TODO: To launch the ImFusion SDK with your ${PROJECT_NAME} plugin, set the ")
	message(STATUS "      IMFUSION_PLUGIN_PATH environment variable to your build output directory")

	if(WIN32)
		message(STATUS "      (i.e. '${CMAKE_BINARY_DIR}/bin/(Debug|Release)')")
	else()
		message(STATUS "      (i.e. '${CMAKE_BINARY_DIR}/lib')")
	endif()

	message(STATUS "      and then launch ${ImFusionSuiteCommand}.")
	if(WIN32)
		message(STATUS "INFO: The Visual Studio project created by CMake is already configured correctly.")
		message(STATUS "      ")
	endif()
endfunction()


# build all plugins into common lib dir
function(imfusion_set_common_target_properties)
	set_target_properties(${PROJECT_NAME} PROPERTIES
		ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
		LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
		RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
	)

	if (WIN32)
		imfusion_generation_VS_user_file()
	endif()
endfunction()


# generate user file so the Visual Studio finds the executables automatically
function(imfusion_generation_VS_user_file)
	get_target_property(USERFILE_COMMAND_RELEASE ImFusionSuite LOCATION)
	get_filename_component(USERFILE_WORKING_DIRECTORY_RELEASE ${USERFILE_COMMAND_RELEASE} PATH)
	set(USERFILE_WORKING_DIRECTORY_DEBUG "${USERFILE_WORKING_DIRECTORY_RELEASE}/../SuiteDev")
	set(USERFILE_COMMAND_DEBUG "${USERFILE_WORKING_DIRECTORY_DEBUG}/ImFusionSuite.exe")
	set(USERFILE_ENVIRONMENT_DEBUG "IMFUSION_PLUGIN_PATH=${CMAKE_BINARY_DIR}/bin/Debug")
	set(USERFILE_ENVIRONMENT_RELEASE "IMFUSION_PLUGIN_PATH=${CMAKE_BINARY_DIR}/bin/Release")

	set(USER_FILE "${PROJECT_NAME}.vcxproj.user")
	configure_file("${CMAKE_SOURCE_DIR}/CMake/UserTemplate.user.in" ${CMAKE_CURRENT_BINARY_DIR}/${USER_FILE} @ONLY)

	set_property(DIRECTORY ${CMAKE_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT ${PROJECT_NAME})
endfunction()
