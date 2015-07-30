cmake_minimum_required(VERSION 2.8)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/../cmake/Modules)

include(AddProject)

project(opensg_csm)

set(OPENSG_MODULES OSGBase OSGSystem OSGWindow OSGWindowGLUT OSGDrawable OSGUtil OSGGroup OSGFileIO OSGImageFileIO)

add_project(opensg_csm
	SOURCES "src/opensg_csm.cpp"
	CONFIGS "config/mono.csm"
	OPENSG_MODULES ${OPENSG_MODULES}
	CSM_MODULE
)

add_project(opensg_trackd
	SOURCES "src/opensg_trackd.cpp"
	CONFIGS "config/mono.csm"
	OPENSG_MODULES ${OPENSG_MODULES}
	CSM_MODULE
	TRACKD_MODULE
)

add_project(opensg_vrpn
	SOURCES "src/opensg_vrpn.cpp"
	CONFIGS "config/mono.csm"
	OPENSG_MODULES ${OPENSG_MODULES}
	CSM_MODULE
	VRPN_MODULE
)
