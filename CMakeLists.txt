INCLUDE(../../cmakemodules/AssureCMakeRootFile.cmake) # Avoid user mistake in CMake source directory

#-----------------------------------------------------------------
# CMake file for the MRPT application:  holonomic-navigator-demo
#
#  Run with "cmake ." at the root directory
#-----------------------------------------------------------------

PROJECT(holonomic_navigator_demo)

IF(wxWidgets_FOUND)

	# --------------------------------------------- 
	# TARGET:
	# ---------------------------------------------
	SET(SRCS
	holonomic_navigator_demoApp.cpp holonomic_navigator_demoApp.h
	holonomic_navigator_demoMain.cpp holonomic_navigator_demoMain.h
	../ReactiveNavigationDemo/DEFAULT_GRIDMAP_DATA.h
	../wx-common/CMyRedirector.h
	CAboutBox.cpp  CAboutBox.h
	MyGLCanvas.cpp MyGLCanvas.h
	EscapeGap.cpp EscapeGap.h
	PathPlanner.cpp PathPlanner.h
	MapGenerator.cpp MapGenerator.h
	/usr/share/sbpl/src/include/sbpl/headers.h
        /usr/share/sbpl/src/discrete_space_information/environment_nav2D.cpp
        /usr/share/sbpl/src/discrete_space_information/environment_navxythetalat.cpp
        /usr/share/sbpl/src/discrete_space_information/environment_navxythetamlevlat.cpp
        /usr/share/sbpl/src/discrete_space_information/environment_nav2Duu.cpp
        /usr/share/sbpl/src/discrete_space_information/environment_XXX.cpp
        /usr/share/sbpl/src/discrete_space_information/environment_robarm.cpp
        /usr/share/sbpl/src/heuristics/embedded_heuristic.cpp
        /usr/share/sbpl/src/planners/adplanner.cpp
        /usr/share/sbpl/src/planners/ANAplanner.cpp
        /usr/share/sbpl/src/planners/araplanner.cpp
        /usr/share/sbpl/src/planners/lazyARA.cpp
        /usr/share/sbpl/src/planners/mhaplanner.cpp
        /usr/share/sbpl/src/planners/ppcpplanner.cpp
        /usr/share/sbpl/src/planners/rstarplanner.cpp
        /usr/share/sbpl/src/planners/viplanner.cpp
        /usr/share/sbpl/src/utils/heap.cpp
        /usr/share/sbpl/src/utils/mdp.cpp
        /usr/share/sbpl/src/utils/utils.cpp
        /usr/share/sbpl/src/utils/2Dgridsearch.cpp
        /usr/share/sbpl/src/utils/config.cpp
	)

	IF(WIN32)
		SET(SRCS ${SRCS} resource.rc)
	ENDIF(WIN32)

	INCLUDE_DIRECTORIES("${MRPT_SOURCE_DIR}/otherlibs/wxThings/")

	SET(TMP_TARGET_NAME "holonomic-navigator-demo")

	# Define the executable target:
	ADD_EXECUTABLE(${TMP_TARGET_NAME} WIN32 ${SRCS} ${MRPT_VERSION_RC_FILE})

	

	# Add the required libraries for linking:
	TARGET_LINK_LIBRARIES(${TMP_TARGET_NAME} ${wxWidgets_LIBRARIES} )
	TARGET_LINK_LIBRARIES(${TMP_TARGET_NAME} ${MRPT_LINKER_LIBS})

	TARGET_LINK_LIBRARIES(${TMP_TARGET_NAME} mrptwxthings)
	ADD_DEPENDENCIES(${TMP_TARGET_NAME} mrptwxthings)

	# Dependencies on MRPT libraries:
	#  Just mention the top-level dependency, the rest will be detected automatically, 
	#  and all the needed #include<> dirs added (see the script DeclareAppDependencies.cmake for further details)
	DeclareAppDependencies(${TMP_TARGET_NAME} mrpt-nav mrpt-gui)

	DeclareAppForInstall(${TMP_TARGET_NAME})

	AppStartMenuLink(${TMP_TARGET_NAME} "Holonomic navigation simulator")   # Add link to the Start menu (in Windows Installable packages)
			
ENDIF(wxWidgets_FOUND)

