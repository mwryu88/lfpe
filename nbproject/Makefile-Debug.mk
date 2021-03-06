#
# Generated Makefile - do not edit!
#
# Edit the Makefile in the project folder instead (../Makefile). Each target
# has a -pre and a -post target defined where you can add customized code.
#
# This makefile implements configuration specific macros and targets.


# Environment
MKDIR=mkdir
CP=cp
GREP=grep
NM=nm
CCADMIN=CCadmin
RANLIB=ranlib
CC=gcc
CCC=g++
CXX=g++
FC=gfortran
AS=as

# Macros
CND_PLATFORM=GNU-Linux
CND_DLIB_EXT=so
CND_CONF=Debug
CND_DISTDIR=dist
CND_BUILDDIR=build

# Include project Makefile
include Makefile

# Object Directory
OBJECTDIR=${CND_BUILDDIR}/${CND_CONF}/${CND_PLATFORM}

# Object Files
OBJECTFILES= \
	${OBJECTDIR}/main.o \
	${OBJECTDIR}/src/icp.o \
	${OBJECTDIR}/src/icpPointToPlane.o \
	${OBJECTDIR}/src/icpPointToPoint.o \
	${OBJECTDIR}/src/kdtree.o \
	${OBJECTDIR}/src/matrix.o


# C Compiler Flags
CFLAGS=

# CC Compiler Flags
CCFLAGS=
CXXFLAGS=

# Fortran Compiler Flags
FFLAGS=

# Assembler Flags
ASFLAGS=

# Link Libraries and Options
LDLIBSOPTIONS=`pkg-config --libs opencv` `pkg-config --libs glew` `pkg-config --libs realsense2` `pkg-config --libs glfw3`  

# Build Targets
.build-conf: ${BUILD_SUBPROJECTS}
	"${MAKE}"  -f nbproject/Makefile-${CND_CONF}.mk bin

bin: ${OBJECTFILES}
	${LINK.cc} -o bin ${OBJECTFILES} ${LDLIBSOPTIONS} -lpthread

${OBJECTDIR}/main.o: main.cpp
	${MKDIR} -p ${OBJECTDIR}
	${RM} "$@.d"
	$(COMPILE.cc) -g -I/usr/local/include -Iinclude `pkg-config --cflags opencv` `pkg-config --cflags glew` `pkg-config --cflags realsense2` `pkg-config --cflags glfw3` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/main.o main.cpp

${OBJECTDIR}/src/icp.o: src/icp.cpp
	${MKDIR} -p ${OBJECTDIR}/src
	${RM} "$@.d"
	$(COMPILE.cc) -g -I/usr/local/include -Iinclude `pkg-config --cflags opencv` `pkg-config --cflags glew` `pkg-config --cflags realsense2` `pkg-config --cflags glfw3` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/src/icp.o src/icp.cpp

${OBJECTDIR}/src/icpPointToPlane.o: src/icpPointToPlane.cpp
	${MKDIR} -p ${OBJECTDIR}/src
	${RM} "$@.d"
	$(COMPILE.cc) -g -I/usr/local/include -Iinclude `pkg-config --cflags opencv` `pkg-config --cflags glew` `pkg-config --cflags realsense2` `pkg-config --cflags glfw3` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/src/icpPointToPlane.o src/icpPointToPlane.cpp

${OBJECTDIR}/src/icpPointToPoint.o: src/icpPointToPoint.cpp
	${MKDIR} -p ${OBJECTDIR}/src
	${RM} "$@.d"
	$(COMPILE.cc) -g -I/usr/local/include -Iinclude `pkg-config --cflags opencv` `pkg-config --cflags glew` `pkg-config --cflags realsense2` `pkg-config --cflags glfw3` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/src/icpPointToPoint.o src/icpPointToPoint.cpp

${OBJECTDIR}/src/kdtree.o: src/kdtree.cpp
	${MKDIR} -p ${OBJECTDIR}/src
	${RM} "$@.d"
	$(COMPILE.cc) -g -I/usr/local/include -Iinclude `pkg-config --cflags opencv` `pkg-config --cflags glew` `pkg-config --cflags realsense2` `pkg-config --cflags glfw3` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/src/kdtree.o src/kdtree.cpp

${OBJECTDIR}/src/matrix.o: src/matrix.cpp
	${MKDIR} -p ${OBJECTDIR}/src
	${RM} "$@.d"
	$(COMPILE.cc) -g -I/usr/local/include -Iinclude `pkg-config --cflags opencv` `pkg-config --cflags glew` `pkg-config --cflags realsense2` `pkg-config --cflags glfw3` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/src/matrix.o src/matrix.cpp

# Subprojects
.build-subprojects:

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r ${CND_BUILDDIR}/${CND_CONF}

# Subprojects
.clean-subprojects:

# Enable dependency checking
.dep.inc: .depcheck-impl

include .dep.inc
