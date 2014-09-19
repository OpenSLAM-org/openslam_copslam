CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

# project name
PROJECT(COPSLAM)

# where are the include files
INCLUDE_DIRECTORIES(inc) 

# compiler flags
ADD_DEFINITIONS(-O2 -w -msse -msse2 -msse3 -msse4)

# add the source dir
ADD_SUBDIRECTORY(./src)
