#------- configuro CMake per Qt ------------------------------
cmake_minimum_required(VERSION 2.8)
#se sono su windows
if (WIN32)
    project(project LANGUAGES CXX)
#se sono su linux
elseif(UNIX)
    project(project)
endif()

#------ setup del tipo di configurazione ---------------------
#set(CMAKE_CONFIGURATION_TYPES "DEBUG_CONFIGURATIONS" CACHE STRING "" FORCE)

#---------- includo Qt ---------------------------------------
set(CMAKE_INCLUDE_CURRENT_DIR ON) #include current directory
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_PREFIX_PATH $ENV{QTDIR}) #cerca QT nel path

#-------- aggiungo pacchetti che mi servono ------------------
find_package(Qt5Core REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(Qt5Gui)
find_package(Qt5PrintSupport)
find_package(Eigen3)

#------- includo directories ---------------------------------
set(FORMS_DIR "${CMAKE_SOURCE_DIR}/forms")
set(INCLUDE_DIR "${CMAKE_SOURCE_DIR}/include")
set(SOURCE_DIR "${CMAKE_SOURCE_DIR}/src")

include_directories(${FORMS_DIR})
include_directories(${INCLUDE_DIR})
include_directories(${SOURCE_DIR})

#------ trovo file -------------------------------------------
file(GLOB_RECURSE SOURCES
    "${FORMS_DIR}/*.ui"
    "${FORMS_DIR}/*.qrc"
    "${INCLUDE_DIR}/*.h"
    "${SOURCE_DIR}/*.cpp"
)


#--- per debug logging on RELWITHDEBINFO ---------------------
set_property(DIRECTORY APPEND PROPERTY COMPILE_DEFINITIONS
    $<$<CONFIG:RELWITHDEBINFO>:QT_MESSAGELOGCONTEXT>
)

#------ aggiungo forms a AUTOUIC (IMPORTANTE) ----------------
set(CMAKE_AUTOUIC_SEARCH_PATHS ${CMAKE_AUTOUIC_SEARCH_PATHS} ${FORMS_DIR})

#----- aggiungo eseguibile -----------------------------------
if (WIN32) 
    add_executable(project WIN32 ${SOURCES})
elseif(UNIX)
    add_executable(project ${SOURCES})
endif()

#---- aggiungo target include per project --------------------
target_include_directories(project PRIVATE ${FORMS_DIR})
target_include_directories(project PRIVATE ${INCLUDE_DIR})
target_include_directories(project PRIVATE ${SOURCE_DIR})

#---- link a librerie ----------------------------------------
target_link_libraries(project
		      Qt5::Widgets
                      Qt5::PrintSupport
                      -pthread
                      ${Eigen_LIBRARIES})

