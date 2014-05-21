
project(qrcodelist)
cmake_minimum_required (VERSION 2.6) 

if(WIN32)
   if(MSVC)
      add_definitions(-DWIN32_LEAN_AND_MEAN)
      add_definitions(-D_CRT_SECURE_NO_WARNINGS)
   endif(MSVC)
else()
   add_definitions(-Wno-write-strings)
endif(WIN32)


add_subdirectory (math)

file(GLOB qrcodelist_srcs "*.cpp")
file(GLOB qrcodelist_incl "*.h")

source_group("Src" FILES ${qrcodelist_srcs})
source_group("Includes" FILES ${qrcodelist_incl})


set( CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake" )

FIND_PACKAGE( GLIB2 REQUIRED )
FIND_PACKAGE( GTHREAD2 REQUIRED )
FIND_PACKAGE( IVY REQUIRED )

add_definitions(-Wall )

INCLUDE_DIRECTORIES(
  ${GLIB2_MAIN_INCLUDE_DIR}
  ${GLIB2_INTERNAL_INCLUDE_DIR}
  ${GTHREAD2_MAIN_INCLUDE_DIR}
  ${IVY_INCLUDE_DIR}
  ${GLIBIVY_INCLUDE_DIR}
  ${GLIB2_MAIN_INCLUDE_DIR}
  ${GLIB2_INTERNAL_INCLUDE_DIR}
  ${GTHREAD2_MAIN_INCLUDE_DIR}
)

include_directories("math")

add_executable (qrcodelist ${qrcodelist_srcs} ${qrcodelist_incl} )

target_link_libraries(qrcodelist ${GLIB2_LIBRARY} ${GTHREAD2_LIBRARY} ${GLIBIVY_LIBRARY_OPTIMIZED} )
target_link_libraries(qrcodelist uav_math)
#target_link_libraries(qrcodelist lpcre)


add_dependencies(qrcodelist uav_math) 
