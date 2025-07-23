# Include all CMakeLists.txt files in subdirectories
file(GLOB_RECURSE SUBDIR_CMAKELISTS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} */CMakeLists.txt)

foreach(CMAKELIST ${SUBDIR_CMAKELISTS})
    get_filename_component(SUBDIR ${CMAKELIST} DIRECTORY)
    add_subdirectory(${SUBDIR})
endforeach()