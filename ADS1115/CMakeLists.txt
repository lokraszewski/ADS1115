# Sources
set(SRC_FILES
    # src/ADS1115.cpp
)

if(UNIX)
    list(APPEND SRC_FILES src/impl/unix.cpp)
endif()

message(STATUS "Building ${PROJECT_NAME} with: ")
foreach (_src ${SRC_FILES})
    message(STATUS "  ${_src}")
endforeach()


## Add serial library
add_library(${PROJECT_NAME} ${SRC_FILES})

# setup include directories
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

if(UNIX)
  target_link_libraries(${PROJECT_NAME} rt pthread)
endif()

 # setup export preprocessor definitions
if(BUILD_SHARED_LIBS)
  target_compile_definitions(${PROJECT_NAME} PRIVATE ${PROJECT_NAME}_EXPORTS)
  target_compile_definitions(${PROJECT_NAME} INTERFACE ${PROJECT_NAME}_IMPORTS)
endif()

if(TESTS_ENABLED)
  message(STATUS "Testing enabled.")
  add_subdirectory(test)
else()
  message(STATUS "Testing disabled.")
endif()
