
## Add serial library
add_library(${PROJECT_NAME} INTERFACE)

# setup include directories
target_include_directories(
  ${PROJECT_NAME}
  INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

if(UNIX)
  target_link_libraries(${PROJECT_NAME} INTERFACE rt pthread)
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
