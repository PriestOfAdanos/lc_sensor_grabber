cmake_minimum_required(VERSION 3.8)
project(web_talker)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(JS_PACKAGED_DIR "${CMAKE_CURRENT_SOURCE_DIR}/src/frontend_dist") 
set(JS_RAW_DIR "${CMAKE_CURRENT_SOURCE_DIR}/src/frontend")
find_package(ament_cmake REQUIRED)

# TODO(PriestOfAdanos): add npm install to be done inside isntall folder, and the same for browserify
# fs.createReadStream('/home/lc/ws/web_talker/src/index.html').pipe(res) TODO(PriestOfAdanos): Needs to be included in build script


file(GLOB list_of_js_frontend_files 
    "${JS_RAW_DIR}/*.js"
)
foreach(FILE ${list_of_js_frontend_files})
get_filename_component(filename ${FILE} NAME)

add_custom_command(
  OUTPUT  ${JS_PACKAGED_DIR}/${filename}
  DEPENDS ${JS_RAW_DIR}/${filename}
  COMMAND browserify ${JS_RAW_DIR}/${filename} >  ${JS_PACKAGED_DIR}/${filename}
  )
  list(APPEND DOC_RESULTS ${JS_PACKAGED_DIR}/${filename})
endforeach()
message(WARNING ${CMAKE_CURRENT_SOURCE_DIR})
add_custom_target(JS_COMPILED ALL
  DEPENDS ${DOC_RESULTS}
  )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

# install rules added by ros2pkg_configure_nodejs
install(FILES 
  package.json
  DESTINATION share/${PROJECT_NAME}/
 )
 
 # only use with javascript projects (not typescript)
 install(DIRECTORY 
  src/
  DESTINATION share/${PROJECT_NAME}/dist
  OPTIONAL
 )
 
install(DIRECTORY 
  config
  dist
  launch
  node_modules
  DESTINATION share/${PROJECT_NAME}/
  OPTIONAL
 )
 
