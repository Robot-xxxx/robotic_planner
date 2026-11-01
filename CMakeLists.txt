cmake_minimum_required(VERSION 3.20)
project(robotic_planner VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

option(RP_BUILD_TESTS "Build robotic_planner tests" ON)
option(RP_BUILD_APPS "Build robotic_planner applications" ON)
option(RP_BUILD_SHARED "Build robotic_planner as a shared library" OFF)
option(RP_WITH_OMPL "Enable the OMPL motion planning backend" ON)
option(RP_WITH_KDL "Enable the KDL IK adapter" ON)
option(RP_WITH_FCL "Enable the FCL collision adapter" ON)
option(RP_WITH_TESSERACT "Enable the Tesseract planning backend" ON)
set(RP_OMPL_ROOT "" CACHE PATH "OMPL installation prefix (contains include/ompl-1.7, lib and share)")
set(RP_TESSERACT_ROOT "" CACHE PATH "Tesseract installation prefix")

include(FetchContent)
set(FETCHCONTENT_QUIET ON)

if(POLICY CMP0167)
  cmake_policy(SET CMP0167 NEW)
endif()
if(POLICY CMP0169)
  cmake_policy(SET CMP0169 OLD)
endif()

FetchContent_Declare(
  eigen_src
  URL https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.zip
  DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)
FetchContent_GetProperties(eigen_src)
if(NOT eigen_src_POPULATED)
  FetchContent_Populate(eigen_src)
endif()
add_library(rp_eigen INTERFACE)
target_include_directories(rp_eigen SYSTEM INTERFACE ${eigen_src_SOURCE_DIR})

FetchContent_Declare(
  json_src
  URL https://github.com/nlohmann/json/releases/download/v3.11.3/json.tar.xz
  DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)
FetchContent_GetProperties(json_src)
if(NOT json_src_POPULATED)
  FetchContent_Populate(json_src)
endif()
add_library(rp_json INTERFACE)
target_include_directories(rp_json SYSTEM INTERFACE ${json_src_SOURCE_DIR}/include)

if(NOT RP_OMPL_ROOT)
  set(_rp_ompl_candidate "${CMAKE_CURRENT_SOURCE_DIR}/../third_party/ompl_env/Library")
  if(EXISTS "${_rp_ompl_candidate}/share/ompl/cmake/omplConfig.cmake")
    set(RP_OMPL_ROOT "${_rp_ompl_candidate}" CACHE PATH
        "Dependency installation prefix (contains include, lib and share)" FORCE)
  endif()
endif()
if(RP_OMPL_ROOT)
  list(APPEND CMAKE_PREFIX_PATH "${RP_OMPL_ROOT}")
  if(EXISTS "${RP_OMPL_ROOT}/share/orocos_kdl/cmake/orocos_kdl-config.cmake")
    set(orocos_kdl_DIR "${RP_OMPL_ROOT}/share/orocos_kdl/cmake" CACHE PATH "" FORCE)
  endif()
  if(EXISTS "${RP_OMPL_ROOT}/cmake/fcl-config.cmake")
    set(fcl_DIR "${RP_OMPL_ROOT}/cmake" CACHE PATH "" FORCE)
  endif()
  if(EXISTS "${RP_OMPL_ROOT}/lib/ccd/ccd-config.cmake")
    set(ccd_DIR "${RP_OMPL_ROOT}/lib/ccd" CACHE PATH "" FORCE)
  endif()
  if(EXISTS "${RP_OMPL_ROOT}/share/octomap/octomap-config.cmake")
    set(octomap_DIR "${RP_OMPL_ROOT}/share/octomap" CACHE PATH "" FORCE)
  endif()
endif()

set(RP_OMPL_AVAILABLE OFF)
if(RP_WITH_OMPL)
  find_package(ompl CONFIG QUIET)
  if(ompl_FOUND)
    set(RP_OMPL_AVAILABLE ON)
    message(STATUS "OMPL found at ${RP_OMPL_ROOT} (${ompl_DIR})")
  else()
    message(WARNING "RP_WITH_OMPL is ON but OMPL was not found; OMPL backends will be unavailable")
  endif()
endif()

set(RP_KDL_AVAILABLE OFF)
if(RP_WITH_KDL)
  find_package(orocos_kdl CONFIG QUIET)
  if(orocos_kdl_FOUND AND TARGET orocos-kdl)
    set(RP_KDL_AVAILABLE ON)
    message(STATUS "KDL found (${orocos_kdl_DIR})")
  else()
    message(WARNING "RP_WITH_KDL is ON but KDL was not found; KDL IK will be unavailable")
  endif()
endif()

set(RP_FCL_AVAILABLE OFF)
if(RP_WITH_FCL)
  find_package(fcl CONFIG QUIET)
  if(fcl_FOUND AND TARGET fcl)
    set(RP_FCL_AVAILABLE ON)
    message(STATUS "FCL found (${fcl_DIR})")
  else()
    message(WARNING "RP_WITH_FCL is ON but FCL was not found; FCL collision will be unavailable")
  endif()
endif()

if(NOT RP_TESSERACT_ROOT)
  set(_rp_tesseract_candidate "${CMAKE_CURRENT_SOURCE_DIR}/../third_party/tesseract_env")
  if(EXISTS "${_rp_tesseract_candidate}/lib/cmake/tesseract/tesseract-config.cmake")
    set(RP_TESSERACT_ROOT "${_rp_tesseract_candidate}" CACHE PATH
        "Tesseract installation prefix" FORCE)
  endif()
endif()

set(RP_TESSERACT_AVAILABLE OFF)
if(RP_WITH_TESSERACT AND RP_TESSERACT_ROOT)
  list(APPEND CMAKE_PREFIX_PATH "${RP_TESSERACT_ROOT}")
  find_package(tesseract CONFIG QUIET COMPONENTS
    common geometry scene_graph state_solver collision collision_bullet collision_fcl
    srdf urdf kinematics kinematics_kdl environment)
  find_package(tesseract_planning CONFIG QUIET COMPONENTS
    command_language motion_planners motion_planners_simple motion_planners_ompl)
  if(tesseract_FOUND AND tesseract_planning_FOUND
     AND TARGET tesseract::environment
     AND TARGET tesseract::motion_planners_ompl)
    set(RP_TESSERACT_AVAILABLE ON)
    message(STATUS "Tesseract found at ${RP_TESSERACT_ROOT}")
  else()
    message(WARNING "RP_WITH_TESSERACT is ON but Tesseract was not found; Tesseract backend will be unavailable")
  endif()
endif()

set(RP_LIBRARY_TYPE STATIC)
if(RP_BUILD_SHARED)
  set(RP_LIBRARY_TYPE SHARED)
endif()

add_library(robotic_planner ${RP_LIBRARY_TYPE}
  src/core/scene_model/scene.cpp
  src/core/kinematics/kinematics.cpp
  src/core/collision/collision.cpp
  src/planning/coverage/coverage.cpp
  src/planning/planner.cpp
  src/adapters/ompl_planner.cpp
  src/adapters/kdl_kinematics.cpp
  src/adapters/tesseract_planner.cpp
  src/api/scene_io.cpp
  src/api/rp_c.cpp
)

if(RP_BUILD_SHARED AND MSVC)
  set_target_properties(robotic_planner PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS ON)
endif()

target_include_directories(robotic_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(robotic_planner PUBLIC rp_eigen PRIVATE rp_json)
target_compile_features(robotic_planner PUBLIC cxx_std_17)

if(RP_OMPL_AVAILABLE)
  target_link_libraries(robotic_planner PRIVATE ompl::ompl)
  target_compile_definitions(robotic_planner PRIVATE RP_WITH_OMPL=1)

  set(_rp_ompl_runtime_dlls
    "${RP_OMPL_ROOT}/bin/boost_filesystem.dll"
    "${RP_OMPL_ROOT}/bin/boost_serialization.dll"
  )
  add_custom_command(TARGET robotic_planner POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different ${_rp_ompl_runtime_dlls}
            "$<TARGET_FILE_DIR:robotic_planner>"
    COMMENT "Copying OMPL runtime DLLs"
  )
endif()

if(RP_KDL_AVAILABLE)
  target_link_libraries(robotic_planner PRIVATE orocos-kdl)
  target_compile_definitions(robotic_planner PRIVATE RP_WITH_KDL=1)
endif()

if(RP_FCL_AVAILABLE)
  target_link_libraries(robotic_planner PRIVATE fcl)
  target_compile_definitions(robotic_planner PRIVATE RP_WITH_FCL=1)

  set(_rp_fcl_runtime_dlls
    "${RP_OMPL_ROOT}/bin/fcl.dll"
    "${RP_OMPL_ROOT}/bin/ccd.dll"
    "${RP_OMPL_ROOT}/bin/octomap.dll"
    "${RP_OMPL_ROOT}/bin/octomath.dll"
  )
  add_custom_command(TARGET robotic_planner POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different ${_rp_fcl_runtime_dlls}
            "$<TARGET_FILE_DIR:robotic_planner>"
    COMMENT "Copying FCL runtime DLLs"
  )
endif()

if(RP_TESSERACT_AVAILABLE)
  target_link_libraries(robotic_planner PRIVATE
    tesseract::environment
    tesseract::motion_planners_ompl
  )
  target_compile_definitions(robotic_planner PRIVATE
    RP_WITH_TESSERACT=1
    RP_TESSERACT_BIN_DIR="${RP_TESSERACT_ROOT}/bin"
  )
  target_link_directories(robotic_planner PRIVATE
    "${RP_OMPL_ROOT}/lib"
    "${RP_TESSERACT_ROOT}/lib"
  )

  file(GLOB _rp_tesseract_runtime_dlls "${RP_TESSERACT_ROOT}/bin/*.dll")
  set(_rp_tesseract_dependency_dlls
    "${RP_OMPL_ROOT}/bin/assimp.dll"
    "${RP_OMPL_ROOT}/bin/zlib.dll"
    "${RP_OMPL_ROOT}/bin/boost_filesystem.dll"
    "${RP_OMPL_ROOT}/bin/boost_serialization.dll"
    "${RP_OMPL_ROOT}/bin/fcl.dll"
    "${RP_OMPL_ROOT}/bin/ccd.dll"
    "${RP_OMPL_ROOT}/bin/fmt.dll"
    "${RP_OMPL_ROOT}/bin/octomap.dll"
    "${RP_OMPL_ROOT}/bin/octomath.dll"
    "${RP_OMPL_ROOT}/bin/spdlog.dll"
    "${RP_OMPL_ROOT}/bin/tinyxml2.dll"
    "${RP_OMPL_ROOT}/bin/yaml-cpp.dll"
  )
  add_custom_command(TARGET robotic_planner POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            ${_rp_tesseract_runtime_dlls} ${_rp_tesseract_dependency_dlls}
            "$<TARGET_FILE_DIR:robotic_planner>"
    COMMENT "Copying Tesseract runtime DLLs"
  )
endif()

if(MSVC)
  target_compile_options(robotic_planner PRIVATE /W4 /permissive- /bigobj)
else()
  target_compile_options(robotic_planner PRIVATE -Wall -Wextra -Wpedantic)
endif()

target_compile_definitions(robotic_planner PRIVATE RP_VERSION_STRING="${PROJECT_VERSION}")

if(RP_BUILD_APPS)
  add_executable(rp_cli apps/rp_cli.cpp)
  target_link_libraries(rp_cli PRIVATE robotic_planner rp_json)
  target_compile_definitions(rp_cli PRIVATE RP_DEFAULT_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/scenes/car_door.json")

  add_executable(rp_validate apps/rp_validate.cpp)
  target_link_libraries(rp_validate PRIVATE robotic_planner rp_json)
  target_compile_definitions(rp_validate PRIVATE RP_DEFAULT_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/scenes/car_door.json")

  if(MSVC)
    target_compile_options(rp_cli PRIVATE /W4 /permissive-)
    target_compile_options(rp_validate PRIVATE /W4 /permissive-)
  endif()
endif()

if(RP_BUILD_TESTS)
  enable_testing()
  foreach(test_name test_core test_coverage test_planner test_capi test_small_range test_ompl test_pose_sequence test_pose_groups test_adapters)
    add_executable(${test_name} tests/${test_name}.cpp)
    target_link_libraries(${test_name} PRIVATE robotic_planner rp_json)
    target_compile_definitions(${test_name} PRIVATE
      RP_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
      RP_DEFAULT_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/scenes/car_door.json"
    )
    add_test(NAME ${test_name} COMMAND ${test_name})
  endforeach()
endif()
