cmake_minimum_required(VERSION 3.15)
project(ur_rtde_c_api VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Options
option(BUILD_SHARED_LIBS "Build shared library" ON)
option(UR_RTDE_FACADE_STATIC_RTDE "Link a static ur_rtde archive into the C facade" OFF)

# Find ur_rtde (expects it to be installed or provide UR_RTDE_DIR)
find_package(ur_rtde REQUIRED)

# Split macOS dylibs clear imported Boost deps because librtde already embeds Boost.
# Static facade builds keep those deps so Boost archives link into libur_rtde_c_api.dylib.
if(APPLE AND NOT UR_RTDE_FACADE_STATIC_RTDE)
  set_target_properties(ur_rtde::rtde PROPERTIES INTERFACE_LINK_LIBRARIES "")
endif()

# Windows/Linux: facade links Boost explicitly. Static macOS facade links Boost archives through ur_rtde.
if(NOT APPLE)
  find_package(Boost REQUIRED COMPONENTS system thread)
endif()

# C API Façade Library
add_library(ur_rtde_c_api
  ur_rtde_c_api.cpp
  ur_rtde_c_api.h
)

target_include_directories(ur_rtde_c_api
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include>
)

# PRIVATE: do not propagate ur_rtde's INTERFACE Boost deps into the facade dylib
# (macOS static librtde already embeds Boost; extra Boost dylibs collide with Rhino).
target_link_libraries(ur_rtde_c_api
  PRIVATE
    ur_rtde::rtde
)

if(NOT APPLE)
  target_link_libraries(ur_rtde_c_api PRIVATE Boost::system Boost::thread)
endif()

# Platform-specific settings
if(WIN32)
  target_compile_definitions(ur_rtde_c_api PRIVATE UR_RTDE_C_API_EXPORTS)
  # Set output name for Windows
  set_target_properties(ur_rtde_c_api PROPERTIES OUTPUT_NAME "ur_rtde_c_api")
elseif(APPLE)
  set_target_properties(ur_rtde_c_api PROPERTIES
    MACOSX_RPATH ON
    INSTALL_RPATH "@loader_path"
    CXX_VISIBILITY_PRESET hidden
    VISIBILITY_INLINES_HIDDEN YES
  )
  target_link_options(ur_rtde_c_api PRIVATE
    "LINKER:-exported_symbols_list,${CMAKE_CURRENT_SOURCE_DIR}/ur_rtde_c_api.exports"
  )
else()
  set_target_properties(ur_rtde_c_api PROPERTIES
    INSTALL_RPATH "$ORIGIN"
  )
endif()

# Install rules
install(TARGETS ur_rtde_c_api
  EXPORT ur_rtde_c_api-targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(FILES ur_rtde_c_api.h DESTINATION include)

# Export for find_package
install(EXPORT ur_rtde_c_api-targets
  FILE ur_rtde_c_api-config.cmake
  NAMESPACE ur_rtde_c_api::
  DESTINATION lib/cmake/ur_rtde_c_api
)
