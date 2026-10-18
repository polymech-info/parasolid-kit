# Build parasolid.dll and its import library.
#
#   cmake -S packages/parasolid-kit -B packages/parasolid-kit/build
#   cmake --build packages/parasolid-kit/build
#
# Outputs (under the cmake build dir):
#   parasolid.dll
#   parasolid.dll.lib   (MSVC) or libparasolid.dll.a (gnu)
# Header: include/parasolid.h
#
# The INTERFACE target `parasolid` carries the header and the import library.
# The DLL path is the target property PARASOLID_DLL.
#
# The library inspects and parses X_T / X_B files. It does not emit a mesh.

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    cmake_minimum_required(VERSION 3.20)
    project(parasolid LANGUAGES NONE)
endif()

set(PARASOLID_CARGO_PROFILE "release" CACHE STRING "Cargo profile: debug or release")
set_property(CACHE PARASOLID_CARGO_PROFILE PROPERTY STRINGS debug release)
if(NOT PARASOLID_CARGO_PROFILE MATCHES "^(debug|release)$")
    message(FATAL_ERROR "PARASOLID_CARGO_PROFILE must be debug or release")
endif()

find_program(PARASOLID_CARGO cargo REQUIRED)
find_program(PARASOLID_RUSTC rustc REQUIRED)

execute_process(
    COMMAND "${PARASOLID_RUSTC}" -vV
    OUTPUT_VARIABLE _parasolid_rustc_vv
    OUTPUT_STRIP_TRAILING_WHITESPACE
)
if(WIN32)
    set(PARASOLID_DLL_NAME "parasolid.dll")
    if(_parasolid_rustc_vv MATCHES "host: [^\r\n]*windows-gnu")
        set(PARASOLID_IMPLIB_NAME "libparasolid.dll.a")
    else()
        set(PARASOLID_IMPLIB_NAME "parasolid.dll.lib")
    endif()
elseif(APPLE)
    set(PARASOLID_DLL_NAME "libparasolid.dylib")
    set(PARASOLID_IMPLIB_NAME "libparasolid.dylib")
else()
    set(PARASOLID_DLL_NAME "libparasolid.so")
    set(PARASOLID_IMPLIB_NAME "libparasolid.so")
endif()

set(_parasolid_cargo_dir "${CMAKE_CURRENT_SOURCE_DIR}/target/${PARASOLID_CARGO_PROFILE}")
set(_parasolid_built_dll "${_parasolid_cargo_dir}/${PARASOLID_DLL_NAME}")
set(_parasolid_built_implib "${_parasolid_cargo_dir}/${PARASOLID_IMPLIB_NAME}")
if(WIN32 AND NOT _parasolid_rustc_vv MATCHES "host: [^\r\n]*windows-gnu")
    set(PARASOLID_STATIC_NAME "parasolid.lib")
else()
    set(PARASOLID_STATIC_NAME "libparasolid.a")
endif()
set(PARASOLID_DLL "${CMAKE_CURRENT_BINARY_DIR}/${PARASOLID_DLL_NAME}")
set(PARASOLID_IMPLIB "${CMAKE_CURRENT_BINARY_DIR}/${PARASOLID_IMPLIB_NAME}")
set(PARASOLID_STATIC "${CMAKE_CURRENT_BINARY_DIR}/${PARASOLID_STATIC_NAME}")

if(PARASOLID_CARGO_PROFILE STREQUAL "release")
    set(_parasolid_cargo_cmd "${PARASOLID_CARGO}" build --release -p parasolid-capi --locked)
else()
    set(_parasolid_cargo_cmd "${PARASOLID_CARGO}" build -p parasolid-capi --locked)
endif()

set(_parasolid_sources
    "${CMAKE_CURRENT_SOURCE_DIR}/Cargo.toml"
    "${CMAKE_CURRENT_SOURCE_DIR}/Cargo.lock"
    "${CMAKE_CURRENT_SOURCE_DIR}/crates/parasolid-capi/Cargo.toml"
    "${CMAKE_CURRENT_SOURCE_DIR}/crates/parasolid-capi/src/lib.rs"
    "${CMAKE_CURRENT_SOURCE_DIR}/crates/parasolid-capi/src/mesh.rs"
    "${CMAKE_CURRENT_SOURCE_DIR}/crates/parasolid-core/Cargo.toml"
)

set(_parasolid_outputs "${PARASOLID_DLL}")
set(_parasolid_copy_cmds
    COMMAND "${CMAKE_COMMAND}" -E copy_if_different "${_parasolid_built_dll}" "${PARASOLID_DLL}")
if(NOT PARASOLID_IMPLIB STREQUAL PARASOLID_DLL)
    list(APPEND _parasolid_outputs "${PARASOLID_IMPLIB}")
    list(APPEND _parasolid_copy_cmds
        COMMAND "${CMAKE_COMMAND}" -E copy_if_different "${_parasolid_built_implib}" "${PARASOLID_IMPLIB}")
endif()
set(_parasolid_built_static "${_parasolid_cargo_dir}/${PARASOLID_STATIC_NAME}")
list(APPEND _parasolid_outputs "${PARASOLID_STATIC}")
list(APPEND _parasolid_copy_cmds
    COMMAND "${CMAKE_COMMAND}" -E copy_if_different "${_parasolid_built_static}" "${PARASOLID_STATIC}")
add_custom_command(
    OUTPUT ${_parasolid_outputs}
    COMMAND ${_parasolid_cargo_cmd}
    ${_parasolid_copy_cmds}
    DEPENDS ${_parasolid_sources}
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
    COMMENT "Build parasolid shared library"
    USES_TERMINAL
    VERBATIM
)

add_custom_target(parasolid_build ALL DEPENDS ${_parasolid_outputs})

add_library(parasolid INTERFACE)
add_dependencies(parasolid parasolid_build)
target_include_directories(parasolid INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/include")
if(WIN32)
    target_link_libraries(parasolid INTERFACE "${PARASOLID_IMPLIB}")
else()
    target_link_libraries(parasolid INTERFACE "${PARASOLID_DLL}")
endif()
set_target_properties(parasolid PROPERTIES
    PARASOLID_DLL "${PARASOLID_DLL}"
    PARASOLID_IMPLIB "${PARASOLID_IMPLIB}"
    PARASOLID_STATIC "${PARASOLID_STATIC}")
