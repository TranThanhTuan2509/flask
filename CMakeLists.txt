
cmake_minimum_required(VERSION 3.16)
project(flask_repro LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()


set(VAMP_BUILD_PYTHON_BINDINGS OFF CACHE BOOL "" FORCE)
set(VAMP_INSTALL_CPP_LIBRARY OFF CACHE BOOL "" FORCE)
set(VAMP_LTO OFF CACHE BOOL "" FORCE)


file(GLOB _VD_LIST ${CMAKE_CURRENT_SOURCE_DIR}/third_party/vamp/build/*/_deps)
if(_VD_LIST)
  list(GET _VD_LIST 0 _VD)
  if(EXISTS ${_VD}/nigh-src AND EXISTS ${_VD}/pdqsort-src AND EXISTS ${_VD}/simdxorshift-src)
    message(STATUS "Using cached VAMP dependencies")
    set(CPM_nigh_SOURCE ${_VD}/nigh-src CACHE PATH "")
    set(CPM_pdqsort_SOURCE ${_VD}/pdqsort-src CACHE PATH "")
    set(CPM_SIMDxorshift_SOURCE ${_VD}/simdxorshift-src CACHE PATH "")
  endif()
endif()
add_subdirectory(third_party/vamp EXCLUDE_FROM_ALL)


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native -mavx2 -Wall -Wextra -Wno-ignored-attributes")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG -fno-math-errno -fno-signed-zeros -fno-trapping-math -fno-rounding-math -ffp-contract=fast -fassociative-math")

find_package(Python 3.10 COMPONENTS Interpreter Development.Module REQUIRED)
execute_process(COMMAND "${Python_EXECUTABLE}" -m nanobind --cmake_dir
                OUTPUT_STRIP_TRAILING_WHITESPACE OUTPUT_VARIABLE nanobind_ROOT)
find_package(nanobind CONFIG REQUIRED)

nanobind_add_module(_core NB_STATIC NOMINSIZE src/bindings.cc src/flat_scalar.cc)
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  set_source_files_properties(src/flat_scalar.cc PROPERTIES COMPILE_OPTIONS "-fno-tree-vectorize;-fno-tree-slp-vectorize")
elseif(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set_source_files_properties(src/flat_scalar.cc PROPERTIES COMPILE_OPTIONS "-fno-vectorize;-fno-slp-vectorize")
endif()
target_include_directories(_core PRIVATE src)
target_link_libraries(_core PRIVATE vamp_cpp)
set_target_properties(_core PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/flask_repro)
