# Copyright (c) 2017 The Khronos Group Inc.
#
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Author:
#
include(FetchContent)

FetchContent_Declare(
  readerwriterqueue
  GIT_REPOSITORY    https://github.com/cameron314/readerwriterqueue
  GIT_TAG           master
)
FetchContent_MakeAvailable(readerwriterqueue)

file(GLOB LOCAL_HEADERS "*.h")
file(GLOB LOCAL_SOURCE "*.cpp")
file(GLOB VULKAN_SHADERS "vulkan_shaders/*.glsl")
file(GLOB D3D_SHADERS "d3d_shaders/*.hlsl")

# Mark HLSL files to prevent the VS generator adding them for automatic shader compilation with FXC (they should be compiled with DXC for SM6 support)
foreach(hlsl_file IN LISTS D3D_SHADERS)
    set_source_files_properties(${hlsl_file} PROPERTIES VS_TOOL_OVERRIDE "None")
endforeach()

# For including compiled shaders
include_directories(${CMAKE_CURRENT_BINARY_DIR})

set(CUDA_LIB_LIST)
set(ENABLE_CUDA_INTEROP FALSE)
if(NOT ANDROID)
    option(BUILD_CUDA_INTEROP "Enables explicit CUDA buffer interop" ON)
    if (BUILD_CUDA_INTEROP)
        find_package(CUDAToolkit)
        if (CUDAToolkit_FOUND)
            set(CUDA_LIB_LIST CUDA::cudart CUDA::cuda_driver)
            set(ENABLE_CUDA_INTEROP TRUE)
            message(STATUS "Enabling CUDA Buffer Interop")
        endif()
    endif()
endif()

if(ANDROID)
    add_library(alxr_engine MODULE
        ${LOCAL_SOURCE}
        ${LOCAL_HEADERS}
        ${VULKAN_SHADERS})
        #$<TARGET_OBJECTS:android_native_app_glue>)
    target_link_libraries(alxr_engine ${ANDROID_LIBRARY} ${ANDROID_LOG_LIBRARY} mediandk)
else()
    set(CMAKE_INSTALL_RPATH $ORIGIN)
    add_library(alxr_engine SHARED
        ${LOCAL_SOURCE}
        ${LOCAL_HEADERS}
        ${VULKAN_SHADERS}
        ${D3D_SHADERS})
endif()
set_target_properties(alxr_engine PROPERTIES FOLDER ${SAMPLES_FOLDER})
target_compile_definitions(alxr_engine PRIVATE ALXR_CLIENT)
if (XR_IS_UWP)
    target_compile_definitions(alxr_engine PRIVATE XR_USE_PLATFORM_UWP)
endif()

source_group("Headers" FILES ${LOCAL_HEADERS})
source_group("Shaders" FILES ${VULKAN_SHADERS})

compile_glsl(run_alxr_engine_glsl_compiles ${VULKAN_SHADERS})
compile_hlsl_sm5(run_alxr_engine_hlsl_sm5_compiles ${D3D_SHADERS})
compile_hlsl_sm6(run_alxr_engine_hlsl_sm6_compiles ${D3D_SHADERS})

add_dependencies(alxr_engine
    generate_openxr_header
    run_alxr_engine_glsl_compiles
    run_alxr_engine_hlsl_sm5_compiles
    run_alxr_engine_hlsl_sm6_compiles
)

set(ALVR_ROOT_DIR ${PROJECT_SOURCE_DIR}/../../../../.. CACHE PATH "Root directory of ALVR source")
set(ALVR_COMMON_DIR ${ALVR_ROOT_DIR}/alvr/client/android/ALVR-common)
set(ALVR_OLD_CLIENT_DIR ${ALVR_ROOT_DIR}/alvr/client/android/app/src/main/cpp)

file(GLOB_RECURSE ALVR_COMMON_HEADERS ${ALVR_COMMON_DIR}/*.h)
file(GLOB_RECURSE ALVR_COMMON_SOURCE ${ALVR_COMMON_DIR}/*.c ${ALVR_COMMON_DIR}/*.cpp)
add_library(alvr_common
    ${ALVR_COMMON_HEADERS}
    ${ALVR_OLD_CLIENT_DIR}/fec.h
    ${ALVR_OLD_CLIENT_DIR}/latency_collector.h
    ${ALVR_COMMON_SOURCE}
    ${ALVR_OLD_CLIENT_DIR}/fec.cpp
    ${ALVR_OLD_CLIENT_DIR}/latency_collector.cpp)
target_include_directories(alvr_common PRIVATE
    ${ALVR_COMMON_DIR}
)
target_compile_definitions(alvr_common PRIVATE ALXR_CLIENT)
if (MSVC)
    target_compile_definitions(alvr_common PRIVATE _CRT_SECURE_NO_WARNINGS PRIVATE _SILENCE_ALL_CXX17_DEPRECATION_WARNINGS)
    target_compile_options(alvr_common PRIVATE /Zc:wchar_t /Zc:forScope /W3 /Zi)
endif()

set(FFMPEG_LIBS)
if(ANDROID)
    set(FFMPEG_LIBS)
    set(ONE_API_TTB_LIBS)
elseif(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    set(PKG_CONFIG_USE_CMAKE_PREFIX_PATH ON)
    find_package(PkgConfig REQUIRED)
    set(PKG_CONFIG_USE_CMAKE_PREFIX_PATH ON)

    pkg_check_modules(LIBAV REQUIRED IMPORTED_TARGET
        libavutil#>=56.70.100
        #libswresample
        #libavdevice
        libavcodec#>=58.134.100
        #libavformat#>=58.76.100
        libavfilter#>=7.110.100
        libswscale#>=5.9.100
    )

    set(FFMPEG_LIBS PkgConfig::LIBAV)
    
    #pkg_check_modules(TBB REQUIRED IMPORTED_TARGET
    #    tbb
    #)
    #set(ONE_API_TTB_LIBS PkgConfig::TBB)
    set(ONE_API_TTB_LIBS)
elseif (WIN32)

    set(FFMPEG_PLATFORM_DIR "win32")
    if (XR_IS_UWP)
        set(FFMPEG_PLATFORM_DIR "uwp")
    endif()

    string(TOLOWER ${XR_TARGET_CPU_ARCH} XR_TARGET_CPU_ARCH_LOWERED)
    set(FFMPEG_DIR ${PROJECT_SOURCE_DIR}/src/external/ffmpeg/${FFMPEG_PLATFORM_DIR}/${XR_TARGET_CPU_ARCH_LOWERED})    
    set(FFMPEG_BIN_DIR ${FFMPEG_DIR}/bin)
    set(FFMPEG_LIB_DIR ${FFMPEG_DIR}/lib)

    function(add_ffmpeg_shared_lib libName importLib dllName)
        add_library(${libName} SHARED IMPORTED)
        set_target_properties(${libName} PROPERTIES
            IMPORTED_LOCATION ${FFMPEG_BIN_DIR}/${dllName}
            IMPORTED_IMPLIB ${FFMPEG_LIB_DIR}/${importLib}
	        INTERFACE_INCLUDE_DIRECTORIES ${FFMPEG_DIR}/include)
    endfunction()  

    add_ffmpeg_shared_lib("libavutil" "avutil.lib" "avutil-57.dll")
    add_ffmpeg_shared_lib("libavdevice" "avdevice.lib" "avdevice-59.dll")
    add_ffmpeg_shared_lib("libswresample" "swresample.lib" "swresample-4.dll")
    add_ffmpeg_shared_lib("libavcodec" "avcodec.lib" "avcodec-59.dll")
    add_ffmpeg_shared_lib("libavformat" "avformat.lib" "avformat-59.dll")
    add_ffmpeg_shared_lib("libavfilter" "avfilter.lib" "avfilter-8.dll")
    add_ffmpeg_shared_lib("libswscale" "swscale.lib" "swscale-6.dll")
    
    # avutil, avresample, avcodec, avformat 
    set(FFMPEG_LIBS libavutil libswresample libavdevice libavcodec libavformat libavfilter libswscale)
    set(ONE_API_TTB_LIBS)
endif()

target_include_directories(alxr_engine
    PRIVATE
    ${PROJECT_SOURCE_DIR}/src
    ${PROJECT_SOURCE_DIR}/src/common

    # for OpenXR headers
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_BINARY_DIR}/include

    # for helper headers
    ${PROJECT_SOURCE_DIR}/external/include

    ${ALVR_COMMON_DIR}
    ${ALVR_COMMON_DIR}/../
    ${ALVR_COMMON_DIR}/../app/src/main/cpp
    ${ALVR_OLD_CLIENT_DIR}
)

if(GLSLANG_VALIDATOR AND NOT GLSLC_COMMAND)
    target_compile_definitions(alxr_engine PRIVATE USE_GLSLANGVALIDATOR)
endif()

if(ENABLE_CUDA_INTEROP)
    target_compile_definitions(alxr_engine PRIVATE XR_ENABLE_CUDA_INTEROP)
endif()

if(Vulkan_FOUND)
    target_include_directories(alxr_engine
        PRIVATE
        ${Vulkan_INCLUDE_DIRS}
    )
endif()

if (XR_IS_UWP)
    target_link_libraries(alxr_engine OneCore.lib WindowsApp.lib)
endif()
target_link_libraries(alxr_engine openxr_loader alvr_common readerwriterqueue ${CUDA_LIB_LIST} ${FFMPEG_LIBS} ${ONE_API_TTB_LIBS})

if(TARGET openxr-gfxwrapper)
    target_link_libraries(alxr_engine openxr-gfxwrapper)
endif()
if(WIN32)
    if(MSVC)
        target_compile_definitions(alxr_engine PRIVATE _CRT_SECURE_NO_WARNINGS)
        target_compile_options(alxr_engine PRIVATE /Zc:wchar_t /Zc:forScope /W3 /WX /Zi)
        target_compile_options(alxr_engine PRIVATE "$<$<NOT:$<CONFIG:Debug>>:/Zi>")
        target_link_options(alxr_engine PRIVATE "$<$<NOT:$<CONFIG:Debug>>:/DEBUG>")
        target_link_options(alxr_engine PRIVATE "$<$<NOT:$<CONFIG:Debug>>:/OPT:REF>")
        target_link_options(alxr_engine PRIVATE "$<$<NOT:$<CONFIG:Debug>>:/OPT:ICF>")
    endif()
    target_link_libraries(alxr_engine ole32)
    if(MSVC)
        # Right now can't build this on MinGW because of directxcolors, etc.
        target_link_libraries(alxr_engine dxguid d3d11 d3d12 d3dcompiler dxgi)
    else()
        target_compile_definitions(alxr_engine PRIVATE MISSING_DIRECTX_COLORS)
    endif()
endif()

if(Vulkan_LIBRARY)
    target_link_libraries(alxr_engine ${Vulkan_LIBRARY})
endif()

if(NOT ANDROID)
    #set(ALVR_BIN_DIR ${ALVR_ROOT_DIR}/target/debug/examples)
    #include(GNUInstallDirs)
    #install(DIRECTORY ${FFMPEG_BIN_DIR} TYPE BIN)
    if(MSVC)
        string(TOLOWER ${CMAKE_BUILD_TYPE} BUILD_TYPE_LOWERED)
        if (${BUILD_TYPE_LOWERED} MATCHES ".*debug.*")
            message("Installing debug CRT depends")
            set(CMAKE_INSTALL_DEBUG_LIBRARIES TRUE)
            set(CMAKE_INSTALL_UCRT_LIBRARIES TRUE)
            #set(CMAKE_INSTALL_DEBUG_LIBRARIES_ONLY TRUE)
        endif()

        include(InstallRequiredSystemLibraries)
        #add_custom_target(copy_debug_libs
        #    COMMAND ${CMAKE_COMMAND} -E copy
        #    ${CMAKE_INSTALL_SYSTEM_RUNTIME_LIBS} "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/$<CONFIG>"
        #    || (COMMAND ${CMAKE_COMMAND} -E echo "Unable to copy the debug libraries, continuing..."))
        foreach(libav_var ${FFMPEG_LIBS})
            install(IMPORTED_RUNTIME_ARTIFACTS ${libav_var}
                RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
                #LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
                #ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})
        endforeach()
    endif()
    
    # if (INSTALL_BUNDLED_FFMPEG AND CMAKE_SYSTEM_NAME STREQUAL "Linux")
    #     foreach(LIBAV_SO ${LIBAV_LINK_LIBRARIES})
    #         file(REAL_PATH ${LIBAV_SO} REAL_LIBAV_SO)
    #         install(FILES ${REAL_LIBAV_SO} DESTINATION ${CMAKE_INSTALL_LIBDIR})
    #     endforeach()
    # endif()
    
    #install(IMPORTED_RUNTIME_ARTIFACTS JsonCpp
    #           RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

    install(TARGETS alxr_engine
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})
    if(WIN32)
        install(FILES $<TARGET_PDB_FILE:alxr_engine> DESTINATION ${CMAKE_INSTALL_BINDIR})
        install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/shaders/ DESTINATION ${CMAKE_INSTALL_BINDIR}/shaders)
    endif()
endif()
