find_package(CesiumNative REQUIRED)

if (CESIUM_NATIVE_FOUND)

    message(STATUS "Found Cesium Native")

    set(LIB_NAME osgEarthCesium)

    if(OSGEARTH_BUILD_SHARED_LIBS)
        add_definitions(-DOSGEARTHCESIUM_LIBRARY)
    else()
        add_definitions(-DOSGEARTHCESIUM_LIBRARY_STATIC)
    endif()

    # Add definitions for linking against async++ and spdlog.  Found in the cesium-omniverse project.
    add_definitions(-DSPDLOG_COMPILED_LIB -DLIBASYNC_STATIC -DGLM_FORCE_XYZW_ONLY -DGLM_FORCE_EXPLICIT_CTOR -DGLM_FORCE_SIZE_T_LENGTH)

    set(TARGET_SRC
        AssetAccessor.cpp
        CesiumIon.cpp
        CesiumCreditsNode.cpp
        CesiumTilesetNode.cpp
        CesiumLayer.cpp
        Context.cpp
        PrepareRenderResources.cpp
        Settings.cpp
        TaskProcessor.cpp
        HyperTerrainBridge.cpp
        HyperTerrainLitShader.cpp
        HyperTerrainPrepareRendererResources.cpp
        HyperTerrainRasterRenderer.cpp
        HyperTerrainImageryFactory.cpp)
        
    set(TARGET_H
        CesiumTilesetNode
        CesiumCreditsNode
        CesiumLayer
        CesiumIon
        Context
        Export
        Settings
        AssetAccessor
        PrepareRenderResources
        TaskProcessor
        HyperTerrainTypes
        HyperTerrainBridge
        HyperTerrainImageryFactory
        HyperTerrainLitShader)

    set(CMAKE_CXX_STANDARD 20)

    add_osgearth_library(
        TARGET ${LIB_NAME}
        PUBLIC_HEADERS ${TARGET_H}
        SOURCES ${TARGET_SRC}
        SHADERS ${TARGET_GLSL}
        TEMPLATES ${TARGET_IN}
        FOLDER "NodeKits")

    # Headers for vcpkg-only deps (async++, glm, …) are not under CESIUM_NATIVE_DIR/include after `cmake --install`.
    if(CESIUM_NATIVE_VCPKG_INSTALLED AND EXISTS "${CESIUM_NATIVE_VCPKG_INSTALLED}/include")
        target_include_directories(${LIB_NAME} BEFORE PRIVATE "${CESIUM_NATIVE_VCPKG_INSTALLED}/include")
    endif()

    target_link_libraries(${LIB_NAME} PRIVATE OE::CESIUM_NATIVE)

    if(MSVC)
        target_compile_options(${LIB_NAME} PRIVATE /Zc:__cplusplus /utf-8 /bigobj)
    endif()

    # Include Cesium Native headers BEFORE anything else (config-specific install prefix).
    target_include_directories(${LIB_NAME} BEFORE PRIVATE
        "$<$<CONFIG:Debug>:${CESIUM_NATIVE_INCLUDE_DIR_DEBUG}>"
        "$<$<NOT:$<CONFIG:Debug>>:${CESIUM_NATIVE_INCLUDE_DIR_RELEASE}>")

endif()
