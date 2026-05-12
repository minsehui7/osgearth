#
# FindCesiumNative.cmake
#
# Inputs:
#   CESIUM_NATIVE_DIR : folder containing include/ and lib/ for Cesium Native
#       (or "CESIUM_NATIVE_DIR" environment variable)
#
# Outputs:
#   CESIUM_NATIVE_FOUND (boolean variable)
#   OE::CESIUM_NATIVE (imported link library)
#
# Usage:
#   find_package(CesiumNative)
#   ...
#   target_link_libraries(TARGET my_target PRIVATE OE::CESIUM_NATIVE)
#
set(CESIUM_NATIVE_DIR "" CACHE PATH "Root directory of cesium-native distribution")

unset(CESIUM_NATIVE_FOUND)

# Location the cesium-native installation:
find_path(CESIUM_NATIVE_INCLUDE_DIR CesiumUtility/Uri.h
    PATHS
        ${CESIUM_NATIVE_DIR}
        $ENV{CESIUM_NATIVE_DIR}
    PATH_SUFFIXES
        include )
        
set(CESIUM_ANY_LIBRARY_MISSING FALSE)
        
# Macro to locate each cesium library.
macro(find_cesium_library MY_LIBRARY_VAR MY_LIBRARY_NAME)

    # start by clearing any existing value so we pick the dependency
    # that built with the cesium-native package
    unset(${MY_LIBRARY_VAR}_LIBRARY_DEBUG CACHE)
    unset(${MY_LIBRARY_VAR}_LIBRARY_RELEASE CACHE)

    if (NOT CESIUM_ANY_LIBRARY_MISSING)
        find_library(${MY_LIBRARY_VAR}_LIBRARY_DEBUG
            NAMES
                ${MY_LIBRARY_NAME}d
            PATHS
                ${CESIUM_NATIVE_DIR}/lib
                $ENV{CESIUM_NATIVE_DIR}
                PATH_SUFFIXES lib64 lib
            NO_DEFAULT_PATH )

        find_library(${MY_LIBRARY_VAR}_LIBRARY_RELEASE
             NAMES
                 ${MY_LIBRARY_NAME}
             PATHS
                 ${CESIUM_NATIVE_DIR}/lib
                 $ENV{CESIUM_NATIVE_DIR}
                 PATH_SUFFIXES lib64 lib
             NO_DEFAULT_PATH )

        set(MY_DEBUG_LIBRARY "${${MY_LIBRARY_VAR}_LIBRARY_DEBUG}")
        set(MY_RELEASE_LIBRARY "${${MY_LIBRARY_VAR}_LIBRARY_RELEASE}")       
        
        if(MY_DEBUG_LIBRARY OR MY_RELEASE_LIBRARY)
            # name of the import for this component:
            set(MY_IMPORT_LIBRARY_NAME "CesiumNative::${MY_LIBRARY_NAME}")

            # create the import library for this component:
            add_library(${MY_IMPORT_LIBRARY_NAME} UNKNOWN IMPORTED)
            
            # Normally you would need to add the include folders to the import library.
            # But because cesium native's includes need to come FIRST, we will omit
            # it and specify it later with a BEFORE argument to include_directories.
            #set_target_properties(${MY_IMPORT_LIBRARY_NAME} PROPERTIES
            #    INTERFACE_INCLUDE_DIRECTORIES "${CESIUM_NATIVE_INCLUDE_DIR}")
            
            set_target_properties(${MY_IMPORT_LIBRARY_NAME} PROPERTIES
                IMPORTED_LOCATION ${MY_RELEASE_LIBRARY}
                IMPORTED_LOCATION_DEBUG ${MY_DEBUG_LIBRARY} )

            # finally, add it to the main list for later.
            list(APPEND CESIUM_NATIVE_IMPORT_LIBRARIES "${MY_IMPORT_LIBRARY_NAME}")
        else()
            message(WARNING "Cesium Native: Could NOT find ${MY_LIBRARY_NAME}")
            set(CESIUM_ANY_LIBRARY_MISSING TRUE)
        endif()
    endif()
endmacro()

# Note, it's not strictly necessary to set all these cache variables,
# but it's nice to be able to see them in cmake-gui. -gw

# -- Cesium-native own libraries --
find_cesium_library(CESIUM_NATIVE_3DTILES_SELECTION Cesium3DTilesSelection)
find_cesium_library(CESIUM_NATIVE_ION_CLIENT CesiumIonClient)
find_cesium_library(CESIUM_NATIVE_ITWIN_CLIENT CesiumITwinClient)
find_cesium_library(CESIUM_NATIVE_CLIENT_COMMON CesiumClientCommon)
find_cesium_library(CESIUM_NATIVE_CURL CesiumCurl)
find_cesium_library(CESIUM_NATIVE_RASTER_OVERLAYS CesiumRasterOverlays)
find_cesium_library(CESIUM_NATIVE_3DTILES_CONTENT Cesium3DTilesContent)
find_cesium_library(CESIUM_NATIVE_3DTILES_READER Cesium3DTilesReader)
find_cesium_library(CESIUM_NATIVE_3DTILES_WRITER Cesium3DTilesWriter)
find_cesium_library(CESIUM_NATIVE_3DTILES Cesium3DTiles)
find_cesium_library(CESIUM_NATIVE_QUANTIZED_MESH_TERRAIN CesiumQuantizedMeshTerrain)
find_cesium_library(CESIUM_NATIVE_GLTF_CONTENT CesiumGltfContent)
find_cesium_library(CESIUM_NATIVE_GLTF_READER CesiumGltfReader)
find_cesium_library(CESIUM_NATIVE_GLTF_WRITER CesiumGltfWriter)
find_cesium_library(CESIUM_NATIVE_GLTF CesiumGltf)
find_cesium_library(CESIUM_NATIVE_GEOSPATIAL CesiumGeospatial)
find_cesium_library(CESIUM_NATIVE_GEOMETRY CesiumGeometry)
find_cesium_library(CESIUM_NATIVE_ASYNC CesiumAsync)
find_cesium_library(CESIUM_NATIVE_JSONREADER CesiumJsonReader)
find_cesium_library(CESIUM_NATIVE_JSONWRITER CesiumJsonWriter)
find_cesium_library(CESIUM_NATIVE_UTILITY CesiumUtility) 
find_cesium_library(CESIUM_NATIVE_VECTOR_DATA CesiumVectorData)

# -- Third-party dependencies (vcpkg-installed, static) --
find_cesium_library(CESIUM_NATIVE_ASYNC++ async++)
find_cesium_library(CESIUM_NATIVE_DRACO draco)
find_cesium_library(CESIUM_NATIVE_KTX ktx)
find_cesium_library(CESIUM_NATIVE_MODPB64 libmodpbase64)
find_cesium_library(CESIUM_NATIVE_S2 s2)
find_cesium_library(CESIUM_NATIVE_SPDLOG spdlog)
find_cesium_library(CESIUM_NATIVE_FMT fmt)
find_cesium_library(CESIUM_NATIVE_TINYXML2 tinyxml2)
find_cesium_library(CESIUM_NATIVE_TURBOJPEG turbojpeg)
find_cesium_library(CESIUM_NATIVE_ADA ada)
find_cesium_library(CESIUM_NATIVE_WEBPDECODER libwebpdecoder)
find_cesium_library(CESIUM_NATIVE_MESHOPTIMIZER meshoptimizer)
find_cesium_library(CESIUM_NATIVE_SQLITE3 sqlite3)
find_cesium_library(CESIUM_NATIVE_SPZ spz)
find_cesium_library(CESIUM_NATIVE_ZSTD zstd)
find_cesium_library(CESIUM_NATIVE_CURL_LIB libcurl)
find_cesium_library(CESIUM_NATIVE_OPENSSL_CRYPTO libcrypto)
find_cesium_library(CESIUM_NATIVE_OPENSSL_SSL libssl)
find_cesium_library(CESIUM_NATIVE_BLEND2D blend2d)
find_cesium_library(CESIUM_NATIVE_ASMJIT asmjit)

# -- brotli (required by cpp-httplib) --
find_cesium_library(CESIUM_NATIVE_BROTLICOMMON brotlicommon)
find_cesium_library(CESIUM_NATIVE_BROTLIDEC brotlidec)
find_cesium_library(CESIUM_NATIVE_BROTLIENC brotlienc)

# -- abseil (required by s2geometry) --
find_cesium_library(CESIUM_NATIVE_ABSL_BASE absl_base)
find_cesium_library(CESIUM_NATIVE_ABSL_SPINLOCK absl_spinlock_wait)
find_cesium_library(CESIUM_NATIVE_ABSL_RAW_LOGGING absl_raw_logging_internal)
find_cesium_library(CESIUM_NATIVE_ABSL_THROW_DELEGATE absl_throw_delegate)
find_cesium_library(CESIUM_NATIVE_ABSL_INT128 absl_int128)
find_cesium_library(CESIUM_NATIVE_ABSL_STRINGS absl_strings)
find_cesium_library(CESIUM_NATIVE_ABSL_STRINGS_INTERNAL absl_strings_internal)
find_cesium_library(CESIUM_NATIVE_ABSL_STRING_VIEW absl_string_view)
find_cesium_library(CESIUM_NATIVE_ABSL_LOG_INTERNAL_MSG absl_log_internal_message)
find_cesium_library(CESIUM_NATIVE_ABSL_LOG_INTERNAL_CHECKOP absl_log_internal_check_op)
find_cesium_library(CESIUM_NATIVE_ABSL_LOG_INTERNAL_NULLGUARD absl_log_internal_nullguard)
find_cesium_library(CESIUM_NATIVE_ABSL_LOG_INTERNAL_LOG_SINK_SET absl_log_internal_log_sink_set)
find_cesium_library(CESIUM_NATIVE_ABSL_LOG_INTERNAL_FORMAT absl_log_internal_format)
find_cesium_library(CESIUM_NATIVE_ABSL_LOG_INTERNAL_GLOBALS absl_log_internal_globals)
find_cesium_library(CESIUM_NATIVE_ABSL_LOG_INTERNAL_PROTO absl_log_internal_proto)
find_cesium_library(CESIUM_NATIVE_ABSL_LOG_INTERNAL_FNMATCH absl_log_internal_fnmatch)
find_cesium_library(CESIUM_NATIVE_ABSL_LOG_INTERNAL_CONDITIONS absl_log_internal_conditions)
find_cesium_library(CESIUM_NATIVE_ABSL_LOG_GLOBALS absl_log_globals)
find_cesium_library(CESIUM_NATIVE_ABSL_LOG_SINK absl_log_sink)
find_cesium_library(CESIUM_NATIVE_ABSL_LOG_SEVERITY absl_log_severity)
find_cesium_library(CESIUM_NATIVE_ABSL_STRERROR absl_strerror)
find_cesium_library(CESIUM_NATIVE_ABSL_EXAMINE_STACK absl_examine_stack)
find_cesium_library(CESIUM_NATIVE_ABSL_STACKTRACE absl_stacktrace)
find_cesium_library(CESIUM_NATIVE_ABSL_SYMBOLIZE absl_symbolize)
find_cesium_library(CESIUM_NATIVE_ABSL_DEBUGGING_INTERNAL absl_debugging_internal)
find_cesium_library(CESIUM_NATIVE_ABSL_DEMANGLE_INTERNAL absl_demangle_internal)
find_cesium_library(CESIUM_NATIVE_ABSL_DEMANGLE_RUST absl_demangle_rust)
find_cesium_library(CESIUM_NATIVE_ABSL_DECODE_RUST_PUNYCODE absl_decode_rust_punycode)
find_cesium_library(CESIUM_NATIVE_ABSL_UTF8 absl_utf8_for_code_point)
find_cesium_library(CESIUM_NATIVE_ABSL_MALLOC_INTERNAL absl_malloc_internal)
find_cesium_library(CESIUM_NATIVE_ABSL_TIME absl_time)
find_cesium_library(CESIUM_NATIVE_ABSL_TIME_ZONE absl_time_zone)
find_cesium_library(CESIUM_NATIVE_ABSL_CIVIL_TIME absl_civil_time)
find_cesium_library(CESIUM_NATIVE_ABSL_HASH absl_hash)
find_cesium_library(CESIUM_NATIVE_ABSL_LOW_LEVEL_HASH absl_low_level_hash)
find_cesium_library(CESIUM_NATIVE_ABSL_RAW_HASH_SET absl_raw_hash_set)
find_cesium_library(CESIUM_NATIVE_ABSL_HASHTABLEZ_SAMPLER absl_hashtablez_sampler)
find_cesium_library(CESIUM_NATIVE_ABSL_STATUS absl_status)
find_cesium_library(CESIUM_NATIVE_ABSL_STATUSOR absl_statusor)
find_cesium_library(CESIUM_NATIVE_ABSL_CORD absl_cord)
find_cesium_library(CESIUM_NATIVE_ABSL_CORD_INTERNAL absl_cord_internal)
find_cesium_library(CESIUM_NATIVE_ABSL_CORDZ_FUNCTIONS absl_cordz_functions)
find_cesium_library(CESIUM_NATIVE_ABSL_CORDZ_HANDLE absl_cordz_handle)
find_cesium_library(CESIUM_NATIVE_ABSL_CORDZ_INFO absl_cordz_info)
find_cesium_library(CESIUM_NATIVE_ABSL_CORDZ_SAMPLE_TOKEN absl_cordz_sample_token)
find_cesium_library(CESIUM_NATIVE_ABSL_CRC32C absl_crc32c)
find_cesium_library(CESIUM_NATIVE_ABSL_CRC_INTERNAL absl_crc_internal)
find_cesium_library(CESIUM_NATIVE_ABSL_CRC_CPU_DETECT absl_crc_cpu_detect)
find_cesium_library(CESIUM_NATIVE_ABSL_CRC_CORD_STATE absl_crc_cord_state)
find_cesium_library(CESIUM_NATIVE_ABSL_STR_FORMAT absl_str_format_internal)
find_cesium_library(CESIUM_NATIVE_ABSL_SYNCHRONIZATION absl_synchronization)
find_cesium_library(CESIUM_NATIVE_ABSL_GRAPHCYCLES absl_graphcycles_internal)
find_cesium_library(CESIUM_NATIVE_ABSL_KERNEL_TIMEOUT absl_kernel_timeout_internal)
find_cesium_library(CESIUM_NATIVE_ABSL_EXPONENTIAL_BIASED absl_exponential_biased)
find_cesium_library(CESIUM_NATIVE_ABSL_PERIODIC_SAMPLER absl_periodic_sampler)
find_cesium_library(CESIUM_NATIVE_ABSL_CITY absl_city)
find_cesium_library(CESIUM_NATIVE_ABSL_VLOG_CONFIG absl_vlog_config_internal)
find_cesium_library(CESIUM_NATIVE_ABSL_TRACING absl_tracing_internal)
find_cesium_library(CESIUM_NATIVE_ABSL_DIE_IF_NULL absl_die_if_null)
find_cesium_library(CESIUM_NATIVE_ABSL_POISON absl_poison)
find_cesium_library(CESIUM_NATIVE_ABSL_LOG_INTERNAL_STRUCTURED_PROTO absl_log_internal_structured_proto)
find_cesium_library(CESIUM_NATIVE_ABSL_LEAK_CHECK absl_leak_check)
find_cesium_library(CESIUM_NATIVE_ABSL_SCOPED_SET_ENV absl_scoped_set_env)
find_cesium_library(CESIUM_NATIVE_ZLIB zlib)
find_cesium_library(CESIUM_NATIVE_JPEG jpeg)
find_cesium_library(CESIUM_NATIVE_WEBP libwebp)
find_cesium_library(CESIUM_NATIVE_SHARPYUV libsharpyuv)
find_cesium_library(CESIUM_NATIVE_ASTCENC astcenc-avx2-static)



if(NOT CESIUM_ANY_LIBRARY_MISSING)
    set(CESIUM_NATIVE_FOUND TRUE)

    # Assemble all the component libraries into one single import library:
    # https://stackoverflow.com/a/48397346/4218920
    
    add_library(OE::CESIUM_NATIVE INTERFACE IMPORTED)
    
    set_property(TARGET OE::CESIUM_NATIVE PROPERTY
        INTERFACE_LINK_LIBRARIES ${CESIUM_NATIVE_IMPORT_LIBRARIES} )

    if(WIN32)
        set_property(TARGET OE::CESIUM_NATIVE APPEND PROPERTY
            INTERFACE_LINK_LIBRARIES crypt32 ws2_32 wldap32 bcrypt)
    endif()
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(CesiumNative DEFAULT_MSG CESIUM_NATIVE_FOUND)

