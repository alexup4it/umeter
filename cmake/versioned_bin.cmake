# Create versioned binary from params.h version info
# Called at build time with: -DPARAMS_H=... -DBIN_DIR=... -DPROJECT_NAME=...
file(STRINGS "${PARAMS_H}" PARAMS_LINES)
foreach(LINE ${PARAMS_LINES})
    if(LINE MATCHES "#define[ \t]+PARAMS_DEVICE_NAME[ \t]+([^ \t]+)")
        string(REPLACE "\"" "" DEVICE_NAME "${CMAKE_MATCH_1}")
    elseif(LINE MATCHES "#define[ \t]+PARAMS_FW_B1[ \t]+([0-9]+)")
        set(FW_B1 "${CMAKE_MATCH_1}")
    elseif(LINE MATCHES "#define[ \t]+PARAMS_FW_B2[ \t]+([0-9]+)")
        set(FW_B2 "${CMAKE_MATCH_1}")
    elseif(LINE MATCHES "#define[ \t]+PARAMS_FW_B3[ \t]+([0-9]+)")
        set(FW_B3 "${CMAKE_MATCH_1}")
    elseif(LINE MATCHES "#define[ \t]+PARAMS_FW_B4[ \t]+([0-9]+)")
        set(FW_B4 "${CMAKE_MATCH_1}")
    endif()
endforeach()
set(VERSIONED_BIN "${DEVICE_NAME}-${FW_B1}.${FW_B2}.${FW_B3}.${FW_B4}.bin")

# Remove old versioned binaries before copying the new one
file(GLOB OLD_BINS "${BIN_DIR}/*.bin")
foreach(OLD_BIN ${OLD_BINS})
    get_filename_component(FNAME "${OLD_BIN}" NAME)
    if(NOT FNAME STREQUAL "${PROJECT_NAME}.bin")
        file(REMOVE "${OLD_BIN}")
    endif()
endforeach()

execute_process(COMMAND "${CMAKE_COMMAND}" -E copy "${BIN_DIR}/${PROJECT_NAME}.bin" "${BIN_DIR}/${VERSIONED_BIN}")
message(STATUS "Versioned binary: ${VERSIONED_BIN}")
