# Generate gitcommit.h with current git commit hash
execute_process(
        COMMAND ${GIT_EXECUTABLE} log --pretty=format:%H -n 1
        OUTPUT_VARIABLE GIT_HASH
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET
)

if(GIT_HASH)
    file(WRITE ${OUTPUT_FILE} "#define GIT_COMMIT_HASH \"${GIT_HASH}\"\n")
else()
    file(WRITE ${OUTPUT_FILE} "#define GIT_COMMIT_HASH \"unknown\"\n")
endif()
