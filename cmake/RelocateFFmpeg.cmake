# cmake-lint does not yet recognize READ_ELF and RPATH_CHANGE.
# cmake-lint: disable=E1126
# Keep dependencies replaceable after relocating an installed SDK.
foreach(library avformat avcodec swscale swresample avutil)
    file(GLOB libraries "${PREFIX}/lib/lib${library}.so.*")
    foreach(path IN LISTS libraries)
        if(NOT IS_SYMLINK "${path}")
            file(READ_ELF "${path}" RPATH rpath RUNPATH runpath)
            if(runpath)
                set(rpath "${runpath}")
            endif()
            if(rpath AND NOT rpath STREQUAL "$ORIGIN")
                file(
                    RPATH_CHANGE
                    FILE
                    "${path}"
                    OLD_RPATH
                    "${rpath}"
                    NEW_RPATH
                    "$ORIGIN")
            endif()
        endif()
    endforeach()
endforeach()
