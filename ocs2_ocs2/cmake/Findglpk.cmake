#
# Module that finds glpk, installed by apt-get install libglpk-dev
#
# Sets the follwing variable:
#
# GLPK_FOUND           True if GLPK available and usable.
# GLPK_INCLUDE_DIRS    Path to the GLPK include dirs.
# GLPK_LIBRARIES       Name to the GLPK library.
#

find_path(glpk_INCLUDE_DIR
        NAMES glpk.h
        HINTS /usr/include
        )

find_library(glpk_LIBRARY
        NAMES glpk
        HINTS /usr/lib /usr/lib/x86_64-linux-gnu
        )

if (glpk_INCLUDE_DIR AND glpk_LIBRARY)
    set(glpk_FOUND TRUE)
endif (glpk_INCLUDE_DIR AND glpk_LIBRARY)

if (glpk_FOUND)
    if (NOT glpk_FOUND_QUIETLY)
        MESSAGE(STATUS "Looking for glpk... - found ${glpk_LIBRARIES}")
        SET(LD_LIBRARY_PATH ${LD_LIBRARY_PATH} ${glpk_LIBRARIES})
    endif (NOT glpk_FOUND_QUIETLY)
else (glpk_FOUND)
    if (glpk_FOUND_REQUIRED)
        message(FATAL_ERROR "Looking for glpk... - Not found")
    endif (glpk_FOUND_REQUIRED)
endif (glpk_FOUND)

mark_as_advanced(glpk_FOUND glpk_INCLUDE_DIR glpk_LIBRARY )

set(glpk_LIBRARIES ${glpk_LIBRARY} )
set(glpk_INCLUDE_DIRS ${glpk_INCLUDE_DIR} )


