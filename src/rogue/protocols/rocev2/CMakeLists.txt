# ----------------------------------------------------------------------------
# Company    : SLAC National Accelerator Laboratory
# ----------------------------------------------------------------------------
# This file is part of the rogue software package. It is subject to
# the license terms in the LICENSE.txt file found in the top-level directory
# of this distribution and at:
#    https://confluence.slac.stanford.edu/display/ppareg/LICENSE.html.
# No part of the rogue software package, including this file, may be
# copied, modified, propagated, or distributed except according to the terms
# contained in the LICENSE.txt file.
# ----------------------------------------------------------------------------

# Pass -DNO_ROCEV2=1 to cmake to skip this module entirely.
if (NO_ROCEV2)
    message(STATUS "RoCEv2 support disabled (-DNO_ROCEV2=1)")
    return()
endif()

# -----------------------------------------------------------------------
# Find libibverbs.
# $CONDA_PREFIX is checked first; explicit system paths follow so the
# conda compiler toolchain does not hide them.
# -----------------------------------------------------------------------
set(_IBVERBS_LIB_HINTS "")
set(_IBVERBS_INC_HINTS "")

if (DEFINED ENV{CONDA_PREFIX})
    list(APPEND _IBVERBS_LIB_HINTS
        "$ENV{CONDA_PREFIX}/lib"
        "$ENV{CONDA_PREFIX}/lib64")
    list(APPEND _IBVERBS_INC_HINTS
        "$ENV{CONDA_PREFIX}/include")
endif()

list(APPEND _IBVERBS_LIB_HINTS
    /usr/lib /usr/lib64
    /usr/lib/x86_64-linux-gnu
    /usr/lib/aarch64-linux-gnu
    /usr/local/lib /usr/local/lib64)

list(APPEND _IBVERBS_INC_HINTS
    /usr/include /usr/local/include)

if (DEFINED IBVERBS_ROOT)
    list(INSERT _IBVERBS_LIB_HINTS 0 "${IBVERBS_ROOT}/lib" "${IBVERBS_ROOT}/lib64")
    list(INSERT _IBVERBS_INC_HINTS 0 "${IBVERBS_ROOT}/include")
endif()

find_library(IBVERBS_LIBRARY NAMES ibverbs
    HINTS ${_IBVERBS_LIB_HINTS} NO_DEFAULT_PATH)
if (NOT IBVERBS_LIBRARY)
    find_library(IBVERBS_LIBRARY NAMES ibverbs)
endif()

find_path(IBVERBS_INCLUDE_DIR NAMES infiniband/verbs.h
    HINTS ${_IBVERBS_INC_HINTS} NO_DEFAULT_PATH)
if (NOT IBVERBS_INCLUDE_DIR)
    find_path(IBVERBS_INCLUDE_DIR NAMES infiniband/verbs.h)
endif()

if (NOT IBVERBS_LIBRARY OR NOT IBVERBS_INCLUDE_DIR)
    message(STATUS "")
    message(STATUS "libibverbs not found — RoCEv2 support will be DISABLED.")
    message(STATUS "  conda         : conda install -c conda-forge rdma-core")
    message(STATUS "  Ubuntu/Debian : sudo apt-get install libibverbs-dev")
    message(STATUS "  Custom path   : cmake -DIBVERBS_ROOT=/path ...")
    message(STATUS "  Skip entirely : cmake -DNO_ROCEV2=1 ...")
    message(STATUS "")
    return()
endif()

message(STATUS "Found libibverbs   : ${IBVERBS_LIBRARY}")
message(STATUS "Found ibverbs hdrs : ${IBVERBS_INCLUDE_DIR}")

# -----------------------------------------------------------------------
# Add sources and include path to the rogue-core OBJECT target.
# -----------------------------------------------------------------------
target_sources(rogue-core PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}/Core.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/Server.cpp")

target_include_directories(rogue-core PRIVATE "${IBVERBS_INCLUDE_DIR}")

# -----------------------------------------------------------------------
# Propagate the ibverbs library path to the top-level CMakeLists so it
# can be linked into rogue-core-shared once that target is defined.
# (rogue-core-shared does not exist yet at this point in the cmake run.)
# This mirrors how BZIP2_LIBRARIES and ZeroMQ_LIBRARY are handled.
# -----------------------------------------------------------------------
set(IBVERBS_LIBRARY "${IBVERBS_LIBRARY}" PARENT_SCOPE)

if (NOT NO_PYTHON)
    target_sources(rogue-core PRIVATE "${CMAKE_CURRENT_LIST_DIR}/module.cpp")
endif()
