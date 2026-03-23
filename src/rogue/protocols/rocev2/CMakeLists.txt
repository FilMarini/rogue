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

# Find libibverbs
find_library(IBVERBS_LIBRARY NAMES ibverbs
    HINTS /usr/lib /usr/lib64 /usr/local/lib)

if(NOT IBVERBS_LIBRARY)
    message(FATAL_ERROR "libibverbs not found. "
        "Install the rdma-core / libibverbs-dev package.")
endif()

find_path(IBVERBS_INCLUDE_DIR infiniband/verbs.h
    HINTS /usr/include /usr/local/include)

if(NOT IBVERBS_INCLUDE_DIR)
    message(FATAL_ERROR "infiniband/verbs.h not found. "
        "Install the rdma-core / libibverbs-dev package.")
endif()

message(STATUS "Found libibverbs: ${IBVERBS_LIBRARY}")
message(STATUS "Found ibverbs headers: ${IBVERBS_INCLUDE_DIR}")

target_sources(rogue-core PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}/Core.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/Server.cpp")

target_include_directories(rogue-core PRIVATE "${IBVERBS_INCLUDE_DIR}")
target_link_libraries(rogue-core "${IBVERBS_LIBRARY}")

if(NOT NO_PYTHON)
    target_sources(rogue-core PRIVATE "${CMAKE_CURRENT_LIST_DIR}/module.cpp")
endif()
