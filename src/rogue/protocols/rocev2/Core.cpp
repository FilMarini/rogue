/**
 * ----------------------------------------------------------------------------
 * Company    : SLAC National Accelerator Laboratory
 * ----------------------------------------------------------------------------
 * Description:
 * RoCEv2 Core - opens the ibverbs device and creates the shared PD/CQ/QP
 * resources that Server builds on top of.
 *
 * Queue Pair type: IBV_QPT_UD (Unreliable Datagram).
 * This matches the RoCEv2 FPGA engine which performs RDMA WRITEs to a UD QP.
 * The GID/port/Q-Key must match what is programmed in the FPGA firmware.
 * ----------------------------------------------------------------------------
 **/
#include "rogue/Directives.h"

#include "rogue/protocols/rocev2/Core.h"

#include <infiniband/verbs.h>
#include <stdint.h>

#include <cstring>
#include <string>

#include "rogue/GeneralError.h"

namespace rpr = rogue::protocols::rocev2;

#ifndef NO_PYTHON
    #include <boost/python.hpp>
namespace bp = boost::python;
#endif

//! Creator
rpr::Core::Core(const std::string& deviceName,
                uint8_t            ibPort,
                uint8_t            gidIndex,
                uint32_t           maxPayload)
    : deviceName_(deviceName),
      ibPort_(ibPort),
      gidIndex_(gidIndex),
      maxPayload_(maxPayload),
      ctx_(nullptr),
      pd_(nullptr),
      cq_(nullptr),
      qp_(nullptr),
      mr_(nullptr) {

    log_ = rogue::Logging::create("rocev2.Core");

    // -----------------------------------------------------------------------
    // 1. Find and open the requested ibverbs device
    // -----------------------------------------------------------------------
    int numDevices = 0;
    struct ibv_device** devList = ibv_get_device_list(&numDevices);
    if (!devList || numDevices == 0)
        throw(rogue::GeneralError::create("rocev2::Core::Core",
                                          "No RDMA devices found on this host"));

    struct ibv_device* dev = nullptr;
    for (int i = 0; i < numDevices; ++i) {
        if (deviceName_ == ibv_get_device_name(devList[i])) {
            dev = devList[i];
            break;
        }
    }

    if (!dev) {
        ibv_free_device_list(devList);
        throw(rogue::GeneralError::create("rocev2::Core::Core",
                                          "RDMA device '%s' not found",
                                          deviceName_.c_str()));
    }

    ctx_ = ibv_open_device(dev);
    ibv_free_device_list(devList);

    if (!ctx_)
        throw(rogue::GeneralError::create("rocev2::Core::Core",
                                          "Failed to open RDMA device '%s'",
                                          deviceName_.c_str()));

    // -----------------------------------------------------------------------
    // 2. Allocate Protection Domain
    // -----------------------------------------------------------------------
    pd_ = ibv_alloc_pd(ctx_);
    if (!pd_)
        throw(rogue::GeneralError::create("rocev2::Core::Core",
                                          "Failed to allocate protection domain"));

    log_->info("Opened RoCEv2 device '%s', port %u, GID index %u",
               deviceName_.c_str(), ibPort_, gidIndex_);
}

//! Destructor - resources are torn down by Server after the thread stops
rpr::Core::~Core() {
    // QP, CQ and MR are owned by Server; it tears them down before calling
    // the base destructor.  We only clean up PD and context here.
    if (pd_)  ibv_dealloc_pd(pd_);
    if (ctx_) ibv_close_device(ctx_);
}

void rpr::Core::setup_python() {
#ifndef NO_PYTHON
    bp::class_<rpr::Core, rpr::CorePtr, boost::noncopyable>("Core", bp::no_init)
        .def("maxPayload", &rpr::Core::maxPayload);
#endif
}
