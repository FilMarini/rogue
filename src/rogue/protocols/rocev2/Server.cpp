/**
 * ----------------------------------------------------------------------------
 * Company    : SLAC National Accelerator Laboratory
 * ----------------------------------------------------------------------------
 * Description:
 * RoCEv2 RC Server — zero-copy receive path.
 *
 * Zero-copy design
 * ----------------
 * The slab is registered as a single MR.  On each CQ completion the
 * relevant slot is wrapped as a rogue Buffer using createBuffer() — this
 * records the existing pointer without any allocation or copy.  The Buffer
 * meta field carries the slot index in its lower 24 bits.
 *
 * When the last FramePtr downstream releases its reference, Buffer::~Buffer()
 * calls retBuffer() on this Pool subclass.  retBuffer() extracts the slot
 * index from meta and re-posts that slot to the QP — completing the cycle
 * without any data movement.
 *
 * This pattern is identical to AxiStreamDma's zero-copy path (see
 * AxiStreamDma.cpp retBuffer / createBuffer usage).
 * ----------------------------------------------------------------------------
 **/
#include "rogue/Directives.h"

#include "rogue/protocols/rocev2/Server.h"

#include <infiniband/verbs.h>
#include <stdint.h>
#include <string.h>

#include <cstdlib>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "rogue/GeneralError.h"
#include "rogue/GilRelease.h"
#include "rogue/Logging.h"
#include "rogue/interfaces/stream/Buffer.h"
#include "rogue/interfaces/stream/Frame.h"
#include "rogue/interfaces/stream/FrameLock.h"
#include "rogue/protocols/rocev2/Core.h"

namespace rpr = rogue::protocols::rocev2;
namespace ris = rogue::interfaces::stream;

#ifndef NO_PYTHON
    #include <boost/python.hpp>
namespace bp = boost::python;
#endif

// SSI Start-of-Frame bit set on every received frame
static const uint8_t SsiSof = 0x02;

// ---------------------------------------------------------------------------
// Factory
// ---------------------------------------------------------------------------
rpr::ServerPtr rpr::Server::create(const std::string& deviceName,
                                   uint8_t            ibPort,
                                   uint8_t            gidIndex,
                                   uint32_t           maxPayload,
                                   uint32_t           rxQueueDepth) {
    return std::make_shared<rpr::Server>(
        deviceName, ibPort, gidIndex, maxPayload, rxQueueDepth);
}

// ---------------------------------------------------------------------------
// Constructor
// ibverbs setup through QP INIT.  Receive thread starts in
// completeConnection() once the FPGA QPN is known.
// ---------------------------------------------------------------------------
rpr::Server::Server(const std::string& deviceName,
                    uint8_t            ibPort,
                    uint8_t            gidIndex,
                    uint32_t           maxPayload,
                    uint32_t           rxQueueDepth)
    : rpr::Core(deviceName, ibPort, gidIndex, maxPayload),
      ris::Master(),
      ris::Slave(),
      cq_(nullptr), qp_(nullptr), mr_(nullptr),
      slab_(nullptr),
      numBufs_(rxQueueDepth),
      thread_(nullptr),
      threadEn_(false) {

    log_ = rogue::Logging::create("rocev2.Server");
    memset(fpgaGid_, 0, 16);

    // -----------------------------------------------------------------------
    // 1. Allocate slab
    //    RC QPs do not prepend a GRH so each slot is exactly maxPayload_.
    // -----------------------------------------------------------------------
    bufSize_  = maxPayload_;
    slabSize_ = numBufs_ * bufSize_;

    slab_ = static_cast<uint8_t*>(aligned_alloc(4096, slabSize_));
    if (!slab_)
        throw(rogue::GeneralError::create("rocev2::Server::Server",
                                          "Failed to allocate RX slab (%u bytes)",
                                          slabSize_));
    memset(slab_, 0, slabSize_);

    // -----------------------------------------------------------------------
    // 2. Register slab as a single MR
    // -----------------------------------------------------------------------
    mr_ = ibv_reg_mr(pd_, slab_, slabSize_,
                     IBV_ACCESS_LOCAL_WRITE | IBV_ACCESS_REMOTE_WRITE);
    if (!mr_)
        throw(rogue::GeneralError::create("rocev2::Server::Server",
                                          "ibv_reg_mr failed"));

    mrAddr_ = reinterpret_cast<uint64_t>(slab_);
    mrRkey_ = mr_->rkey;

    log_->info("MR: addr=0x%016" PRIx64 " rkey=0x%08x size=%u",
               mrAddr_, mrRkey_, slabSize_);

    // -----------------------------------------------------------------------
    // 3. Create Completion Queue
    // -----------------------------------------------------------------------
    cq_ = ibv_create_cq(ctx_,
                         static_cast<int>(numBufs_),
                         nullptr, nullptr, 0);
    if (!cq_)
        throw(rogue::GeneralError::create("rocev2::Server::Server",
                                          "ibv_create_cq failed"));

    // -----------------------------------------------------------------------
    // 4. Create RC Queue Pair
    // -----------------------------------------------------------------------
    struct ibv_qp_init_attr qpAttr;
    memset(&qpAttr, 0, sizeof(qpAttr));
    qpAttr.qp_type          = IBV_QPT_RC;
    qpAttr.sq_sig_all       = 0;
    qpAttr.send_cq          = cq_;
    qpAttr.recv_cq          = cq_;
    qpAttr.cap.max_recv_wr  = numBufs_;
    qpAttr.cap.max_send_wr  = 1;
    qpAttr.cap.max_recv_sge = 1;
    qpAttr.cap.max_send_sge = 1;

    qp_ = ibv_create_qp(pd_, &qpAttr);
    if (!qp_)
        throw(rogue::GeneralError::create("rocev2::Server::Server",
                                          "ibv_create_qp (RC) failed"));

    hostQpn_ = qp_->qp_num;

    // -----------------------------------------------------------------------
    // 5. QP: RESET → INIT
    // -----------------------------------------------------------------------
    {
        struct ibv_qp_attr attr;
        memset(&attr, 0, sizeof(attr));
        attr.qp_state        = IBV_QPS_INIT;
        attr.pkey_index      = 0;
        attr.port_num        = ibPort_;
        attr.qp_access_flags = IBV_ACCESS_REMOTE_WRITE |
                               IBV_ACCESS_REMOTE_READ  |
                               IBV_ACCESS_LOCAL_WRITE;

        if (ibv_modify_qp(qp_, &attr,
                          IBV_QP_STATE      |
                          IBV_QP_PKEY_INDEX |
                          IBV_QP_PORT       |
                          IBV_QP_ACCESS_FLAGS))
            throw(rogue::GeneralError::create("rocev2::Server::Server",
                                              "QP RESET→INIT failed"));
    }

    // -----------------------------------------------------------------------
    // 6. Read host GID
    // -----------------------------------------------------------------------
    union ibv_gid gid;
    if (ibv_query_gid(ctx_, ibPort_, gidIndex_, &gid))
        throw(rogue::GeneralError::create("rocev2::Server::Server",
                                          "ibv_query_gid failed"));
    memcpy(hostGid_, gid.raw, 16);

    // -----------------------------------------------------------------------
    // 7. Random starting PSNs
    // -----------------------------------------------------------------------
    hostRqPsn_ = static_cast<uint32_t>(random()) & 0xFFFFFF;
    hostSqPsn_ = static_cast<uint32_t>(random()) & 0xFFFFFF;

    log_->info("RC QP ready: qpn=0x%06x rqPsn=0x%06x sqPsn=0x%06x",
               hostQpn_, hostRqPsn_, hostSqPsn_);
}

// ---------------------------------------------------------------------------
// setFpgaGid
// ---------------------------------------------------------------------------
void rpr::Server::setFpgaGid(const std::vector<uint8_t>& gidBytes) {
    if (gidBytes.size() != 16)
        throw(rogue::GeneralError::create("rocev2::Server::setFpgaGid",
                                          "GID must be 16 bytes, got %zu",
                                          gidBytes.size()));
    memcpy(fpgaGid_, gidBytes.data(), 16);
    log_->info("FPGA GID stored");
}

// ---------------------------------------------------------------------------
// completeConnection — finish handshake and start receive thread
// ---------------------------------------------------------------------------
void rpr::Server::completeConnection(uint32_t fpgaQpn, uint32_t fpgaRqPsn, uint32_t pmtu) {
    log_->info("completeConnection: fpgaQpn=0x%06x fpgaRqPsn=0x%06x",
               fpgaQpn, fpgaRqPsn);

    // QP: INIT → RTR
    {
        union ibv_gid dgid;
        memcpy(dgid.raw, fpgaGid_, 16);

        struct ibv_qp_attr attr;
        memset(&attr, 0, sizeof(attr));
        attr.qp_state              = IBV_QPS_RTR;
        attr.path_mtu              = static_cast<ibv_mtu>(pmtu);
        attr.dest_qp_num           = fpgaQpn;
        attr.rq_psn                = fpgaRqPsn;
        attr.max_dest_rd_atomic    = 16;
        attr.min_rnr_timer         = 1;
        attr.ah_attr.is_global     = 1;
        attr.ah_attr.grh.dgid      = dgid;
        attr.ah_attr.grh.sgid_index= gidIndex_;
        attr.ah_attr.grh.hop_limit = 64;
        attr.ah_attr.port_num      = ibPort_;
        attr.ah_attr.sl            = 0;

        if (ibv_modify_qp(qp_, &attr,
                          IBV_QP_STATE              |
                          IBV_QP_AV                 |
                          IBV_QP_PATH_MTU           |
                          IBV_QP_DEST_QPN           |
                          IBV_QP_RQ_PSN             |
                          IBV_QP_MAX_DEST_RD_ATOMIC |
                          IBV_QP_MIN_RNR_TIMER))
            throw(rogue::GeneralError::create("rocev2::Server::completeConnection",
                                              "QP INIT→RTR failed"));
    }

    log_->info("QP → RTR");

    // QP: RTR → RTS
    {
        struct ibv_qp_attr attr;
        memset(&attr, 0, sizeof(attr));
        attr.qp_state      = IBV_QPS_RTS;
        attr.sq_psn        = hostSqPsn_;
        attr.timeout       = 14;
        attr.retry_cnt     = 3;
        attr.rnr_retry     = 3;
        attr.max_rd_atomic = 16;

        if (ibv_modify_qp(qp_, &attr,
                          IBV_QP_STATE            |
                          IBV_QP_SQ_PSN           |
                          IBV_QP_TIMEOUT          |
                          IBV_QP_RETRY_CNT        |
                          IBV_QP_RNR_RETRY        |
                          IBV_QP_MAX_QP_RD_ATOMIC))
            throw(rogue::GeneralError::create("rocev2::Server::completeConnection",
                                              "QP RTR→RTS failed"));
    }

    log_->info("QP → RTS — ready to receive RDMA WRITEs");

    // Pre-post all receive WRs
    for (uint32_t i = 0; i < numBufs_; ++i) postRecvWr(i);

    // Start receive thread
    std::shared_ptr<int> scopePtr = std::make_shared<int>(0);
    threadEn_.store(true);
    thread_ = new std::thread(&rpr::Server::runThread, this,
                              std::weak_ptr<int>(scopePtr));

#ifndef __MACH__
    pthread_setname_np(thread_->native_handle(), "RoCEv2Server");
#endif
}

// ---------------------------------------------------------------------------
// getGid
// ---------------------------------------------------------------------------
std::string rpr::Server::getGid() const {
    std::ostringstream oss;
    for (int i = 0; i < 16; i += 2) {
        if (i) oss << ':';
        oss << std::hex << std::setfill('0')
            << std::setw(2) << static_cast<int>(hostGid_[i])
            << std::setw(2) << static_cast<int>(hostGid_[i+1]);
    }
    return oss.str();
}

// ---------------------------------------------------------------------------
// postRecvWr — post a receive WR for slot `slot`
// wr_id == slot index so no lookup is needed on completion
// ---------------------------------------------------------------------------
void rpr::Server::postRecvWr(uint32_t slot) {
    uint8_t* bufStart = slab_ + (static_cast<uint64_t>(slot) * bufSize_);

    struct ibv_sge sge;
    memset(&sge, 0, sizeof(sge));
    sge.addr   = reinterpret_cast<uint64_t>(bufStart);
    sge.length = bufSize_;
    sge.lkey   = mr_->lkey;

    struct ibv_recv_wr wr;
    memset(&wr, 0, sizeof(wr));
    wr.wr_id   = static_cast<uint64_t>(slot);
    wr.sg_list = &sge;
    wr.num_sge = 1;
    wr.next    = nullptr;

    struct ibv_recv_wr* bad = nullptr;
    if (ibv_post_recv(qp_, &wr, &bad))
        throw(rogue::GeneralError::create("rocev2::Server::postRecvWr",
                                          "ibv_post_recv failed (slot=%u)", slot));
}

// ---------------------------------------------------------------------------
// retBuffer — zero-copy hook
//
// Called by Buffer::~Buffer() when the last FramePtr holding this slot is
// released by downstream.  We re-post the slot to the QP so the FPGA can
// write into it again.
//
// meta lower 24 bits = slot index (set in createBuffer() call in runThread)
// ---------------------------------------------------------------------------
void rpr::Server::retBuffer(uint8_t* data, uint32_t meta, uint32_t rawSize) {
    // Extract slot index from lower 24 bits of meta
    uint32_t slot = meta & 0x00FFFFFF;

    log_->debug("retBuffer: re-posting slot=%u", slot);

    // Re-post the slot to the QP — no lock needed, ibv_post_recv is thread-safe
    // If the QP is already destroyed (during shutdown) just update counters
    if (threadEn_.load() && qp_) {
        try {
            postRecvWr(slot);
        } catch (...) {
            // Swallow errors during shutdown
        }
    }

    // Update pool accounting — mirrors AxiStreamDma::retBuffer pattern
    decCounter(rawSize);
}

// ---------------------------------------------------------------------------
// runThread — CQ polling loop (zero-copy version)
// ---------------------------------------------------------------------------
void rpr::Server::runThread(std::weak_ptr<int> lockPtr) {
    while (!lockPtr.expired()) continue;

    log_->logThreadId();
    log_->info("RoCEv2 receive thread started (zero-copy)");

    struct ibv_wc wc;

    while (threadEn_.load()) {
        int n = ibv_poll_cq(cq_, 1, &wc);
        if (n == 0)  continue;
        if (n < 0) { log_->warning("ibv_poll_cq error"); continue; }

        uint32_t slot = static_cast<uint32_t>(wc.wr_id);

        if (wc.status != IBV_WC_SUCCESS) {
            log_->warning("CQ error: %s (slot=%u)",
                          ibv_wc_status_str(wc.status), slot);
            // Re-post so we don't permanently lose a slot
            postRecvWr(slot);
            continue;
        }

        if (wc.opcode != IBV_WC_RECV_RDMA_WITH_IMM) {
            log_->warning("Unexpected opcode %d (slot=%u), re-posting",
                          wc.opcode, slot);
            postRecvWr(slot);
            continue;
        }

        // ------------------------------------------------------------------
        // Decode immediate value
        //   bits [7:0]  = channel id
        //   bits [31:8] = reserved
        // ------------------------------------------------------------------
        uint8_t  channel    = static_cast<uint8_t>(wc.imm_data & 0xFF);
        uint32_t payloadLen = wc.byte_len;

        if (payloadLen == 0 || payloadLen > bufSize_) {
            log_->warning("Bad payload len=%u (slot=%u), re-posting",
                          payloadLen, slot);
            postRecvWr(slot);
            continue;
        }

        // ------------------------------------------------------------------
        // Zero-copy: wrap the slab slot directly as a rogue Buffer.
        //
        // createBuffer() records the existing pointer — no allocation, no
        // copy.  The slot index is stored in the lower 24 bits of meta so
        // retBuffer() can re-post it when downstream is done.
        //
        // NOTE: we do NOT call postRecvWr() here.  retBuffer() does it
        // when the last downstream reference is released.
        // ------------------------------------------------------------------
        uint8_t* slotPtr = slab_ + (static_cast<uint64_t>(slot) * bufSize_);

        // meta = slot index in lower 24 bits
        ris::BufferPtr buff = createBuffer(slotPtr,
                                           slot & 0x00FFFFFF,
                                           payloadLen,
                                           bufSize_);
        buff->setPayload(payloadLen);

        ris::FramePtr frame = ris::Frame::create();
        frame->appendBuffer(buff);
        frame->setChannel(channel);
        frame->setFirstUser(SsiSof);
        frame->setLastUser(0);

        log_->debug("RX slot=%u channel=%u len=%u (zero-copy)",
                    slot, channel, payloadLen);

        // Push frame downstream — slot stays live until frame is released
        sendFrame(frame);
    }

    log_->info("RoCEv2 receive thread stopped");
}

// ---------------------------------------------------------------------------
// acceptFrame — TX not supported
// ---------------------------------------------------------------------------
void rpr::Server::acceptFrame(ris::FramePtr frame) {
    log_->warning("RoCEv2 Server::acceptFrame: TX not supported, dropping");
}

// ---------------------------------------------------------------------------
// stop / destructor
// ---------------------------------------------------------------------------
void rpr::Server::stop() {
    if (threadEn_.load()) {
        threadEn_.store(false);
        if (thread_) { thread_->join(); delete thread_; thread_ = nullptr; }
    }
    if (qp_)   { ibv_destroy_qp(qp_);  qp_   = nullptr; }
    if (cq_)   { ibv_destroy_cq(cq_);  cq_   = nullptr; }
    if (mr_)   { ibv_dereg_mr(mr_);    mr_   = nullptr; }
    if (slab_) { free(slab_);          slab_ = nullptr; }
}

rpr::Server::~Server() { this->stop(); }

// ---------------------------------------------------------------------------
// Python bindings
// ---------------------------------------------------------------------------
void rpr::Server::setup_python() {
#ifndef NO_PYTHON
    bp::class_<rpr::Server,
               rpr::ServerPtr,
               bp::bases<rpr::Core, ris::Master, ris::Slave>,
               boost::noncopyable>(
        "Server",
        bp::init<std::string, uint8_t, uint8_t, uint32_t, uint32_t>(
            (bp::arg("deviceName"),
             bp::arg("ibPort")        = 1,
             bp::arg("gidIndex")      = 0,
             bp::arg("maxPayload")    = rpr::DefaultMaxPayload,
             bp::arg("rxQueueDepth")  = rpr::DefaultRxQueueDepth)))
        .def("create",             &rpr::Server::create)
        .staticmethod("create")
        .def("setFpgaGid",         &rpr::Server::setFpgaGid)
        .def("completeConnection", &rpr::Server::completeConnection, (bp::arg("fpgaQpn"), bp::arg("fpgaRqPsn"), bp::arg("pmtu")=5))
        .def("getQpn",             &rpr::Server::getQpn)
        .def("getGid",             &rpr::Server::getGid)
        .def("getRqPsn",           &rpr::Server::getRqPsn)
        .def("getSqPsn",           &rpr::Server::getSqPsn)
        .def("getMrAddr",          &rpr::Server::getMrAddr)
        .def("getMrRkey",          &rpr::Server::getMrRkey)
        .def("stop",               &rpr::Server::stop);

    bp::implicitly_convertible<rpr::ServerPtr, rpr::CorePtr>();
    bp::implicitly_convertible<rpr::ServerPtr, ris::MasterPtr>();
    bp::implicitly_convertible<rpr::ServerPtr, ris::SlavePtr>();
#endif
}
