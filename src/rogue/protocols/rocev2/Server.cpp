/**
 * ----------------------------------------------------------------------------
 * Company    : SLAC National Accelerator Laboratory
 * ----------------------------------------------------------------------------
 * Description:
 * RoCEv2 Server
 *
 * Receives RDMA WRITE-with-Immediate operations from an FPGA and forwards
 * each one as a rogue ris::Frame into the downstream stream pipeline.
 *
 * Immediate value layout (host byte order after ntohl, which ibverbs applies):
 *
 *  31       8  7        0
 *  ┌─────────┬──────────┐
 *  │reserved │ channel  │
 *  │  (24b)  │   (8b)   │
 *  └─────────┴──────────┘
 *
 * Receive flow:
 *   Constructor
 *     → registers the pool slab as an MR
 *     → creates a UD QP and transitions it to RTS
 *     → pre-posts rxQueueDepth Receive Work Requests (RWRs)
 *     → launches the CQ-polling thread
 *
 *   runThread (CQ polling loop)
 *     → ibv_poll_cq() returns IBV_WC_RECV_RDMA_WITH_IMM
 *     → looks up the FramePtr by wr_id
 *     → sets payload length from wc.byte_len
 *     → sets channel from imm_data[7:0]
 *     → sets firstUser = 0x2 (SSI SOF, mandatory for rogue protocols)
 *     → calls sendFrame() to push frame downstream
 *     → allocates a fresh frame and re-posts its buffer as a new RWR
 *
 * UD QP details:
 *   - Q-Key: 0x11111111  (must match FPGA firmware)
 *   - The FPGA performs RDMA WRITEs targeting this QP's receive buffers.
 *     UD RDMA WRITE-with-Immediate requires the target to have a posted RWR;
 *     the 40-byte UD Global Routing Header (GRH) is prepended by the HCA and
 *     is skipped when reading payload (first 40 bytes are GRH).
 * ----------------------------------------------------------------------------
 **/
#include "rogue/Directives.h"

#include "rogue/protocols/rocev2/Server.h"

#include <infiniband/verbs.h>
#include <stdint.h>

#include <cstring>
#include <memory>
#include <string>
#include <thread>

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

// UD GRH size prepended by the HCA on every UD receive
static const uint32_t GrhSize = 40;

// SSI Start-of-Frame bit in firstUser (bit 1)
static const uint8_t SsiSof = 0x02;

// Q-Key for the UD QP - must match FPGA firmware
static const uint32_t QKey = 0x11111111;

// ---------------------------------------------------------------------------
// Factory
// ---------------------------------------------------------------------------
rpr::ServerPtr rpr::Server::create(const std::string& deviceName,
                                   uint8_t            ibPort,
                                   uint8_t            gidIndex,
                                   uint32_t           maxPayload,
                                   uint32_t           rxQueueDepth) {
    rpr::ServerPtr r =
        std::make_shared<rpr::Server>(deviceName, ibPort, gidIndex, maxPayload, rxQueueDepth);
    return r;
}

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------
rpr::Server::Server(const std::string& deviceName,
                    uint8_t            ibPort,
                    uint8_t            gidIndex,
                    uint32_t           maxPayload,
                    uint32_t           rxQueueDepth)
    : rpr::Core(deviceName, ibPort, gidIndex, maxPayload),
      ris::Master(),
      ris::Slave(),
      threadEn_(false),
      nextWrId_(0) {

    log_ = rogue::Logging::create("rocev2.Server");

    // -----------------------------------------------------------------------
    // 1. Configure the rogue pool.
    //    Each buffer must hold the GRH (40 B) plus the maximum payload.
    // -----------------------------------------------------------------------
    uint32_t bufSize = GrhSize + maxPayload_;
    setFixedSize(bufSize);
    setPoolSize(rxQueueDepth * 4);  // keep 4x headroom in the pool

    // -----------------------------------------------------------------------
    // 2. Register the pool memory with ibverbs.
    //    We register a representative block here.  Because rogue's Pool
    //    allocates individual buffers via malloc(), we register each buffer
    //    individually when we post the RWR (see postRecvWr).  The MR pointer
    //    stored in Core (mr_) is therefore per-buffer and managed in postRecvWr.
    //    We set Core::mr_ = nullptr to signal "per-buffer registration" mode.
    // -----------------------------------------------------------------------
    mr_ = nullptr;  // per-buffer MRs are managed in postRecvWr / runThread

    // -----------------------------------------------------------------------
    // 3. Create Completion Queue
    //    Size = rxQueueDepth completions.  We poll it manually so no
    //    completion channel is needed.
    // -----------------------------------------------------------------------
    cq_ = ibv_create_cq(ctx_,
                         static_cast<int>(rxQueueDepth),
                         nullptr,   // cq_context
                         nullptr,   // completion channel (polling mode)
                         0);        // comp_vector
    if (!cq_)
        throw(rogue::GeneralError::create("rocev2::Server::Server",
                                          "Failed to create completion queue"));

    // -----------------------------------------------------------------------
    // 4. Create Unreliable Datagram Queue Pair
    // -----------------------------------------------------------------------
    struct ibv_qp_init_attr qpAttr;
    memset(&qpAttr, 0, sizeof(qpAttr));
    qpAttr.qp_type          = IBV_QPT_UD;
    qpAttr.sq_sig_all        = 0;
    qpAttr.send_cq           = cq_;   // not used for RX-only, but required
    qpAttr.recv_cq           = cq_;
    qpAttr.cap.max_recv_wr   = rxQueueDepth;
    qpAttr.cap.max_send_wr   = 1;     // TX not used; minimal
    qpAttr.cap.max_recv_sge  = 1;
    qpAttr.cap.max_send_sge  = 1;

    qp_ = ibv_create_qp(pd_, &qpAttr);
    if (!qp_)
        throw(rogue::GeneralError::create("rocev2::Server::Server",
                                          "Failed to create queue pair"));

    // -----------------------------------------------------------------------
    // 5. Transition QP: RESET → INIT → RTR → RTS
    // -----------------------------------------------------------------------

    // RESET → INIT
    {
        struct ibv_qp_attr attr;
        memset(&attr, 0, sizeof(attr));
        attr.qp_state   = IBV_QPS_INIT;
        attr.pkey_index = 0;
        attr.port_num   = ibPort_;
        attr.qkey       = QKey;

        if (ibv_modify_qp(qp_, &attr,
                          IBV_QP_STATE | IBV_QP_PKEY_INDEX |
                          IBV_QP_PORT  | IBV_QP_QKEY))
            throw(rogue::GeneralError::create("rocev2::Server::Server",
                                              "QP RESET→INIT transition failed"));
    }

    // INIT → RTR
    {
        struct ibv_qp_attr attr;
        memset(&attr, 0, sizeof(attr));
        attr.qp_state = IBV_QPS_RTR;

        if (ibv_modify_qp(qp_, &attr, IBV_QP_STATE))
            throw(rogue::GeneralError::create("rocev2::Server::Server",
                                              "QP INIT→RTR transition failed"));
    }

    // RTR → RTS
    {
        struct ibv_qp_attr attr;
        memset(&attr, 0, sizeof(attr));
        attr.qp_state  = IBV_QPS_RTS;
        attr.sq_psn    = 0;

        if (ibv_modify_qp(qp_, &attr, IBV_QP_STATE | IBV_QP_SQ_PSN))
            throw(rogue::GeneralError::create("rocev2::Server::Server",
                                              "QP RTR→RTS transition failed"));
    }

    log_->info("RoCEv2 Server QP number: 0x%06x  Q-Key: 0x%08x  port: %u  GID idx: %u",
               qp_->qp_num, QKey, ibPort_, gidIndex_);

    // -----------------------------------------------------------------------
    // 6. Pre-post rxQueueDepth receive work requests
    // -----------------------------------------------------------------------
    for (uint32_t i = 0; i < rxQueueDepth; ++i) {
        ris::FramePtr frame = reqLocalFrame(bufSize, false);
        postRecvWr(frame);
    }

    // -----------------------------------------------------------------------
    // 7. Start the receive thread
    // -----------------------------------------------------------------------
    std::shared_ptr<int> scopePtr = std::make_shared<int>(0);
    threadEn_ = true;
    thread_   = new std::thread(&rpr::Server::runThread, this,
                                std::weak_ptr<int>(scopePtr));

#ifndef __MACH__
    pthread_setname_np(thread_->native_handle(), "RoCEv2Server");
#endif
}

// ---------------------------------------------------------------------------
// Destructor / stop
// ---------------------------------------------------------------------------
rpr::Server::~Server() {
    this->stop();
}

void rpr::Server::stop() {
    if (threadEn_) {
        threadEn_ = false;
        thread_->join();
        delete thread_;
        thread_ = nullptr;
    }

    // Tear down ibverbs resources in reverse order
    if (qp_) { ibv_destroy_qp(qp_);   qp_ = nullptr; }
    if (cq_) { ibv_destroy_cq(cq_);   cq_ = nullptr; }

    // Deregister any MRs that are still in the wr map
    {
        std::lock_guard<std::mutex> lock(wrMapMtx_);
        for (auto& kv : wrMap_) {
            // The MR laddr is stored as wr_id; retrieve the mr pointer
            // from the SGE lkey lookup is not straightforward here.
            // Instead we store MRs in a parallel map - see postRecvWr.
        }
        wrMap_.clear();
    }
}

// ---------------------------------------------------------------------------
// Post a single receive work request
// ---------------------------------------------------------------------------
void rpr::Server::postRecvWr(ris::FramePtr frame) {
    ris::BufferPtr buff = *(frame->beginBuffer());

    // Register this buffer's memory with the HCA.
    // We use IBV_ACCESS_LOCAL_WRITE so the HCA can write incoming data into it.
    struct ibv_mr* mr = ibv_reg_mr(pd_,
                                   buff->begin(),
                                   buff->getAvailable(),
                                   IBV_ACCESS_LOCAL_WRITE);
    if (!mr)
        throw(rogue::GeneralError::create("rocev2::Server::postRecvWr",
                                          "ibv_reg_mr failed"));

    // Scatter/Gather entry pointing at the full buffer
    struct ibv_sge sge;
    memset(&sge, 0, sizeof(sge));
    sge.addr   = reinterpret_cast<uint64_t>(buff->begin());
    sge.length = buff->getAvailable();
    sge.lkey   = mr->lkey;

    // Work Request
    struct ibv_recv_wr wr;
    memset(&wr, 0, sizeof(wr));

    uint64_t wrId = nextWrId_++;
    wr.wr_id   = wrId;
    wr.sg_list = &sge;
    wr.num_sge = 1;
    wr.next    = nullptr;

    // Store the frame and MR so runThread can retrieve them on completion
    {
        std::lock_guard<std::mutex> lock(wrMapMtx_);
        wrMap_[wrId] = frame;
        mrMap_[wrId] = mr;
    }

    struct ibv_recv_wr* bad = nullptr;
    if (ibv_post_recv(qp_, &wr, &bad))
        throw(rogue::GeneralError::create("rocev2::Server::postRecvWr",
                                          "ibv_post_recv failed"));

    log_->debug("Posted RWR id=%" PRIu64, wrId);
}

// ---------------------------------------------------------------------------
// Receive thread - polls the CQ
// ---------------------------------------------------------------------------
void rpr::Server::runThread(std::weak_ptr<int> lockPtr) {
    // Wait for the constructor to finish (scopePtr goes out of scope)
    while (!lockPtr.expired()) continue;

    log_->logThreadId();
    log_->info("RoCEv2 receive thread started");

    struct ibv_wc wc;

    while (threadEn_) {
        // Busy-poll the completion queue.
        // For lower CPU usage you can replace this with ibv_get_cq_event()
        // using a completion channel - at the cost of higher latency.
        int n = ibv_poll_cq(cq_, 1, &wc);

        if (n == 0) continue;

        if (n < 0) {
            log_->warning("ibv_poll_cq returned error");
            continue;
        }

        // ------------------------------------------------------------------
        // Retrieve the pre-posted frame associated with this completion
        // ------------------------------------------------------------------
        ris::FramePtr frame;
        struct ibv_mr* mr = nullptr;
        {
            std::lock_guard<std::mutex> lock(wrMapMtx_);
            auto it = wrMap_.find(wc.wr_id);
            if (it == wrMap_.end()) {
                log_->warning("Completion for unknown wr_id=%" PRIu64, wc.wr_id);
                continue;
            }
            frame = it->second;
            wrMap_.erase(it);

            // Retrieve companion MR (see mrMap_ parallel map)
            auto mit = mrMap_.find(wc.wr_id);
            if (mit != mrMap_.end()) {
                mr = mit->second;
                mrMap_.erase(mit);
            }
        }

        // ------------------------------------------------------------------
        // Handle errors
        // ------------------------------------------------------------------
        if (wc.status != IBV_WC_SUCCESS) {
            log_->warning("CQ completion error: %s (wr_id=%" PRIu64 ")",
                          ibv_wc_status_str(wc.status), wc.wr_id);

            // Deregister the MR and re-post a fresh buffer
            if (mr) ibv_dereg_mr(mr);
            ris::FramePtr fresh = reqLocalFrame(GrhSize + maxPayload_, false);
            postRecvWr(fresh);
            continue;
        }

        // ------------------------------------------------------------------
        // We only expect RDMA WRITE-with-Immediate completions
        // ------------------------------------------------------------------
        if (wc.opcode != IBV_WC_RECV_RDMA_WITH_IMM) {
            log_->warning("Unexpected opcode %d, dropping", wc.opcode);
            if (mr) ibv_dereg_mr(mr);
            ris::FramePtr fresh = reqLocalFrame(GrhSize + maxPayload_, false);
            postRecvWr(fresh);
            continue;
        }

        // ------------------------------------------------------------------
        // Decode the immediate value
        //
        //  bits [7:0]  = channel id
        //  bits [31:8] = reserved
        //
        // ibverbs delivers imm_data in host byte order (already ntohl'd).
        // ------------------------------------------------------------------
        uint8_t channel = static_cast<uint8_t>(wc.imm_data & 0xFF);

        // ------------------------------------------------------------------
        // The UD GRH (40 bytes) is prepended by the HCA.  Actual payload
        // starts at offset 40.  wc.byte_len includes the GRH.
        // ------------------------------------------------------------------
        uint32_t totalLen   = wc.byte_len;
        uint32_t payloadLen = (totalLen > GrhSize) ? (totalLen - GrhSize) : 0;

        if (payloadLen == 0) {
            log_->warning("Zero-length payload after GRH strip, dropping");
            if (mr) ibv_dereg_mr(mr);
            ris::FramePtr fresh = reqLocalFrame(GrhSize + maxPayload_, false);
            postRecvWr(fresh);
            continue;
        }

        // ------------------------------------------------------------------
        // Adjust the frame: skip GRH, set payload length
        // ------------------------------------------------------------------
        ris::BufferPtr buff = *(frame->beginBuffer());

        // Move the buffer start pointer past the GRH.
        // rogue Buffer tracks payload via setPayload(); we shift the begin
        // pointer by adjusting the header size (which rogue calls "head room").
        buff->setHeadRoom(GrhSize);
        buff->setPayload(payloadLen);

        // Set rogue stream metadata
        frame->setChannel(channel);
        frame->setFirstUser(SsiSof);   // SSI SOF on every frame
        frame->setLastUser(0);

        log_->debug("RX frame: channel=%" PRIu8 " len=%" PRIu32, channel, payloadLen);

        // ------------------------------------------------------------------
        // Push the frame into the rogue pipeline
        // ------------------------------------------------------------------
        sendFrame(frame);

        // ------------------------------------------------------------------
        // Deregister the consumed MR and post a fresh buffer in its place
        // ------------------------------------------------------------------
        if (mr) ibv_dereg_mr(mr);

        ris::FramePtr fresh = reqLocalFrame(GrhSize + maxPayload_, false);
        postRecvWr(fresh);
    }

    log_->info("RoCEv2 receive thread stopped");
}

// ---------------------------------------------------------------------------
// acceptFrame - TX path (not implemented for RX-only server)
// ---------------------------------------------------------------------------
void rpr::Server::acceptFrame(ris::FramePtr frame) {
    log_->warning("RoCEv2 Server::acceptFrame called but TX is not supported. "
                  "Dropping frame.");
}

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
             bp::arg("ibPort")      = 1,
             bp::arg("gidIndex")    = 0,
             bp::arg("maxPayload")  = rpr::DefaultMaxPayload,
             bp::arg("rxQueueDepth") = rpr::DefaultRxQueueDepth)))
        .def("create",   &rpr::Server::create)
        .staticmethod("create");

    bp::implicitly_convertible<rpr::ServerPtr, rpr::CorePtr>();
    bp::implicitly_convertible<rpr::ServerPtr, ris::MasterPtr>();
    bp::implicitly_convertible<rpr::ServerPtr, ris::SlavePtr>();
#endif
}
