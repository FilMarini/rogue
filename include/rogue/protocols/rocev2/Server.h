/**
 * ----------------------------------------------------------------------------
 * Company    : SLAC National Accelerator Laboratory
 * ----------------------------------------------------------------------------
 * Description:
 * RoCEv2 RC Server — zero-copy receive path.
 *
 * Each slab slot is wrapped directly as a rogue Buffer and handed to
 * downstream without any memcpy.  The slot is re-posted to the QP only
 * when the last FramePtr reference is released by downstream, via the
 * overridden retBuffer() hook.
 *
 * Connection sequence
 * -------------------
 *   Step 1 — Constructor
 *     Allocates slab, registers MR, creates RC QP, transitions to INIT.
 *     Exposes getters for Python to read host QP parameters.
 *
 *   Step 2 — Python (_RoCEv2.py)
 *     Configures FPGA engine via AXI-lite, retrieves FPGA QPN.
 *
 *   Step 3 — setFpgaGid(gidBytes)
 *     Stores FPGA GID derived from IP address.
 *
 *   Step 4 — completeConnection(fpgaQpn, fpgaRqPsn)
 *     Transitions host QP INIT→RTR→RTS, pre-posts all receive WRs,
 *     starts the CQ-polling receive thread.
 *
 * Zero-copy mechanism
 * -------------------
 *   On CQ completion:
 *     createBuffer(slotPtr, slotIndex, ...)  — wraps slab memory, no copy
 *     sendFrame(frame)                       — downstream holds FramePtr ref
 *
 *   On last FramePtr release (downstream done):
 *     Buffer::~Buffer() → retBuffer(data, meta=slotIndex, ...)
 *     retBuffer() → postRecvWr(slotIndex)   — re-posts slot to QP
 *
 * Flow control
 * ------------
 *   Slots are only re-posted when downstream releases frames.  If all
 *   rxQueueDepth slots are held by downstream the FPGA receives RNR NAKs
 *   and retries.  Size rxQueueDepth to cover your downstream latency.
 *
 * Immediate value format (bits):
 *   [7:0]  = channel id
 *   [31:8] = reserved
 * ----------------------------------------------------------------------------
 **/

#ifndef ROGUE_PROTOCOLS_ROCEV2_SERVER_H
#define ROGUE_PROTOCOLS_ROCEV2_SERVER_H

#include <infiniband/verbs.h>
#include <stdint.h>

#include <atomic>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "rogue/interfaces/stream/Frame.h"
#include "rogue/interfaces/stream/Master.h"
#include "rogue/interfaces/stream/Pool.h"
#include "rogue/interfaces/stream/Slave.h"
#include "rogue/protocols/rocev2/Core.h"

namespace rogue {
namespace protocols {
namespace rocev2 {

class Server : public rogue::protocols::rocev2::Core,
               public rogue::interfaces::stream::Master,
               public rogue::interfaces::stream::Slave {
  private:
    // -----------------------------------------------------------------------
    // ibverbs resources
    // -----------------------------------------------------------------------
    struct ibv_cq* cq_;
    struct ibv_qp* qp_;
    struct ibv_mr* mr_;

    // Contiguous slab — subdivided into numBufs_ fixed-size slots.
    // Registered with the HCA once; never copied from.
    uint8_t*  slab_;
    uint32_t  slabSize_;
    uint32_t  numBufs_;
    uint32_t  bufSize_;       // == maxPayload_ (no GRH on RC)

    // -----------------------------------------------------------------------
    // Host-side RC parameters
    // -----------------------------------------------------------------------
    uint32_t hostQpn_;
    uint8_t  hostGid_[16];
    uint32_t hostRqPsn_;
    uint32_t hostSqPsn_;
    uint64_t mrAddr_;
    uint32_t mrRkey_;

    // FPGA GID — set by setFpgaGid() before completeConnection()
    uint8_t  fpgaGid_[16];

    // -----------------------------------------------------------------------
    // Receive thread
    // -----------------------------------------------------------------------
    std::thread*      thread_;
    std::atomic<bool> threadEn_;

    std::shared_ptr<rogue::Logging> log_;

    void postRecvWr(uint32_t slot);
    void runThread(std::weak_ptr<int> lockPtr);

  protected:
    // -----------------------------------------------------------------------
    // Zero-copy hook — called by Buffer::~Buffer() when the last downstream
    // reference to a frame is released.  Re-posts the slab slot to the QP.
    // The slot index is encoded in the lower 24 bits of meta.
    // -----------------------------------------------------------------------
    void retBuffer(uint8_t* data, uint32_t meta, uint32_t rawSize) override;

  public:
    static std::shared_ptr<rogue::protocols::rocev2::Server> create(
        const std::string& deviceName,
        uint8_t            ibPort,
        uint8_t            gidIndex,
        uint32_t           maxPayload   = DefaultMaxPayload,
        uint32_t           rxQueueDepth = DefaultRxQueueDepth);

    Server(const std::string& deviceName,
           uint8_t            ibPort,
           uint8_t            gidIndex,
           uint32_t           maxPayload,
           uint32_t           rxQueueDepth);

    ~Server();
    void stop();

    void setFpgaGid(const std::string& gidBytes);
    void completeConnection(uint32_t fpgaQpn, uint32_t fpgaRqPsn, uint32_t pmtu = 5);

    uint32_t    getQpn()    const { return hostQpn_; }
    std::string getGid()    const;
    uint32_t    getRqPsn()  const { return hostRqPsn_; }
    uint32_t    getSqPsn()  const { return hostSqPsn_; }
    uint64_t    getMrAddr() const { return mrAddr_; }
    uint32_t    getMrRkey() const { return mrRkey_; }

    void acceptFrame(rogue::interfaces::stream::FramePtr frame) override;

    static void setup_python();
};

typedef std::shared_ptr<rogue::protocols::rocev2::Server> ServerPtr;

}  // namespace rocev2
}  // namespace protocols
}  // namespace rogue

#endif
