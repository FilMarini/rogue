/**
 * ----------------------------------------------------------------------------
 * Company    : SLAC National Accelerator Laboratory
 * ----------------------------------------------------------------------------
 * Description:
 * RoCEv2 RC Server
 *
 * Connection sequence
 * -------------------
 *   Step 1 — Constructor
 *     ibv_reg_mr()      → mrAddr_, mrRkey_
 *     ibv_create_qp(RC) → hostQpn_
 *     QP RESET → INIT
 *     Exposes getters so Python can read hostQpn_, getGid(),
 *     hostRqPsn_, hostSqPsn_, mrAddr_, mrRkey_.
 *
 *   Step 2 — Python (_RoCEv2.py / RoceEngine.setup_connection)
 *     Writes host parameters to FPGA via AXI-lite metadata bus.
 *     Reads back fpgaQpn from FPGA QP-create response.
 *
 *   Step 3 — setFpgaGid(gid_bytes)   [called by Python before step 4]
 *     Stores the 16-byte FPGA GID derived from the IP address.
 *
 *   Step 4 — completeConnection(fpgaQpn, fpgaRqPsn)
 *     QP INIT → RTR  (uses stored FPGA GID as dgid in ah_attr)
 *     QP RTR  → RTS
 *     Starts the CQ-polling receive thread.
 *
 * Immediate-value format (agreed with FPGA firmware):
 *   bits [7:0]  = channel id
 *   bits [31:8] = reserved
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
    struct ibv_mr* mr_;       // single MR covering the whole RX slab

    // Contiguous RX slab — subdivided into numBufs_ fixed-size slots
    uint8_t*  slab_;
    uint32_t  slabSize_;
    uint32_t  numBufs_;
    uint32_t  bufSize_;       // == maxPayload_ (no GRH on RC)

    // -----------------------------------------------------------------------
    // Host-side RC parameters — exposed via getters
    // -----------------------------------------------------------------------
    uint32_t hostQpn_;
    uint8_t  hostGid_[16];    // 128-bit GID of this host NIC
    uint32_t hostRqPsn_;      // starting receive PSN advertised to FPGA
    uint32_t hostSqPsn_;      // starting send PSN
    uint64_t mrAddr_;         // virtual address of slab (FPGA writes here)
    uint32_t mrRkey_;         // rkey the FPGA must use

    // -----------------------------------------------------------------------
    // FPGA-side GID — set by Python before completeConnection()
    // -----------------------------------------------------------------------
    uint8_t  fpgaGid_[16];    // derived from FPGA IP address

    // -----------------------------------------------------------------------
    // Receive thread
    // -----------------------------------------------------------------------
    std::thread*      thread_;
    std::atomic<bool> threadEn_;
    uint32_t          nextSlot_;

    std::shared_ptr<rogue::Logging> log_;

    void postRecvWr(uint32_t slot);
    void runThread(std::weak_ptr<int> lockPtr);

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

    // Store the 16-byte FPGA GID (called by Python, derived from FPGA IP)
    void setFpgaGid(const std::vector<uint8_t>& gidBytes);

    // Finish host QP handshake and start RX thread (called by Python)
    void completeConnection(uint32_t fpgaQpn, uint32_t fpgaRqPsn);

    // Getters — Python reads these to configure the FPGA engine
    uint32_t    getQpn()    const { return hostQpn_; }
    std::string getGid()    const;   // formatted as "xxxx:xxxx:..." string
    uint32_t    getRqPsn()  const { return hostRqPsn_; }
    uint32_t    getSqPsn()  const { return hostSqPsn_; }
    uint64_t    getMrAddr() const { return mrAddr_; }
    uint32_t    getMrRkey() const { return mrRkey_; }

    // TX path not supported
    void acceptFrame(rogue::interfaces::stream::FramePtr frame) override;

    static void setup_python();
};

typedef std::shared_ptr<rogue::protocols::rocev2::Server> ServerPtr;

}  // namespace rocev2
}  // namespace protocols
}  // namespace rogue

#endif
