/**
 * ----------------------------------------------------------------------------
 * Company    : SLAC National Accelerator Laboratory
 * ----------------------------------------------------------------------------
 * Description:
 * RoCEv2 Server - receives RDMA WRITE-with-Immediate frames from an FPGA
 * and injects them into the rogue stream pipeline as ris::Frames.
 *
 * Immediate value format (32 bits, big-endian on wire, host-endian in wc):
 *
 *  31       8  7        0
 *  ┌─────────┬──────────┐
 *  │reserved │ channel  │
 *  │  (24b)  │   (8b)   │
 *  └─────────┴──────────┘
 *
 * byte_len is taken directly from wc.byte_len (free from the CQ completion).
 * firstUser is hardcoded to 0x2 (SSI SOF) on every frame because each
 * RDMA WRITE is always an atomic, self-contained frame.
 * ----------------------------------------------------------------------------
 **/

#ifndef ROGUE_PROTOCOLS_ROCEV2_SERVER_H
#define ROGUE_PROTOCOLS_ROCEV2_SERVER_H

#include <infiniband/verbs.h>
#include <stdint.h>

#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

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
    // Receive thread
    std::thread* thread_;
    bool         threadEn_;

    // Maps wr_id → pre-posted frame so we can find the buffer on completion
    std::map<uint64_t, rogue::interfaces::stream::FramePtr> wrMap_;
    // Maps wr_id → ibv_mr* so we can deregister memory after completion
    std::map<uint64_t, struct ibv_mr*>                       mrMap_;
    std::mutex                                               wrMapMtx_;
    uint64_t                                                 nextWrId_;

    // Logging
    std::shared_ptr<rogue::Logging> log_;

    // Post a single receive work request backed by the given frame/buffer
    void postRecvWr(rogue::interfaces::stream::FramePtr frame);

    // Receive thread entry point
    void runThread(std::weak_ptr<int> lockPtr);

  public:
    // Factory method (mirrors udp::Server pattern)
    static std::shared_ptr<rogue::protocols::rocev2::Server> create(
        const std::string& deviceName,
        uint8_t            ibPort,
        uint8_t            gidIndex,
        uint32_t           maxPayload = DefaultMaxPayload,
        uint32_t           rxQueueDepth = DefaultRxQueueDepth);

    Server(const std::string& deviceName,
           uint8_t            ibPort,
           uint8_t            gidIndex,
           uint32_t           maxPayload,
           uint32_t           rxQueueDepth);

    ~Server();
    void stop();

    // ris::Slave interface - TX path back to FPGA (not implemented for
    // receive-only use; logs a warning and drops the frame).
    void acceptFrame(rogue::interfaces::stream::FramePtr frame) override;

    static void setup_python();
};

typedef std::shared_ptr<rogue::protocols::rocev2::Server> ServerPtr;

}  // namespace rocev2
}  // namespace protocols
}  // namespace rogue

#endif  // ROGUE_PROTOCOLS_ROCEV2_SERVER_H
