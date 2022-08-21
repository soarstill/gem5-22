
#ifndef __PUDMA_PUCORE4_HH__
#define __PUDMA_PUCORE4_HH__

#include <string>

#include "base/trace.hh"
#include "dev/dma_device.hh"
#include "dev/pudma/pu_engine4.hh"
#include "mem/packet.hh"
#include "mem/request.hh"
#include "params/PuCore4.hh"
#include "sim/sim_object.hh"
#include "sim/system.hh"

namespace gem5
{

class PuCmd;
class PuEngine4;

class PuCore4 : public SimObject
{
  PARAMS(PuCore4)

  private:
    /**
     * Fill the buffer with the next chunk of data
     */
    void processEvent() {};

    /// An event that wraps the above function
    EventFunctionWrapper event;

    /**
     * Fills the buffer for one iteration. If the buffer isn't full, this
     * function will enqueue another event to continue filling.
     */
    void fillBuffer();

    /// The bytes processed per tick
    float bandwidth;

    /// The size of the buffer we are going to fill
    int bufferSize;

    /// The buffer we are putting our message in
    char *buffer;

    /// The message to put into the buffer.
    std::string message;

    /// The amount of the buffer we've used so far.
    int bufferUsed;

    uint8_t *bufferA;
    uint8_t *bufferB;
    uint8_t *bufferC;

    int bufferSizeA;
    int bufferSizeB;
    int bufferSizeC;

    Tick coreLatency;
    std::string opcode ;

    Addr m_dram2_base;  // start address of DRAM2
    Addr m_rom1_base;   // start address of Data A
    Addr m_rom2_base ;  // start address of Data B

  public:

   PuCore4(const PuCore4Params &p);
    ~PuCore4();

    /**
     * Called by an outside object. Starts off the events to fill the buffer
     * with a goodbye message.
     *
     * @param name the name of the object we are saying goodbye to.
     */
    void compute(PuCmd *cmd, PuEngine4 * pue4);

    void sayHello(std::string mesg, PuEngine4 * pue4);

    void startup() override;

  protected:
    void OnComplete(PuEngine4 * pue4);

private:

    // functions for Matrix add operation
    void startAdd(PuCmd *cmd, PuEngine4 *pue4); // init
    void processAdd(PuCmd *cmd, PuEngine4 *pue4); // Callback
    Tick doAdd(PuCmd *cmd); // calulate C = A + B
};

} // namespace gem5

#endif // __PUDMA_PUCORE4_HH__
