#include "init.h"
#include "virtual_com_port.h"
#include "orion_protocol/orion_frame_transport.h"
#include "orion_protocol/orion_cobs_framer.h"
#include "orion_protocol/orion_header.h"
#include "orion_protocol/orion_minor.h"
#include "communication.h"
#include "business_logic.h"

static carmen_hardware::VirtualComPort com_port;
static orion::COBSFramer cobs_framer;
static orion::FrameTransport frame_transport = orion::FrameTransport(&com_port, &cobs_framer);
static orion::Minor minor(&frame_transport);

static carmen_hardware::Communication communication(&minor);
static carmen_hardware::BusinessLogic business_logic;

void app_init(void)
{
  communication.setPidCallback(business_logic.)

  Communication.getInstance(&minor);
}
