#include "orion_master.h"

class OrionMasterHelper: public OrionMaster
{
public:

  // Use Exceptions in case of error
  template<Command, Result> Result SendAndReceive(Command &command);

  template<Command, Result> Result SendAndReceive(Command &command, uint32_t retryTimeout);

  template<Command, Result> Result SendAndReceive(Command &command, uint32_t retryTimeout, uint8_t retryCount);

}