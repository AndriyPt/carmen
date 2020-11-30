//.$file${../src::business_logic::business_logic_base.cpp} vvvvvvvvvvvvvvvvvvv
//
// Model: carmen.qm
// File:  ${../src::business_logic::business_logic_base.cpp}
//
// This code has been generated by QM 5.1.0 <www.state-machine.com/qm/>.
// DO NOT EDIT SECTIONS BETWEEN THE COMMENTS "$...vvv".."$end...^^^".
// All your changes in these sections will be lost.
//
// This program is open source software: you can redistribute it and/or
// modify it under the terms of the GNU General Public License as published
// by the Free Software Foundation.
//
// This program is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
// or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
// for more details.
//
//.$endhead${../src::business_logic::business_logic_base.cpp} ^^^^^^^^^^^^^^^^
#include "business_logic_base.h"

//.$skip${QP_VERSION} vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
//. Check for the minimum required QP version
#if (QP_VERSION < 680U) || (QP_VERSION != ((QP_RELEASE^4294967295U) % 0x3E8U))
#error qpcpp version 6.8.0 or higher required
#endif
//.$endskip${QP_VERSION} ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//.$define${application::BusinessLogicBase} vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
namespace carmen_hardware {

//.${application::BusinessLogicBase} .........................................
//.${application::BusinessLogicBas~::SM} .....................................
Q_STATE_DEF(BusinessLogicBase, initial) {
    //.${application::BusinessLogicBas~::SM::initial}
    return tran(&init);
}
//.${application::BusinessLogicBas~::SM::init} ...............................
Q_STATE_DEF(BusinessLogicBase, init) {
    QP::QState status_;
    switch (e->sig) {
        //.${application::BusinessLogicBas~::SM::init::BL_SET_MOTORS_SPEED}
        case BL_SET_MOTORS_SPEED_SIG: {
            this->setMotorsSpeedHandler();
            status_ = Q_RET_HANDLED;
            break;
        }
        //.${application::BusinessLogicBas~::SM::init::BL_SET_MOTORS_PID}
        case BL_SET_MOTORS_PID_SIG: {
            this->setMotorsPidHandler();
            status_ = Q_RET_HANDLED;
            break;
        }
        default: {
            status_ = super(&top);
            break;
        }
    }
    return status_;
}

} // namespace carmen_hardware
//.$enddef${application::BusinessLogicBase} ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^