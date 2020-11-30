//.$file${../include::public::types.h} vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
//
// Model: carmen.qm
// File:  ${../include::public::types.h}
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
//.$endhead${../include::public::types.h} ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^


//.$declare${application::ImuData} vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
namespace carmen_hardware {

//.${application::ImuData} ...................................................
class ImuData {
public:
    uint8_t velocity_x;
    uint16_t acceleration_y;
};

} // namespace carmen_hardware
//.$enddecl${application::ImuData} ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^