// TODO: Hax.... Cmake should be making this unnecessary.
#include <AbstractPlatform.h>
#include <I2CAdapter.h>

#define ACK_CHECK_EN   0x01     /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS  0x00     /*!< I2C master will not check ack from slave */

#include <Wire.h>


/*******************************************************************************
* ___     _                                  This is a template class for
*  |   / / \ o    /\   _|  _. ._ _|_  _  ._  defining arbitrary I/O adapters.
* _|_ /  \_/ o   /--\ (_| (_| |_) |_ (/_ |   Adapters must be instanced with
*                             |              a BusOp as the template param.
*******************************************************************************/


/*
* Init the hardware for the bus.
* There is only one correct pin combination for each i2c bus (surprisingly).
*/
int8_t I2CAdapter::bus_init() {
  switch (ADAPTER_NUM) {
    case 0:
      Wire.setSDA(_bus_opts.sda_pin);
      Wire.setSCL(_bus_opts.scl_pin);
      Wire.setClock(_bus_opts.freq);
      Wire.begin();
      busOnline(true);
      break;
    case 1:
      Wire1.setSDA(_bus_opts.sda_pin);
      Wire1.setSCL(_bus_opts.scl_pin);
      Wire1.setClock(_bus_opts.freq);
      Wire1.begin();
      busOnline(true);
      break;
    default:
      break;
  }
  return (busOnline() ? 0:-1);
}


int8_t I2CAdapter::bus_deinit() {
  busOnline(false);
  switch (ADAPTER_NUM) {
    case 0:  // Dedicated pins
      break;
    case 1:
      break;
    default:
      return -1;
  }
  return 0;
}



void I2CAdapter::printHardwareState(StringBuilder* output) {
  output->concatf("-- I2C%d (%sline)\n", adapterNumber(), (_adapter_flag(I2C_BUS_FLAG_BUS_ONLINE)?"on":"OFF"));
}


int8_t I2CAdapter::generateStart() {
  return busOnline() ? 0 : -1;
}


int8_t I2CAdapter::generateStop() {
  return busOnline() ? 0 : -1;
}



/*******************************************************************************
* ___     _                              These members are mandatory overrides
*  |   / / \ o     |  _  |_              from the BusOp class.
* _|_ /  \_/ o   \_| (_) |_)
*******************************************************************************/

XferFault I2CBusOp::begin() {
  set_state(XferState::INITIATE);  // Indicate that we now have bus control.
  switch (device->adapterNumber()) {
    case 0:
    case 1:
      // {
      //   // TODO: Most of this is ugly and reeks of duress. Re-work on async conversion.
      //   switch (opcode) {
      //     case BusOpcode::TX_CMD:
      //       set_state(XferState::TX_WAIT);
      //       xfer_fault = (0 != Chip_I2CM_XferBlocking(b_enum, &xfer_struct)) ? XferFault::NONE : XferFault::DEV_FAULT;
      //       break;
      //     case BusOpcode::TX:
      //       if (buf_len) {
      //         const uint8_t TEMP_BUF_LEN = (-1 != sub_addr) ? buf_len + 1 : buf_len;
      //         uint8_t stacked_buf[TEMP_BUF_LEN];
      //         uint8_t i = 0;
      //         if (-1 != sub_addr) {
      //           stacked_buf[i++] = (uint8_t) sub_addr;
      //         }
      //         while (i < TEMP_BUF_LEN) {
      //           stacked_buf[i] = *(buf + i);
      //           i++;
      //         }
      //         xfer_struct.txSz   = TEMP_BUF_LEN;
      //         xfer_struct.txBuff = stacked_buf;
      //         set_state(XferState::TX_WAIT);
      //         xfer_fault = (0 != Chip_I2CM_XferBlocking(b_enum, &xfer_struct)) ? XferFault::NONE : XferFault::BUS_FAULT;
      //       }
      //       break;
      //     case BusOpcode::RX:
      //       if (buf_len) {
      //         xfer_struct.txSz   = (-1 != sub_addr) ? 1 : 0;
      //         xfer_struct.txBuff = (-1 != sub_addr) ? (uint8_t*) &sub_addr : nullptr;
      //         xfer_struct.rxSz   = buf_len;
      //         xfer_struct.rxBuff = buf;
      //         set_state(XferState::RX_WAIT);
      //         xfer_fault = (0 != Chip_I2CM_XferBlocking(b_enum, &xfer_struct)) ? XferFault::NONE : XferFault::BUS_FAULT;
      //       }
      //       break;
      //     default:
      //       abort(XferFault::BAD_PARAM);
      //       break;
      //   }
      // }
      break;
    default:
      abort(XferFault::BAD_PARAM);
      break;
  }
  markComplete();
  return getFault();
}


/*
* FreeRTOS doesn't have a concept of interrupt, but we might call this
*   from an I/O thread.
*/
XferFault I2CBusOp::advance(uint32_t status_reg) {
  return getFault();
}
