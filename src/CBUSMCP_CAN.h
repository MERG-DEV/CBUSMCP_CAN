
/*

  Copyright (C) Duncan Greenwood 2017 (duncan_greenwood@hotmail.com)

  This work is licensed under the:
      Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License.
   To view a copy of this license, visit:
      http://creativecommons.org/licenses/by-nc-sa/4.0/
   or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.

   License summary:
    You are free to:
      Share, copy and redistribute the material in any medium or format
      Adapt, remix, transform, and build upon the material

    The licensor cannot revoke these freedoms as long as you follow the license terms.

    Attribution : You must give appropriate credit, provide a link to the license,
                  and indicate if changes were made. You may do so in any reasonable manner,
                  but not in any way that suggests the licensor endorses you or your use.

    NonCommercial : You may not use the material for commercial purposes. **(see note below)

    ShareAlike : If you remix, transform, or build upon the material, you must distribute
                 your contributions under the same license as the original.

    No additional restrictions : You may not apply legal terms or technological measures that
                                 legally restrict others from doing anything the license permits.

   ** For commercial use, please contact the original copyright holder(s) to agree licensing terms

    This software is distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE

*/

#pragma once

// header files

#include <SPI.h>

#include <CBUS.h>               // abstract base class
#include <CBUSMCP_CAN.h>        // header for this class
#include <mcp_can.h>

// constants

static const byte CSPIN = 10;                               // SPI chip select pin
static const byte INTPIN = 2;                               // interrupt pin
static const byte NUM_BUFFS = 4;                            // default value
static const uint32_t CANBITRATE = 125000UL;                // 125Kb/s - fixed for CBUS
static const uint32_t OSCFREQ = 16000000UL;                 // crystal frequency default

/// class definitions

// buffer item type

typedef struct _buffer_entry {
  unsigned long _item_insert_time;
  CANFrame _item;
} buffer_entry_t;

//
/// a circular buffer class
//

class circular_buffer {

public:
  circular_buffer(byte num_items);
  ~circular_buffer();
  bool available(void);
  void put(const CANFrame *cf);
  CANFrame *peek(void);
  CANFrame *get(void);
  unsigned long insert_time(void);
  bool full(void);
  void clear(void);
  bool empty(void);
  byte size(void);
  byte free_slots(void);
  unsigned int puts();
  unsigned int gets();
  byte hwm(void);
  unsigned int overflows(void);

private:
  bool _full;
  byte _head, _tail, _capacity, _size, _hwm;
  unsigned int _puts, _gets, _overflows;
  buffer_entry_t *_buffer;
};

//
/// an implementation of the abstract base CBUS class
/// using the MCP_CAN library for MCP2515
//

class CBUSMCP_CAN : public CBUSbase {

public:

  CBUSMCP_CAN();
  CBUSMCP_CAN(CBUSConfig *the_config);
  ~CBUSMCP_CAN();

  // these methods are declared virtual in the base class and must be implemented by the derived class#ifdef ARDUINO_ARCH_RP2040
#ifdef ARDUINO_ARCH_RP2040
  bool begin(bool poll = false, SPIClassRP2040 spi = SPI);    // note default args
#else
  bool begin(bool poll = false, SPIClass spi = SPI);    // note default arguments
#endif
  bool available(void);
  CANFrame getNextMessage(void);
  bool sendMessage(CANFrame *msg, bool rtr = false, bool ext = false, byte priority = DEFAULT_PRIORITY);    // note default arguments
  void reset(void);

  // these methods are specific to this implementation
  // they are not declared or implemented by the base CBUS class
  void setNumBuffers(byte num_rx_buffers, byte _num_tx_buffers = 2);

#ifdef ARDUINO_ARCH_RP2040
  void setPins(byte cs_pin, byte int_pin, byte mosi_pin, byte miso_pin, byte sck_pin);
#else
  void setPins(byte cs_pin, byte int_pin);
#endif

  void printStatus(void);
  void setOscFreq(unsigned long freq);

  MCP_CAN *canp = 0;
  circular_buffer *tx_buffer, *rx_buffer;

private:
  void initMembers(void);
  unsigned long _osc_freq;
  byte _csPin, _intPin;
  byte _num_rx_buffers, _num_tx_buffers;
  bool _poll;
#ifdef ARDUINO_ARCH_RP2040
  byte _mosi_pin, _miso_pin, _sck_pin;
#endif

};

