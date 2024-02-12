
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

// AVR libC library
// #include <util/atomic.h>

// Arduino libraries
#include <SPI.h>

// 3rd party libraries
#include <Streaming.h>

// CBUS device-specific library
#include <CBUSMCP_CAN.h>

// interrupt handler
static void isr(void);

// globals
static CBUSMCP_CAN *pp;

//
/// constructor and destructor
//

CBUSMCP_CAN::CBUSMCP_CAN() {

  initMembers();
}

CBUSMCP_CAN::CBUSMCP_CAN(CBUSConfig *the_config) : CBUSbase(the_config) {

  initMembers();
}

void CBUSMCP_CAN::initMembers(void) {

  _num_rx_buffers = NUM_BUFFS;
  _num_tx_buffers = 0;
  eventhandler = NULL;
  eventhandlerex = NULL;
  framehandler = NULL;
  _csPin = CSPIN;
  _intPin = INTPIN;
  _osc_freq = OSCFREQ;
  _poll = false;
  pp = this;
}

CBUSMCP_CAN::~CBUSMCP_CAN() {

  free(tx_buffer);
  free(rx_buffer);
}

//
/// initialise the CAN controller and buffers, and attach the ISR
/// default poll arg is set to false, so as not to break existing code
//

#ifdef ARDUINO_ARCH_RP2040
bool CBUSMCP_CAN::begin(bool poll, SPIClassRP2040& spi)
#else
bool CBUSMCP_CAN::begin(bool poll, SPIClass& spi)
#endif
{
  _numMsgsSent = 0;
  _numMsgsRcvd = 0;
  _poll = poll;

  // allocate tx and tx buffers -- tx is currently unused
  rx_buffer = new circular_buffer(_num_rx_buffers);
  tx_buffer = new circular_buffer(_num_tx_buffers);

#ifdef ARDUINO_ARCH_RP2040
  spi.setTX(_mosi_pin);
  spi.setRX(_miso_pin);
  spi.setSCK(_sck_pin);
  spi.setCS(_csPin);
#endif

  // init SPI
  spi.begin();

  canp = new MCP_CAN(&spi, _csPin);     // pass pointer to SPI object to support devices with multiple SPI peripherals

  if (canp == nullptr) {
    // DEBUG_SERIAL << F("> error creating MCP_CAN object") << endl;
    return false;
  }

  byte freq = MCP_16MHZ;

  switch (_osc_freq) {
  case 8000000UL:
    freq = MCP_8MHZ;
    break;
  case 16000000UL:
    freq = MCP_16MHZ;
    break;
  case 20000000UL:
    freq = MCP_20MHZ;
    break;
  default:
    return false;
  }

  if (canp->begin(MCP_ANY, CAN_125KBPS, freq) != CAN_OK) {
    // DEBUG_SERIAL << F("> error from MCP_CAN begin") << endl;
    return false;
  }

  if (canp->setMode(MCP_NORMAL) != MCP2515_OK) {
    // DEBUG_SERIAL << F("> error setting CAN mode") << endl;
    return false;
  }

  // attach interrupt handler if not using polling
  if (!_poll) {
    pinMode(_intPin, INPUT_PULLUP);
    spi.usingInterrupt(digitalPinToInterrupt(_intPin));
    attachInterrupt(digitalPinToInterrupt(_intPin), isr, LOW);
  }

  return true;
}

//
/// check for one or more messages in the receive buffer
//

bool CBUSMCP_CAN::available(void) {

  if (_poll) {
    isr();
  }

  return rx_buffer->available();
}

//
/// must call available() first to ensure a message is available in the buffer
//

CANFrame CBUSMCP_CAN::getNextMessage(void) {

  CANFrame cf;

  ++_numMsgsRcvd;
  memcpy((CANFrame *)&cf, rx_buffer->get(), sizeof(CANFrame));
  return cf;
}

//
/// send a CBUS message
//

bool CBUSMCP_CAN::sendMessage(CANFrame *msg, bool rtr, bool ext, byte priority) {

  // caller must populate the message data
  // this method will create the correct frame header (CAN ID and priority bits)
  // rtr and ext default to false unless arguments are supplied - see method definition in .h
  // priority defaults to 1011 low/medium

  makeHeader(msg, priority);                      // default priority unless user overrides

  if (ext)
    msg->id |= 0x80000000;

  if (rtr)
    msg->id |= 0x40000000;

  if (canp->sendMsgBuf(msg->id, msg->len, msg->data) == CAN_OK) {
    ++_numMsgsSent;
    return true;
  } else {
    return false;
  }
}

//
/// display the CAN bus status instrumentation
//

void CBUSMCP_CAN::printStatus(void) {

  // removed so that no libraries produce serial output
  // can be implemented in user's sketch

  /*
    DEBUG_SERIAL << F("> CBUS status:");
    DEBUG_SERIAL << F(" messages received = ") << _numMsgsRcvd << F(", sent = ") << _numMsgsSent << F(", receive errors = ") << \
           canp->errorCountRX() << F(", transmit errors = ") << canp->errorCountTX() << F(", error flag = ") \
          << canp->getError() << endl;
  */
  return;
}

//
/// reset the MCP2515 transceiver
//

void CBUSMCP_CAN::reset(void) {

  delete canp;
  begin();
}

//
/// set the CS and interrupt pins - option to override defaults
//

#ifdef ARDUINO_ARCH_RP2040
void CBUSMCP_CAN::setPins(byte cs_pin, byte int_pin, byte mosi_pin, byte miso_pin, byte sck_pin)
#else
void CBUSMCP_CAN::setPins(byte cs_pin, byte int_pin)
#endif
{

#ifdef ARDUINO_ARCH_RP2040
  _mosi_pin = mosi_pin;
  _miso_pin = miso_pin;
  _sck_pin = sck_pin;
#endif

  _csPin = cs_pin;
  _intPin = int_pin;
}
//
/// set the number of CAN frame receive buffers
/// this can be tuned according to bus load and available memory
//

void CBUSMCP_CAN::setNumBuffers(byte num_rx_buffers, byte _num_tx_buffers) {
  _num_rx_buffers = num_rx_buffers;
  _num_tx_buffers = _num_tx_buffers;
}

//
/// set the MCP2515 crystal frequency
/// default is 16MHz but some modules have an 8MHz or 20MHz crystal
//

void CBUSMCP_CAN::setOscFreq(unsigned long freq) {
  _osc_freq = freq;
}

//
/// interrupt handler -- non-class method
/// read message from chip and insert into the receive buffer
//

static void isr(void) {
  CANFrame cf;

  if (pp->canp->readMsgBuf((unsigned long *)&cf.id, &cf.len, cf.data) == CAN_OK) {
    cf.ext = ((cf.id & 0x80000000) == 0x80000000);
    cf.rtr = ((cf.id & 0x40000000) == 0x40000000);
    pp->rx_buffer->put(&cf);
  }
}

///
/// a circular buffer class
///

/// constructor and destructor

circular_buffer::circular_buffer(byte num_items) {

  _head = 0;
  _tail = 0;
  _hwm = 0;
  _capacity = num_items;
  _size = 0;
  _puts = 0;
  _gets = 0;
  _overflows = 0;
  _full = false;
  _buffer = (buffer_entry_t *)malloc(num_items * sizeof(buffer_entry_t));
}

circular_buffer::~circular_buffer() {
  free(_buffer);
}

/// if buffer has one or more stored items

bool circular_buffer::available(void) {

  return (_size > 0);
}

/// store an item to the buffer - overwrite oldest item if buffer is full
/// only called from an interrupt context so we don't need to worry about subsequent interrupts

void circular_buffer::put(const CANFrame * item) {

  memcpy((CANFrame*)&_buffer[_head]._item, (const CANFrame *)item, sizeof(CANFrame));
  _buffer[_head]._item_insert_time = micros();

  // if the buffer is full, this put will overwrite the oldest item

  if (_full) {
    _tail = (_tail + 1) % _capacity;
    ++_overflows;
  }

  _head = (_head + 1) % _capacity;
  _full = _head == _tail;
  _size = size();
  _hwm = (_size > _hwm) ? _size : _hwm;
  ++_puts;

  return;
}

/// retrieve the next item from the buffer

CANFrame *circular_buffer::get(void) {

  CANFrame *p = nullptr;

  // should always call ::available first to avoid returning null pointer

  if (_size > 0) {
    p = &_buffer[_tail]._item;
    _full = false;
    _tail = (_tail + 1) % _capacity;
    _size = size();
    ++_gets;
  }

  return p;
}

/// get the insert time of the current buffer tail item
/// must be called before the item is removed by ::get

unsigned long circular_buffer::insert_time(void) {

  return (_buffer[_tail]._item_insert_time);
}

/// peek at the next item in the buffer without removing it

CANFrame *circular_buffer::peek(void) {

  // should always call ::available first to avoid this

  if (_size == 0) {
    return nullptr;
  }

  return (&_buffer[_tail]._item);
}

/// clear all items

void circular_buffer::clear(void) {

  _head = 0;
  _tail = 0;
  _full = false;
  _size = 0;

  return;
}

/// return high water mark

byte circular_buffer::hwm(void) {

  return _hwm;
}

/// return full indicator

bool circular_buffer::full(void) {

  return _full;
}

/// recalculate number of items in the buffer

byte circular_buffer::size(void) {

  byte size = _capacity;

  if (!_full) {
    if (_head >= _tail) {
      size = _head - _tail;
    } else {
      size = _capacity + _head - _tail;
    }
  }

  _size = size;
  return _size;
}

/// return empty indicator

bool circular_buffer::empty(void) {

  return (!_full && (_head == _tail));
}

/// return number of free slots

byte circular_buffer::free_slots(void) {

  return (_capacity - _size);
}

/// number of puts

unsigned int circular_buffer::puts(void) {

  return _puts;
}

/// number of gets

unsigned int circular_buffer::gets(void) {

  return _gets;
}

/// number of overflows

unsigned int circular_buffer::overflows(void) {

  return _overflows;
}
