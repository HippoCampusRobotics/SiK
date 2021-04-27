// -*- Mode: C; c-basic-offset: 8; -*-
//
// Copyright (c) 2013 Luke Hovington, All Rights Reserved
// Copyright (c) 2012 Andrew Tridgell, All Rights Reserved
// Copyright (c) 2011 Michael Smith, All Rights Reserved
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  o Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  o Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in
//    the documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
// INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
// HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
// STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
// OF THE POSSIBILITY OF SUCH DAMAGE.
//

///
/// @file	tdm.c
///
/// time division multiplexing code
///

#include "tdm.h"

#include <stdarg.h>

#include "crc.h"
#include "freq_hopping.h"
#include "golay.h"
#include "hippolink.h"
#include "packet.h"
#include "radio.h"
#include "timer.h"

#define SYNC_BIT 0x8000
#define IS_SYNC(x) (x & SYNC_BIT)
/// removes the sync bit of the nodeid in the radio packet _trailer
#define REMOVE_SYNC_BIT(x) (x & 0x7FFF)
#define ADD_SYNC_BIT(x) (x | SYNC_BIT)

/// stats packets are marked with a window of 0
#define STATS_WINDOW 0

#define IS_STATS_PACKET(window, len) (window == STATS_WINDOW && len != 0)

#define IS_CMD_PACKET(command) (command == 1)

#define RSSI_REPORT_UPDATE_TICKS 12500

/// the state of the tdm system
enum _tdm_state { TDM_TRANSMIT, TDM_RECEIVE, TDM_SYNC };
__pdata static enum _tdm_state _tdm_state;
__pdata static uint16_t
    _transmitting_node;  // sequence the nodes can transmit in.
__pdata static uint16_t
    _paramNodeDestination;                 // User defined Packet destination
__pdata static uint16_t _nodeDestination;  // Real Packet Destination (as some
                                           // messages should be broadcasted)

/// a packet buffer for the TDM code
__xdata uint8_t _pbuf[MAX_PACKET_LENGTH];

/// how many 16usec ticks are remaining in the current state
__pdata static uint16_t _tdm_state_remaining;

/// This is enough to hold at least 3 packets and is based
/// on the configured air data rate.
__pdata static uint16_t _tx_window_width;
__pdata static uint16_t _tx_sync_width;

/// the maximum data packet size we can fit
__pdata static uint8_t _max_data_packet_length;

/// the silence period between transmit windows
/// This is calculated as the number of ticks it would take to transmit
/// two zero length packets
__pdata static uint16_t _silence_period;

// Half the linkupdate frequency to display test data
static __bit _test_display_toggle;

#if USE_TICK_YIELD
// records if the node so far has yielded to us,
// as soon as a node doesn't yield we stop transmitting until our turn again
__pdata static uint16_t _lastTransmitWindow;
// if it's our transmitters turn, we have yielded and someone else has
// transmitted
static __bit _received_packet;
static __bit _yielded_slot;

/// whether we have yielded our window to the other radio, or should send a
/// yield packet
static __bit _transmit_yield;

enum tdm_yield {
    YIELD_SET = true,
    YIELD_GET = false,
    YIELD_TRANSMIT = true,
    YIELD_RECEIVE = false,
    YIELD_NO_DATA = false,
    YIELD_DATA = true
};
#endif  // USE_TICK_YIELD

// activity indication
// when the 16 bit timer2_tick() value wraps we check if we have received a
// packet since the last wrap (ie. every second)
// If we have the green radio LED is held on.
// Otherwise it blinks every 1 seconds. The _received_sync flag
// is set for any received sync packet
static __bit _blink_state;
static __bit _received_sync;
__pdata static uint8_t _sync_count;  // the amount of successfull times synced
static __bit _sync_any;

/// the latency in 16usec timer2 ticks for sending a zero length packet
__pdata static uint16_t _packet_latency;

/// the time in 16usec ticks for sending one byte
__pdata static uint16_t _ticks_per_byte;

/// number of 16usec ticks to wait for a preamble to turn into a packet
/// This is set when we get a preamble interrupt, and causes us to delay
/// sending for a maximum packet latency. This is used to make it more likely
/// that two radios that happen to be exactly in sync in their sends
/// will eventually get a packet through and get their transmit windows
/// sorted out
__pdata uint16_t _transmit_wait;

/// the long term duty cycle we are aiming for
__pdata uint8_t _duty_cycle;

/// the average duty cycle we have been transmitting
__data static float _average_duty_cycle;

/// duty cycle offset due to temperature
__pdata uint8_t _duty_cycle_offset;

/// set to true if we need to wait for our duty cycle average to drop
static bool _duty_cycle_wait;

/// how many ticks we have transmitted for in this TDM round
__pdata static uint16_t _transmitted_ticks;

/// the LDB (listen before talk) RSSI threshold
__pdata static uint8_t _lbt_rssi;

/// how long we have listened for for LBT
__pdata static uint16_t _lbt_listen_time;

/// how long we have to listen for before LBT is OK
__pdata static uint16_t _lbt_min_time;

/// random addition to LBT listen time (see European regs)
__pdata static uint16_t _lbt_rand;

/// test data to display in the main loop. Updated when the tick
/// counter wraps, zeroed when display has happened
__pdata uint8_t _test_display;

// Statisics packet recive count
__pdata uint16_t _received_packets_count;
// set to 0 when we should send statistics packets
__pdata uint16_t _statistics_transmit_stats;
// handle ati5 command, as this is a long and doesn't fit into the buffer
__pdata uint8_t _ati5_id;

/// set when we should send a MAVLink report pkt
extern uint8_t seen_mavlink;

struct tdm_trailer {
    uint16_t window : 13;
    uint16_t command : 1;
    uint16_t bonus : 1;
    uint16_t resend : 1;
    uint16_t nodeid;
};
__pdata struct tdm_trailer _trailer;

/// buffer to hold a remote AT command before sending
static __bit send_at_command;
static __pdata uint16_t send_at_command_to;
static __xdata char remote_at_cmd[AT_CMD_MAXLEN + 1];

// local nodeCount
__pdata static uint16_t nodeCount;

/// display RSSI output
void tdm_show_rssi(void) {
    // Using printfl helps a bit but still overloads the cpu when AT&T=RSSI is
    // used. This causes pauses and eventualy the nodes drift out of sync
    __pdata uint8_t i;
    for (i = 0; i < (nodeCount - 1) && i < MAX_NODE_RSSI_STATS; i++) {
        if (i != nodeId) {
            printfl("[%u] L/R RSSI: %u/%u  L/R noise: %u/%u\n", (unsigned)i,
                    (unsigned)statistics[i].average_rssi,
                    (unsigned)remote_statistics[i].average_rssi,
                    (unsigned)statistics[nodeId].average_noise,
                    (unsigned)remote_statistics[i].average_noise);
        }
    }
    printfl(
        "[%u] pkts: %u txe=%u rxe=%u stx=%u srx=%u ecc=%u/%u temp=%d dco=%u\n",
        (unsigned)nodeId, (unsigned)_received_packets_count,
        (unsigned)errors.tx_errors, (unsigned)errors.rx_errors,
        (unsigned)errors.serial_tx_overflow,
        (unsigned)errors.serial_rx_overflow, (unsigned)errors.corrected_errors,
        (unsigned)errors.corrected_packets, (int)radio_temperature(),
        (unsigned)_duty_cycle_offset);
    _received_packets_count = 0;
}

/// display test output
static void display_test_output(void) {
    if (_test_display & AT_TEST_RSSI) {
        tdm_show_rssi();
    }
}

/// estimate the flight time for a packet given the payload size
///
/// @param packet_len		payload length in bytes
///
/// @return			flight time in 16usec ticks
static uint16_t flight_time_estimate(__pdata uint8_t packet_len) {
    return _packet_latency + (packet_len * _ticks_per_byte);
}

/// update the TDM state machine
///
static void tdm_state_update(__pdata uint16_t tdelta) {
    // update the amount of time we are waiting for a preamble
    // to turn into a real packet
    if (tdelta > _transmit_wait) {
        _transmit_wait = 0;
    } else {
        _transmit_wait -= tdelta;
    }

    // have we passed the next transition point?
    while (tdelta >= _tdm_state_remaining) {
#ifdef WATCH_DOG_ENABLE
        // Tickle Watchdog
        PCA0CPH5 = 0;
#endif  // WATCH_DOG_ENABLE
        if ((_transmitting_node < 0x8000 || nodeId == BASE_NODEID) &&
            (_transmitting_node++ % nodeCount) == nodeId) {
            _tdm_state = TDM_TRANSMIT;
            _transmitting_node %= nodeCount;
        }
        // We need to -1 from _transmitting_node as it was incremented above
        // Remember we have incremented nodeCount to allow for the sync period
        else if (_transmitting_node < 0x8000 &&
                 (_transmitting_node - 1 % nodeCount) == nodeCount - 1) {
            _tdm_state = TDM_SYNC;
        } else {
            // Check for Bonus?
            _tdm_state = TDM_RECEIVE;  // If there are other nodes yet to
                                       // transmit lets hear them first
        }
#ifdef DEBUG_PINS_SYNC
        if (_tdm_state == TDM_SYNC) {
            P0 |= 0x02;
        } else {
            P0 &= ~0x02;
        }
#endif  // DEBUG_PINS_SYNC

        // work out the time remaining in this state
        tdelta -= _tdm_state_remaining;

        if (_tdm_state == TDM_SYNC)
            _tdm_state_remaining = _tx_sync_width;
        else {
            _tdm_state_remaining = _tx_window_width;
            // change frequency when finishing transmitting or reciving
            fhop_window_change();
        }

        radio_receiver_on();

        if (num_fh_channels > 1) {
            // reset the LBT listen time
            _lbt_listen_time = 0;
            _lbt_rand = 0;
        }

        if (_tdm_state == TDM_TRANSMIT &&
            (_duty_cycle - _duty_cycle_offset) != 100) {
            // update duty cycle averages
            _average_duty_cycle = (0.95 * _average_duty_cycle) +
                                  (0.05 * (100.0 * _transmitted_ticks) /
                                   (2 * (_silence_period + _tx_window_width)));
            _transmitted_ticks = 0;
            _duty_cycle_wait =
                (_average_duty_cycle >= (_duty_cycle - _duty_cycle_offset));
        }

        // no longer waiting for a packet
        _transmit_wait = 0;
    }

    // set right receive channel
    if (_tdm_state == TDM_SYNC) {
        radio_set_channel(fhop_sync_channel());
    } else {
        radio_set_channel(fhop_receive_channel());
    }

    _tdm_state_remaining -= tdelta;
}

#if USE_TICK_YIELD
/// update if another is yielding or has yielded (if we want to transmit)
///
static uint8_t tdm_yield_update(__pdata uint8_t set_yield,
                                __pdata uint8_t no_data) {
    // Sort out the sync period first..
    if (_tdm_state == TDM_SYNC && !set_yield) {
        if (nodeId == BASE_NODEID) {
            return YIELD_TRANSMIT;
        } else {
            _lastTransmitWindow = nodeId | 0x8000;
            return YIELD_RECEIVE;
        }
    }

    if (_tdm_state != TDM_TRANSMIT) {
        if (_received_packet) {
            _received_packet = false;
#ifdef DEBUG_PINS_YIELD
            P2 &= ~0x40;
#endif  // DEBUG_PINS_YIELD
        }

        // REMEMBER nodeCount is set one higher than the user has set, this is
        // to add sync to the sequence _transmitting_node points to the next
        // slot so we also have to remove one from here
        if (set_yield == YIELD_GET) {
            if ((_transmitting_node != 0 &&
                 (_lastTransmitWindow & 0x7FFF) ==
                     ((_transmitting_node - 1) % (nodeCount - 1))) ||
                (_transmitting_node == 0 &&
                 (_lastTransmitWindow & 0x7FFF) == (nodeCount - 2))) {
                return YIELD_TRANSMIT;
            } else {
                return YIELD_RECEIVE;
            }
        } else if (no_data) {
            // Mark the first packet to send as a interrupt packet
            if ((_lastTransmitWindow & 0x7FFF) != _trailer.nodeid) {
                _transmit_yield = true;
            }

            // Make sure all nodes so far have yielded to us..
            // Make sure the node waits for a random amount of time..
            if (_lastTransmitWindow < 0x8000 &&
                _trailer.nodeid ==
                    ((_lastTransmitWindow + 1) % (nodeCount - 1))) {
                _lastTransmitWindow = _trailer.nodeid;
                _transmit_wait = _packet_latency +
                                 ((uint16_t)rand()) % (_packet_latency * 2);
            }
            // Ensure that other nodes get a chance to respond before we take
            // their slot.
            else {
                _lastTransmitWindow = _trailer.nodeid | 0x8000;
                // This gives prefrance to a lower nodeId, need to think of a
                // better way of yielding a slot
                _transmit_wait =
                    (_packet_latency * (nodeId + 1) +
                     ((uint16_t)rand()) % (_packet_latency * (nodeId + 2)));
            }
            // We can't have a delay more than 4* packet_lantency..
            _transmit_wait %= (_packet_latency * 4);
        }
        // Change the Window so we don't send any data without politely asking
        // first
        else {
            _lastTransmitWindow = nodeId | 0x8000;
        }

        // Ensure it defaults to receive though this should only be reached when
        // setting yield
        return YIELD_RECEIVE;
    } else if (_tdm_state == TDM_TRANSMIT) {  // We must be in Transmit Mode
        // If we have recived a packet during our Transmit window we have been
        // yielded..
        if (_received_packet) {
            _lastTransmitWindow = 0x8000;
            return YIELD_RECEIVE;
        }

        if (_yielded_slot) {
            // Change the Window so we don't send any data without politely
            // asking first
            _lastTransmitWindow = nodeId | 0x8000;
        } else {
            _lastTransmitWindow = nodeId;
        }

        if (_transmit_yield) {
            // reset the yield flag
            _transmit_yield = false;

            // Lets wait to see if anyone else needs to transmit
            _transmit_wait = _packet_latency * 6;

            return YIELD_RECEIVE;
        }
        return YIELD_TRANSMIT;
    }
    return YIELD_TRANSMIT;
}
#endif  // USE_TICK_YIELD

/// called to check temperature
///
static void temperature_update(void) {
    register int16_t diff;
    if (radio_get_transmit_power() <= 20) {
        _duty_cycle_offset = 0;
        return;
    }

    diff = radio_temperature() - MAX_PA_TEMPERATURE;
    if (diff <= 0 && _duty_cycle_offset > 0) {
        // under temperature
        _duty_cycle_offset -= 1;
    } else if (diff > 10) {
        // getting hot!
        _duty_cycle_offset += 10;
    } else if (diff > 5) {
        // well over temperature
        _duty_cycle_offset += 5;
    } else if (diff > 0) {
        // slightly over temperature
        _duty_cycle_offset += 1;
    }
    // limit to minimum of 20% duty cycle to ensure link stays up OK
    if ((_duty_cycle - _duty_cycle_offset) < 20) {
        _duty_cycle_offset = _duty_cycle - 20;
    }
}

/// blink the radio LED if we have not received any packets
///
static uint8_t unlock_count, temperature_count;
static void link_update(void) {
    if (nodeId == BASE_NODEID || _received_sync) {
        unlock_count = 0;
        _received_sync = false;
        fhop_set_locked(true);
#ifdef TDM_SYNC_LOGIC
        TDM_SYNC_PIN = true;
#endif  // TDM_SYNC_LOGIC
    } else {
        unlock_count++;
    }

    // Unlock greater than second
    if (unlock_count < 2) {
        RADIO_LED(LED_ON);
    } else {
        _sync_count = 0;
        RADIO_LED(_blink_state);
        _blink_state = !_blink_state;
        _transmitting_node = 0xFFFF;

        memset(remote_statistics, 0, sizeof(remote_statistics));
        memset(statistics, 0, sizeof(statistics));

        // reset statistics when unlocked
        _received_packets_count = 0;

#ifdef TDM_SYNC_LOGIC
        TDM_SYNC_PIN = false;
#endif  // TDM_SYNC_LOGIC
    }

    // Go into sync mode if we have dropped out
    if (unlock_count > 3) {
        if (_sync_any) {
            if (unlock_count % 5 == 4) {
                fhop_window_change();  // Try our luck on another channel
            }
        } else {
            fhop_set_locked(false);  // Set channel back to sync and try again
            radio_set_channel(fhop_sync_channel());
        }

#ifdef UNLOCK_RESET_SEC
        // Everything is broken :(
        if (unlock_count > UNLOCK_RESET_SEC && !at_mode_active) {
            // Reset the device using sofware reset
            RSTSRC |= 0x10;
        }
#endif  // UNLOCK_RESET_SEC
    }

    _statistics_transmit_stats = 0;

    // Half the display rate
    if (_test_display_toggle) {
        _test_display = at_testmode;
    }
    _test_display_toggle = !_test_display_toggle;

    temperature_count++;
    if (temperature_count == 4) {
        // check every 2 seconds
        temperature_update();
        temperature_count = 0;
    }
}

// dispatch an AT command to the remote system
void tdm_remote_at(__pdata uint16_t destination) {
    memcpy(remote_at_cmd, at_cmd, strlen(at_cmd) + 1);
    send_at_command_to = destination;
    send_at_command = true;
}

// handle an incoming at command from the remote radio
static void handle_at_command(__pdata uint8_t len) {
    if (len < 2 || len > AT_CMD_MAXLEN || _pbuf[0] != (uint8_t)'R' ||
        _pbuf[1] != (uint8_t)'T') {
        // assume its an AT command reply
        register uint8_t i;
        for (i = 0; i < len; i++) {
            putchar(_pbuf[i]);
        }
        return;
    }

    // Set the return address..
    send_at_command_to = _trailer.nodeid;

    // setup the command in the at_cmd buffer
    memcpy(at_cmd, _pbuf, len);
    at_cmd[len] = '\0';
    at_cmd[0] = 'A';  // replace 'R'
    at_cmd_len = len;

#ifdef WATCH_DOG_ENABLE
    // Pat the Watchdog
    PCA0CPH5 = 0;
#endif  // WATCH_DOG_ENABLE

    // Capture ATI5 and proccess separatly
    if (len == 4 && at_cmd[2] == (uint8_t)'I' && at_cmd[3] == (uint8_t)'5') {
        _ati5_id = 0;
        packet_ati5_inject(_ati5_id++);
    } else {
        // run the AT command, capturing any output to the packet buffer
        // this reply buffer will be sent at the next opportunity
        packet_at_inject();
    }

#ifdef WATCH_DOG_ENABLE
    // Pat the Watchdog
    PCA0CPH5 = 0;
#endif  // WATCH_DOG_ENABLE
}

static void handle_sync_packet() {
    if (_transmitting_node == BASE_NODEID) {
        if (!(++_sync_count)) {
            _sync_count = 0xFF;
        }
    }
    _transmitting_node = BASE_NODEID;
    set_transmit_channel(REMOVE_SYNC_BIT(_trailer.nodeid));
    _received_sync = true;
}

static void handle_anysync_packet() {
    if (_transmitting_node == _trailer.nodeid + 1) {
        if (!(++_sync_count)) {
            _sync_count = 0xFF;
        }
        _transmitting_node = _trailer.nodeid + 1;
        _received_sync = true;
    }
}

static void handle_stats_packet(uint8_t len) {
    if ((_trailer.nodeid < MAX_NODE_RSSI_STATS) &&
        len ==
            (sizeof(struct statistics) + sizeof(_statistics_transmit_stats))) {
        len -= sizeof(_statistics_transmit_stats);

        // is the stats packet for us?
        if (((uint16_t *)(_pbuf + len))[0] == nodeId)
            memcpy(remote_statistics + _trailer.nodeid, _pbuf, len);
    }
}

static void handle_non_stats_packet(uint8_t len) {
    _tdm_state = _trailer.window;
#if USE_TICK_YIELD
    // 0 length package means someone wants to yield his remaining
    // transmission time.
    tdm_yield_update(YIELD_SET, len == 0);
#endif  // USE_TICK_YIELD
    if (IS_CMD_PACKET(_trailer.command)) {
        if (len > 1) {
            handle_at_command(len);
        }
    } else if (len != 0 && !packet_is_duplicate(len, _pbuf, _trailer.resend) &&
               !at_mode_active) {
        ACTIVITY_LED(LED_ON);
        serial_write_buf(_pbuf, len);
        ACTIVITY_LED(LED_OFF);
    }
}

/**
 * @brief Handles a received radio packet.
 *
 * @param len Length of the received packet.
 * @return Returns true if received a non-stats packet. The remaining time of
 * the current tdm state will be updated according to the packets window data
 * in the _trailer.
 */
static bool handle_received_packet(register uint8_t len) {
    bool ret = false;
    _transmit_wait = 0;
    if (len < sizeof(_trailer)) {
        printf("Invalid length.. %u\n", len);
        return false;
    }

#if USE_TICK_YIELD
    if (_tdm_state == TDM_TRANSMIT) {
        _received_packet = true;
        _lastTransmitWindow = 0x8000;
#ifdef DEBUG_PINS_YIELD
        P2 |= 0x40;
#endif  // DEBUG_PINS_YIELD
    }
#endif  // USE_TICK_YIELD

    memcpy(&_trailer, _pbuf + len - sizeof(_trailer), sizeof(_trailer));
    len -= sizeof(_trailer);

    if (IS_SYNC(_trailer.nodeid)) {
        handle_sync_packet();
        return false;
    } else if (_sync_any && !_trailer.bonus) {
        handle_anysync_packet();
    }

    if (_trailer.nodeid < MAX_NODE_RSSI_STATS)
        statistics[_trailer.nodeid].average_rssi =
            (radio_last_rssi() +
             7 * (uint16_t)statistics[_trailer.nodeid].average_rssi / 8);
    _received_packets_count++;

    if (IS_STATS_PACKET(_trailer.window, len)) {
        handle_stats_packet(len);
        // don't count control packets in the stats
        _received_packets_count--;
    } else if (_trailer.window != STATS_WINDOW) {
        // we've got some real message data!
        handle_non_stats_packet(len);
        ret = true;
    }
    return ret;
}

static inline bool is_in_silence_period() {
    return ((_tdm_state_remaining > _tx_window_width - _silence_period) ||
            (_tdm_state == TDM_SYNC &&
             _tdm_state_remaining > _tx_sync_width - _silence_period));
}

static inline bool still_doing_lbt(uint16_t tdelta) {
    if (!_lbt_rssi) return false;
    if (radio_current_rssi() < _lbt_rssi)
        // channel seems to be free.
        _lbt_listen_time += tdelta;
    else {
        // probably someone is talking on our channel. wait for some time.
        _lbt_listen_time = 0;
        if (!_lbt_rand) _lbt_rand = ((uint16_t)rand()) % _lbt_rand;
    }
    return (_lbt_listen_time < _lbt_min_time + _lbt_rand);
}

static inline uint8_t get_max_transmit() {
    if (_tdm_state_remaining < 2 * _packet_latency)
        return 0;
    else
        return (_tdm_state_remaining - 2 * _packet_latency) / _ticks_per_byte;
}

static inline bool is_good_to_send(register uint8_t max_transmit) {
    if (_duty_cycle_wait || _tdm_state_remaining < _packet_latency)
        return false;
    if (max_transmit < sizeof(_trailer) + 1) return false;
    return true;
}

static inline uint8_t prepare_packet(register uint8_t max_transmit) {
    __pdata uint8_t len = 0;
    if (_tdm_state == TDM_SYNC) return 0;
    len = strlen(remote_at_cmd);
    if (send_at_command && max_transmit >= len) {
        memcpy(_pbuf, remote_at_cmd, len);
        _trailer.command = 1;
        _nodeDestination = send_at_command_to;
        send_at_command = false;
    } else {
        len = packet_get_next(max_transmit, _pbuf);
        _trailer.command = packet_is_injected();
        if (_trailer.command) {
            _nodeDestination = send_at_command_to;
            packet_ati5_inject(_ati5_id++);
        }
    }
    if (len > _max_data_packet_length) {
        panic("oversized tdm packet");
    }
    return len;
}

static inline bool want_yielded_send() {
    return (serial_read_available() > 0 && _transmit_yield &&
            _tdm_state == TDM_RECEIVE);
}

static inline uint8_t prepare_to_send_in_yielded_window() {
    if (_tdm_state_remaining < _tx_window_width / 4) return 0;
    _pbuf[0] = 0xff;
    _trailer.command = 1;
    _nodeDestination = 0xFFFF;
    return 1;
}

// a stack carary to detect a stack overflow
__at(0xFF) uint8_t __idata _canary;

/// main loop for time division multiplexing transparent serial
///
void tdm_serial_loop(void) {
    __pdata uint16_t last_t = timer2_tick();
    __pdata uint16_t last_hippolink_rssi_report = last_t;
    __pdata uint16_t last_link_update = last_t;

    _canary = 42;

    for (;;) {
        __pdata uint8_t len;
        __pdata uint16_t tnow = timer2_tick(), tdelta;
        __pdata uint8_t max_xmit;

        if (_canary != 42) {
            panic("stack blown\n");
        }

        if (pdata_canary != 0x41) {
            panic("pdata canary changed\n");
        }
#ifdef WATCH_DOG_ENABLE
        // Throw the Watchdog a stick
        PCA0CPH5 = 0;
#endif  // WATCH_DOG_ENABLE

        // give the AT command processor a chance to handle a command
        at_command();

        // display test data if needed
        if (_test_display) {
            display_test_output();
            _test_display = 0;
        }

        if (tnow - last_hippolink_rssi_report > RSSI_REPORT_UPDATE_TICKS) {
            if (hippolink_rssi_report(nodeId, nodeCount))
                last_hippolink_rssi_report = tnow;
        }

        // get the time before we check for a packet coming in
        tnow = timer2_tick();

        // see if we have received a packet
        if (radio_receive_packet(&len, _pbuf)) {
            if (handle_received_packet(len)) {
                last_t = tnow;
            }
            continue;
        }

        // see how many 16usec ticks have passed and update
        // the tdm state machine. We re-fetch tnow as a bad
        // packet could have cost us a lot of time.
        tnow = timer2_tick();
        tdelta = tnow - last_t;
        tdm_state_update(tdelta);
        last_t = tnow;

        // wait for the silence period to expire, to allow radio's to switch
        // channel
        if (is_in_silence_period()) {
            continue;
        }

        // update link status approximately every 0.5s
        if (tnow - last_link_update > 32768) {
            link_update();
            last_link_update = tnow;
        }

        if (still_doing_lbt(tdelta)) {
            continue;
        }

        // we are allowed to transmit in our transmit window
        // or in the other radios transmit window if we have
        // bonus ticks
#if USE_TICK_YIELD
        if (tdm_yield_update(YIELD_GET, YIELD_NO_DATA) == YIELD_RECEIVE) {
#ifdef DEBUG_PINS_TRANSMIT_RECEIVE
            P2 &= ~0x04;
#endif  // DEBUG_PINS_TRANSMIT_RECEIVE
            continue;
        }
#ifdef DEBUG_PINS_TRANSMIT_RECEIVE
        P2 |= 0x04;
#endif  // DEBUG_PINS_TRANSMIT_RECEIVE
#ifdef DEBUG_PINS_TX_RX_STATE
        P2 &= ~0x08;
#endif  // DEBUG_PINS_TX_RX_STATE
#else   // USE_TICK_YIELD
        // If we arn't in transmit or our node id isn't BASE_NODEID and in
        // tdm_sync
        if (_tdm_state != TDM_TRANSMIT) {
            if (_tdm_state != TDM_SYNC || nodeId != BASE_NODEID) {
                continue;
            }
        }
#endif  // USE_TICK_YIELD

        if (_transmit_wait != 0) {
            // we're waiting for a preamble to turn into a packet
            continue;
        }

        if (radio_preamble_detected() || radio_receive_in_progress()) {
            // a preamble has been detected. Don't
            // transmit for a while
            _transmit_wait = _packet_latency;

#if USE_TICK_YIELD
            // If we detect a incoming packet during our transmit period
            // lets be quiet for the remainder of our period.
            // Sometimes we may not recive a packet as it has been filtered by
            // the radio
            if (_tdm_state == TDM_TRANSMIT) {
                _received_packet = true;
                _lastTransmitWindow = 0x8000;
#ifdef DEBUG_PINS_YIELD
                P2 |= 0x40;
#endif  // DEBUG_PINS_YIELD
            }
#endif  // USE_TICK_YIELD

            continue;
        }

#ifdef WATCH_DOG_ENABLE
        // Pat the Watchdog
        PCA0CPH5 = 0;
#endif  // WATCH_DOG_ENABLE

        // Dont send anything until we have received 20 good sync bytes
        if (nodeId != BASE_NODEID && _sync_count < 20) {
            continue;
        }

        // sample the background noise when it is out turn to
        // transmit, but we are not transmitting,
        // averaged over around 4 samples
        statistics[nodeId].average_noise =
            (radio_current_rssi() +
             3 * (uint16_t)statistics[nodeId].average_noise) /
            4;

        max_xmit = get_max_transmit();
        if (!is_good_to_send) continue;

        max_xmit -= sizeof(_trailer) + 1;
        if (max_xmit > _max_data_packet_length) {
            max_xmit = _max_data_packet_length;
        }

#if USE_TICK_YIELD
        // Check to see if we need to send a dummy packet to inform everyone in
        // the network we want to send data. This is done when we are yielding
        // only
        if (want_yielded_send()) {
            // if more than 1/4 of the slot is passed it wouldn't be worth
            // transmitting in this slot
            len = prepare_to_send_in_yielded_window();
            if (!len) {
                continue;
            }
        } else {
            len = prepare_packet(max_xmit);
        }
#else
        len = prepare_packet(max_xmit);
#endif  // USE_TICK_YIELD
        // ask the packet system for the next packet to send
        // no data is to be sent during a sync period

        _trailer.bonus = (_tdm_state == TDM_RECEIVE);
        _trailer.resend = packet_is_resend();

        // Are we in transmit phase and have space for a stats packet
        if (_tdm_state == TDM_TRANSMIT && len == 0 &&
            max_xmit >=
                (sizeof(statistics) + sizeof(_statistics_transmit_stats))
            // Do we need to send a stats packet
            && _statistics_transmit_stats < (nodeCount - 1) &&
            nodeId < MAX_NODE_RSSI_STATS
            // Yeild at the start of our time period to allow better data
            // throughput
            &&
            _tdm_state_remaining < (_tx_window_width - _packet_latency * 2)) {
            // Catch for Node 0
            if (_statistics_transmit_stats == nodeId) {
                _statistics_transmit_stats++;
            }

            len = sizeof(struct statistics);
            statistics[_statistics_transmit_stats].average_noise =
                statistics[nodeId].average_noise;
            memcpy(_pbuf, statistics + _statistics_transmit_stats, len);
            memcpy(_pbuf + len, &_statistics_transmit_stats,
                   sizeof(_statistics_transmit_stats));
            len += sizeof(_statistics_transmit_stats);

            _statistics_transmit_stats++;

            // Catch for last node
            if (_statistics_transmit_stats == nodeId) {
                _statistics_transmit_stats++;
            }

            // mark a stats packet with a zero window
            _trailer.window = 0;
            _trailer.resend = 0;
            _trailer.command = 0;
        } else if (_tdm_state != TDM_TRANSMIT && len == 0 &&
                   !(_tdm_state == TDM_SYNC && nodeId == BASE_NODEID)) {
            continue;  // If we have nothing contructive to send be quiet..
        } else {
            // calculate the control word as the number of
            // 16usec ticks that will be left in this
            // tdm state after this packet is transmitted
            _trailer.window =
                (uint16_t)(_tdm_state_remaining -
                           flight_time_estimate(len + sizeof(_trailer)));
        }

        // if in sync mode and we are the base, add the channel and sync bit
        if (_tdm_state == TDM_SYNC && nodeId == BASE_NODEID) {
            _trailer.nodeid = get_transmit_channel() | 0x8000;
        } else {
            _trailer.nodeid = nodeId;
        }

        memcpy(_pbuf + len, &_trailer, sizeof(_trailer));

        // If the command byte is set the _nodeDestination has already been set
        if (!_trailer.command) {
            if (len != 0 && _trailer.window != 0) {
                // show the user that we're sending real data
                ACTIVITY_LED(LED_ON);
                _nodeDestination = _paramNodeDestination;
            } else {  // Default to broadcast
                _nodeDestination = 0xFFFF;
            }
        }

#if USE_TICK_YIELD
        if (_tdm_state == TDM_TRANSMIT) {
            if (len == 0) {
                // sending a zero byte packet gives up
                // our window, but doesn't change the
                // start of the next window
                _transmit_yield = true;
                _yielded_slot = true;
            } else {
                _yielded_slot = false;
            }
        }
        // If we are here we must have permission to send data
        else if (_tdm_state == TDM_RECEIVE) {
            _lastTransmitWindow &= 0x7FFF;
        }
#endif  // USE_TICK_YIELD

        // after sending a packet leave a bit of time before
        // sending the next one. The receivers don't cope well
        // with back to back packets
#if USE_TICK_YIELD
        if (_transmit_yield && _tdm_state == TDM_RECEIVE) {
            _transmit_yield = false;
            _transmit_wait = 2 * _packet_latency;
#ifdef DEBUG_PINS_TX_RX_STATE
            P2 |= 0x08;
#endif  // DEBUG_PINS_TX_RX_STATE
        } else {
            _transmit_wait = _packet_latency;
        }
#else
        _transmit_wait = packet_latency;
#endif  // USE_TICK_YIELD

        // if we're implementing a duty cycle, add the
        // transmit time to the number of ticks we've been transmitting
        if ((_duty_cycle - _duty_cycle_offset) != 100) {
            _transmitted_ticks += flight_time_estimate(len + sizeof(_trailer));
        }

#ifdef WATCH_DOG_ENABLE
        // Feed Watchdog
        PCA0CPH5 = 0;
#endif  // WATCH_DOG_ENABLE

        // start transmitting the packet
        if (!radio_transmit(len + sizeof(_trailer), _pbuf, _nodeDestination,
                            _tdm_state_remaining) &&
            len != 0) {
            packet_force_resend();
        }

        if (_lbt_rssi != 0) {
            // reset the LBT listen time
            _lbt_listen_time = 0;
            _lbt_rand = 0;
        }

        // set right receive channel
        radio_set_channel(fhop_receive_channel());

        // re-enable the receiver
        radio_receiver_on();

        if (len != 0 && _trailer.window != 0) {
            ACTIVITY_LED(LED_OFF);
        }
    }
}

bool tdm_state_sync() { return _received_sync; }

// setup a 16 bit node count
//
void tdm_set_node_count(__pdata uint16_t count) {
    nodeCount = count + 1;  // add 1 for the sync channel
}

// setup a 16 bit node destination
//
void tdm_set_node_destination(__pdata uint16_t destination) {
    _paramNodeDestination = destination;
}

void tdm_set_sync_any(__pdata uint8_t any) { _sync_any = any; }

void tdm_set_lbt_rssi(__pdata uint8_t lbt_rssi) { _lbt_rssi = lbt_rssi; }

void tdm_set_duty_cycle(__pdata uint8_t duty_cycle) {
    _duty_cycle = duty_cycle;
}

// initialise the TDM subsystem
void tdm_init(void) {
    __pdata uint16_t i;
    __pdata uint8_t air_rate = radio_air_rate();
    __pdata uint32_t window_width;

#define REGULATORY_MAX_WINDOW (((1000000UL / 16) * 4) / 10)
#define LBT_MIN_TIME_USEC 5000

    // tdm_build_timing_table();

    // calculate how many 16usec ticks it takes to send each byte
    _ticks_per_byte = (8 + (8000000UL / (air_rate * 1000UL))) / 16;

    // Check for rounding errors, and round up if needed..
    if (10000UL * _ticks_per_byte <
        (8 + (8000000UL / (air_rate * 1000UL))) * 625) {
        _ticks_per_byte += 1;
    }

    // calculate the minimum packet latency in 16 usec units
    // we initially assume a preamble length of 40 bits, then
    // adjust later based on actual preamble length. This is done
    // so that if one radio has antenna diversity and the other
    // doesn't, then they will both using the same TDM round timings
    // 8 bytes header 5 bytes preamble and 13 ticks response time for the radio
    _packet_latency = (8 + (10 / 2)) * _ticks_per_byte + 13;

    if (feature_golay) {
        _max_data_packet_length =
            (MAX_PACKET_LENGTH / 2) - (6 + sizeof(_trailer));

        // golay encoding doubles the cost per byte
        _ticks_per_byte *= 2;

        // and adds 4 bytes
        _packet_latency += 4 * _ticks_per_byte;
    } else {
        _max_data_packet_length = MAX_PACKET_LENGTH - sizeof(_trailer);
    }

    // set the silence period to between changing channels
    _silence_period = 2 * _packet_latency;

    // set the transmit window to allow for 2 full sized packets
    window_width = 2 * ((_max_data_packet_length * (uint32_t)_ticks_per_byte) +
                        _packet_latency) +
                   _silence_period + _packet_latency;

    // if LBT is enabled, we need at least 3*5ms of window width
    if (_lbt_rssi != 0) {
        // min listen time is 5ms
        _lbt_min_time = LBT_MIN_TIME_USEC / 16;
        window_width = constrain(window_width, 3 * _lbt_min_time, window_width);
    }

    // make sure it fits in the 13 bits of the _trailer window
    if (window_width > 0x1FFF) {
        window_width = 0x1FFF;
    }

    // the window width cannot be more than 0.4 seconds to meet US regulations
    if (window_width >= REGULATORY_MAX_WINDOW) {
        window_width = REGULATORY_MAX_WINDOW;
    }

    _tx_window_width = window_width;

    // Window size of 4 statistic packets
    window_width = 4 * (((sizeof(_trailer)) * (uint32_t)_ticks_per_byte) +
                        _packet_latency) +
                   _silence_period + _packet_latency;
    _tx_sync_width = window_width;

    // now adjust the _packet_latency for the actual preamble
    // length, so we get the right flight time estimates, while
    // not changing the round timings
    _packet_latency += ((settings.preamble_length - 10) / 2) * _ticks_per_byte;

    // tell the packet subsystem our max packet size, which it
    // needs to know for MAVLink packet boundary detection
    i = (_tx_window_width - _packet_latency) / _ticks_per_byte;
    if (i > _max_data_packet_length) {
        i = _max_data_packet_length;
    }
    packet_set_max_xmit(i);

    // Clear Values..
    _trailer.nodeid = 0xFFFF;
    _transmitting_node = 0xFFFF;
    memset(remote_statistics, 0, sizeof(remote_statistics));
    memset(statistics, 0, sizeof(statistics));

    // crc_test();

    // tdm_test_timing();

    // golay_test();

    _ati5_id = PARAM_MAX;
    unlock_count = 6;
    RADIO_LED(LED_OFF);

#if USE_TICK_YIELD
    _received_packet = false;
#ifdef DEBUG_PINS_YIELD
    P2 &= ~0x40;
#endif  // DEBUG_PINS_YIELD
#endif  // USE_TICK_YIELD

#ifdef TDM_SYNC_LOGIC
    TDM_SYNC_PIN = false;
#endif  // TDM_SYNC_LOGIC
}

/// report tdm timings
///
void tdm_report_timing(void) {
    printf("[%u] _silence_period: %u\n", nodeId, (unsigned)_silence_period);
    delay_msec(1);
    printf("[%u] _tx_window_width: %u\n", nodeId, (unsigned)_tx_window_width);
    delay_msec(1);
    printf("[%u] _max_data_packet_length: %u\n", nodeId,
           (unsigned)_max_data_packet_length);
    delay_msec(1);
}
