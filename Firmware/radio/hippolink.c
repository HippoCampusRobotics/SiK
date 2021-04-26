#include "serial.h"
#include "radio.h"

#define HIPPOLINK_MSG_ID_RADIO_RSSI_REPORT 1

#define HIPPOLINK_RADIO_RSSI_REPORT_CRC_EXTRA 123

#define HEADER_LENGTH 4
#define CRC_LENGTH 2
#define COBS_OFFSET 1
#define COBS_OVERHEAD 2

static void hippolink_fill_radio_rssi_report(uint8_t node_id, uint8_t remote_id);
static void hippolink_crc(register uint8_t crc_extra);
static void cobs_enocde(register uint8_t hippolink_len);

extern __xdata uint8_t pbuf[MAX_PACKET_LENGTH];
static __pdata uint8_t seqnum = 0;
static __pdata uint8_t _node_index = 0;

struct hippolink_header_s
{
    uint8_t msg_len;
    uint8_t seq;
    uint8_t node_id;
    uint8_t msg_id;
};

struct hippolink_radio_rssi_report_s
{
    uint8_t rem_id;
    uint8_t rssi;
    uint8_t rem_rssi;
    uint8_t noise;
    uint8_t rem_noise;
};

bool hippolink_rssi_report(uint8_t node_id, uint8_t node_count)
{
    uint8_t full_packet_size;
    full_packet_size = (sizeof(struct hippolink_radio_rssi_report_s) + HEADER_LENGTH + CRC_LENGTH + COBS_OVERHEAD);
    if (serial_write_space() < full_packet_size)
        return false;
    _node_index++;
    if (_node_index == node_id)
        _node_index++;
    if (_node_index >= node_count)
        _node_index = 0;
    hippolink_fill_radio_rssi_report(node_id, _node_index);
    cobs_enocde(full_packet_size - COBS_OVERHEAD);
    serial_write_buf(pbuf, full_packet_size);
    return true;
}

static void hippolink_fill_radio_rssi_report(uint8_t node_id, uint8_t remote_id)
{
    struct hippolink_header_s *header;
    struct hippolink_radio_rssi_report_s *payload;
    header = (struct hippolink_header_s *)&pbuf[COBS_OFFSET];
    payload = (struct hippolink_radio_rssi_report_s *)&pbuf[COBS_OFFSET + HEADER_LENGTH];

    header->msg_len = sizeof(struct hippolink_radio_rssi_report_s);
    header->seq = seqnum++;
    header->node_id = node_id;
    header->msg_id = HIPPOLINK_MSG_ID_RADIO_RSSI_REPORT;

    payload->rssi = statistics[remote_id].average_rssi;
    payload->rem_rssi = remote_statistics[remote_id].average_rssi;
    payload->noise = statistics[remote_id].average_noise;
    payload->rem_noise = remote_statistics[remote_id].average_noise;
    payload->rem_id = remote_id;

    hippolink_crc(HIPPOLINK_RADIO_RSSI_REPORT_CRC_EXTRA);
}

static void hippolink_crc(register uint8_t crc_extra)
{
    uint8_t length = pbuf[COBS_OFFSET];
    __pdata uint16_t sum = 0xFFFF;
    __pdata uint8_t i, stoplen;

    stoplen = length + HEADER_LENGTH + COBS_OFFSET;

    pbuf[stoplen++] = crc_extra;
    for (i = COBS_OFFSET; i < stoplen; i++)
    {
        register uint8_t tmp;
        tmp = pbuf[i] ^ (uint8_t)(sum & 0xFF);
        tmp ^= (tmp << 4);
        sum = (sum >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4);
    }
    pbuf[i-1] = (uint8_t)(sum & 0xFF);
    pbuf[i] = (uint8_t)(sum >> 8);
}

static void cobs_enocde(register uint8_t hippolink_len)
{
    uint8_t i;
    uint8_t offset = 0;
    pbuf[0] = 0;
    pbuf[hippolink_len + 1] = 0;
    for (i = 1; i <= hippolink_len + 1; i++)
    {
        offset++;
        if (!pbuf[i])
        {
            pbuf[i - offset] = offset;
            offset = 0;
        }
    }
}