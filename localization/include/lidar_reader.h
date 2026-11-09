#ifndef LIDAR_READER_H
#define LIDAR_READER_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Bytes-only counterpart of LidarStream's unqualified arrival clock path.
 * No serial port, host clock, thread, SDK or localisation policy is owned here.
 * Config values have the same meanings as PacketTimingConfig/TimedSectorAssembler.
 */
typedef struct lidar_reader_config_v1 {
    double bearing_sign, bearing_zero_rad, direction, entry, width;
    double nominal_tick_ns, max_raw_angle_gap_deg;
    double max_interpacket_step_factor, angle_gap_quantization_margin_deg;
    double max_angle_step_deg;
    uint64_t timestamp_modulus, max_counter_gap_ticks, max_packet_tick_delta;
    uint64_t max_receive_gap_ns, max_points_per_packet;
    uint64_t max_rx_chunk_bytes, max_buffer_bytes, max_batch_span_ns;
    uint64_t max_points, max_span_ns, max_point_gap_ns, max_stall_ns;
} lidar_reader_config_v1;

typedef struct lidar_reader_point_v1 {
    uint64_t packet_offset;
    int64_t time_ns;
    double x, y, raw_angle_deg, intensity;
    uint32_t point_index;
    uint8_t valid_return;
} lidar_reader_point_v1;

typedef struct lidar_reader_status_v1 {
    /* rx_batches, rx_bytes, scan_packets, status_packets, ack_packets,
     * crc_errors, timing_rejections, sector_rejections, scans_emitted */
    uint64_t counters[9];
    uint8_t needs_new_epoch;
    char last_error[256];
} lidar_reader_status_v1;

/* Called for a whole completed sector, in time order, before later packets in
 * the same batch. Points are borrowed for this call only. Return 1 on success.
 * Arrival-primed output remains timing_qualified=false, as in the reference.
 */
typedef int (*lidar_reader_scan_v1)(void *user, const lidar_reader_point_v1 *points,
                                  size_t count);

enum lidar_reader_result_v1 {
    LIDAR_READER_OK = 0,
    LIDAR_READER_SOURCE_EPOCH = -1,
    LIDAR_READER_CALLBACK = -2,
    LIDAR_READER_INTERNAL = -3,
    LIDAR_READER_TIMESTAMP_VALUE = -4
};

uint32_t lidar_reader_abi_version(void);
void *lidar_reader_create_v1(const lidar_reader_config_v1 *config);
void lidar_reader_destroy_v1(void *reader);
int lidar_reader_feed_v1(void *reader, const uint8_t *data, size_t size,
                         int64_t arrival_ns, lidar_reader_scan_v1 scan, void *user);
int lidar_reader_get_status_v1(void *reader, lidar_reader_status_v1 *status);

#ifdef __cplusplus
}
#endif
#endif
