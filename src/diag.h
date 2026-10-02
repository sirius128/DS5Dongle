//
// Haptics pipeline diagnostics, exported through the tail of feature report 0xF9.
// All counters run on core0 (USB audio, BT send path and GET_REPORT), so plain
// increments are sufficient. Window fields (peak/sumsq) reset on every read.
//

#ifndef DS5_BRIDGE_DIAG_H
#define DS5_BRIDGE_DIAG_H

#include <cstdint>

struct HapticDiag {
    // cumulative
    uint32_t usb_frames_in;      // 48 kHz frames read from the USB audio OUT endpoint
    uint32_t rs_frames_out;      // 3 kHz frames produced by the resampler
    uint32_t blocks_made;        // 64-byte haptic blocks pushed to haptics_fifo
    uint32_t blocks_overwritten; // blocks discarded because haptics_fifo was full
    uint32_t bt39_sent;          // 0x39 audio/haptic reports handed to bt_write
    uint32_t bt_write_fail;      // bt_write could not queue (send_fifo full)
    uint32_t l2cap_send_err;     // l2cap_send returned non-zero
    uint32_t out_clipped;        // haptic int8 samples clamped
    // window (reset on read)
    uint16_t in_peak;            // max |int16| on haptic channels 3/4 from USB
    uint16_t out_peak;           // max |int8| sent to the controller
    float in_sumsq;              // sum of squares of normalised USB haptic samples
    float out_sumsq;             // sum of squares of normalised int8 haptic samples
    uint32_t in_count;
    uint32_t out_count;
    uint8_t send_fifo_max;       // max send_fifo level observed
};

extern HapticDiag haptic_diag;

// Serialises diagnostics into buf (little endian) and resets window fields.
// Returns bytes written.
uint16_t diag_fill(uint8_t *buf, uint16_t len);

#endif //DS5_BRIDGE_DIAG_H
