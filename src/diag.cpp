//
// Haptics pipeline diagnostics. See diag.h.
//
// Layout appended to feature report 0xF9 (offsets inside the returned buffer):
//   2  u8  layout version (1)
//   3  u8  controller connected
//   4  u32 uptime_ms
//   8  u32 usb_frames_in        12 u32 rs_frames_out
//  16  u32 blocks_made          20 u32 blocks_overwritten
//  24  u32 bt39_sent            28 u32 bt_write_fail
//  32  u32 l2cap_send_err       36 u32 out_clipped
//  40  u16 in_peak              42 u16 out_peak
//  44  u16 in_rms  (x32767)     46 u16 out_rms (x32767)
//  48  u8  send_fifo level      49 u8  send_fifo max (window)
//  50  u8  haptics_fifo level   51 u8  packetCounter
//

#include "diag.h"

#include <cmath>
#include <cstring>

#include "bt.h"
#include "pico/time.h"
#include "pico/util/queue.h"

HapticDiag haptic_diag{};

extern queue_t send_fifo;
extern queue_t haptics_fifo;
extern uint8_t packetCounter;

static constexpr uint16_t DIAG_END = 52;

static void put_u32(uint8_t *p, uint32_t v) { memcpy(p, &v, 4); }
static void put_u16(uint8_t *p, uint16_t v) { memcpy(p, &v, 2); }

static uint16_t rms_u16(float sumsq, uint32_t count) {
    if (count == 0) return 0;
    const float rms = sqrtf(sumsq / static_cast<float>(count));
    const float scaled = rms * 32767.0f;
    return static_cast<uint16_t>(scaled > 65535.0f ? 65535.0f : scaled);
}

uint16_t diag_fill(uint8_t *buf, uint16_t len) {
    if (len < DIAG_END) return 0;
    HapticDiag &d = haptic_diag;
    uint8_t *p = buf;
    p[2] = 1;
    p[3] = bt_is_connected() ? 1 : 0;
    put_u32(p + 4, to_ms_since_boot(get_absolute_time()));
    put_u32(p + 8, d.usb_frames_in);
    put_u32(p + 12, d.rs_frames_out);
    put_u32(p + 16, d.blocks_made);
    put_u32(p + 20, d.blocks_overwritten);
    put_u32(p + 24, d.bt39_sent);
    put_u32(p + 28, d.bt_write_fail);
    put_u32(p + 32, d.l2cap_send_err);
    put_u32(p + 36, d.out_clipped);
    put_u16(p + 40, d.in_peak);
    put_u16(p + 42, d.out_peak);
    put_u16(p + 44, rms_u16(d.in_sumsq, d.in_count));
    put_u16(p + 46, rms_u16(d.out_sumsq, d.out_count));
    p[48] = static_cast<uint8_t>(queue_get_level(&send_fifo));
    p[49] = d.send_fifo_max;
    p[50] = static_cast<uint8_t>(queue_get_level(&haptics_fifo));
    p[51] = packetCounter;

    d.in_peak = d.out_peak = 0;
    d.in_sumsq = d.out_sumsq = 0.0f;
    d.in_count = d.out_count = 0;
    d.send_fifo_max = 0;
    return DIAG_END;
}
