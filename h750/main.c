/* H750 自动记录器：上电准备 -> 加速度触发 -> 记录10秒 -> 生成可读文件。
 * 相机、TF 均在触发前完成初始化；只采集数据，不进行姿态控制。
 */
#include "stm32h7xx_hal.h"
#include "protocol.h"
#include "timebase.h"
#include "auto_record.h"
#include "avi.h"
#include "camera_perf.h"
#include "camera_test.h"
#include "green_detector.h"
#include "jpeg_enc.h"
#include "jpeg_hw.h"
#include "record_profile.h"
#include "usb_tf.h"
#include "fs_test.h"
#include "ff.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>

#define DART_RECORDING_VERSION "2.1.6-latest"

extern int board_clock_start(void);
extern int board_storage_start(void);
extern unsigned board_sd_ready(void);

typedef enum { WAIT_BOOT, PREPARING, ARMED, RECORDING, SAVING, DONE, ERROR_STATE,
               USB_READER, DISARMED } record_state;
static const char *const state_names[] = {
    "BOOT_WAIT", "PREPARING", "READY", "RECORDING", "SAVING", "DONE", "ERROR", "USB_READ", "DISARMED"
};
static record_state state = WAIT_BOOT;
volatile uint32_t g_boot_stage, g_fault_cfsr, g_fault_hfsr;
static uint32_t link_seq, log_seq, uart_errors, reply_overflows, imu_overflows;
static dart_parser parser;
typedef struct { uint32_t rx_us; dart_imu imu; } imu_entry;
static imu_entry imu_queue[1024] __attribute__((section(".camera_d2"), aligned(32)));
static imu_entry pre_trigger[DART_AUTO_PRE_SAMPLES] __attribute__((section(".camera_d2"), aligned(32)));
/* JPEG输出59392字节，与相机缓冲和掩膜共用AXI；该段加SD缓冲后已满。
 * 硬件编码的条带重排缓冲放DTCM，不增加AXI或D2的占用。 */
static uint8_t jpeg_out[DART_JPEG_OUT_MAX] __attribute__((section(".camera_buffer"), aligned(32)));
static volatile unsigned imu_head, imu_tail, accept_imu;
static dart_packet replies[8];
static volatile unsigned reply_head, reply_tail, remote_started, remote_stopped;
static FATFS fs;
static FIL video, logfile, csvfile, detfile;
static unsigned video_open, log_open, csv_open, det_open, mounted, failed, camera_mode, camera_running;
static unsigned automatic, auto_fired, auto_disabled, first_imu = 1;
static unsigned pre_head, pre_count;
static dart_trigger trigger;
static uint32_t start_us, end_us, trigger_f1_us, prepare_us, session_token;
static uint32_t saved_frames, saved_imu, first_frame, last_frame, biggest, movi_bytes;
static uint32_t last_seq, imu_gaps, max_write_ms, session_imu_overflows, session_crc_errors, session_uart_errors;
static uint32_t start_camera_frames, start_camera_drops, start_camera_invalid;
static uint32_t csv_rows;
static uint32_t det_evals, det_hits, enc_fails;
/* 分阶段耗时（微秒累计）：定位板上 200ms/帧瓶颈，INFO.TXT 与 status 可见。 */
static uint32_t t_detect_us, t_annotate_us, t_encode_us, t_write_us, t_frame_us;
/* 耗时明细放DTCM的普通.bss，不占已满的相机AXI SRAM；只在主循环访问。 */
static record_profile frame_profile;
static uint32_t profile_export_us, profile_export_failed;
static char directory[16], video_path[40], log_path[40], csv_path[40], info_path[40], det_path[40], profile_path[40];
#define MAX_FRAMES 600u
/* 与主机验证管线（tests/onboard_pipeline.c 默认值）一致的编码质量。 */
#define DART_JPEG_QUALITY 75
static uint32_t offsets[MAX_FRAMES], sizes[MAX_FRAMES];
static uint8_t log_buffer[8192], convert_buffer[512];
static unsigned log_used;
static uint32_t sync_sent, sync_ready_time, sync_f1_time;
static volatile unsigned sync_ready;
static unsigned sync_pending;
static volatile uint32_t last_imu_rx;
static uint32_t last_good_frame, last_led_sent;

void _init(void) {} void _fini(void) {}
void SysTick_Handler(void) { HAL_IncTick(); }
uint32_t board_micros(void)
{
    uint32_t irq = __get_PRIMASK();
    __disable_irq();
    uint32_t ms = HAL_GetTick(), cycles = SysTick->VAL, load = SysTick->LOAD;
    if (SCB->ICSR & SCB_ICSR_PENDSTSET_Msk) { ms++; cycles = SysTick->VAL; }
    __set_PRIMASK(irq);
    return dart_systick_micros(ms, load, cycles);
}

static int wire_send(uint8_t type, const void *data, unsigned n)
{
    uint8_t wire[DART_WIRE_MAX];
    size_t length = dart_encode(type, link_seq++, data, n, wire);
    if (!length) return -1;
    for (size_t i = 0; i < length; i++) {
        uint32_t begin = board_micros();
        while (!(USART3->ISR & USART_ISR_TXE_TXFNF))
            if ((uint32_t)(board_micros() - begin) > 2000) { uart_errors++; return -1; }
        USART3->TDR = wire[i];
    }
    return 0;
}

static void reply(const char *fmt, ...)
{
    char text[DART_PAYLOAD_MAX + 1]; va_list args;
    va_start(args, fmt); vsnprintf(text, sizeof(text), fmt, args); va_end(args);
    wire_send(DART_REPLY, text, (unsigned)strlen(text));
}
void bench_puts(const char *s)
{
    char line[DART_PAYLOAD_MAX + 1]; unsigned n = 0;
    while (*s) {
        if (*s == '\r') { s++; continue; }
        if (*s == '\n') { if (n) { wire_send(DART_REPLY, line, n); n = 0; } s++; continue; }
        line[n++] = *s++;
        if (n == DART_PAYLOAD_MAX) { wire_send(DART_REPLY, line, n); n = 0; }
    }
    if (n) wire_send(DART_REPLY, line, n);
}
void HardFault_Handler(void)
{
    g_fault_cfsr = SCB->CFSR; g_fault_hfsr = SCB->HFSR;
    reply("H750 HARDFAULT stage=%lu cfsr=%08lX hfsr=%08lX",
          (unsigned long)g_boot_stage, (unsigned long)g_fault_cfsr, (unsigned long)g_fault_hfsr);
    for (;;) {}
}
static void uart_init(void)
{
    __HAL_RCC_GPIOB_CLK_ENABLE(); __HAL_RCC_USART3_CLK_ENABLE();
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = GPIO_PIN_10 | GPIO_PIN_11; gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_PULLUP; gpio.Speed = GPIO_SPEED_FREQ_HIGH; gpio.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOB, &gpio);
    USART3->CR1 = 0; USART3->CR2 = 0; USART3->CR3 = 0; USART3->PRESC = 0;
    USART3->BRR = (HAL_RCC_GetPCLK1Freq() + 460800) / 921600; USART3->ICR = 0xFFFFFFFF;
    USART3->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE_RXFNEIE;
    /* 串口优先于相机帧结束中断，避免等待 DMA 排空时丢字节。 */
    HAL_NVIC_SetPriority(USART3_IRQn, 1, 0); HAL_NVIC_EnableIRQ(USART3_IRQn);
}
void USART3_IRQHandler(void)
{
    uint32_t flags = USART3->ISR;
    if (flags & (USART_ISR_ORE | USART_ISR_FE | USART_ISR_NE | USART_ISR_PE)) {
        uart_errors++; USART3->ICR = USART_ICR_ORECF | USART_ICR_FECF | USART_ICR_NECF | USART_ICR_PECF;
    }
    if (!(flags & USART_ISR_RXNE_RXFNE)) return;
    dart_packet p;
    if (dart_parse(&parser, (uint8_t)USART3->RDR, &p) != 1) return;
    if (p.type == DART_IMU && p.length == sizeof(dart_imu) && accept_imu) {
        unsigned next = (imu_head + 1) & 1023;
        if (next == imu_tail) { imu_overflows++; return; }
        last_imu_rx = board_micros(); imu_queue[imu_head].rx_us = last_imu_rx;
        memcpy(&imu_queue[imu_head].imu, p.data, sizeof(dart_imu)); __DMB(); imu_head = next;
    } else if (p.type == DART_SYNC && p.length == 8) {
        if (dart_get32(p.data) == sync_sent) {
            sync_ready_time = board_micros(); sync_f1_time = dart_get32(p.data + 4); sync_ready = 1;
        }
    } else {
        if (p.type == DART_REPLY &&
            ((p.length == 11 && !memcmp(p.data, "IMU STARTED", 11)) ||
             (p.length == 12 && !memcmp(p.data, "IMU WATCHING", 12)))) remote_started = 1;
        if (p.type == DART_REPLY && p.length == 11 && !memcmp(p.data, "IMU STOPPED", 11)) remote_stopped = 1;
        unsigned next = (reply_head + 1) & 7;
        if (next == reply_tail) { reply_overflows++; return; }
        replies[reply_head] = p; __DMB(); reply_head = next;
    }
}

static void send_led(void)
{
    unsigned led = state == ARMED ? 2 : state == RECORDING ? 3 : state == SAVING ? 4 :
                   state == DONE ? 5 : state == ERROR_STATE ? 6 : state == USB_READER ? 7 : 1;
    char text[16]; snprintf(text, sizeof(text), "LED %u", led);
    wire_send(DART_COMMAND, text, (unsigned)strlen(text)); last_led_sent = HAL_GetTick();
}
static FRESULT write_exact(FIL *file, const void *data, UINT count)
{
    UINT written = 0; uint32_t begin = HAL_GetTick();
    FRESULT result = f_write(file, data, count, &written);
    uint32_t elapsed = HAL_GetTick() - begin;
    if (elapsed > max_write_ms) max_write_ms = elapsed;
    return result == FR_OK && written != count ? FR_DISK_ERR : result;
}
static int flush_log(void)
{
    if (!log_used) return 0;
    if (write_exact(&logfile, log_buffer, log_used) != FR_OK) { failed = 1; return -1; }
    log_used = 0; return 0;
}
static int log_packet(uint8_t kind, uint32_t time, const void *data, unsigned n)
{
    uint8_t payload[DART_PAYLOAD_MAX], wire[DART_WIRE_MAX];
    if (n + 4 > sizeof(payload)) return -1;
    dart_put32(payload, time); if (n) memcpy(payload + 4, data, n);
    unsigned count = (unsigned)dart_encode(kind, log_seq++, payload, n + 4, wire);
    if (log_used + count > sizeof(log_buffer) && flush_log()) return -1;
    memcpy(log_buffer + log_used, wire, count); log_used += count; return 0;
}
static void log_event(const char *text) { if (log_open) log_packet(19, board_micros(), text, (unsigned)strlen(text)); }
static void save_imu(const imu_entry *entry)
{
    if (first_imu) first_imu = 0;
    else if (entry->imu.sequence != last_seq + 1) imu_gaps += entry->imu.sequence - last_seq - 1;
    last_seq = entry->imu.sequence;
    if (!log_packet(16, entry->rx_us, &entry->imu, sizeof(entry->imu))) saved_imu++;
}
static void sync_request(void)
{
    uint8_t time[4]; sync_sent = board_micros(); dart_put32(time, sync_sent); sync_ready = 0;
    wire_send(DART_SYNC, time, 4); sync_pending = 1;
}

static void begin_recording(const imu_entry *entry)
{
    /* 以确认超过阈值的那个样本为零时刻，不把准备文件的时间计入10秒。 */
    start_us = entry->rx_us; end_us = start_us + DART_AUTO_DURATION_US;
    trigger_f1_us = entry->imu.time_us; state = RECORDING;
    if (automatic) auto_fired = 1;
    start_camera_frames = camera_frames; start_camera_drops = camera_drops; start_camera_invalid = camera_invalid;
    char text[76];
    snprintf(text, sizeof(text), "TRIGGER f1_us=%lu h7_us=%lu auto=%u norm_mg=%lu",
             (unsigned long)trigger_f1_us, (unsigned long)start_us, automatic,
             (unsigned long)dart_accel_norm_mg(entry->imu.accel));
    log_event(text);
    unsigned oldest = (pre_head + DART_AUTO_PRE_SAMPLES - pre_count) % DART_AUTO_PRE_SAMPLES;
    for (unsigned i = 0; i < pre_count && !failed; i++) save_imu(&pre_trigger[(oldest + i) % DART_AUTO_PRE_SAMPLES]);
    pre_count = 0;
    reply("SESSION START id=%06lX duration=10 auto=%u", (unsigned long)session_token, automatic);
    send_led();
}
static void drain_imu(void)
{
    while (imu_tail != imu_head) {
        imu_entry entry = imu_queue[imu_tail]; __DMB(); imu_tail = (imu_tail + 1) & 1023;
        if (state == RECORDING) {
            /* STOP 的往返期间还可能收到数据，只保存截止时刻以前的样本。 */
            if ((int32_t)(entry.rx_us - end_us) < 0) save_imu(&entry);
        } else if (state == PREPARING || state == ARMED) {
            pre_trigger[pre_head] = entry; pre_head = (pre_head + 1) % DART_AUTO_PRE_SAMPLES;
            if (pre_count < DART_AUTO_PRE_SAMPLES) pre_count++;
            if (state == ARMED && dart_trigger_update(&trigger, &entry.imu)) begin_recording(&entry);
        }
    }
}
static void save_frame(const uint8_t *bytes, uint32_t n, uint32_t time)
{
    if ((int32_t)(time - start_us) < 0 || (int32_t)(time - end_us) >= 0) return;
    if (saved_frames >= MAX_FRAMES) { failed = 1; return; }
    uint8_t chunk[8]; memcpy(chunk, "00dc", 4); dart_put32(chunk + 4, n);
    offsets[saved_frames] = 4 + movi_bytes; sizes[saved_frames] = n;
    if (write_exact(&video, chunk, 8) != FR_OK || write_exact(&video, bytes, n) != FR_OK) { failed = 1; return; }
    if (n & 1) { uint8_t zero = 0; if (write_exact(&video, &zero, 1) != FR_OK) { failed = 1; return; } }
    if (!saved_frames) first_frame = time;
    last_frame = time; if (n > biggest) biggest = n;
    uint8_t info[12]; dart_put32(info, saved_frames); dart_put32(info + 4, n); dart_put32(info + 8, camera_frames);
    log_packet(17, time, info, 12); saved_frames++; movi_bytes += 8 + n + (n & 1);
}
/* 板内完整识别管线：检测 -> 标注 -> JPEG 编码 -> AVI 帧落盘 + DETECT.CSV 行。
 * bytes 指向相机 slot 里的 YUV422 帧（slot 已被 camera_take 持有，DMA 不会再写）；
 * 黄圈与坐标文本直接画在该缓冲内，随后整帧编码成 Baseline JPEG。
 * 标注在检测之后进行，因此圆环不会影响检测（阈值判定只看原始帧）。 */
static void process_frame(const uint8_t *bytes, uint32_t time)
{
    if ((int32_t)(time - start_us) < 0 || (int32_t)(time - end_us) >= 0) return;
    if (saved_frames >= MAX_FRAMES) { failed = 1; return; }
    det_evals++;
    uint32_t t0 = board_micros(), t1, t2, t3, t4;
    record_profile_row timing = {0};
    timing.capture_us = time - start_us;
    timing.queue_us = t0 - time;
    timing.avi_index = UINT32_MAX;
    timing.result = PROFILE_PROCESS_ERROR;
    unsigned frame_index = saved_frames;
    dart_detection det;
    int result = green_detect(bytes, &det);
    t1 = board_micros();
    timing.stage_us[0] = t1 - t0;
    if (result) { failed = 1; t4 = t1; goto profile_complete; }
    if (det.found) det_hits++;
    green_annotate((uint8_t *)bytes, &det, (int)saved_frames,
                   (int32_t)(((time - start_us) / 1000u) * 256u));
    t2 = board_micros();
    timing.stage_us[1] = t2 - t1;
    uint32_t jpeg_size = DART_JPEG_OUT_MAX;
    result = jpeg_encode_yuv422(bytes, DART_JPEG_QUALITY, jpeg_out, &jpeg_size);
    t3 = board_micros();
    timing.stage_us[2] = t3 - t2;
    if (result) { enc_fails++; timing.result = PROFILE_ENCODE_ERROR; t4 = t3; goto profile_complete; }
    save_frame(jpeg_out, jpeg_size, time);
    if (det_open) {
        char line[96];
        int32_t cx = det.found ? det.center_x_q16 : 0, cy = det.found ? det.center_y_q16 : 0;
        int n = snprintf(line, sizeof(line), "%u,%ld,%u,%ld.%02lu,%ld.%02lu,%d\r\n",
                         frame_index, (long)(time - start_us), det.found ? 1u : 0u,
                         (long)(cx >> 16), (unsigned long)(((uint32_t)cx & 0xFFFFu) * 100u >> 16),
                         (long)(cy >> 16), (unsigned long)(((uint32_t)cy & 0xFFFFu) * 100u >> 16),
                         det.found ? det.radius : 0);
        if (n < 0 || (unsigned)n >= sizeof(line) || write_exact(&detfile, line, (UINT)n) != FR_OK) failed = 1;
    }
    t4 = board_micros();
    timing.stage_us[3] = t4 - t3;
    timing.result = failed || saved_frames != frame_index + 1 ? PROFILE_WRITE_ERROR : PROFILE_OK;
    if (saved_frames == frame_index + 1) timing.avi_index = frame_index;
profile_complete:
    /* 错误帧也计时；采集期间不新增printf/f_write/串口输出。 */
    timing.total_us = t4 - t0;
    timing.camera_drops = camera_drops - start_camera_drops;
    t_detect_us += timing.stage_us[0]; t_annotate_us += timing.stage_us[1];
    t_encode_us += timing.stage_us[2]; t_write_us += timing.stage_us[3];
    t_frame_us += timing.total_us;
    uint32_t cycle_start = DWT->CYCCNT;
    uint32_t slot = record_profile_append(&frame_profile, &timing);
    record_profile_note_cycles(&frame_profile, slot, DWT->CYCCNT - cycle_start);
}
static void stop_stream(void)
{
    wire_send(DART_COMMAND, "IMU STOP", 8);
    uint32_t begin = board_micros();
    while (!remote_stopped && (uint32_t)(board_micros() - begin) < 30000) {}
    accept_imu = 0;
}
static void release_card(void)
{
    if (mounted) { f_mount(NULL, "0:", 0); mounted = 0; }
    fs_disk_write_enabled = 0; usb_tf_set_busy(0);
}
static void cancel_preparation(void)
{
    /* 仅删除本次尚未触发、由本程序新建的空会话，不碰任何已有采集文件。 */
    stop_stream();
    if (camera_running) { camera_stop(); camera_running = 0; }
    if (video_open) { f_close(&video); f_unlink(video_path); video_open = 0; }
    if (log_open) { f_close(&logfile); f_unlink(log_path); log_open = 0; }
    if (csv_open) { f_close(&csvfile); f_unlink(csv_path); csv_open = 0; }
    if (det_open) { f_close(&detfile); f_unlink(det_path); det_open = 0; }
    if (directory[0]) f_unlink(directory);
    release_card(); pre_count = 0;
}

static int export_csv(void)
{
    /* 高速采集结束后才转换CSV；避免每毫秒格式化文本影响采样和录像。 */
    FIL input; dart_parser decode = {0}; dart_packet packet;
    if (f_open(&input, log_path, FA_READ) != FR_OK) return -1;
    unsigned used = 0; int error = 0; UINT count;
    for (;;) {
        if (f_read(&input, convert_buffer, sizeof(convert_buffer), &count) != FR_OK) { error = 1; break; }
        if (!count) break;
        for (unsigned i = 0; i < count; i++) {
            if (dart_parse(&decode, convert_buffer[i], &packet) != 1) continue;
            if (packet.type != 16 || packet.length != 4 + sizeof(dart_imu)) continue;
            dart_imu sample; memcpy(&sample, packet.data + 4, sizeof(sample));
            char line[256]; int n = dart_imu_csv_line(line, sizeof(line), &sample, trigger_f1_us);
            if (n < 0 || (unsigned)n >= sizeof(line)) { error = 1; break; }
            if (used + (unsigned)n > sizeof(log_buffer)) {
                if (write_exact(&csvfile, log_buffer, used) != FR_OK) { error = 1; break; }
                used = 0;
            }
            memcpy(log_buffer + used, line, (unsigned)n); used += (unsigned)n; csv_rows++;
        }
        if (error) break;
        if (HAL_GetTick() - last_led_sent >= 500) send_led();
    }
    if (used && write_exact(&csvfile, log_buffer, used) != FR_OK) error = 1;
    if (f_close(&input) != FR_OK || decode.errors || decode.count || csv_rows != saved_imu) error = 1;
    return error ? -1 : 0;
}
static int export_profile(void)
{
    /* 相机、IMU及原始文件均已停止/关闭；复用日志缓冲，不在采集期写耗时CSV。 */
    FIL output;
    if (f_open(&output, profile_path, FA_WRITE | FA_CREATE_NEW) != FR_OK) return -1;
    int error = write_exact(&output, record_profile_csv_header,
                            (UINT)strlen(record_profile_csv_header)) != FR_OK;
    unsigned used = 0;
    for (uint32_t i = 0; i < frame_profile.stored && !error; ++i) {
        char line[256];
        int n = record_profile_csv_line(line, sizeof(line), i + 1, &frame_profile.rows[i]);
        if (n < 0 || (unsigned)n >= sizeof(line)) { error = 1; break; }
        if (used + (unsigned)n > sizeof(log_buffer)) {
            if (write_exact(&output, log_buffer, used) != FR_OK) { error = 1; break; }
            used = 0;
        }
        memcpy(log_buffer + used, line, (unsigned)n); used += (unsigned)n;
        if (HAL_GetTick() - last_led_sent >= 500) send_led();
    }
    if (!error && used && write_exact(&output, log_buffer, used) != FR_OK) error = 1;
    if (f_close(&output) != FR_OK) error = 1;
    return error ? -1 : 0;
}
static void write_profile_info(FIL *info)
{
    if (!camera_mode) return;
    char text[1600];
    uint32_t count = frame_profile.attempts;
    uint32_t divisor = count ? count : 1;
    int n = snprintf(text, sizeof(text),
        "\r\n耗时诊断文件：%s\r\n"
        "处理尝试=%lu；明细行=%lu；容量=%u；因容量丢弃明细=%lu（汇总仍包含它们）。\r\n"
        "结果码：0正常=%lu；1图像处理错误=%lu；2编码错误=%lu；3保存错误=%lu。\r\n"
        "下列均值按全部处理尝试计算；未执行的后续阶段记0，失败帧不会被排除。\r\n"
        "排队 平均=%lu us；最大=%lu us（帧完成至开始处理；取最新帧，应不超过一个帧周期约20000 us）。\r\n"
        "图像处理 平均=%lu us；最大=%lu us。\r\n"
        "绘图 平均=%lu us；最大=%lu us。\r\n"
        "JPEG编码 平均=%lu us；最大=%lu us。\r\n"
        "保存 平均=%lu us；最大=%lu us（AVI、原有DETECT文本、可能的DLG缓冲刷写）。\r\n"
        "处理总计 平均=%lu us；最大=%lu us（不含排队）。\r\n"
        "上述为墙钟耗时，包含中断抢占；帧完成时间不是曝光时刻。\r\n"
        "RAM追加平均=%lu cycles；最大=%lu cycles；CPU=%lu Hz。\r\n"
        "cycles只覆盖追加及汇总，不含全部计时开销；不是严格的纯CPU执行时间。\r\n"
        "耗时CSV导出=%lu us；导出失败=%lu；仅在采集结束后写入。\r\n"
        "JPEG后端：" JPEG_ENCODER_BACKEND "；质量=%u；4:2:0。\r\n"
        "JPEG累计重排=%lu us；轮询=%lu us（含FIFO搬运/中断）；最后错误=%lu；HAL错误=0x%08lX。\r\n"
        "TIMING中编码时间包含重排与硬件处理；未启用JPEG MDMA。\r\n",
        profile_path, (unsigned long)count, (unsigned long)frame_profile.stored,
        RECORD_PROFILE_CAPACITY, (unsigned long)frame_profile.dropped_rows,
        (unsigned long)frame_profile.result_count[0], (unsigned long)frame_profile.result_count[1],
        (unsigned long)frame_profile.result_count[2], (unsigned long)frame_profile.result_count[3],
        (unsigned long)(frame_profile.queue_sum / divisor), (unsigned long)frame_profile.queue_max,
        (unsigned long)(frame_profile.stage_sum[0] / divisor), (unsigned long)frame_profile.stage_max[0],
        (unsigned long)(frame_profile.stage_sum[1] / divisor), (unsigned long)frame_profile.stage_max[1],
        (unsigned long)(frame_profile.stage_sum[2] / divisor), (unsigned long)frame_profile.stage_max[2],
        (unsigned long)(frame_profile.stage_sum[3] / divisor), (unsigned long)frame_profile.stage_max[3],
        (unsigned long)(frame_profile.total_sum / divisor), (unsigned long)frame_profile.total_max,
        (unsigned long)(frame_profile.append_cycle_sum / divisor), (unsigned long)frame_profile.append_cycle_max,
        (unsigned long)SystemCoreClock, (unsigned long)profile_export_us, (unsigned long)profile_export_failed,
        DART_JPEG_QUALITY, (unsigned long)jpeg_hw_pack_us, (unsigned long)jpeg_hw_poll_us,
        (unsigned long)jpeg_hw_error, (unsigned long)jpeg_hw_hal_error);
    if (n < 0 || (unsigned)n >= sizeof(text) || write_exact(info, text, (UINT)n) != FR_OK) failed = 1;
}
static void write_info(void)
{
    FIL info;
    if (f_open(&info, info_path, FA_WRITE | FA_CREATE_NEW) != FR_OK) { failed = 1; return; }
    char text[2400];
    int n = snprintf(text, sizeof(text),
        "\xEF\xBB\xBF" "DART 自动记录器 " DART_RECORDING_VERSION "\r\n"
        "会话编号：%lu\r\n启动方式：%s\r\n结果：%s\r\n"
        "触发阈值：三轴加速度合量严格大于 2.000g，连续至少20ms。\r\n"
        "采集时间：触发确认后10秒；IMU另保留至多约0.2秒触发前数据。\r\n"
        "触发时刻：F103=%lu us；H750接收=%lu us（各自开机计时，无真实日期）。\r\n"
        "录像：320x240 MJPEG（机内逐帧绿灯检测，画圈并标注中心坐标）；保存帧数=%lu，JPEG编码失败=%lu。\r\n"
        "IMU原始记录=%lu条；CSV=%lu行；序号缺口=%lu。\r\n"
        "IMU队列溢出=%lu；串口CRC错误=%lu；UART错误=%lu。\r\n"
        "录像期间相机丢帧=%lu；无效帧=%lu；相机错误位=%lu。\r\n"
        "机载识别：评估帧=%lu；命中绿灯帧=%lu；中心坐标详见DETECT.CSV。\r\n"
        "处理耗时（累计）：图像处理=%lums；绘图=%lums；JPEG编码=%lums；保存=%lums；全帧=%lums。\r\n"
        "数据：IMU.CSV可直接用Excel打开，表头含单位；IMU.DLG保留原始数据和视频帧时间。\r\n"
        "CSV时间：以触发确认样本为0秒，负数表示触发前缓存。\r\n"
        "角速度单位：度/秒；加速度单位：g，1g=9.80665m/s^2，静止合量约1g。\r\n"
        "XYZ为传感器原始轴，尚未做机体坐标变换、零偏校准或姿态解算。\r\n"
        "有效标志位：1陀螺有效，2加速度有效，4加速度新数据，8陀螺新数据，16 SPI错误，32采样迟到。\r\n"
        "刷新位只是读取瞬间的状态；1000Hz为定时读取频率，六轴并非硬件同步采样。\r\n"
        "合量触发与姿态无关，横向推动也可能触发；不是仅检测箭体轴向。\r\n"
        "视频帧时间为采集结束时间，非曝光开始时间；两个芯片时间由DLG同步包估算。\r\n"
        "文件时间不代表真实日期；每次上电只自动触发一次。完成后才可断电。\r\n",
        (unsigned long)session_token, automatic ? "加速度自动触发" : "电脑手动命令",
        failed ? "存在错误，请检查计数与原始日志" : "正常结束",
        (unsigned long)trigger_f1_us, (unsigned long)start_us,
        (unsigned long)saved_frames, (unsigned long)enc_fails,
        (unsigned long)saved_imu, (unsigned long)csv_rows, (unsigned long)imu_gaps,
        (unsigned long)(imu_overflows - session_imu_overflows),
        (unsigned long)(parser.errors - session_crc_errors), (unsigned long)(uart_errors - session_uart_errors),
        (unsigned long)(camera_drops - start_camera_drops), (unsigned long)(camera_invalid - start_camera_invalid),
        (unsigned long)camera_errors,
        (unsigned long)det_evals, (unsigned long)det_hits,
        (unsigned long)(t_detect_us / 1000), (unsigned long)(t_annotate_us / 1000),
        (unsigned long)(t_encode_us / 1000), (unsigned long)(t_write_us / 1000),
        (unsigned long)(t_frame_us / 1000));
    if (n < 0 || (unsigned)n >= sizeof(text) || write_exact(&info, text, (UINT)n) != FR_OK) failed = 1;
    write_profile_info(&info);
    if (f_close(&info) != FR_OK) failed = 1;
}
static void close_session(void)
{
    if (camera_running) { camera_stop(); camera_running = 0; }
    stop_stream(); drain_imu();
    if (camera_mode) {
        const uint8_t *bytes; uint32_t n, time; int slot;
        while ((slot = camera_take(&bytes, &n, &time)) >= 0) {
            if (!failed) process_frame(bytes, time); camera_release((unsigned)slot);
        }
        if (!saved_frames) failed = 1;
    }
    state = SAVING; send_led();
    if (video_open && !failed) {
        uint8_t header[224], index[16]; memcpy(index, "idx1", 4); dart_put32(index + 4, saved_frames * 16);
        if (write_exact(&video, index, 8) != FR_OK) failed = 1;
        for (unsigned i = 0; i < saved_frames && !failed; i++) {
            memcpy(index, "00dc", 4); dart_put32(index + 4, 16); dart_put32(index + 8, offsets[i]); dart_put32(index + 12, sizes[i]);
            if (write_exact(&video, index, 16) != FR_OK) failed = 1;
        }
        uint32_t bytes = (uint32_t)f_tell(&video);
        uint32_t span = saved_frames > 1 ? (uint32_t)((uint64_t)(last_frame - first_frame) * saved_frames / (saved_frames - 1)) : 40000;
        if (f_truncate(&video) != FR_OK) failed = 1;
        dart_avi_header(header, saved_frames, span, bytes, movi_bytes, biggest, 320, 240);
        if (f_lseek(&video, 0) != FR_OK || write_exact(&video, header, sizeof(header)) != FR_OK) failed = 1;
    }
    if (video_open) { if (f_close(&video) != FR_OK) failed = 1; video_open = 0; }
    char summary[76];
    snprintf(summary, sizeof(summary), "END frames=%lu imu=%lu gaps=%lu fail=%u",
             (unsigned long)saved_frames, (unsigned long)saved_imu, (unsigned long)imu_gaps, failed);
    log_event(summary);
    if (log_open) {
        if (flush_log() || f_truncate(&logfile) != FR_OK) failed = 1;
        if (f_close(&logfile) != FR_OK) failed = 1;
        log_open = 0;
    }
    if (csv_open) {
        if (export_csv()) failed = 1;
        if (f_truncate(&csvfile) != FR_OK) failed = 1;
        if (f_close(&csvfile) != FR_OK) failed = 1;
        csv_open = 0;
    }
    if (det_open) {
        if (f_truncate(&detfile) != FR_OK) failed = 1;
        if (f_close(&detfile) != FR_OK) failed = 1;
        det_open = 0;
    }
    if (camera_mode) {
        uint32_t export_begin = board_micros();
        profile_export_failed = export_profile() != 0;
        profile_export_us = board_micros() - export_begin;
        if (profile_export_failed) failed = 1;
    }
    /* 转换完成后追加最终标记；FINAL覆盖采集、CSV转换和耗时导出的结果。 */
    if (f_open(&logfile, log_path, FA_WRITE | FA_OPEN_EXISTING) == FR_OK) {
        log_open = 1;
        if (f_lseek(&logfile, f_size(&logfile)) != FR_OK) failed = 1;
        snprintf(summary, sizeof(summary), "FINAL failed=%u csv_rows=%lu", failed, (unsigned long)csv_rows);
        log_event(summary); if (flush_log()) failed = 1;
        if (f_close(&logfile) != FR_OK) failed = 1; log_open = 0;
    } else failed = 1;
    write_info(); release_card(); state = failed ? ERROR_STATE : DONE; send_led();
    reply("SESSION END id=%06lX frames=%lu imu=%lu failed=%u",
          (unsigned long)session_token, (unsigned long)saved_frames, (unsigned long)saved_imu, failed);
    reply("SESSION FILE %s CSV=%lu", directory[0] ? directory : csv_path, (unsigned long)csv_rows);
    reply("SESSION ERR gaps=%lu qdrop=%lu crc=%lu uart=%lu write_ms=%lu",
          (unsigned long)imu_gaps, (unsigned long)(imu_overflows - session_imu_overflows),
          (unsigned long)(parser.errors - session_crc_errors), (unsigned long)(uart_errors - session_uart_errors), (unsigned long)max_write_ms);
}

static int select_paths(uint32_t token, unsigned auto_mode)
{
    directory[0] = 0;
    if (auto_mode) {
        /* FAT 8.3兼容目录：REC00001、REC00002……，永不覆盖已有记录。 */
        DIR root; FILINFO item; uint32_t maximum = 0;
        if (f_opendir(&root, "0:") != FR_OK) return -1;
        FRESULT result;
        while ((result = f_readdir(&root, &item)) == FR_OK && item.fname[0]) {
            if (strlen(item.fname) == 8 && !memcmp(item.fname, "REC", 3) && strspn(item.fname + 3, "0123456789") == 5) {
                uint32_t value = (uint32_t)strtoul(item.fname + 3, NULL, 10); if (value > maximum) maximum = value;
            }
        }
        f_closedir(&root); if (result != FR_OK || maximum >= 99999) return -1;
        session_token = maximum + 1; snprintf(directory, sizeof(directory), "REC%05lu", (unsigned long)session_token);
        if (f_mkdir(directory) != FR_OK) { directory[0] = 0; return -1; }
        snprintf(video_path, sizeof(video_path), "%s/VIDEO.AVI", directory);
        snprintf(log_path, sizeof(log_path), "%s/IMU.DLG", directory);
        snprintf(csv_path, sizeof(csv_path), "%s/IMU.CSV", directory);
        snprintf(info_path, sizeof(info_path), "%s/INFO.TXT", directory);
        snprintf(det_path, sizeof(det_path), "%s/DETECT.CSV", directory);
        snprintf(profile_path, sizeof(profile_path), "%s/TIMING.CSV", directory);
    } else {
        session_token = token;
        snprintf(video_path, sizeof(video_path), "D%06lX.AVI", (unsigned long)token);
        snprintf(log_path, sizeof(log_path), "D%06lX.DLG", (unsigned long)token);
        snprintf(csv_path, sizeof(csv_path), "D%06lX.CSV", (unsigned long)token);
        snprintf(info_path, sizeof(info_path), "D%06lX.TXT", (unsigned long)token);
        snprintf(det_path, sizeof(det_path), "D%06lX.DET", (unsigned long)token);
        snprintf(profile_path, sizeof(profile_path), "D%06lX.PRF", (unsigned long)token);
        FILINFO info;
        const char *paths[] = {video_path, log_path, csv_path, info_path, det_path, profile_path};
        for (unsigned i = 0; i < sizeof(paths) / sizeof(paths[0]); i++)
            if (f_stat(paths[i], &info) != FR_NO_FILE) return -1;
    }
    return 0;
}
static void prepare_session(uint32_t token, unsigned auto_mode, unsigned use_camera)
{
    /* 准备失败时精确报出：哪个步骤(step) + FatFs/驱动错误码(code)。 */
    const char *step = "SD"; int code = 0;
    if (state == RECORDING || state == SAVING || usb_tf_connected() || !board_sd_ready()) { reply("ERR BUSY/USB/SD"); return; }
    automatic = auto_mode; camera_mode = use_camera; failed = 0;
    saved_frames = saved_imu = imu_gaps = movi_bytes = biggest = log_used = log_seq = max_write_ms = csv_rows = 0;
    det_evals = det_hits = enc_fails = 0;
    t_detect_us = t_annotate_us = t_encode_us = t_write_us = t_frame_us = 0;
    record_profile_reset(&frame_profile); profile_export_us = profile_export_failed = 0;
    pre_head = pre_count = 0; first_imu = 1; memset(&trigger, 0, sizeof(trigger));
    session_imu_overflows = imu_overflows; session_crc_errors = parser.errors; session_uart_errors = uart_errors;
    directory[0] = 0; state = PREPARING; send_led(); usb_tf_set_busy(1);
    if (camera_mode) { step = "CAMCONF"; if ((code = camera_configure()) != 0) goto fail; }
    if (camera_mode) { step = "JPEG HW INIT"; if ((code = jpeg_hw_prepare(DART_JPEG_QUALITY)) != 0) goto fail; }
    step = "MOUNT"; if ((code = f_mount(&fs, "0:", 1)) != FR_OK) goto fail;
    mounted = 1; FATFS *volume; DWORD clusters;
    step = "GETFREE"; if ((code = f_getfree("0:", &clusters, &volume)) != FR_OK) goto fail;
    if ((uint64_t)clusters * volume->csize * 512 < 96u * 1024u * 1024u) { step = "SPACE"; code = -1; goto fail; }
    fs_disk_write_enabled = 1;
    step = "PATHS"; if (select_paths(token, auto_mode)) { code = -1; goto fail; }
    if (camera_mode) {
        step = "OPEN AVI"; if ((code = f_open(&video, video_path, FA_WRITE | FA_CREATE_NEW)) != FR_OK) goto fail;
        video_open = 1;
        step = "RESERVE AVI"; if ((code = f_expand(&video, 64u * 1024u * 1024u, 1)) != FR_OK) goto fail;
        uint8_t header[224] = {0};
        step = "WRITE AVI HDR"; if ((code = write_exact(&video, header, sizeof(header))) != FR_OK) goto fail;
    }
    step = "OPEN DLG"; if ((code = f_open(&logfile, log_path, FA_WRITE | FA_CREATE_NEW)) != FR_OK) goto fail;
    log_open = 1;
    step = "RESERVE DLG"; if ((code = f_expand(&logfile, 1024u * 1024u, 1)) != FR_OK) goto fail;
    step = "OPEN CSV"; if ((code = f_open(&csvfile, csv_path, FA_WRITE | FA_CREATE_NEW)) != FR_OK) goto fail;
    csv_open = 1;
    step = "RESERVE CSV"; if ((code = f_expand(&csvfile, 4u * 1024u * 1024u, 1)) != FR_OK) goto fail;
    step = "WRITE CSV HDR"; if ((code = write_exact(&csvfile, dart_imu_csv_header, (UINT)strlen(dart_imu_csv_header))) != FR_OK) goto fail;
    if (camera_mode) {
        /* 机载识别结果：每保存一帧写一行中心坐标，可直接用 Excel 打开。 */
        step = "OPEN DET"; if ((code = f_open(&detfile, det_path, FA_WRITE | FA_CREATE_NEW)) != FR_OK) goto fail;
        det_open = 1;
        step = "RESERVE DET"; if ((code = f_expand(&detfile, 64u * 1024u, 1)) != FR_OK) goto fail;
        static const char det_header[] = "\xEF\xBB\xBF帧号,帧时刻us,检出,质心X_像素,质心Y_像素,半径_像素\r\n";
        step = "WRITE DET HDR"; if ((code = write_exact(&detfile, det_header, (UINT)strlen(det_header))) != FR_OK) goto fail;
    }
    log_event("VERSION " DART_RECORDING_VERSION);
    if (camera_mode) log_event("JPEG " JPEG_ENCODER_BACKEND " quality=75 subsampling=420");
    log_event("gyro=1000Hz/2000dps accel=1600Hz/24g image=320x240/YUV422 fps=measured");
    log_event("CONFIG threshold_mg=2000 hold_us=20000 duration_us=10000000 pre=200");
    if (camera_mode) { step = "CAMSTART"; if ((code = camera_start()) != 0) goto fail; camera_running = 1; }
    remote_started = remote_stopped = 0; imu_head = imu_tail = 0;
    last_imu_rx = last_good_frame = prepare_us = board_micros(); accept_imu = 1;
    wire_send(DART_COMMAND, "IMU WATCH", 9); sync_request();
    reply("SESSION PREPARE auto=%u id=%06lX", automatic, (unsigned long)session_token);
    return;
fail:
    failed = 1; cancel_preparation(); state = ERROR_STATE; send_led();
    reply("ERR PREPARE step=%s code=%d", step, code);
}
static void status(void)
{
    /* 每个串口包最多80字节；版本号独立于计数，避免长计数截断状态。 */
    reply("DART H750 v=" DART_RECORDING_VERSION " state=%s sd=%u", state_names[state], board_sd_ready());
    reply("RECORD frames=%lu imu=%lu", (unsigned long)saved_frames, (unsigned long)saved_imu);
    reply("AUTO threshold=2000mg hold=20ms duration=10s fired=%u disabled=%u", auto_fired, auto_disabled);
    reply("DET eval=%lu hit=%lu encfail=%lu", (unsigned long)det_evals,
          (unsigned long)det_hits, (unsigned long)enc_fails);
    reply("JPEG " JPEG_ENCODER_BACKEND " err=%lu hal=%08lX",
          (unsigned long)jpeg_hw_error, (unsigned long)jpeg_hw_hal_error);
    reply("TIME det=%lums ann=%lums enc=%lums write=%lums frame=%lums",
          (unsigned long)(t_detect_us / 1000), (unsigned long)(t_annotate_us / 1000),
          (unsigned long)(t_encode_us / 1000), (unsigned long)(t_write_us / 1000),
          (unsigned long)(t_frame_us / 1000));
    reply("ERR cam=%lu drop=%lu imuq=%lu crc=%lu uart=%lu", (unsigned long)camera_errors,
          (unsigned long)camera_drops, (unsigned long)imu_overflows, (unsigned long)parser.errors, (unsigned long)uart_errors);
}
static void command(char *text)
{
    unsigned token, seconds; char tail;
    if (!strncmp(text, "PING ", 5)) reply("PONG %s", text + 5);
    else if (!strcmp(text, "STATUS") || !strcmp(text, "AUTO STATUS")) status();
    else if (!strcmp(text, "STOP") || !strcmp(text, "AUTO OFF")) {
        auto_disabled = 1;
        if (state == RECORDING) { end_us = board_micros(); close_session(); }
        else if (state == PREPARING || state == ARMED) { cancel_preparation(); state = DISARMED; send_led(); }
        else { if (state == WAIT_BOOT) state = DISARMED; reply("OK IDLE AUTO_OFF"); }
    } else if (!strcmp(text, "AUTO ARM")) {
        if (auto_fired || state == RECORDING || state == SAVING) reply("ERR AUTO ONE_SHOT; reboot to rearm");
        else { if (state == PREPARING || state == ARMED) cancel_preparation(); auto_disabled = 0; prepare_session(0, 1, 1); }
    } else if ((sscanf(text, "START %x %u %c", &token, &seconds, &tail) == 2 ||
                sscanf(text, "LOG START %x %u %c", &token, &seconds, &tail) == 2) && token <= 0xFFFFFF && seconds == 10) {
        if (state == PREPARING || state == ARMED) cancel_preparation();
        auto_disabled = 1; prepare_session(token, 0, strncmp(text, "LOG ", 4) != 0);
    } else if (!strcmp(text, "CAM ID") && state != RECORDING && state != PREPARING && state != ARMED && !usb_tf_connected()) camera_command("ID");
    else if (!strcmp(text, "CAM TIMING") && state != RECORDING && state != PREPARING && state != ARMED && !usb_tf_connected()) camera_timing();
    else if (!strcmp(text, "CAM INIT") && state != RECORDING && state != PREPARING && state != ARMED && !usb_tf_connected()) {
        int result = camera_configure(); reply("CAM CONFIG result=%d", result); if (!result) camera_timing();
    }
    else if (!strcmp(text, "DISK") && state != RECORDING && state != SAVING && state != PREPARING && !usb_tf_connected()) {
        uint32_t d[13];
        int bad = fs_disk_probe(d);
        reply("DISK ready=%lu sectors=%lu scanned=%lu part_off=%lu cardstate=%lu preverr=%lx",
              (unsigned long)d[0], (unsigned long)d[1], (unsigned long)d[2],
              (unsigned long)d[3], (unsigned long)d[4], (unsigned long)d[5]);
        reply("DISK rawread=%lu lasterr=%lx b0=%02X%02X%02X%02X sig=%04lX%s",
              (unsigned long)d[6], (unsigned long)d[7],
              (unsigned)d[8] & 0xFF, (unsigned)(d[8] >> 8) & 0xFF,
              (unsigned)(d[8] >> 16) & 0xFF, (unsigned)(d[8] >> 24) & 0xFF,
              (unsigned long)d[12], bad ? " FAIL" : " OK");
    }
    else reply("COMMANDS STATUS AUTO OFF/ARM START/LOG START <6HEX> 10 STOP CAM ID DISK");
}
static void process_replies(void)
{
    while (reply_tail != reply_head) {
        dart_packet p = replies[reply_tail]; reply_tail = (reply_tail + 1) & 7;
        if (p.type == DART_COMMAND) { char text[DART_PAYLOAD_MAX + 1]; memcpy(text, p.data, p.length); text[p.length] = 0; command(text); }
    }
}

int main(void)
{
    SCB_EnableICache(); HAL_Init();
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    g_boot_stage = 1; uart_init();
    int clock = board_clock_start(); g_boot_stage = 2; uart_init();
    HAL_Delay(200); int storage = board_storage_start(); g_boot_stage = 3;
    __HAL_RCC_D2SRAM1_CLK_ENABLE(); __HAL_RCC_D2SRAM2_CLK_ENABLE(); __HAL_RCC_D2SRAM3_CLK_ENABLE(); __DSB();
    HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0); uart_init(); usb_tf_start();
    g_boot_stage = 4; uint32_t boot_tick = HAL_GetTick();
    reply("BOOT Dart AutoRecord H750 " DART_RECORDING_VERSION " clock=%d storage=%d", clock, storage);
    if (clock || storage) state = ERROR_STATE;
    for (;;) {
        process_replies();
        if (HAL_GetTick() - last_led_sent >= 500) send_led();
        /* 电脑在原生USB端连接时只读卡，避免拿起板子读文件时误触发。 */
        if (usb_tf_connected() && (state == WAIT_BOOT || state == PREPARING || state == ARMED)) {
            if (state == PREPARING || state == ARMED) cancel_preparation();
            state = USB_READER; send_led();
        }
        if (state == WAIT_BOOT && !auto_disabled && HAL_GetTick() - boot_tick >= 2500) prepare_session(0, 1, 1);
        if (state == PREPARING || state == ARMED || state == RECORDING) {
            drain_imu();
            const uint8_t *bytes; uint32_t n, time; int slot = camera_mode ? camera_take(&bytes, &n, &time) : -1;
            if (slot >= 0) {
                last_good_frame = board_micros();
                if (state == RECORDING) process_frame(bytes, time);
                camera_release((unsigned)slot);
            }
            if (sync_ready && log_open) {
                uint8_t pair[12]; dart_put32(pair, sync_sent); dart_put32(pair + 4, sync_f1_time); dart_put32(pair + 8, sync_ready_time);
                log_packet(18, sync_ready_time, pair, 12); sync_ready = 0; sync_pending = 0;
            }
            if (state == PREPARING && remote_started && pre_count && (!camera_mode || camera_frames)) {
                if (automatic) { state = ARMED; memset(&trigger, 0, sizeof(trigger)); reply("AUTO READY threshold=2000mg hold=20ms folder=%s", directory); send_led(); }
                else { unsigned latest = (pre_head + DART_AUTO_PRE_SAMPLES - 1) % DART_AUTO_PRE_SAMPLES; begin_recording(&pre_trigger[latest]); }
            }
            if (state == PREPARING && (uint32_t)(board_micros() - prepare_us) > 2000000u) failed = 1;
            if (state == ARMED || state == RECORDING) {
                if ((uint32_t)(board_micros() - sync_sent) > 1000000u && !sync_pending) sync_request();
                if (sync_pending && (uint32_t)(board_micros() - sync_sent) > 100000u) sync_pending = 0;
                if (dart_is_stale(board_micros(), last_imu_rx, 1000000u) ||
                    (camera_mode && dart_is_stale(board_micros(), last_good_frame, 1000000u)) || camera_errors) failed = 1;
                if (imu_overflows != session_imu_overflows) failed = 1;
            }
            if (state == RECORDING && (failed || (int32_t)(board_micros() - end_us) >= 0)) close_session();
            else if (failed && (state == PREPARING || state == ARMED)) {
                cancel_preparation(); state = ERROR_STATE; send_led(); reply("ERR AUTO IMU/CAMERA/QUEUE; not armed");
            }
        }
    }
}
