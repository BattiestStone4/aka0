#ifndef VI_CAPTURE_HPP
#define VI_CAPTURE_HPP

#include <opencv2/opencv.hpp>

// ============================================================================
// Camera source selection
// ============================================================================
// USE_UART_CAMERA=0 (default): Starry board, CVI SDK VI hardware capture
// USE_UART_CAMERA=1:            Linux host, UART ESP32-CAM capture
// ============================================================================
#ifndef USE_UART_CAMERA
#define USE_UART_CAMERA 0
#endif

#if USE_UART_CAMERA
// ---- Linux UART mode ----

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/select.h>
#include <sys/time.h>
#include <stdlib.h>

#ifndef CVI_SUCCESS
#define CVI_SUCCESS 0
#endif
#ifndef CVI_FAILURE
#define CVI_FAILURE (-1)
#endif
typedef uint8_t CVI_U8;

#define USE_VPSS_RESIZE 0

// UART protocol constants (shared with ESP32 firmware control.h)
#define UART_CMD_INIT             0x01
#define UART_CMD_GET_CAMERA_INFO  0x02
#define UART_CMD_GET_CAMERA_FRAME 0x03
#define UART_RESP_MASK            0x80
#define UART_RESP_FRAME_CHUNK     0x90
#define UART_SLIP_END             0xC0
#define UART_SLIP_ESC             0xDB
#define UART_SLIP_ESC_END         0xDC
#define UART_SLIP_ESC_ESC         0xDD
#define UART_FRAME_CHUNK_SIZE     4096
#define UART_FRAME_MAX_SIZE       (2 * 1024 * 1024)
#define UART_RX_BUF_SIZE          (64 * 1024)
#define UART_MAX_PACKET_SIZE      (UART_FRAME_CHUNK_SIZE + 64)
#define UART_DEFAULT_DEV          "/dev/ttyUSB0"
#define UART_DEFAULT_BAUD         115200
#define UART_DEFAULT_TIMEOUT_MS   2000
#define UART_CHUNK_TIMEOUT_MS     1000

#else
// ---- Starry CVI SDK mode ----

#include <linux/cvi_type.h>
#include <linux/cvi_common.h>
#include "cvi_vpss.h"
#include "sample_comm.h"

#define USE_VPSS_RESIZE 1

#endif // USE_UART_CAMERA

class VICapture {
public:
    VICapture();
    ~VICapture();

    int init();
    void deinit();

#if USE_UART_CAMERA
    // No-op stubs (ESP32-CAM handles its own resolution)
    int initVpssResize(int, int, int, int) { return CVI_SUCCESS; }
    void deinitVpssResize() {}
#elif USE_VPSS_RESIZE
    int initVpssResize(int input_w, int input_h, int output_w, int output_h);
    void deinitVpssResize();
#endif

    int getFrameAsBGR(CVI_U8 chn, cv::Mat& bgr_image);

private:
#if USE_UART_CAMERA
    int      m_fd;
    uint8_t  m_rx_buf[UART_RX_BUF_SIZE];
    size_t   m_rx_len;
    uint8_t  m_seq;

    static speed_t baudToSpeed(int baud);
    static size_t slipEncode(const uint8_t *in, size_t in_len, uint8_t *out);
    static int slipDecode(const uint8_t *in, size_t in_len,
                          uint8_t *out, size_t *out_len);
    int writeAll(const uint8_t *data, size_t len);
    int readSlipFrame(int timeout_ms, uint8_t *frame, size_t *frame_len);
    int uartSend(uint8_t cmd, const uint8_t *payload, uint16_t payload_len);
    int uartRecv(int timeout_ms, uint8_t *type, uint8_t *seq,
                 const uint8_t **payload, uint16_t *payload_len,
                 uint8_t *frame_buf);
    int uartRequest(uint8_t cmd, const uint8_t *req, uint16_t req_len,
                    uint8_t *rsp_buf, const uint8_t **rsp_payload, uint16_t *rsp_len);
#else
    SAMPLE_VI_CONFIG_S m_stViConfig;
    SAMPLE_INI_CFG_S m_stIniCfg;
#if USE_VPSS_RESIZE
    VPSS_GRP m_VpssGrp;
    VPSS_CHN m_VpssChn;
    bool m_bVpssInited;
#endif
#endif
};

#endif // VI_CAPTURE_HPP
