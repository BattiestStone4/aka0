#include "vi_capture.hpp"
#include "logger.hpp"
#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include <unistd.h>

#if USE_UART_CAMERA
// ============================================================================
// Linux UART Camera Implementation (ESP32-CAM over UART)
// ============================================================================

#include <time.h>
#include <stdlib.h>

VICapture::VICapture() : m_fd(-1), m_rx_len(0), m_seq(0) {
    memset(m_rx_buf, 0, sizeof(m_rx_buf));
}

VICapture::~VICapture() { deinit(); }

// ---- CRC-16/CCITT ----

static uint16_t crc16Ccitt(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int j = 0; j < 8; j++) {
            if (crc & 0x8000)
                crc = (crc << 1) ^ 0x1021;
            else
                crc <<= 1;
        }
    }
    return crc;
}

// ---- SLIP framing ----

size_t VICapture::slipEncode(const uint8_t *in, size_t in_len, uint8_t *out) {
    size_t pos = 0;
    out[pos++] = UART_SLIP_END;
    for (size_t i = 0; i < in_len; i++) {
        if (in[i] == UART_SLIP_END) {
            out[pos++] = UART_SLIP_ESC;
            out[pos++] = UART_SLIP_ESC_END;
        } else if (in[i] == UART_SLIP_ESC) {
            out[pos++] = UART_SLIP_ESC;
            out[pos++] = UART_SLIP_ESC_ESC;
        } else {
            out[pos++] = in[i];
        }
    }
    out[pos++] = UART_SLIP_END;
    return pos;
}

int VICapture::slipDecode(const uint8_t *in, size_t in_len,
                           uint8_t *out, size_t *out_len) {
    size_t j = 0;
    for (size_t i = 0; i < in_len; i++) {
        if (in[i] == UART_SLIP_ESC) {
            if (++i >= in_len) return -1;
            if (in[i] == UART_SLIP_ESC_END)
                out[j++] = UART_SLIP_END;
            else if (in[i] == UART_SLIP_ESC_ESC)
                out[j++] = UART_SLIP_ESC;
            else
                return -1;
        } else {
            out[j++] = in[i];
        }
    }
    *out_len = j;
    return 0;
}

// ---- UART helpers ----

speed_t VICapture::baudToSpeed(int baud) {
    switch (baud) {
    case 9600:    return B9600;
    case 19200:   return B19200;
    case 38400:   return B38400;
    case 57600:   return B57600;
    case 115200:  return B115200;
    case 230400:  return B230400;
    case 460800:  return B460800;
    case 921600:  return B921600;
    case 1000000: return B1000000;
    case 1500000: return B1500000;
    case 2000000: return B2000000;
    case 2500000: return B2500000;
    case 3000000: return B3000000;
    case 3500000: return B3500000;
    case 4000000: return B4000000;
    default:      return 0;
    }
}

int VICapture::writeAll(const uint8_t *data, size_t len) {
    size_t sent = 0;
    while (sent < len) {
        ssize_t n = write(m_fd, data + sent, len - sent);
        if (n < 0) {
            if (errno == EAGAIN || errno == EINTR) continue;
            return -1;
        }
        sent += n;
    }
    return 0;
}

static int64_t nowMs(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * 1000 + ts.tv_nsec / 1000000;
}

int VICapture::readSlipFrame(int timeout_ms, uint8_t *frame, size_t *frame_len) {
    int64_t deadline = nowMs() + timeout_ms;

    for (;;) {
        size_t start = 0;
        while (start < m_rx_len && m_rx_buf[start] == UART_SLIP_END)
            start++;
        if (start > 0 && start < m_rx_len) {
            memmove(m_rx_buf, m_rx_buf + start, m_rx_len - start);
            m_rx_len -= start;
        } else if (start >= m_rx_len) {
            m_rx_len = 0;
        }

        for (size_t i = 0; i < m_rx_len; i++) {
            if (m_rx_buf[i] == UART_SLIP_END) {
                if (i > 0) {
                    if (slipDecode(m_rx_buf, i, frame, frame_len) == 0) {
                        memmove(m_rx_buf, m_rx_buf + i + 1, m_rx_len - i - 1);
                        m_rx_len -= i + 1;
                        return 0;
                    }
                }
                memmove(m_rx_buf, m_rx_buf + i + 1, m_rx_len - i - 1);
                m_rx_len -= i + 1;
                break;
            }
        }

        int64_t remain = deadline - nowMs();
        if (remain <= 0) return -1;

        struct timeval tv = {
            .tv_sec  = remain / 1000,
            .tv_usec = (remain % 1000) * 1000
        };
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(m_fd, &rfds);

        int ret = select(m_fd + 1, &rfds, NULL, NULL, &tv);
        if (ret > 0) {
            ssize_t n = read(m_fd, m_rx_buf + m_rx_len, UART_RX_BUF_SIZE - m_rx_len);
            if (n > 0) m_rx_len += n;
        }
    }
}

// ---- Protocol layer ----

static size_t buildPacket(uint8_t type, uint8_t seq,
                           const uint8_t *payload, uint16_t payload_len,
                           uint8_t *out) {
    out[0] = type;
    out[1] = seq;
    out[2] = payload_len & 0xFF;
    out[3] = (payload_len >> 8) & 0xFF;
    if (payload_len > 0 && payload)
        memcpy(out + 4, payload, payload_len);
    uint16_t crc = crc16Ccitt(out, 4 + payload_len);
    out[4 + payload_len]     = crc & 0xFF;
    out[4 + payload_len + 1] = (crc >> 8) & 0xFF;
    return 4 + payload_len + 2;
}

static int parsePacket(const uint8_t *data, size_t len,
                        uint8_t *type, uint8_t *seq,
                        const uint8_t **payload, uint16_t *payload_len) {
    if (len < 6) return -1;
    *type = data[0];
    *seq  = data[1];
    *payload_len = data[2] | ((uint16_t)data[3] << 8);
    if (len != (size_t)(4 + *payload_len + 2)) return -1;
    *payload = data + 4;

    uint16_t recv_crc = data[4 + *payload_len] |
                        ((uint16_t)data[4 + *payload_len + 1] << 8);
    uint16_t calc_crc = crc16Ccitt(data, 4 + *payload_len);
    if (recv_crc != calc_crc) return -1;
    return 0;
}

int VICapture::uartSend(uint8_t cmd, const uint8_t *payload, uint16_t payload_len) {
    uint8_t pkt[UART_MAX_PACKET_SIZE];
    size_t pkt_len = buildPacket(cmd, m_seq, payload, payload_len, pkt);

    uint8_t slip_buf[UART_MAX_PACKET_SIZE * 2 + 2];
    size_t slip_len = slipEncode(pkt, pkt_len, slip_buf);

    int ret = writeAll(slip_buf, slip_len);
    if (ret == 0) m_seq = (m_seq + 1) & 0xFF;
    return ret;
}

int VICapture::uartRecv(int timeout_ms, uint8_t *type, uint8_t *seq,
                         const uint8_t **payload, uint16_t *payload_len,
                         uint8_t *frame_buf) {
    size_t frame_len = 0;
    if (readSlipFrame(timeout_ms > 0 ? timeout_ms : UART_DEFAULT_TIMEOUT_MS,
                      frame_buf, &frame_len) != 0)
        return -1;
    return parsePacket(frame_buf, frame_len, type, seq, payload, payload_len);
}

int VICapture::uartRequest(uint8_t cmd, const uint8_t *req, uint16_t req_len,
                            uint8_t *rsp_buf,
                            const uint8_t **rsp_payload, uint16_t *rsp_len) {
    uint8_t sent_seq = m_seq;
    if (uartSend(cmd, req, req_len) != 0) return -1;

    uint8_t rtype, rseq;
    if (uartRecv(0, &rtype, &rseq, rsp_payload, rsp_len, rsp_buf) != 0)
        return -1;
    if (rtype != (cmd | UART_RESP_MASK) || rseq != sent_seq)
        return -1;
    return 0;
}

// ---- VICapture public methods (UART mode) ----

int VICapture::init() {
    const char *dev = getenv("UART_DEV");
    if (!dev) dev = UART_DEFAULT_DEV;
    int baud = UART_DEFAULT_BAUD;
    const char *baud_env = getenv("UART_BAUD");
    if (baud_env) baud = atoi(baud_env);

    m_fd = open(dev, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (m_fd < 0) {
        fprintf(stderr, "[UART] open %s failed: %s\n", dev, strerror(errno));
        return CVI_FAILURE;
    }

    speed_t speed = baudToSpeed(baud);
    if (speed == 0) {
        fprintf(stderr, "[UART] unsupported baud: %d\n", baud);
        close(m_fd); m_fd = -1;
        return CVI_FAILURE;
    }

    struct termios tty;
    tcgetattr(m_fd, &tty);
    cfmakeraw(&tty);
    tty.c_cflag |= CREAD | CLOCAL;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;
    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);
    tcsetattr(m_fd, TCSANOW, &tty);
    tcflush(m_fd, TCIOFLUSH);

    m_rx_len = 0;
    m_seq = 0;

    uint8_t rsp_buf[UART_MAX_PACKET_SIZE];
    const uint8_t *payload;
    uint16_t plen;
    if (uartRequest(UART_CMD_INIT, NULL, 0, rsp_buf, &payload, &plen) != 0) {
        fprintf(stderr, "[UART] init command failed\n");
        close(m_fd); m_fd = -1;
        return CVI_FAILURE;
    }

    LOGI("[UART] camera initialized (dev=%s baud=%d)", dev, baud);
    return CVI_SUCCESS;
}

void VICapture::deinit() {
    if (m_fd >= 0) {
        close(m_fd);
        m_fd = -1;
    }
}

int VICapture::getFrameAsBGR(CVI_U8 chn, cv::Mat& bgr_image) {
    uint8_t rsp_buf[UART_MAX_PACKET_SIZE];
    const uint8_t *payload;
    uint16_t plen;

    if (uartRequest(UART_CMD_GET_CAMERA_FRAME, NULL, 0, rsp_buf, &payload, &plen) != 0)
        return CVI_FAILURE;

    if (plen < 4) return CVI_FAILURE;

    uint32_t frame_len = payload[0] | ((uint32_t)payload[1] << 8) |
                         ((uint32_t)payload[2] << 16) | ((uint32_t)payload[3] << 24);

    if (frame_len == 0 || frame_len > UART_FRAME_MAX_SIZE)
        return CVI_FAILURE;

    uint8_t *data = (uint8_t*)malloc(frame_len);
    if (!data) return CVI_FAILURE;

    size_t received = 0;
    if (plen > 4) {
        size_t extra = plen - 4;
        if (extra > frame_len) extra = frame_len;
        memcpy(data, payload + 4, extra);
        received = extra;
    }

    uint8_t chunk_buf[UART_MAX_PACKET_SIZE];
    while (received < frame_len) {
        uint8_t ctype, cseq;
        const uint8_t *cpayload;
        uint16_t cplen;

        if (uartRecv(UART_CHUNK_TIMEOUT_MS, &ctype, &cseq, &cpayload, &cplen, chunk_buf) != 0) {
            free(data);
            return CVI_FAILURE;
        }
        if (ctype != UART_RESP_FRAME_CHUNK) {
            free(data);
            return CVI_FAILURE;
        }

        size_t to_copy = cplen;
        if (received + to_copy > frame_len)
            to_copy = frame_len - received;
        memcpy(data + received, cpayload, to_copy);
        received += to_copy;
    }

    cv::Mat raw(1, frame_len, CV_8UC1, data);
    bgr_image = cv::imdecode(raw, cv::IMREAD_COLOR);
    free(data);

    if (bgr_image.empty()) return CVI_FAILURE;
    return CVI_SUCCESS;
}

#else
// ============================================================================
// Starry CVI SDK VI Capture Implementation (original)
// ============================================================================

#include <linux/cvi_common.h>
#include <linux/cvi_comm_video.h>
#include "cvi_buffer.h"
#include "cvi_ae_comm.h"
#include "cvi_awb_comm.h"
#include "cvi_comm_isp.h"
#include "cvi_comm_sns.h"
#include "cvi_ae.h"
#include "cvi_awb.h"
#include "cvi_isp.h"
#include "cvi_sns_ctrl.h"
#include "cvi_vpss.h"
#include "cvi_sys.h"
#include "cvi_vi.h"
#include "sample_comm.h"

VICapture::VICapture() {
#if USE_VPSS_RESIZE
    m_VpssGrp = 0;
    m_VpssChn = 0;
    m_bVpssInited = false;
#endif
    memset(&m_stViConfig, 0, sizeof(SAMPLE_VI_CONFIG_S));
    memset(&m_stIniCfg, 0, sizeof(SAMPLE_INI_CFG_S));
}

VICapture::~VICapture() {
    deinit();
#if USE_VPSS_RESIZE
    deinitVpssResize();
#endif
}

int VICapture::init() {
    MMF_VERSION_S stVersion;
    PIC_SIZE_E enPicSize;
    SIZE_S stSize;
    CVI_S32 s32Ret = CVI_SUCCESS;
    LOG_LEVEL_CONF_S log_conf;

    CVI_SYS_GetVersion(&stVersion);
    SAMPLE_PRT("MMF Version:%s\n", stVersion.version);

    log_conf.enModId = CVI_ID_LOG;
    log_conf.s32Level = CVI_DBG_INFO;
    CVI_LOG_SetLevelConf(&log_conf);

    // Get config from ini if found.
    if (SAMPLE_COMM_VI_ParseIni(&m_stIniCfg)) {
        SAMPLE_PRT("Parse complete\n");
    }

    // Set sensor number
    CVI_VI_SetDevNum(m_stIniCfg.devNum);

    /************************************************
   * step1:  Config VI
   ************************************************/
    s32Ret = SAMPLE_COMM_VI_IniToViCfg(&m_stIniCfg, &m_stViConfig);
    if (s32Ret != CVI_SUCCESS)
        return s32Ret;

    /************************************************
   * step2:  Get input size
   ************************************************/
    s32Ret = SAMPLE_COMM_VI_GetSizeBySensor(m_stIniCfg.enSnsType[0], &enPicSize);
    if (s32Ret != CVI_SUCCESS) {
        LOGE("SAMPLE_COMM_VI_GetSizeBySensor failed with %#x", s32Ret);
        return s32Ret;
    }

    s32Ret = SAMPLE_COMM_SYS_GetPicSize(enPicSize, &stSize);
    if (s32Ret != CVI_SUCCESS) {
        LOGE("SAMPLE_COMM_SYS_GetPicSize failed with %#x", s32Ret);
        return s32Ret;
    }

    /************************************************
   * step3:  Init modules
   ************************************************/
    s32Ret = SAMPLE_PLAT_SYS_INIT(stSize);
    if (s32Ret != CVI_SUCCESS) {
        LOGE("sys init failed. s32Ret: 0x%x !", s32Ret);
        return s32Ret;
    }

    s32Ret = SAMPLE_PLAT_VI_INIT(&m_stViConfig);
    if (s32Ret != CVI_SUCCESS) {
        LOGE("vi init failed. s32Ret: 0x%x !", s32Ret);
        return s32Ret;
    }

    return CVI_SUCCESS;
}

void VICapture::deinit() {
    SAMPLE_COMM_VI_DestroyIsp(&m_stViConfig);
    SAMPLE_COMM_VI_DestroyVi(&m_stViConfig);
    SAMPLE_COMM_SYS_Exit();
}

#if USE_VPSS_RESIZE
int VICapture::initVpssResize(int input_w, int input_h, int output_w, int output_h) {
    if (m_bVpssInited)
        return CVI_SUCCESS;

    CVI_S32 s32Ret;
    VPSS_GRP_ATTR_S stVpssGrpAttr;
    VPSS_CHN_ATTR_S stVpssChnAttr;
    CVI_BOOL abChnEnable[VPSS_MAX_PHY_CHN_NUM] = {0};

    // Set group attribute
    memset(&stVpssGrpAttr, 0, sizeof(VPSS_GRP_ATTR_S));
    stVpssGrpAttr.stFrameRate.s32SrcFrameRate = -1;
    stVpssGrpAttr.stFrameRate.s32DstFrameRate = -1;
    stVpssGrpAttr.enPixelFormat = PIXEL_FORMAT_NV21;
    stVpssGrpAttr.u32MaxW = input_w;
    stVpssGrpAttr.u32MaxH = input_h;
    stVpssGrpAttr.u8VpssDev = 0;

    // Create and start VPSS group
    s32Ret = CVI_VPSS_CreateGrp(m_VpssGrp, &stVpssGrpAttr);
    if (s32Ret != CVI_SUCCESS) {
        LOGE("CVI_VPSS_CreateGrp failed: 0x%x", s32Ret);
        return s32Ret;
    }

    // Set channel attribute for resize output
    memset(&stVpssChnAttr, 0, sizeof(VPSS_CHN_ATTR_S));
    stVpssChnAttr.u32Width = output_w;
    stVpssChnAttr.u32Height = output_h;
    stVpssChnAttr.enVideoFormat = VIDEO_FORMAT_LINEAR;
#if USE_VPSS_BGR
    stVpssChnAttr.enPixelFormat = PIXEL_FORMAT_BGR_888;
#else
    stVpssChnAttr.enPixelFormat = PIXEL_FORMAT_NV21;
#endif
    stVpssChnAttr.stFrameRate.s32SrcFrameRate = -1;
    stVpssChnAttr.stFrameRate.s32DstFrameRate = -1;
    stVpssChnAttr.u32Depth = 1;
    stVpssChnAttr.bMirror = CVI_TRUE;
    stVpssChnAttr.bFlip = CVI_TRUE;
    stVpssChnAttr.stAspectRatio.enMode = ASPECT_RATIO_NONE;
    stVpssChnAttr.stNormalize.bEnable = CVI_FALSE;

    s32Ret = CVI_VPSS_SetChnAttr(m_VpssGrp, m_VpssChn, &stVpssChnAttr);
    if (s32Ret != CVI_SUCCESS) {
        LOGE("CVI_VPSS_SetChnAttr failed: 0x%x (Format: %d)", s32Ret, stVpssChnAttr.enPixelFormat);
        // Fallback to NV21 if BGR is not supported (unlikely on this platform)
        LOGW("VPSS BGR_888 not supported? Trying NV21...");
        stVpssChnAttr.enPixelFormat = PIXEL_FORMAT_NV21;
        s32Ret = CVI_VPSS_SetChnAttr(m_VpssGrp, m_VpssChn, &stVpssChnAttr);
        if (s32Ret != CVI_SUCCESS) {
             LOGE("CVI_VPSS_SetChnAttr fallback failed: 0x%x", s32Ret);
             CVI_VPSS_DestroyGrp(m_VpssGrp);
             return s32Ret;
        }
    }


    abChnEnable[m_VpssChn] = CVI_TRUE;
    s32Ret = CVI_VPSS_EnableChn(m_VpssGrp, m_VpssChn);
    if (s32Ret != CVI_SUCCESS) {
        LOGE("CVI_VPSS_EnableChn failed: 0x%x", s32Ret);
        CVI_VPSS_DestroyGrp(m_VpssGrp);
        return s32Ret;
    }

    s32Ret = CVI_VPSS_StartGrp(m_VpssGrp);
    if (s32Ret != CVI_SUCCESS) {
        LOGE("CVI_VPSS_StartGrp failed: 0x%x", s32Ret);
        CVI_VPSS_DisableChn(m_VpssGrp, m_VpssChn);
        CVI_VPSS_DestroyGrp(m_VpssGrp);
        return s32Ret;
    }

    m_bVpssInited = true;
    LOGI("VPSS initialized: %dx%d -> %dx%d (PixelFormat=%s, Flip/Mirror enabled)",
         input_w, input_h, output_w, output_h,
         (stVpssChnAttr.enPixelFormat == PIXEL_FORMAT_BGR_888) ? "BGR_888" : "NV21");
    return CVI_SUCCESS;
}

void VICapture::deinitVpssResize() {
    if (m_bVpssInited) {
        CVI_VPSS_StopGrp(m_VpssGrp);
        CVI_VPSS_DisableChn(m_VpssGrp, m_VpssChn);
        CVI_VPSS_DestroyGrp(m_VpssGrp);
        m_bVpssInited = false;
    }
}
#endif

int VICapture::getFrameAsBGR(CVI_U8 chn, cv::Mat& bgr_image) {
    VIDEO_FRAME_INFO_S stVideoFrame;
    struct timeval t1, t2;
    long get_frame_us, mmap_us, resize_us, cvt_us, flip_us, munmap_us, release_us;

    gettimeofday(&t1, NULL);
    if (CVI_VI_GetChnFrame(0, chn, &stVideoFrame, 3000) == 0) {
        gettimeofday(&t2, NULL);
        get_frame_us = (t2.tv_sec - t1.tv_sec) * 1000000 + (t2.tv_usec - t1.tv_usec);

#if USE_VPSS_RESIZE
        if (!m_bVpssInited) {
             LOGE("VPSS not initialized!");
             CVI_VI_ReleaseChnFrame(0, chn, &stVideoFrame);
             return CVI_FAILURE;
        }

        // Use VPSS hardware to resize
        gettimeofday(&t1, NULL);

        // Send frame to VPSS for hardware resize
        CVI_S32 s32Ret = CVI_VPSS_SendFrame(m_VpssGrp, &stVideoFrame, -1);
        if (s32Ret != CVI_SUCCESS) {
            LOGE("CVI_VPSS_SendFrame failed: 0x%x", s32Ret);
            CVI_VI_ReleaseChnFrame(0, chn, &stVideoFrame);
            return CVI_FAILURE;
        }

        // Get resized frame from VPSS
        VIDEO_FRAME_INFO_S stResizedFrame;
        s32Ret = CVI_VPSS_GetChnFrame(m_VpssGrp, m_VpssChn, &stResizedFrame, 1000);
        if (s32Ret != CVI_SUCCESS) {
            LOGE("CVI_VPSS_GetChnFrame failed: 0x%x", s32Ret);
        }

        gettimeofday(&t2, NULL);
        resize_us = (t2.tv_sec - t1.tv_sec) * 1000000 + (t2.tv_usec - t1.tv_usec);
        mmap_us = 0; // No separate mmap needed with VPSS

        // Get resized frame info
        gettimeofday(&t1, NULL);

        int width = stResizedFrame.stVFrame.u32Width;
        int height = stResizedFrame.stVFrame.u32Height;
        int stride_y = stResizedFrame.stVFrame.u32Stride[0];

        // VPSS Hardware BGR Output Path (Default)
        if (stResizedFrame.stVFrame.enPixelFormat == PIXEL_FORMAT_BGR_888) {
             size_t image_size = stride_y * height;
             // Use Cached Mmap for faster memory copy (vital for performance)
             CVI_VOID* vir_addr = CVI_SYS_MmapCache(stResizedFrame.stVFrame.u64PhyAddr[0], image_size);

             // Invalidate cache to ensure we read fresh data from DRAM
             CVI_SYS_IonInvalidateCache(stResizedFrame.stVFrame.u64PhyAddr[0], vir_addr, image_size);

             if (bgr_image.empty() || bgr_image.rows != height || bgr_image.cols != width || bgr_image.type() != CV_8UC3) {
                 bgr_image.create(height, width, CV_8UC3);
             }

             // Copy from specialized hardware memory to standard BGR Mat
             if (stride_y == width * 3) {
                 memcpy(bgr_image.data, vir_addr, image_size);
             } else {
                 for (int i = 0; i < height; i++) {
                     memcpy(bgr_image.data + i * width * 3, (uint8_t*)vir_addr + i * stride_y, width * 3);
                 }
             }

             CVI_SYS_Munmap(vir_addr, image_size);

             gettimeofday(&t2, NULL);
             cvt_us = (t2.tv_sec - t1.tv_sec) * 1000000 + (t2.tv_usec - t1.tv_usec);
             //printf("[VI-DETAIL] GetFrame: %.1fms, VPSS_Resize+CvB: %.1fms, Cvt: %.1fms (VPSSDirect+Cache)\n",
             //       get_frame_us / 1000.0, resize_us / 1000.0, cvt_us / 1000.0);

             CVI_VPSS_ReleaseChnFrame(m_VpssGrp, m_VpssChn, &stResizedFrame);
             CVI_VI_ReleaseChnFrame(0, chn, &stVideoFrame);
             return CVI_SUCCESS;
        }

        // Fallback for NV21 if BGR failed
        int stride_uv = stResizedFrame.stVFrame.u32Stride[1];

        // Map resized frame memory
        size_t y_size = stride_y * height;
        size_t uv_size = stride_uv * height / 2;
        size_t image_size = y_size + uv_size;

        CVI_VOID* vir_addr = CVI_SYS_MmapCache(stResizedFrame.stVFrame.u64PhyAddr[0], image_size);
        CVI_SYS_IonInvalidateCache(stResizedFrame.stVFrame.u64PhyAddr[0], vir_addr, image_size);

        cv::Mat yuv_continuous(height * 3 / 2, width, CV_8UC1);

        if (stride_y == width && stride_uv == width) {
            memcpy(yuv_continuous.data, vir_addr, width * height * 3 / 2);
        } else {
            for (int i = 0; i < height; i++) {
                memcpy(yuv_continuous.data + i * width, (uint8_t*)vir_addr + i * stride_y, width);
            }
            for (int i = 0; i < height / 2; i++) {
                memcpy(yuv_continuous.data + height * width + i * width, (uint8_t*)vir_addr + y_size + i * stride_uv, width);
            }
        }

        cv::cvtColor(yuv_continuous, bgr_image, cv::COLOR_YUV2BGR_NV21);

        gettimeofday(&t2, NULL);
        cvt_us = (t2.tv_sec - t1.tv_sec) * 1000000 + (t2.tv_usec - t1.tv_usec);

        //printf("[VI-DETAIL] GetFrame: %.1fms, VPSS_Resize: %.1fms, Cvt: %.1fms (Stride=%d)\n",
        //       get_frame_us / 1000.0, resize_us / 1000.0, cvt_us / 1000.0, stride_y);

        CVI_SYS_Munmap(vir_addr, image_size);
        CVI_VPSS_ReleaseChnFrame(m_VpssGrp, m_VpssChn, &stResizedFrame);
        CVI_VI_ReleaseChnFrame(0, chn, &stVideoFrame);

        return CVI_SUCCESS;
#else
        // Software resize path
        size_t image_size = stVideoFrame.stVFrame.u32Length[0] + stVideoFrame.stVFrame.u32Length[1] +
                            stVideoFrame.stVFrame.u32Length[2];
        CVI_VOID* vir_addr;
        CVI_U32 plane_offset;

        int width = stVideoFrame.stVFrame.u32Width;
        int height = stVideoFrame.stVFrame.u32Height;

        // Map physical memory to virtual address
        gettimeofday(&t1, NULL);
        vir_addr = CVI_SYS_Mmap(stVideoFrame.stVFrame.u64PhyAddr[0], image_size);
        CVI_SYS_IonInvalidateCache(stVideoFrame.stVFrame.u64PhyAddr[0], vir_addr, image_size);
        gettimeofday(&t2, NULL);
        mmap_us = (t2.tv_sec - t1.tv_sec) * 1000000 + (t2.tv_usec - t1.tv_usec);

        // Map virtual addresses for each plane
        plane_offset = 0;
        for (int i = 0; i < 3; i++) {
            if (stVideoFrame.stVFrame.u32Length[i] != 0) {
                stVideoFrame.stVFrame.pu8VirAddr[i] = (CVI_U8*)vir_addr + plane_offset;
                plane_offset += stVideoFrame.stVFrame.u32Length[i];
            }
        }

        // Check pixel format and convert accordingly
        // Optimization: Use INTER_NEAREST for faster resize
        cv::Mat yuv_small;
        gettimeofday(&t1, NULL);
        if (stVideoFrame.stVFrame.enPixelFormat == 19) { // NV21
            // NV21: Y plane + VU interleaved plane
            cv::Mat yuv_nv21(height * 3 / 2, width, CV_8UC1, vir_addr);
            // Use INTER_NEAREST - 3x faster than INTER_LINEAR for resize
            cv::resize(yuv_nv21, yuv_small, cv::Size(640, 640 * 3 / 2), 0, 0, cv::INTER_NEAREST);
        } else if (stVideoFrame.stVFrame.enPixelFormat == 18) { // NV12
            cv::Mat yuv_nv12(height * 3 / 2, width, CV_8UC1, vir_addr);
            cv::resize(yuv_nv12, yuv_small, cv::Size(640, 640 * 3 / 2), 0, 0, cv::INTER_NEAREST);
        } else { // Default to I420
            cv::Mat yuv_i420(height * 3 / 2, width, CV_8UC1, vir_addr);
            cv::resize(yuv_i420, yuv_small, cv::Size(640, 640 * 3 / 2), 0, 0, cv::INTER_NEAREST);
        }
        gettimeofday(&t2, NULL);
        resize_us = (t2.tv_sec - t1.tv_sec) * 1000000 + (t2.tv_usec - t1.tv_usec);

        gettimeofday(&t1, NULL);
        if (stVideoFrame.stVFrame.enPixelFormat == 19) { // NV21
            cv::cvtColor(yuv_small, bgr_image, cv::COLOR_YUV2BGR_NV21);
        } else if (stVideoFrame.stVFrame.enPixelFormat == 18) { // NV12
            cv::cvtColor(yuv_small, bgr_image, cv::COLOR_YUV2BGR_NV12);
        } else { // Default to I420
            cv::cvtColor(yuv_small, bgr_image, cv::COLOR_YUV2BGR_I420);
        }
        gettimeofday(&t2, NULL);
        cvt_us = (t2.tv_sec - t1.tv_sec) * 1000000 + (t2.tv_usec - t1.tv_usec);

        LOGD("[VI-DETAIL] GetFrame: %.1fms, Mmap: %.1fms, Resize: %.1fms, Cvt: %.1fms", get_frame_us / 1000.0,
               mmap_us / 1000.0, resize_us / 1000.0, cvt_us / 1000.0);

        // Flip image 180 degrees (camera is upside down)
        gettimeofday(&t1, NULL);
        cv::flip(bgr_image, bgr_image, -1);
        gettimeofday(&t2, NULL);
        flip_us = (t2.tv_sec - t1.tv_sec) * 1000000 + (t2.tv_usec - t1.tv_usec);

        gettimeofday(&t1, NULL);
        CVI_SYS_Munmap(vir_addr, image_size);
        gettimeofday(&t2, NULL);
        munmap_us = (t2.tv_sec - t1.tv_sec) * 1000000 + (t2.tv_usec - t1.tv_usec);

        gettimeofday(&t1, NULL);
        if (CVI_VI_ReleaseChnFrame(0, chn, &stVideoFrame) != 0) {
            LOGE("CVI_VI_ReleaseChnFrame NG");
            return CVI_FAILURE;
        }
        gettimeofday(&t2, NULL);
        release_us = (t2.tv_sec - t1.tv_sec) * 1000000 + (t2.tv_usec - t1.tv_usec);

        LOGD("[VI-DETAIL] Flip: %.1fms, Munmap: %.1fms, Release: %.1fms", flip_us / 1000.0, munmap_us / 1000.0,
               release_us / 1000.0);

        return CVI_SUCCESS;
#endif // USE_VPSS_RESIZE
    }

    LOGE("CVI_VI_GetChnFrame NG");
    return CVI_FAILURE;
}

#endif // USE_UART_CAMERA
