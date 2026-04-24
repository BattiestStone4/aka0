/*
 * ESP32-CAM UART test tool (pure C, based on uart_client.c)
 *
 * Usage:
 *   ./camera_test ping
 *   ./camera_test init
 *   ./camera_test info
 *   ./camera_test frame [--out /tmp/test.jpg]
 *   ./camera_test bench [-n 50]
 *
 * PINMUX (SG2002):
 *   GPIOP19 -> UART3_TX   devmem 0x030010D4 32 5
 *   GPIOP20 -> UART3_RX   devmem 0x030010D8 32 5
 */

#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/select.h>
#include <sys/time.h>
#include <time.h>
#include <sys/stat.h>
#include <getopt.h>

/* ---- Protocol constants (from control.h) ---- */

#define CMD_INIT            0x01
#define CMD_GET_CAMERA_INFO 0x02
#define CMD_GET_CAMERA_FRAME 0x03
#define CMD_PING            0x7F

#define RESP_MASK           0x80
#define RESP_FRAME_CHUNK    0x90

#define SLIP_END            0xC0
#define SLIP_ESC            0xDB
#define SLIP_ESC_END        0xDC
#define SLIP_ESC_ESC        0xDD

#define FRAME_CHUNK_SIZE    4096
#define FRAME_MAX_SIZE      (2 * 1024 * 1024)

#define DEFAULT_DEV        "/dev/cvi-camera"
#define DEFAULT_BAUD       1500000
#define DEFAULT_TIMEOUT_MS 2000
#define CHUNK_TIMEOUT_MS   1000
#define RX_BUF_SIZE        (64 * 1024)
#define MAX_PACKET_SIZE    (FRAME_CHUNK_SIZE + 64)

struct CameraInfo {
    uint16_t width;
    uint16_t height;
    uint8_t format;
    uint8_t connected;
};

/* ---- CRC-16/CCITT ---- */

static uint16_t crc16_ccitt(const uint8_t *data, size_t len)
{
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

/* ---- SLIP ---- */

static size_t slip_encode(const uint8_t *in, size_t in_len, uint8_t *out)
{
    size_t pos = 0;
    out[pos++] = SLIP_END;
    for (size_t i = 0; i < in_len; i++) {
        if (in[i] == SLIP_END) {
            out[pos++] = SLIP_ESC;
            out[pos++] = SLIP_ESC_END;
        } else if (in[i] == SLIP_ESC) {
            out[pos++] = SLIP_ESC;
            out[pos++] = SLIP_ESC_ESC;
        } else {
            out[pos++] = in[i];
        }
    }
    out[pos++] = SLIP_END;
    return pos;
}

static int slip_decode(const uint8_t *in, size_t in_len,
                       uint8_t *out, size_t *out_len)
{
    size_t j = 0;
    for (size_t i = 0; i < in_len; i++) {
        if (in[i] == SLIP_ESC) {
            if (++i >= in_len) return -1;
            if (in[i] == SLIP_ESC_END)
                out[j++] = SLIP_END;
            else if (in[i] == SLIP_ESC_ESC)
                out[j++] = SLIP_ESC;
            else
                return -1;
        } else {
            out[j++] = in[i];
        }
    }
    *out_len = j;
    return 0;
}

/* ---- UART port ---- */

typedef struct {
    int      fd;
    uint8_t  rx_buf[RX_BUF_SIZE];
    size_t   rx_len;
} uart_port_t;

static speed_t baud_to_speed(int baud)
{
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

static int uart_open(uart_port_t *port, const char *dev, int baud)
{
    port->rx_len = 0;
    port->fd = open(dev, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (port->fd < 0) {
        perror("open");
        return -1;
    }

    speed_t speed = baud_to_speed(baud);
    if (speed == 0) {
        fprintf(stderr, "unsupported baudrate: %d\n", baud);
        close(port->fd);
        return -1;
    }

    struct termios tty;
    tcgetattr(port->fd, &tty);
    cfmakeraw(&tty);
    tty.c_cflag |= CREAD | CLOCAL;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;
    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);
    tcsetattr(port->fd, TCSANOW, &tty);
    tcflush(port->fd, TCIOFLUSH);
    return 0;
}

static void uart_close(uart_port_t *port)
{
    if (port->fd >= 0) {
        close(port->fd);
        port->fd = -1;
    }
}

static int uart_write_all(uart_port_t *port, const uint8_t *data, size_t len)
{
    size_t sent = 0;
    while (sent < len) {
        ssize_t n = write(port->fd, data + sent, len - sent);
        if (n < 0) {
            if (errno == EAGAIN || errno == EINTR) continue;
            perror("write");
            return -1;
        }
        sent += n;
    }
    return 0;
}

static int64_t now_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * 1000 + ts.tv_nsec / 1000000;
}

static int uart_read_slip_frame(uart_port_t *port, int timeout_ms,
                                uint8_t *frame, size_t *frame_len)
{
    int64_t deadline = now_ms() + timeout_ms;

    for (;;) {
        /* Skip leading END markers */
        size_t start = 0;
        while (start < port->rx_len && port->rx_buf[start] == SLIP_END)
            start++;

        if (start > 0 && start < port->rx_len) {
            memmove(port->rx_buf, port->rx_buf + start, port->rx_len - start);
            port->rx_len -= start;
        } else if (start >= port->rx_len) {
            port->rx_len = 0;
        }

        /* Find terminating END */
        for (size_t i = 0; i < port->rx_len; i++) {
            if (port->rx_buf[i] == SLIP_END) {
                if (i > 0) {
                    if (slip_decode(port->rx_buf, i, frame, frame_len) == 0) {
                        memmove(port->rx_buf, port->rx_buf + i + 1,
                                port->rx_len - i - 1);
                        port->rx_len -= i + 1;
                        return 0;
                    }
                }
                memmove(port->rx_buf, port->rx_buf + i + 1,
                        port->rx_len - i - 1);
                port->rx_len -= i + 1;
                break;
            }
        }

        int64_t remain = deadline - now_ms();
        if (remain <= 0) {
            fprintf(stderr, "read frame timeout\n");
            return -1;
        }

        struct timeval tv = {
            .tv_sec  = remain / 1000,
            .tv_usec = (remain % 1000) * 1000
        };
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(port->fd, &rfds);

        int ret = select(port->fd + 1, &rfds, NULL, NULL, &tv);
        if (ret > 0) {
            ssize_t n = read(port->fd, port->rx_buf + port->rx_len,
                             RX_BUF_SIZE - port->rx_len);
            if (n > 0)
                port->rx_len += n;
        }
    }
}

/* ---- Protocol ---- */

typedef struct {
    uart_port_t *port;
    int          timeout_ms;
    uint8_t      seq;
} proto_t;

static void proto_init(proto_t *p, uart_port_t *port, int timeout_ms)
{
    p->port = port;
    p->timeout_ms = timeout_ms;
    p->seq = 0;
}

static size_t build_packet(uint8_t type, uint8_t seq,
                           const uint8_t *payload, uint16_t payload_len,
                           uint8_t *out)
{
    out[0] = type;
    out[1] = seq;
    out[2] = payload_len & 0xFF;
    out[3] = (payload_len >> 8) & 0xFF;
    if (payload_len > 0 && payload)
        memcpy(out + 4, payload, payload_len);
    uint16_t crc = crc16_ccitt(out, 4 + payload_len);
    out[4 + payload_len]     = crc & 0xFF;
    out[4 + payload_len + 1] = (crc >> 8) & 0xFF;
    return 4 + payload_len + 2;
}

static int parse_packet(const uint8_t *data, size_t len,
                        uint8_t *type, uint8_t *seq,
                        const uint8_t **payload, uint16_t *payload_len)
{
    if (len < 6) return -1;
    *type = data[0];
    *seq  = data[1];
    *payload_len = data[2] | ((uint16_t)data[3] << 8);
    if (len != (size_t)(4 + *payload_len + 2)) return -1;
    *payload = data + 4;

    uint16_t recv_crc = data[4 + *payload_len] |
                        ((uint16_t)data[4 + *payload_len + 1] << 8);
    uint16_t calc_crc = crc16_ccitt(data, 4 + *payload_len);
    if (recv_crc != calc_crc) return -1;
    return 0;
}

static int proto_send(proto_t *p, uint8_t cmd,
                      const uint8_t *payload, uint16_t payload_len)
{
    uint8_t pkt[MAX_PACKET_SIZE];
    size_t pkt_len = build_packet(cmd, p->seq, payload, payload_len, pkt);

    uint8_t slip_buf[MAX_PACKET_SIZE * 2 + 2];
    size_t slip_len = slip_encode(pkt, pkt_len, slip_buf);

    int ret = uart_write_all(p->port, slip_buf, slip_len);
    if (ret == 0)
        p->seq = (p->seq + 1) & 0xFF;
    return ret;
}

static int proto_recv(proto_t *p, int timeout_ms,
                      uint8_t *type, uint8_t *seq,
                      const uint8_t **payload, uint16_t *payload_len,
                      uint8_t *frame_buf)
{
    size_t frame_len = 0;
    if (uart_read_slip_frame(p->port, timeout_ms > 0 ? timeout_ms : p->timeout_ms,
                             frame_buf, &frame_len) != 0)
        return -1;
    return parse_packet(frame_buf, frame_len, type, seq, payload, payload_len);
}

static int proto_request(proto_t *p, uint8_t cmd,
                         const uint8_t *req_payload, uint16_t req_len,
                         uint8_t *rsp_buf,
                         const uint8_t **rsp_payload, uint16_t *rsp_len)
{
    uint8_t sent_seq = p->seq;
    if (proto_send(p, cmd, req_payload, req_len) != 0) return -1;

    uint8_t rtype, rseq;
    if (proto_recv(p, 0, &rtype, &rseq, rsp_payload, rsp_len, rsp_buf) != 0) {
        fprintf(stderr, "recv failed\n");
        return -1;
    }
    if (rtype != (cmd | RESP_MASK) || rseq != sent_seq) {
        fprintf(stderr, "unexpected response: type=0x%02X seq=%u, "
                "expected type=0x%02X seq=%u\n",
                rtype, rseq, cmd | RESP_MASK, sent_seq);
        return -1;
    }
    return 0;
}

/* ---- Commands ---- */

static int do_ping(proto_t *p)
{
    uint8_t rsp_buf[MAX_PACKET_SIZE];
    const uint8_t *payload;
    uint16_t plen;
    const uint8_t ping_data[] = "ping";

    struct timeval t0, t1;
    gettimeofday(&t0, NULL);
    if (proto_request(p, CMD_PING, ping_data, 4, rsp_buf, &payload, &plen) != 0)
        return 1;
    gettimeofday(&t1, NULL);
    long ms = (t1.tv_sec - t0.tv_sec) * 1000 + (t1.tv_usec - t0.tv_usec) / 1000;
    printf("[ping] OK  %ldms  payload_len=%u\n", ms, plen);
    return 0;
}

static int do_init(proto_t *p)
{
    uint8_t rsp_buf[MAX_PACKET_SIZE];
    const uint8_t *payload;
    uint16_t plen;

    if (proto_request(p, CMD_INIT, NULL, 0, rsp_buf, &payload, &plen) != 0)
        return 1;
    printf("[init] OK  payload_len=%u\n", plen);
    return 0;
}

static int do_info(proto_t *p)
{
    uint8_t rsp_buf[MAX_PACKET_SIZE];
    const uint8_t *payload;
    uint16_t plen;

    if (proto_request(p, CMD_GET_CAMERA_INFO, NULL, 0, rsp_buf, &payload, &plen) != 0)
        return 1;
    if (plen < sizeof(struct CameraInfo)) {
        fprintf(stderr, "invalid camera info length: %u\n", plen);
        return 1;
    }
    struct CameraInfo ci;
    memcpy(&ci, payload, sizeof(ci));
    printf("[info] %ux%u  format=%u  connected=%u\n",
           ci.width, ci.height, ci.format, ci.connected);
    return 0;
}

static int do_frame(proto_t *p, const char *out_path)
{
    uint8_t rsp_buf[MAX_PACKET_SIZE];
    const uint8_t *payload;
    uint16_t plen;

    struct timeval t0, t1;
    gettimeofday(&t0, NULL);

    if (proto_request(p, CMD_GET_CAMERA_FRAME, NULL, 0, rsp_buf, &payload, &plen) != 0)
        return 1;
    if (plen < 4) {
        fprintf(stderr, "frame response too short\n");
        return 1;
    }
    uint32_t frame_len = payload[0] | ((uint32_t)payload[1] << 8) |
                         ((uint32_t)payload[2] << 16) | ((uint32_t)payload[3] << 24);
    if (frame_len == 0 || frame_len > FRAME_MAX_SIZE) {
        fprintf(stderr, "invalid frame length %u\n", frame_len);
        return 1;
    }

    uint8_t *data = malloc(frame_len);
    if (!data) {
        fprintf(stderr, "malloc failed\n");
        return 1;
    }
    size_t received = 0;

    if (plen > 4) {
        size_t extra = plen - 4;
        if (extra > frame_len) extra = frame_len;
        memcpy(data, payload + 4, extra);
        received = extra;
    }

    uint8_t chunk_buf[MAX_PACKET_SIZE];
    while (received < frame_len) {
        uint8_t ctype, cseq;
        const uint8_t *cpayload;
        uint16_t cplen;

        if (proto_recv(p, CHUNK_TIMEOUT_MS, &ctype, &cseq,
                       &cpayload, &cplen, chunk_buf) != 0) {
            fprintf(stderr, "\nchunk recv failed at %zu/%u\n", received, frame_len);
            free(data);
            return 1;
        }
        if (ctype != RESP_FRAME_CHUNK) {
            fprintf(stderr, "\nunexpected packet type=0x%02X\n", ctype);
            free(data);
            return 1;
        }

        size_t to_copy = cplen;
        if (received + to_copy > frame_len)
            to_copy = frame_len - received;
        memcpy(data + received, cpayload, to_copy);
        received += to_copy;

        printf("\r[frame] %zu/%u (%u%%)",
               received, frame_len, (unsigned)(received * 100 / frame_len));
        fflush(stdout);
    }
    gettimeofday(&t1, NULL);
    long ms = (t1.tv_sec - t0.tv_sec) * 1000 + (t1.tv_usec - t0.tv_usec) / 1000;
    printf("\n[frame] received %u bytes in %ldms\n", frame_len, ms);

    /* Ensure output directory exists */
    {
        char *dir = strdup(out_path);
        char *slash = strrchr(dir, '/');
        if (slash) {
            *slash = 0;
            mkdir(dir, 0755);
        }
        free(dir);
    }

    FILE *f = fopen(out_path, "wb");
    if (!f) {
        perror("fopen");
        free(data);
        return 1;
    }
    fwrite(data, 1, frame_len, f);
    fclose(f);
    free(data);
    printf("[frame] saved to %s\n", out_path);
    return 0;
}

static int do_bench(proto_t *p, int count)
{
    printf("[bench] capturing %d frames...\n", count);
    int ok = 0, fail = 0;
    struct timeval t_start, t_last;
    gettimeofday(&t_start, NULL);
    t_last = t_start;

    for (int i = 0; i < count; i++) {
        uint8_t rsp_buf[MAX_PACKET_SIZE];
        const uint8_t *payload;
        uint16_t plen;

        struct timeval t0, t1;
        gettimeofday(&t0, NULL);

        if (proto_request(p, CMD_GET_CAMERA_FRAME, NULL, 0, rsp_buf, &payload, &plen) != 0) {
            fail++;
            printf("[%d/%d] FAILED\n", i + 1, count);
            continue;
        }
        if (plen < 4) { fail++; continue; }

        uint32_t frame_len = payload[0] | ((uint32_t)payload[1] << 8) |
                             ((uint32_t)payload[2] << 16) | ((uint32_t)payload[3] << 24);

        /* drain chunks */
        size_t received = 0;
        if (plen > 4) received = plen - 4;
        uint8_t chunk_buf[MAX_PACKET_SIZE];
        while (received < frame_len) {
            uint8_t ctype, cseq;
            const uint8_t *cpayload;
            uint16_t cplen;
            if (proto_recv(p, CHUNK_TIMEOUT_MS, &ctype, &cseq, &cpayload, &cplen, chunk_buf) != 0)
                break;
            size_t to_copy = cplen;
            if (received + to_copy > frame_len) to_copy = frame_len - received;
            received += to_copy;
        }

        gettimeofday(&t1, NULL);
        long ms = (t1.tv_sec - t0.tv_sec) * 1000 + (t1.tv_usec - t0.tv_usec) / 1000;
        ok++;

        float fps_inst = 1000.0f / ms;
        float fps_avg = (i + 1) * 1000.0f / ((t1.tv_sec - t_start.tv_sec) * 1000 +
                                                (t1.tv_usec - t_start.tv_usec) / 1000);
        printf("[%d/%d] %u bytes  %ldms  FPS: %.1f  avg: %.1f\n",
               i + 1, count, frame_len, ms, fps_inst, fps_avg);
        t_last = t1;
    }

    printf("\n[bench] done: %d ok, %d failed\n", ok, fail);
    return fail > 0 ? 1 : 0;
}

/* ---- CLI ---- */

static void usage(const char *prog)
{
    fprintf(stderr,
        "ESP32-CAM UART test tool\n\n"
        "Usage: %s [options] <command>\n\n"
        "Options:\n"
        "  --dev DEV       UART device (default: %s)\n"
        "  --baud RATE     Baudrate (default: %d)\n"
        "  --timeout MS    Timeout in ms (default: %d)\n"
        "  --out PATH      Output path for frame (default: /tmp/esp_frame.jpg)\n"
        "  -n COUNT        Frame count for bench (default: 50)\n"
        "\n"
        "Commands:\n"
        "  ping            Send ping\n"
        "  init            Send init\n"
        "  info            Get camera info\n"
        "  frame           Get camera frame as JPEG\n"
        "  bench           Benchmark frame capture\n",
        prog, DEFAULT_DEV, DEFAULT_BAUD, DEFAULT_TIMEOUT_MS);
}

int main(int argc, char **argv)
{
    const char *dev = DEFAULT_DEV;
    int baud = DEFAULT_BAUD;
    int timeout_ms = DEFAULT_TIMEOUT_MS;
    const char *out_path = "/tmp/esp_frame.jpg";
    int bench_count = 50;

    static struct option long_opts[] = {
        {"dev",     required_argument, NULL, 'd'},
        {"baud",    required_argument, NULL, 'b'},
        {"timeout", required_argument, NULL, 't'},
        {"out",     required_argument, NULL, 'o'},
        {"count",   required_argument, NULL, 'n'},
        {"help",    no_argument,       NULL, 'h'},
        {NULL, 0, NULL, 0}
    };

    int opt;
    while ((opt = getopt_long(argc, argv, "d:b:t:o:n:h", long_opts, NULL)) != -1) {
        switch (opt) {
        case 'd': dev = optarg; break;
        case 'b': baud = atoi(optarg); break;
        case 't': timeout_ms = atoi(optarg); break;
        case 'o': out_path = optarg; break;
        case 'n': bench_count = atoi(optarg); break;
        case 'h': usage(argv[0]); return 0;
        default:  usage(argv[0]); return 1;
        }
    }

    if (optind >= argc) {
        usage(argv[0]);
        return 1;
    }
    const char *cmd = argv[optind];

    printf("=== ESP32-CAM Test ===\n");
    printf("Device: %s  Baud: %d  Timeout: %dms\n", dev, baud, timeout_ms);

    uart_port_t port;
    if (uart_open(&port, dev, baud) != 0)
        return 1;

    proto_t proto;
    proto_init(&proto, &port, timeout_ms);

    int ret;
    if (strcmp(cmd, "ping") == 0)
        ret = do_ping(&proto);
    else if (strcmp(cmd, "init") == 0)
        ret = do_init(&proto);
    else if (strcmp(cmd, "info") == 0)
        ret = do_info(&proto);
    else if (strcmp(cmd, "frame") == 0)
        ret = do_frame(&proto, out_path);
    else if (strcmp(cmd, "bench") == 0)
        ret = do_bench(&proto, bench_count);
    else {
        fprintf(stderr, "unknown command: %s\n", cmd);
        ret = 1;
    }

    uart_close(&port);
    return ret;
}
