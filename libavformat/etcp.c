/*
 * Etcp socket protocol
 * Copyright (c) 2013 Luca Barbato
 *
 * This file is part of FFmpeg.
 *
 * FFmpeg is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * FFmpeg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with FFmpeg; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

/**
 * @file
 *
 * Etcp socket url_protocol
 *
 */

#include "libavutil/avstring.h"
#include "libavutil/opt.h"
#include "os_support.h"
#include "network.h"
#include <sys/un.h>
#include <linux/if_ether.h>
#include <net/if.h>
#include "url.h"


typedef struct EtcpContext {
    const AVClass *class;
    struct sockaddr_ll addr;
	uint8_t header[14];
    int timeout;
    int listen;
    int type;
    int fd;
} EtcpContext;

#define OFFSET(x) offsetof(EtcpContext, x)
#define ED AV_OPT_FLAG_DECODING_PARAM|AV_OPT_FLAG_ENCODING_PARAM
static const AVOption etcp_options[] = {
    { "listen",    "Open socket for listening",             OFFSET(listen),  AV_OPT_TYPE_INT,   { .i64 = 0 },                    0,       1, ED },
    { "timeout",   "Timeout in ms",                         OFFSET(timeout), AV_OPT_TYPE_INT,   { .i64 = -1 },                  -1, INT_MAX, ED },
    { "type",      "Socket type",                           OFFSET(type),    AV_OPT_TYPE_INT,   { .i64 = SOCK_STREAM },    INT_MIN, INT_MAX, ED, "type" },
    { "stream",    "Stream (reliable stream-oriented)",     0,               AV_OPT_TYPE_CONST, { .i64 = SOCK_STREAM },    INT_MIN, INT_MAX, ED, "type" },
    { "datagram",  "Datagram (unreliable packet-oriented)", 0,               AV_OPT_TYPE_CONST, { .i64 = SOCK_DGRAM },     INT_MIN, INT_MAX, ED, "type" },
    { "seqpacket", "Seqpacket (reliable packet-oriented",   0,               AV_OPT_TYPE_CONST, { .i64 = SOCK_SEQPACKET }, INT_MIN, INT_MAX, ED, "type" },
    { NULL }
};

static const AVClass etcp_class = {
    .class_name = "etcp",
    .item_name  = av_default_item_name,
    .option     = etcp_options,
    .version    = LIBAVUTIL_VERSION_INT,
};

static int etcp_open(URLContext *h, const char *filename, int flags)
{
    EtcpContext *s = h->priv_data;
    int i, fd, ret;
	memset(&if_idx, 0, sizeof(struct ifreq));
    av_strstart(filename, "etcp:", &filename);
    //s->addr.sun_family = AF_ETCP;
    //av_strlcpy(s->addr.sun_path, filename, sizeof(s->addr.sun_path));

    if ((fd = ff_socket(AF_PACKET, SOCK_RAW, htons(ETH_P_ALL))) < 0)
        return ff_neterrno();
	sscanf(filename, "%02x%02x%02x%02x%02x%02x:%02x:%02x:%02x:%02x:%02x:%02x", 
														(unsigned int *)&s->header[0], (unsigned int *)&s->header[1], (unsigned int *)&s->header[2], 
														(unsigned int *)&s->header[3], (unsigned int *)&s->header[4], (unsigned int *)&s->header[5],
														(unsigned int *)&s->header[6], (unsigned int *)&s->header[7], (unsigned int *)&s->header[8], 
														(unsigned int *)&s->header[9], (unsigned int *)&s->header[10], (unsigned int *)&s->header[11]);
	s->header[12] = 0x08;
	s->header[13] = 0x80;
	printf("HASH: ");
	for (i = 0; i < 14; ++i)
		printf("%x", s->header[i]);
	printf("\n");
    if (s->listen) {
        fd = ff_listen_bind(fd, (struct sockaddr *)&s->addr,
                            sizeof(s->addr), s->timeout, h);
        if (fd < 0) {
            ret = fd;
            goto fail;
        }
    } else {
        //ret = ff_listen_connect(fd, (struct sockaddr *)&s->addr,
        //                        sizeof(s->addr), s->timeout, h, 0);
		int ret = 1;
        if (ret < 0)
            goto fail;
    }

    s->fd = fd;
    return 0;

fail:
    if (s->listen && AVUNERROR(ret) != EADDRINUSE)
        unlink(s->addr.sun_path);
    if (fd >= 0)
        closesocket(fd);
    return ret;
}

static int etcp_read(URLContext *h, uint8_t *buf, int size)
{
	printf("YOOOOOO: %d\n,", size);
    EtcpContext *s = h->priv_data;
    int ret;

    if (!(h->flags & AVIO_FLAG_NONBLOCK)) {
        ret = ff_network_wait_fd(s->fd, 0);
        if (ret < 0)
            return ret;
    }
    ret = recv(s->fd, buf, size, 0);
    return ret < 0 ? ff_neterrno() : ret;
}

static int etcp_write(URLContext *h, const uint8_t *buf, int size)
{
    EtcpContext *s = h->priv_data;
	uint8_t *frame;
    int ret;
	frame = (uint8_t *) malloc(size+14);
	memcpy(frame, s->header, 14);
	memcpy(frame+14, buf, size);
    if (!(h->flags & AVIO_FLAG_NONBLOCK)) {
        ret = ff_network_wait_fd(s->fd, 1);
        if (ret < 0)
            return ret;
    }
    ret = sendto(s->fd, frame, size+14, (struct sockaddr*)&s->addr, sizeof(struct sockaddr_ll) < 0);
	printf("YOOOOOO: %d %d\n,", size, ret);
    return ret < 0 ? ff_neterrno() : ret;
}

static int etcp_close(URLContext *h)
{
    EtcpContext *s = h->priv_data;
    if (s->listen)
        unlink(s->addr.sun_path);
    closesocket(s->fd);
    return 0;
}

static int etcp_get_file_handle(URLContext *h)
{
    EtcpContext *s = h->priv_data;
    return s->fd;
}

URLProtocol ff_etcp_protocol = {
    .name                = "etcp",
    .url_open            = etcp_open,
    .url_read            = etcp_read,
    .url_write           = etcp_write,
    .url_close           = etcp_close,
    .url_get_file_handle = etcp_get_file_handle,
    .priv_data_size      = sizeof(EtcpContext),
    .priv_data_class     = &etcp_class,
    .flags               = URL_PROTOCOL_FLAG_NETWORK,
};
