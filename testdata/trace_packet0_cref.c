#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "opus.h"

static uint32_t be32(const unsigned char *p) {
  return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | (uint32_t)p[3];
}

int main(int argc, char **argv) {
  if (argc != 2) {
    fprintf(stderr, "usage: %s <testvector.bit>\n", argv[0]);
    return 2;
  }
  int target_packet_idx = 0;
  const char *target_env = getenv("OPUS_TRACE_PACKET_IDX");
  if (target_env && target_env[0] != '\0') {
    target_packet_idx = atoi(target_env);
    if (target_packet_idx < 0) {
      target_packet_idx = 0;
    }
  }
  FILE *f = fopen(argv[1], "rb");
  if (!f) {
    perror("fopen");
    return 2;
  }
  int err = 0;
  OpusDecoder *dec = opus_decoder_create(48000, 1, &err);
  if (err != OPUS_OK || !dec) {
    fprintf(stderr, "opus_decoder_create failed: %s\n", opus_strerror(err));
    fclose(f);
    return 2;
  }

  int packet_idx = 0;
  while (1) {
    unsigned char hdr[8];
    if (fread(hdr, 1, 8, f) != 8) {
      fprintf(stderr, "short header\n");
      fclose(f);
      opus_decoder_destroy(dec);
      return 2;
    }
    uint32_t len = be32(hdr);
    uint32_t expected_fr = be32(hdr + 4);
    unsigned char *pkt = NULL;
    if (len > 0) {
      pkt = (unsigned char *)malloc(len);
      if (!pkt) {
        fprintf(stderr, "oom\n");
        fclose(f);
        opus_decoder_destroy(dec);
        return 2;
      }
      if (fread(pkt, 1, len, f) != len) {
        fprintf(stderr, "short packet\n");
        free(pkt);
        fclose(f);
        opus_decoder_destroy(dec);
        return 2;
      }
    }
    opus_int16 pcm[5760];
    int n = opus_decode(dec, pkt, (opus_int32)len, pcm, 5760, 0);
    free(pkt);
    if (n < 0) {
      fprintf(stderr, "opus_decode failed: %s\n", opus_strerror(n));
      fclose(f);
      opus_decoder_destroy(dec);
      return 1;
    }
    if (packet_idx == target_packet_idx) {
      opus_uint32 got_fr = 0;
      opus_decoder_ctl(dec, OPUS_GET_FINAL_RANGE(&got_fr));
      fprintf(stderr, "[CREF] packet%d final_range expected=0x%08x got=0x%08x\n", target_packet_idx, expected_fr, got_fr);
      fprintf(stderr, "[CREF] packet%d samples=%d\n", target_packet_idx, n);
      break;
    }
    packet_idx++;
  }
  fclose(f);
  opus_decoder_destroy(dec);
  return 0;
}

