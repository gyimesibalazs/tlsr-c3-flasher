#pragma once
#include <stdint.h>
#include <stddef.h>
typedef int esp_err_t;
#define ESP_OK 0
#define ESP_FAIL -1
typedef union {
  struct {
    uint16_t duration0 : 15;
    uint16_t level0 : 1;
    uint16_t duration1 : 15;
    uint16_t level1 : 1;
  };
  uint32_t val;
} rmt_symbol_word_t;
typedef struct SimRmtChannel *rmt_channel_handle_t;
typedef struct SimRmtEncoder *rmt_encoder_handle_t;
typedef int rmt_clock_source_t;
#define RMT_CLK_SRC_DEFAULT 0
typedef struct { size_t num_symbols; } rmt_tx_done_event_data_t;
typedef struct { rmt_symbol_word_t *received_symbols; size_t num_symbols; struct { uint32_t is_last : 1; } flags; } rmt_rx_done_event_data_t;
typedef bool (*rmt_tx_done_callback_t)(rmt_channel_handle_t, const rmt_tx_done_event_data_t *, void *);
typedef bool (*rmt_rx_done_callback_t)(rmt_channel_handle_t, const rmt_rx_done_event_data_t *, void *);
esp_err_t rmt_enable(rmt_channel_handle_t ch);
esp_err_t rmt_disable(rmt_channel_handle_t ch);
esp_err_t rmt_del_channel(rmt_channel_handle_t ch);
