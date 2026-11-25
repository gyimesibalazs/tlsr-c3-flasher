#pragma once
#include "driver/rmt_types.h"
#include "driver/gpio.h"
typedef struct { rmt_tx_done_callback_t on_trans_done; } rmt_tx_event_callbacks_t;
typedef struct { gpio_num_t gpio_num; rmt_clock_source_t clk_src; uint32_t resolution_hz; size_t mem_block_symbols;
                 size_t trans_queue_depth; int intr_priority; struct { uint32_t invert_out : 1; } flags; } rmt_tx_channel_config_t;
typedef struct { int loop_count; struct { uint32_t eot_level : 1; uint32_t queue_nonblocking : 1; } flags; } rmt_transmit_config_t;
typedef struct { int dummy; } rmt_copy_encoder_config_t;
esp_err_t rmt_new_tx_channel(const rmt_tx_channel_config_t *c, rmt_channel_handle_t *ret);
esp_err_t rmt_new_copy_encoder(const rmt_copy_encoder_config_t *c, rmt_encoder_handle_t *ret);
esp_err_t rmt_del_encoder(rmt_encoder_handle_t e);
esp_err_t rmt_tx_register_event_callbacks(rmt_channel_handle_t ch, const rmt_tx_event_callbacks_t *cbs, void *user);
esp_err_t rmt_transmit(rmt_channel_handle_t ch, rmt_encoder_handle_t enc, const void *payload, size_t bytes,
                       const rmt_transmit_config_t *c);
esp_err_t rmt_tx_wait_all_done(rmt_channel_handle_t ch, int timeout_ms);
