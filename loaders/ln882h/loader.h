// Shared definitions of the LN882H loader: protocol constants, error codes, init stages (the protocol itself is
// described in loader.c).
#pragma once
#include <stdint.h>

#define LN_PROTO 2                      // protocol version, in the banner, the version line and the info answer

// frame bytes on the wire
#define LN_FR_START 0xC0
#define LN_FR_END 0xC1
#define LN_FR_ESC 0xDB

#define LN_MAX_DATA 4096                // the most flash data in one read answer or write request
#define LN_HEAD 5                       // type, seq, status, length (2)
#define LN_MAX_BODY (LN_HEAD + 8 + LN_MAX_DATA + 4)

// commands (an answer has the command's type with bit 7 set; LN_T_ERROR answers a frame that could not be read)
#define LN_C_INFO 0x01
#define LN_C_READ 0x02
#define LN_C_CRC 0x03
#define LN_C_ERASE 0x04
#define LN_C_WRITE 0x05
#define LN_C_STATUS 0x06
#define LN_C_SRWRITE 0x07
#define LN_C_REBOOT 0x08
#define LN_T_ERROR 0xFF

// status codes of an answer; with an error the data is a 32-bit detail (an address or a register value)
#define LN_OK 0
#define LN_E_CRC 1                      // the frame's CRC-32 is wrong
#define LN_E_FRAME 2                    // bad escape, too short, too long, length field wrong
#define LN_E_CMD 3                      // unknown command
#define LN_E_ARGS 4                     // wrong argument length or value
#define LN_E_RANGE 5                    // beyond the flash
#define LN_E_ALIGN 6                    // erase not on whole 4 k sectors
#define LN_E_NOT_ERASED 7               // a write would have to turn 0 bits into 1: detail = first such address
#define LN_E_VERIFY 8                   // the read-back after a write differs: detail = address
#define LN_E_ERASE 9                    // not blank after the erase: detail = address
#define LN_E_WEL 10                     // the flash did not set its write enable latch: detail = status register
#define LN_E_BUSY 11                    // the flash stayed busy: detail = status register
#define LN_E_QSPI 12                    // the QSPI controller did not finish a transfer
#define LN_E_NOFLASH 13                 // no flash answered at start: detail = the JEDEC id read

// init stages: the banner prints each number before the stage runs, so the UART log shows where a start stopped
#define LN_S_UART 1                     // clocks set, UART0 set up from scratch at 115200, interrupts off
#define LN_S_CACHE 2                    // flash cache off
#define LN_S_QSPI 3                     // QSPI controller set up, flash woken from deep power-down
#define LN_S_ID 4                       // JEDEC id read
#define LN_S_DETAILS 5                  // status registers, unique id, MAC from the flash OTP
#define LN_S_READY 6

void loader_init(void);
void loader_poll(void);
