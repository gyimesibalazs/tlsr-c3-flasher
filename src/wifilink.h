// Wi-Fi link: the host can reach the flasher over TCP as well as over USB, with the same command protocol.
//
// The C3 runs as an access point of its own or joins an existing network (station); the setting is kept in NVS and
// set by the host with the WIFI command (SSID and password in hex, so they may hold spaces; they are never printed
// back). A TCP server on WIFI_PORT takes one client at a time: a new connection replaces the old one once it has
// been admitted (a stranger without the key cannot push the current client out). The name is announced over mDNS
// (<name>.local, service _c3flasher._tcp).
//
// The first line of every connection is looked at before anything else: "AUTH <key>" is taken out of the stream
// and checked, never passed on (not to the command parser, not to a bridged target). Without a key set, any other
// first bytes (or none within PROBE_MS) admit the connection and are passed on as they came. With a key set, a
// connection must send "AUTH <key>" first, except while the UART bridge belongs to a TCP client: then connections
// from that client's address need no key (esptool cannot send it), see wifi_bridge_owner.
//
// Output is gathered and sent a line at a time (or when the caller waits for input, or on flush), so that answers
// made of several print calls go as one TCP segment. A client that stops reading for WRITE_STALL_MS, or whose
// connection fails, is dropped; writes to it then return 0.
//
// Starting Wi-Fi gives the UART log's store back to the heap and raises the heap reserve the log leaves free.
#pragma once
#include <Arduino.h>

static const uint16_t WIFI_PORT = 7171;

void wifi_setup();                     // at boot: starts what NVS holds
void wifi_poll();                      // accepts, checks and admits connections; notices a closed client
Stream *wifi_client();                 // the admitted client while connected, or NULL
uint32_t wifi_client_seq();            // changes whenever another client is admitted
bool wifi_alive(Stream *s);            // s is the admitted client and still connected
// The UART bridge was opened by the current TCP client (on = true): from now until it is closed (on = false),
// connections from that client's address are admitted without the key. Closing it drops a client that was admitted
// that way, when a key is set.
void wifi_bridge_owner(bool on);
// what the admitted client's socket takes at once, without waiting (the UART logger's live output); output gathered
// before goes first. Returns the bytes taken (0: none now), -1 when no client is connected.
int wifi_send_now(const uint8_t *d, int n);
void wifi_command(int argc, char **argv, Print &out);   // WIFI ...
