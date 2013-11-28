#include "contiki.h"
#include "sys/ctimer.h"
#include "sys/etimer.h"
#include "lib/random.h"
#include "cooja-radio-driver-cb.h"

#include <stdio.h>
#include <stdlib.h>

#define ON  1
#define OFF 0

#define MESSAGE_SIZE  8
#define NODE_COUNT    8

typedef uint8_t gumbo_addr_t;

struct gumbo_node_data {
  gumbo_addr_t id;
  uint8_t sensor_reading;
  uint8_t sensor_reading2;
  uint8_t rssi;
  uint8_t hops;
};

static struct gumbo_node_data g_db[NODE_COUNT];

PROCESS(gumbo_master, "Top level gumbo node process.");
PROCESS(gumbo_slave, "Handles the mechanics of sending and receiving packets.");
AUTOSTART_PROCESSES(&gumbo_master);

void set_datapacket_flag(void *);
void send_sync(void);
void receive_handler(const char *, int);
void sample_temperature(void);

extern const struct radio_driver cooja_radio_driver;
static gumbo_addr_t g_node_address;

PROCESS_THREAD(gumbo_master, ev, data)
{
  PROCESS_BEGIN();
  
  g_node_address = (gumbo_addr_t) simMoteID;
  static int state = OFF;

  static struct etimer on_off_timer;            /* turns the radio on and off */
  etimer_set(&on_off_timer, 1 * CLOCK_SECOND);

  static struct ctimer datapacket_flag_timer;   /* sends the data for this node */
  static int dp_flag = 0;
  ctimer_set(&datapacket_flag_timer, 5 * CLOCK_SECOND, set_datapacket_flag, (void * ) &dp_flag);

  cooja_radio_driver.init();

  while (1) {
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&on_off_timer));
    etimer_reset(&on_off_timer);

    switch (state) {
    case OFF:
      cooja_radio_driver.on();
      process_start(&gumbo_slave, NULL);

      if (dp_flag) {                            /* send data packet for this node */
        ctimer_reset(&datapacket_flag_timer);
        send_data_packet(0);
        dp_flag = 0;
      }

      state = ON;
      break;

    case ON:
      process_exit(&gumbo_slave);
      cooja_radio_driver.off();

      state = OFF;
      break;

    default:
      state = OFF;
    }
  }

  PROCESS_END();
}

PROCESS_THREAD(gumbo_slave, ev, data)
{
  PROCESS_BEGIN();

  send_sync();
  sample_temperature();

  while (1) {
    PROCESS_YIELD();
  }

  PROCESS_END();
}

void send_data_packet(gumbo_addr_t addr)
{
  char packet[MESSAGE_SIZE];

  packet[0] = MESSAGE_SIZE;
  packet[1] = 'd';
  packet[2] = g_node_address;
  packet[3] = g_db[addr].id;
  packet[4] = g_db[addr].sensor_reading;
  packet[5] = g_db[addr].sensor_reading2;
  packet[6] = g_db[addr].rssi;
  packet[7] = (addr == 0) ? 0 : g_db[addr].hops + 1;

  cooja_radio_driver.send(packet, MESSAGE_SIZE);
}

void send_sync(void)
{
  char packet[MESSAGE_SIZE];

  packet[0] = MESSAGE_SIZE;
  packet[1] = 'w';
  packet[2] = g_node_address;
  memset(&packet[3], 0, MESSAGE_SIZE - 3);

  cooja_radio_driver.send(packet, MESSAGE_SIZE);
}

void receiver_handler(const char *msg, int len)
{
  if (len > MESSAGE_SIZE) {
    printf("Message in buffer is too large to copy.\n");
    return;
  }

  if (msg[1] == 'w')
    printf("Received sync packet, source: %d.\n", (int) msg[2]);
  else if (msg[1] == 'd')
    printf( "Received data packet: (src: %d, sr: %d, sr2: %d, rssi: %d, hops: %d)\n",
            (int) msg[2], (int) msg[4], (int) msg[5], (int) msg[6], (int) msg[7]  );
  else
    printf("Received unrecognized packet.\n");
}

void set_datapacket_flag(void *arg)
{
  int *ptr_flag;

  ptr_flag = (int *) arg;
  *ptr_flag = 1;
}

void sample_temperature(void) {
  g_db[0].sensor_reading = (uint8_t) random_rand() % 256;
}

