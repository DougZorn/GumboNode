#include "contiki.h"
#include "dev/cooja-radio.h"
#include "sys/etimer.h"
#include "sys/ctimer.h"
#include "lib/random.h"
#include "net/packetbuf.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// node states
#define OFF                 0
#define ON                  1

#define MSG                 "420"
#define BUFFER_SIZE         32

PROCESS(gumbo_master, "Controller node.");
PROCESS(gumbo_slave, "Sender and receiver node.");
AUTOSTART_PROCESSES(&gumbo_master);

extern const struct radio_driver cooja_radio_driver;

void gumbo_send(void *arg) {
  cooja_radio_driver.send(MSG, strlen(MSG));
}

PROCESS_THREAD(gumbo_master, ev, data)
{
  PROCESS_BEGIN();

  static struct etimer on_off_timer;
  static clock_time_t on_duty_cycle  = 19 * CLOCK_SECOND;
  static clock_time_t off_duty_cycle = 1 * CLOCK_SECOND;
  
  static int state = OFF;
  
  etimer_set(&on_off_timer, off_duty_cycle);
  
  while (1) {
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&on_off_timer));
    
    if (state == OFF) {
      cooja_radio_driver.init();
      cooja_radio_driver.on();
      process_start(&gumbo_slave, NULL);
     
      etimer_set(&on_off_timer, on_duty_cycle); 
      state = ON;
    } else {
      cooja_radio_driver.off();
      process_exit(&gumbo_slave);
      
      etimer_set(&on_off_timer, off_duty_cycle);
      state = OFF;
    }
  }
    
  PROCESS_END();
}

PROCESS_THREAD(gumbo_slave, ev, data)
{
  PROCESS_BEGIN();
  
  static struct ctimer send_timer;
  clock_time_t send_countdown;
  char buffer[BUFFER_SIZE + 1];
  
  send_countdown = (random_rand() % 5 + 1) * CLOCK_SECOND;
  ctimer_set(&send_timer, send_countdown, gumbo_send, NULL);

  while (1) {
    PROCESS_WAIT_EVENT_UNTIL(packetbuf_totlen() > 0);
    
    if (packetbuf_totlen() > BUFFER_SIZE)
      printf("Received packet, but it is too large to copy.");
    else {
      memset(buffer, '\0', BUFFER_SIZE+1);
      packetbuf_copyto(buffer);
      printf("Received packet: %s\n", buffer);
    }
  }
  
  PROCESS_END();
}
