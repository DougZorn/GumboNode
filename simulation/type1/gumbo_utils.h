#ifndef GUMBO_UTILS_H_
#define GUMBO_UTILS_H_

#include "contiki.h"
#include "lib/list.h"
#include "lib/memb.h"
#include "cooja-radio-cb.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>

typedef int BOOL;
#ifndef TRUE
#define TRUE  1
#define FALSE 0
#endif

/* packet opcodes */
#define QUERY_OPCODE      3
#define BEGIN_OPCODE      2
#define CONFIRM_OPCODE    1
#define CONTINUE_OPCODE   1
#define STOP_OPCODE       0

#define NEW_DATA  1
#define OLD_DATA  0

#define SUCCESS     1
#define END_OF_DATA 0

#define OPCODE(msg)     ( *((gumbo_opcode_t *) msg) )
#define WIDE_DATA(msg)  ( *((gumbo_addr_t *) &msg[sizeof(gumbo_opcode_t)]) )

/* packet types */
typedef uint16_t gumbo_addr_t;
typedef uint16_t gumbo_opcode_t;
typedef uint8_t gumbo_data_t;
typedef uint8_t gumbo_rev_t;

struct gumbo_node_data {
  void *next;
  gumbo_addr_t addr;
  gumbo_data_t data;
  gumbo_rev_t rev;
};

#define MAX_GUMBO_NODES 256
#define MESSAGE_SIZE (sizeof(gumbo_addr_t) + sizeof(gumbo_data_t) + sizeof(gumbo_rev_t))

LIST(gumbo_node_entries);
MEMB(gumbo_mem_pool, struct gumbo_node_data, MAX_GUMBO_NODES);

BOOL is_opcode_packet(const char *);
BOOL is_data_packet(const char *);

int log_and_save(const char *);

void send_query_message(gumbo_addr_t);
void send_confirm_message(gumbo_addr_t);
void send_continue_message(gumbo_addr_t);
void send_stop_message(gumbo_addr_t);

int send_first_data_packet();
int send_next_data_packet();

#endif

