#include "gumbo_utils.h"

#define THRESHOLD           10
#define WITHIN_THRESHOLD(x) (x < THRESHOLD || x + THRESHOLD > MAX_REVISION)

extern const struct radio_driver cooja_radio_driver;
static int g_data_count = 0;

void send_opcode_message(gumbo_opcode_t, gumbo_addr_t);
void build_packet(struct gumbo_node_data *, char *);
void extract_packet_data(const char *, struct gumbo_node_data *);
int compare_revisions(gumbo_rev_t, gumbo_rev_t);
struct gumbo_node_data *find_entry(gumbo_addr_t);

BOOL is_opcode_packet(const char *msg)
{
  gumbo_opcode_t opcode;
  memcpy(&opcode, msg, sizeof(gumbo_opcode_t));
  
  if (  opcode == QUERY_OPCODE || opcode == CONFIRM_OPCODE ||
        opcode == STOP_OPCODE || opcode == CONTINUE_OPCODE || opcode == BEGIN_OPCODE  )
  
    return TRUE;
  else
    return FALSE;
}

int log_and_save(const char *message)
{
  struct gumbo_node_data buffer;
  struct gumbo_node_data *old_data;
  struct gumbo_node_data *new_data;
  
  extract_packet_data(message, &buffer);
  old_data = find_entry(buffer.addr);
  
  if (old_data == NULL) {
    if (g_data_count == MAX_GUMBO_NODES) {
      old_data = list_chop(gumbo_node_entries);
      memb_free(&gumbo_mem_pool, old_data);
      --g_data_count;
    }
  }
  else if (compare_revisions(buffer.rev, old_data->rev) > 0) {
    list_remove(gumbo_node_entries, old_data);
    memb_free(&gumbo_mem_pool, old_data);
    --g_data_count;
  }
  else      /* we already have the current, or a newer version */
    return OLD_DATA;
  
  new_data = (struct gumbo_node_data *) memb_alloc(&gumbo_mem_pool);
  memcpy(new_data, &buffer, sizeof(struct gumbo_node_data));
  list_push(gumbo_node_entries, new_data);
  ++g_data_count;
  
  printf("New data: {addr: %d, data: %d}\n", new_data->addr, new_data->data);
  
  return NEW_DATA;
}

struct gumbo_node_data *find_entry(gumbo_addr_t addr) {
  struct gumbo_node_data *itr;
  
  itr = list_head(gumbo_node_entries);
  while (itr) {
    if (itr->addr == addr)
      return itr;
    
    itr = list_item_next(itr);
  }
  
  return NULL;
}

void extract_packet_data(const char *buffer, struct gumbo_node_data *dest)
{
  int data_offset, rev_offset;

  data_offset = sizeof(gumbo_addr_t);
  rev_offset = data_offset + sizeof(gumbo_data_t);
  
  memcpy(&dest->addr, buffer, sizeof(gumbo_addr_t));
  memcpy(&dest->data, &buffer[data_offset], sizeof(gumbo_data_t));
  memcpy(&dest->rev, &buffer[rev_offset], sizeof(gumbo_rev_t));
}

int compare_revisions(gumbo_rev_t r1, gumbo_rev_t r2)
{
  int diff, loop_around_cutoff;
  
  if (r1 == r2)
    return 0;
  
  diff = r1 - r2;
  loop_around_cutoff = MAX_REVISION - THRESHOLD;
  if (diff > loop_around_cutoff)
    return MAX_REVISION - diff;
  else if (diff < -loop_around_cutoff)
    return MAX_REVISION + diff;
  else
    return diff;
}

/**********************************************************************
 * command sending functions
 */

void send_query_message(gumbo_addr_t addr)    { send_opcode_message(QUERY_OPCODE, addr); }
void send_confirm_message(gumbo_addr_t addr)  { send_opcode_message(CONFIRM_OPCODE, addr); }
void send_continue_message(gumbo_addr_t addr) { send_opcode_message(CONTINUE_OPCODE, addr); }
void send_stop_message(gumbo_addr_t addr)     { send_opcode_message(STOP_OPCODE, addr); }
void send_begin_message(gumbo_addr_t addr)    { send_opcode_message(BEGIN_OPCODE, addr); }

void send_opcode_message(gumbo_opcode_t opcode, gumbo_addr_t addr)
{
  char buffer[MESSAGE_SIZE];
  int offset = sizeof(gumbo_opcode_t);
  
  memcpy(buffer, &opcode, sizeof(gumbo_opcode_t));
  memcpy(&buffer[offset], &addr, sizeof(gumbo_addr_t));

  //if (cooja_radio_driver.channel_clear())  
    cooja_radio_driver.send(buffer, MESSAGE_SIZE);
}

/**********************************************************************
 * data sending functions
 */

static struct gumbo_node_data *internal_itr = NULL;

int send_first_data_packet() {
  struct gumbo_node_data *head;
  char buffer[MESSAGE_SIZE];
  
  head = list_head(gumbo_node_entries);
  if (!head)
    return END_OF_DATA;

  build_packet(head, buffer);
  cooja_radio_driver.send(buffer, MESSAGE_SIZE);
  
//  internal_itr = list_item_next(head);
  return SUCCESS;
}

int send_next_data_packet() {
  char buffer[MESSAGE_SIZE];

  if (internal_itr == NULL)
    return END_OF_DATA;
    
  build_packet(internal_itr, buffer);
  cooja_radio_driver.send(buffer, MESSAGE_SIZE);
    
  internal_itr = list_item_next(internal_itr);
  return SUCCESS;
}

void build_packet(struct gumbo_node_data *node, char *buffer) {
  int data_offset, rev_offset;
  
  data_offset = sizeof(gumbo_addr_t);
  rev_offset  = data_offset + sizeof(gumbo_data_t);
  
  memcpy(buffer, &node->addr, sizeof(gumbo_addr_t));
  memcpy(&buffer[data_offset], &node->data, sizeof(gumbo_data_t));
  memcpy(&buffer[rev_offset], &node->rev, sizeof(gumbo_rev_t));
}

void add_entry(gumbo_addr_t addr, gumbo_data_t data)
{
  struct gumbo_node_data *itr;
  struct gumbo_node_data *new;
  gumbo_rev_t last_revision_number = 0;
  
  itr = find_entry(addr);
  if (itr == NULL) {
    if (g_data_count == MAX_GUMBO_NODES) {
      itr = list_chop(gumbo_node_entries);
      memb_free(&gumbo_mem_pool, itr);
      --g_data_count;
    }
  } else {
    last_revision_number = itr->rev;
    list_remove(gumbo_node_entries, itr);
    memb_free(&gumbo_mem_pool, itr);
    --g_data_count;
  }
  
  new = (struct gumbo_node_data *) memb_alloc(&gumbo_mem_pool);
  new->addr = addr;
  new->data = data;
  new->rev  = RING_INCREMENT(last_revision_number);
  
  list_push(gumbo_node_entries, new);
  ++g_data_count;
}

/*********************************************
 * temperature reading is a random value
 */


void read_temperature_old(gumbo_addr_t this_addr) {
  gumbo_data_t new_temp = (gumbo_data_t) random_rand() % 100;
  add_entry(this_addr, new_temp);
}

