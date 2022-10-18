#ifndef __TABLE__
#define __TABLE__

extern struct neighbor_table* neighborTable[TABLE_SIZE];
extern struct routing_table* routingTable[TABLE_SIZE];
extern struct position_table* positionTable[TABLE_SIZE];

int hashCode(int key);
struct neighbor_table* search_neighbor(int key);
void insert_neighbor(int key, struct fanet_rc_message data, float link_quality);
struct neighbor_table* delete_neighbor(struct neighbor_table* item);
void check_neighbor_table_timeout();
void enumeration_neighbor_table(char* neighbor_list);
struct routing_table* search_route(int key);
void insert_route(int key, int data, int hopcount);
struct routing_table* delete_route(struct routing_table* item);

void enumeration_routing_table(char* routing_list);
void check_routing_table_timeout(); 
int add_route_to_array_and_count(unsigned char *dstArray, unsigned char *nextArray, unsigned char *hopArray);
struct position_table* search_position(int key); 
void insert_position(int key, struct fanet_rc_message data); 
struct position_table* delete_position(struct position_table* item);
void check_position_table_timeout();
void display_position_table();

#endif
