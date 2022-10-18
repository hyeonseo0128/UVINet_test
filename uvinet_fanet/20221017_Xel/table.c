#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>

#include "route.h"
#include "logging.h"


//struct neighbor_table *dummyItem;

int hashCode(int key) {
    return key % TABLE_SIZE;
}

struct neighbor_table* search_neighbor(int key) {
    int hashIndex = hashCode(key);
    while (neighborTable[hashIndex] != NULL) {
        if (neighborTable[hashIndex]->key == key)
            return neighborTable[hashIndex];
        ++hashIndex;
        hashIndex %= TABLE_SIZE;
    }
    return NULL;
}

void insert_neighbor(int key, struct fanet_rc_message data, float link_quality) {
    struct neighbor_table *item = (struct neighbor_table*)malloc(sizeof(struct neighbor_table));
    item->key = key;
    item->item = data;
    item->link_quality = link_quality;
    item->timestamp = (int)time(NULL); //add timestamp
    int hashIndex = hashCode(key);
    while (neighborTable[hashIndex] != NULL && neighborTable[hashIndex]->key != -1) {
        ++hashIndex;
        hashIndex %= TABLE_SIZE;
    }
    neighborTable[hashIndex] = item;
}

struct neighbor_table* delete_neighbor(struct neighbor_table* item) {
    int key = item->key;
    int hashIndex = hashCode(key);
    while (neighborTable[hashIndex] != NULL) {
        if (neighborTable[hashIndex]->key == key) {
            struct neighbor_table* temp = neighborTable[hashIndex];
            free(temp);
            neighborTable[hashIndex] = NULL;//dummyItem;
            return NULL;
            //return temp;
        }
        ++hashIndex;
        hashIndex %= TABLE_SIZE;
    }
    return NULL;
}

void check_neighbor_table_timeout() {
    int i = 0;
    int j = 0;
    char buf[1024];
    int timestamp = (int)time(NULL);
    j += sprintf(buf+j, "Neighbr Table:");
    for (i =0; i < TABLE_SIZE; i++) {
        if (neighborTable[i] != NULL) {
            j += sprintf(buf+j, " (%d, %s, %d)", neighborTable[i]->key, neighborTable[i]->item.ip_address, neighborTable[i]->timestamp);
            //test timestamp value old or not
            if (timestamp - neighborTable[i]->timestamp > VALID_DURATION) {
                j += sprintf(buf+j, " Old Deleted");
                delete_neighbor(neighborTable[i]);
            }
            else {
                j += sprintf(buf+j, " New Preserve");
            }
        }
    }
    //printf("\n");
    LOG_INFO(buf);
}

void enumeration_neighbor_table(char* neighbor_list) {
    int i = 0;
    int j = 0;
    char buf[1024]={0};
    //j += sprintf(buf+j, "Neighbor Table:");
    for (i = 0; i < TABLE_SIZE; i++) {
        if (neighborTable[i] != NULL) {
            j += sprintf(buf+j, "%d,", neighborTable[i]->key);
        }
    }
    LOG_INFO(buf);
    if (strlen(buf) != 0)
        strncpy(neighbor_list, buf, strlen(buf)-1);
}

struct routing_table* search_route(int key) {
    int hashIndex = hashCode(key);
    while (routingTable[hashIndex] != NULL) {
        if (routingTable[hashIndex]->key == key)
            return routingTable[hashIndex];
        ++hashIndex;
        hashIndex %= TABLE_SIZE;
    }
    return NULL;
}

void insert_route(int key, int data, int hopcount) {
    struct routing_table *item = (struct routing_table*)malloc(sizeof(struct routing_table));
    item->key = key;
    item->nexthop = data;
    item->hop_count = hopcount;
    item->timestamp = (int)time(NULL);
    int hashIndex = hashCode(key);
    while (routingTable[hashIndex] != NULL && routingTable[hashIndex]->key != -1) {
        ++hashIndex;
        hashIndex %= TABLE_SIZE;
    }
    routingTable[hashIndex] = item;
}

struct routing_table* delete_route(struct routing_table* item) {
    int key = item->key;
    int hashIndex = hashCode(key);
    while (routingTable[hashIndex] != NULL) {
        if (routingTable[hashIndex]->key == key) {
            struct routing_table* temp = routingTable[hashIndex];
            free(temp);
            routingTable[hashIndex] = NULL; //dummyItem2;
            return NULL;
            //return temp;
        }
        ++hashIndex;
        hashIndex %= TABLE_SIZE;
    }
    return NULL;
}

void enumeration_routing_table(char* routing_list) {
    int i = 0;
    int j = 0;
    char buf[1024]={0};
    //j += sprintf(buf+j, "Neighbor Table:");
    for (i = 0; i < TABLE_SIZE; i++) {
        if (routingTable[i] != NULL) {
            j += sprintf(buf+j, "%d:%d,", routingTable[i]->key, routingTable[i]->nexthop);
        }
    }
    LOG_INFO(buf);
    if (strlen(buf) != 0)
        strncpy(routing_list, buf, strlen(buf)-1);
}

void check_routing_table_timeout() {
    int i = 0;
    int j = 0;
    char buf[1024];
    int timestamp = (int)time(NULL);
    j += sprintf(buf+j, "Routing Table:");
    for (i = 0; i < TABLE_SIZE; i++) {
        if (routingTable[i] != NULL) {
            j += sprintf(buf+j, " (%d, %d, %d, %d)", routingTable[i]->key, routingTable[i]->nexthop, routingTable[i]->hop_count, routingTable[i]->timestamp);
            struct neighbor_table *item_neighbor = search_neighbor(routingTable[i]->nexthop);
            if (item_neighbor == NULL) {
                //j += sprintf(buf+j, " Neighbor Deleted");
                printf("check!!\n");
                delete_route(routingTable[i]);
            }
            //if (timestamp - routingTable[i]->timestamp > (10 * (routingTable[i]->hop_count) + 1) * VALID_DURATION) {
            else if ( (timestamp - routingTable[i]->timestamp) > ((2 * (routingTable[i]->hop_count) + 1) * VALID_DURATION) ) {
                j += sprintf(buf+j, " Old Deleted");
                delete_route(routingTable[i]);
            } else {
                j += sprintf(buf+j, " New Preserve");
            }
        }
    }
    //printf("\n");
    LOG_INFO(buf);
}

int add_route_to_array_and_count(unsigned char *dstArray, unsigned char *nextArray, unsigned char *hopArray) {
    int i = 0;
    int count = 0;
    for (i = 0; i < TABLE_SIZE; i++) {
        if (routingTable[i] != NULL) {
			if (routingTable[i]->hop_count == 0) {
	            dstArray[count] = routingTable[i]->key;
		        nextArray[count] = routingTable[i]->nexthop;
			    hopArray[count] = routingTable[i]->hop_count;
				count++;
			}
        }
    }
    return count;
}

struct position_table* search_position(int key) {
    int hashIndex = hashCode(key);
    while (positionTable[hashIndex] != NULL) {
        if (positionTable[hashIndex]->key == key)
            return positionTable[hashIndex];
        ++hashIndex;
        hashIndex %= TABLE_SIZE;
    }
    return NULL;
}

void insert_position(int key, struct fanet_rc_message data) {
    struct position_table *item = (struct position_table*)malloc(sizeof(struct position_table));
    item->key = key;
    item->x_coord = data.x_coord;
    item->y_coord = data.y_coord;
    item->z_coord = data.z_coord;
    item->timestamp = (int)time(NULL); //add timestamp
    int hashIndex = hashCode(key);
    while (positionTable[hashIndex] != NULL && positionTable[hashIndex]->key != -1) {
        ++hashIndex;
        hashIndex %= TABLE_SIZE;
    }
    positionTable[hashIndex] = item;
}

struct position_table* delete_position(struct position_table* item) {
    int key = item->key;
    int hashIndex = hashCode(key);
    while (positionTable[hashIndex] != NULL) {
        if (positionTable[hashIndex]->key == key) {
            struct position_table* temp = positionTable[hashIndex];
            free(temp);
            positionTable[hashIndex] = NULL;//dummyItem;
            return NULL;
            //return temp;
        }
        ++hashIndex;
        hashIndex %= TABLE_SIZE;
    }
    return NULL;
}

void check_position_table_timeout() {
    int i = 0;
    int j = 0;
    char buf[1024];
    int timestamp = (int)time(NULL);
    j += sprintf(buf+j, "Positon Table:");
    for (i =0; i < TABLE_SIZE; i++) {
        if (positionTable[i] != NULL) {
            j += sprintf(buf+j, " (%d, %f, %f, %f, %d)", positionTable[i]->key, positionTable[i]->x_coord, positionTable[i]->y_coord, positionTable[i]->z_coord, positionTable[i]->timestamp);
            //test timestamp value old or not
            if (timestamp - positionTable[i]->timestamp > VALID_DURATION) {
                j += sprintf(buf+j, " Old");
                delete_position(positionTable[i]);
            }
            else {
                j += sprintf(buf+j, " New");
            }
        }
    }
    //printf("\n");
    LOG_INFO(buf);
}

void display_position_table() {
    int i = 0;
    int j = 0;
    char buf[1024];
    j += sprintf(buf+j, "Positon Table:");
    for (i =0; i < TABLE_SIZE; i++) {
        if (positionTable[i] != NULL) {
            j += sprintf(buf+j, " (%d, %f, %f, %f, %d)", positionTable[i]->key, positionTable[i]->x_coord, positionTable[i]->y_coord, positionTable[i]->z_coord, positionTable[i]->timestamp);
        }
    }
    //printf("\n");
    LOG_INFO(buf);
}
