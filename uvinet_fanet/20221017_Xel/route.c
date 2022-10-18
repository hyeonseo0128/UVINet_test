#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <signal.h>
#include <math.h>
#include <time.h>
#include <pthread.h>
#include <libgen.h>
#include <memory.h>
#include <stdbool.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/poll.h>
#include <termios.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/time.h>

#include "route.h"
#include "table.h"
#include "logging.h"
#include "vector.h"
#include "LatLong-UTMconversion.h"


pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
int sd, bcast_sd;
int sd_thread, bcast_sd_thread;
struct sockaddr_in broadcastAddr;
struct sockaddr_in remoteServAddr[TABLE_SIZE];

unsigned char receive_buffer_com1[15000]={0};
unsigned char *mp_com1 = receive_buffer_com1;
int csize_com1 = 0;
int *p_csize_com1 = &csize_com1;
int asize_com1 = 0;
int *p_asize_com1 = &asize_com1;

void process_serial_to_sock(unsigned char **mp, int *p_csize, int *p_asize, unsigned char *receive_buffer, \
                            int sd, struct sockaddr_in *remoteServAddr, int bcast_sd, struct sockaddr_in broadcastAddr); 
 

void* do_serial_data_processing(void *data)
{
    while(TRUE)
    {
        //process_serial_to_sock(&mp_com1, p_csize_com1, p_asize_com1, receive_buffer_com1, sd, remoteServAddr, bcast_sd, broadcastAddr);
        process_serial_to_sock(&mp_com1, p_csize_com1, p_asize_com1, receive_buffer_com1, sd_thread, remoteServAddr, bcast_sd_thread, broadcastAddr);
        usleep(1000);
    }
}

pthread_mutex_t table_update_mutex = PTHREAD_MUTEX_INITIALIZER;

//tas socket
int tas_sockfd;

//notification
void send_control_message(char *msg, int size);
void call_routing_module(char *msg, int size, int id);

struct map_id_to_ip ipmap[TABLE_SIZE]={};


float sign_lat, sign_lon;
float sign_x, sign_y;

char UTMZone[4];
int RefEllipsoid = 23;//WGS-84. See list with file "LatLong- UTM conversion.cpp" for id numbers

void convert_coordinate_to_xy(float lat, float lon, float *x, float *y)
{
    LLtoUTM(RefEllipsoid, lat, lon, y, x, UTMZone);
    LOG_INFO("x: %f, y: %f", *x, *y);

}

void convert_xy_to_coordinate(float x, float y, float *latitude, float *longitude)
{
    UTMtoLL(RefEllipsoid, y, x, UTMZone, latitude, longitude);
    LOG_INFO("latitude: %f, longitude: %f", *latitude, *longitude);
}

float calculate_distance_between_nodes(float nx, float ny, float nz, float dstx, float dsty, float dstz)
{
    float power = pow((dstx-nx), 2) + pow((dsty-ny), 2) + pow((dstz-nz), 2);
    return sqrt(power);
}

int select_closest_to_destination(struct fanet_packet fanet_pkt)
{
    float dstx = fanet_pkt.dst_x_coord;
    float dsty = fanet_pkt.dst_y_coord;
    float dstz = fanet_pkt.dst_z_coord;
    float x,y,z;
    float neighbor_dist[TABLE_SIZE];
    int neighbor_node[TABLE_SIZE];

    int destination_id = fanet_pkt.dst_id[0];
    int i, count = 0;

    for (i = 0; i < TABLE_SIZE; i++) {
        if (neighborTable[i] != NULL && neighborTable[i]->item.node_id == destination_id) {
            LOG_INFO("############################Matching##########################\n");
            return destination_id;
        }
    }

    for (i = 0; i < TABLE_SIZE; i++) {
        if (neighborTable[i] != NULL) {
            x = neighborTable[i]->item.x_coord;
            y = neighborTable[i]->item.y_coord;
            z = neighborTable[i]->item.z_coord;
            neighbor_dist[count] = calculate_distance_between_nodes(x, y, z, dstx, dsty, dstz);
            neighbor_node[count] = neighborTable[i]->item.node_id;
            LOG_INFO("Neighbor distance: %f %d", neighbor_dist[count], neighbor_node[count]);
            LOG_INFO("Location information: %f %f %f %f %f %f", x, y, z, dstx, dsty, dstz);
            count++;
        }
    }

    if (count == 0)
        return -1;

    int j = 0;
    float min = neighbor_dist[0];
    int node_id = neighbor_node[0];
    if (count == 1) {
    } else {
        for (j = 1; j < count; j++) {
            if (neighbor_dist[j] < min) {
                min = neighbor_dist[j];
                node_id = neighbor_node[j];
            }
        }
    }

    float mydist = calculate_distance_between_nodes(my_x, my_y, my_z, dstx, dsty, dstz);
    if (min <= mydist) {
        LOG_INFO("Selected nodeid %d, Min distance %f", node_id, min);
        return node_id;
    } else {
        return -1;
    }
}

int find_multicast_route(struct fanet_packet fanet_pkt, int *selected_neighbor_node)
{
    int selected_neighbor_count = 0;
    int destination_count = fanet_pkt.dst_count;
    int dst_index;

    for (dst_index = 0; dst_index < destination_count; dst_index++) {
        struct routing_table *item_route = search_route(fanet_pkt.dst_id[dst_index]);
        if (item_route != NULL) {
            selected_neighbor_node[dst_index] = item_route->nexthop;
            selected_neighbor_count++;
        } else {
            selected_neighbor_node[dst_index] = -1;
        }
    }
    
    return selected_neighbor_count;
}

int build_multicast_packet_and_send(int sd, struct sockaddr_in *remoteServAddr, struct fanet_packet fanet_pkt, int *selected_neighbor_node, int selected_neighbor_count)
{
    int destination_count = fanet_pkt.dst_count;
    int table_index, dst_index;
    int valid_counter, nexthop;
    int dst_id[5];
    struct fanet_packet fanet_mcast_pkt;
    memcpy(&fanet_mcast_pkt, &fanet_pkt, sizeof(struct fanet_packet));
     
    for (table_index = 0; table_index < TABLE_SIZE; table_index++) {

        valid_counter = 0;
        nexthop = table_index;
        for (dst_index = 0; dst_index < destination_count; dst_index++) {
            if (selected_neighbor_node[dst_index] == table_index) {
                dst_id[valid_counter] = fanet_pkt.dst_id[dst_index];
                valid_counter++;
            }
        }

        if (valid_counter > 0) {
            fanet_mcast_pkt.dst_count = valid_counter;
            memcpy(fanet_mcast_pkt.dst_id, dst_id, sizeof(dst_id));
            int hopcount = fanet_mcast_pkt.hop_count;
            fanet_mcast_pkt.hop_count = hopcount + 1; 
            if (sendto(sd, (char*)&fanet_mcast_pkt, sizeof(fanet_mcast_pkt), 0, (struct sockaddr *)&remoteServAddr[nexthop], sizeof(remoteServAddr[0])) < 0)
                LOG_ERROR("Cannot send data");
            else
                LOG_INFO("Send to node %d", nexthop);
        }

    }

    return 0;
}

void handle_packet_from_ethnet(int n, char *msg)
{
	
	float latitude = 0;
    float longitude = 0;
    float altitude = 0;

	sscanf(msg, "%f:%f:%f", &latitude, &longitude, &my_z);
    LOG_INFO("###################Read Location from Ethernet: %f %f %f#########################", latitude, longitude, my_z);

	convert_coordinate_to_xy(latitude, longitude, &my_x, &my_y);
	//LOG_INFO("Converted Location: %f %f %f", my_x, my_y, my_z);

}

int recv_pack_cnt = 0;
bool recv_pack_bool = true;

void handle_packet_from_sock(int n, char *msg, struct sockaddr_in cliAddr, struct sockaddr_in *remoteServAddr, \
                            int bcast_sd, struct sockaddr_in broadcastAddr, int sd, struct sockaddr_in locAddr)
{

    struct fanet_packet fanet_packet;
    struct datalink_payload received_datalink;

    if (strcmp(inet_ntoa(cliAddr.sin_addr), my_ip) == 0) { //broadcast from me or fanet_info / heartbeat unicast
        LOG_INFO("From me");
        memset(&fanet_packet, 0, sizeof(fanet_packet));
        memcpy(&fanet_packet, msg, n);
        //LOG_INFO("Type:%u DstID:%u SrcID:%u SequenceNr:%d HopCount:%d Length:%u", fanet_packet.type, fanet_packet.dst_id[0], \
                                    fanet_packet.src_id, fanet_packet.seqno, fanet_packet.hop_count, fanet_packet.plen);

        if (fanet_packet.type == 5 || fanet_packet.type == 7) { //case: fanet monitoring info msg || heartbeat msg

            if (fanet_packet.dst_id[0] == my_id) { //this data is to me
                char rawData[MAX_MSG];
                memcpy(rawData, fanet_packet.data, fanet_packet.plen);
                memset(&received_datalink, 0, sizeof(struct datalink_payload));
                memcpy(&received_datalink, rawData, fanet_packet.plen);

                if (fanet_packet.type == 5)
                    LOG_INFO("Type 5 (fanet monitoring info) --> Target to Fanet data processing ");
                else if (fanet_packet.type == 7)
                    LOG_INFO("Type 7 (heartbeat message) --> Target to Fanet data processing ");

                if (received_datalink.header.msg_id == 79) {
                    struct to_gcs_msg app_msg;
                    memcpy(&app_msg, received_datalink.data, sizeof(struct to_gcs_msg));
                    app_msg.hop_count = fanet_packet.hop_count;
                    memset(received_datalink.data, 0, sizeof(received_datalink.data));
                    memcpy(received_datalink.data, (char*)&app_msg, sizeof(struct to_gcs_msg));
                }

				//send to localsocket (fanet_data_processing)
                if (sendto(sd, (char*)&received_datalink, fanet_packet.plen, 0, (struct sockaddr *)&locAddr, sizeof(locAddr)) < 0)
                    LOG_ERROR("Send data to Fanet processing"); 

            } else {
                //send fanet data to peer
                if (selector_hop_routing) { //hop routing
                    struct routing_table *item_route = search_route(fanet_packet.dst_id[0]);
                    if (item_route != NULL) {
                        int nexthop = item_route->nexthop;
                        int rc = sendto(sd, (char*)&fanet_packet, sizeof(fanet_packet), 0, (struct sockaddr *)&remoteServAddr[nexthop], sizeof(remoteServAddr[0]));
                        if (rc < 0) {
                            LOG_ERROR("Cannot send data");
                            //close(sd);
                            //exit(1);
                        }
                        LOG_INFO("Send to node %d", nexthop);
                    } else {
                        LOG_INFO("No find route");
                    }   
                } else { //geo routing
                    struct position_table *item_position = search_position(fanet_packet.dst_id[0]);
                    if (item_position != NULL) {
                        fanet_packet.dst_x_coord = item_position->x_coord;
                        fanet_packet.dst_y_coord = item_position->y_coord;
                        fanet_packet.dst_z_coord = item_position->z_coord;

                        int nexthop = select_closest_to_destination(fanet_packet);
                        if (nexthop == -1) {
                            LOG_INFO("Fail to find Next hop via geo routing");
                            //select a candidate route using HOP ROUTING
                            struct routing_table *item_route = search_route(fanet_packet.dst_id[0]);
                            if (item_route != NULL) {
                                int nexthop = item_route->nexthop;
                                int rc = sendto(sd, (char*)&fanet_packet, sizeof(fanet_packet), 0, (struct sockaddr *)&remoteServAddr[nexthop], sizeof(remoteServAddr[0]));
                                if (rc < 0) {
                                    LOG_ERROR("Cannot send data");
                                    //close(sd);
                                    //exit(1);
                                }
                                LOG_INFO("Send to node %d", nexthop);
                            } else {
                                LOG_INFO("No find route");
                            }

                        } else {
                            LOG_INFO("Next hop is %d", nexthop);
                            if (sendto(sd, (char*)&fanet_packet, sizeof(fanet_packet), 0, (struct sockaddr *)&remoteServAddr[nexthop], sizeof(remoteServAddr[0])) < 0)
                                LOG_ERROR("Cannot send data");
                        }
                    } else {
                        LOG_INFO("No find route in the positon table (dst location)");
                    }
                }
            }
        }
       
    } else {
        LOG_INFO("From others");
        memset(&fanet_packet, 0, sizeof(fanet_packet));
        memcpy(&fanet_packet, msg, n);
        LOG_INFO("Type:%u DstID:%u SrcID:%u SequenceNr:%d HopCount:%d Length:%u", fanet_packet.type, fanet_packet.dst_id[0], \
                                    fanet_packet.src_id, fanet_packet.seqno, fanet_packet.hop_count, fanet_packet.plen);

        if (fanet_packet.type == 3) {//case: hello message
            LOG_INFO("Packet Type: Hello Packet from Direct Neighbor");
            int current_time = (int)time(NULL);
            struct fanet_rc_message fanet_hello_msg;
            memset(&fanet_hello_msg, 0, sizeof(fanet_hello_msg));
            memcpy(&fanet_hello_msg, fanet_packet.data, fanet_packet.plen);

            int neighbor_id = fanet_hello_msg.node_id;
            int hello_bufsize = vector_size(hello_table[neighbor_id]);
            LOG_INFO("Vector Size is %d", hello_bufsize);
            if (hello_bufsize < HELLO_BUF_SIZE) { //size is under 10
                vector_push_back(hello_table[neighbor_id], current_time);
            } else { //size is over (10) HELLO_BUF_SIZE
                vector_erase(hello_table[neighbor_id], 0);
                vector_push_back(hello_table[neighbor_id], current_time);
            }

            if (hello_table[neighbor_id]) {
                size_t i;
                for(i = 0; i < vector_size(hello_table[neighbor_id]); ++i) {
                    LOG_INFO("v[%d] = %d", i, hello_table[neighbor_id][i]);
                }
            }

            int total_size = vector_size(hello_table[neighbor_id]);
            int time_gap = hello_table[neighbor_id][total_size - 1] - hello_table[neighbor_id][0];

            LOG_INFO("total_size %d, time_gap %d", total_size, time_gap);
            
            //build fanet_hello_ack_message delivered to the hello_node;
            struct fanet_hello_ack_message fanet_msg_hello_ack;
            memset(&fanet_msg_hello_ack, 0, sizeof(fanet_msg_hello_ack));
            fanet_msg_hello_ack.node_id = my_id;
            fanet_msg_hello_ack.x_coord = my_x;
            fanet_msg_hello_ack.y_coord = my_y;
            fanet_msg_hello_ack.z_coord = my_z;
            fanet_msg_hello_ack.ack_id = fanet_packet.seqno; //insert received rc packet seqno
            fanet_msg_hello_ack.pkt_count = total_size;
            fanet_msg_hello_ack.time_gap = time_gap;
            int count = add_route_to_array_and_count(fanet_msg_hello_ack.dst, fanet_msg_hello_ack.next, fanet_msg_hello_ack.hop); //insert routing info to hello ack msg
            fanet_msg_hello_ack.route_count = count;
    
            struct fanet_packet fanet_pkt_hello_ack;
            memset(&fanet_pkt_hello_ack, 0, sizeof(fanet_pkt_hello_ack));
            fanet_pkt_hello_ack.type = 6; //fanet_hello_ack_type (type 6)
            fanet_pkt_hello_ack.dst_count = 1;
            fanet_pkt_hello_ack.dst_id[0] = fanet_packet.src_id;
            fanet_pkt_hello_ack.dst_x_coord= 0;
            fanet_pkt_hello_ack.dst_y_coord= 0;
            fanet_pkt_hello_ack.dst_z_coord= 0;
            fanet_pkt_hello_ack.src_id = my_id;
            fanet_pkt_hello_ack.seqno = 0;
            fanet_pkt_hello_ack.hop_count = 0;


            fanet_pkt_hello_ack.plen= sizeof(fanet_msg_hello_ack);
            memcpy(fanet_pkt_hello_ack.data, (char*)&fanet_msg_hello_ack, fanet_pkt_hello_ack.plen);

            //send fanet_rc_ack_msg 
            if (sendto(sd, (char*)&fanet_pkt_hello_ack, sizeof(fanet_pkt_hello_ack), 0, (struct sockaddr *)&remoteServAddr[fanet_packet.src_id], sizeof(remoteServAddr[0])) < 0)
                LOG_ERROR("Cannot send hello ack message data");


        }
        else if (fanet_packet.type == 6) { //case: hello ack message
            LOG_INFO("Hello ACK from Neighbor Node: %u", fanet_packet.src_id);

            if (fanet_packet.dst_id[0] == my_id) { //dst of faent_packet is to me

                LOG_INFO("Hello Ack Dst is mine ");

                char rawData[MAX_MSG];
                memset(rawData, 0, sizeof(rawData));
                memcpy(rawData, fanet_packet.data, fanet_packet.plen);
                struct fanet_hello_ack_message fanet_msg_hello_ack;
                memset(&fanet_msg_hello_ack, 0, sizeof(struct fanet_hello_ack_message));
                memcpy(&fanet_msg_hello_ack, rawData, fanet_packet.plen);
                LOG_INFO("node id: %u, x_coord: %f, y_coord: %f, z_coord: %f, ack_id: %d, pkt_count: %d, time_gap: %d, route_count: %d", \
                            fanet_msg_hello_ack.node_id, fanet_msg_hello_ack.x_coord, fanet_msg_hello_ack.y_coord, fanet_msg_hello_ack.z_coord, \
                            fanet_msg_hello_ack.ack_id, fanet_msg_hello_ack.pkt_count, fanet_msg_hello_ack.time_gap, fanet_msg_hello_ack.route_count);

                struct fanet_rc_message fanet_msg_rc;
                memset(&fanet_msg_rc, 0, sizeof(struct fanet_rc_message));
                fanet_msg_rc.node_id = fanet_msg_hello_ack.node_id;
                memcpy(fanet_msg_rc.ip_address, ipmap[fanet_packet.src_id].ip, sizeof(ipmap[fanet_packet.src_id].ip));
                fanet_msg_rc.x_coord = fanet_msg_hello_ack.x_coord;
                fanet_msg_rc.y_coord = fanet_msg_hello_ack.y_coord;
                fanet_msg_rc.z_coord = fanet_msg_hello_ack.z_coord;
                fanet_msg_rc.pkt_count = fanet_msg_hello_ack.pkt_count;
                fanet_msg_rc.time_gap = fanet_msg_hello_ack.time_gap;

                pthread_mutex_lock(&table_update_mutex);
                struct neighbor_table *item_neighbor = search_neighbor(fanet_packet.src_id);
                if (item_neighbor != NULL) {
                    delete_neighbor(item_neighbor);
                }
                float avg_time, link_quality;
                if (fanet_msg_rc.pkt_count == 1)
                    avg_time = (float) fanet_msg_rc.time_gap / (fanet_msg_rc.pkt_count);
                else 
                    avg_time = (float) fanet_msg_rc.time_gap / (fanet_msg_rc.pkt_count-1);

                if (fanet_msg_rc.time_gap == 0)
                    link_quality = 1;
                else
                    link_quality = fanet_msg_rc.pkt_count / avg_time;

                LOG_INFO("link_quality = %f", link_quality);
                insert_neighbor(fanet_packet.src_id, fanet_msg_rc, link_quality); //need to check
                check_neighbor_table_timeout();

                struct routing_table *item_route = search_route(fanet_packet.src_id);
                if (item_route != NULL) {
                    delete_route(item_route);
                }
                insert_route(fanet_packet.src_id, fanet_packet.src_id, 0); //insert neighbor to route table

                //update routing table using fanet_msg_hello_ack (dst, next, hop)
                int i;
                for (i = 0; i < fanet_msg_hello_ack.route_count; i++) {
                    if (my_id != fanet_msg_hello_ack.dst[i]) {
                        struct routing_table *item_route = search_route(fanet_msg_hello_ack.dst[i]);
                        if (item_route == NULL && fanet_msg_hello_ack.hop[i] < 2) //## propagation limit (2)
                            insert_route(fanet_msg_hello_ack.dst[i], fanet_packet.src_id, fanet_msg_hello_ack.hop[i]+1);
                    }
                }

                check_routing_table_timeout();

                pthread_mutex_unlock(&table_update_mutex);

                struct position_table *item_position = search_position(fanet_packet.src_id);
                if (item_position != NULL) {
                    delete_position(item_position);
                }
                insert_position(fanet_packet.src_id, fanet_msg_rc);
                //check_position_table_timeout(); //timeout obsolete
                display_position_table();

             }
        }
        else if (fanet_packet.type == 0) { //case: routing control message
            LOG_INFO("Packet Type: Routing Control Packet");
            LOG_INFO("table seqno:%d, packet seqno:%d", rc_msg_seqno_table[fanet_packet.src_id], fanet_packet.seqno);
            int current_time = (int)time(NULL);
            int rc_msg_from_me = 0;

            if ((rc_msg_seqno_table[fanet_packet.src_id] < fanet_packet.seqno) || (current_time - rc_msg_time_table[fanet_packet.src_id] > WAKEUP_LIMIT)) {
                int old_seqno = rc_msg_seqno_table[fanet_packet.src_id];
                rc_msg_seqno_table[fanet_packet.src_id] = fanet_packet.seqno;
                rc_msg_time_table[fanet_packet.src_id] = current_time;

                struct fanet_rc_message fanet_rc_msg;
                memset(&fanet_rc_msg, 0, sizeof(fanet_rc_msg));
                memcpy(&fanet_rc_msg, fanet_packet.data, fanet_packet.plen);

                if (fanet_rc_msg.hop_count == 0) { //case 1: neighbor node

                } else { //case 2: multihop node case (routing control packet)
                    LOG_INFO("Routing Control from Multihop Node Src: %u Hop Count: %d", fanet_rc_msg.node_id, fanet_rc_msg.hop_count); 

                    if (fanet_rc_msg.node_id != my_id) {
                        LOG_INFO("Routing Control Message is not initiated from Me");

                        pthread_mutex_lock(&table_update_mutex);
                        // struct neighbor_table *item_neighbor = search_neighbor(fanet_rc_msg.prior_id);
                        // if (item_neighbor == NULL) {
                        //     LOG_INFO("Neighbor is NULL and Add Neighbor ID: %d", fanet_rc_msg.prior_id);
                        //     insert_neighbor(fanet_rc_msg.prior_id, fanet_rc_msg, 0);
                        //     check_neighbor_table_timeout();
                        // }

                        struct routing_table *item_route = search_route(fanet_rc_msg.node_id);
                        if (item_route != NULL) { //(the nodeid already in routing table)
                            /*if (item_route->hop_count >= fanet_rc_msg.hop_count) {
                                LOG_INFO("Routing Table Hop Count is Higher or Equal--> Update");
                                delete_route(item_route);
                                insert_route(fanet_rc_msg.node_id, fanet_rc_msg.prior_id, fanet_rc_msg.hop_count); //dst_node,nexthop_node,hopcount
                                check_routing_table_timeout();
                            } else {
                                LOG_INFO("Routing Table Hop Count is Lower --> No Update");
                            }*/
                            if (item_route->hop_count > fanet_rc_msg.hop_count) {
                                LOG_INFO("Routing Table Hop Count is Higher than RC Hop Count --> Update");
                                delete_route(item_route);
                                insert_route(fanet_rc_msg.node_id, fanet_rc_msg.prior_id, fanet_rc_msg.hop_count); //dst_node,nexthop_node,hopcount
                                check_routing_table_timeout();

                            } else if (item_route->hop_count == fanet_rc_msg.hop_count) {
                                LOG_INFO("Routing Table Hop Count is Equal to RC Hop Count --> Check");
                                struct neighbor_table *item_nb_from_rtb = search_neighbor(item_route->nexthop);
                                struct neighbor_table *item_nb_from_msg = search_neighbor(fanet_rc_msg.prior_id);
                                //NULL processing
                                if (item_nb_from_rtb != NULL && item_nb_from_msg != NULL) {
                                    if (item_nb_from_rtb->key == item_nb_from_msg->key) {
                                        delete_route(item_route);
                                        insert_route(fanet_rc_msg.node_id, fanet_rc_msg.prior_id, fanet_rc_msg.hop_count); //dst_node,nexthop_node,hopcount
                                        check_routing_table_timeout();
                                    } else {
                                        if (item_nb_from_rtb->link_quality < item_nb_from_msg->link_quality) {
                                            delete_route(item_route);
                                            insert_route(fanet_rc_msg.node_id, fanet_rc_msg.prior_id, fanet_rc_msg.hop_count); //dst_node,nexthop_node,hopcount
                                            check_routing_table_timeout();
                                            LOG_INFO("Update Action");
                                        } else {
                                            LOG_INFO("No Action");
                                        }
                                    }
                                } else {
                                    LOG_INFO("Neighbor information is NULL");
                                }
                                
                            } else if (item_route->hop_count < fanet_rc_msg.hop_count) {
                                LOG_INFO("Routing Table Hop Count is Lower than RC Hop Count --> No Update");
                            }
                        } else { //(routing table for the nodeid is null)
                            insert_route(fanet_rc_msg.node_id, fanet_rc_msg.prior_id, fanet_rc_msg.hop_count); //dst_node,nexthop_node,hopcount
                            check_routing_table_timeout();
                        }
                        pthread_mutex_unlock(&table_update_mutex);

                        struct position_table *item_position = search_position(fanet_rc_msg.node_id);
                        if (item_position != NULL) {
                            delete_position(item_position);
                        }
                        insert_position(fanet_rc_msg.node_id, fanet_rc_msg);
                        //check_position_table_timeout();
                    } else {
                        LOG_INFO("Routing Control Message is initiated from Me");
                        rc_msg_from_me = 1;

                    }
                }

                //increase hop count and send again;
                if (rc_msg_from_me == 0) {
                    int hopcount = fanet_rc_msg.hop_count;
                    fanet_rc_msg.hop_count = hopcount + 1;
                    fanet_rc_msg.prior_id = my_id;
                    memset(fanet_packet.data, 0, fanet_packet.plen);
                    fanet_packet.plen = sizeof(fanet_rc_msg);
                    fanet_packet.hop_count = hopcount + 1;
                    memcpy(fanet_packet.data, (char*)&fanet_rc_msg, fanet_packet.plen);

                    LOG_INFO("Send Again Routing Control Message Src:%d", fanet_rc_msg.node_id);
                    if (sendto(bcast_sd, (char*)&fanet_packet, sizeof(fanet_packet), 0, (struct sockaddr *)&broadcastAddr, sizeof(broadcastAddr)) < 0)
                        LOG_ERROR("Cannot Send Routing Control Message Broadcast");
                                            
                }


            } else {
                LOG_INFO("Already Received Routing Control Packet");
            }

        } else if (fanet_packet.type == 2) { //case: broadcast message ########### broadcast message check!! ############
            LOG_INFO("Packet Type: Broadcast Packet");
            //if (bcast_seqno_table[fanet_packet.src_id] < fanet_packet.seqno) { // New Broadcast packet
            //    bcast_seqno_table[fanet_packet.src_id] = fanet_packet.seqno;
                LOG_DEBUG("Broadcast Packet Src: %u, Sequence_Number: %d", fanet_packet.src_id, fanet_packet.seqno);
                //Send to serial port
                char rawData[MAX_MSG];
                memset(rawData, 0, sizeof(rawData));
                memcpy(rawData, fanet_packet.data, fanet_packet.plen);
                memset(&received_datalink, 0, sizeof(struct datalink_payload));
                memcpy(&received_datalink, rawData, fanet_packet.plen);
                LOG_INFO("%X %X %d %d %d %d %d %d %d %d %s", received_datalink.header.header1, received_datalink.header.header2, \
                received_datalink.header.dlen, received_datalink.header.seqno, received_datalink.header.src_id, received_datalink.header.src_port, \
                received_datalink.header.dst_id, received_datalink.header.dst_port, received_datalink.header.priority, \
                received_datalink.header.msg_id, received_datalink.data);
                int i;
                //if (received_datalink.header.dst_id == 255 && my_id != 0) { //TODO all uav:253 or all ugv:254
                if (received_datalink.header.dst_id == 255) { //TODO all uav:253 or all ugv:254
                    if (received_datalink.header.dst_port == 1) { //from udp to FC or GMCS
                        i = write(fd_com1, rawData, fanet_packet.plen);
                        LOG_INFO("Serial Write fd_com1 size: %d", i);
                    } else if (received_datalink.header.dst_port == 2) { //from udp to DMAC
                        i = write(fd_com2, rawData, fanet_packet.plen);
                        LOG_INFO("Serial Write fd_com2 size: %d", i);
                    } else if (received_datalink.header.dst_port == 0) { //from udp to Fanet
                        LOG_INFO("To Fanet");
                        //sendto(sd, (char*)rawData, fanet_packet.plen, 0, (struct sockaddr *)&locAddr, sizeof(locAddr));
                    }
                }
                //increase hop count and broadcast again;  ################# broadcast message not to send again check!!!! ################
           //     int hopcount = fanet_packet.hop_count;
           //     fanet_packet.hop_count = hopcount + 1;
           //     LOG_DEBUG("Send Again Broadcast Message Src: %u", fanet_packet.src_id);
           //      if (sendto(bcast_sd, (char*)&fanet_packet, sizeof(fanet_packet), 0, (struct sockaddr *)&broadcastAddr, sizeof(broadcastAddr)) < 0) 
           //         LOG_ERROR("Cannot Send Broadcast Message");
           //     
           // } else { //Already Received Broadcast packet
           //     LOG_DEBUG("Already Received Broadcast Packet Src: %u, Sequence_Number: %d", fanet_packet.src_id, fanet_packet.seqno);
           // }

         } else if (fanet_packet.type == 1 || fanet_packet.type == 5 || fanet_packet.type == 7 || fanet_packet.type == 8) { //case: unicast message || fanet monitoring info || heartbeat msg || heartbeat ack msg
            LOG_INFO("Packet Type: Unicast Packet");
            //determine if dst of fanet_packet is me or not if it is to me, copy data of fanet_packet and send the data to serial port

            //check if fanet_packet is multicast or not
            if (fanet_packet.dst_count > 1) {
                //TODO
                //multicast packet
                int selected_neighbor_node[TABLE_SIZE];
                int selected_neighbor_count = find_multicast_route(fanet_packet, selected_neighbor_node);
                LOG_INFO("Multicast Packet Processing Selected Neighbor Count: %d", selected_neighbor_count);
                build_multicast_packet_and_send(sd, remoteServAddr, fanet_packet, selected_neighbor_node, selected_neighbor_count);

            } else if (fanet_packet.dst_count == 1) {
                //unicast packet
                if (fanet_packet.dst_id[0] == my_id) { //dst of fanet_packet is me
                    char rawData[MAX_MSG];
                    memset(rawData, 0, sizeof(rawData));
                    memcpy(rawData, fanet_packet.data, fanet_packet.plen);
                    memset(&received_datalink, 0, sizeof(struct datalink_payload));
                    memcpy(&received_datalink, rawData, fanet_packet.plen);
                    //fanet_packet.hop_count
                    if (fanet_packet.type == 5 || fanet_packet.type == 7 || fanet_packet.type == 8) {

                        if (fanet_packet.type == 5) {
                            LOG_INFO("Type 5 --> Target to Fanet data processing ");
						} else if (fanet_packet.type == 7) {
                            LOG_INFO("Type 7 --> Performance Measure Send Back Again  to GCS");
							//send_back_again ####
							fanet_packet.type = 8;
							fanet_packet.dst_id[0] = 0;
                            fanet_packet.src_id = my_id;
					
							//send fanet data to peer
            				if (selector_hop_routing) { //hop routing
                				struct routing_table *item_route = search_route(fanet_packet.dst_id[0]);
                				if (item_route != NULL) {
                    				int nexthop = item_route->nexthop;
                    				LOG_INFO("Next hop is %d", nexthop);
                    				if (sendto(sd, (char*)&fanet_packet, sizeof(fanet_packet), 0, (struct sockaddr *)&remoteServAddr[nexthop], sizeof(remoteServAddr[0])) < 0) {
                        				LOG_ERROR("Cannot send data");
                        				//close(sd);
                        				//exit(1);
                    				}
                				} else {
                    				LOG_INFO("No find route");
                				}
            				} else { //geo routing
                				struct position_table *item_position = search_position(fanet_packet.dst_id[0]);
                				if (item_position != NULL) {
                    				fanet_packet.dst_x_coord = item_position->x_coord;
                    				fanet_packet.dst_y_coord = item_position->y_coord;
                    				fanet_packet.dst_z_coord = item_position->z_coord;

                    				int nexthop = select_closest_to_destination(fanet_packet);
                    				if (nexthop == -1) {
                        				LOG_INFO("Fail to find Next hop");
                        				//TODO
                        				//select another candidate route
                    				} else {
                        				LOG_INFO("Next hop is %d", nexthop);
                        				if (sendto(sd, (char*)&fanet_packet, sizeof(fanet_packet), 0, (struct sockaddr *)&remoteServAddr[nexthop], sizeof(remoteServAddr[0])) < 0) {
                            				LOG_ERROR("Cannot send data");
                            			//close(sd);
                            			//exit(1);
                        				}
                    				}
                				} else {
                    				LOG_INFO("No find route (Dst Location in Position Table)");
                				}
						
							}
							return;
						} else if (fanet_packet.type == 8) {
							LOG_INFO("Type 8 --> Target to Fanet data processing ");
						}

                        if (received_datalink.header.msg_id == 79) {
                            struct to_gcs_msg app_msg;
                            memcpy(&app_msg, received_datalink.data, sizeof(struct to_gcs_msg));
                            app_msg.hop_count = fanet_packet.hop_count;
                            memset(received_datalink.data, 0, sizeof(received_datalink.data));
                            memcpy(received_datalink.data, (char*)&app_msg, sizeof(struct to_gcs_msg));
                        }

                        if (sendto(sd, (char*)&received_datalink, fanet_packet.plen, 0, (struct sockaddr *)&locAddr, sizeof(locAddr)) < 0)
                            LOG_ERROR("Cannot Send Data to FANET processing");
                    } else {
                        LOG_INFO("%X %X %d %d %d %d %d %d %d %d %2x", received_datalink.header.header1, received_datalink.header.header2, \
                        received_datalink.header.dlen, received_datalink.header.seqno, received_datalink.header.src_id, received_datalink.header.src_port, \
                        received_datalink.header.dst_id, received_datalink.header.dst_port, received_datalink.header.priority, \
                        received_datalink.header.msg_id, received_datalink.data);
                        int i;
                        if (received_datalink.header.dst_port == 1) { //from udp to FC or GMCS
                            i = write(fd_com1, rawData, fanet_packet.plen);
                            LOG_INFO("fd_com1 size: %d", i);
                        } else if (received_datalink.header.dst_port == 2) { //from udp to DMAC
                            i = write(fd_com2, rawData, fanet_packet.plen);
                            LOG_INFO("fd_com2 size: %d", i);
                        } else if (received_datalink.header.dst_port == 0) { //to Fanet
                            LOG_INFO("To Fanet");
                        	//sendto(sd, (char*)rawData, fanet_packet.plen, 0, (struct sockaddr *)&locAddr, sizeof(locAddr));
                        } else if (received_datalink.header.dst_port == 3) { //to MMS
                            LOG_INFO("To MMS");
							send_control_message(received_datalink.data, received_datalink.header.dlen);
                        }
                   }

                } else { // if dst_id is not my_id to be rouuting after increasing hop count for checking ping-pong
                    int hopcount = fanet_packet.hop_count;
                    if (hopcount < MAX_HOP) {

                        if (selector_hop_routing) { //hop routing
                            fanet_packet.hop_count = hopcount + 1;
                            struct routing_table *item = search_route(fanet_packet.dst_id[0]);
                            if (item != NULL) {
                                int nexthop = item->nexthop;
                                LOG_INFO("Next hop is %d", nexthop);
                                if (sendto(sd, (char*)&fanet_packet, sizeof(fanet_packet), 0, (struct sockaddr *)&remoteServAddr[nexthop], sizeof(remoteServAddr[0])) < 0)
                                    LOG_ERROR("Cannot send data");
                                LOG_INFO("Send to node %d", nexthop);
                            } else {
                                LOG_INFO("Fail to find Next hop");
                            }
                        } else { //geo routing
                            fanet_packet.hop_count = hopcount + 1;
                            int nexthop = select_closest_to_destination(fanet_packet);
                            if (nexthop == -1) {
                                LOG_INFO("Fail to find Next hop via geo routing");
                                //select a candidate route using HOP ROUTING
                                struct routing_table *item_route = search_route(fanet_packet.dst_id[0]);
                                if (item_route != NULL) {
                                    int nexthop = item_route->nexthop;
                                    int rc = sendto(sd, (char*)&fanet_packet, sizeof(fanet_packet), 0, (struct sockaddr *)&remoteServAddr[nexthop], sizeof(remoteServAddr[0]));
                                    if (rc < 0) {
                                        LOG_ERROR("Cannot send data");
                                        //close(sd);
                                        //exit(1);
                                    }
                                    LOG_INFO("Send to node %d", nexthop);
                                } else {
                                    LOG_INFO("No find route");
                                }

                            } else {
                                LOG_INFO("Next hop is %d", nexthop);
                                if (sendto(sd, (char*)&fanet_packet, sizeof(fanet_packet), 0, (struct sockaddr *)&remoteServAddr[nexthop], sizeof(remoteServAddr[0])) < 0)
                                    LOG_ERROR("Cannot send data");
                                LOG_INFO("Send to node %d", nexthop);
                            }
                        }

                    } else {
                        LOG_INFO("Drop packet due to hopcount exceed MAX_HOP");
                    }

                }

            }

        } else if (fanet_packet.type == 4) { //(deprecate) case: routing control ack msg 
            LOG_INFO("Routing Control ACK from Neighbor Node: %u", fanet_packet.src_id);

            if (fanet_packet.dst_id[0] == my_id) { //dst of faent_packet is to me

                LOG_INFO("Routing Control Ack Dst is mine ");

                char rawData[MAX_MSG];
                memset(rawData, 0, sizeof(rawData));
                memcpy(rawData, fanet_packet.data, fanet_packet.plen);
                struct fanet_rc_ack_message fanet_msg_rc_ack;
                memset(&fanet_msg_rc_ack, 0, sizeof(struct fanet_rc_ack_message));
                memcpy(&fanet_msg_rc_ack, rawData, fanet_packet.plen);
                LOG_INFO("node id: %u, x_coord: %f, y_coord: %f, z_coord: %f, velocity: %d, direction: %d, ack_id: %d, seqno: %d, hop_count: %d", \
                            fanet_msg_rc_ack.node_id, fanet_msg_rc_ack.x_coord, fanet_msg_rc_ack.y_coord, fanet_msg_rc_ack.z_coord, \
                            fanet_msg_rc_ack.velocity, fanet_msg_rc_ack.direction, fanet_msg_rc_ack.ack_id, fanet_msg_rc_ack.seqno, fanet_msg_rc_ack.hop_count);
                
                struct fanet_rc_message fanet_msg_rc;
                memset(&fanet_msg_rc, 0, sizeof(struct fanet_rc_message));
                fanet_msg_rc.node_id = fanet_msg_rc_ack.node_id;
                memcpy(fanet_msg_rc.ip_address, ipmap[fanet_packet.src_id].ip, sizeof(ipmap[fanet_packet.src_id].ip));
                fanet_msg_rc.x_coord = fanet_msg_rc_ack.x_coord;
                fanet_msg_rc.y_coord = fanet_msg_rc_ack.y_coord;
                fanet_msg_rc.z_coord = fanet_msg_rc_ack.z_coord;
                fanet_msg_rc.velocity = fanet_msg_rc_ack.velocity;
                fanet_msg_rc.direction = fanet_msg_rc_ack.direction;
                //fanet_msg_rc.neighbor_count = fanet_msg_rc_ack.neighbor_count; //need to change
                fanet_msg_rc.hop_count= fanet_msg_rc_ack.hop_count;
                //fanet_msg_rc.prior_id= fanet_msg_rc_ack.prior_id;

                pthread_mutex_lock(&table_update_mutex);
                struct neighbor_table *item_neighbor = search_neighbor(fanet_packet.src_id);
                if (item_neighbor != NULL) {
                    delete_neighbor(item_neighbor);
                }
                insert_neighbor(fanet_packet.src_id, fanet_msg_rc, 0); //need to check
                check_neighbor_table_timeout();

                struct routing_table *item_route = search_route(fanet_packet.src_id);
                if (item_route != NULL) {
                    delete_route(item_route);
                }
                insert_route(fanet_packet.src_id, fanet_packet.src_id, fanet_msg_rc_ack.hop_count); //insert neighbor to route table
                check_routing_table_timeout();
                pthread_mutex_unlock(&table_update_mutex);

                struct position_table *item_position = search_position(fanet_packet.src_id);
                if (item_position != NULL) {
                    delete_position(item_position);
                }
                insert_position(fanet_packet.src_id, fanet_msg_rc);
                //check_position_table_timeout(); //timeout obsolete
                display_position_table();

             }
        }

    }

}


void process_serial_to_sock(unsigned char **mp, int *p_csize, int *p_asize, unsigned char *receive_buffer, \
                            int sd, struct sockaddr_in *remoteServAddr, int bcast_sd, struct sockaddr_in broadcastAddr) {

    int front_count = 0;
    int rear_count = 0;
    int pair_count = 0;
    int copy_size = 0;
    char print_buffer[MAX_MSG];
    char temp_buffer[15000];
    struct datalink_payload received_datalink;

    int parser_index = 0;
    int i = 0;
    
    for (i = 0; i < 14999; i++) {
        if (receive_buffer[i] == 0xAA && receive_buffer[i+1] == 0x55) {
            if (pair_count == 0) {
                front_count = i;
                pair_count++;
            } else {
                rear_count = i;
                parser_index = 1;
                break;
            }
        }
    }

    if (rear_count == 0) { //copy size from data size
        for (i = 0; i < 14999; i++) {
            if (receive_buffer[i] == 0xAA && receive_buffer[i+1] == 0x55) {
                if (receive_buffer[i+2] != 0) {
                    copy_size = receive_buffer[i+2]+12;
                    //copy_size = receive_buffer[i+2];
                    //if (receive_buffer[front_count+copy_size-1] != 0) {
                    //if (receive_buffer[front_count+copy_size-3] != 0) {
                    if (*p_asize >= (front_count+copy_size)) {
                        rear_count = front_count+copy_size;
                        LOG_DEBUG("Paring using size_info Serial data size is *p_asize: %d copy_size: %d rear_count: %d", *p_asize, copy_size, rear_count);
                        parser_index = 2;
                        break;
                     }
                 }
             }
         }
    }

    if (rear_count > 0) {

        pthread_mutex_lock(&mutex);

        copy_size = rear_count - front_count;
		if (copy_size <= MAX_MSG) {
        	LOG_DEBUG("Serial data size is Copy Size: %d Rear Count: %d Parser_index: %d(1:startend,2:size)", copy_size, rear_count, parser_index);
        	memset(print_buffer, 0, MAX_MSG);
        	memcpy(print_buffer, &receive_buffer[front_count], copy_size);
        	memset(&received_datalink, 0, sizeof(struct datalink_payload));
        	memcpy(&received_datalink, print_buffer, copy_size);

			//if (received_datalink.header.msg_id == 101 || received_datalink.header.msg_id == 201 || received_datalink.header.msg_id == 202) {
        	//
            //	printf("%X %X %d %d %d %d %d %d %d %d %2x\n", received_datalink.header.header1, received_datalink.header.header2, \
        	//	received_datalink.header.dlen, received_datalink.header.seqno, received_datalink.header.src_id, received_datalink.header.src_port,\
         	//	received_datalink.header.dst_id, received_datalink.header.dst_port, received_datalink.header.priority, \
            //	received_datalink.header.msg_id, received_datalink.data);
			//}
			
			LOG_INFO("%X %X %d %d %d %d %d %d %d %d %2x", received_datalink.header.header1, received_datalink.header.header2, \
        	received_datalink.header.dlen, received_datalink.header.seqno, received_datalink.header.src_id, received_datalink.header.src_port,\
         	received_datalink.header.dst_id, received_datalink.header.dst_port, received_datalink.header.priority, \
            received_datalink.header.msg_id, received_datalink.data);
		}

        memset(temp_buffer, 0, 15000);
        LOG_DEBUG("Before print p_asize = %d", *p_asize);
        int temp_count = *p_asize - rear_count;
        memcpy(temp_buffer, &receive_buffer[rear_count], temp_count);
        memset(receive_buffer, 0, 15000);
        memcpy(receive_buffer, temp_buffer, temp_count);
        LOG_DEBUG("Before print p_csize = %d", *p_csize);
        *mp = *mp - copy_size + *p_csize;
        LOG_DEBUG("*mp address: %p", *mp);
        *p_csize = 0;
        *p_asize = *p_asize - copy_size;
        LOG_DEBUG("After print p_asize = %d", *p_asize);
        LOG_DEBUG("After print p_csize = %d", *p_csize);

        pthread_mutex_unlock(&mutex);

        if (copy_size > MAX_MSG) {
			LOG_INFO("size of data is larger than MAX_MAG(512), do not send");
			//LOG_INFO("size of data is larger than 255, do not send");
			return;
		}

        //check if dst_id is broadcast addres or not
        if (received_datalink.header.dst_id == 255) { //broadcast TODO all uav 253 or all ugv 254
            LOG_INFO("send to broadcast address");

            struct fanet_packet fanet_pkt;  //pack fanet data and send to peer after refering route table
            fanet_pkt.type = 2; //fanet broadcast type
            fanet_pkt.dst_count = 1;
            fanet_pkt.dst_id[0] = 255;
            fanet_pkt.src_id = my_id;
            fanet_pkt.seqno = bcast_seqno_table[my_id]+1; //0;
	    	bcast_seqno_table[my_id] = fanet_pkt.seqno;
            fanet_pkt.hop_count = 0;
            fanet_pkt.plen= copy_size;
            memcpy(fanet_pkt.data, print_buffer, copy_size);

            int rc = sendto(bcast_sd, (char*)&fanet_pkt, sizeof(fanet_pkt), 0, (struct sockaddr *)&broadcastAddr, sizeof(broadcastAddr));
            if (rc < 0) {
                LOG_ERROR("Cannot send data");
                //close(bcast_sd);
                //exit(1);
            }
        } else if (received_datalink.header.dst_id == my_id) { //send to me
            //TODO
            //send to local processing socket
            //from FC for location updates
            if (received_datalink.header.dst_port == 0 && received_datalink.header.msg_id == 201) {
                LOG_WARNING("location update data");
                struct vehicle_navi_data_rsp vnd_rsp;
                LOG_DEBUG("vehicle navi data rsp size = %d", sizeof(struct vehicle_navi_data_rsp));
                memset(&vnd_rsp, 0 , sizeof(struct vehicle_navi_data_rsp));
                memcpy(&vnd_rsp, received_datalink.data, sizeof(struct vehicle_navi_data_rsp));
                LOG_WARNING("%X %X %X %X %X %X %X %X %X %X %ld %ld %X", vnd_rsp.current_op_mode, vnd_rsp.uplink_packet_hz, \
                vnd_rsp.roll_angle, vnd_rsp.pitch_angle, vnd_rsp.yaw_course_heading, vnd_rsp.air_ground_speed, \
                vnd_rsp.pressure_altitude, vnd_rsp.gps_altitude, vnd_rsp.rate_of_climb, vnd_rsp.gps_operation, \
                vnd_rsp.latitude, vnd_rsp.longitude, vnd_rsp.warning);

                long lat_in, long_in;
                lat_in = (vnd_rsp.latitude[0] << 24) + (vnd_rsp.latitude[1] << 16) + (vnd_rsp.latitude[2] << 8) + (vnd_rsp.latitude[3] << 0);
                long_in = (vnd_rsp.longitude[0] << 24) + (vnd_rsp.longitude[1] << 16) + (vnd_rsp.longitude[2] << 8) + (vnd_rsp.longitude[3] << 0);

				float lat_f, long_f;
				lat_f = lat_in/10000000.0;
				long_f = long_in/10000000.0;

                LOG_WARNING("geolocation: %lf, %lf", lat_f, long_f);

				convert_coordinate_to_xy(lat_f, long_f, &my_x, &my_y);

                //TODO
                //function: handle packet from ethernet

            }
        } else { // send to others
            LOG_INFO("send to unicast address");

            struct fanet_packet fanet_pkt;  //pack fanet data and send to peer after refering route table
            memset(&fanet_pkt, 0, sizeof(fanet_pkt));
            fanet_pkt.type = 1; //fanet unicast type
            fanet_pkt.dst_count = 1;
            fanet_pkt.dst_id[0] = received_datalink.header.dst_id;
            fanet_pkt.src_id = my_id;
            fanet_pkt.seqno = 0;
            fanet_pkt.hop_count = 0;
            fanet_pkt.plen= copy_size;
            memcpy(fanet_pkt.data, print_buffer, copy_size);

            //send fanet data to peer
            if (selector_hop_routing) { //hop routing
                struct routing_table *item_route = search_route(fanet_pkt.dst_id[0]);
                if (item_route != NULL) {
                    int nexthop = item_route->nexthop;
                    LOG_INFO("Next hop is %d", nexthop);
                    if (sendto(sd, (char*)&fanet_pkt, sizeof(fanet_pkt), 0, (struct sockaddr *)&remoteServAddr[nexthop], sizeof(remoteServAddr[0])) < 0) {
                        LOG_ERROR("Cannot send data");
                        //close(sd);
                        //exit(1);
                    }
                } else {
                    LOG_INFO("No find route");
                }
            } else { //geo routing
                struct position_table *item_position = search_position(fanet_pkt.dst_id[0]);
                if (item_position != NULL) {
                    fanet_pkt.dst_x_coord = item_position->x_coord;
                    fanet_pkt.dst_y_coord = item_position->y_coord;
                    fanet_pkt.dst_z_coord = item_position->z_coord;

                    int nexthop = select_closest_to_destination(fanet_pkt);
                    if (nexthop == -1) {
                        LOG_INFO("Fail to find Next hop");
                        //TODO
                        //select another candidate route
                    } else {
                        LOG_INFO("Next hop is %d", nexthop);
                        if (sendto(sd, (char*)&fanet_pkt, sizeof(fanet_pkt), 0, (struct sockaddr *)&remoteServAddr[nexthop], sizeof(remoteServAddr[0])) < 0) {
                            LOG_ERROR("Cannot send data");
                            //close(sd);
                            //exit(1);
                        }
                    }
                } else {
                    LOG_INFO("No find route (Dst Location in Position Table)");
                }
            }
        }
    }
    //LOG_DEBUG("count kk = %d", kk);
    //}
}

int parse_geolocation_file()
{
    FILE* file = fopen(GEO_FILE_NAME, "r");
    char line[128];
    char bottom_string[32];
    char top_string[32];
    char dist_string[32];
    BOOL bottom_init = TRUE;
    BOOL top_init = TRUE;
    BOOL dist_init = TRUE;

    float bottom_latitude, bottom_longitude, top_latitude, top_longitude;
    float dist_y, dist_x;

    if (file != NULL) {
        while (fgets(line, sizeof(line), file) != NULL) {
            if (bottom_init == TRUE) {
                if (sscanf(line, "%s %f %f", bottom_string, &bottom_latitude, &bottom_longitude) != 3)
                    return -1;
                LOG_INFO("%s:\t%f\t%f", bottom_string, bottom_latitude, bottom_longitude);
                sign_lat = bottom_latitude;
                sign_lon = bottom_longitude;
                bottom_init = FALSE;
            } else if (top_init == TRUE) {
                if (sscanf(line, "%s %f %f", top_string, &top_latitude, &top_longitude) != 3)
                    return -1;
                LOG_INFO("%s:\t%f\t%f", top_string, top_latitude, top_longitude);
                top_init = FALSE;
            } else if (dist_init == TRUE) {
                if (sscanf(line, "%s %f %f", dist_string, &dist_y, &dist_x) != 3)
                    return -1;
                LOG_INFO("%s:\t%f\t%f", dist_string, dist_y, dist_x);
                sign_y = dist_y /(top_latitude - bottom_latitude);
                sign_x = dist_x /(top_longitude - bottom_longitude);
                LOG_INFO("sing y and x:\t%f\t%f", sign_y, sign_x);
                dist_init = FALSE;
            }
        }
        fclose(file);
    } else {
        LOG_ERROR("georef.txt doesn't exist");
        return -1;
    }

    return 0;
}

int parse_config_from_file(struct map_id_to_ip *real)
{
    FILE* file = fopen(CONFIG_FILE_NAME, "r"); /* should check the result */
    char line[128];
    struct map_id_to_ip temp[TABLE_SIZE] = {};

    char pkt_delivery_rate[32];
    char hello_schedule_interval[32];
    char max_hop[32];
    char table_valid_time[32];
    char rc_schedule_interval[32];
    char wakeup_limit[32];
    char valid_neighbor_gap[32];

    char mobiusstring[32];
    char mobidistring[32];
    char testbednamestring[32];
    char finfo_schedule_interval[32];
    char mqttstring[32];
    char idstring[32];
    char locstring[32];

    int ct = 0;

    BOOL pkt_rate_init = TRUE;
    BOOL hello_schedule_init = TRUE;
    BOOL hop_init = TRUE;
    BOOL table_duration_init = TRUE;
    BOOL rc_schedule_init = TRUE;
    BOOL wakeup_init = TRUE;
    BOOL valid_neighbor_init = TRUE;

    BOOL mobius_init = TRUE;
    BOOL mobius_direct_init = TRUE;
    BOOL testbed_name_init = TRUE;
    BOOL finfo_schedule_init = TRUE;
    BOOL mqtt_init = TRUE;
    BOOL my_init = TRUE;
    BOOL loc_init = TRUE;
    
    if (file != NULL) {
        while (fgets(line, sizeof(line), file) != NULL) {
            if (pkt_rate_init == TRUE) {
                if (sscanf(line, "%s %d", pkt_delivery_rate, &PKT_DELIVERY_RATE) != 2) 
                    return -1;
                LOG_INFO("%s\t%d", pkt_delivery_rate, PKT_DELIVERY_RATE);
                pkt_rate_init = FALSE;
            } else if (hello_schedule_init == TRUE) {
                if (sscanf(line, "%s %d", hello_schedule_interval, &HELLO_SCHEDULE_INTERVAL) != 2)
                    return -1;
                LOG_INFO("%s\t%d", hello_schedule_interval, HELLO_SCHEDULE_INTERVAL);
                hello_schedule_init = FALSE;
            } else if (hop_init == TRUE) {
                if (sscanf(line, "%s %d", max_hop, &MAX_HOP) != 2) 
                    return -1;
                LOG_INFO("%s\t%d", max_hop, MAX_HOP);
                hop_init = FALSE;
            } else if (table_duration_init == TRUE) {
                if (sscanf(line, "%s %d", table_valid_time, &VALID_DURATION) != 2) 
                    return -1;
                LOG_INFO("%s\t%d", table_valid_time, VALID_DURATION);
                table_duration_init = FALSE;
            } else if (rc_schedule_init == TRUE) {
                if (sscanf(line, "%s %d", rc_schedule_interval, &SCHEDULE_INTERVAL) != 2)
                    return -1;
                LOG_INFO("%s\t%d", rc_schedule_interval, SCHEDULE_INTERVAL);
                rc_schedule_init = FALSE;
            } else if (wakeup_init == TRUE) {
                if (sscanf(line, "%s %d", wakeup_limit, &WAKEUP_LIMIT) != 2)
                    return -1;
                LOG_INFO("%s\t%d", wakeup_limit, WAKEUP_LIMIT);
                wakeup_init = FALSE;
            } else if (valid_neighbor_init == TRUE) {
                if (sscanf(line, "%s %d", valid_neighbor_gap, &VALID_NEIGHBOR_GAP) != 2)
                    return -1;
                LOG_INFO("%s\t%d", valid_neighbor_gap, VALID_NEIGHBOR_GAP);
                valid_neighbor_init = FALSE;
            } else if (mobius_init == TRUE) {
                if (sscanf(line, "%s %s", mobiusstring, mobius_on_off) != 2) 
                    return -1;
                LOG_INFO("%s\t%s", mobiusstring, mobius_on_off);
                mobius_init = FALSE;
            } else if (mobius_direct_init == TRUE) {
                if (sscanf(line, "%s %s", mobidistring, mobius_direct_on_off) != 2) 
                    return -1;
                LOG_INFO("%s\t%s", mobidistring, mobius_direct_on_off);
                mobius_direct_init = FALSE;
            } else if (testbed_name_init == TRUE) {
                if (sscanf(line, "%s %s", testbednamestring, testbed_name) != 2) 
                    return -1;
                LOG_INFO("%s\t%s", testbednamestring, testbed_name);
                testbed_name_init = FALSE;
            } else if (finfo_schedule_init == TRUE) {
                if (sscanf(line, "%s %d", finfo_schedule_interval, &FINFO_SCHEDULE_INTERVAL) != 2) 
                    return -1;
                LOG_INFO("%s\t%d", finfo_schedule_interval, FINFO_SCHEDULE_INTERVAL);
                finfo_schedule_init = FALSE;
            } else if (mqtt_init == TRUE) {
                if (sscanf(line, "%s %s", mqttstring, mqtt_ip) != 2) 
                    return -1;
                LOG_INFO("%s\t%s", mqttstring, mqtt_ip);
                mqtt_init = FALSE;
            } else if (my_init == TRUE) {
                if (sscanf(line, "%s %d %s %s", idstring, &my_id, my_ip, my_type) != 4)
                    return -1;
                LOG_INFO("%s:\t%d\t%s\t%s", idstring, my_id, my_ip, my_type);
                my_init = FALSE;
            } else if (loc_init == TRUE) {
                float latitude, longitude;
                if (sscanf(line, "%s %f %f %f", locstring, &latitude, &longitude, &my_z) != 4)
                    return -1;
                convert_coordinate_to_xy(latitude, longitude, &my_x, &my_y);
                LOG_INFO("%s:\t%f\t%f\t%f", locstring, my_x, my_y, my_z);
                loc_init = FALSE;
            } else {
                if (sscanf(line, "%d %s %s", &temp[ct].id, (char *)&temp[ct].ip, (char*)&temp[ct].type) != 3)
                    return -1;
                LOG_INFO("%d \t%s\t%s", temp[ct].id, temp[ct].ip, temp[ct].type);
                ct++;
            }
        }
        fclose(file);
    } else {
        LOG_ERROR("%s doesn't exist", CONFIG_FILE_NAME);
        return -1;
    }

    int i;
    for (i = 0; i < ct; i++) {
        //printf("%d %s %s\n", temp[i].id, temp[i].ip, temp[i].type);
        strcpy(real[temp[i].id].ip, temp[i].ip);
        strcpy(real[temp[i].id].type, temp[i].type);
    }

    //for (int i = 0; i < TABLE_SIZE; i++){
    //    printf("%2d:\t%s\t%s\n", i, real[i].ip, real[i].type);
    // }

    return 0;
}

void call_routing_module(char *msg, int size, int dst_id)
{
    int sd;
    struct sockaddr_in unicastAddr;
      
    sd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sd < 0)
        perror("cannot open socket");
                   
    memset(&unicastAddr, 0, sizeof(unicastAddr));
    unicastAddr.sin_family = AF_INET;
    unicastAddr.sin_addr.s_addr = inet_addr(my_ip);
    unicastAddr.sin_port = htons(REMOTE_SERVER_PORT);

    struct datalink_header msg_header;
    struct datalink_tail msg_tail;
    struct datalink_payload msg_payload;
    memset(&msg_header, 0, sizeof(struct datalink_header));
    memset(&msg_tail, 0, sizeof(struct datalink_tail));
    memset(&msg_payload, 0, sizeof(struct datalink_payload));

    msg_header.header1 = 0xAA;
    msg_header.header2 = 0x55;
    msg_header.dlen = sizeof(struct datalink_payload);
    msg_header.src_id = my_id;
    msg_header.src_port = 0;
    msg_header.dst_id = dst_id;
    msg_header.dst_port = 0;
    msg_header.priority = 0;
    msg_header.msg_id = 80; //#### command message id
    msg_tail.MSB = 12;
    msg_tail.LSB = 31;
    msg_payload.header = msg_header;
    msg_payload.tail = msg_tail;

    struct fanet_packet fanet_pkt_command;
    memset(&fanet_pkt_command, 0, sizeof(fanet_pkt_command));
    fanet_pkt_command.type = 5;  //#### fanet_monitoring_info
    fanet_pkt_command.dst_count = 1;
    fanet_pkt_command.dst_id[0] = dst_id; 

    struct position_table *item_position = search_position(fanet_pkt_command.dst_id[0]);
    if (item_position!= NULL) {
        fanet_pkt_command.dst_x_coord = item_position->x_coord;
        fanet_pkt_command.dst_y_coord = item_position->y_coord;
        fanet_pkt_command.dst_z_coord = item_position->z_coord;
    }

    fanet_pkt_command.src_id = my_id;
    fanet_pkt_command.seqno = 0;
    fanet_pkt_command.hop_count = 0;
    fanet_pkt_command.plen = sizeof(msg_payload);

    memcpy(msg_payload.data, msg, size);
    memcpy(fanet_pkt_command.data, (char*)&msg_payload, fanet_pkt_command.plen);

    int rc;
    if (rc = sendto(sd, (char*)&fanet_pkt_command, sizeof(fanet_pkt_command), 0, (struct sockaddr *)&unicastAddr, sizeof(unicastAddr)) < 0)
        LOG_ERROR("Cannot send data");
    LOG_INFO("return value: %d", rc);

    close(sd);
}

void send_control_message(char *msg, int size)
{
    int eth_sd;
    struct sockaddr_in eth_addr;
    eth_sd = socket(AF_INET, SOCK_DGRAM, 0);
    if (eth_sd < 0) {
        LOG_ERROR("cannot open socket");
        //exit(1);
    }

    eth_addr.sin_family = AF_INET;
    eth_addr.sin_addr.s_addr = inet_addr(ETH_IP_ADDRESS);
    eth_addr.sin_port = htons(ETH_SERVER_PORT);
    bzero(&(eth_addr.sin_zero), 8);

    if (sendto(eth_sd, msg, size, 0, (struct sockaddr*)&eth_addr, sizeof(struct sockaddr)) < 0) {
        LOG_ERROR("Cannot send data");
        //exit(1);
    }
    LOG_INFO("send to ethernet size:%d message:%s", size, msg);
    close(eth_sd);
}

void *tas_thread(void *arg) //tas connection creation thread
{
    LOG_INFO("creation tas thread");

    int numbytes;
    char buf[1024];

    char *notiregi = "{\"ctname\":\"control\",\"con\":\"hello\"}";
    if (send(tas_sockfd, notiregi, strlen(notiregi), 0) == -1){
        perror("send");
	    //exit(1);
    }

    while (1) {

        if ((numbytes=recv(tas_sockfd, buf, 1024, 0)) == -1) {
        	perror("recv");
            //exit(1);
			close(tas_sockfd);
        	struct sockaddr_in their_addr;
    
       		if ((tas_sockfd = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
           		LOG_ERROR("socket: socket creation error");
           		//exit(1);
       		}

			bzero((char *)&their_addr, sizeof(their_addr));
       		their_addr.sin_family = AF_INET;      /* host byte order */
       		their_addr.sin_port = htons(3105);    /* short, network byte order */
       		their_addr.sin_addr.s_addr = inet_addr("127.0.0.1");

       		if (connect(tas_sockfd, (struct sockaddr *)&their_addr, sizeof(struct sockaddr)) == -1) {
           		LOG_ERROR("connect: connection refused error");
           		//exit(1);
       		}
		
		}	

      	LOG_INFO("Received text=: %s", buf);

		if (strlen(buf) > 5) {

        	int i = 0;
        	char con_buffer[255]={0};
        	char *ptr = strtok(buf, "\"");

        	while (ptr != NULL) {
            	if (i == 3) 
                	break;
            	i++;
            	ptr = strtok(NULL, "\"");
        	}

        	LOG_INFO("con: %s", ptr);
        	strcpy(con_buffer, ptr);

        	char *constring = "control";
			char *startstring = "start_data";
			char *stopstring = "stop_data";

        	if (strcmp(constring, con_buffer)) {
				if (!strcmp(startstring, con_buffer)) {
		    		//start_send_data();	
				} else if (!strcmp(stopstring, con_buffer)) {
					//stop_send_data();
				} else {
            		send_control_message(con_buffer, strlen(con_buffer));
				}
        	}

		}
        
    }

    close(tas_sockfd);
}

void *schedule_hello_msg(void *arg)
{
    int sd;
    struct sockaddr_in broadcastAddr;
    char *broadcastIP = BCAST_IP_ADDRESS;
    unsigned short broadcastPort = REMOTE_SERVER_PORT;
    int broadcastPermission = 1;

    time_t t;
    srand((unsigned) time(&t));

    sd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sd < 0)
        perror("cannot open socket");

    if (setsockopt(sd, SOL_SOCKET, SO_BROADCAST, &broadcastPermission, sizeof(broadcastPermission)) < 0)
        perror("sesockopt (SO_BROADCAST)");

    memset(&broadcastAddr, 0, sizeof(broadcastAddr));
    broadcastAddr.sin_family = AF_INET;
    broadcastAddr.sin_addr.s_addr = inet_addr(broadcastIP);
    broadcastAddr.sin_port = htons(broadcastPort);
    
    struct fanet_rc_message fanet_hello_msg;
    fanet_hello_msg.node_id = my_id;
    memcpy(fanet_hello_msg.ip_address, my_ip, sizeof(my_ip));
    fanet_hello_msg.x_coord = my_x;
    fanet_hello_msg.y_coord = my_y;
    fanet_hello_msg.z_coord = my_z;
    fanet_hello_msg.velocity = 0;
    fanet_hello_msg.direction = 0;
    fanet_hello_msg.hop_count = 0;
    fanet_hello_msg.prior_id = my_id;

    int hello_count = 0;

    while (1) {

        hello_count++;
        LOG_INFO("Hello  message counter #%d", hello_count);

        struct fanet_packet fanet_pkt_hello;
        memset(&fanet_pkt_hello, 0, sizeof(fanet_pkt_hello));
        fanet_pkt_hello.type = 3; //fanet_hello_type
        fanet_pkt_hello.dst_count = 1;
        fanet_pkt_hello.dst_id[0] = 0;
        fanet_pkt_hello.dst_x_coord= 0;
        fanet_pkt_hello.dst_y_coord= 0;
        fanet_pkt_hello.dst_z_coord= 0;
        fanet_pkt_hello.src_id = my_id;
        fanet_pkt_hello.seqno = hello_count;
        fanet_pkt_hello.hop_count = 0;

        //update my location
        fanet_hello_msg.x_coord = my_x;
        fanet_hello_msg.y_coord = my_y;
        fanet_hello_msg.z_coord = my_z;

        fanet_pkt_hello.plen= sizeof(fanet_hello_msg);
        memcpy(fanet_pkt_hello.data, (char*)&fanet_hello_msg, fanet_pkt_hello.plen);

        if (sendto(sd, (char*)&fanet_pkt_hello, sizeof(fanet_pkt_hello), 0, (struct sockaddr *)&broadcastAddr, sizeof(broadcastAddr)) < 0)
            LOG_ERROR("Routing Control Message Broadcast Send -1");

        sleep(HELLO_SCHEDULE_INTERVAL);
    }

    close(sd);
}

void *schedule_rc_msg(void *arg)
{
    int sd;
    struct sockaddr_in broadcastAddr;
    char *broadcastIP = BCAST_IP_ADDRESS;
    unsigned short broadcastPort = REMOTE_SERVER_PORT;
    int broadcastPermission = 1;

    time_t t;
    srand((unsigned) time(&t));

    sd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sd < 0)
        LOG_ERROR("cannot open socket");

    if (setsockopt(sd, SOL_SOCKET, SO_BROADCAST, &broadcastPermission, sizeof(broadcastPermission)) < 0)
        LOG_ERROR("sesockopt (SO_BROADCAST)");

    memset(&broadcastAddr, 0, sizeof(broadcastAddr));
    broadcastAddr.sin_family = AF_INET;
    broadcastAddr.sin_addr.s_addr = inet_addr(broadcastIP);
    broadcastAddr.sin_port = htons(broadcastPort);
    
    struct fanet_rc_message fanet_rc_msg;
    fanet_rc_msg.node_id = my_id;
    memcpy(fanet_rc_msg.ip_address, my_ip, sizeof(my_ip));
    fanet_rc_msg.x_coord = my_x;
    fanet_rc_msg.y_coord = my_y;
    fanet_rc_msg.z_coord = my_z;
    fanet_rc_msg.velocity = 0;
    fanet_rc_msg.direction = 0;
    fanet_rc_msg.hop_count = 0;
    fanet_rc_msg.prior_id = my_id;
    //fanet_rc_msg.neighbor_count = add_neighbors_and_count(fanet_rc_msg.nexthop);
    //printf("Neighbor Table Size: %d\n", fanet_rc_msg.neighbor_count);

    int rc_count = 0;
    int check_count = 0;

    while (1) {
        rc_count++;
        LOG_INFO("Routing control message counter #%d", rc_count);

        struct fanet_packet fanet_pkt_rc;
        memset(&fanet_pkt_rc, 0, sizeof(fanet_pkt_rc));
        fanet_pkt_rc.type = 0; //fanet_rc_type
        fanet_pkt_rc.dst_count = 1;
        fanet_pkt_rc.dst_id[0] = 0;
        fanet_pkt_rc.dst_x_coord= 0;
        fanet_pkt_rc.dst_y_coord= 0;
        fanet_pkt_rc.dst_z_coord= 0;
        fanet_pkt_rc.src_id = my_id;
        fanet_pkt_rc.seqno = rc_count;
        fanet_pkt_rc.hop_count = 0;

        //update my location
        fanet_rc_msg.x_coord = my_x;
        fanet_rc_msg.y_coord = my_y;
        fanet_rc_msg.z_coord = my_z;

        fanet_pkt_rc.plen= sizeof(fanet_rc_msg);
        memcpy(fanet_pkt_rc.data, (char*)&fanet_rc_msg, fanet_pkt_rc.plen);

        rc_msg_seqno_table[fanet_pkt_rc.src_id] = fanet_pkt_rc.seqno;
        rc_msg_time_table[fanet_pkt_rc.src_id] = (int)time(NULL);

        if (sendto(sd, (char*)&fanet_pkt_rc, sizeof(fanet_pkt_rc), 0, (struct sockaddr *)&broadcastAddr, sizeof(broadcastAddr)) < 0)
            LOG_ERROR("Routing Control Message Broadcast Send -1");

        sleep(SCHEDULE_INTERVAL + rand()%2);

        if (check_count % 2 == 0) {
            LOG_INFO("Check Routing Table Enter");
            pthread_mutex_lock(&table_update_mutex);
            check_neighbor_table_timeout();
            check_routing_table_timeout();
            pthread_mutex_unlock(&table_update_mutex);
        }
        check_count++;

    }

    close(sd);
}

void *generate_heart_beat_msg(void *arg) {

    int sd;
    struct sockaddr_in unicastAddr;
    int broadcastPermission = 1;

    sd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sd < 0)
        perror("cannot open socket");

    if (setsockopt(sd, SOL_SOCKET, SO_BROADCAST, &broadcastPermission, sizeof(broadcastPermission)) < 0)
        perror("sesockopt (SO_BROADCAST)");

    memset(&unicastAddr, 0, sizeof(unicastAddr));
    unicastAddr.sin_family = AF_INET;
    unicastAddr.sin_addr.s_addr = inet_addr(my_ip);
    unicastAddr.sin_port = htons(REMOTE_SERVER_PORT);

    struct datalink_header hb_header;
    struct datalink_tail hb_tail;
    struct datalink_payload hb_payload;
    memset(&hb_header, 0, sizeof(struct datalink_header));
    memset(&hb_tail, 0, sizeof(struct datalink_tail));
    memset(&hb_payload, 0, sizeof(struct datalink_payload));
    hb_header.header1 = 0xAA;
    hb_header.header2 = 0x55;
    hb_header.dlen = sizeof(struct datalink_payload);
    hb_header.src_id = my_id;
    hb_header.src_port = 0;
    hb_header.dst_id = 0; //###### destination_id 0 (send to gcs node) 
    hb_header.dst_port = 0;
    hb_header.priority = 0;
    hb_header.msg_id = 78;  //####### message_id 8
    hb_tail.MSB = 12;
    hb_tail.LSB = 31;
    hb_payload.tail = hb_tail;

    unsigned char sequence_number = 0;

    while (1) {

        hb_header.seqno = sequence_number++;
        hb_payload.header = hb_header;

        struct fanet_packet fanet_pkt_heartbeat;
        memset(&fanet_pkt_heartbeat, 0, sizeof(fanet_pkt_heartbeat));
        fanet_pkt_heartbeat.type = 7;  //#### heart beat message to test if each node delivers data well or not
        fanet_pkt_heartbeat.dst_count = 1;
        fanet_pkt_heartbeat.dst_id[0] = 0; //#### destination node id 0 
        
        fanet_pkt_heartbeat.src_id = my_id;
        fanet_pkt_heartbeat.hop_count = 0;
        fanet_pkt_heartbeat.plen = sizeof(hb_payload);
        memcpy(fanet_pkt_heartbeat.data, (char*)&hb_payload, fanet_pkt_heartbeat.plen);

        int rc = sendto(sd, (char*)&fanet_pkt_heartbeat, sizeof(fanet_pkt_heartbeat), 0, (struct sockaddr *)&unicastAddr, sizeof(unicastAddr));
        LOG_INFO("return value: %d", rc);      

        //fanet info uplaod interval
        sleep(FINFO_SCHEDULE_INTERVAL * 5);
    }

}

void create_heartbeat_send_to_tas(int nodeid, int seqno, char *data)
{

    char *str1 = "{\"ctname\":\"heartbeat\"";
    char *str2 = ",\"con\":\"";
    char *str3 = "_";
    char *endstr = "\"}";

    char typestr[10]={0};
    sprintf(typestr, "%s",ipmap[nodeid].type);
    char tostr[10]={0};
    sprintf(tostr, "%d", nodeid);
    char seqstr[10]={0};
    sprintf(seqstr, "%d", seqno);
    
    strcat(data,str1);
    strcat(data,str2);
    strcat(data,tostr);
    strcat(data,str3);
    strcat(data,seqstr);    
    strcat(data,endstr);

    LOG_INFO("send string: %s", data); 
    
}

void create_data_send_to_tas(char *data)
{

    char *str1 = "{\"ctname\":\"";
    char *str2 = "_";
    char *str3 = "\",\"con\":";
    char *endstr = "}}";

    char typestr[10]={0};
    sprintf(typestr, "%s",ipmap[my_id].type);
    char tostr[10]={0};
    sprintf(tostr, "%d", my_id);

    char neighbor_list[40]={0};
    enumeration_neighbor_table(neighbor_list);
    LOG_INFO("neighbor list for sending: %s", neighbor_list); 

    char routing_list[40]={0};            
    enumeration_routing_table(routing_list);
    LOG_INFO("routing list for sending: %s", routing_list); 

    float lat, lon;
    convert_xy_to_coordinate(my_x, my_y, &lat, &lon);
    char location_info[40]={0};
    sprintf(location_info, "%f,%f,%f", lat, lon, my_z);
    LOG_INFO("location list for sending: %s", location_info); 

    char *con1 = "{\"neighbor\":\"";
    char *con2 = "\",\"routing\":\"";
    char *con3 = "\",\"location\":\"";
    char *con4 = "\""; 

    
    strcat(data,str1);
    strcat(data,typestr);
    strcat(data,str2);
    strcat(data,tostr);
    strcat(data,str3);
    strcat(data,con1);
    strcat(data,neighbor_list);
    strcat(data,con2);
    strcat(data,routing_list);
    strcat(data,con3);
    strcat(data,location_info);
    strcat(data,con4);
    strcat(data,endstr); 
    
}

void *fanet_info_create_to_mobius(void *arg)
{
    int sd;
    struct sockaddr_in unicastAddr;
    int broadcastPermission = 1;

    sd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sd < 0)
        perror("cannot open socket");

    if (setsockopt(sd, SOL_SOCKET, SO_BROADCAST, &broadcastPermission, sizeof(broadcastPermission)) < 0)
        perror("sesockopt (SO_BROADCAST)");

    memset(&unicastAddr, 0, sizeof(unicastAddr));
    unicastAddr.sin_family = AF_INET;
    unicastAddr.sin_addr.s_addr = inet_addr(my_ip);
    unicastAddr.sin_port = htons(REMOTE_SERVER_PORT);

    struct datalink_header finfo_header;
    struct datalink_tail finfo_tail;
    struct datalink_payload finfo_payload;
    memset(&finfo_header, 0, sizeof(struct datalink_header));
    memset(&finfo_tail, 0, sizeof(struct datalink_tail));
    memset(&finfo_payload, 0, sizeof(struct datalink_payload));
    finfo_header.header1 = 0xAA;
    finfo_header.header2 = 0x55;
    finfo_header.dlen = sizeof(struct datalink_payload);
    finfo_header.src_id = my_id;
    finfo_header.src_port = 0;
    finfo_header.dst_id = 0; //###### destination_id 0 (send to gcs node) 
    finfo_header.dst_port = 0;
    finfo_header.priority = 0;
    finfo_header.msg_id = 79;  //####### message_id 9 
    finfo_tail.MSB = 12;
    finfo_tail.LSB = 31;
    finfo_payload.header = finfo_header;
    finfo_payload.tail = finfo_tail;

    while (1) {

         if (strcmp(mobius_direct_on_off, "ON") == 0) { //direct send data to mobius

            //send to local tas

            char sendata[1024]={0};
            create_data_send_to_tas(sendata);
            //ex) char *sendata = "{\"ctname\":\"UAV_11\",\"con\":\"neighbor: routing: location: \"}";
		    if (send(tas_sockfd, sendata, strlen(sendata), 0) == -1) {
                perror("send");
		        //exit(1);
				close(tas_sockfd);
		    	//struct hostent *he;
        		struct sockaddr_in their_addr;

        		//he = gethostbyname("localhost");
    
        		if ((tas_sockfd = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
            		LOG_ERROR("socket: socket creation error");
            		//exit(1);
        		}

				bzero((char *)&their_addr, sizeof(their_addr));
        		their_addr.sin_family = AF_INET;      /* host byte order */
        		their_addr.sin_port = htons(3105);    /* short, network byte order */
        		//their_addr.sin_addr.s_addr = *((struct in_addr *)he->h_addr);
        		their_addr.sin_addr.s_addr = inet_addr("127.0.0.1");
        		//bzero(&(their_addr.sin_zero), 8);     /* zero the rest of the struct */

        		if (connect(tas_sockfd, (struct sockaddr *)&their_addr, sizeof(struct sockaddr)) == -1) {
            		LOG_ERROR("connect: connection refused error");
            		//exit(1);
        		}
			}

        }
        
        else { //indirect send data to mobius

        	struct fanet_packet fanet_pkt_monitor;
            memset(&fanet_pkt_monitor, 0, sizeof(fanet_pkt_monitor));
        	fanet_pkt_monitor.type = 5;  //#### fanet monitoring information 
        	fanet_pkt_monitor.dst_count = 1;
        	fanet_pkt_monitor.dst_id[0] = 0; //#### destination node id 0 
        	struct position_table *item_position = search_position(fanet_pkt_monitor.dst_id[0]);
        	if (item_position!= NULL) {
            	fanet_pkt_monitor.dst_x_coord = item_position->x_coord;
            	fanet_pkt_monitor.dst_y_coord = item_position->y_coord;
            	fanet_pkt_monitor.dst_z_coord = item_position->z_coord;
        	}
        	fanet_pkt_monitor.src_id = my_id;
            fanet_pkt_monitor.seqno = 0;
        	fanet_pkt_monitor.hop_count = 0;
        	fanet_pkt_monitor.plen = sizeof(finfo_payload);

            struct to_gcs_msg app_msg;
        	char neighbor_list[40]={0};
            char routing_list[40]={0};
        	enumeration_neighbor_table(neighbor_list);
            enumeration_routing_table(routing_list);
        	LOG_INFO("neighbor list for sending: %s", neighbor_list); 
        	LOG_INFO("rouing list for sending: %s", routing_list); 
            strcpy(app_msg.neighbor_list, neighbor_list);
            strcpy(app_msg.routing_list, routing_list);

            float lat, lon;
            convert_xy_to_coordinate(my_x, my_y, &lat, &lon);
            app_msg.x_coord = lat;
            app_msg.y_coord = lon;
            app_msg.z_coord = my_z;
            app_msg.hop_count = 0;
            LOG_INFO("xyz: %f %f %f and hopcnt: %d", app_msg.x_coord, app_msg.y_coord, app_msg.z_coord, app_msg.hop_count);

        	memcpy(finfo_payload.data, (char*)&app_msg, sizeof(struct to_gcs_msg)); 
        	memcpy(fanet_pkt_monitor.data, (char*)&finfo_payload, fanet_pkt_monitor.plen);

        	int rc = sendto(sd, (char*)&fanet_pkt_monitor, sizeof(fanet_pkt_monitor), 0, (struct sockaddr *)&unicastAddr, sizeof(unicastAddr));
            LOG_INFO("return value: %d", rc);
        }

        //fanet info uplaod interval
        sleep(FINFO_SCHEDULE_INTERVAL);
    }

    close(sd);
}

void *target_to_fanet_data_processing()
{
    struct sockaddr_in servAddr;
    struct sockaddr_in cliAddr;
    char buf[MAX_MSG];
    int fd, n;
    socklen_t cliLen;

    fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) {
        LOG_ERROR("Socket creation failed");
        exit(1);
    }
    
    /*bind local server port */
    servAddr.sin_family = AF_INET;
    servAddr.sin_addr.s_addr = htonl(INADDR_ANY);
    servAddr.sin_port = htons(LOCAL_PROCESSING_PORT);

    if (bind(fd, (struct sockaddr *)&servAddr, sizeof(servAddr)) < 0) {
        LOG_ERROR("Cannot bind port number %d", LOCAL_PROCESSING_PORT);
        exit(1);
    }
    cliLen = sizeof(cliAddr);

    struct datalink_payload received_datalink;

    while (1) {
        n = recvfrom(fd, buf, MAX_MSG, 0, (struct sockaddr *)&cliAddr, &cliLen);
        if (n < 0) {
            LOG_ERROR("Cannot recvfrom()");
            //exit(1);
        }
        LOG_INFO("Target to Fanet Data Processing");
        LOG_INFO("From %s : UDP %u Data size: %d", inet_ntoa(cliAddr.sin_addr), ntohs(cliAddr.sin_port), n);
        memset(&received_datalink, 0, sizeof(struct datalink_payload));
        memcpy(&received_datalink, buf, n);
        LOG_INFO("%X %X %d %d %d %d %d %d %d %d %s", received_datalink.header.header1, received_datalink.header.header2, \
        received_datalink.header.dlen, received_datalink.header.seqno, received_datalink.header.src_id, received_datalink.header.src_port,\
        received_datalink.header.dst_id, received_datalink.header.dst_port, received_datalink.header.priority, \
        received_datalink.header.msg_id, received_datalink.data);
        
        //send to Mobius server
        if (received_datalink.header.msg_id == 78) { //case: datalink heartbeat msg

            /*char *mobiuson = "ON";
            if (strcmp(mobius_on_off, mobiuson) == 0) {

                char sendata[1024]={0};    
                create_heartbeat_send_to_tas(received_datalink.header.src_id, received_datalink.header.seqno, sendata);

                if (send(tas_sockfd, sendata, strlen(sendata), 0) == -1) {
                    perror("send");
		            //exit(1);
		        }
                
	        }*/

			struct pm_msg to_pm_msg;
			memcpy(&to_pm_msg, received_datalink.data, sizeof(struct pm_msg));
			//LOG_WARNING("sec: %d usec: %d", to_pm_msg.sec, to_pm_msg.usec); 

			struct timeval tv;
			gettimeofday(&tv, NULL);
			LOG_WARNING("diff sec: %d diff usec: %d", tv.tv_sec - to_pm_msg.sec, tv.tv_usec - to_pm_msg.usec);
		
	

        }
        else if (received_datalink.header.msg_id == 79) { //case: datalink fanet info msg

            char *mobiuson = "ON";
            if (strcmp(mobius_on_off, mobiuson) == 0) {

	        }

        } else if (received_datalink.header.msg_id == 80) { //case: datalink command msg

            LOG_INFO("Notification message delivered via Routing Call process");
            char cmd_msg[60] = {0};
            strcpy(cmd_msg, received_datalink.data);
            send_control_message(cmd_msg, strlen(cmd_msg)); // send to ethernet socket
            LOG_INFO("Command Message: %s", cmd_msg);

        }
            //TODO
    }
}


int main(int argc, char **argv)
{
    if (argc != 3) {
        fputs("usage: ./route |-c|-f| |-h|-g|\n", stderr);
        fputs("       -c: output console\n", stderr);
        fputs("       -f: output file\n", stderr);
        fputs("       -h: routing hop \n", stderr);
        fputs("       -g: routing geolocation\n", stderr);
        return -1;
    }

    char *strF = "-f";
    char *strC = "-c";
    char *strH = "-h";
    char *strG = "-g";

    if (!strcmp(strF, argv[1]))
        selector_logout = TRUE;
    else if (!strcmp(strC, argv[1]))
        selector_logout = FALSE;
    else {
        fputs("error: the arguments are wrong.\n", stderr);
        return -1;
    }

    if (!strcmp(strH, argv[2]))
        selector_hop_routing = TRUE;
    else if (!strcmp(strG, argv[2]))
        selector_hop_routing = FALSE;
    else {
        fputs("error: the arguments are wrong.\n", stderr);
        return -1;
    }

    LOG_INFO("FANET START");

    if (parse_geolocation_file() < 0) {
        LOG_ERROR("georef.txt wrong format");
        return -1;
    }
    //struct map_id_to_ip ipmap[TABLE_SIZE]={};
    if (parse_config_from_file(ipmap) < 0) {
        LOG_ERROR("config.txt wrong format");
        return -1;
    }

    int i;
    for (i = 0; i < TABLE_SIZE; i++) {
        bcast_seqno_table[i] = 0;
        rc_msg_seqno_table[i] = 0;
        rc_msg_time_table[i] = 0;
        neighborTable[i] = NULL;
        routingTable[i] = NULL;
        positionTable[i] = NULL;
    }

	//unsigned char buf_com1[MAX_MSG];
	unsigned char buf_com1[128];
	unsigned char buf_com2[MAX_MSG];

	struct termios newtio_com1, newtio_com2;
   	fd_com1 = open(SERIALDEVICE_COM1, O_RDWR | O_NOCTTY | O_NONBLOCK);
	fd_com2 = open(SERIALDEVICE_COM2, O_RDWR | O_NOCTTY | O_NONBLOCK);

	if (fd_com1 < 0) {
		LOG_ERROR("%s: No such file or directory", SERIALDEVICE_COM1);
	}

	if (fd_com2 < 0) {
		LOG_ERROR("%s: No such file or directory", SERIALDEVICE_COM2);
	}

	memset(&newtio_com1, 0, sizeof(newtio_com1));
	//newtio_com1.c_iflag = IGNPAR | ICRNL; //non-parity
	newtio_com1.c_iflag = IGNPAR; // | IGNCR; //non-parity
	newtio_com1.c_oflag = 0;
	newtio_com1.c_cflag= CS8 | BAUDRATE_COM1 | CREAD | CLOCAL | HUPCL;
	newtio_com1.c_lflag &= ~ICANON;
    //newtio_com1.c_cc[VTIME] = 10;
    //newtio_com1.c_cc[VMIN] = 72;

	tcflush(fd_com1, TCIFLUSH);
	tcsetattr(fd_com1, TCSANOW, &newtio_com1);

	memset(&newtio_com2, 0, sizeof(newtio_com2));
	//newtio_com2.c_iflag = IGNPAR | ICRNL; //non-parity
	newtio_com2.c_iflag = IGNPAR; // | IGNCR; //non-parity
	newtio_com2.c_oflag = 0;
	newtio_com2.c_cflag= CS8 | BAUDRATE_COM2 | CREAD | CLOCAL | HUPCL;
	newtio_com2.c_lflag &= ~ICANON;
    //newtio_com2.c_cc[VTIME] = 10;
    //newtio_com2.c_cc[VMIN] = 72;

	tcflush(fd_com2, TCIFLUSH);
	tcsetattr(fd_com2, TCSANOW, &newtio_com2);

    int sockfd;
    int ethfd; //socket for receiving geolocation
    struct sockaddr_in servAddr, localprocessingAddr; //, remoteServAddr[TABLE_SIZE], broadcastAddr;
    char msg[MAX_MSG];
    int broadcast = 1;

    /* socket creation for receiving*/
    sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) {
          LOG_ERROR("Socket creation failed");
          exit(1);
    }
    if (setsockopt(sockfd, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast)) == -1) {
          perror("setsockopt (SO_BROADCAST)");
          exit(1);
    }
    /* bind local server port */
	memset(&servAddr, 0, sizeof(servAddr));
    servAddr.sin_family = AF_INET;
    servAddr.sin_addr.s_addr = htonl(INADDR_ANY);
    servAddr.sin_port = htons(LOCAL_SERVER_PORT);

    if (bind(sockfd, (struct sockaddr *)&servAddr, sizeof(servAddr)) < 0) {
        LOG_ERROR("Cannot bind port number %d", LOCAL_SERVER_PORT);
        exit(1);
    }

    LOG_INFO("Waiting for data on port UDP %u", LOCAL_SERVER_PORT);

	/* socket creation for dji data*/
	ethfd = socket(AF_INET, SOCK_DGRAM, 0);
	if (ethfd < 0) {
          LOG_ERROR("Socket creation failed");
          exit(1);
    }
	
    if (setsockopt(ethfd, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast)) == -1) {
          perror("setsockopt (SO_BROADCAST)");
          exit(1);
    }

    /* bind ethernet server port */
    struct sockaddr_in ethservAddr;
	memset(&ethservAddr, 0, sizeof(ethservAddr));
    ethservAddr.sin_family = AF_INET;
    ethservAddr.sin_addr.s_addr = htonl(INADDR_ANY);
    ethservAddr.sin_port = htons(ETH_SERVER_PORT);
	
    if (bind(ethfd, (struct sockaddr *)&ethservAddr, sizeof(ethservAddr)) < 0) {
        LOG_ERROR("Cannot bind port number %d", ETH_SERVER_PORT);
        exit(1);
    }
	
	LOG_INFO("Waiting for data on port UDP %u", ETH_SERVER_PORT);
	
	/* remote server address setting */
    for (i = 0; i < TABLE_SIZE; i++) {
        remoteServAddr[i].sin_family = AF_INET;
        remoteServAddr[i].sin_addr.s_addr = inet_addr(ipmap[i].ip);
        remoteServAddr[i].sin_port = htons(REMOTE_SERVER_PORT);
    }

    // local processing server address setting
    memset(&localprocessingAddr, 0, sizeof(localprocessingAddr));
    localprocessingAddr.sin_family = AF_INET;
    localprocessingAddr.sin_addr.s_addr = inet_addr(my_ip);
    localprocessingAddr.sin_port = htons(LOCAL_PROCESSING_PORT);

    /* broadcast address setting */
    memset(&broadcastAddr, 0, sizeof(broadcastAddr));
    broadcastAddr.sin_family = AF_INET;
    broadcastAddr.sin_addr.s_addr = inet_addr(BCAST_IP_ADDRESS);
    broadcastAddr.sin_port = htons(REMOTE_SERVER_PORT);

	/* socket creation for sending */
	sd = socket(AF_INET, SOCK_DGRAM, 0);
	if (sd < 0) {
		LOG_ERROR("Socket creation failed");
		exit(1);
	}

	if (setsockopt(sd, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast)) == -1) {
		LOG_ERROR("setsockopt (SO_BROADCAST)");
        exit(1);
	}
	
	/* socket creation for sending */
	sd_thread = socket(AF_INET, SOCK_DGRAM, 0);
	if (sd_thread < 0) {
		LOG_ERROR("Socket creation failed");
		exit(1);
	}

	if (setsockopt(sd_thread, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast)) == -1) {
		LOG_ERROR("setsockopt (SO_BROADCAST)");
        exit(1);
	}


    bcast_sd = socket(AF_INET, SOCK_DGRAM, 0);
    if (bcast_sd < 0) {
        perror("Socket creation failed");
        exit(1);
    }

	if (setsockopt(bcast_sd, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast)) == -1) {
		LOG_ERROR("setsockopt (SO_BROADCAST)");
        exit(1);
    }


    bcast_sd_thread = socket(AF_INET, SOCK_DGRAM, 0);
    if (bcast_sd < 0) {
        perror("Socket creation failed");
        exit(1);
    }

	if (setsockopt(bcast_sd_thread, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast)) == -1) {
		LOG_ERROR("setsockopt (SO_BROADCAST)");
        exit(1);
    }

    struct pollfd poll_events[NUM_POLL_FD];
    poll_events[0].fd = fd_com1;
    poll_events[0].events = POLLIN | POLLERR;
    poll_events[0].revents = 0;
    poll_events[1].fd = fd_com2;
    poll_events[1].events = POLLIN | POLLERR;
    poll_events[1].revents = 0;
    poll_events[2].fd = sockfd;
    poll_events[2].events = POLLIN | POLLERR;
    poll_events[2].revents = 0;
    poll_events[3].fd = ethfd;
    poll_events[3].events = POLLIN | POLLERR;
    poll_events[3].revents = 0;	


    //create_ae_and_contaitner for fanet monitoring ui
    if (strcmp(mobius_on_off, "ON") == 0) {
     
        //struct hostent *he;
        struct sockaddr_in their_addr;

        //he = gethostbyname("localhost");
    
        if ((tas_sockfd = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
            LOG_ERROR("socket: socket creation error");
            //exit(1);
        }

		bzero((char *)&their_addr, sizeof(their_addr));
        their_addr.sin_family = AF_INET;      /* host byte order */
        their_addr.sin_port = htons(3105);    /* short, network byte order */
		their_addr.sin_addr.s_addr = inet_addr("127.0.0.1");
        //their_addr.sin_addr.s_addr = *((struct in_addr *)he->h_addr);
        //bzero(&(their_addr.sin_zero), 8);     /* zero the rest of the struct */

        if (connect(tas_sockfd, (struct sockaddr *)&their_addr, sizeof(struct sockaddr)) == -1) {
            LOG_ERROR("connect: connection refused error");
            //exit(1);
        }

    }


    int pthread_res;
    pthread_t tst_thread;
    if (strcmp(mobius_on_off, "ON") == 0) {
    	pthread_res = pthread_create(&tst_thread, NULL, &tas_thread, NULL);
    	if (pthread_res != 0) {
            perror("Thread creation failed");
            exit(1);
   		}
    }

    int pthread_hello; //periodic hello message broadcasting thread
    pthread_t hello_thread;
    pthread_hello = pthread_create(&hello_thread, NULL, &schedule_hello_msg, NULL);
    if (pthread_hello != 0) {
        perror("Thread creation failed");
        exit(1);
    }

    int pthread_rc; //periodic routing control message broadcasting thread
    pthread_t rc_thread;
    pthread_rc = pthread_create(&rc_thread, NULL, &schedule_rc_msg, NULL);
    if (pthread_rc != 0) {
        perror("Thread creation failed");
        exit(1);
    }

    int pthread_fs; //handling to fanet data processing thread
    pthread_t fs_thread;
    pthread_fs = pthread_create(&fs_thread, NULL, &target_to_fanet_data_processing, NULL);
    if (pthread_fs != 0) {
        perror("Thread creation failed");
        exit(1);
    }

    int pthread_mobius_interaction; //mobius fanet monitoring data upload thread
	pthread_t mobius_interaction_thread;
    if (strcmp(mobius_on_off, "ON") == 0) {
    	pthread_mobius_interaction = pthread_create(&mobius_interaction_thread, NULL, &fanet_info_create_to_mobius, NULL);
        if (pthread_mobius_interaction != 0) {
            perror("Thread creation failed");
        }
    }

    //int pthread_hb;
    //pthread_t hb_thread;
    //if (strcmp(mobius_on_off, "ON") == 0) {
    //    pthread_hb = pthread_create(&hb_thread, NULL, &generate_heart_beat_msg, NULL);
    //    if (pthread_hb != 0) {
    //        perror("Thread creation failed");
    //        exit(1);
    //    }
    //}

    int pthread_serial;
    pthread_t serial_thread;
    pthread_serial = pthread_create(&serial_thread, NULL, &do_serial_data_processing, NULL);
    if (pthread_serial != 0) {
        perror("Thread creation failed");
        exit(1);
    }

    //unsigned char receive_buffer_com1[15000]={0}; &&
    //unsigned char *mp_com1 = receive_buffer_com1;
    //int csize_com1 = 0;
    //int *p_csize_com1 = &csize_com1;
    //int asize_com1 = 0;
    //int *p_asize_com1 = &asize_com1;

    LOG_DEBUG("*mp_com1 address: %p", mp_com1); 
    LOG_DEBUG("receive_buffer_com1 address: %p", receive_buffer_com1);

    unsigned char receive_buffer_com2[15000]={0};
    unsigned char *mp_com2 = receive_buffer_com2;
    int csize_com2 = 0;
    int *p_csize_com2 = &csize_com2;
    int asize_com2 = 0;
    int *p_asize_com2 = &asize_com2;

    int cliLen, n;
    int nread;
    struct sockaddr_in cliAddr;
    int poll_state;

    while (1) {
	    poll_state = poll((struct pollfd*)&poll_events, NUM_POLL_FD, 1000);

        if (poll_state > 0) {
	    	if (poll_events[0].revents & POLLIN) { //from serial(FC) ####
               	memset(buf_com1, 0, sizeof(buf_com1));
				nread = read(fd_com1, buf_com1, sizeof(buf_com1));
				//LOG_DEBUG("buf_com1 read size %d", nread);
				if (nread > 0) {
                    pthread_mutex_lock(&mutex);
					if (asize_com1 < 14500) {
						//LOG_DEBUG("asize_com1 is less than 14500");
						mp_com1 = memcpy(mp_com1 + csize_com1, buf_com1, nread);
						csize_com1 = nread;
						asize_com1 += csize_com1;
					} else {
						LOG_DEBUG("asize_com1 is greater than 14500");
						memset(receive_buffer_com1, 0, sizeof(receive_buffer_com1));
						mp_com1 = receive_buffer_com1;
						csize_com1 = 0;
						asize_com1 = 0;
					}
                    pthread_mutex_unlock(&mutex);
                   	//process_serial_to_sock(&mp_com1, p_csize_com1, p_asize_com1, receive_buffer_com1, sd, remoteServAddr, bcast_sd, broadcastAddr);
    	    	}
           	}
           	if (poll_events[0].revents & POLLERR) {
               	LOG_ERROR("Serial FC POLLERR");
               	break;
           	}

	    	if (poll_events[1].revents & POLLIN) { //from serial(DMAC) ####
               	memset(buf_com2, 0 , sizeof(buf_com2));
	    		nread = read(fd_com2, buf_com2, sizeof(buf_com2));
				LOG_DEBUG("buf_com2 read size %d", nread);
		    	if (nread > 0) {
                   	mp_com2 = memcpy(mp_com2 + csize_com2, buf_com2, nread);
                   	csize_com2 = nread;
                   	asize_com2 += csize_com2;
                   	process_serial_to_sock(&mp_com2, p_csize_com2, p_asize_com2, receive_buffer_com2, sd, remoteServAddr, bcast_sd, broadcastAddr);
               	}
	    	}	
           	if (poll_events[1].revents & POLLERR) {
               	LOG_ERROR("Serial DMAC POLLERR");
               	break;
           	}

	    	if (poll_events[2].revents & POLLIN) { //from udp socket ####
	        	LOG_DEBUG("Received from UDP socket");
               	/* init buffer */
               	memset(msg,0,MAX_MSG);
               	/* receive message */
               	cliLen = sizeof(cliAddr);
               	n = recvfrom(sockfd, msg, MAX_MSG, 0, (struct sockaddr *)&cliAddr, &cliLen);
               	LOG_INFO("Received from IP:%s port:%u size:%d", inet_ntoa(cliAddr.sin_addr), ntohs(cliAddr.sin_port), n);
               	handle_packet_from_sock(n, msg, cliAddr, remoteServAddr, bcast_sd, broadcastAddr, sd, localprocessingAddr);
	    	}
           	if (poll_events[2].revents & POLLERR) {
               	LOG_ERROR("UDP socket POLLERR");
               	break;
           	}
			
			if (poll_events[3].revents & POLLIN) { //from ethernet socket ####
	        	LOG_DEBUG("Received from Ethernet socket");
               	/* init buffer */
               	memset(msg, 0, MAX_MSG);
               	/* receive message */
				cliLen = sizeof(cliAddr);
               	n = recvfrom(ethfd, msg, MAX_MSG, 0, (struct sockaddr *)&cliAddr, &cliLen);
               	LOG_INFO("Ethernet Received from IP:%s port:%u size:%d", inet_ntoa(cliAddr.sin_addr), ntohs(cliAddr.sin_port), n);
				handle_packet_from_ethnet(n, msg);
       	    }
            if (poll_events[3].revents & POLLERR) {
                LOG_ERROR("Ethernet socket POLLERR");
                break;
            }
        }
	}

    close(fd_com1);
    close(fd_com2);
    close(sockfd);

    return 0;

}
