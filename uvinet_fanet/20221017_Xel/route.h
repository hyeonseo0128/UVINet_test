#ifndef __ROUTE__
#define __ROUTE__

#define SERIALDEVICE_COM1 "/dev/ttyTHS1"
#define SERIALDEVICE_COM2 "/dev/ttyUSB0"
#define BAUDRATE_COM1 B57600
#define BAUDRATE_COM2 B57600
#define NUM_POLL_FD 4 //serial 1,2 and udp socket 2 (for wave and ethernet)

#define LOCAL_SERVER_PORT 1500
#define REMOTE_SERVER_PORT 1500
#define LOCAL_PROCESSING_PORT 3000
#define ETH_SERVER_PORT 2376

#define MAX_MSG 512
#define TABLE_SIZE 100 //number of all nodes
#define HELLO_BUF_SIZE 10

#define BCAST_IP_ADDRESS "172.30.100.255"
#define ETH_IP_ADDRESS "192.168.10.2"

#define CONFIG_FILE_NAME "config.txt"
#define GEO_FILE_NAME "georef.txt"

typedef enum {FALSE, TRUE} BOOL;

int my_id;
char my_ip[16];
char my_type[8];
char mqtt_ip[16];
char mobius_on_off[8];
char mobius_direct_on_off[8];
char testbed_name[16];
float my_x, my_y, my_z;
int selector_logout;
int selector_hop_routing;
int fd_com1, fd_com2;

int PKT_DELIVERY_RATE;
int HELLO_SCHEDULE_INTERVAL;
int MAX_HOP;
int VALID_DURATION;
int SCHEDULE_INTERVAL;
int WAKEUP_LIMIT;
int VALID_NEIGHBOR_GAP;
int FINFO_SCHEDULE_INTERVAL;

struct to_gcs_msg {
    char neighbor_list[40];
    char routing_list[40];
    float x_coord;
    float y_coord;
    float z_coord;
    int hop_count;    
};

struct pm_msg {
	int sec;
	int usec;
};

struct map_id_to_ip {
    int id;
    char ip[16];
    char type[8]; //GCS,UGV,UAV
};

struct datalink_header {
    unsigned char header1;
    unsigned char header2;
    unsigned char dlen;
    unsigned char seqno;
    unsigned char src_id;
    unsigned char src_port;
    unsigned char dst_id;
    unsigned char dst_port;
    unsigned char priority;
    unsigned char msg_id;
};

struct datalink_tail {
    unsigned char MSB;
    unsigned char LSB;
};

struct datalink_payload {
    struct datalink_header header;
    char data[100];
    struct datalink_tail tail;
};

struct fanet_packet {
    unsigned char type; //0(rc_msg),1(unicast_data),2(bcast_data),3(hello_msg),4(rc_ack),5(fanet_moniroting_info),6(hello_ack),7(heart_beat)
    unsigned char src_id;
    unsigned char dst_count;
    unsigned char dst_id[5];
    float dst_x_coord;
    float dst_y_coord;
    float dst_z_coord;
    unsigned short seqno;
    unsigned short plen;
    unsigned short hop_count;
    char data[200];
};

struct fanet_rc_message {
    unsigned char node_id;
    char ip_address[16];
    float x_coord;
    float y_coord;
    float z_coord;
    unsigned int velocity;
    unsigned int direction;
    unsigned short hop_count;
    unsigned short prior_id;
    unsigned short pkt_count;
    unsigned short time_gap;
};

struct fanet_rc_ack_message {
    unsigned char node_id;
    float x_coord;
    float y_coord;
    float z_coord;
    unsigned int velocity;
    unsigned int direction;
    unsigned short ack_id; //received_bcast_rc_message_id
    unsigned short seqno;
    unsigned short hop_count;
};

struct fanet_hello_ack_message {
    unsigned char node_id;
    float x_coord;
    float y_coord;
    float z_coord;
    unsigned short ack_id; //received_bcast_hello_message_id
    unsigned short pkt_count;
    unsigned short time_gap;
    unsigned char route_count;
    unsigned char dst[20];
    unsigned char next[20];
    unsigned char hop[20];
};

struct neighbor_table {
    int key; //node_id
    struct fanet_rc_message item;
    float link_quality;
    int timestamp;
};

struct routing_table {
    int key; //dest_node_id
    int nexthop;
    int hop_count;
    int timestamp;
};

struct position_table {
    int key; //node_id
    float x_coord;
    float y_coord;
    float z_coord;
    int timestamp;
};

int * hello_table[TABLE_SIZE];
int bcast_seqno_table[TABLE_SIZE];
int rc_msg_seqno_table[TABLE_SIZE];
int rc_msg_time_table[TABLE_SIZE];
struct neighbor_table* neighborTable[TABLE_SIZE];
struct routing_table* routingTable[TABLE_SIZE];
struct position_table* positionTable[TABLE_SIZE];

struct vehicle_navi_data_rsp {
    unsigned char current_op_mode;
    unsigned char uplink_packet_hz;
    unsigned char roll_angle;
    unsigned char pitch_angle;
    unsigned char yaw_course_heading[3];
    unsigned char air_ground_speed[3];
    unsigned char pressure_altitude[2];
    unsigned char gps_altitude[2];
    unsigned char rate_of_climb;
    unsigned char gps_operation;
    unsigned char latitude[4];
    unsigned char longitude[4];
    unsigned char warning;
};


#endif
