import logging
from socket import *
import time
import threading
import pickle
import netifaces as ni
import serial
from struct import *
import binascii
import time
import config


logging.basicConfig(
    format='%(asctime)s.%(msecs)03d:%(levelname)s:[%(filename)s:%(lineno)d] > %(message)s',
    datefmt='%m/%d/%Y %H:%M:%S',
    level=logging.DEBUG
)


MY_ID = config.ID["MY_ID"]
MY_IP_ADDRESS = config.IP['MY_IP_ADDRESS']
# MY_IP_ADDRESS = ni.ifaddresses('eth0')[ni.AF_INET][0]['addr']

node_info = [0, MY_ID, MY_IP_ADDRESS]  # seqno, myid, myipaddress
neighbor_dict = {}
id_ip_dict = {MY_ID : MY_IP_ADDRESS}


BCAST_IP_ADDRESS = config.IP['BCAST_IP_ADDRESS']
BCAST_PORT_NUMBER = config.IP['BCAST_PORT_NUMBER']
# BCAST_IP_ADDRESS = ni.ifaddresses('eth0')[ni.AF_INET][0]['broadcast']

UCAST_IP_ADDRESS = config.IP['UCAST_IP_ADDRESS']
UCAST_PORT_NUMBER = config.IP['UCAST_PORT_NUMBER']

ID_IP_SERVER_IP_ADDRESS = config.IP['ID_IP_SERVER_IP_ADDRESS']
ID_IP_SERVER_PORT_NUMBER = config.IP['ID_IP_SERVER_PORT_NUMBER']

BMSG_PORT_NUMBER = config.IP['BMSG_PORT_NUMBER']

bcast_send_socket = socket(family=AF_INET, type=SOCK_DGRAM)
bcast_send_socket.setsockopt(SOL_SOCKET, SO_BROADCAST, 1)

bmsg_recv_socket = socket(family=AF_INET, type=SOCK_DGRAM)
bmsg_recv_socket.setsockopt(SOL_SOCKET, SO_BROADCAST, 1)

bmsg_recv_socket.bind(('', BMSG_PORT_NUMBER))

ucast_send_socket = socket(family=AF_INET, type=SOCK_DGRAM)
ucast_recv_socket = socket(family=AF_INET, type=SOCK_DGRAM)

ucast_recv_socket.bind(('', UCAST_PORT_NUMBER))

id_ip_client_socket = socket(family=AF_INET, type=SOCK_DGRAM)
id_ip_client_socket.settimeout(3)

id_ip_server_socket = socket(family=AF_INET, type=SOCK_DGRAM)
id_ip_server_socket.bind(('', ID_IP_SERVER_PORT_NUMBER))

#ser = serial.Serial('/dev/ttyTHS1', 57600, timeout=0)
ser = serial.Serial(config.Serial['Path'], config.Serial['Baud'], timeout=config.Serial['Timeout'])


def serial_data_reader():
    #ser = serial.Serial('/dev/ttyTHS1', 57600, timeout=0)
    count = 0
    b_count = 0
    buffer_array = []
    parsing_array = []

    while True:
        if ser.readable():
            res = ser.read()
            size = len(res)
            if size > 0:
                count = count + 1
                # print("count:{} size:{} {} bytes".format(count, size, res))
                buffer_array.append(res)
                # print(buffer_array)
                for i in range(len(buffer_array)):
                    if i + 2 < len(buffer_array):
                        if buffer_array[i] == b'\xAA' and buffer_array[i + 1] == b'\x55':
                            # print('find')
                            u_data = unpack('>B', buffer_array[i + 2])
                            # print("data size: {}".format(u_data))
                            if len(buffer_array[i:]) >= int(u_data[0])+12:
                                k = int(u_data[0])+12

                                if i > 0:
                                    print("############ before del {}".format(buffer_array))
                                    del(buffer_array[:i])
                                    print("############ after del {}".format(buffer_array))
                                    break


                                if i + k <= len(buffer_array):
                                    # parsing_array.append(buffer_array[i:k])
                                    print("parsing")
                                    #print("buffer array size: {}".format(len(buffer_array)))
                                    # print("buffer array: {}".format(buffer_array))
                                    #  print("i value {} k value {}".format(i, k))
                                    dl = b''.join(buffer_array[i:k])
                                    print(binascii.hexlify(dl))
                                    print(dl)

                                    # time.sleep(0.05)
                                    # ser.write(dl)
                                    # print(parsing_array)
                                    del buffer_array[i:k]

                                    if len(dl) > 0:
                                        try:
                                            up_parser_start = ">ccBBBBBBBB"
                                            up_parser_mid = k - 12
                                            up_parser_end = "sBB"
                                            up_parser = "%s%d%s" % (up_parser_start, up_parser_mid, up_parser_end)
                                            print(up_parser)
                                            un_dl = unpack(up_parser, dl)
                                            #un_dl = unpack('>ccBBBBBBBB50sBB', dl)
                                            print(
                                            "header1 {} header2 {} payload length {} packet sequence {} source ID {} port {} destination ID {} port {} packet priority {} message ID {}".format( \
                                                un_dl[0], un_dl[1], un_dl[2], un_dl[3], un_dl[4], un_dl[5], un_dl[6],
                                                un_dl[7], un_dl[8], un_dl[9]))
                                            print(un_dl)

                                            id = un_dl[6]

                                            if id == 255:
                                                ip = '172.30.100.255'
                                                print(id, ip)
                                                bcast_send_socket.sendto(dl, (BCAST_IP_ADDRESS, BMSG_PORT_NUMBER))
                                                break

                                            if id in id_ip_dict:
                                                print(f"Yes, key: '{id}' exists in dictionary")
                                                ip = id_ip_dict[id]
                                                print(id, ip)
                                                ucast_sender(dl, ip)
                                            else:
                                                print(f"No, key: '{id}' does not exists in dictionary")
                                        except Exception as e:
                                            print(e)
                                            continue


serial_dl_thread = threading.Thread(target=serial_data_reader)
serial_dl_thread.start()


def bmsg_receiver():
    while True:
        msg, addr = bmsg_recv_socket.recvfrom(1024)
        if addr[0] == MY_IP_ADDRESS:
            logging.debug("bmsg my ip address")
        else:
            logging.debug("bmsg other ip address")


        logging.debug('----recv---- {0} {1} {2}'.format(len(msg), msg, addr))


bmsg_recv_thread = threading.Thread(target=bmsg_receiver)
bmsg_recv_thread.start()

def ucast_receiver():
    count = 0
    while True:
        msg, addr = ucast_recv_socket.recvfrom(1024)
        if addr[0] == MY_IP_ADDRESS:
            logging.debug("ucast my ip address")
            continue
        else:
            logging.debug("ucast other ip address")

        logging.debug('----recv---- {0} {1} {2} {3}'.format(count, len(msg), binascii.hexlify(msg), addr))
        count = count + 1
        ser.write(msg)


def ucast_sender(message, ip_address):
    # message = ['ucast message hot what comes down', 'this is message for hot']
    # for k in range(10000):
    #     time.sleep(1)
    #     for i in range(1):
    logging.debug('Sending.. Message {0}'.format(message))
    ucast_send_socket.sendto(message, (ip_address, UCAST_PORT_NUMBER))


ucast_recv_thread = threading.Thread(target=ucast_receiver)
ucast_recv_thread.start()

def id_ip_client():
    count = 0
    while True:
        if count < 50:
            time.sleep(3)
        else:
            time.sleep(20)
        count = count + 1

        message = pickle.dumps(node_info)
        id_ip_client_socket.sendto(message, (ID_IP_SERVER_IP_ADDRESS, ID_IP_SERVER_PORT_NUMBER))
        try:
            msg, addr = id_ip_client_socket.recvfrom(1024)
            logging.debug('----recv from server---- {0} {1}'.format(msg, addr))

            server_id_ip_dic = pickle.loads(msg)
            logging.debug(server_id_ip_dic)

            id_ip_dict.clear()
            for dkey, dvalue in server_id_ip_dic.items():
                id_ip_dict[dkey] = dvalue
            logging.info('local id_ip table {}'.format(id_ip_dict))
        except:
            logging.error("Timeout raised")
            continue


def id_ip_server():
    while True:
        msg, addr = id_ip_server_socket.recvfrom(1024)
        if addr[0] == MY_IP_ADDRESS:
            logging.debug("ucast my ip address")
            continue
        else:
            logging.debug("id ip ucast other ip address")

        received_node_id_ip = pickle.loads(msg)
        logging.debug('----recv id ip info---- {0} {1}'.format(received_node_id_ip, addr))

        id_ip_dict[received_node_id_ip[1]] = received_node_id_ip[2]
        logging.info(id_ip_dict)

        message = pickle.dumps(id_ip_dict)
        id_ip_server_socket.sendto(message, addr)


#id_ip_server_thread = threading.Thread(target=id_ip_server)
#id_ip_server_thread.start()

id_ip_client_thread = threading.Thread(target=id_ip_client)
id_ip_client_thread.start()



