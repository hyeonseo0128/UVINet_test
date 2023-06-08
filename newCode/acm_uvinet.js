const dgram = require('dgram');
const { SerialPort } = require('serialport');
const os = require('os');
const broadcastAddress = require('broadcast-address');

const UCAST_HOST = '127.0.0.1';
const UCAST_PORT = 20001; 
const BCAST_HOST = broadcastAddress('eth0');
const BCAST_PORT = 20002; 
const SERIAL_PORT = '/dev/ttyTHS1'; 
const BAUD_RATE = 115200; 

const udpSocket = dgram.createSocket('udp4');
const bcastSocket = dgram.createSocket('udp4');

const serialPort = new SerialPort({
    path: SERIAL_PORT,
    baudRate: BAUD_RATE
});

const net_int = os.networkInterfaces();
const MY_IP_ADDRESS = net_int['eth0'][0].address;
console.log('my ip address:', MY_IP_ADDRESS);

let id_ip_dic = {1: '172.30.10.30', 2: '172.30.10.30', 3: '172.30.10.30', 255: '172.30.10.255',
                 4: '172.30.10.30', 5: '172.30.10.30', 6: '172.30.10.30', 7: '172.30.10.255',
                 8: '172.30.10.30', 9: '172.30.10.30', 10: '172.30.10.30'};
console.log(id_ip_dic);

udpSocket.on('message', (message, rinfo) => {
    console.log('udpsocket received:', rinfo.address, rinfo.port, 'message:', message.toString('hex'));
    if (rinfo.address ==  MY_IP_ADDRESS) {
        console.log('ucasts my ip address');
    } else {
        console.log('ucast other ip address');
        serialPort.write(message, (err) => {
            if (err) {
                console.error('Serial port write error:', err);
            }
        });
    }
});


bcastSocket.on('message', (message, rinfo) => {
    console.log('bcastsocket received:', rinfo.address, rinfo.port, 'message:', message.toString('hex'));
    if (rinfo.address == MY_IP_ADDRESS) {
        console.log('bcast my ip address');
    } else {
        console.log('bcast other ip address');
        serialPort.write(message, (err) => {
            if (err) {
                console.error('Serial port write error:', err);
            }
        });
    }
});

let count = 0;
let b_count = 0;
let buffer_array = Buffer.alloc(0);
let parsing_array = Buffer.alloc(0);

serialPort.on('data', (data) => {
    console.log('serial received:' + data.toString('hex'));
    size = data.length
    //console.log('size:',size);
    if (size > 0){
        count = count + 1;
        buffer_array = Buffer.concat([buffer_array, data]);
        //console.log('buffer', buffer_array);
        for (i = 0; i < buffer_array.length; i++){
            if (i + 2 < buffer_array.length){
                if (buffer_array[i] == 0xaa && buffer_array[i+1] == 0x55){
                    //console.log("###init###");
                    //console.log('i value', i, 'barray - (i)', buffer_array.length - (i), 'length value', buffer_array[i+2]);
                    if ((buffer_array.length - (i)) >= buffer_array[i+2] + 12){
                        k = buffer_array[i+2] + 12;
                        //console.log('k value', k);
                        
                        if (i > 0){
                            console.log('####### before del', buffer_array);
                            buffer_array = buffer_array.slice(i, buffer_array.length);
                            console.log('####### after del', buffer_array);
                            break;
                        }
                        
                        if (i + k <= buffer_array.length){
                            //console.log('parsing');
                            dl = buffer_array.slice(i, k);
                            //console.log('parsing data', dl.toString('hex'));
                            buffer_array = buffer_array.slice(k, buffer_array.length);

                            if (dl.length > 0){
                                console.log('header1', dl[0].toString(16), 'header2', dl[1].toString(16), 'payload length', dl[2], 'packet sequence', dl[3], 'source ID', dl[4], 'port', dl[5], 'destination ID', dl[6],
                                            'port', dl[7], 'packet priority', dl[8], 'message Id', dl[9]);
                            
                                id = dl[6];
                                if (id == 255){
                                    ip = id_ip_dic[id.toString()];
                                    console.log('id', id, 'ip', ip);
                                    //send_broadcast();
                                    bcastSocket.send(dl, 0, dl.length, BCAST_PORT, BCAST_HOST, (err) => {
                                        if (err) {
                                            console.error('BCAST send error:', err);
                                        }
                                    });

                                    break;
                                }
                                
                                ip = id_ip_dic[id.toString()];
                                console.log('id', id, 'ip', ip);
                                //send_unicast();
                                udpSocket.send(dl, 0, dl.length, UCAST_PORT, ip, (err) => {
                                    if (err) {
                                        console.error('UCAST send error:', err);
                                    }
                                });
                            }
                        }
                    }
                }           
            }
        }
    }


});

udpSocket.on('listening', () => {
    console.log('UCAST socket listening on', UCAST_HOST + ':' + UCAST_PORT);
});

bcastSocket.on('listening', () => {
    bcastSocket.setBroadcast(true);
    console.log('BCAST socket listening on', BCAST_HOST + ':' + BCAST_PORT);
});

serialPort.on('open', () => {
    console.log('Serial port connected on', SERIAL_PORT);
});

udpSocket.bind(UCAST_PORT);
bcastSocket.bind(BCAST_PORT);
