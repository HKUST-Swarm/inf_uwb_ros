#include "uwb_helper.h"

#include <algorithm>
#include <fcntl.h>
#include <iostream>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>

#define NODE_HEADER_LEN_FRAME0 11
#define NODE_HEADER_LEN_FRAME1 27
#define NODE_HEADER_LEN_FRAME2 119
#define REMOTE_HEADER_LENGTH0 4

int HEADER_LENGTH[3] = {NODE_HEADER_LEN_FRAME0,
                        NODE_HEADER_LEN_FRAME1,
                        NODE_HEADER_LEN_FRAME2};

UWBHelperNode::UWBHelperNode(std::string serial_name,
                             int baudrate,
                             bool enable_debug_output) {
    this->enable_debug_output = enable_debug_output;
    if (open_port(serial_name, baudrate)) {
        printf("Open port %s[%d] successful!\n\n", serial_name.c_str(), baudrate);
        uwb_ok = true;
    } else {
        printf("Can't open serial port; Use WiFi Only\n");
        uwb_ok = false;
    }
}

bool UWBHelperNode::open_port(std::string serial_name, int baudrate) {
    printf("Trying to open port %s\n", serial_name.c_str());
    serial_fd = open(serial_name.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (serial_fd == -1) {
        perror("open_port: Unable to open port ");
        return false;
    } else
        fcntl(serial_fd, F_SETFL, 0);

    configure_port(baudrate);

    return true;
}

void UWBHelperNode::configure_port(int baudrate) {
    struct termios tty;

    if (tcgetattr(serial_fd, &tty) < 0) {
        printf("Error from tcgetattr");
        exit(-1);
        return;
    }

#if defined(__MACH__)
    speed_t spd = baudrate;
#else
    speed_t spd = B921600;
    switch (baudrate) {
    case 3000000:
        spd = B3000000;
        break;
    case 1000000:
        spd = B1000000;
        break;
    case 230400:
        spd = B230400;
        break;
    case 460800:
        spd = B460800;
        break;
    case 921600:
        spd = B921600;
        break;
    case 115200:
    default:
        spd = B115200;
        break;
    }
#endif
    cfsetospeed(&tty, spd);
    cfsetispeed(&tty, spd);

    tty.c_cflag |= (CLOCAL | CREAD); /* ignore modem controls */
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;      /* 8-bit characters */
    tty.c_cflag &= ~PARENB;  /* no parity bit */
    tty.c_cflag &= ~CSTOPB;  /* only need 1 stop bit */
    tty.c_cflag &= ~CRTSCTS; /* no hardware flowcontrol */

    /* setup for non-canonical mode */
    tty.c_iflag &=
        ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    tty.c_oflag &= ~OPOST;

    /* fetch bytes as they become available */
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 1;

    if (tcsetattr(serial_fd, TCSANOW, &tty) != 0) {
        printf("Error from tcsetattr");
        exit(-1);
        return;
    }
    printf("Successful set port\n");
    return;
}

void UWBHelperNode::delete_first_n_buf(int _len) {
    buf.erase(buf.begin(), buf.begin() + _len);
}

void UWBHelperNode::read_and_parse() {
    while (serial_available_bytes() > 0) {
        uint8_t c = read_byte_from_serial();
        // printf("%c",c);
        buf.push_back(c);
        int ret = this->parse_data();
        if (ret == NODE_FRAME2) {
            this->on_node_data_updated();
        }
    }
}

int UWBHelperNode::parse_data() {
    if (this->read_status == WAIT_FOR_HEADER) {
        this->parse_header();
    }

    if (this->read_status == WAIT_FOR_NODE_DETAIL) {
        bool ret = false;
        if (this->read_wait_remote_node_num > 0) {
            if (recv_type_now == NODE_FRAME0) {
                ret = parse_remote_node_details_frame0();
            } else if(recv_type_now == NODE_FRAME2) {
                ret = parse_remote_node_details_frame2();
            }

            if (ret) {
                this->read_wait_remote_node_num--;
            }
        } else {
            this->read_status = WAIT_FOR_NODE_CHECKSUM;
        }
    }

    if (this->read_status == WAIT_FOR_NODE_CHECKSUM) {
        if (buf.size() > 0) {
            uint8_t checksum = buf[0];
            this->delete_first_n_buf(1);
            this->read_status = WAIT_FOR_HEADER;
            return this->recv_type_now;
        }
    }
    return -1;
}

void UWBHelperNode::send_broadcast_data(std::vector<uint8_t> msg) {
    this->serial_write((uint8_t *)msg.data(), msg.size());
}

void UWBHelperNode::on_node_data_updated() {
    for (auto it : nodes_info) {
        int _id = it.first;
        if (active_node_set.find(_id) != active_node_set.end())
            nodes_info[_id].active = true;
        else
            nodes_info[_id].active = false;
    }

    if (this->enable_debug_output) {
        printf("\rSYSTIME %d NODES %ld", this->sys_time, this->nodes_info.size());
        for (auto it : nodes_info) {
            std::string str = it.second.to_str();
            printf("%s;;", str.c_str());
        }
    }

    vaild_node_quantity = active_node_set.size();
    this->active_node_set.clear();
}

int UWBHelperNode::serial_available_bytes() {
    int bytes = 0;
    ioctl(serial_fd, FIONREAD, &bytes);
    return bytes;
}

uint8_t
UWBHelperNode::read_byte_from_serial() {
    char c;
    int n = read(serial_fd, &c, sizeof(char));
    // printf("%2x ", c);
    return c;
}

#pragma pack(push, 1)
struct NodeHeaderFrame0 {
    char head[2];
    uint16_t frame_length;
    uint8_t role_table;
    uint8_t id;
    char reserved[4];
    uint8_t recv_node_nums;
};
#pragma pack(pop)

#pragma pack(push, 1)
struct NodeHeaderFrame2 {
    char head[2];
    uint16_t frame_length;
    uint8_t role_table;
    uint8_t id;
    uint32_t lps_time;
    char reserved[108];
    uint8_t vaild_node_quantity;
};
#pragma pack(pop)

int UWBHelperNode::parse_node_header_frame2() {
    NodeHeaderFrame2 nh;
    memcpy(&nh, buf.data(), sizeof(nh));
    // printf("id %d t %d n %d\n",
    // nh.id,
    // nh.lps_time,
    // nh.remote_num
    // );
    this->sys_time = nh.lps_time;
    this->self_id = nh.id;
    this->vaild_node_quantity = nh.vaild_node_quantity;
    this->delete_first_n_buf(NODE_HEADER_LEN_FRAME2);
    return this->vaild_node_quantity;
}

int UWBHelperNode::parse_node_header_frame0() {
    // printf("Parsing header 0");
    NodeHeaderFrame0 nh;
    memcpy(&nh, buf.data(), sizeof(nh));
    // printf("id %d t %d n %d\n",
    // nh.id,
    // nh.lps_time,
    // nh.remote_num
    // );
    this->self_id = nh.id;
    // printf("ROLD %d", nh.role_table);
    this->delete_first_n_buf(NODE_HEADER_LEN_FRAME0);
    return nh.recv_node_nums;
}

bool UWBHelperNode::is_node_msg_header() {
    return (buf[0] == 0x55 && buf[1] == 0x02) ||
           (buf[0] == 0x55 && buf[1] == 0x04) ||
           (buf[0] == 0x55 && buf[1] == 0x03);
}

uint8_t
UWBHelperNode::frame_type() {
    if (buf[0] == 0x55 && buf[1] == 0x02) {
        return NODE_FRAME0;
    }

    if (buf[0] == 0x55 && buf[1] == 0x03) {
        // Detected header 1
        // printf("Detected header1");
        return NODE_FRAME1;
    }

    if (buf[0] == 0x55 && buf[1] == 0x04) {
        return NODE_FRAME2;
    }

    return 255;
}

void UWBHelperNode::parse_header() {
    // printf("parsing header...Buf size %d\n", buf.size());
    while (buf.size() > 2 && !is_node_msg_header()) {
        delete_first_n_buf(1);
    }

    recv_type_now = frame_type();
    // printf("frame type %d\n", recv_type_now);

    if (recv_type_now == 255) {
        // delete_first_n_buf(1);
        return;
    }

    size_t recv_nums = HEADER_LENGTH[recv_type_now];

    if (buf.size() < recv_nums) {
        return;
    }

    int node_num = 0;
    switch (recv_type_now) {
    case NODE_FRAME0:
        node_num = this->parse_node_header_frame0();

        if (node_num > 100 || node_num < 0) {
            printf("Error Node num; throw");
            return;
        }
        // printf("Msg recv, node num %d\n", node_num);

        break;
    case NODE_FRAME1:
        fprintf(stderr, "FRAME1 detected! Exit");
        exit(-1);
        break;
    case NODE_FRAME2:
        node_num = this->parse_node_header_frame2();
        if (sys_time > 0) {
            on_system_time_update();
        }
        if (node_num > 100 || node_num < 0) {
            printf("Error Node num; throw");
            return;
        }
        // printf("Node Frame Number %d\n", node_num);
        break;
    default:
        break;
    }

    // printf("Found node header\n");

    if (node_num > 10) {
        fprintf(stderr, "INVAILD node num %d, set to zero", node_num);
        node_num = 0;
    }

    this->read_wait_remote_node_num = node_num;
    this->read_status = WAIT_FOR_NODE_DETAIL;
}

bool UWBHelperNode::parse_remote_node_details_frame2() {
    size_t header_length = sizeof(RemoteNodeFrame2);

    if (buf.size() < header_length)
        return false;
    RemoteNodeFrame2 nh;
    memcpy(&nh, buf.data(), sizeof(nh));

    // No data msg recved
    delete_first_n_buf(header_length);
    this->on_node_data_recv(nh);
    return true;
}

bool UWBHelperNode::parse_remote_node_details_frame0() {
    size_t header_length = sizeof(RemoteNodeHeaderFrame0);

    if (buf.size() < header_length)
        return false;
    RemoteNodeHeaderFrame0 nh;
    memcpy(&nh, buf.data(), sizeof(nh));

    if (nh.data_length == 0) {
        // printf("Zero datalength %d", nh.id);
        return true;
    } else {
        // printf("D %d", nh.data_length);
        if (buf.size() <  REMOTE_HEADER_LENGTH0 + nh.data_length) {
            return false;
        } else {
            Buffer msg(nh.data_length);
            std::copy(buf.begin() + REMOTE_HEADER_LENGTH0,
                      buf.begin() + REMOTE_HEADER_LENGTH0 + nh.data_length,
                      msg.begin());

            delete_first_n_buf(REMOTE_HEADER_LENGTH0 + nh.data_length);
            this->on_broadcast_data_recv(nh.id, msg);
            return true;
        }
    }
    return false;
}

void UWBHelperNode::on_broadcast_data_recv(int _id,
                                           std::vector<uint8_t> _msg) {
    char *str = (char *)_msg.data();
    if (this->enable_debug_output) {
        printf("RECV DATA @ %d on systime %d, msg\n%s\n",
               _id,
               this->sys_time,
               str);
    }
}

void UWBHelperNode::serial_write(uint8_t *data, int len) {
    write(serial_fd, data, len);
}

void UWBHelperNode::on_node_data_recv(RemoteNodeFrame2 nf) {
    this->active_node_set.insert(nf.id);
    int _id = nf.id;
    // printf("Insert to active node set%d\n", _id);
    if (nodes_info.find(_id) == nodes_info.end()) {
        RemoteNodeInfo info;
        nodes_info[_id] = info;
        nodes_info[_id].id = _id;
    }

    nodes_info[_id].distance = ((double)nf.distance) / 1000;
    nodes_info[_id].fp_rssi = (double)nf.fp_rssi / -2.0;
    nodes_info[_id].active = true;
    nodes_info[_id].dis_time = nf.rx_lps_systime;
    // printf("ID %d RX %ld\n", _id, nf.rx_lps_systime);
    nodes_info[_id].rx_rssi = nf.rx_rssi / -2.0;
    nodes_info[_id].role = nf.role;
}
