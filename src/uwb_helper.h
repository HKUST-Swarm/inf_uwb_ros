#pragma once

#include <cstdint>
#include <map>
#include <set>
#include <stdio.h>
#include <string>
#include <vector>
#include <unistd.h>

#define MAX_DRONE_NUM 10

#pragma pack(push, 1)
struct RemoteNodeFrame2 {
    uint8_t role;
    uint8_t id;
    int32_t distance : 24;
    uint8_t fp_rssi;
    uint8_t rx_rssi;
    uint32_t rx_lps_systime;
    char reserved[2];
};
#pragma pack(pop)

#pragma pack(push, 1)
struct RemoteNodeHeaderFrame0 {
    uint8_t role;
    uint8_t id;
    uint16_t data_length;
};
#pragma pack(pop)

typedef std::vector<uint8_t> Buffer;
struct RemoteNodeInfo {
    double distance = -1;
    uint32_t dis_time;
    double fp_rssi = 0;
    double rx_rssi = 0;
    bool active = false;
    int id = 0;
    int role = 0;
    std::string to_str() {
        char str[100] = {0};
        sprintf(str, " id:%d role %d dis:%f fprssi:%f rxrssi:%f active:%d", id,
                role, distance, fp_rssi, rx_rssi, active);
        std::string _str(str);
        return _str;
    }
};

class UWBHelperNode {
public:
    enum {
        WAIT_FOR_HEADER,
        WAIT_FOR_NODE_DETAIL,
        WAIT_FOR_NODE_CHECKSUM,
    };

    enum {
        NODE_FRAME0,
        NODE_FRAME1,
        NODE_FRAME2
    };

    UWBHelperNode(std::string serial_name, int baudrate,
                  bool enable_debug_output = false);
    std::vector<uint8_t> buf;
    int self_id;
    int sys_time = -1;
    int vaild_node_quantity = 0;

    uint8_t recv_type_now = -1;

    int64_t sum_check = 0;

    void read_and_parse();

    void send_broadcast_data(std::vector<uint8_t> msg);

    bool uwb_ok = false;

    void close_port() {
        printf("Closing port\n");
        close(serial_fd);
    }

protected:
    void reset_checksum();
    void delete_first_n_buf(int _len);
    int parse_data();

    bool enable_debug_output;

    virtual void on_broadcast_data_recv(int _id, Buffer _msg);
    void on_node_data_recv(RemoteNodeFrame2 nf);

    virtual void on_node_data_updated();
    std::map<int, RemoteNodeInfo> nodes_info;

    uint8_t frame_type();

private:
    int serial_fd;
    void configure_port(int baudrate);
    bool open_port(std::string serial_name, int baudrate);



    uint8_t read_byte_from_serial();
    int serial_available_bytes();

    int read_status = WAIT_FOR_HEADER;
    int read_wait_remote_node_num = 0;

    std::set<int> active_node_set;

    int parse_node_header_frame2();
    int parse_node_header_frame0();

    bool is_node_msg_header();

    void parse_header();

    virtual void on_system_time_update() {};

    bool parse_remote_node_details_frame2();
    bool parse_remote_node_details_frame0();

    void serial_write(uint8_t *data, int len);
};