#include "uwb_helper.h"
#include <inf_uwb_ros/data_buffer.h>
#include <inf_uwb_ros/incoming_broadcast_data.h>
#include <inf_uwb_ros/remote_uwb_info.h>
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/TimeReference.h>
#include <std_msgs/Header.h>
#include <std_msgs/String.h>
#include <unistd.h>
#include <mutex>
#include <lcm/lcm-cpp.hpp>
#include <inf_uwb_ros/SwarmData_t.hpp>
#include <thread>
#include <unordered_set>
#include <signal.h>

using namespace inf_uwb_ros;

#define MAX_SEND_BYTES 40

class UWBRosNodeofNode : public UWBHelperNode {
    ros::Timer fast_timer, slow_timer;
    std::mutex send_lock;

    double send_freq = 100;
    int send_buffer_size = 80;

    lcm::LCM lcm;

    bool lcm_ok = false;

    std::unordered_set<int32_t> sent_msgs;

public:
    UWBRosNodeofNode(std::string serial_name, std::string lcm_uri, int baudrate, ros::NodeHandle nh, bool enable_debug): 
        UWBHelperNode(serial_name, baudrate, false),
        lcm(lcm_uri) {
        nh.param<int>("send_buffer", send_buffer_size, 80);
        nh.param<int>("self_id", self_id, -1);
        nh.param<double>("send_freq", send_freq, 50);
        double recv_freq = 100;
        nh.param<double>("recv_freq", recv_freq, 100);

        remote_node_pub = nh.advertise<remote_uwb_info>("remote_nodes", 1);
        broadcast_data_pub = nh.advertise<incoming_broadcast_data>("incoming_broadcast_data", 1);

        recv_bdmsg = nh.subscribe("send_broadcast_data", 1, &UWBRosNodeofNode::on_send_broadcast_req, this, ros::TransportHints().tcpNoDelay());
        fast_timer = nh.createTimer(ros::Duration(1/recv_freq), &UWBRosNodeofNode::fast_timer_callback, this);
        slow_timer = nh.createTimer(ros::Duration(1/send_freq), &UWBRosNodeofNode::send_broadcast_data_callback, this);
        time_reference_pub = nh.advertise<sensor_msgs::TimeReference>("time_ref", 1);


        if (!lcm.good()) {
            ROS_ERROR("LCM %s failed", lcm_uri.c_str());
            // exit(-1);
        } else {
            ROS_INFO("LCM OK");
            lcm_ok = true;
        }
        lcm.subscribe("SWARM_DATA", &UWBRosNodeofNode::on_swarm_data_lcm, this);
    }


    int lcm_handle() {
        return lcm.handle();
    }


protected:
    void on_swarm_data_lcm(const lcm::ReceiveBuffer* rbuf,
                const std::string& chan, 
                const SwarmData_t* msg) {
        // on_broadcast_data_recv(msg->sender_id, msg->mavlink_msg);
        // ROS_INFO("Recv remote %d", msg->msg_id);
        if (sent_msgs.find(msg->msg_id) == sent_msgs.end()) {
            ROS_INFO("On remote lcm data");
            ros::Time stamp(msg->sec, msg->nsec);
            incoming_broadcast_data data;
            data.header.stamp = stamp;
            data.lps_time = sys_time;
            data.remote_id = msg->sender_id;
            data.data = msg->mavlink_msg;
            broadcast_data_pub.publish(data);
        } else {
            sent_msgs.erase(msg->msg_id);
        }
    }
    // void
    void send_broadcast_data_callback(const ros::TimerEvent &e) {
        if (uwb_ok) {
            static int c = 0;
            send_lock.lock();
            ROS_INFO_THROTTLE(1.0, "Send buffer %ld", send_buffer.size());
            if (send_buffer.size() > 2 * send_buffer_size) {
                ROS_WARN("Send buffer size %ld to big!", send_buffer.size());
            }
            if (send_buffer.size() <= send_buffer_size) {
                if (send_buffer.size() > 0) {
                    // if (c++ % 2 ==)
                    this->send_broadcast_data(send_buffer);
                    send_buffer.clear();
                }
            } else {
                std::vector<uint8_t> sub(&send_buffer[0], &send_buffer[send_buffer_size]);
                this->send_broadcast_data(sub);
                send_buffer.erase(send_buffer.begin(), send_buffer.begin() + send_buffer_size);
            }
            send_lock.unlock();
        }
    }

    void fast_timer_callback(const ros::TimerEvent &e) {
        this->read_and_parse();
    }

    virtual void on_send_broadcast_req(data_buffer msg) {
        if (lcm_ok && (msg.send_method == 1 || msg.send_method == 2)) {
            send_by_lcm(msg.data, msg.header.stamp);
        }

        if (msg.send_method == 0 || msg.send_method == 2) {
            //Insert to UWB send buffer
            send_lock.lock();
            send_buffer.insert(send_buffer.end(), msg.data.begin(), msg.data.end());
            send_lock.unlock();
        }
    }

    virtual void send_by_lcm(std::vector<uint8_t> buf, ros::Time stamp) {
        // ROS_INFO("Sending data %ld with LCM self_id %d", buf.size(), self_id);
        SwarmData_t data;
        data.mavlink_msg_len = buf.size();
        data.mavlink_msg = buf;
        
        data.sec = stamp.sec;
        data.nsec = stamp.nsec;
        data.sender_id = self_id;

        data.msg_id = rand() + data.nsec;

        sent_msgs.insert(data.msg_id);
        lcm.publish("SWARM_DATA", &data);
    }
    
    virtual void on_broadcast_data_recv(int _id, Buffer _msg) override {
        UWBHelperNode::on_broadcast_data_recv(_id, _msg);
        // printf("ID %d Recv broadcast data %s", _id, (char*)_msg.data());
        // printf("ID %d Recv broadcast data len %d\n", _id, _msg.size());

        incoming_broadcast_data data;
        data.header.stamp = ros::Time::now();
        data.lps_time = sys_time;
        data.remote_id = _id;
        data.data = _msg;
        broadcast_data_pub.publish(data);
    }

    virtual void on_system_time_update() override {
        sensor_msgs::TimeReference tr;
        tr.header.stamp = ros::Time::now();
        tr.source = "LPS";
        tr.time_ref = ros::Time(0) + ros::Duration(sys_time/1000.0);

        time_reference_pub.publish(tr);
    }

    virtual void on_node_data_updated() override {
        UWBHelperNode::on_node_data_updated();

        static int count = 0;
        remote_uwb_info info;
        info.sys_time = this->sys_time;
        info.remote_node_num = this->nodes_info.size();
        info.self_id = this->self_id;
        info.header.stamp = ros::Time::now();
        for (auto k : this->nodes_info) {
            int _id = k.first;
            RemoteNodeInfo nod = k.second;
            info.node_ids.push_back(_id);
            info.node_dis.push_back(nod.distance);
            info.recv_distance_time.push_back(nod.dis_time);
            info.active.push_back(nod.active);
            info.fp_rssi.push_back(nod.fp_rssi);
            info.rx_rssi.push_back(nod.rx_rssi);
        }
        remote_node_pub.publish(info);
        if (count++ % 50 == 1) {
            ROS_INFO("[c%d,ts %d] ID %d nodes total %d active %d\n", count, sys_time, self_id, info.remote_node_num, vaild_node_quantity);
            fflush(stdout);
        }
    }

private:
    ros::Publisher remote_node_pub, broadcast_data_pub, time_reference_pub;
    ros::Subscriber recv_bdmsg;

    std::vector<uint8_t> send_buffer;

};

UWBRosNodeofNode * uwbhelper;

void shutdown_handler(int sig) {
    printf("Shutting down uwb .....\n");
    uwbhelper->close_port();
    exit(0);
}

int main(int argc, char **argv) {
    ROS_INFO("INF UWB ROS\nIniting\n");

    ros::init(argc, argv, "uwb_node");

    ros::NodeHandle nh("uwb_node");
    int baudrate = 921600;
    std::string serial_name;

    nh.param<int>("baudrate", baudrate, 921600);
    nh.param<std::string>("serial_name", serial_name, "/dev/ttyUSB0");

    uwbhelper = new UWBRosNodeofNode(serial_name, "udpm://224.0.0.251:7667?ttl=1", baudrate, nh, true);
    signal(SIGINT, shutdown_handler);
    signal(SIGTERM, shutdown_handler);
    signal(SIGKILL, shutdown_handler);
    signal(SIGQUIT, shutdown_handler);
    std::thread thread([&] {
        while(0 == uwbhelper->lcm_handle()) {
        }
    });

    ros::MultiThreadedSpinner spinner(2);

    spinner.spin();
}