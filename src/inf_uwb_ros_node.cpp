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

using namespace inf_uwb_ros;

#define MAX_SEND_BYTES 40

class UWBRosNodeofNode : public UWBHelperNode {
    ros::Timer fast_timer, slow_timer;
    std::mutex send_lock;
public:
    UWBRosNodeofNode(std::string serial_name, int baudrate, ros::NodeHandle nh, bool enable_debug) : UWBHelperNode(serial_name, baudrate, false) {
        remote_node_pub = nh.advertise<remote_uwb_info>("remote_nodes", 1);
        broadcast_data_pub = nh.advertise<incoming_broadcast_data>("incoming_broadcast_data", 1);

        recv_bdmsg = nh.subscribe("send_broadcast_data", 1, &UWBRosNodeofNode::on_send_broadcast_req, this, ros::TransportHints().tcpNoDelay());
        fast_timer = nh.createTimer(ros::Duration(0.001), &UWBRosNodeofNode::fast_timer_callback, this);
        slow_timer = nh.createTimer(ros::Duration(0.01), &UWBRosNodeofNode::send_broadcast_data_callback, this);
        time_reference_pub = nh.advertise<sensor_msgs::TimeReference>("time_ref", 1);
    }

protected:
    // void
    void send_broadcast_data_callback(const ros::TimerEvent &e) {
        send_lock.lock();
        // ROS_INFO("Send buffer %ld", send_buffer.size());
        if (send_buffer.size() <= MAX_SEND_BYTES) {
            if (send_buffer.size() > 0) {
                this->send_broadcast_data(send_buffer);
                send_buffer.clear();
            }
        } else {
            std::vector<uint8_t> sub(&send_buffer[0], &send_buffer[MAX_SEND_BYTES]);
            this->send_broadcast_data(sub);
            send_buffer.erase(send_buffer.begin(), send_buffer.begin() + MAX_SEND_BYTES);
        }
        send_lock.unlock();
    }

    void fast_timer_callback(const ros::TimerEvent &e) {
        this->read_and_parse();
    }
    virtual void on_send_broadcast_req(data_buffer msg) {
        // this->send_broadcast_data(msg.data);
        // return;
        // ROS_INFO("msg size %d", msg.data.size());
        send_lock.lock();
        send_buffer.insert(send_buffer.end(), msg.data.begin(), msg.data.end());
        send_lock.unlock();
    }
    
    virtual void on_broadcast_data_recv(int _id, Buffer _msg) override {
        UWBHelperNode::on_broadcast_data_recv(_id, _msg);
        // printf("Recv broadcast data %s", (char*)_msg.data());

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

int main(int argc, char **argv) {
    ROS_INFO("INF UWB ROS\nIniting\n");

    ros::init(argc, argv, "uwb_node");

    ros::NodeHandle nh("uwb_node");
    int baudrate = 921600;
    std::string serial_name;
    nh.param<int>("baudrate", baudrate, 921600);
    nh.param<std::string>("serial_name", serial_name, "/dev/ttyUSB0");

    UWBRosNodeofNode uwbhelper(serial_name, baudrate, nh, true);

    ros::MultiThreadedSpinner spinner(2);
    spinner.spin();
}