#include "uwb_helper.h"
#include <swarmcomm_msgs/data_buffer.h>
#include <swarmcomm_msgs/incoming_broadcast_data.h>
#include <swarmcomm_msgs/remote_uwb_info.h>
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/TimeReference.h>
#include <std_msgs/Header.h>
#include <std_msgs/String.h>
#include <unistd.h>
#include <mutex>
#include <lcm/lcm-cpp.hpp>
#include <swarmcomm_msgs/SwarmData_t.hpp>
#include <thread>
#include <unordered_set>
#include <signal.h>
#include <bspline/Bspline.h>
#include <swarmcomm_msgs/Bspline_t.hpp>
#include <queue>

using namespace swarmcomm_msgs;

#define BACKWARD_HAS_DW 1
#include <backward.hpp>
namespace backward
{
    backward::SignalHandling sh;
}

#define MAX_SEND_BYTES 40


class UWBRosNodeofNode : public UWBHelperNode {
    ros::Timer fast_timer, slow_timer;
    std::mutex send_lock;

    double send_freq = 100;
    int send_buffer_size = 80;

    lcm::LCM lcm;

    bool lcm_ok = false;

    ros::Publisher swarm_traj_pub;
    ros::Subscriber swarm_traj_sub;

    int latency_buffer_size;
    bool sim_latency;

public:
    UWBRosNodeofNode(std::string serial_name, std::string lcm_uri, int baudrate, ros::NodeHandle nh, bool enable_debug): 
        UWBHelperNode(serial_name, baudrate, false),
        lcm(lcm_uri) {
        nh.param<int>("send_buffer", send_buffer_size, 80);
        nh.param<int>("self_id", self_id, -1);
        nh.param<double>("send_freq", send_freq, 50);
        double recv_freq = 100;
        nh.param<double>("recv_freq", recv_freq, 100);

        nh.param<int>("latency_buffer_size", latency_buffer_size, 100);
        nh.param<bool>("sim_latency", sim_latency, false);

        remote_node_pub = nh.advertise<remote_uwb_info>("remote_nodes", 1);
        broadcast_data_pub = nh.advertise<incoming_broadcast_data>("incoming_broadcast_data", 1);
        swarm_traj_pub = nh.advertise<bspline::Bspline>("/planning/swarm_traj_recv", 10);

        swarm_traj_sub = nh.subscribe("/planning/swarm_traj_send", 10, &UWBRosNodeofNode::broadcast_bspline, this, ros::TransportHints().tcpNoDelay());
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
        lcm.subscribe("SWARM_TRAJ", &UWBRosNodeofNode::incoming_bspline_data_callback, this);
    }


    int lcm_handle() {
        return lcm.handle();
    }


protected:

    int count_remote = 0;
    void on_swarm_data_lcm(const lcm::ReceiveBuffer* rbuf,
                const std::string& chan, 
                const SwarmData_t* msg) {
        // on_broadcast_data_recv(msg->sender_id, msg->mavlink_msg);
	auto _msg_id = msg->msg_id;
	//ROS_INFO("Recv remote %d", _msg_id);
        count_remote ++;
        if (msg->sender_id != self_id) {
            // ROS_INFO_THROTTLE(1.0, "On remote lcm data");
            ros::Time stamp(msg->sec, msg->nsec);
            incoming_broadcast_data data;
            data.header.stamp = stamp;
            data.lps_time = sys_time;
            data.remote_id = msg->sender_id;
            data.data = msg->mavlink_msg;
            broadcast_data_pub.publish(data);
        }
    }
    // void
    void send_broadcast_data_callback(const ros::TimerEvent &e) {
        if (uwb_ok) {
            static int c = 0;
            send_lock.lock();
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


    void incoming_bspline_data_callback(const lcm::ReceiveBuffer* rbuf,
        const std::string& chan, 
        const Bspline_t* msg) {
        if(msg->drone_id != self_id) 
        {
            bspline::Bspline bspl;
            bspl.start_time = ros::Time(msg->start_time_sec, msg->start_time_nsec);
            bspl.drone_id = msg->drone_id;
            bspl.order = msg->order;
            bspl.traj_id = msg->traj_id;

            for (int i = 0; i < msg->knots_num; i++) {
                bspl.knots.push_back(msg->knots[i]);
            }

            for (int i = 0; i < msg->pos_pts_num; i++) {
                geometry_msgs::Point pt;
                pt.x = msg->pos_pts_x[i];
                pt.y = msg->pos_pts_y[i];
                pt.z = msg->pos_pts_z[i];
                bspl.pos_pts.push_back(pt);
            }

            for (int i = 0; i < msg->yaw_pts_num; i++) {
                bspl.yaw_pts.push_back(msg->yaw_pts[i]);
            }

            bspl.yaw_dt = msg->yaw_dt;
            swarm_traj_pub.publish(bspl);
        }

    }

    void broadcast_bspline(const bspline::Bspline & bspl) {
        Bspline_t _bspl;
        _bspl.start_time_sec = bspl.start_time.sec;
        _bspl.start_time_nsec = bspl.start_time.nsec;
        _bspl.drone_id = bspl.drone_id;
        _bspl.order = bspl.order;
        _bspl.traj_id = bspl.traj_id;
        _bspl.knots_num = bspl.knots.size();

        for (size_t i = 0; i < bspl.knots.size(); i ++) {
            _bspl.knots.push_back(bspl.knots[i]);
        }

        _bspl.pos_pts_num = bspl.pos_pts.size();
        for (size_t i = 0; i < bspl.pos_pts.size(); i ++) {
            _bspl.pos_pts_x.push_back(bspl.pos_pts[i].x);
            _bspl.pos_pts_y.push_back(bspl.pos_pts[i].y);
            _bspl.pos_pts_z.push_back(bspl.pos_pts[i].z);
        }

        _bspl.yaw_pts_num = bspl.yaw_pts.size();

        for (size_t i = 0; i < bspl.yaw_pts.size(); i ++) {
            _bspl.yaw_pts.push_back(bspl.yaw_pts[i]);
        }

        _bspl.yaw_dt = bspl.yaw_dt;

        _bspl.msg_id = rand() + _bspl.start_time_nsec + _bspl.traj_id;
        // sent_msgs.insert(_bspl.msg_id);
        // ROS_INFO("BSPLINE SIZE %ld", _bspl.getEncodedSize());

        lcm.publish("SWARM_TRAJ", &_bspl);

    }

    std::queue<std::vector<uint8_t>> buf_queue;
    virtual void send_by_lcm(std::vector<uint8_t> buf, ros::Time stamp) {
        // ROS_INFO("Sending data %ld with LCM self_id %d", buf.size(), self_id);
        SwarmData_t data;
        if (!sim_latency) {
            data.mavlink_msg_len = buf.size();
            data.mavlink_msg = buf;
        } else {
            buf_queue.push(buf);
            if(buf_queue.size() > latency_buffer_size) {
                auto _buf = buf_queue.front();
                data.mavlink_msg = _buf;
                data.mavlink_msg_len = _buf.size();
                buf_queue.pop();
            } else {
                return;
            }
        }
        
        data.sec = stamp.sec;
        data.nsec = stamp.nsec;
        data.sender_id = self_id;

        data.msg_id = rand() + data.nsec;

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
            if (_id < MAX_DRONE_NUM) {
                RemoteNodeInfo nod = k.second;
                info.node_ids.push_back(_id);
                info.node_dis.push_back(nod.distance);
                info.recv_distance_time.push_back(nod.dis_time);
                info.active.push_back(nod.active);
                info.fp_rssi.push_back(nod.fp_rssi);
                info.rx_rssi.push_back(nod.rx_rssi);
            }
        }
        remote_node_pub.publish(info);
        if (count++ % 50 == 1) {
            ROS_INFO("[c%d,ts %d] ID %d nodes total %d active %d send_buf %ld lcm_msg %d/s\n", count, sys_time, self_id, info.remote_node_num, vaild_node_quantity, send_buffer.size(), count_remote*2);
            count_remote = 0;
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
    // signal(SIGINT, shutdown_handler);
    // signal(SIGTERM, shutdown_handler);
    // signal(SIGKILL, shutdown_handler);
    // signal(SIGQUIT, shutdown_handler);
    std::thread thread([&] {
        while(0 == uwbhelper->lcm_handle()) {
        }
    });

    ros::MultiThreadedSpinner spinner(2);
    spinner.spin();
    // ros::spin();
}
