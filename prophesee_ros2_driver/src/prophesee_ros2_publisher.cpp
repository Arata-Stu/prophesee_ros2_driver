/*******************************************************************
 * File : prophesee_ros2_publisher.cpp                             *
 *                                                                 *
 * Copyright: (c) 2015-2019 Prophesee                              *
 *******************************************************************/

#include "prophesee_ros2_publisher.hpp"
#include <mutex>
#include <thread>
#include <chrono>
#include <functional>

// ROS2用のロガー、タイマー、その他の機能はrclcppで提供
// カスタムメッセージのインクルードも済んでいる前提

PropheseeWrapperPublisher::PropheseeWrapperPublisher()
    : Node("prophesee_ros2_publisher"),
    event_delta_t_(rclcpp::Duration::from_seconds(0))
    {

    // パラメータの宣言とデフォルト値の設定
    this->declare_parameter<std::string>("camera_name", "PropheseeCamera_optical_frame");
    this->declare_parameter<bool>("publish_cd", true);
    this->declare_parameter<std::string>("bias_file", "");
    this->declare_parameter<std::string>("raw_file_to_read", "");
    this->declare_parameter<double>("event_delta_t", 100.0e-6);

    // パラメータの取得
    this->get_parameter("camera_name", camera_name_);
    this->get_parameter("publish_cd", publish_cd_);
    this->get_parameter("bias_file", biases_file_);
    this->get_parameter("raw_file_to_read", raw_file_to_read_);
    double event_delta_t_param;
    this->get_parameter("event_delta_t", event_delta_t_param);
    event_delta_t_ = rclcpp::Duration::from_seconds(event_delta_t_param);

    // トピック名の設定
    const std::string topic_cam_info = "/prophesee/" + camera_name_ + "/camera_info";
    const std::string topic_cd_event_buffer = "/prophesee/" + camera_name_ + "/cd_events_buffer";

    // パブリッシャーの作成
    pub_info_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(topic_cam_info, 1);
    if (publish_cd_) {
        pub_cd_events_ = this->create_publisher<prophesee_event_msgs::msg::EventArray>(topic_cd_event_buffer, 500);
    }

    // カメラがオープンできるまでループ（ROS2のrclcpp::ok()で状態を確認）
    while (!openCamera() && rclcpp::ok()) {
        RCLCPP_INFO(this->get_logger(), "Trying to open camera...");
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    // カメラのランタイムエラーコールバックの設定
    camera_.add_runtime_error_callback([this](const Metavision::CameraException &e) {
        RCLCPP_WARN(this->get_logger(), "%s", e.what());
    });

    // センサの設定取得
    Metavision::CameraConfiguration config = camera_.get_camera_configuration();
    auto &geometry = camera_.geometry();
    RCLCPP_INFO(this->get_logger(), "[CONF] Width:%i, Height:%i", geometry.width(), geometry.height());
    RCLCPP_INFO(this->get_logger(), "[CONF] Serial number: %s", config.serial_number.c_str());

    // カメラ情報メッセージの設定
    cam_info_msg_.width = geometry.width();
    cam_info_msg_.height = geometry.height();
    cam_info_msg_.header.frame_id = camera_name_;

    // カメラ情報を定期的にパブリッシュするタイマーを作成（5Hz相当）
    camera_info_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(200),
        std::bind(&PropheseeWrapperPublisher::publishCameraInfo, this)
    );
}

PropheseeWrapperPublisher::~PropheseeWrapperPublisher() {
    camera_.stop();
}

bool PropheseeWrapperPublisher::openCamera() {
    bool camera_is_opened = false;

    try {
        if (raw_file_to_read_.empty()) {
            camera_ = Metavision::Camera::from_first_available();
            if (!biases_file_.empty()) {
                RCLCPP_INFO(this->get_logger(), "[CONF] Loading bias file: %s", biases_file_.c_str());
                camera_.biases().set_from_file(biases_file_);
            }
        } else {
            camera_ = Metavision::Camera::from_file(raw_file_to_read_);
            RCLCPP_INFO(this->get_logger(), "[CONF] Reading from raw file: %s", raw_file_to_read_.c_str());
        }
        camera_is_opened = true;
    } catch (Metavision::CameraException &e) {
        RCLCPP_WARN(this->get_logger(), "%s", e.what());
    }
    return camera_is_opened;
}

void PropheseeWrapperPublisher::startPublishing() {
    camera_.start();
    start_timestamp_ = this->now();
    last_timestamp_ = start_timestamp_;

    if (publish_cd_) {
        // CDイベントのパブリッシュ処理を別スレッドで開始
        std::thread cd_thread(&PropheseeWrapperPublisher::publishCDEvents, this);
        cd_thread.detach();
    }
    RCLCPP_INFO(this->get_logger(), "Started publishing.");
    // ノードのスピンはmain()で実施
}

void PropheseeWrapperPublisher::publishCameraInfo() {
    // サブスクライバが存在する場合にカメラ情報をパブリッシュ
    if (pub_info_->get_subscription_count() > 0) {
        cam_info_msg_.header.stamp = this->now();
        pub_info_->publish(cam_info_msg_);
    }
}

void PropheseeWrapperPublisher::publishCDEvents() {
    try {
        Metavision::CallbackId cd_callback = camera_.cd().add_callback(
            [this](const Metavision::EventCD *ev_begin, const Metavision::EventCD *ev_end) {
                if (pub_cd_events_->get_subscription_count() <= 0)
                    return;

                if (ev_begin < ev_end) {
                    // イベントバッファの現在時刻を計算（開始時刻に相対する形で計算）
                    event_buffer_current_time_ = start_timestamp_ +
                        rclcpp::Duration::from_nanoseconds(static_cast<int64_t>(ev_begin->t * 1000.0));

                    if (event_buffer_.empty()) {
                        event_buffer_start_time_ = event_buffer_current_time_;
                    }
                    // イベントをバッファに挿入
                    event_buffer_.insert(event_buffer_.end(), ev_begin, ev_end);

                    event_buffer_current_time_ = start_timestamp_ +
                        rclcpp::Duration::from_nanoseconds(static_cast<int64_t>((ev_end - 1)->t * 1000.0));
                }

                if ((event_buffer_current_time_ - event_buffer_start_time_) >= event_delta_t_) {
                    prophesee_event_msgs::msg::EventArray event_buffer_msg;
                    event_buffer_msg.header.stamp = event_buffer_current_time_;
                    auto &geometry = camera_.geometry();
                    event_buffer_msg.height = geometry.height();
                    event_buffer_msg.width  = geometry.width();
                    event_buffer_msg.events.resize(event_buffer_.size());

                    // バッファ内の各イベントをROS2メッセージ形式に変換
                    for (size_t i = 0; i < event_buffer_.size(); ++i) {
                        const Metavision::EventCD &src_event = event_buffer_[i];
                        prophesee_event_msgs::msg::Event &dst_event = event_buffer_msg.events[i];
                        dst_event.x = src_event.x;
                        dst_event.y = src_event.y;
                        dst_event.polarity = src_event.p;
                        dst_event.ts = start_timestamp_ +
                            rclcpp::Duration::from_nanoseconds(static_cast<int64_t>(src_event.t * 1000.0));
                    }
                    pub_cd_events_->publish(event_buffer_msg);
                    event_buffer_.clear();
                    RCLCPP_DEBUG(this->get_logger(),
                                 "CD data available, buffer size: %d at time: %u",
                                 static_cast<int>(event_buffer_msg.events.size()),
                                 event_buffer_msg.header.stamp.nanosec);
                }
            }
        );
    } catch (Metavision::CameraException &e) {
        RCLCPP_WARN(this->get_logger(), "%s", e.what());
        publish_cd_ = false;
    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PropheseeWrapperPublisher>();
    node->startPublishing();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
