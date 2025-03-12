/*******************************************************************
* File : prophesee_ros2_viewer.hpp                                 *
*                                                                 *
* Copyright: (c) 2015-2019 Prophesee                              *
*******************************************************************/

#ifndef PROPHESEE_ROS2_VIEWER_H_
#define PROPHESEE_ROS2_VIEWER_H_

#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>

#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>

#include <prophesee_event_msgs/msg/event.hpp>
#include <prophesee_event_msgs/msg/event_array.hpp>

#include "cd_frame_generator.hpp"

/// \brief Main class ROS listener and viewer (ROS2版)
///
/// カメラ情報とCDイベントのトピックを購読し、画面上に可視化する
class PropheseeWrapperViewer : public rclcpp::Node {
public:
    /// \brief コンストラクタ
    PropheseeWrapperViewer();

    /// \brief デストラクタ
    virtual ~PropheseeWrapperViewer();

    /// \brief 現在のCDデータを表示する
    void showData();

    /// \brief フレームジェネレータが初期化済みかどうかを返す
    bool isInitialized();

private:
    /// \brief カメラ情報トピック受信時のコールバック
    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);

    /// \brief センサの幅と高さでフレームジェネレータ等を初期化する
    bool init(const unsigned int &sensor_width, const unsigned int &sensor_height);

    /// \brief ディスプレイ用ウィンドウを作成する
    void create_window(const std::string &window_name, const unsigned int &sensor_width,
                    const unsigned int &sensor_height, const int &shift_x = 0, const int &shift_y = 0);

    /// \brief カメラ情報購読用サブスクリプション
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_cam_info_;

    /// \brief CDイベント購読用サブスクリプション
    rclcpp::Subscription<prophesee_event_msgs::msg::EventArray>::SharedPtr sub_cd_events_;

    /// \brief CDイベントからフレームを生成するクラス
    CDFrameGenerator cd_frame_generator_;

    /// \brief CDイベント表示用ウィンドウ名
    std::string cd_window_name_;

    /// \brief 表示に使用するイベント蓄積時間（us）
    int display_acc_time_;

    /// \brief フレームジェネレータが初期化済みか
    bool initialized_;

    /// \brief CDイベントを可視化するかどうか
    bool show_cd_;
};

#endif /* PROPHESEE_ROS2_VIEWER_H_ */
