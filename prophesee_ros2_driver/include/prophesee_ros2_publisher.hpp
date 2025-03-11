/*******************************************************************
 * File : prophesee_ros_publisher.hpp                             *
 *                                                                 *
 * Copyright: (c) 2015-2019 Prophesee                              *
 *******************************************************************/

#ifndef PROPHESEE_ROS_PUBLISHER_H_
#define PROPHESEE_ROS_PUBLISHER_H_

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/camera_info.hpp>
#include <metavision/sdk/stream/camera.h>
#include <vector>
#include <string>

// ※ カスタムメッセージのインクルード（ROS2版）
// ※ 実際のパスは環境に合わせて調整してください
#include "prophesee_event_msgs/msg/event.hpp"
#include "prophesee_event_msgs/msg/event_array.hpp"

/// \brief ROS2向けのメインパブリッシャークラス
///
/// PropheseeセンサからのデータをROSトピックへパブリッシュする
class PropheseeWrapperPublisher : public rclcpp::Node {
public:
    /// \brief コンストラクタ
    PropheseeWrapperPublisher();

    /// \brief デストラクタ
    virtual ~PropheseeWrapperPublisher();

    /// \brief カメラを起動し、データのパブリッシュを開始する
    void startPublishing();

private:
    /// \brief カメラのオープン処理
    bool openCamera();

    /// \brief CDイベントのパブリッシュ処理（コールバック内で実行）
    void publishCDEvents();

    /// \brief 定期的にカメラ情報をパブリッシュするタイマーコールバック
    void publishCameraInfo();

    /// \brief カメラ情報パブリッシャー
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_info_;

    /// \brief CDイベントパブリッシャー
    rclcpp::Publisher<prophesee_event_msgs::msg::EventArray>::SharedPtr pub_cd_events_;

    /// \brief カメラクラスのインスタンス
    Metavision::Camera camera_;

    /// \brief CDイベントの蓄積用バッファ
    std::vector<Metavision::EventCD> event_buffer_;

    /// \brief カメラ情報のメッセージ
    sensor_msgs::msg::CameraInfo cam_info_msg_;

    /// \brief カメラ設定ファイル（バイアス）のパス
    std::string biases_file_;

    /// \brief ライブカメラの代わりに読み込むrawファイルのパス
    std::string raw_file_to_read_;

    /// \brief カメラ名（文字列）
    std::string camera_name_;

    /// \brief タイムスタンプ（カメラ情報パブリッシュ用）
    rclcpp::Time start_timestamp_, last_timestamp_;

    /// \brief CDイベントのパブリッシュ有無フラグ
    bool publish_cd_;

    /// \brief アクティビティフィルタの時間深度（マイクロ秒単位）
    int activity_filter_temporal_depth_;

    /// \brief CDイベントパッケージングの時間間隔
    rclcpp::Duration event_delta_t_;

    /// \brief イベントバッファの開始および現在のタイムスタンプ
    rclcpp::Time event_buffer_start_time_, event_buffer_current_time_;

    /// \brief カメラ情報パブリッシュ用のタイマー
    rclcpp::TimerBase::SharedPtr camera_info_timer_;

    /// \brief 重力加速度（地球表面、[m/s^2]）
    static constexpr double GRAVITY = 9.81;

    /// \brief ドライバで固定されたCDイベントのデルタタイム（64マイクロ秒）
    static constexpr double EVENT_DEFAULT_DELTA_T = 6.4e-05;
};

#endif /* PROPHESEE_ROS_PUBLISHER_H_ */
