/*******************************************************************
 * File : prophesee_ros_viewer.cpp                                 *
 *                                                                 *
 * Copyright: (c) 2015-2019 Prophesee                              *
 *******************************************************************/

 #include "prophesee_ros2_viewer.hpp"
 #include <opencv2/highgui.hpp>
 #include <chrono>
 #include <thread>
 #include <functional>
 
 PropheseeWrapperViewer::PropheseeWrapperViewer()
 : Node("prophesee_ros_viewer"),
   cd_window_name_("CD Events"),
   display_acc_time_(5000),
   initialized_(false),
   show_cd_(true)
 {
     std::string camera_name("");
     // パラメータの宣言と取得
     this->declare_parameter<std::string>("camera_name", "");
     this->declare_parameter<bool>("show_cd", true);
     this->declare_parameter<int>("display_accumulation_time", 5000);
 
     this->get_parameter("camera_name", camera_name);
     this->get_parameter("show_cd", show_cd_);
     this->get_parameter("display_accumulation_time", display_acc_time_);
 
     // トピック名の生成
     const std::string topic_cam_info = "/prophesee/" + camera_name + "/camera_info";
     const std::string topic_cd_event_buffer = "/prophesee/" + camera_name + "/cd_events_buffer";
 
     // カメラ情報トピックの購読
     sub_cam_info_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
         topic_cam_info,
         1,
         std::bind(&PropheseeWrapperViewer::cameraInfoCallback, this, std::placeholders::_1)
     );
 
     // CDイベントトピックの購読（有効な場合）
     if (show_cd_) {
         sub_cd_events_ = this->create_subscription<prophesee_event_msgs::msg::EventArray>(
             topic_cd_event_buffer,
             500,
             [this](const prophesee_event_msgs::msg::EventArray::SharedPtr msg) {
                 cd_frame_generator_.add_events(msg);
             }
         );
     }
 }
 
 PropheseeWrapperViewer::~PropheseeWrapperViewer() {
     if (!initialized_)
         return;
 
     // CDフレームジェネレータのスレッド停止
     if (show_cd_)
         cd_frame_generator_.stop();
 
     // ウィンドウを破棄
     cv::destroyAllWindows();
 }
 
 bool PropheseeWrapperViewer::isInitialized() {
     return initialized_;
 }
 
 void PropheseeWrapperViewer::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
     if (initialized_)
         return;
 
     if (msg->width != 0 && msg->height != 0)
         init(msg->width, msg->height);
 }
 
 bool PropheseeWrapperViewer::init(const unsigned int &sensor_width, const unsigned int &sensor_height) {
     if (show_cd_) {
         // CDイベント表示用ウィンドウの作成
         create_window(cd_window_name_, sensor_width, sensor_height, 0, 0);
         // CDフレームジェネレータの初期化
         cd_frame_generator_.init(sensor_width, sensor_height);
         cd_frame_generator_.set_display_accumulation_time_us(display_acc_time_);
         // スレッド開始
         cd_frame_generator_.start();
     }
 
     initialized_ = true;
     return true;
 }
 
 void PropheseeWrapperViewer::create_window(const std::string &window_name, const unsigned int &sensor_width,
                                             const unsigned int &sensor_height, const int &shift_x, const int &shift_y) {
     // OpenCV4以降の場合、ウィンドウフラグは cv::WINDOW_AUTOSIZE などを利用
     cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE);
     cv::resizeWindow(window_name, sensor_width, sensor_height);
     cv::moveWindow(window_name, shift_x, shift_y);
 }
 
 void PropheseeWrapperViewer::showData() {
     if (!show_cd_)
         return;
 
     // 現在時刻はNodeのnow()メソッドで取得（ROS2のタイムスタンプ）
     rclcpp::Time now = this->now();
     // もし直近のイベント受信時刻が0.5秒以上前ならリセットする
     if (cd_frame_generator_.get_last_ros_timestamp() < now - rclcpp::Duration(0, 500000000)) {
         cd_frame_generator_.reset();
         initialized_ = false;
     }
 
     const cv::Mat &cd_frame = cd_frame_generator_.get_current_frame();
     if (!cd_frame.empty()) {
         cv::imshow(cd_window_name_, cd_frame);
     }
 }
 
 // UI操作用の関数（waitKey後に追加待機）
 int process_ui_for(const int &delay_ms) {
     auto then = std::chrono::high_resolution_clock::now();
     int key = cv::waitKey(delay_ms);
     auto now = std::chrono::high_resolution_clock::now();
     std::this_thread::sleep_for(std::chrono::milliseconds(
         delay_ms - std::chrono::duration_cast<std::chrono::milliseconds>(now - then).count()
     ));
     return key;
 }
 
 int main(int argc, char **argv) {
     rclcpp::init(argc, argv);
     auto viewer_node = std::make_shared<PropheseeWrapperViewer>();
 
     // カメラ情報コールバックによる初期化を待つ
     while (rclcpp::ok() && !viewer_node->isInitialized()) {
         rclcpp::spin_some(viewer_node);
         std::this_thread::sleep_for(std::chrono::milliseconds(10));
     }
 
     // メインループ：イベント処理と表示
     while (rclcpp::ok()) {
         rclcpp::spin_some(viewer_node);
         viewer_node->showData();
         process_ui_for(33);
     }
 
     rclcpp::shutdown();
     return 0;
 }
 