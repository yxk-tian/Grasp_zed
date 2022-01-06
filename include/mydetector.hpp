#include <iostream>
#include <memory>
#include <cmath>
#include <sl/Camera.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <string>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigen>
#include <vector>
#include <fstream>
#include <sys/time.h>

#define OPENCV
#include "yolo_v2_class.hpp"

#define HOG 1
#define FIXEDWINDOW 1
#define MULTISCALE 1
#define LAB 1

using namespace sl;
using namespace cv;
using namespace std;

struct Frame
{
        cv::Mat rgb_image;           // simple rgb image
        sl::Mat depth_map;           // zed depth map for distance detect这是zed的深度地图
        sl::SensorsData sensor_data; // zed sensor_data e.g. IMU
        double time_stamp;           // time in ms, from init to now 时间
};

// class FrameBuffer //定义了一个环形队列的类
// {
// public:
//         FrameBuffer(size_t size) : frames_(size), mutexs_(size),
//                                    front_idx_(0), rear_idx_(0), last_get_timestamp_(0.0) {}
//         ~FrameBuffer() = default;

//         bool Push(const Frame &frame)
//         {
//                 // 1, 2, ..., size - 1, 0
//                 const size_t new_front_idx = (front_idx_ + 1) % frames_.size();
//                 unique_lock<timed_mutex> lock(mutexs_[new_front_idx], chrono::milliseconds(2));
//                 if (!lock.owns_lock()) // try for 2ms to lock, try not to be blocked
//                 {
//                         return false;
//                 }

//                 frames_[new_front_idx] = frame;
//                 // cover the oldest one in rear
//                 if (new_front_idx == rear_idx_)
//                         rear_idx_ = (rear_idx_ + 1) % frames_.size();
//                 front_idx_ = new_front_idx;
//                 return true;
//         }

//         bool GetLatest(Frame &frame)
//         {
//                 volatile const size_t front_idx = front_idx_; // meaning of volatile?
//                 //try for 2ms to lock
//                 // unique_lock<timed_mutex> lock(mutexs_[front_idx], chrono::milliseconds(2));
//                 // if (!lock.owns_lock() || frames_[front_idx].time_stamp == last_get_timestamp_ || frames_[front_idx].rgb_image.empty()) // !.empty() avoid blinking
//                 // {
//                 //         return false;
//                 // }

//                 frame = frames_[front_idx];
//                 last_get_timestamp_ = frames_[front_idx].time_stamp; // get too fast!
//                 return true;
//         }

// private:
//         std::vector<Frame> frames_;
//         std::vector<std::timed_mutex> mutexs_;

//         size_t front_idx_;
//         size_t rear_idx_;
//         double last_get_timestamp_;
// };

class PanTiltDetector
{
public:
        explicit PanTiltDetector() // : buffer_(5) //初始化buffer_为5
        {
                // this->YoloInit();
                this->CameraInit();

                // udp_send = std::make_unique<udpSendHandler>(30);
                // serial_ptr_ = std::make_unique<SerialHandler>(20); //串口发送长度为20
                // kalman_ptr_ = std::make_unique<KalmanFilter>(4, 4, 0);
                // kalman_ptr_->transitionMatrix = (cv::Mat_<float>(4, 4) << // 转移矩阵该如何定义
                //                                      1,
                //                                  0, 0.0167, 0,
                //                                  0, 1, 0, 0.0167,
                //                                  0, 0, 1, 0,
                //                                  0, 0, 0, 1);

                // setIdentity(kalman_ptr_->measurementMatrix);                 // H
                // setIdentity(kalman_ptr_->processNoiseCov, Scalar::all(1));   // Q
                // kalman_ptr_->measurementNoiseCov = (cv::Mat_<float>(4, 4) << // R
                //                                         2,
                //                                     0, 0, 0,
                //                                     0, 2, 0, 0,
                //                                     0, 0, 10, 0,
                //                                     0, 0, 0, 10);
                // setIdentity(kalman_ptr_->errorCovPost, Scalar::all(1)); // P
                // kalman_ptr_->statePost = (cv::Mat_<float>(4, 1) <<      // x_hat init
                //                               0,
                //                           0, 0, 0);

                // Vector2d kp(10, 10);
                // Vector2d ki(0.0, 0.0);
                // Vector2d kd(0.2, 0.2);
                // Vector2d max_abs_error(160, 90);
                // Vector2d u_max(100, 100);
                // Vector2d u_min(-100, -100);

                // pid = std::make_unique<PID>(kp, ki, kd, PID::BOTH_BOUND, 0.0167); // 60hz
                // pid->setThreshold(u_max, u_min, max_abs_error);

                // count_ = 0;

                // pitch_kf_ = std::make_unique<robot_detection::KalmanFilter>();
                // yaw_kf_ = std::make_unique<robot_detection::KalmanFilter>();
                // pitch_kf_->Init(0.0167, 1, 0.5, 0.0002);
                // yaw_kf_->Init(0.0167, 1, 0.5, 0.0002);

                // roi_ = cv::Rect(0, 0, 0, 0);
        }

        virtual ~PanTiltDetector()
        {
                zed_ptr->close();
                cv::destroyAllWindows();
        }

        /* thread interface */
        void FrameProduce();
        void DetectRealize();
        void getangle(double &yaw, double &pitch, double &depth);
        // void ServoControl();
        // void Visualization();
        // void FeedbackHandle();
        /* *** */

private:
        void Drawer(cv::Mat &frame, vector<bbox_t> outs, vector<string> classes);
        void DrawBoxes(cv::Mat &frame, vector<string> classes, int classId, float conf, int left, int top, int right, int bottom);
        // void YoloInit();
        void CameraInit();
        void VisualHandler(Frame &frame, const vector<bbox_t> &outs);
        void DrawCross(cv::Mat &frame, cv::Point point, cv::Scalar color,
                       int size, int thickness);
        cv::Mat slMat2cvMat(const sl::Mat &input); // convert stereolab space to opencv space
        void bgra2bgr(cv::Mat &image);
        // void TrackFilter(const cv::Point &last, cv::Point &current,
        //                  const cv::Point &bound);
        Eigen::Vector2d pixel2angle(const cv::Point &pixel);
        cv::Point angle2pixel(const Eigen::Vector2d &angle);

        std::unique_ptr<Camera> zed_ptr;
        std::unique_ptr<RuntimeParameters> runtime_parameters;
        // std::unique_ptr<KalmanFilter> kalman_ptr_;
        // std::unique_ptr<udpSendHandler> udp_send;//TODO udp该咋加
        // std::unique_ptr<SerialHandler> serial_ptr_;
        // std::unique_ptr<PID> pid;
        // std::unique_ptr<Detector> detector; // yolo based detector
        std::vector<string> classes_;
        std::pair<Frame, std::vector<bbox_t>> latest_detect_; // for servo control
        std::mutex image_mutex_;                              // lock for servo control
        std::mutex plot_mutex_;                               // lock for visualization
        // FrameBuffer buffer_;                                  // circular buffer

        std::vector<cv::Point> target_tube_; // a slide window for filter

        cv::Mat image_plot_;
        // cv::Mat image_detect_;
        Frame detect_;
        // std::unique_ptr<Frame> detect_;
        int image_width_;
        int image_height_;
        int count_; // for debug only

        bool quit_flag_;

        // std::unique_ptr<robot_detection::KalmanFilter> pitch_kf_;
        // std::unique_ptr<robot_detection::KalmanFilter> yaw_kf_;
        double last_yaw_, last_pitch_;
        float depth_;
        bool converge_flag_ = false;
        bool switch_process_flag_ = true;
        bool switch_flag_ = false;

        // cv::Rect roi_;
        // std::unique_ptr<KCFTracker> tracker_ptr_;
        // TrackingUtility track_utility_;
};