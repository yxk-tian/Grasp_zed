#include "mydetector.hpp"

// void PanTiltDetector::YoloInit()
// {
//         string classes_file = "/home/yxk/桌面/workspace/yolo/obj.names";
//         string model_config = "/home/yxk/桌面/workspace/yolo/yolo-obj.cfg";
//         string model_weights = "/home/yxk/桌面/workspace/yolo/yolo-obj_last.weights";

//         ifstream ifs(classes_file.c_str());
//         string line;
//         while (getline(ifs, line))
//                 classes_.push_back(line); //
//         detector = std::make_unique<Detector>(model_config, model_weights, 0);
//         std::cout << "Yolo init success!" << std::endl;
// }

void PanTiltDetector::CameraInit()
{
        InitParameters init_params; // set camera params
        init_params.camera_resolution = RESOLUTION::HD720;
        init_params.depth_mode = DEPTH_MODE::PERFORMANCE;
        init_params.coordinate_units = UNIT::METER;
        init_params.camera_fps = 60;

        zed_ptr = std::make_unique<Camera>();
        if (zed_ptr->open(init_params) != ERROR_CODE::SUCCESS) // open camera
        {
                printf("Fail to init camera!!!");
                zed_ptr->close();
                quit_flag_ = true;
        }
        else
                quit_flag_ = false;
        lcong48;

        runtime_parameters = std::make_unique<RuntimeParameters>();
        runtime_parameters->sensing_mode = SENSING_MODE::STANDARD;
        Resolution image_size = zed_ptr->getCameraInformation().camera_configuration.resolution;
        image_width_ = image_size.width;
        image_height_ = image_size.height;
        std::cout << "Camera init success!" << std::endl;
}

void PanTiltDetector::FrameProduce()
{
        // auto start_time = chrono::high_resolution_clock::now();
        while (!quit_flag_)
        {
                if (zed_ptr->grab(*runtime_parameters) != ERROR_CODE::SUCCESS)
                        continue;
                // get zed image in format U8_C$

                sl::Mat image_zed(image_width_, image_height_, MAT_TYPE::U8_C4);
                zed_ptr->retrieveImage(image_zed, VIEW::LEFT, MEM::CPU,
                                       Resolution(image_width_, image_height_));

                // sl::Mat depth_map; // get zed depth image
                // zed_ptr->retrieveMeasure(depth_map, MEASURE::DEPTH, MEM::CPU,
                //                          Resolution(image_width_, image_height_));

                sl::SensorsData sensor_data;
                zed_ptr->getSensorsData(sensor_data, TIME_REFERENCE::IMAGE);

                cv::Mat image_ocv = this->slMat2cvMat(image_zed); // trans to opencv format

                this->bgra2bgr(image_ocv); // convert the bgra format to bgr

                detect_.rgb_image = image_ocv;
                // detect_.depth_map = depth_map; //TODO这有问题

                detect_.sensor_data = sensor_data;
                imwrite("/home/yxk/workspace/yolo_darknet-master/data/test.jpg", detect_.rgb_image);
                // zed_ptr->close();

                // namedWindow("test", WINDOW_AUTOSIZE);
                // imshow("test", detect_.rgb_image);
                // waitKey();
                // float depth2;
                // detect_.depth_map.getValue(640, 360, &depth2); //TODO得到深度值，这错了
                // cout << depth2 << endl;
                // double time_stamp = (static_cast<chrono::duration<double,
                //                                                   std::milli>>(chrono::high_resolution_clock::now() - start_time))
                //                         .count();

                // buffer_.Push(Frame{image_ocv, depth_map, sensor_data, time_stamp});
                // std::cout << "Camera is running!" << std::endl;
                return;
        }
}

void PanTiltDetector::DetectRealize()
{
        // this->YoloInit();
        struct timeval tv1, tv2;
        long long T;
        string classesFile = "/home/yxk/桌面/workspace/yolo/obj.names";
        string modelConfig = "/home/yxk/桌面/workspace/yolo/yolo-obj.cfg";
        string modelWeights = "/home/yxk/桌面/workspace/yolo/yolo-obj_last.weights";

        //加载类别名
        vector<string> classes;
        ifstream ifs(classesFile.c_str());
        string line;
        while (getline(ifs, line))
                classes.push_back(line);
        //加载网络模型，0是指定第一块GPU
        Detector detector(modelConfig, modelWeights, 0);
        // cv::Mat frame = imread("/home/yxk/workspace/yolo_darknet-master/data/test.jpg");
        // this->CameraInit();
        // this->FrameProduce();

        cv::Mat frame = detect_.rgb_image;
        // Frame frame;
        // std::vector<bbox_t> detect_outs;

        while (!quit_flag_)
        {
                if (frame.empty())
                        break;
                // 可以尝试在这输出图像看看是不是真的得到了
                // namedWindow("test", WINDOW_AUTOSIZE);
                // imshow("test", frame.rgb_image);
                // waitKey();

                //开始计时
                gettimeofday(&tv1, NULL);
                shared_ptr<image_t> detImg = detector.mat_to_image_resize(frame);
                //前向预测
                vector<bbox_t> detect_outs = detector.detect_resized(*detImg, frame.cols, frame.rows, 0.25);

                // latest_detect_.first = detect_; //定义第一个数是frame
                // shared_ptr<image_t> detImg = detector->mat_to_image_resize(frame);
                // detect_outs = detector->detect_resized(*detImg, frame.cols, frame.rows, 0.25);
                // detect_outs = detector->detect(frame.rgb_image, 0.2); //TODO 找到这个detect 得到第二个数，也就是包络框
                // latest_detect_.second = detect_outs;

                this->Drawer(frame, detect_outs, classes);
                //结束计时
                gettimeofday(&tv2, NULL);
                //计算用时
                T = (tv2.tv_sec - tv1.tv_sec) * 1000 + (tv2.tv_usec - tv1.tv_usec) / 1000;
                std::cout << T << "ms" << endl;
                imwrite("/home/yxk/workspace/yolo_darknet-master/data/result.jpg", frame);

                // cv::namedWindow("Grasp_detect", CV_WINDOW_AUTOSIZE);
                // if (frame.empty())
                //         break;

                this->VisualHandler(detect_, detect_outs);
                // cv::imshow("Grasp_detect", detect_.rgb_image);

                break; //
        }
}

void PanTiltDetector::Drawer(cv::Mat &frame, vector<bbox_t> outs, vector<string> classes)
{
        //获取所有最佳检测框信息
        for (int i = 0; i < outs.size(); i++)
        {
                this->DrawBoxes(frame, classes, outs[i].obj_id, outs[i].prob, outs[i].x, outs[i].y,
                                outs[i].x + outs[i].w, outs[i].y + outs[i].h);
        }
}

void PanTiltDetector::DrawBoxes(cv::Mat &frame, vector<string> classes, int classId, float conf, int left, int top, int right, int bottom)
{
        //画检测框
        rectangle(frame, Point(left, top), Point(right, bottom), Scalar(255, 178, 50), 3);

        //该检测框对应的类别和置信度
        string label = format("%.2f", conf);
        if (!classes.empty())
        {
                CV_Assert(classId < (int)classes.size());
                label = classes[classId] + ":" + label;
        }

        //将标签显示在检测框顶部
        int baseLine;
        Size labelSize = getTextSize(label, FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
        top = max(top, labelSize.height);
        rectangle(frame, Point(left, top - round(1.5 * labelSize.height)), Point(left + round(1.5 * labelSize.width), top + baseLine), Scalar(255, 255, 255), FILLED);
        putText(frame, label, Point(left, top), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(0, 0, 0), 1);
}

void PanTiltDetector::VisualHandler(Frame &frame, const vector<bbox_t> &outs)
{
        sl::float3 eular_angles = frame.sensor_data.imu.pose.getEulerAngles();
        Eigen::Vector2d gimbal_pose(eular_angles[1] * 180.0 / M_PI,
                                    eular_angles[0] * 180.0 / M_PI);

        cv::Point center(640, 360), nearest_target(640, 360); //给了一个中心的坐标和最近坐标
        float depth_value = 0, radius = 0, min_dis = 999;
        //
        sl::Mat depth_map; // get zed depth image
        zed_ptr->retrieveMeasure(depth_map, MEASURE::DEPTH, MEM::CPU,
                                 Resolution(image_width_, image_height_));
        zed_ptr->close();

        for (auto it = outs.begin(); it != outs.end(); it++)
        {
                if (it->obj_id == 0) // detect person
                {
                        center.x = it->x + it->w / 2;
                        center.y = it->y + it->h / 2;
                        depth_map.getValue(center.x, center.y, &depth_value); //TODO得到深度值，这错了
                        // cout << center.x << " " << center.y << endl;
                        // std::cout << "深度值:" << depth_value << endl;
                        if (depth_value < min_dis) // find nearest target
                        {
                                nearest_target = cv::Point(center.x, center.y);
                                min_dis = depth_value;
                        }
                        cv::rectangle(frame.rgb_image, cv::Point(it->x, it->y),
                                      cv::Point(it->x + it->w, it->y + it->h),
                                      switch_process_flag_ ? Scalar(255, 178, 50) : Scalar(0, 0, 255), 2); // box all detected target
                }
        }

        if (!target_tube_.size()) // slide window init
        {
                target_tube_.push_back(nearest_target);
                target_tube_.push_back(nearest_target);
                Eigen::Vector3d yaw_reset(gimbal_pose(0), 0, 0);
                Eigen::Vector3d pitch_reset(0, 0, 0);
                //         return;
        }

        // remove detection failure
        // if (min_dis == 999)
        //         nearest_target = target_tube_.back();
        if (min_dis == 999)
        {
                std::cout << "识别不成功,再来一次！" << endl;
                return;
        }

        Eigen::Vector2d target_angle = this->pixel2angle(nearest_target);

        double yaw = target_angle(0) + gimbal_pose(0);
        double pitch = target_angle(1) + gimbal_pose(1);

        last_yaw_ = yaw;
        last_pitch_ = pitch;
        depth_ = min_dis;
        std::cout << "yaw:" << last_yaw_ << " "
                  << "pitch:" << last_pitch_ << endl;
        std::cout << "depth:" << depth_ << endl;
}

Eigen::Vector2d PanTiltDetector::pixel2angle(const cv::Point &pixel)
{
        return Eigen::Vector2d(atan2(pixel.x - 640, 702.9439) * 180 / M_PI,
                               atan2(360 - pixel.y, 703.2455) * 180 / M_PI); //返回两个角度值
}

cv::Point PanTiltDetector::angle2pixel(const Eigen::Vector2d &angle)
{
        return cv::Point((int)(tan(angle(0) * M_PI / 180) * 702.9439) + 640,
                         360 - (int)(tan(angle(1) * M_PI / 180) * 703.2455));
}

cv::Mat PanTiltDetector::slMat2cvMat(const sl::Mat &input)
{
        int cv_type = -1;
        switch (input.getDataType())
        {
        case MAT_TYPE::F32_C1:
                cv_type = CV_32FC1;
                break;
        case MAT_TYPE::F32_C2:
                cv_type = CV_32FC2;
                break;
        case MAT_TYPE::F32_C3:
                cv_type = CV_32FC3;
                break;
        case MAT_TYPE::F32_C4:
                cv_type = CV_32FC4;
                break;

        case MAT_TYPE::U8_C1:
                cv_type = CV_8UC1;
                break;
        case MAT_TYPE::U8_C2:
                cv_type = CV_8UC2;
                break;
        case MAT_TYPE::U8_C3:
                cv_type = CV_8UC3;
                break;
        case MAT_TYPE::U8_C4:
                cv_type = CV_8UC4;
                break;
        default:
                break;
        }

        return cv::Mat(input.getHeight(), input.getWidth(), cv_type,
                       input.getPtr<sl::uchar1>(MEM::CPU));
}

void PanTiltDetector::bgra2bgr(cv::Mat &image)
{
        std::vector<cv::Mat> bgra_channels;
        cv::split(image, bgra_channels); // 4 - channels
        std::vector<cv::Mat> bgr_channels;
        bgr_channels.push_back(bgra_channels[0]);
        bgr_channels.push_back(bgra_channels[1]);
        bgr_channels.push_back(bgra_channels[2]);
        cv::merge(bgr_channels, image);
}
void PanTiltDetector::getangle(double &yaw, double &pitch, double &depth)
{
        yaw = last_yaw_;
        pitch = last_pitch_;
        depth = depth_;
}