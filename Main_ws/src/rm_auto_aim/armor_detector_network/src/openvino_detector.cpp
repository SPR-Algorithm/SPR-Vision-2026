// Copyright 2025 ZhangSuHang

#include <algorithm>
#include "openvino_armor_detector/openvino_detector.hpp"

namespace fyt::auto_aim
{
static const int INPUT_W =640;
static const int INPUT_H =640;

cv::Mat OpenVINODetector::letterbox(const cv::Mat & img )
{
  std::vector<int> new_shape = {INPUT_W, INPUT_H};
  // Get current image shape [height, width]

  int img_h = img.rows;
  int img_w = img.cols;

  // Compute scale ratio(new / old) and target resized shape
  float scale = std::min(new_shape[1] * 1.0 / img_h, new_shape[0] * 1.0 / img_w);
  int resize_h = static_cast<int>(round(img_h * scale));
  int resize_w = static_cast<int>(round(img_w * scale));

  // Compute padding
  int pad_h = new_shape[1] - resize_h;
  int pad_w = new_shape[0] - resize_w;

  // Resize and pad image while meeting stride-multiple constraints
  cv::Mat resized_img;
  cv::resize(img, resized_img, cv::Size(resize_w, resize_h));

  // divide padding into 2 sides
  float half_h = pad_h * 1.0 / 2;
  float half_w = pad_w * 1.0 / 2;

  // Compute padding boarder
  int top = static_cast<int>(round(half_h - 0.1));
  int bottom = static_cast<int>(round(half_h + 0.1));
  int left = static_cast<int>(round(half_w - 0.1));
  int right = static_cast<int>(round(half_w + 0.1));

  // Add border
  cv::copyMakeBorder(
    resized_img, resized_img, top, bottom, left, right, cv::BORDER_CONSTANT, cv::Scalar(
      114, 114,
      114));

  return resized_img;
}

void OpenVINODetector::set_callback(DetectorCallback callback)
{
  infer_callback_ = callback;
}

std::future<bool> OpenVINODetector::push_input(const cv::Mat & resize_img, int detect_color_,int64_t timestamp_nanosec)//创建推理请求函数，回调为infer_callback
{
  if (resize_img.empty()) {
    // return false when img is empty
    return std::async([]() {return false;});
  }

  // Start async detect
//   return std::async(
//     std::launch::async, &OpenVINODetector::infer_callback, this, resize_img, detect_color_,
//     timestamp_nanosec);

  return std::async(
    std::launch::async, 
    [this, img = resize_img.clone(), detect_color_, timestamp_nanosec]() { // 使用 clone() 深拷贝
        return this->infer_callback(img, detect_color_, timestamp_nanosec);
    }
    );
}


OpenVINODetector::OpenVINODetector(string model_path_xml, string model_path_bin, string device){//构造函数

    input_shape = {1, static_cast<unsigned long>(IMAGE_HEIGHT), static_cast<unsigned long>(IMAGE_WIDTH), 3};
    model = core.read_model(model_path_xml, model_path_bin);
    // Step . Inizialize Preprocessing for the model
    ppp = new ov::preprocess::PrePostProcessor(model);
    // Specify input image format
    ppp->input().tensor().set_element_type(ov::element::u8).set_layout("NHWC").set_color_format(ov::preprocess::ColorFormat::BGR); 
    //NHWC:batchsize,height,width,channels
    // Specify preprocess pipeline to input image without resizing
    ppp->input().preprocess().convert_element_type(ov::element::f32).convert_color(ov::preprocess::ColorFormat::RGB).scale({255., 255., 255.});
    //  Specify model's input layout
    ppp->input().model().set_layout("NCHW");
    // Specify output results format
    ppp->output().tensor().set_element_type(ov::element::f32);
    // Embed above steps in the graph
    model = ppp->build();
    compiled_model = core.compile_model(model, device);
}



bool OpenVINODetector::infer_callback(Mat img, int detect_color,int64_t timestamp_nanosec){//推理回调函数
    std::lock_guard<std::mutex> lock(mutex_);
    objects.clear();
    tmp_objects.clear();
    ious.clear();
    // Step 3. Read input image
    // resize image

    // Step 5. Create tensor from image
    //int rows = img.rows;
    //int cols = img.cols;

    //uchar* input_data = (uchar *)img.data; // 创建一个新的float数组、
    cv::Mat img_copy = img.clone();
    uchar* input_data = img_copy.data;
    ov::Tensor input_tensor = ov::Tensor(compiled_model.input().get_element_type(),
     compiled_model.input().get_shape(), input_data);
     
    // Step 6. Create an infer request for model inference
    ov::InferRequest infer_request = compiled_model.create_infer_request();
    infer_request.set_input_tensor(input_tensor);
    //double ta = cv::getTickCount();
    infer_request.infer();
    //double tb = cv::getTickCount();
    //std::cout <<"timeab: "<< (tb - ta) / cv::getTickFrequency() * 1000 << " "<<std::endl;

    //Step 7. Retrieve inference results

    // Step 8. get result
    // -------- Step 8. Post-process the inference result -----------

    auto output = infer_request.get_output_tensor(0);
    ov::Shape output_shape = output.get_shape();
    //std::cout << "The shape of output tensor:"<<output_shape << std::endl;
    // 25200 x 85 Matrix
    cv::Mat output_buffer(output_shape[1], output_shape[2], CV_32F, output.data());
    float conf_threshold = 0.45 ;
    float nms_threshold = 0.45;
    std::vector<cv::Rect> boxes;
    std::vector<int> class_ids;
    std::vector<float> class_scores;
    std::vector<float> confidences;
    // cx,cy,w,h,confidence,c1,c2,...c80

    //std::cout<<output_buffer.cols<<endl;
    for (int i = 0; i < output_buffer.rows; i++) {
        //通过置信度阈值筛选
        float confidence = output_buffer.at<float>(i, 8);
        confidence = sigmoid(confidence);
        if (confidence < conf_threshold)
        {
            continue;
        }
        //颜色和类别向量
        cv::Mat color_scores = output_buffer.row(i).colRange(9, 13);  //color
        cv::Mat classes_scores = output_buffer.row(i).colRange(13, 22); //num
        cv::Point class_id,color_id;
        int _class_id,_color_id;
        double score_color, score_num;
        cv::minMaxLoc(classes_scores, NULL, &score_num, NULL, &class_id);
        cv::minMaxLoc(color_scores, NULL, &score_color, NULL, &color_id);
        // cout<<score_color<<" "<<color_id.x<<endl;
        // cout<<score_num<<" "<<class_id.x<<endl;
        // class score: 0~3
        //cout<<"class_id.x:"<<class_id.x<<endl;
        //cout<<"detect_color:"<<detect_color<<endl;
        
        // None 或者Purple 丢掉
        if(color_id.x == 2 || color_id.x == 3)
        {
            continue;
        }
        else if(detect_color == 0 && color_id.x == 1)   // detect blue
        {
            continue;
        }
        else if(detect_color == 1 && color_id.x == 0)   // detect red
        {
            continue;
        }

        _class_id = class_id.x;
        _color_id = color_id.x;
        ArmorObject obj;
        obj.prob = confidence;
        obj.color = _color_id;
        obj.label = _class_id;
        obj.landmarks[0]=output_buffer.at<float>(i, 0);
        obj.landmarks[1]=output_buffer.at<float>(i, 1);
        obj.landmarks[2]=output_buffer.at<float>(i, 2);
        obj.landmarks[3]=output_buffer.at<float>(i, 3);
        obj.landmarks[4]=output_buffer.at<float>(i, 4);
        obj.landmarks[5]=output_buffer.at<float>(i, 5);
        obj.landmarks[6]=output_buffer.at<float>(i, 6);
        obj.landmarks[7]=output_buffer.at<float>(i, 7);
        obj.center=cv::Point2f((obj.landmarks[0]+obj.landmarks[2]+obj.landmarks[4]+obj.landmarks[6])/4,
                                (obj.landmarks[1]+obj.landmarks[3]+obj.landmarks[5]+obj.landmarks[7])/4);
        obj.length = cv::norm(cv::Point2f(obj.landmarks[0] - obj.landmarks[6])-cv::Point2f(obj.landmarks[1]-obj.landmarks[7]));
        obj.width = cv::norm(cv::Point2f(obj.landmarks[0] - obj.landmarks[2])-cv::Point2f(obj.landmarks[1]-obj.landmarks[3]));
        obj.ratio = obj.length / obj.width;

        if(obj.label == 1 ){//新赛季只有英雄和基地是大装甲板
            obj.type=ArmorType::LARGE;
        }
          else if(obj.label == 8){
            obj.type=ArmorType::LARGE;
        }
          else{
            obj.type=ArmorType::SMALL;
        }
        //std::vector<cv::Point2f> points;
        //landmarks为左上逆时针，points应为左下顺时针，与中南对应
        obj.points.push_back(cv::Point2f(obj.landmarks[2]*obj.coefficient_width, obj.landmarks[3]*obj.coefficient_height));
        obj.points.push_back(cv::Point2f(obj.landmarks[0]*obj.coefficient_width, obj.landmarks[1]*obj.coefficient_height));
        obj.points.push_back(cv::Point2f(obj.landmarks[6]*obj.coefficient_width, obj.landmarks[7]*obj.coefficient_height));
        obj.points.push_back(cv::Point2f(obj.landmarks[4]*obj.coefficient_width, obj.landmarks[5]*obj.coefficient_height));

        // Find the minimum and maximum x and y coordinates
        float min_x = obj.points[0].x;
        float max_x = obj.points[0].x;
        float min_y = obj.points[0].y;
        float max_y = obj.points[0].y;

        for (int i = 1; i < int(obj.points.size()); i++)
        {
            if (obj.points[i].x < min_x)
                min_x = obj.points[i].x;
            if (obj.points[i].x > max_x)
                max_x = obj.points[i].x;
            if (obj.points[i].y < min_y)
                min_y = obj.points[i].y;
            if (obj.points[i].y > max_y)
                max_y = obj.points[i].y;
        }

        // Create the rectangle
        cv::Rect rect(min_x, min_y, max_x - min_x, max_y - min_y);
        obj.rect = rect;
        objects.push_back(obj);
        boxes.push_back(rect);
        confidences.push_back(score_num);
    }

    // NMS
    std::vector<int> indices;
    cv::dnn::NMSBoxes(boxes, confidences, conf_threshold, nms_threshold, indices);
    for(int valid_index:indices){
        if(valid_index < int(objects.size())){
            tmp_objects.push_back(objects[valid_index]);
        }
    }

    // // ---------------------- 纯置信度筛选逻辑 ----------------------
    // if (!tmp_objects.empty()) {
    //     // 使用标准库算法直接比较prob字段
    //     auto max_iter = std::max_element(
    //         tmp_objects.begin(),
    //         tmp_objects.end(),
    //         [](const ArmorObject& a, const ArmorObject& b) {
    //             // 直接比较置信度概率值
    //             return a.prob < b.prob;
    //         });

    //     // 仅保留置信度最高的单个目标
    //     tmp_objects = {*max_iter};  // C++11初始化列表语法
    // }


    // ---------------------- 增强型筛选逻辑 ----------------------
    if (!tmp_objects.empty()) {
        const float CONFIDENCE_DIFF_THRESHOLD = 0.05f;  // 置信度差异阈值
        const float CONFIDENCE_WEIGHT = 0.1f;          // 置信度权重
        const float AREA_WEIGHT = 0.45f;                // 面积权重
        const float ANGLE_WEIGHT = 0.45f;               // 角度权重

        // 计算装甲板面积（基于四点坐标）
        auto calc_area = [](const ArmorObject& obj) {
            std::vector<cv::Point2f> contour = {obj.points[0], obj.points[1], 
                                            obj.points[2], obj.points[3]};
            return cv::contourArea(contour);  // 使用OpenCV面积计算
        };

        // 计算装甲板正向角度（越小越正对）
        auto calc_angle = [](const ArmorObject& obj) {
            cv::Point2f p1 = obj.points[1];  // 左上点
            cv::Point2f p2 = obj.points[2];  // 右上点
            float dx = p2.x - p1.x;
            float dy = p1.y - p2.y;          // Y轴向下需取反
            return (dx == 0) ? CV_PI/2 : std::abs(std::atan(dy/dx));
        };

        // 综合评分排序
        std::sort(tmp_objects.begin(), tmp_objects.end(),
            [=](const ArmorObject& a, const ArmorObject& b) {
                // 当置信度差异显著时优先置信度
                if (std::abs(a.prob - b.prob) > CONFIDENCE_DIFF_THRESHOLD) {
                    return a.prob > b.prob;
                }
                
                // 否则计算综合评分
                float score_a = a.prob * CONFIDENCE_WEIGHT + 
                            calc_area(a) * AREA_WEIGHT - 
                            calc_angle(a) * ANGLE_WEIGHT;
                
                float score_b = b.prob * CONFIDENCE_WEIGHT +
                            calc_area(b) * AREA_WEIGHT -
                            calc_angle(b) * ANGLE_WEIGHT;
                
                return score_a > score_b;
            });

        tmp_objects = {tmp_objects[0]};
    }

    // ---------------------- 筛选逻辑结束 ----------------------

    //callback
    if (this->infer_callback_) {
    this->infer_callback_(tmp_objects, timestamp_nanosec, img);
    return true;
    }

    return false;
}

}

