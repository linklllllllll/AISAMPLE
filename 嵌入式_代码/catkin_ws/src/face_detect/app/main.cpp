// Copyright (C) 2018-2022 Intel Corporation
// SPDX-License-Identifier: Apache-2.0
//

#include <chrono>  // NOLINT

#include <algorithm>
#include <deque>
#include <map>
#include <memory>
#include <limits>
#include <set>
#include <string>
#include <inference_engine.hpp>
#include <utility>
#include <iostream>
#include <limits>
#include <vector>
#include <math.h>
#include <Eigen/Eigen>
#include <stdlib.h>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <gflags/gflags.h>
#include <utils/images_capture.h>
#include <monitors/presenter.h>
#include "sensor_msgs/CompressedImage.h"
#include "sensor_msgs/image_encodings.h"
#include <sensor_msgs/CameraInfo.h>
#include "detectors.hpp"
#include "face.hpp"
#include "visualizer.hpp"
#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "utils/args_helper.hpp"
 
#include "utils/ocv_common.hpp"
#include "utils/slog.hpp"
#include "cnn.hpp"
 
#include "detector.hpp"
#include "face_reid.hpp"
#include "tracker.hpp"
#include "logger.hpp"
 
#include "std_msgs/Bool.h"
#include "std_msgs/Int32.h"
#include "geometry_msgs/PoseStamped.h"
using namespace cv;
using namespace std;
using namespace Eigen;

using namespace InferenceEngine;
#define tcout                  std::cout
#define file_name_t            std::string
#define imread_t               cv::imread
#define NMS_THRESH 0.45
#define BBOX_CONF_THRESH 0.60

static const int INPUT_W = 416;
static const int INPUT_H = 416;
static const int NUM_CLASSES = 1;
namespace {
constexpr char h_msg[] = "show the help message and exit";
DEFINE_bool(h, false, h_msg);

constexpr char m_msg[] = "path to an .xml file with a trained Face Detection model";
DEFINE_string(m, "/home/ubuntu/face_recon/intel (1)/face-detection-adas-0001/FP16/face-detection-adas-0001.xml", m_msg);

constexpr char i_msg[] = "an input to process. The input must be a single image, a folder of images, video file or camera id. Default is 0";
DEFINE_string(i, "2", i_msg);

constexpr char bb_enlarge_coef_msg[] = "coefficient to enlarge/reduce the size of the bounding box around the detected face. Default is 1.2";
DEFINE_double(bb_enlarge_coef, 1.2, bb_enlarge_coef_msg);

constexpr char d_msg[] =
    "specify a device to infer on (the list of available devices is shown below). "
    "Use '-d HETERO:<comma-separated_devices_list>' format to specify HETERO plugin. "
    "Use '-d MULTI:<comma-separated_devices_list>' format to specify MULTI plugin. "
    "Default is CPU";
DEFINE_string(d, "CPU", d_msg);

constexpr char dx_coef_msg[] = "coefficient to shift the bounding box around the detected face along the Ox axis";
DEFINE_double(dx_coef, 1, dx_coef_msg);

constexpr char dy_coef_msg[] = "coefficient to shift the bounding box around the detected face along the Oy axis";
DEFINE_double(dy_coef, 1, dy_coef_msg);

constexpr char fps_msg[] = "maximum FPS for playing video";
DEFINE_double(fps, -std::numeric_limits<double>::infinity(), fps_msg);

constexpr char lim_msg[] = "number of frames to store in output. If 0 is set, all frames are stored. Default is 1000";
DEFINE_uint32(lim, 1000, lim_msg);

constexpr char loop_msg[] = "enable reading the input in a loop";
DEFINE_bool(loop, false, loop_msg);

constexpr char mag_msg[] = "path to an .xml file with a trained Age/Gender Recognition model";
DEFINE_string(mag, "/home/ubuntu/face_recon/intel (1)/age-gender-recognition-retail-0013/FP16/age-gender-recognition-retail-0013.xml", mag_msg);

constexpr char mam_msg[] = "path to an .xml file with a trained Antispoofing Classification model";
DEFINE_string(mam, "/home/ubuntu/face_recon/public/anti-spoof-mn3/FP16/anti-spoof-mn3.xml", mam_msg);

constexpr char mem_msg[] = "path to an .xml file with a trained Emotions Recognition model";
DEFINE_string(mem, "/home/ubuntu/face_recon/intel (1)/emotions-recognition-retail-0003/FP16/emotions-recognition-retail-0003.xml", mem_msg);

constexpr char mhp_msg[] = "path to an .xml file with a trained Head Pose Estimation model";
DEFINE_string(mhp, "/home/ubuntu/face_recon/intel (1)/head-pose-estimation-adas-0001/FP16/head-pose-estimation-adas-0001.xml", mhp_msg);

constexpr char mlm_msg[] = "path to an .xml file with a trained Facial Landmarks Estimation model";
DEFINE_string(mlm, "/home/ubuntu/face_recon/intel (1)/facial-landmarks-35-adas-0002/FP16/facial-landmarks-35-adas-0002.xml", mlm_msg);

constexpr char o_msg[] = "name of the output file(s) to save";
DEFINE_string(o, "", o_msg);

constexpr char r_msg[] = "output inference results as raw values";
DEFINE_bool(r, false, r_msg);

constexpr char show_msg[] = "(don't) show output";
DEFINE_bool(show, true, show_msg);

constexpr char show_emotion_bar_msg[] = "(don't) show emotion bar";
DEFINE_bool(show_emotion_bar, true, show_emotion_bar_msg);

constexpr char smooth_msg[] = "(don't) smooth person attributes";
DEFINE_bool(smooth, true, smooth_msg);

constexpr char t_msg[] = "probability threshold for detections. Default is 0.5";
DEFINE_double(t, 0.5, t_msg);

constexpr char u_msg[] = "resource utilization graphs. Default is cdm. "
    "c - average CPU load, d - load distribution over cores, m - memory usage, h - hide";
DEFINE_string(u, "cdm", u_msg);
cv::Scalar xAxisColor = cv::Scalar(0, 0, 255);
cv::Scalar yAxisColor = cv::Scalar(0, 255, 0);
cv::Scalar zAxisColor = cv::Scalar(255, 0, 0);
float scale = 50;
int axisThickness = 2;
double yaw    ;
double pitch  ;
double roll   ;
cv::Mat static_resize(cv::Mat& img) {
    float r = std::min(INPUT_W / (img.cols*1.0), INPUT_H / (img.rows*1.0));
    // r = std::min(r, 1.0f);
    int unpad_w = r * img.cols;
    int unpad_h = r * img.rows;
    cv::Mat re(unpad_h, unpad_w, CV_8UC3);
    cv::resize(img, re, re.size());
    //cv::Mat out(INPUT_W, INPUT_H, CV_8UC3, cv::Scalar(114, 114, 114));
    cv::Mat out(INPUT_H, INPUT_W, CV_8UC3, cv::Scalar(114, 114, 114));
    re.copyTo(out(cv::Rect(0, 0, re.cols, re.rows)));
    return out;
}
void blobFromImage(cv::Mat& img, Blob::Ptr& blob){
    int channels = 3;
    int img_h = img.rows;
    int img_w = img.cols;
    InferenceEngine::MemoryBlob::Ptr mblob = InferenceEngine::as<InferenceEngine::MemoryBlob>(blob);
    if (!mblob) 
    {
        THROW_IE_EXCEPTION << "We expect blob to be inherited from MemoryBlob in matU8ToBlob, "
            << "but by fact we were not able to cast inputBlob to MemoryBlob";
    }
    // locked memory holder should be alive all time while access to its buffer happens
    auto mblobHolder = mblob->wmap();

    float *blob_data = mblobHolder.as<float *>();

    for (size_t c = 0; c < channels; c++) 
    {
        for (size_t  h = 0; h < img_h; h++) 
        {
            for (size_t w = 0; w < img_w; w++) 
            {
                blob_data[c * img_w * img_h + h * img_w + w] =
                    (float)img.at<cv::Vec3b>(h, w)[c];
            }
        }
    }
}
struct Object
{
    cv::Rect_<float> rect;
    int label;
    float prob;
};

int clamp(
	const int 									x)
{
    if (x > 255)
        return 255;
    if (x < 0)
        return 0;
	
    return x;
}
int imgBrightness(
	const Mat									srcImg,//输入图片
	const float									brightness,//亮度比值
	Mat 										&outImg)//输出图片
{
	 
	//
	int nRet = 0;
	int row, col;
	int srcWidth, srcHeight;
	int rgbmeans[3];
	double redSum, greenSum, blueSum;
	double total;
	int pixelValue;
 
	//r、g、b像素值累加
	redSum = 0;
	greenSum = 0;
	blueSum = 0;
	//
	srcWidth = srcImg.cols;
	srcHeight = srcImg.rows;	
	total = srcWidth * srcHeight;
 
	//获取rgb means
	for(row = 0; row < srcHeight; row++){
		auto ptr = srcImg.ptr<uchar>(row);  
		int tr = 0, tg = 0, tb = 0;
		for(col = 0; col < srcWidth; col++){
			tr =  ptr[2];
			tg =  ptr[1];
			tb =  ptr[0];
			
			redSum += tr;
			greenSum += tg;
			blueSum +=tb;
 
			ptr += 3;
		}
	}
	rgbmeans[0] = (int)(redSum / total);
	rgbmeans[1] = (int)(greenSum / total);
	rgbmeans[2] = (int)(blueSum / total);
 
	// 调整亮度
	outImg = srcImg.clone();
	for(row = 0; row < srcHeight; row++) {
		auto ptr = srcImg.ptr<uchar>(row);  
		auto qtr = outImg.ptr<uchar>(row);  
		int tr = 0, tg = 0, tb = 0;
		for(col = 0; col < srcWidth; col++) {
			//获取r、g、b值
			tr =  ptr[2];
			tg =  ptr[1];
			tb =  ptr[0];	
	        
	        // 均值消减
	        tr -=rgbmeans[0];
	        tg -=rgbmeans[1];
	        tb -=rgbmeans[2];
	        	        
	        // 亮度调整
	        tr += (int)(rgbmeans[0] * brightness);
	        tg += (int)(rgbmeans[1] * brightness);
	        tb += (int)(rgbmeans[2] * brightness);
		
		//为目标输出图片赋值
			qtr[0] = clamp(tb);
			qtr[1] = clamp(tg);
			qtr[2] = clamp(tr);
 
			ptr += 3;
			qtr += 3;
		}
    }
	
	return nRet;
}
struct GridAndStride
{
    int grid0;
    int grid1;
    int stride;
};

static void generate_grids_and_stride(const int target_w, const int target_h, std::vector<int>& strides, std::vector<GridAndStride>& grid_strides)
{
    for (auto stride : strides)
    {
        int num_grid_w = target_w / stride;
        int num_grid_h = target_h / stride;
        for (int g1 = 0; g1 < num_grid_h; g1++)
        {
            for (int g0 = 0; g0 < num_grid_w; g0++)
            {
                grid_strides.push_back((GridAndStride){g0, g1, stride});
            }
        }
    }
}

static void generate_yolox_proposals(std::vector<GridAndStride> grid_strides, const float* feat_ptr, float prob_threshold, std::vector<Object>& objects)
{

    const int num_anchors = grid_strides.size();

    for (int anchor_idx = 0; anchor_idx < num_anchors; anchor_idx++)
    {
        const int grid0 = grid_strides[anchor_idx].grid0;
        const int grid1 = grid_strides[anchor_idx].grid1;
        const int stride = grid_strides[anchor_idx].stride;

	const int basic_pos = anchor_idx * (NUM_CLASSES + 5);

        // yolox/models/yolo_head.py decode logic
        //  outputs[..., :2] = (outputs[..., :2] + grids) * strides
        //  outputs[..., 2:4] = torch.exp(outputs[..., 2:4]) * strides
        float x_center = (feat_ptr[basic_pos + 0] + grid0) * stride;
        float y_center = (feat_ptr[basic_pos + 1] + grid1) * stride;
        float w = exp(feat_ptr[basic_pos + 2]) * stride;
        float h = exp(feat_ptr[basic_pos + 3]) * stride;
        float x0 = x_center - w * 0.5f;
        float y0 = y_center - h * 0.5f;

        float box_objectness = feat_ptr[basic_pos + 4];
        for (int class_idx = 0; class_idx < NUM_CLASSES; class_idx++)
        {
            float box_cls_score = feat_ptr[basic_pos + 5 + class_idx];
            float box_prob = box_objectness * box_cls_score;
            if (box_prob > prob_threshold)
            {
                Object obj;
                obj.rect.x = x0;
                obj.rect.y = y0;
                obj.rect.width = w;
                obj.rect.height = h;
                obj.label = class_idx;
                obj.prob = box_prob;

                objects.push_back(obj);
            }

        } // class loop

    } // point anchor loop
}

static inline float intersection_area(const Object& a, const Object& b)
{
    cv::Rect_<float> inter = a.rect & b.rect;
    return inter.area();
}

static void qsort_descent_inplace(std::vector<Object>& faceobjects, int left, int right)
{
    int i = left;
    int j = right;
    float p = faceobjects[(left + right) / 2].prob;

    while (i <= j)
    {
        while (faceobjects[i].prob > p)
            i++;

        while (faceobjects[j].prob < p)
            j--;

        if (i <= j)
        {
            // swap
            std::swap(faceobjects[i], faceobjects[j]);

            i++;
            j--;
        }
    }

    #pragma omp parallel sections
    {
        #pragma omp section
        {
            if (left < j) qsort_descent_inplace(faceobjects, left, j);
        }
        #pragma omp section
        {
            if (i < right) qsort_descent_inplace(faceobjects, i, right);
        }
    }
}


static void qsort_descent_inplace(std::vector<Object>& objects)
{
    if (objects.empty())
        return;

    qsort_descent_inplace(objects, 0, objects.size() - 1);
}

static void nms_sorted_bboxes(const std::vector<Object>& faceobjects, std::vector<int>& picked, float nms_threshold)
{
    picked.clear();

    const int n = faceobjects.size();

    std::vector<float> areas(n);
    for (int i = 0; i < n; i++)
    {
        areas[i] = faceobjects[i].rect.area();
    }

    for (int i = 0; i < n; i++)
    {
        const Object& a = faceobjects[i];

        int keep = 1;
        for (int j = 0; j < (int)picked.size(); j++)
        {
            const Object& b = faceobjects[picked[j]];

            // intersection over union
            float inter_area = intersection_area(a, b);
            float union_area = areas[i] + areas[picked[j]] - inter_area;
            // float IoU = inter_area / union_area
            if (inter_area / union_area > nms_threshold)
                keep = 0;
        }

        if (keep)
            picked.push_back(i);
    }
}


static void decode_outputs(const float* prob, std::vector<Object>& objects, float scale, const int img_w, const int img_h) {
        std::vector<Object> proposals;
        std::vector<int> strides = {8, 16, 32};
        std::vector<GridAndStride> grid_strides;

        generate_grids_and_stride(INPUT_W, INPUT_H, strides, grid_strides);
        generate_yolox_proposals(grid_strides, prob,  BBOX_CONF_THRESH, proposals);
        qsort_descent_inplace(proposals);

        std::vector<int> picked;
        nms_sorted_bboxes(proposals, picked, NMS_THRESH);
        int count = picked.size();
        objects.resize(count);

        for (int i = 0; i < count; i++)
        {
            objects[i] = proposals[picked[i]];

            // adjust offset to original unpadded
            float x0 = (objects[i].rect.x) / scale;
            float y0 = (objects[i].rect.y) / scale;
            float x1 = (objects[i].rect.x + objects[i].rect.width) / scale;
            float y1 = (objects[i].rect.y + objects[i].rect.height) / scale;

            // clip
            x0 = std::max(std::min(x0, (float)(img_w - 1)), 0.f);
            y0 = std::max(std::min(y0, (float)(img_h - 1)), 0.f);
            x1 = std::max(std::min(x1, (float)(img_w - 1)), 0.f);
            y1 = std::max(std::min(y1, (float)(img_h - 1)), 0.f);

            objects[i].rect.x = x0;
            objects[i].rect.y = y0;
            objects[i].rect.width = x1 - x0;
            objects[i].rect.height = y1 - y0;
        }
}

const float color_list[80][3] =
{
    {0.000, 0.447, 0.741},
    {0.850, 0.325, 0.098},
    {0.929, 0.694, 0.125},
    {0.494, 0.184, 0.556},
    {0.466, 0.674, 0.188},
    {0.301, 0.745, 0.933},
    {0.635, 0.078, 0.184},
    {0.300, 0.300, 0.300},
    {0.600, 0.600, 0.600},
    {1.000, 0.000, 0.000},
    {1.000, 0.500, 0.000},
    {0.749, 0.749, 0.000},
    {0.000, 1.000, 0.000},
    {0.000, 0.000, 1.000},
    {0.667, 0.000, 1.000},
    {0.333, 0.333, 0.000},
    {0.333, 0.667, 0.000},
    {0.333, 1.000, 0.000},
    {0.667, 0.333, 0.000},
    {0.667, 0.667, 0.000},
    {0.667, 1.000, 0.000},
    {1.000, 0.333, 0.000},
    {1.000, 0.667, 0.000},
    {1.000, 1.000, 0.000},
    {0.000, 0.333, 0.500},
    {0.000, 0.667, 0.500},
    {0.000, 1.000, 0.500},
    {0.333, 0.000, 0.500},
    {0.333, 0.333, 0.500},
    {0.333, 0.667, 0.500},
    {0.333, 1.000, 0.500},
    {0.667, 0.000, 0.500},
    {0.667, 0.333, 0.500},
    {0.667, 0.667, 0.500},
    {0.667, 1.000, 0.500},
    {1.000, 0.000, 0.500},
    {1.000, 0.333, 0.500},
    {1.000, 0.667, 0.500},
    {1.000, 1.000, 0.500},
    {0.000, 0.333, 1.000},
    {0.000, 0.667, 1.000},
    {0.000, 1.000, 1.000},
    {0.333, 0.000, 1.000},
    {0.333, 0.333, 1.000},
    {0.333, 0.667, 1.000},
    {0.333, 1.000, 1.000},
    {0.667, 0.000, 1.000},
    {0.667, 0.333, 1.000},
    {0.667, 0.667, 1.000},
    {0.667, 1.000, 1.000},
    {1.000, 0.000, 1.000},
    {1.000, 0.333, 1.000},
    {1.000, 0.667, 1.000},
    {0.333, 0.000, 0.000},
    {0.500, 0.000, 0.000},
    {0.667, 0.000, 0.000},
    {0.833, 0.000, 0.000},
    {1.000, 0.000, 0.000},
    {0.000, 0.167, 0.000},
    {0.000, 0.333, 0.000},
    {0.000, 0.500, 0.000},
    {0.000, 0.667, 0.000},
    {0.000, 0.833, 0.000},
    {0.000, 1.000, 0.000},
    {0.000, 0.000, 0.167},
    {0.000, 0.000, 0.333},
    {0.000, 0.000, 0.500},
    {0.000, 0.000, 0.667},
    {0.000, 0.000, 0.833},
    {0.000, 0.000, 1.000},
    {0.000, 0.000, 0.000},
    {0.143, 0.143, 0.143},
    {0.286, 0.286, 0.286},
    {0.429, 0.429, 0.429},
    {0.571, 0.571, 0.571},
    {0.714, 0.714, 0.714},
    {0.857, 0.857, 0.857},
    {0.000, 0.447, 0.741},
    {0.314, 0.717, 0.741},
    {0.50, 0.5, 0}
};

    
static void draw_objects(const cv::Mat& bgr, const std::vector<Object>& objects)
{
    static const char* class_names[] = {
        "tonsil"
    };

    cv::Mat image = bgr.clone();

    for (size_t i = 0; i < objects.size(); i++)
    {
        const Object& obj = objects[i];
        
        cv::Scalar color = cv::Scalar(color_list[obj.label][0], color_list[obj.label][1], color_list[obj.label][2]);
        float c_mean = cv::mean(color)[0];
        cv::Scalar txt_color;
        if (c_mean > 0.5){
            txt_color = cv::Scalar(0, 0, 0);
        }else{
            txt_color = cv::Scalar(255, 255, 255);
        }

        cv::rectangle(image, obj.rect, color * 255, 2);

        char text[256];
        
         
        

        int baseLine = 0;
        cv::Size label_size = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 0.4, 1, &baseLine);

        cv::Scalar txt_bk_color = color * 0.7 * 255;

        int x = obj.rect.x;
        int y = obj.rect.y + 1;
        //int y = obj.rect.y - label_size.height - baseLine;
        if (y > image.rows)
            y = image.rows;
         
            
            sprintf(text, "%s %.1f%%  ", class_names[obj.label], obj.prob * 100 ); 
            // fprintf(stderr, "[%f,%f,%f]" ,real_x,real_y,real_z); 


        cv::rectangle(image, cv::Rect(cv::Point(x, y), cv::Size(label_size.width, label_size.height + baseLine)),
                      txt_bk_color, -1);

        cv::putText(image, text, cv::Point(x, y + label_size.height),
                    cv::FONT_HERSHEY_SIMPLEX, 0.4, txt_color, 1);
         
    }
 
    cv::imshow("mouth", image); 
    cv::waitKey(1); 
}
 Core ie;
       
        CNNNetwork network = ie.ReadNetwork("/home/ubuntu/catkin_ws/src/face_detect/app/yolox_s1.xml");
       
       
        InputInfo::Ptr input_info = network.getInputsInfo().begin()->second;
        std::string input_name = network.getInputsInfo().begin()->first;
        DataPtr output_info = network.getOutputsInfo().begin()->second;
        std::string output_name = network.getOutputsInfo().begin()->first;

        
   
        ExecutableNetwork executable_network = ie.LoadNetwork(network, "CPU");
 
         InferRequest infer_request = executable_network.CreateInferRequest();
class FaceRecognizer {
public:
    virtual ~FaceRecognizer() = default;

    virtual bool LabelExists(const std::string& label) const = 0;
    virtual std::string GetLabelByID(int id) const = 0;
    virtual std::vector<std::string> GetIDToLabelMap() const = 0;

    virtual std::vector<int> Recognize(const cv::Mat& frame, const detection::DetectedObjects& faces) = 0;
};

class FaceRecognizerNull : public FaceRecognizer {
public:
    bool LabelExists(const std::string&) const override { return false; }

    std::string GetLabelByID(int) const override {
        return EmbeddingsGallery::unknown_label;
    }

    std::vector<std::string> GetIDToLabelMap() const override { return {}; }

    std::vector<int> Recognize(const cv::Mat&, const detection::DetectedObjects& faces) override {
        return std::vector<int>(faces.size(), EmbeddingsGallery::unknown_id);
    }
};

class FaceRecognizerDefault : public FaceRecognizer {
public:
    FaceRecognizerDefault(
            const CnnConfig& landmarks_detector_config,
            const CnnConfig& reid_config,
            const detection::DetectorConfig& face_registration_det_config,
            const std::string& face_gallery_path,
            double reid_threshold,
            int min_size_fr,
            bool crop_gallery,
            bool greedy_reid_matching) :
        landmarks_detector(landmarks_detector_config),
        face_reid(reid_config),
        face_gallery(face_gallery_path, reid_threshold, min_size_fr, crop_gallery,
                     face_registration_det_config, landmarks_detector, face_reid,
                     greedy_reid_matching)
    {
        if (face_gallery.size() == 0) {
            slog::warn << "Face reid gallery is empty!" << slog::endl;
        } else {
            slog::info << "Face reid gallery size: " << face_gallery.size() << slog::endl;
        }
    }

    bool LabelExists(const std::string& label) const override {
        return face_gallery.LabelExists(label);
    }

    std::string GetLabelByID(int id) const override {
        return face_gallery.GetLabelByID(id);
    }

    std::vector<std::string> GetIDToLabelMap() const override {
        return face_gallery.GetIDToLabelMap();
    }

    std::vector<int> Recognize(const cv::Mat& frame, const detection::DetectedObjects& faces) override {
        const int maxLandmarksBatch = landmarks_detector.maxBatchSize();
        int numFaces = (int)faces.size();

        std::vector<cv::Mat> landmarks;
        std::vector<cv::Mat> embeddings;
        std::vector<cv::Mat> face_rois;

        auto face_roi = [&](const detection::DetectedObject& face1) {
            return frame(face1.rect);
        };
        if (numFaces < maxLandmarksBatch) {
            std::transform(faces.begin(), faces.end(), std::back_inserter(face_rois), face_roi);
            landmarks_detector.Compute(face_rois, &landmarks, cv::Size(2, 5));
            AlignFaces(&face_rois, &landmarks);
            face_reid.Compute(face_rois, &embeddings);
        } else {
            auto embedding = [&](cv::Mat& emb) { return emb; };
            for (int n = numFaces; n > 0; n -= maxLandmarksBatch) {
                landmarks.clear();
                face_rois.clear();
                size_t start_idx = size_t(numFaces) - n;
                size_t end_idx = start_idx + std::min(numFaces, maxLandmarksBatch);
                std::transform(faces.begin() + start_idx, faces.begin() + end_idx, std::back_inserter(face_rois), face_roi);

                landmarks_detector.Compute(face_rois, &landmarks, cv::Size(2, 5));

                AlignFaces(&face_rois, &landmarks);

                std::vector<cv::Mat> batch_embeddings;
                face_reid.Compute(face_rois, &batch_embeddings);
                std::transform(batch_embeddings.begin(), batch_embeddings.end(), std::back_inserter(embeddings), embedding);
            }
        }

        return face_gallery.GetIDsByEmbeddings(embeddings);
    }

private:
    VectorCNN landmarks_detector;
    VectorCNN face_reid;
    EmbeddingsGallery face_gallery;
};

Eigen::Quaterniond euler2Quaternion(const double roll, const double pitch, const double yaw)
{
    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitX());
 
    Eigen::Quaterniond q = rollAngle * yawAngle * pitchAngle;
 
    return q;
}
 
void buildCameraMatrix(cv::Mat& cameraMatrix, int cx, int cy, float focalLength) {
    if (!cameraMatrix.empty()) return;
    cameraMatrix = cv::Mat::zeros(3, 3, CV_32F);
    cameraMatrix.at<float>(0) = focalLength;
    cameraMatrix.at<float>(2) = static_cast<float>(cx);
    cameraMatrix.at<float>(4) = focalLength;
    cameraMatrix.at<float>(5) = static_cast<float>(cy);
    cameraMatrix.at<float>(8) = 1;
}
void draw111(cv::Mat& frame, cv::Point3f cpoint, HeadPoseDetection::Results headPose) {
    yaw   = headPose.angle_y;
    pitch = headPose.angle_p;
    roll  = headPose.angle_r;

    pitch *= CV_PI / 180.0;
    yaw   *= CV_PI / 180.0;
    roll  *= CV_PI / 180.0;

    cv::Matx33f Rx(1, 0, 0,
                   0, static_cast<float>(cos(pitch)), static_cast<float>(-sin(pitch)),
                   0, static_cast<float>(sin(pitch)), static_cast<float>(cos(pitch)));

    cv::Matx33f Ry(static_cast<float>(cos(yaw)), 0, static_cast<float>(-sin(yaw)),
                   0, 1, 0,
                   static_cast<float>(sin(yaw)), 0, static_cast<float>(cos(yaw)));

    cv::Matx33f Rz(static_cast<float>(cos(roll)), static_cast<float>(-sin(roll)), 0,
                   static_cast<float>(sin(roll)),  static_cast<float>(cos(roll)), 0,
                   0, 0, 1);


    auto r = cv::Mat(Rz*Ry*Rx);
    cv::Mat cameraMatrix;
    buildCameraMatrix(cameraMatrix, frame.cols / 2, frame.rows / 2, 950.0);

    cv::Mat xAxis(3, 1, CV_32F), yAxis(3, 1, CV_32F), zAxis(3, 1, CV_32F), zAxis1(3, 1, CV_32F);

    xAxis.at<float>(0) = 1 * scale;
    xAxis.at<float>(1) = 0;
    xAxis.at<float>(2) = 0;

    yAxis.at<float>(0) = 0;
    yAxis.at<float>(1) = -1 * scale;
    yAxis.at<float>(2) = 0;

    zAxis.at<float>(0) = 0;
    zAxis.at<float>(1) = 0;
    zAxis.at<float>(2) = -1 * scale;

    zAxis1.at<float>(0) = 0;
    zAxis1.at<float>(1) = 0;
    zAxis1.at<float>(2) = 1 * scale;

    cv::Mat o(3, 1, CV_32F, cv::Scalar(0));
    o.at<float>(2) = cameraMatrix.at<float>(0);

    xAxis = r * xAxis + o;
    yAxis = r * yAxis + o;
    zAxis = r * zAxis + o;
    zAxis1 = r * zAxis1 + o;

    cv::Point p1, p2;

    p2.x = static_cast<int>((xAxis.at<float>(0) / xAxis.at<float>(2) * cameraMatrix.at<float>(0)) + cpoint.x);
    p2.y = static_cast<int>((xAxis.at<float>(1) / xAxis.at<float>(2) * cameraMatrix.at<float>(4)) + cpoint.y);
    cv::line(frame, cv::Point(static_cast<int>(cpoint.x), static_cast<int>(cpoint.y)), p2, xAxisColor, axisThickness);

    p2.x = static_cast<int>((yAxis.at<float>(0) / yAxis.at<float>(2) * cameraMatrix.at<float>(0)) + cpoint.x);
    p2.y = static_cast<int>((yAxis.at<float>(1) / yAxis.at<float>(2) * cameraMatrix.at<float>(4)) + cpoint.y);
    cv::line(frame, cv::Point(static_cast<int>(cpoint.x), static_cast<int>(cpoint.y)), p2, yAxisColor, axisThickness);

    p1.x = static_cast<int>((zAxis1.at<float>(0) / zAxis1.at<float>(2) * cameraMatrix.at<float>(0)) + cpoint.x);
    p1.y = static_cast<int>((zAxis1.at<float>(1) / zAxis1.at<float>(2) * cameraMatrix.at<float>(4)) + cpoint.y);

    p2.x = static_cast<int>((zAxis.at<float>(0) / zAxis.at<float>(2) * cameraMatrix.at<float>(0)) + cpoint.x);
    p2.y = static_cast<int>((zAxis.at<float>(1) / zAxis.at<float>(2) * cameraMatrix.at<float>(4)) + cpoint.y);
    cv::line(frame, p1, p2, zAxisColor, axisThickness);
    cv::circle(frame, p2, 3, zAxisColor, axisThickness);
}
void parse(int argc, char *argv[]) {
    gflags::ParseCommandLineFlags(&argc, &argv, false);
  
    slog::info << ov::get_openvino_version() << slog::endl;
}}
// namespace
    

    // --------------------------- 1. Loading Inference Engine -----------------------------
    ov::Core core;

    FaceDetection faceDetector(FLAGS_m, FLAGS_t, FLAGS_r,
                                static_cast<float>(FLAGS_bb_enlarge_coef), static_cast<float>(FLAGS_dx_coef), static_cast<float>(FLAGS_dy_coef));
    
    HeadPoseDetection headPoseDetector(FLAGS_mhp, FLAGS_r);
   
    FacialLandmarksDetection facialLandmarksDetector(FLAGS_mlm, FLAGS_r);
    std::unique_ptr<FaceRecognizer> face_recognizer;
    std::unique_ptr<AsyncDetection<detection::DetectedObject>> face_detector;
    
    // ---------------------------------------------------------------------------------------------------

    // --------------------------- 2. Reading IR models and loading them to plugins ----------------------
   
 
    // ----------------------------------------------------------------------------------------------------

    Timer timer;
    std::ostringstream out;
    size_t framesCounter = 0;
    double msrate = 1000.0 / FLAGS_fps;
    std::list<Face::Ptr> faces;
    size_t id = 0;
class ImageConverter
 {
 
	ros::NodeHandle			nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber	image_sub_color;//接收彩色图像
	image_transport::Subscriber	image_sub_depth;//接收深度图像

	ros::Subscriber camera_info_sub_;//接收深度图像对应的相机参数话题
	ros::Publisher pub;//姿态话题发布
    ros::Publisher pub2;
    ros::Publisher pub3;
    ros::Subscriber id_sub;

    int id1111=0 ;
    int count=0;
	sensor_msgs::CameraInfo		camera_info;
	geometry_msgs::PoseStamped  ps;
    std_msgs::Bool mouth_judge;
	/* Mat depthImage,colorImage; */
	Mat	colorImage;
    Mat	colorImage1;
    Mat	colorImage2;
	Mat	depthImage	= Mat::zeros( 480, 640, CV_16UC1 );//注意这里要修改为你接收的深度图像尺寸
  

public:
	ImageConverter() : it_( nh_ )
	{

        
        
            // Load face detector
            detection::DetectorConfig face_config("/home/ubuntu/face_recon/intel/face-detection-adas-0001/FP16/face-detection-adas-0001.xml");
            face_config.m_deviceName = "CPU";
            face_config.m_core = core;
            face_config.is_async = true;
            face_config.confidence_threshold = 0.6;
            face_config.input_h = 480;
            face_config.input_w = 640;
            face_config.increase_scale_x = 1.15;
            face_config.increase_scale_y =1.15;
            face_detector.reset(new detection::FaceDetection(face_config));
     detection::DetectorConfig face_registration_det_config("/home/ubuntu/face_recon/intel/face-detection-adas-0001/FP16/face-detection-adas-0001.xml");
            face_registration_det_config.m_deviceName = "CPU";
            face_registration_det_config.m_core = core;
            face_registration_det_config.is_async = false;
            face_registration_det_config.confidence_threshold = 0.9;
            face_registration_det_config.increase_scale_x = 1.15;
            face_registration_det_config.increase_scale_y = 1.15;

            CnnConfig reid_config("/home/ubuntu/face_recon/intel/face-reidentification-retail-0095/FP16/face-reidentification-retail-0095.xml", "Face Re-Identification");
            reid_config.m_deviceName = "CPU";
            reid_config.m_max_batch_size = 16;
            reid_config.m_core = core;

            CnnConfig landmarks_config("/home/ubuntu/face_recon/intel/landmarks-regression-retail-0009/FP16/landmarks-regression-retail-0009.xml", "Facial Landmarks Regression");
            landmarks_config.m_deviceName = "CPU";
            landmarks_config.m_max_batch_size = 16;
            landmarks_config.m_core = core;
            face_recognizer.reset(new FaceRecognizerDefault(
                landmarks_config, reid_config,
                face_registration_det_config,
                "/home/ubuntu/face_recon/faces_gallery.json", 0.9, 128, false, false));



    Load(faceDetector).into(core, FLAGS_d);
    
    Load(headPoseDetector).into(core, FLAGS_d);
 
    Load(facialLandmarksDetector).into(core, FLAGS_d);
        
    //topic sub:
		image_sub_depth = it_.subscribe( "/camera/aligned_depth_to_color/image_raw",
						 1, &ImageConverter::imageDepthCb, this);
		image_sub_color = it_.subscribe( "/camera/color/image_raw", 1,
						 &ImageConverter::imageColorCb, this );
		camera_info_sub_ =
			nh_.subscribe( "/camera/aligned_depth_to_color/camera_info", 1,
				       &ImageConverter::cameraInfoCb, this );
    //topic pub:
        pub = nh_.advertise<geometry_msgs::PoseStamped>("/face",100);
		pub2= nh_.advertise<geometry_msgs::PoseStamped>("/face_for_sample",1);
        pub3= nh_.advertise<std_msgs::Bool>("/mouth_judge",1);

        id_sub = nh_.subscribe("/ID",1,&ImageConverter::IdCallback, this);
	}


	~ImageConverter()
	{
	
	}

    void IdCallback(const std_msgs::Int32 &msg)
    {
        id1111 = msg.data;
        //  fprintf( stderr, "(%d,\n",id );
    }

	void cameraInfoCb( const sensor_msgs::CameraInfo &msg )
	{
		camera_info = msg;
	}


	void imageDepthCb( const sensor_msgs:: ImageConstPtr &msg )
	{
		cv_bridge::CvImagePtr cv_ptr;

		try {
			cv_ptr =
				cv_bridge::toCvCopy( msg, sensor_msgs::image_encodings::TYPE_16UC1 );
             
			depthImage = cv_ptr->image;
            // cv::Mat depth = cv_ptr1->image;
            
             

		} catch ( cv_bridge::Exception &e ) {
			ROS_ERROR( "cv_bridge exception: %s", e.what() );
			return;
		}
	}

    void imageColorCb( const sensor_msgs:: ImageConstPtr &msg )
	{
        auto startTime = std::chrono::steady_clock::now();
        PerformanceMetrics metrics;
        cv_bridge::CvImagePtr cv_ptr;
        cv_bridge::CvImagePtr cv_ptr1;
        cv_bridge::CvImagePtr cv_ptr2;
		try {
			cv_ptr		= cv_bridge::toCvCopy( msg, sensor_msgs::image_encodings::BGR8 );
            cv_ptr1		= cv_bridge::toCvCopy( msg, sensor_msgs::image_encodings::BGR8 );
			cv_ptr2		= cv_bridge::toCvCopy( msg, sensor_msgs::image_encodings::BGR8 );
            colorImage	= cv_ptr->image;
            colorImage1	= cv_ptr1->image;
            colorImage2	= cv_ptr2->image;
           
		} catch ( cv_bridge::Exception &e ) {
			ROS_ERROR( "cv_bridge exception: %s", e.what() );
			return;
		}
	    cv::Mat frame = colorImage;
        cv::Mat frame1 = colorImage1;
        cv::Mat frame2 = colorImage2;
        
        // cv::Mat frame1 =cv::imread("/home/linyao/Downloads/IMG_20220516_162422.jpg");
        // cv::resize(frame1, frame1, Size(640, 480), 0, 0, cv::INTER_LINEAR);
        if (!frame.data) {
        throw std::runtime_error("Can't read an image from the input");
        }

        Presenter presenter(FLAGS_u, 60, {frame.cols / 4, 60});
        Visualizer visualizer{frame.size()};
        // //////////////////////////////////////////////////
        faceDetector.submitRequest(frame);
        LazyVideoWriter videoWriter{FLAGS_o, FLAGS_fps > 0.0 ? FLAGS_fps : 30, FLAGS_lim};

        auto startTimeNextFrame = std::chrono::steady_clock::now();
        cv::Mat nextFrame = colorImage;


        while (frame.data) {
            std::chrono::time_point<std::chrono::steady_clock> start = std::chrono::steady_clock::now();
        timer.start("total");
        const auto startTimePrevFrame = startTime;
        cv::Mat prevFrame = std::move(frame);
         cv::Mat prevFrame1 = std::move(frame1);
         cv::Mat prevFrame2 = std::move(frame2);
       
        startTime = startTimeNextFrame;
        // frame = std::move(nextFrame);
        framesCounter++;

        // Retrieving face detection results for the previous frame
        std::vector<FaceDetection::Result> prev_detection_results = faceDetector.fetchResults();
   
        // No valid frame to infer if previous frame is the last
        if (frame.data) {
            if (frame.size() != prevFrame.size()) {
                throw std::runtime_error("Images of different size are not supported");
            }
            faceDetector.submitRequest(frame);
             
        }
        
        for (auto &&face : prev_detection_results) {
            cv::Rect clippedRect = face.location & cv::Rect({0, 0}, prevFrame.size());
            const cv::Mat& crop = prevFrame(clippedRect);

            
            headPoseDetector.enqueue(crop);
           
            facialLandmarksDetector.enqueue(crop);
             
        }

        // Running Age/Gender Recognition, Head Pose Estimation, Emotions Recognition, Facial Landmarks Estimation and Antispoofing Classifier networks simultaneously
       
        headPoseDetector.submitRequest();
      
        facialLandmarksDetector.submitRequest();
       

        // Read the next frame while waiting for inference results
        startTimeNextFrame = std::chrono::steady_clock::now();
       

        //  Postprocessing
        std::list<Face::Ptr> prev_faces;

        if (!FLAGS_smooth) {
            prev_faces.insert(prev_faces.begin(), faces.begin(), faces.end());
        }

        faces.clear();
        detection::DetectedObjects faces222;
        detection::DetectedObject faces111;

        // For every detected face
        for (size_t i = 0; i < prev_detection_results.size(); i++) {
            auto& result = prev_detection_results[i];
            cv::Rect rect = result.location & cv::Rect({0, 0}, prevFrame.size());
            faces111.rect=rect;
            faces111.confidence=result.confidence;

            Face::Ptr face;
            if (FLAGS_smooth) {
                face = matchFace(rect, prev_faces);
                float intensity_mean = calcMean(prevFrame(rect));

                if ((face == nullptr) ||
                    ((std::abs(intensity_mean - face->_intensity_mean) / face->_intensity_mean) > 0.07f)) {
                    face = std::make_shared<Face>(id++, rect);
                } else {
                    prev_faces.remove(face);
                }

                face->_intensity_mean = intensity_mean;
                face->_location = rect;
            } else {
                face = std::make_shared<Face>(id++, rect);
            }

                   

            face->headPoseEnable(headPoseDetector.enabled());
            if (face->isHeadPoseEnabled()) {
                face->updateHeadPose(headPoseDetector[i]);
            }

            face->landmarksEnable(facialLandmarksDetector.enabled());
            if (face->isLandmarksEnabled()) {
                face->updateLandmarks(facialLandmarksDetector[i]);
            }

           

            faces.push_back(face);
            faces222.push_back(faces111);
        }
          
        auto ids = face_recognizer->Recognize(prevFrame, faces222);
         std::string draws;
                for (size_t i = 0; i <prev_detection_results.size(); i++) {
                     
                    if(ids[i]==0)
                    {
                        draws="ly";
                        fprintf( stderr, "(%s,\n",draws.c_str());
                    }
                      if(ids[i]==1)
                    {
                        draws="dw";
                        fprintf( stderr, "(%s,\n",draws.c_str());
                    }
 
                    
                }
      
   
        visualizer.depth=depthImage;
        visualizer.cam_info= camera_info;
        visualizer.drawMap.setTo(0);
        std::vector<Face::Ptr> newFaces;
        int i=0;
        cv::Mat mouth;
         
            for (auto&& face : faces) 
            {
                
                    // visualizer.drawFace(prev_frame, face, false);
                    if (face->isHeadPoseEnabled()) 
                    {
                    cv::Point3f center(static_cast<float>(face->_location.x + face->_location.width / 2),
                                    static_cast<float>(face->_location.y + face->_location.height / 2),
                                    0.0f);
                   draw111(prevFrame1, center, face->getHeadPose());
                                  
         
                    }
                       
                      cv::Rect RectRange(static_cast<float>(face->_location.x),static_cast<float>(face->_location.y),static_cast<float>(face->_location.width),static_cast<float>(face->_location.height));
                      cv::rectangle(prevFrame1,RectRange, Scalar(0,0,255),2,8);
                     
                        
                             if(ids[i]==0)
                    {
                        draws="ly";
                         
                        // fprintf( stderr, "(%s,\n",draws.c_str());
                    }
                      else if(ids[i]==1)
                    {
                        draws="Davi";
                         
                        // fprintf( stderr, "(%s,\n",draws.c_str());
                    }
                    else if(ids[i]==2)
                    {
                        draws="Jerry";
                         
                        // fprintf( stderr, "(%s,\n",draws.c_str());
                    }
                    else
                    {
                        draws="unknow";
                         
                    }
                   
                    
                    if (!draws.empty()) {
                          cv::putText(prevFrame1, draws, cv::Point(RectRange.x, RectRange.y),  cv::FONT_HERSHEY_PLAIN, 3, Scalar(255,255,0), 2);
                    
          
                    }
                    fprintf( stderr, "(%d,\n",id1111 );

                      




                     if (face->isLandmarksEnabled()&&(ids[i]==id1111)) 
                    //  if (face->isLandmarksEnabled()) 
                    {
                        
                        auto& normed_landmarks = face->getLandmarks();
                        size_t n_lm = normed_landmarks.size();
                        int x1[4];
                        int y1[4];
                        int num=0;
                        for (size_t i_lm = 8; i_lm < 12; ++i_lm) {
                            float normed_x = normed_landmarks[2 * i_lm];
                            float normed_y = normed_landmarks[2 * i_lm + 1];

                            int x_lm = face->_location.x + static_cast<int>(face->_location.width * normed_x);
                            int y_lm = face->_location.y + static_cast<int>(face->_location.height * normed_y);
                            x1[num]=x_lm;
                            y1[num]=y_lm;
                            num++;

                            cv::circle(prevFrame1, cv::Point(x_lm, y_lm), 1 + static_cast<int>(0.012 * face->_location.width), cv::Scalar(0, 255, 255), -1);
                        }
                        
                        int center_x= (x1[0]+x1[1])/2;
                        int center_y= (y1[3]+y1[2])/2;
                        int dst_x=x1[1]-x1[0];
                        int dst_y=y1[3]-y1[2];
                        int flag=0;
                        int w = x1[1]-x1[0];
                        int h = y1[3]-y1[2];
                         
                        int x111=dst_x*1.5;
                        int y111=dst_x*1.5;
                        if(x1[0]*0.95+x111>prevFrame2.cols){x111=prevFrame2.cols-x1[0]*0.95;}
                        if(y1[2]*0.95+y111>prevFrame2.rows){y111=prevFrame2.rows-y1[2]*0.95;}

                         if ((float)dst_y/(float)dst_x>0.78)
                        {
                            flag=1;
                        }
                        if ((float)dst_y/(float)dst_x<0.78)
                        {
                            flag=0;
                        }

                        if(flag==1)
                        {
                            mouth = prevFrame2(Rect(x1[0]*0.95,y1[2]*0.95,x111,y111));////
                        // mouth = prevFrame2;
                         fprintf( stderr, "(%0.2f,%0.2f",x111+x1[0]*0.95,y111+y1[2]*0.95);
 
                        cv::resize(mouth, mouth, Size(416,416), 0, 0, cv::INTER_LINEAR);
                        // imgBrightness(mouth,1,mouth);

                        cv::Mat pr_img = static_resize(mouth);
                        Blob::Ptr imgBlob = infer_request.GetBlob(input_name);     // just wrap Mat data by Blob::Ptr
                        blobFromImage(pr_img, imgBlob);
                        infer_request.Infer();
                        const Blob::Ptr output_blob = infer_request.GetBlob(output_name);
                        MemoryBlob::CPtr moutput = as<MemoryBlob>(output_blob);
                        if (!moutput) {
                            throw std::logic_error("We expect output to be inherited from MemoryBlob, "
                                                "but by fact we were not able to cast output to MemoryBlob");
                        }
                        // locked memory holder should be alive all time while access to its buffer
                        // happens
                        auto moutputHolder = moutput->rmap();
                        const float* net_pred = moutputHolder.as<const PrecisionTrait<Precision::FP32>::value_type*>();
                        
                        int img_w = mouth.cols;
                        int img_h = mouth.rows;
                        float scale = std::min(INPUT_W / (mouth.cols*1.0), INPUT_H / (mouth.rows*1.0));
                        std::vector<Object> objects;

                        decode_outputs(net_pred, objects, scale, img_w, img_h);
                        draw_objects(mouth, objects);
                         if(objects.size() !=0)
                        {
                             cv::circle(prevFrame1, cv::Point(x1[0]*0.95+(objects[0].rect.x+objects[0].rect.width/2)*(dst_x+30)/416, y1[0]*0.95+(objects[0].rect.y/4+objects[0].rect.height/2)*(dst_x+30)/416), 1 + static_cast<int>(0.012 * face->_location.width), cv::Scalar(255, 0, 0), -1);
                            //  cv::circle(prevFrame1, cv::Point(x1[0]*0.95, y1[0]*0.95), 1 + static_cast<int>(0.012 * face->_location.width), cv::Scalar(255, 0, 0), -1);

                        }




                        }
                        




                        // cv::circle(prevFrame1, cv::Point(center_x, center_y), 1 + static_cast<int>(0.012 * face->_location.width), cv::Scalar(255, 0, 0), -1);
                        float distance_sum = 0;
                        int   effective_pixel = 0;
                         for(int y = center_y-h/8;y < center_y+h/8;y++)
                        {
                            for(int x = center_x-w/8;x < center_x+w/8;x++)
                            {
                                //不是0就有位置信息
                                if(depthImage.at<uint16_t>(y,x))//出现位置信息
                                {
                                    distance_sum += depthImage.at<uint16_t>(y,x);
                                    effective_pixel++;
                                }
                            }
                        }

                        Eigen::Quaterniond q=euler2Quaternion(roll, pitch, yaw);
                         cv::circle(prevFrame1, cv::Point(static_cast<float>(face->_location.x + face->_location.width / 2),
                                    static_cast<float>(face->_location.y + face->_location.height / 2)), 1 + static_cast<int>(0.012 * face->_location.width), cv::Scalar(0, 255, 255), -1);
                        

                        float effective_distance = distance_sum/effective_pixel;
                        float real_z = 0.001 * effective_distance;
                        float real_x =(static_cast<float>(face->_location.x) - camera_info.K.at( 2 ) ) / camera_info.K.at( 0 ) * real_z;
                        float real_y =(static_cast<float>(face->_location.y) - camera_info.K.at( 5 ) ) / camera_info.K.at( 4 ) * real_z;

                        float distance_sum_c = 0;
                        int   effective_pixel_c = 0;
                        int center_x1=static_cast<float>(face->_location.x + face->_location.width / 2);
                        int center_y1=static_cast<float>(face->_location.y + face->_location.height / 2);
                        int h1= static_cast<float>(face->_location.height);
                        int w1= static_cast<float>( face->_location.width);
                         for(int y = center_y1-h1/8;y < center_y1+h1/8;y++)
                        {
                            for(int x = center_x1-w1/8;x < center_x1+w1/8;x++)
                            {
                                //不是0就有位置信息
                                if(depthImage.at<uint16_t>(y,x))//出现位置信息
                                {
                                    distance_sum_c += depthImage.at<uint16_t>(y,x);
                                    effective_pixel_c++;
                                }
                            }
                        }
                         float effective_distance_c = distance_sum_c/effective_pixel_c;
                        float real_zc = 0.001 * effective_distance_c;
                        float real_xc =(static_cast<float>(face->_location.x) - camera_info.K.at( 2 ) ) / camera_info.K.at( 0 ) * real_z;
                        float real_yc =(static_cast<float>(face->_location.y) - camera_info.K.at( 5 ) ) / camera_info.K.at( 4 ) * real_z;
                        //人脸跟随的话题
                        ps.header.frame_id = "camera_color_optical_frame";
                        ps.header.stamp = ros::Time(0);
                        ps.pose.position.x = real_xc;
                        ps.pose.position.y = real_yc;
                        ps.pose.position.z = real_zc;

                        ps.pose.orientation.x = q.x();
                        ps.pose.orientation.y = q.y();
                        ps.pose.orientation.z = q.z();
                        ps.pose.orientation.w = q.w();

                        pub.publish(ps);
                        

                        

                       
                        //核酸检测的话题
                        count ++;
                        if (count >= 15)
                        {
                            count = 0;
                            ps.pose.position.x = real_x;
                            ps.pose.position.y = real_y;
                            ps.pose.position.z = real_z;
                            pub2.publish(ps);
                            if(flag == 1)
                                mouth_judge.data = true;
                            else  mouth_judge.data = false;
                            pub3.publish(mouth_judge);
                            
                        }

                      

                        // fprintf( stderr, "(%0.2f,%0.2f,%0.2f),(%0.2f,%0.2f,%0.2f),(%0.2f,%0.2f,%0.2f,%0.2f),(%0.2f,%0.2f,%0.2f)\n",real_x, real_y, real_z ,yaw,pitch,roll,q.x(),q.y(),q.z(),q.w(),real_xc, real_yc, real_zc );
                        //  fprintf( stderr, "(%d,%d,%d\n",dst_x,dst_y,flag);
                        // char distance_str[300];
                        // sprintf(distance_str,"the center of the mouth:%0.2fm",real_z);
                        // cv::putText(prevFrame, distance_str, cv::Point(0,30), cv::FONT_HERSHEY_PLAIN, 1, Scalar(0,255,0), 1);
                    }
                     i++;

                
            }

        // drawing faces
        // visualizer.draw(prevFrame, faces);
 

        timer.finish("total");

        videoWriter.write(prevFrame1);
        // videoWriter.write(depthImage);

        int delay = std::max(1, static_cast<int>(msrate - timer["total"].getLastCallDuration()));
        std::chrono::time_point<std::chrono::steady_clock> end = std::chrono::steady_clock::now();

    std::chrono::duration<double> elapsed = end - start;
     

 
        char distance_str[300];
        sprintf(distance_str,"FPS:%.2f",1/elapsed.count());
        cv::putText(prevFrame1, distance_str, cv::Point(0,30), cv::FONT_HERSHEY_PLAIN, 2, Scalar(255,0,0), 2);
        if (FLAGS_show) {
            cv::imshow("Detection results",prevFrame1);
             
            // cv::imshow("Detection results", depthImage);
            int key = cv::waitKey(delay);
            if ('P' == key || 'p' == key || '0' == key || ' ' == key) {
                key = cv::waitKey(0);
            }
            if (27 == key || 'Q' == key || 'q' == key) {
                break;
            }
            presenter.handleKey(key);
        }
    }

      




    }
 };






int main(int argc, char *argv[]) {
    ros::init( argc, argv, "yolox_node" );
	ImageConverter imageconverter;
	ros::spin();
}