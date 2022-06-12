// ----------------------------- OpenPose C++ API Tutorial - Example 1 - Body from image -----------------------------
// It reads an image, process it, and displays it with the pose keypoints.

// Third-party dependencies
#include <opencv2/opencv.hpp>
// Command-line user interface
#define OPENPOSE_FLAGS_DISABLE_POSE
#include <openpose/flags.hpp>
// OpenPose dependencies
#include <openpose/headers.hpp>
#include <k4a/k4a.h>
#include <k4a/k4a.hpp>
//pytorch
#include <iostream>
#include "torch/script.h"
#include "torch/torch.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
// #include "opencv2/imgcodecs.hpp"
#include <vector>
#include <chrono>
#include <string>
#include <clipArm.h>
#include <queue>

// Custom OpenPose flags
// Producer
DEFINE_string(image_path, "/home/y/Downloads/20210913/Train/Train/images/img1.jpg",
    "Process an image. Read all standard formats (jpg, png, bmp, etc.).");
// Display
DEFINE_bool(no_display,                 false,
    "Enable to disable the visual display.");

std::vector<k4a_float2_t> pixels;
//1080p
const int image_w=1920;
const int image_h=1080;


// This worker will just read and return all the jpg files in a directory
void display(const std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>>& datumsPtr)
{
    try
    {
        // User's displaying/saving/other processing here
            // datum.cvOutputData: rendered frame with pose or heatmaps
            // datum.poseKeypoints: Array<float> with the estimated pose
        if (datumsPtr != nullptr && !datumsPtr->empty())
        {
            // Display image
            const cv::Mat cvMat = OP_OP2CVCONSTMAT(datumsPtr->at(0)->cvOutputData);
            if (!cvMat.empty())
            {
                // cv::imshow(OPEN_POSE_NAME_AND_VERSION + " - Tutorial C++ API", cvMat);
                cv::imwrite("./result/dectedPic.jpg",cvMat);
                // cv::waitKey(0);
            }
            else
                op::opLog("Empty cv::Mat as output.", op::Priority::High, __LINE__, __FUNCTION__, __FILE__);
        }
        else
            op::opLog("Nullptr or empty datumsPtr found.", op::Priority::High);
    }
    catch (const std::exception& e)
    {
        op::error(e.what(), __LINE__, __FUNCTION__, __FILE__);
    }
}

void printKeypoints(const std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>>& datumsPtr)
{
    try
    {
        // Example: How to use the pose keypoints
        if (datumsPtr != nullptr && !datumsPtr->empty())
        {
            // //Alternative 1
            // op::opLog("Body keypoints: " + datumsPtr->at(0)->poseKeypoints.toString(), op::Priority::High);

            // // Alternative 2
            // op::opLog(datumsPtr->at(0)->poseKeypoints, op::Priority::High);

            // // Alternative 3
            // std::cout << datumsPtr->at(0)->poseKeypoints << std::endl;

            // Alternative 4 - Accesing each element of the keypoints
            op::opLog("\nKeypoints:", op::Priority::High);
            const auto& poseKeypoints = datumsPtr->at(0)->poseKeypoints;
            op::opLog("Person pose keypoints:", op::Priority::High);
            for (auto person = 0 ; person < poseKeypoints.getSize(0) ; person++)
            {
                op::opLog("Person " + std::to_string(person) + " (x, y, score):", op::Priority::High);
                for (auto bodyPart = 0 ; bodyPart < poseKeypoints.getSize(1) ; bodyPart++)
                {
                    std::string valueToPrint;
                    for (auto xyscore = 0 ; xyscore < poseKeypoints.getSize(2) ; xyscore++)
                        valueToPrint += std::to_string(   poseKeypoints[{person, bodyPart, xyscore}]   ) + " ";                    
                    op::opLog(valueToPrint, op::Priority::High);
                }
            }
            for (int i = 2; i < 5; i++)
            {
                k4a_float2_t p;
                p.xy.x = poseKeypoints[{0, i, 0}];
                p.xy.y = poseKeypoints[{0, i, 1}];
                pixels.push_back(p);
            }
            pixels[0].xy.x=pixels[0].xy.x/3+pixels[1].xy.x*2/3;
            pixels[0].xy.y=pixels[0].xy.y/3+pixels[1].xy.y*2/3;
            
            op::opLog(" ", op::Priority::High);
        }
        else
            op::opLog("Nullptr or empty datumsPtr found.", op::Priority::High);
    }
    catch (const std::exception& e)
    {
        op::error(e.what(), __LINE__, __FUNCTION__, __FILE__);
    }
}

int tutorialApiCpp(const cv::Mat cvImageToProcess)
{
    try
    {
        op::opLog("Starting OpenPose demo...", op::Priority::High);
        const auto opTimer = op::getTimerInit();

        // Configuring OpenPose
        op::opLog("Configuring OpenPose...", op::Priority::High);
        op::Wrapper opWrapper{op::ThreadManagerMode::Asynchronous};
        // Set to single-thread (for sequential processing and/or debugging and/or reducing latency)
        if (FLAGS_disable_multi_thread)
            opWrapper.disableMultiThreading();

        // Starting OpenPose
        op::opLog("Starting thread(s)...", op::Priority::High);
        opWrapper.start();

        // Process and display image
        // const cv::Mat cvImageToProcess = cv::imread(FLAGS_image_path);
        const op::Matrix imageToProcess = OP_CV2OPCONSTMAT(cvImageToProcess);
        auto datumProcessed = opWrapper.emplaceAndPop(imageToProcess);
        if (datumProcessed != nullptr)
        {
            printKeypoints(datumProcessed);
            if (!FLAGS_no_display)
                display(datumProcessed);
        }
        else
            op::opLog("Image could not be processed.", op::Priority::High);

        // Measuring total time
        op::printTime(opTimer, "OpenPose demo successfully finished. Total time: ", " seconds.", op::Priority::High);

        // Return
        return 0;
    }
    catch (const std::exception&)
    {
        return -1;
    }
}

float getSlope(const k4a_float2_t p1,const k4a_float2_t p2)
{
    float k;
    k=(p1.xy.y-p2.xy.y)/(p1.xy.x-p2.xy.x);
    std::cout<<"斜率："<<k<<std::endl;
    return k;
}

k4a_float2_t getP(float k,const k4a_float2_t p,float distance,int flag)
{
    k4a_float2_t pp;
    pp.xy.x=p.xy.x+distance*flag;
    pp.xy.y=k*(pp.xy.x-p.xy.x)+p.xy.y;
    return pp;
}


float getDiv(std::queue<k4a_float2_t> pixels,cv::Mat img)
{
    float div=0;
    // for (size_t i = 0; i < 3; i++)
    // {
        if (pixels.back().xy.y > 0 && pixels.back().xy.y < image_h && pixels.back().xy.x < image_w && pixels.back().xy.x > 0)
        {
            // div += img.at<cv::Vec3b>(pixels.back().xy.y, pixels.back().xy.x)[i] - img.at<cv::Vec3b>(pixels.front().xy.y, pixels.front().xy.x)[i];
            div = img.at<uint16_t>(pixels.back().xy.y, pixels.back().xy.x) - img.at<uint16_t>(pixels.front().xy.y, pixels.front().xy.x);
        
        }
        else
        {
            std::cout<<"****像素超出范围*****"<<std::endl;
            return 1000;
        }
    // }
    return div;
}

std::vector<k4a_float2_t> findPixels(std::vector<k4a_float2_t> armCenterLine,float Slope, int threshold,cv::Mat src,int d,int &length)
{
    std::vector<k4a_float2_t> segPixels;
    int plk=2;
    std::queue<float> distance;
    for (size_t i = 0; i < armCenterLine.size(); i++)
    {
        std::queue<k4a_float2_t> pixels;
        float n = 0;
        //预存一些参照像素
        for (n = 0; n < 2; n+=0.2)
        {
            k4a_float2_t p = getP(Slope, armCenterLine[i], n, d);
            pixels.push(p);
            segPixels.push_back(p);
        }
        float dis = 0;
        //预存一些像素长度
        if (i < 2)
        {
            while (getDiv(pixels, src) < threshold)
            {
                // std::cout<<"kkkkkkkkk"<<std::endl;
                k4a_float2_t p = getP(Slope, armCenterLine[i], n, d);
                dis = sqrt(pow(p.xy.x - armCenterLine[i].xy.x, 2) + pow(p.xy.y - armCenterLine[i].xy.y, 2));
                pixels.push(p);
                segPixels.push_back(p);
                pixels.pop();
                n += 0.2;

                if(i==1)
                {
                    length++;
                }
            }

            distance.push(dis);
            continue;
        }
        //分割图像
        while (getDiv(pixels, src) < threshold+plk)
        {
            k4a_float2_t p = getP(Slope, armCenterLine[i], n, d);
            dis = sqrt(pow(p.xy.x - armCenterLine[i].xy.x, 2) + pow(p.xy.y - armCenterLine[i].xy.y, 2));
            //如果长度过长则剪断
            if (distance.back() - dis > -0.1)
            {
                pixels.push(p);
                segPixels.push_back(p);
                pixels.pop();
            }
            else
            {
                break;
            }
            if (!(p.xy.y > 0 && p.xy.y < image_h && p.xy.x < image_w && p.xy.x > 0))
            {
                break;
            }
            n += 0.2;
        }
        distance.push(dis);
        distance.pop();
    }
    return segPixels;
}

std::vector<k4a_float2_t> segmentPixels(std::vector<k4a_float2_t> featurePixels,cv::Mat src,int threshold,cv::Mat img,cv::Mat depthImage)
{
    cv::Mat grad_x, grad_y;
    cv::Mat abs_grad_x, abs_grad_y, sobelDst;

    //求 X方向梯度
    // cv::Sobel(src, grad_x, CV_16S, 1, 0, 3, 1, 1, cv::BORDER_DEFAULT);
    // cv::convertScaleAbs(grad_x, abs_grad_x);
    // namedWindow("X方向Sobel", 0);
    // imshow("X方向Sobel", abs_grad_x);

    //求Y方向梯度
    // cv::Sobel(src, grad_y, CV_16S, 0, 1, 3, 1, 1, cv::BORDER_DEFAULT);
    // cv::convertScaleAbs(grad_y, abs_grad_y);
    // namedWindow("Y方向Sobel", 0);
    // imshow("Y方向Sobel", abs_grad_y);

    //合并梯度(近似)
    // cv::addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0, sobelDst);
    // namedWindow("整体方向Sobel", 0);
    // imshow("整体方向Sobel", sobelDst);
    sobelDst=src.clone();

    //根据像素值提取边缘
    //提取连线的像素
    std::vector<k4a_float2_t> armCenterLine1;
    std::vector<k4a_float2_t> armCenterLine2;
    float k1 = getSlope(featurePixels[0], featurePixels[1]);
    float k2 = getSlope(featurePixels[1], featurePixels[2]);
    for (float i = 0; i < int(featurePixels[0].xy.x - featurePixels[1].xy.x) * 4; i++)
    {
        armCenterLine1.push_back(getP(k1, featurePixels[1], i / 4, 1));
    }
    for (float i = 0; i < int(featurePixels[1].xy.x - featurePixels[2].xy.x) * 4; i++)
    {
        armCenterLine2.push_back(getP(k2, featurePixels[2], i / 4, 1));
    }

    //获取垂线的斜率
    float k11 = -1 / k1;
    float k22 = -1 / k2;
    std::vector<k4a_float2_t> segPixels;//分割后的像素

    //此处手臂摆放角度有影响
    int length[4]={0,0,0,0}; 
    std::vector<k4a_float2_t> segPixels1=findPixels(armCenterLine1,k11,threshold+15,sobelDst,1,length[0]);
    std::vector<k4a_float2_t> segPixels2=findPixels(armCenterLine1,k11,threshold+15,sobelDst,-1,length[1]);
    std::vector<k4a_float2_t> segPixels3=findPixels(armCenterLine2,k22,threshold,sobelDst,1,length[2]);
    std::vector<k4a_float2_t> segPixels4=findPixels(armCenterLine2,k22,threshold,sobelDst,-1,length[3]);

    k4a_float2_t p1=segPixels1[length[0]];
    k4a_float2_t p2=segPixels3.back();
    float k33=(p2.xy.y-p1.xy.y)/(p2.xy.x-p1.xy.x);
    float b33=p1.xy.y-k33*p1.xy.x;
    std::cout << p1.xy.x << std::endl;
    std::cout << p1.xy.y<< std::endl;
    std::cout << p2.xy.x << std::endl;
    std::cout << p2.xy.y<< std::endl;
    float boundminx=0;
    float boundmaxx=p1.xy.x;
    float boundminy=featurePixels[1].xy.y;
    float boundmaxy=0;
    if(p2.xy.x>featurePixels[1].xy.x)
    {
        boundminx=featurePixels[1].xy.x;
    }
    else
    {
        boundminx=p2.xy.x;
    }
    if(p1.xy.y<p2.xy.y)
    {
        boundmaxy=p2.xy.y;
    }
    else
    {
        boundmaxy=p1.xy.y;
    }
    std::vector<k4a_float2_t> segPixels5;
    for (float i = boundminx; i < boundmaxx; i=i+0.2)
    {
        for (float j = boundminy; j <boundmaxy ; j=j+0.2)
        {
            if(k11*i-j+featurePixels[1].xy.y-k11*featurePixels[1].xy.x<=0
            &&k22*i-j+featurePixels[1].xy.y-k22*featurePixels[1].xy.x>=0
            &&k33*i-j+p1.xy.y-k33*p1.xy.x>=0)
            {
                k4a_float2_t p;
                p.xy.x=i;
                p.xy.y=j;
                segPixels5.push_back(p);
                // std::cout<<"i:"<<i<<" j:"<<j<<std::endl;
            }
        }        
    }
    std::cout <<"segPixels5.size():"<< segPixels5.size() << std::endl;
    


    segPixels.insert(segPixels.end(),segPixels1.begin(),segPixels1.end());
    segPixels.insert(segPixels.end(),segPixels2.begin(),segPixels2.end());
    segPixels.insert(segPixels.end(),segPixels3.begin(),segPixels3.end());
    segPixels.insert(segPixels.end(),segPixels4.begin(),segPixels4.end());  
    segPixels.insert(segPixels.end(),segPixels5.begin(),segPixels5.end());    
    std::cout << segPixels.size() << std::endl;

    // std::vector<int> order;
    // for (size_t i = 0; i < segPixels.size(); i++)
    // {
    //     order.push_back(i);
    // }
    
    // std::sort(order.begin(),order.end(),[segPixels](int n1,int n2) -> bool { 
    //     if(segPixels[n1].xy.x!=segPixels[n2].xy.x) 
    //     return segPixels[n1].xy.x<segPixels[n2].xy.x; 
    //     else
    //     return segPixels[n1].xy.y<segPixels[n2].xy.y;        
    //     });
    // for (size_t i = 0; i < 10; i++)
    // {
    //     std::cout<<order[i]<<std::endl;
    //     std::cout<<segPixels[order[i]].xy.x<<" "<<segPixels[order[i]].xy.y<<std::endl;
    // }
    
    // for (size_t i = 0; i < segPixels.size(); i++)
    // {
    //     segPixels[order[i]]
    // }
    cv::Mat iim=cv::Mat::zeros(image_h,image_w,CV_8UC1);//用于删除重复点  

    for (size_t i = 0; i < segPixels.size(); i++)
    {
        if (segPixels[i].xy.y > 0 && segPixels[i].xy.y < image_h && segPixels[i].xy.x < image_w && segPixels[i].xy.x > 0)
        {
            // img.at<uint16_t>(segPixels[i].xy.y, segPixels[i].xy.x) = 255;
            img.at<cv::Vec3b>(segPixels[i].xy.y, segPixels[i].xy.x)[2] = 255;
            depthImage.at<cv::uint16_t>(segPixels[i].xy.y, segPixels[i].xy.x) = 255;
            iim.at<cv::uint8_t>(segPixels[i].xy.y, segPixels[i].xy.x)= 255;
        }
    }

    std::vector<k4a_float2_t> segPixels6;
    for (size_t i = 0; i < image_h; i++)
    {
        for (size_t j = 0; j < image_w; j++)
        {
            if(iim.at<cv::uint8_t>(i,j)>10)
            {
                k4a_float2_t p;
                p.xy.x=j;
                p.xy.y=i;
                segPixels6.push_back(p);
            }
        }
    }
    std::cout << segPixels6.size() << std::endl;    

    // cv::namedWindow("Result", 0);
    // cv::imshow("Result", sobelDst);
    cv::imwrite("./result/segmentedRGB.jpg", img);
    cv::imwrite("./result/mask.jpg", iim);
    cv::imwrite("./result/segmentedDepth.png", depthImage);
    // cv::waitKey(0);
    return segPixels6;
}

std::vector<k4a_float2_t> segment(cv::Mat image)
{
    //配置参数
    // std::string path =argv[1];
    // char path[] = "3.jpg";
    int img_size = 512;
    torch::DeviceType CUDA_type;
    CUDA_type = torch::kCUDA;
    torch::DeviceType CPU_type;
    CPU_type = torch::kCPU;
    torch::Device device(CPU_type);
    // std::cout << "cuda support:" << (torch::cuda::is_available() ? "true" : "false") << std::endl;

    // 读取图片
    // cv::Mat image = cv::imread(path);
    if (image.empty())
        fprintf(stderr, "Can not load image\n");

    cv::Mat resizedImg;
    cv::resize(image, resizedImg, cv::Size(img_size, img_size),1);
    // cv::cvtColor(resizedImg, resizedImg, cv::COLOR_BGR2RGB); // bgr->rgb
    std::vector<int64_t> dims = {1, img_size, img_size, 3};
    torch::Tensor img_var = torch::from_blob(resizedImg.data, dims, torch::kByte); //将图像转化成张量
    img_var = img_var.permute({0, 3, 1, 2}); //将张量的参数顺序转化为 torch输入的格式 1,3,512,512
    at::Tensor img_float = img_var.toType(torch::kFloat);//将kByte转化为kFloat
    img_float = img_float.div(255); //归一化到[0,1]区间
    img_float = img_float.to(device);//将数据存储到gpu中
    //加载pytorch模型
    torch::jit::script::Module module = torch::jit::load("jitter.pt");
    module.to(device);
    module.eval();
    at::Tensor result = module.forward({img_float}).toTensor();
    // at::Tensor nn = result.to(torch::kCPU).mul(100).to(torch::kU8);
    at::Tensor nn = result.mul(100).to(torch::kU8);
    at::Tensor mm = nn[0][0];
    cv::Mat PredictedImage = cv::Mat::ones(cv::Size(img_size, img_size), CV_8UC1);
    memcpy(PredictedImage.data, mm.data_ptr(), img_size * img_size * sizeof(torch::kU8));
    cv::Mat thresholdImg=PredictedImage.clone();
    for (size_t i = 0; i < 512; i++)
    {
        for (size_t j = 0; j < 512; j++)
        {
            if (thresholdImg.at<unsigned char>(i, j) > 50)
            {
                thresholdImg.at<unsigned char>(i,j)=255;
            }
            else
            {
                thresholdImg.at<unsigned char>(i, j) = 0;
            }
        }        
    }
    cv::imwrite("prediction.jpg", thresholdImg);
    // cv::waitKey(0);
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(thresholdImg, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
    double maxArea = 0;
    int m=0;
    std::vector<cv::Point> maxContour;
    for (size_t i = 0; i < contours.size(); i++)
    {
        double area = cv::contourArea(contours[i]);
        if (area > maxArea)
        {
            maxArea = area;
            maxContour = contours[i];
            m=i;
        }
    }
    cv::Rect maxRect = cv::boundingRect(maxContour);
    // 显示连通域
    cv::Mat result1;
    thresholdImg.copyTo(result1);
    cv::rectangle(result1, maxRect, cv::Scalar(255));
    // cv::imshow("largest region", result1);
    // cv::waitKey();

    cv::Mat Seg_image = cv::Mat::zeros(cv::Size(512, 512), CV_8UC1);
    drawContours(Seg_image,contours,m, cv::Scalar(255),cv::FILLED);

    // cv::imshow("Result", Seg_image);
    // cv::waitKey();

    for (size_t i = 0; i < img_size; i++)
    {
        for (size_t j = 0; j < img_size; j++)
        {
            if (Seg_image.at<unsigned char>(i, j) > 50)
            {
                // image4.at<unsigned char>(i,j)=255;
                resizedImg.at<cv::Vec3b>(i, j)[0] = 255;
                // resizedImg.at<cv::Vec3b>(i, j)[1] = 0;
                // resizedImg.at<cv::Vec3b>(i, j)[2] = 0;
            }
        }
    }
    cv::Mat Result_image;
    cv::resize(Seg_image,Result_image,cv::Size(1280,720),1);
    std::vector<k4a_float2_t> segedPixels;

    cv::Mat ii_image = cv::Mat::zeros(cv::Size(1280, 720), CV_8UC1);
    // std::ofstream infile("arm.txt");
	// assert(infile.is_open());
    // std::cout<<"writing"<<std::endl;
    for (size_t i = 0; i < 720; i++)
    {
        for (size_t j = 0; j < 1280; j++)
        {
            if(Result_image.at<unsigned char>(i, j) >200)
            {
                k4a_float2_t pixel;
                pixel.xy.x=j;
                pixel.xy.y=i;
                segedPixels.push_back(pixel);
                // ii_image.at<unsigned char>(i,j)=255;
                // infile<<pixel.xy.x<<" "<<pixel.xy.y<<std::endl;
            }
        }        
    }
    // std::cout<<"writing"<<std::endl;
    // cv::imwrite("./Result_co.jpg", ii_image);

    cv::imwrite("./Result_image.jpg", Result_image);

    cv::resize(resizedImg,resizedImg,cv::Size(1280,720),1);
    cv::imwrite("./Result_colorimage.jpg", resizedImg);
    // cv::imshow("prediction", resizedImg);
    // cv::waitKey(0);
    // cv::imshow("prediction", Result_image);
    // cv::waitKey(0);
    std::cout<<"successful"<<std::endl;
    // return Result_image;
    return segedPixels;
}

int work()
{
    // Parsing command line flags
    // gflags::ParseCommandLineFlags(&argc, &argv, true);
    // gflags::ParseCommandLineFlags(&argc, &argv, true);

    //找到并打开设备
    uint32_t count = k4a_device_get_installed_count();
    if (count == 0)
    {
        std::cout << "No k4a devices attached!\n";
    }

    k4a_device_t device=NULL;
    // Open the first plugged in Kinect device 打开设备
    if (K4A_FAILED(k4a_device_open(K4A_DEVICE_DEFAULT, &device)))
    {
        std::cout << "Failed to open k4a device!\n";
    }

    // Get the size of the serial number 获取设备序列号
    size_t serial_size = 0;
    k4a_device_get_serialnum(device, NULL, &serial_size);

    // Allocate memory for the serial, then acquire it
    char *serial = (char *)(malloc(serial_size));
    k4a_device_get_serialnum(device, serial, &serial_size);
    std::cout << "Opened device: %s\n";
    std::cout << serial << std::endl;
    free(serial);

    // Configure a stream of 720P BRGA color data at 30 frames per second 设置参数
    k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
    config.color_resolution = K4A_COLOR_RESOLUTION_1080P;
    config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
    config.camera_fps = K4A_FRAMES_PER_SECOND_30;
    config.synchronized_images_only = true;

    k4a_calibration_t calibration;
    if (K4A_RESULT_SUCCEEDED !=
        k4a_device_get_calibration(device, config.depth_mode, config.color_resolution, &calibration))
    {
        std::cout << "Failed to get calibration\n";
    }
    k4a_transformation_t transformation = k4a_transformation_create(&calibration);

    // Start the camera with the given configuration
    if (K4A_FAILED(k4a_device_start_cameras(device, &config)))
    {
        std::cout << "Failed to start cameras!\n";
        k4a_device_close(device);
    }

    k4a_capture_t capture=NULL;
	const int32_t TIMEOUT_IN_MS = 10000;
    switch (k4a_device_get_capture(device, &capture, TIMEOUT_IN_MS))
	{
	case K4A_WAIT_RESULT_SUCCEEDED:
		break;
	case K4A_WAIT_RESULT_TIMEOUT:
		std::cout << "Timed out waiting for a capture\n";
		break;
	case K4A_WAIT_RESULT_FAILED:
		std::cout << "Failed to read a capture\n";
		break;
	}    

    k4a_image_t color_image=NULL;
    k4a_image_t depth_image=NULL;
    k4a_image_t color_depth_image=NULL;

    //获取各个类型的图像数据
    color_image=k4a_capture_get_color_image(capture);
    depth_image=k4a_capture_get_depth_image(capture);
    int color_image_width_pixels = k4a_image_get_width_pixels(color_image);
    int color_image_height_pixels = k4a_image_get_height_pixels(color_image);

    k4a_image_t point_cloud_image = NULL;
	if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
												 color_image_width_pixels,
												 color_image_height_pixels,
												 color_image_width_pixels * 3 * (int)sizeof(int16_t),
												 &point_cloud_image))
	{
		std::cout << "Failed to create point cloud image\n";
	}

    if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_DEPTH16,
                                                 color_image_width_pixels,
                                                 color_image_height_pixels,
                                                 color_image_width_pixels * (int)sizeof(uint16_t),
                                                 &color_depth_image))
    {
        printf("Failed to create transformed depth image\n");
    }

    if (color_image == 0)
    {
        printf("Failed to get color image from capture\n");
    }
	if (depth_image == 0)
	{
		std::cout << "Failed to get depth image from capture\n";
	}


    if (K4A_RESULT_SUCCEEDED != k4a_transformation_depth_image_to_color_camera(transformation, depth_image, color_depth_image))
    {
        printf("Failed to compute transformed depth image\n");
    }   


    cv::Mat cv_rgbImage_with_alpha=cv::Mat(color_image_height_pixels,color_image_width_pixels,CV_8UC4,
                        (void*)k4a_image_get_buffer(color_image),cv::Mat::AUTO_STEP);
    cv::Mat cvImageToProcess;
    cv::cvtColor(cv_rgbImage_with_alpha, cvImageToProcess, cv::COLOR_BGRA2BGR);
    cv::imwrite("./result/origin_rgbpic.jpg",cvImageToProcess);//存储彩色图像
    // cv::Mat segmentedImg=segment(cvImageToProcess);
    // std::vector<k4a_float2_t> segedpixels=segment(cvImageToProcess);

    // cv::Mat cv_rgbImage_wi=cv::Mat(color_image_height_pixels,color_image_width_pixels,CV_16UC1,
    //                     (void*)k4a_image_get_buffer(color_depth_image),cv::Mat::AUTO_STEP);
    // cv::imshow("1",cv_rgbImage_wi);cv::waitKey(0);
    
    //将深度图像变换到彩色图像空间中
    if (K4A_RESULT_SUCCEEDED != k4a_transformation_depth_image_to_point_cloud(transformation,
                                                                              color_depth_image,
                                                                              K4A_CALIBRATION_TYPE_COLOR,
                                                                              point_cloud_image))
    {
        printf("Failed to compute point cloud\n");
        return false;
    }

    struct color_point_t
	{
		int16_t xyz[3];
		uint8_t rgb[3];
	};

	std::vector<color_point_t> points;
	int16_t *point_cloud_image_data = (int16_t *)(void *)k4a_image_get_buffer(point_cloud_image);
	uint8_t *color_image_data = k4a_image_get_buffer(color_image);
    //存储点云
	for (int i = 0; i < color_image_width_pixels * color_image_height_pixels; i++)
	{
		color_point_t point;
		point.xyz[0] = point_cloud_image_data[3 * i + 0];
		point.xyz[1] = point_cloud_image_data[3 * i + 1];
		point.xyz[2] = point_cloud_image_data[3 * i + 2];
		if (point.xyz[2] == 0)
		{
			continue;
		}

		point.rgb[0] = color_image_data[4 * i + 0];
		point.rgb[1] = color_image_data[4 * i + 1];
		point.rgb[2] = color_image_data[4 * i + 2];
		uint8_t alpha = color_image_data[4 * i + 3];

		if (point.rgb[0] == 0 && point.rgb[1] == 0 && point.rgb[2] == 0 && alpha == 0)
		{
			continue;
		}

		points.push_back(point);
	}
    
    //写入不带颜色的点云
    std::ofstream infile1("./result/xyz.txt");
	assert(infile1.is_open());
    for (int i = 0; i < points.size(); i++)
    {
        infile1 << points[i].xyz[0];
        infile1 << " ";
        infile1 << points[i].xyz[1];
        infile1 << " ";
        infile1 << points[i].xyz[2];        
        infile1 << std::endl;
    }

    std::ofstream infile2("./result/xyzrgb.txt");
	assert(infile2.is_open());

    //写入带颜色的点云
    for (int i = 0; i < points.size(); i++)
    {
        infile2 << points[i].xyz[0];
        infile2 << " ";
        infile2 << points[i].xyz[1];
        infile2 << " ";
        infile2 << points[i].xyz[2];
        infile2 << " ";
        infile2 << (float)points[i].rgb[2];
        infile2 << " ";
        infile2 << (float)points[i].rgb[1];
        infile2 << " ";
        infile2 << (float)points[i].rgb[0];
        infile2 << std::endl;
    }
    
    // cv::imshow("1",cvImageToProcess);cv::waitKey(0);
    std::cout<<"-----开始检测人体姿态------"<<std::endl;
    tutorialApiCpp(cvImageToProcess);//检测人体姿态

    k4a_float3_t ppp;
    int valid;
    int16_t *depth_image_data = (int16_t *)(void *)k4a_image_get_buffer(color_depth_image);


    float k1=-1/getSlope(pixels[0],pixels[1]);//大臂连线的垂线斜率
    float k2=-1/getSlope(pixels[1],pixels[2]);//小臂连线的垂线斜率
    //存储垂线上的像素点
    std::vector<k4a_float2_t> clipPixels;
    for (size_t i = 0; i < 16; i++)
    {
        clipPixels.push_back(getP(k1, pixels[0], i, -1));
        clipPixels.push_back(getP(k1, pixels[0], i, 1));
    }
    for (size_t i = 0; i < 16; i++)
    {
        clipPixels.push_back(getP(k1, pixels[1], i, -1));
        clipPixels.push_back(getP(k1, pixels[1], i, 1));
    }
    for (size_t i = 0; i < 16; i++)
    {
        clipPixels.push_back(getP(k2, pixels[1], i, -1));
        clipPixels.push_back(getP(k2, pixels[1], i, 1));
    }
    for (size_t i = 0; i < 16; i++)
    {
        clipPixels.push_back(getP(k2, pixels[2], i, -1));
        clipPixels.push_back(getP(k2, pixels[2], i, 1));
    }

    //将深度图存入cv::mat,用以分割
    cv::Mat cv_depthimage=cv::Mat(color_image_height_pixels,color_image_width_pixels,CV_16UC1,
                        (void*)k4a_image_get_buffer(color_depth_image),cv::Mat::AUTO_STEP);



    cv::Mat cv_depthMat1=cv_depthimage.clone();//用于分割
    cv::Mat cv_depthMat2=cv_depthimage.clone();//用于存储
    // cv::Mat cv_depthMat3=cv::Mat::zeros(color_image_height_pixels,color_image_width_pixels,CV_32FC1);//用于
    //查找最大深度
    // int depth=0;
    // int a=0;
    // for (int i = 0; i < color_image_height_pixels; i++)
    // {
    //     for (int j = 0; j < color_image_width_pixels; j++)
    //     {
    //         // std::cout<<cv_depthimage.ptr<uint16_t>(i)[j]<<std::endl;
    //         // cv_depthMat3.ptr<float>(i)[j]=cv_depthMat2.ptr<uint16_t>(i)[j];
    //         a=cv_depthMat2.ptr<uint16_t>(i)[j];
    //         if(a>depth)
    //         depth=a;
    //     }
    // }
    // depth/=255;
    for (int i = 0; i < color_image_height_pixels; i++)
    {
        for (int j = 0; j < color_image_width_pixels; j++)
        {            
            float p=cv_depthMat2.ptr<uint16_t>(i)[j];
            // p/=depth;
            // p=pow(p,2);
            p*=4;
            // p*=255;
            cv_depthMat2.ptr<uint16_t>(i)[j]=p;
            cv_depthMat1.ptr<uint16_t>(i)[j]=p*8;
            // std::cout<<cv_depthMat3.ptr<float>(i)[j]<<std::endl;
        }
    }
    // std::cout<<"depth:"<<depth<<std::endl;
    cv::imwrite("./result/depth.png",cv_depthMat1);
    // cv::imshow("depth",cv_depthimage);
    // cv::waitKey(0);

    //存储关节特征的三维点
    std::ofstream infile3("./result/featureXYZ.txt");
    assert(infile3.is_open());
    for (size_t i = 0; i < 3; i++)
    {
        float depth = depth_image_data[color_image_width_pixels * (int)pixels[i].xy.y + (int)pixels[i].xy.x];
        k4a_calibration_2d_to_3d(&calibration, &pixels[i], depth, K4A_CALIBRATION_TYPE_COLOR, K4A_CALIBRATION_TYPE_COLOR, &ppp, &valid);
        std::cout << pixels[i].xy.x << "###" << pixels[i].xy.y << std::endl;
        std::cout << ppp.xyz.x << "&&&" << ppp.xyz.y << "&&&" << ppp.xyz.z << std::endl;
        infile3 << ppp.xyz.x << " " << ppp.xyz.y << " " << ppp.xyz.z << std::endl;
    }

    //存储垂线的三维点
    std::ofstream infile4("./result/clipXYZ.txt");
    assert(infile4.is_open());
    for (size_t i = 0; i < clipPixels.size(); i++)
    {
        float depth = depth_image_data[color_image_width_pixels * (int)clipPixels[i].xy.y + (int)clipPixels[i].xy.x];
        k4a_calibration_2d_to_3d(&calibration, &clipPixels[i], depth, K4A_CALIBRATION_TYPE_COLOR, K4A_CALIBRATION_TYPE_COLOR, &ppp, &valid);
        infile4 << ppp.xyz.x << " " << ppp.xyz.y << " " << ppp.xyz.z << std::endl;
    }

    //从深度图里分割手臂
    std::vector<k4a_float2_t> segedpixels=segmentPixels(pixels,cv_depthMat2,20,cvImageToProcess,cv_depthMat1);
    // std::vector<k4a_float2_t> segedpixels=segmentPixels(pixels,cvImageToProcess,80,cvImageToProcess);

    //存储手臂的点云
    std::ofstream infile5("./result/segedArm.txt");
    assert(infile5.is_open());
    for (size_t i = 0; i < segedpixels.size(); i++)
    {
        float depth = depth_image_data[color_image_width_pixels * (int)segedpixels[i].xy.y + (int)segedpixels[i].xy.x];
        k4a_calibration_2d_to_3d(&calibration, &segedpixels[i], depth, K4A_CALIBRATION_TYPE_COLOR, K4A_CALIBRATION_TYPE_COLOR, &ppp, &valid);
        infile5 << ppp.xyz.x << " " << ppp.xyz.y << " " << ppp.xyz.z << std::endl;
    }

    //给手臂分段
    clip();

    // Running tutorialApiCpp
    // return tutorialApiCpp();
    return 0;
}
