#include <opencv2/opencv.hpp>
#include <iostream>

using namespace std;

int main() {
    cout << "OpenCV测试程序" << endl;
    cout << "OpenCV版本: " << CV_VERSION << endl;
    
    // 尝试打开视频文件
    cv::VideoCapture cap("output.avi");
    
    if (!cap.isOpened()) {
        cout << "无法打开视频文件 input_video.mp4" << endl;
        cout << "请确保文件存在且路径正确" << endl;
        return -1;
    }
    
    cout << "视频打开成功!" << endl;
    
    // 获取视频信息
    double fps = cap.get(cv::CAP_PROP_FPS);
    int total_frames = cap.get(cv::CAP_PROP_FRAME_COUNT);
    
    cout << "视频FPS: " << fps << endl;
    cout << "总帧数: " << total_frames << endl;
    
    double frame_width = cap.get(cv::CAP_PROP_FRAME_WIDTH);
    double frame_height = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
    
    std::cout << "视频帧大小: " << frame_width << " x " << frame_height << std::endl;
    cap.release();
    return 0;
}