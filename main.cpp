#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include <vector>
#include <opencv2/imgproc.hpp>

using namespace cv;
using namespace std;

std::vector<cv::Point2f> detectRedLightBarEndpoints(const cv::Mat& image) {
    std::vector<cv::Point2f> endpoints;
    cv::Mat hsv, mask1 ,mask2, mask;
    cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv, cv::Scalar(0, 100, 100), cv::Scalar(30, 255, 255), mask1);
    cv::inRange(hsv, cv::Scalar(150, 100, 100), cv::Scalar(180, 255, 255), mask2);
    cv::Mat redmask = mask1 | mask2;
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::morphologyEx(redmask, mask, cv::MORPH_OPEN, kernel);
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    for (auto& contour : contours) {
        if (cv::contourArea(contour) < 50) continue;
        cv::RotatedRect rect = cv::minAreaRect(contour);
        float w = rect.size.width, h = rect.size.height;
        if (w > h) { std::swap(w, h); rect.angle += 90; }
        if (h/w < 2.0 || h < 20) continue;
        float angle = rect.angle * CV_PI / 180;
        float half = h / 2;
        endpoints.push_back(cv::Point2f(
            rect.center.x + half * sin(angle),
            rect.center.y + half * cos(angle)
        ));
        endpoints.push_back(cv::Point2f(
            rect.center.x - half * sin(angle),
            rect.center.y - half * cos(angle)
        ));
    }
    return endpoints;
}

void solveArmorPose(const std::vector<cv::Point2f>& imagePoints, cv::Mat& rvec, cv::Mat& tvec, const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs) {
    std::vector<cv::Point3f> objectPoints;
    float armor_width = 0.14f;
    float armor_height = 0.06f; 

    objectPoints.push_back(cv::Point3f(-armor_width/2, -armor_height/2, 0));
    objectPoints.push_back(cv::Point3f(armor_width/2, -armor_height/2, 0));
    objectPoints.push_back(cv::Point3f(armor_width/2, armor_height/2, 0));
    objectPoints.push_back(cv::Point3f(-armor_width/2, armor_height/2, 0));
    
    cv::solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec, false, cv::SOLVEPNP_EPNP);
}

void drawCoordinateAxes(cv::Mat& image, const cv::Mat& cameraMatrix, 
                       const cv::Mat& distCoeffs, const cv::Mat& rvec, 
                       const cv::Mat& tvec, float length = 0.05f) {
    std::vector<cv::Point3f> axisPoints;
    axisPoints.push_back(cv::Point3f(0, 0, 0));
    axisPoints.push_back(cv::Point3f(length, 0, 0));
    axisPoints.push_back(cv::Point3f(0, length, 0));
    axisPoints.push_back(cv::Point3f(0, 0, length));
    
    std::vector<cv::Point2f> imagePoints;
    cv::projectPoints(axisPoints, rvec, tvec, cameraMatrix, distCoeffs, imagePoints);
    
    cv::line(image, imagePoints[0], imagePoints[1], cv::Scalar(0, 0, 200), 2);
    cv::line(image, imagePoints[0], imagePoints[2], cv::Scalar(0, 200, 0), 2);
    cv::line(image, imagePoints[0], imagePoints[3], cv::Scalar(200, 0, 0), 2);
}
cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 9.28130989e+02, 0, 3.77572945e+02, 0, 9.30138391e+02, 2.83892859e+02, 0, 0, 1.0000);
cv::Mat distCoeffs = (cv::Mat_<double>(1, 5) << -2.54433647e-01, 5.69431382e-01, 3.65405229e-03, -1.09433818e-03, -1.33846840e+00);

int main(int argc, char** argv) {
    if (argc != 2) {
        cout << "用法: " << argv[0] << " <视频文件路径>" << endl;
        return -1;
    }
    string videoPath = argv[1];
    VideoCapture input(videoPath);

    std::string outputVideoPath = "output.avi";
    int fourcc = cv::VideoWriter::fourcc('M', 'J', 'P', 'G'); 
    double frame_width = input.get(cv::CAP_PROP_FRAME_WIDTH);
    double frame_height = input.get(cv::CAP_PROP_FRAME_HEIGHT);
    double fps = input.get(cv::CAP_PROP_FPS); 
    cv::Size frameSize(frame_width, frame_height); 
    cv::VideoWriter videoWriter(outputVideoPath, fourcc, fps, frameSize);
    cv::Mat frame;
    int frameCount = 0;
    while (true) {
        input >> frame;
        if (frame.empty()) break;
        // cv::Mat gray,binary;
        // cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        // cv::threshold(gray, binary, 0, 255, cv::THRESH_BINARY_INV | cv::THRESH_OTSU);
        // cv::Mat binary_bgr;
        // cv::cvtColor(binary, binary_bgr, cv::COLOR_GRAY2BGR);
        // videoWriter << binary_bgr; 
        std::vector<cv::Point2f> endpoints = detectRedLightBarEndpoints(frame);
        for (const auto& point : endpoints) {
            cv::circle(frame, point, 3, cv::Scalar(0, 255, 0), -1);
        }
        for (int i=0; i<endpoints.size(); i+=2){
            for (int j=i+2; j<endpoints.size(); j+=2){
                cv::Point2f vec1 = endpoints[i] - endpoints[i+1];
                cv::Point2f vec2 = endpoints[j] - endpoints[j+1];
                float dot_product = vec1.x * vec2.x + vec1.y * vec2.y;
                float norm1 = sqrt(vec1.x * vec1.x + vec1.y * vec1.y);
                float norm2 = sqrt(vec2.x * vec2.x + vec2.y * vec2.y);
                // if(int((endpoints[i].x - endpoints[i+1].x) * (endpoints[j].y - endpoints[j+1].y)) == int((endpoints[i].x - endpoints[i+1].x) * (endpoints[j].y - endpoints[j+1].y))){
                //     std::vector<cv::Point> endpoints_int = {
                //     cv::Point(int(endpoints[i].x + 0.5), int(endpoints[i].y + 0.5)), 
                //     cv::Point(int(endpoints[j].x + 0.5), int(endpoints[j].y + 0.5)),   
                //     cv::Point(int(endpoints[j+1].x + 0.5), int(endpoints[j+1].y + 0.5)),
                //     cv::Point(int(endpoints[i+1].x + 0.5), int(endpoints[i+1].y + 0.5)) 
                    // cv::Point(100, 200), 
                    // cv::Point(100, 300),   
                    // cv::Point(200, 300),
                    // cv::Point(200, 200) 
                // };
                if (norm1 > 0 && norm2 > 0) {
                    float cos_angle = dot_product / (norm1 * norm2);
                    if (fabs(cos_angle) > 0.9) { 
                        std::vector<cv::Point> endpoints_int = {
                            cv::Point(int(endpoints[i].x + 0.5), int(endpoints[i].y + 0.5)), 
                            cv::Point(int(endpoints[j].x + 0.5), int(endpoints[j].y + 0.5)),   
                            cv::Point(int(endpoints[j+1].x + 0.5), int(endpoints[j+1].y + 0.5)),
                            cv::Point(int(endpoints[i+1].x + 0.5), int(endpoints[i+1].y + 0.5)) 
                        };
                        cv::polylines(frame, endpoints_int, true, cv::Scalar(255, 0, 0), 1);
                        std::vector<cv::Point2f> armorPoints = {
                            endpoints[i],    
                            endpoints[j],    
                            endpoints[j+1], 
                            endpoints[i+1]  
                        };
                        cv::Mat rvec, tvec;
                        solveArmorPose(armorPoints, rvec, tvec, cameraMatrix, distCoeffs);
                        drawCoordinateAxes(frame, cameraMatrix, distCoeffs, rvec, tvec, 0.05f);
                    }
                }
            }
        } 
        // cv::Mat hsv, mask1 ,mask2, mask;
        // cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
        // cv::inRange(hsv, cv::Scalar(0, 100, 100), cv::Scalar(30, 255, 255), mask1);
        // cv::inRange(hsv, cv::Scalar(150, 100, 100), cv::Scalar(180, 255, 255), mask2);
        // cv::Mat redmask = mask1 | mask2;
        // cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
        // cv::morphologyEx(redmask, mask, cv::MORPH_OPEN, kernel);
        // cv::Mat mask_bgr;
        // cv::cvtColor(mask, mask_bgr, cv::COLOR_GRAY2BGR);
        videoWriter << frame;
    }
    videoWriter.release();
    std::cout << "视频保存完成！" << std::endl;
    return 0;
}