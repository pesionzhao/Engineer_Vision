#pragma once
#include<opencv4/opencv2/opencv.hpp>
#include<iostream>
#include"Serial/Serial.h"
using namespace std;
enum 
{
	YELLOW, WHITE
};
class Stone
{
public:
	Stone();
	~Stone();
    void detect_stone(cv::Mat src, int type);
	void detect_stone_white(cv::Mat src);
    int find_sign(cv::Mat ROI_stone, cv::Mat& src);
	int detect_dot(cv::Mat ROI);
    void remove_spot(cv::Mat src, cv::Mat& dst);
    string converttostring(float a)//浮点数转字符串用于putText
	{
		ostringstream os;
		if (os << a)
			return os.str();
		return "invalid conversion";

    }
    //int receive_color;
private:
    cv::Rect stone_rect;//矿石Rect
    float distance; //对正矿石左右移动的距离
    float space; //与矿石之间的距离
    int type; //识别模式（黄色或者白色）
    cv::Rect sign_l;//左标志
    cv::Rect sign_r;//右标志
    cv::Rect sign_fulldark;//全黑标志
    int center_x; //矿石中心横坐标
    Serial serial; //串口
    int receive_color = 1; //接收到操作手发出的识别模式
    //float turn_ratio=0.0; 工程车旋转角度，用于对正矿石


};

