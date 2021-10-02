#include"stone.h"
void Stone::detect_stone(cv::Mat src,int tem_type)
{
    int flag = 0;
    cout << "-------begin-------" << endl;
    cv::Mat binary = cv::Mat::zeros(src.size(), CV_8UC1);
    cv::Mat hsv_src;
    cv::cvtColor(src, hsv_src, cv::COLOR_RGB2HSV);//turn hsv to detect color
    cv::Mat dst = cv::Mat::zeros(src.size(), src.type());
    //this->type = tem_type;//从键盘中读取识别模式
    //检测黄矿石
    if (this->type == 1) //if we detect yellow
    {
    cv::Mat mask;
    cv::inRange(hsv_src, cv::Scalar(0, 0, 90), cv::Scalar(180, 180, 245), mask);//get yellow stone mask
    //cv::imshow("mask", mask);
    //分离出黄矿石
    for (int r = 0; r < src.rows; r++)
    {
        for (int c = 0; c < src.cols; c++)
        {
            if (mask.at<uchar>(r, c) == 255)
            {
                dst.at<cv::Vec3b>(r, c) = src.at<cv::Vec3b>(r, c);
            }
        }
    }
    //cv::imshow("dst", dst);
    cv::cvtColor(dst, dst, cv::COLOR_BGR2GRAY);
    cv::threshold(dst, binary, 40, 255, cv::THRESH_BINARY_INV);//binary yellow stone
    //检测视觉标签
    flag = find_sign(binary, src);
    cv::putText(src,"to find white stone",cv::Point(src.rows / 3, src.cols / 3), 2, 1, cv::Scalar(0, 0, 255));
    if (flag == 1||flag==2)
    {
        cv::putText(src,"OK",cv::Point(src.rows / 4, src.cols / 2), 2, 1, cv::Scalar(0, 0, 255));
    }
    else
    {
        cv::putText(src,"NO",cv::Point(src.rows / 4, src.cols / 2), 2, 1, cv::Scalar(0, 0, 255));
    }
    }
    //检测白矿石
    else if (this->type == 0)
    {
        cv::Mat ROI_stone, ROI_mask;
        cv::Mat mask;
        cv::inRange(hsv_src, cv::Scalar(80, 0, 0), cv::Scalar(110, 255, 255), mask);//get white stone mask
        //cv::imshow("mask", mask);
        //分离出白色矿石
        for (int r = 0; r < src.rows; r++)
        {
            for (int c = 0; c < src.cols; c++)
            {
                if (mask.at<uchar>(r, c) == 255)
                {
                    dst.at<cv::Vec3b>(r, c) = src.at<cv::Vec3b>(r, c);
                }
            }
        }
        //cv::imshow("dst", dst);
        cv::cvtColor(dst, dst, cv::COLOR_BGR2GRAY);
        cv::threshold(dst, binary, 40, 255, cv::THRESH_BINARY);//binary yellow stone
        //cv::imshow("bin", binary);
        remove_spot(binary,dst);//去除噪声

        //找到白色矿石轮廓
        vector<vector<cv::Point>>contours;
        vector<cv::Vec4i>hierarchy;
        cv::findContours(dst.clone(), contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
        double stone_area = 0;
        int index = 0;
        for (int i = 0; i < contours.size(); i++)
        {
            double contour_area = cv::contourArea(contours[i]);
            if (contour_area > stone_area)//delete contours by area
            {
                stone_area = contour_area;
                index = i;
            }
            //cout << cv::contourArea(contours[i]) << endl;
        }
        if (stone_area > 10000)//面积阈值暂定为10000，后期需要改
        {
            this->stone_rect = cv::boundingRect(contours[index]);//get stone ROI rect
            ROI_mask = binary(this->stone_rect).clone();
            cv::drawContours(binary, contours, index, cv::Scalar(255), -1);
            ROI_stone = binary(this->stone_rect);
            cv::bitwise_xor(ROI_stone, ROI_mask, ROI_stone);//get sign of rect
            //cv::imshow("ROI_stone",ROI_stone);
            //cv::waitKey(0);
            flag = find_sign(ROI_stone, src);

        }
        else
        {
            cout << "can't find enough big area of stone" << endl;
        }
        cv::putText(src,"to find yellow stone",cv::Point(src.rows / 3, src.cols / 3), 2, 1, cv::Scalar(0, 0, 255));
        if (flag == 1||flag==2)
        {
            cv::putText(src,"OK",cv::Point(src.rows / 4, src.cols / 2), 2, 1, cv::Scalar(0, 0, 255));
        }
        else
        {
            cv::putText(src,"NO",cv::Point(src.rows / 4, src.cols / 2), 2, 1, cv::Scalar(0, 0, 255));
        }
    }

    cout << "-------stop---------" << endl;
    cv::imshow("src", src);
}

//匹配标签完成并传输对应信号  return 1:正确识别两个左右标签  2：正确识别一个左或右标签  0：default
int Stone::find_sign(cv::Mat ROI_stone, cv::Mat& src)
{
    int sign_l = 0, sign_r = 0, sign_fulldark = 0;//左标签，右标签，全黑标签个数
	vector<vector<cv::Point>>contours;
	vector<cv::Vec4i>hierarchy;
    cv::findContours(ROI_stone.clone(), contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);//get contours of sign
	for (int i = 0; i < contours.size(); i++)
	{
        if (cv::contourArea(contours[i]) < 1000)//detele fake sign by area
			continue;
		cv::Rect rects = cv::boundingRect(contours[i]);
        if (((float)rects.height / rects.width < 0.8) || ((float)rects.height / rects.width > 1.2))//标签的长宽比筛查，阈值暂定0.8~1.2
			continue;
        //cout << "area of sign == " << cv::contourArea(contours[i]) << endl;
        //判断标签类型
        switch (detect_dot(ROI_stone(rects).clone()))
		{
		case 0:
			this->sign_l = cv::Rect(rects.tl() + this->stone_rect.tl(), rects.br() + this->stone_rect.tl());
			sign_l++;
			cv::rectangle(src, rects.tl() + this->stone_rect.tl(), rects.br() + this->stone_rect.tl(), cv::Scalar(0, 0, 255), 2);
            //cv::imshow("ROI_Sign_l", ROI_stone(rects));
			//cv::waitKey(0);
			break;
		case 1:
			this->sign_r = cv::Rect(rects.tl() + this->stone_rect.tl(), rects.br() + this->stone_rect.tl());
			sign_r++;
			cv::rectangle(src, rects.tl() + this->stone_rect.tl(), rects.br() + this->stone_rect.tl(), cv::Scalar(0, 0, 255), 2);
            //cv::imshow("ROI_Sign_r", ROI_stone(rects));
			//cv::waitKey(0);
			break;
        //因为矿石不发生翻转，全黑标签在下方，识别全黑标签已删除
		//case 2:
		//	this->sign_fulldark = cv::Rect(rects.tl() + this->stone_rect.tl(), rects.br() + this->stone_rect.tl());
		//	sign_fulldark++;
		//	cv::rectangle(src, rects.tl() + this->stone_rect.tl(), rects.br() + this->stone_rect.tl(), cv::Scalar(0, 0, 255), 2);
		//	//cv::imshow("ROI_Sign", ROI_stone(rects));
		//	//cv::waitKey(0);
		//	break;
		default:
			break;
		}

        //this->sign.push_back(cv::Rect(rects.tl() + this->stone_rect.tl(), rects.br() + this->stone_rect.tl()));//push back the sign to vector

	}
    //cv::imshow("ROI", ROI_stone);
    if (sign_fulldark > 0)//矿石发生翻转的情况
	{
		int sum_sign = sign_l + sign_r + sign_fulldark;
        cout << "find dark sign" << endl;
        if ((sum_sign == 3) && (sign_fulldark == 2))//条形码面
		{
            cout << "bar code" << endl;

		}
		else if ((sum_sign == 2) && (sign_fulldark == 1))
		{
			if (sign_l == 1)
			{
                cout << "rotate 90 clockwise" << endl;
				this->sign_r = this->sign_fulldark;
				sign_r = 1;
			}
			else
			{
                cout << "rotate 180 clockwise" << endl;
				this->sign_l = this->sign_fulldark;
				sign_l = 1;
			}
		}
	}
    cout << "portion of right and left" << (float)this->sign_r.height / this->sign_l.height << endl;
    if ((((this->sign_l.br().x + this->sign_l.tl().x) / 2) < ((this->sign_r.br().x + this->sign_r.tl().x) / 2)) && (sign_l == 1) && (sign_r == 1))//识别到完整两个视觉标签
	{
		this->center_x = ((this->sign_l.br().x + this->sign_l.tl().x) / 2 + (this->sign_r.br().x + this->sign_r.tl().x) / 2) / 2;
		cv::line(src, cv::Point(this->center_x, 0), cv::Point(this->center_x, src.rows), cv::Scalar(255, 255, 0));
        int diff = this->sign_r.tl().x - this->sign_l.br().x;
		int distance_img = src.cols / 2 - this->center_x;
		this->distance = 9.0 * distance_img / diff;
        this->space = (float)9*1280/diff;//根据小孔成像原理实地测量，换相机需要更改参数!!!


        serial.serialMode(this->distance, this->space , this->type , this->receive_color);
        int type1 = this->receive_color;
        this->type = type1;
		cv::putText(src, "go left" + converttostring(this->distance) + "cm", cv::Point(src.rows / 2, src.cols / 2), 3, 1, cv::Scalar(0, 255, 255));
        cv::putText(src, "space" + converttostring(this->space) + "cm", cv::Point(src.rows / 2+30, src.cols / 2+30), 3, 1, cv::Scalar(0, 0, 255));
        //TODO:车子没有正对矿石时还需要考虑转动角使其正对
        return 1;

	}
    else if ((sign_l == 1) && (sign_r == 0))//只识别到左标签
	{
        cout << "only find left sign,go back and move right" << endl;
        this->distance = (src.cols/2-this->sign_l.br().x)/rate - 4.5;
        float rate = (this->sign_l.br().x-this->sign_l.tl().x)/3.0;
        this->space = (float)1280/rate;//根据小孔成像原理实地测量，换相机需要更改参数!!!
        serial.serialMode(this->distance, this->space , this->type , this->receive_color);
        this->type = this->receive_color;
        cv::putText(src, "go left" + converttostring(this->distance) + "cm", cv::Point(src.rows / 2, src.cols / 2), 3, 1, cv::Scalar(0, 255, 255));
        cv::putText(src, "space" + converttostring(this->space) + "cm", cv::Point(src.rows / 2+30, src.cols / 2+30), 3, 1, cv::Scalar(0, 0, 255));
        return 2;
	}
    else if ((sign_l == 0) && (sign_r == 1))//只识别到右标签
	{
        cout << "only find right sign,go back and move left" << endl;
        this->distance = (src.cols/2-this->sign_r.tl().x)/rate + 4.5;
        float rate = (this->sign_r.br().x-this->sign_r.tl().x)/3.0;
        this->space = (float)1280/rate;//根据小孔成像原理实地测量，换相机需要更改参数!!!
        serial.serialMode(this->distance, this->space , this->type , this->receive_color);
        this->type = this->receive_color;
        cv::putText(src, "go left" + converttostring(this->distance) + "cm", cv::Point(src.rows / 2, src.cols / 2), 3, 1, cv::Scalar(0, 255, 255));
        cv::putText(src, "space" + converttostring(this->space) + "cm", cv::Point(src.rows / 2+30, src.cols / 2+30), 3, 1, cv::Scalar(0, 0, 255));
        return 2;
	}
    else if (sign_l + sign_r > 2)//识别到多个左右标签
	{
        cout << "find many signs" << endl;
		if (sign_l > sign_r)
		{
            cout << "move left and go ahead" << endl;
		}
		else
		{
            cout << "move right and go ahead" << endl;
		}
	}
    else//未识别到标签
	{
        cout << "find stone,but no sign" << endl;
	}
    serial.serialMode(0, 0, this->type , this->receive_color);
    this->type = this->receive_color;
    return 0;
}


///判断标签类型   return 0:左标志   1：右标志   2：全黑标志   4：default
int Stone::detect_dot(cv::Mat ROI)
{
	int other_area = 0, sign_area = 0;
	for (int i = 0; i < ROI.rows; i++)
	{
		for (int j = 0; j < ROI.cols; j++)
		{
			if (ROI.at<uchar>(i, j) == 0)
			{
				other_area++;
			}
			else
			{
				sign_area++;
			}
		}
	}
    //cout << "portion of dark sign " << (float)sign_area / ROI.cols / ROI.rows << endl;//黑色标签在整个ROI中的占比
    if (((float)sign_area/ROI.cols/ROI.rows >0.4)&&((float)sign_area / ROI.cols / ROI.rows < 0.65))//占比阈值暂定0.4~0.65
	{
		int left_area = 0;
		int right_area = 0;
		int darkl_area = 0;
		int darkr_area = 0;
        //分别左标签与右标签白色与黑色的面积
		for (int i = 0; i < ROI.rows; i++)
		{
			for (int j = 0; j < ROI.cols; j++)
			{
				if ((i > ROI.rows / 2) && (j < ROI.cols / 2))
				{
					if (ROI.at<uchar>(i, j) == 0)
					{
						left_area++;
					}
				}
				else if ((i > ROI.rows / 2) && (j > ROI.cols / 2))
				{
					if (ROI.at<uchar>(i, j) == 0)
					{
						right_area++;
					}
				}
				if ((i < ROI.rows / 3) || (j < ROI.cols / 3))
				{
					if (ROI.at<uchar>(i, j) == 255)
					{
						darkl_area++;
					}
				}
				if ((i < ROI.rows / 3) || (j > ROI.cols * 2 / 3))
				{
					if (ROI.at<uchar>(i, j) == 255)
					{
						darkr_area++;
					}
				}
			}
		}
        //cout << "portion of right sign " << (float)darkr_area / (ROI.cols * ROI.rows * 5 / 9) << endl;
        //cout << "portion of other part of right sign" << (float)left_area * 4 / ROI.cols / ROI.rows << endl;
        //cout << "portion of left sign " << (float)darkl_area / (ROI.cols * ROI.rows * 5 / 9) << endl;
        //cout << "portion of other part of left sign" << (float)right_area * 4 / ROI.cols / ROI.rows << endl;
        if (((float)left_area * 4 / ROI.cols / ROI.rows > 0.9)&&((float)darkr_area / (ROI.cols * ROI.rows * 5 / 9)>0.7))//标签中黑白分别对应ROI的占比作为阈值，暂定0.9/0.7
		{
            //cout << "right sign" << endl;
			return 1;
        }
        else if (((float)right_area * 4 / ROI.cols / ROI.rows > 0.9) && ((float)darkl_area / (ROI.cols * ROI.rows * 5 / 9) > 0.7))//标签中黑白分别对应ROI的占比作为阈值，暂定0.9/0.7
		{
            //cout << "left sign" << endl;
			return 0;
		}
	}
	else if ((float)sign_area / ROI.cols / ROI.rows > 1)
	{
        //cout << "find full_dark" << endl;
		return 2;
	}
	return 4;
}

//去除噪声
void Stone::remove_spot(cv::Mat src, cv::Mat& dst)
{
    dst = src.clone();
    uchar *pdata = (uchar*)src.data;
    uchar *qdata = (uchar*)dst.data;
    pdata = pdata+5*src.cols+6;//从6行6列开始遍历
    qdata = qdata+5*dst.cols+6;

    for (int i = 5; i < src.rows-5; i++)
    {
        for (int j = 5; j < src.cols-5; j++)
        {
            int index_white = (*(pdata-1)+*(pdata+1)+*(pdata+1+src.cols)+*(pdata-1+src.cols)+*(pdata-1-src.cols)+*(pdata+1-src.cols)
                               +*(pdata+src.cols)+*(pdata-src.cols))/255;//计算周围8邻域的白点个数
            if (index_white>5)
            {
                *qdata = 255;
            }
            else if(index_white<2)
            {
                *qdata =  0;
            }
            pdata++;
            qdata++;
        }
        pdata = pdata+11;
        qdata = qdata+11;
    }
}
Stone::Stone()
{
	this->sign_l = cv::Rect(0, 0, 0, 0);
	this->sign_r = cv::Rect(0, 0, 0, 0);
	this->stone_rect = cv::Rect(0, 0, 0, 0);
    this->type = 1;
}

Stone::~Stone()
{
}
