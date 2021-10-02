#include "stone.h"
#include"GxCamera/GxCamera.h"
#include<X11/Xlib.h>

using namespace cv;
using namespace std;

pthread_t thread1;
pthread_t thread2;
void* imageUpdatingThread(void* PARAM);
void* detectingThread(void* PARAM);

Mat srcFrame = Mat::zeros(480,640,CV_8UC3);
pthread_mutex_t Globalmutex;
pthread_cond_t GlobalCondCV;
bool imageReadable = false;
bool newFrame=true;
void* param;

//import Galaxy Camera
GxCamera camera;



int main(int argc, char** argv)
{
    //For MutiTHread
    //XInitThreads();
    //Init mutex
    pthread_mutex_init(&Globalmutex,NULL);
    //Init cond
    pthread_cond_init(&GlobalCondCV,NULL);
    //Create thread 1 -- image acquisition thread
    pthread_create(&thread1,NULL,imageUpdatingThread,NULL);
    //Create thread 2 -- armor Detection thread
    pthread_create(&thread2,NULL,detectingThread,NULL);
    //Wait for children thread
    pthread_join(thread1,NULL);
    pthread_join(thread2,NULL);
    pthread_mutex_destroy(&Globalmutex);
    return 0;
}


void* imageUpdatingThread(void* PARAM)
{
    //init camrea lib
    camera.initLib();

    //   open device      SN号
    camera.openDevice("KJ0190120006");
    //camera.openDevice("KE0200010113");

    //Attention:   (Width-64)%2=0; (Height-64)%2=0; X%16=0; Y%2=0;
    //   ROI             Width           Height       X       Y
    camera.setRoiParam(   640,            480,        80,     120);

    //   ExposureGain          autoExposure  autoGain  ExposureTime  AutoExposureMin  AutoExposureMax  Gain(<=16)  AutoGainMin  AutoGainMax  GrayValue
    camera.setExposureGainParam(    false,     true,      5500,          1000,              2000,         16,         15,            16,        127);

    //   WhiteBalance             Applied?       light source type
    camera.setWhiteBalanceParam(    true,    GX_AWB_LAMP_HOUSE_ADAPTIVE);

    //   Acquisition Start!
    camera.acquisitionStart(&srcFrame);
}

void* detectingThread(void* PARAM)
{
    char chKey;
    bool bRun = true;

    int s_type = 1;
    Stone stone;
    do
    {
        double t;
        double fps;
        t = (double)cv::getTickCount();

        Mat myFrame;
        //consumer gets image
        pthread_mutex_lock(&Globalmutex);
        while (!imageReadable) {
            pthread_cond_wait(&GlobalCondCV,&Globalmutex);
        }
        if(newFrame)
            srcFrame.copyTo(myFrame);
        imageReadable = false;
        pthread_mutex_unlock(&Globalmutex);
        stone.detect_stone(myFrame,s_type);//检测入口
        // from here you can use mat image as the name 'myFrame'

        //imshow("Frame",myFrame);

        t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
        fps = 1.0 / t;
        cv::Mat dst = cv::Mat::zeros(myFrame.size(),myFrame.type());
        cv::putText(dst, "FPS : " + stone.converttostring(fps),  cv::Point(myFrame.rows / 6, myFrame.cols / 6), 3, 1, cv::Scalar(0, 255, 255));
        cv::imshow("fps",dst);
        chKey = waitKey(1);//手动切换识别模式
        switch (chKey) {
        case 'w':
            s_type = 1;
            break;
        case 'y':
            s_type = 0;
            break;
        case 27:
            bRun = false;
            break;
        default:
            break;
        }

        cout << "fps = " << fps << endl;
    } while (bRun);
}

