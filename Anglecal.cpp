
#include <opencv.hpp>
#include <imgproc.hpp>
#include <vector>
#include <iostream>
#include <cmath>
#include <stdio.h>
#include <cv.h>
#include <highgui.h>
#include <time.h>
#include <windows.h>
#define PI 3.1415926
#define angle(x) (PI*(x)/180.0

using namespace cv;
using std::vector;
using namespace std;

float X, Y=0;
//Ԥ�趨λ�˵Ķ��˰뾶
float pole_radius = 0.23;
//Ԥ��LED�Ƹ߶ȺͶ�λ�˸߶�
float LED_height = 33.600;
float pole_height = 2.00;
//����ԭͼԲ��λ��
int Fx = 787;//748;
int Fy = 1025;
//����ROIͼԲ��λ��
int fx = 200;
int fy = 200;
double ShadowAngle;

const int alpha_slider_max = 100;
const int beta_slider_max = 100;
int alpha_slider,beta_slider;

int arroundgray(int,int,Mat);

int show1 = 0;
int manu1 = 0;
int auto1 = 0;
int parameter1 = 0;


Mat src1;
Mat src2;
Mat dst;
Mat absdiff_pic,absdiff_grey;
Mat test1,test2;



void   Delay(int   time)//time*1000Ϊ����
{
    clock_t   now   =   clock();

    while(   clock()   -   now   <   time   );
}

float PXtoCM(float SL)
{
    float k = 290.0, L_cm = 0;
    L_cm = SL / k;
    return L_cm;
}

void ShowPicture(char name[],Mat pic)
{
    namedWindow(name, WINDOW_NORMAL);
    imshow(name, pic);
}

float calPole_distance(double shadowDis)
{
    float poleDis;
    poleDis=((LED_height-pole_height)/pole_height)*shadowDis;
    return poleDis;
}

float anglework(Vec4i l)
{
    cout<<"l1 point location  "<<l[0]<<","<<l[1]<<"and   "<<l[2]<<","<<l[3]<<endl;
    double cosa = abs(l[0]-l[2]);
    cout<<"cosa    "<<cosa<<endl;
    double cosb = sqrt((l[0]-l[2])*(l[0]-l[2]) + (l[3]-l[1])*(l[3]-l[1]));
    cout<<"cosb    "<<cosb<<endl;
    double cosvalue = cosa*1.000/cosb*1.000;
    cout<<"cosvalue    "<<cosvalue<<endl;
    double a = acos(cosvalue);
    cout<<"a    "<<a<<endl;
    a = a*180.00/3.1415926;
    cout<<"a    "<<a<<endl;
    return a;
}

int angleex(Vec4i l,float anglecount)
{
    float dstart,dend;
    dstart = sqrt((l[0]-fx)*(l[0]-fx) + (l[1]-fy)*(l[1]-fy));
    dend = sqrt((l[2]-fx)*(l[2]-fx) + (l[3]-fy)*(l[3]-fy));
    if(dstart < dend)
    {
        //��ô��0��1���������Բ�ģ�Ϊ��ʼ��
        cout<<"(l[0],l[1])�����"<<endl;
        int x = l[2] - l[0];
        int y = l[1] - l[3];
        //�ж�����
        if(x>0&&y>0)
        {
            cout<<"Ӱ���ڵ�һ����"<<endl;
            cout<<"Ӱ�ӽǶ�Ϊ"<<anglecount<<endl;
            ShadowAngle=anglecount;
        }
        else if(x<0&&y>0)
        {
            cout<<"Ӱ���ڵڶ�����"<<endl;
            cout<<"Ӱ�ӽǶ�Ϊ"<<180.00-anglecount<<endl;
            ShadowAngle=180.00-anglecount;
        }
        else if(x<0&&y<0)
        {
            cout<<"Ӱ���ڵ�������"<<endl;
            cout<<"Ӱ�ӽǶ�Ϊ"<<180.00+anglecount<<endl;
            ShadowAngle=180.00+anglecount;
        }
        else if(x>0&&y<0)
        {
            cout<<"Ӱ���ڵ�������"<<endl;
            cout<<"Ӱ�ӽǶ�Ϊ"<<360.00-anglecount<<endl;
            ShadowAngle=360.00-anglecount;
        }
        return 1;
    }
    else
    {
        cout<<"(l[2],l[3])�����"<<endl;
        int x = l[0] - l[2];
        int y = l[3] - l[1];
        //�ж�����
        if(x>0&&y>0)
        {
            cout<<"Ӱ���ڵ�һ����"<<endl;
            cout<<"Ӱ�ӽǶ�Ϊ"<<anglecount<<endl;
            ShadowAngle=anglecount;
        }
        else if(x<0&&y>0)
        {
            cout<<"Ӱ���ڵڶ�����"<<endl;
            cout<<"Ӱ�ӽǶ�Ϊ"<<180-anglecount<<endl;
            ShadowAngle=180.00-anglecount;
        }
        else if(x<0&&y<0)
        {
            cout<<"Ӱ���ڵ�������"<<endl;
            cout<<"Ӱ�ӽǶ�Ϊ"<<180+anglecount<<endl;
            ShadowAngle=180.00+anglecount;
        }
        else if(x>0&&y<0)
        {
            cout<<"Ӱ���ڵ�������"<<endl;

            cout<<"Ӱ�ӽǶ�Ϊ"<<360-anglecount<<endl;
            ShadowAngle=360.00-anglecount;
        }
        return 2;
    }
}

void DeletePicEdge(Mat pic)
{
    int picRadius=700;
    int picX,picY;

    for ( picX = 0; picX < pic.cols; picX++)
        for ( picY = 0; picY < pic.rows; picY++)
        {
            int A=(picX-Fx);
            int B=(picY-Fy);
            if( (A*A)+(B*B)>(picRadius*picRadius) )
                pic.at<uchar>(picY, picX) = 0;

        }
}

Point TraversalTwo(Vec4i m,float finalAngle,Mat erodemat)
{
    if((abs(finalAngle-90)<=1))
    {
        int gray[3] = {0,0,0};
        int endpoint = 0;
        int x = Fx;
        int y = Fy;
        Mat temp = erodemat.clone();
        while(endpoint == 0&&y > 0)
        {
            gray[0] = erodemat.at<uchar>(y,x);
            gray[1] = erodemat.at<uchar>(y-1,x);
            if(abs(gray[0]-gray[1])>100)
            {
                endpoint = 1;
            }
            else
            {
                y = y-1;
            }
            temp.at<uchar>(x,y)=255;
        }
        imwrite("up90.jpg",temp);
        return Point(x,y);

        //y--����
    }
    else if((abs(finalAngle-270)<=1))
    {
        //y++����
        int gray[3] = {0,0,0};
        int endpoint = 0;
        double x = Fx;
        double y = Fy;
        Mat temp = erodemat.clone();
        while(endpoint == 0&&y < erodemat.rows)
        {
            gray[0] = erodemat.at<uchar>(y,x);
            gray[1] = erodemat.at<uchar>(y+1,x);
            if(abs(gray[0]-gray[1])>100)
            {
                endpoint = 1;
            }
            else
            {
                y = y+1;
            }
            temp.at<uchar>(y,x)=255;
        }
        imwrite("down270.jpg",temp);
        return Point(x,y);
    }
    else if((finalAngle>=0&&finalAngle<90)||(finalAngle>270&&finalAngle<=360))
    {
        //x+
        int gray[3] = {0,0,0};
        int endpoint = 0;
        double x = Fx;
        double y = Fy;
        cout<<"m point   ("<<m[0]<<","<<m[1]<<")   and   ("<<m[2]<<","<<m[3]<<")"<<endl;
        double k = tan(finalAngle*3.14159/180.000);
        cout<<"k  "<<k<<endl;
        k = -k;
        //y = k*(x-Fx)+Fy
        Mat temp = erodemat.clone();
        while(endpoint == 0&&y > 0)
        {
            gray[0] = erodemat.at<uchar>(k*(x-Fx)+Fy,x);
            gray[1] = erodemat.at<uchar>(k*(x+1-Fx)+Fy,x+1);
            if(abs(gray[0]-gray[1])>100)
            {
                endpoint = 1;
            }
            else
            {
                x = x+1;
            }
            temp.at<uchar>((int)(k*(x-Fx)+Fy),x)=255;
        }
        cout<<"final point   ("<<x<<","<<k*(x-Fx)+Fy<<")"<<endl;
        imwrite("right180.jpg",temp);
        return Point(x,k*(x-Fx)+Fy);
    }
    else if(finalAngle>90&&finalAngle<270)
    {
        //x-
        int gray[3] = {0,0,0};
        int endpoint = 0;
        double x = Fx;
        double y = Fy;
        cout<<"m point   ("<<m[0]<<","<<m[1]<<")   and   ("<<m[2]<<","<<m[3]<<")"<<endl;
        double k = tan(finalAngle*3.14159/180.000);//double(m[3]-m[1])/double(m[2]-m[0]);
        cout<<"k  "<<k<<endl;
        k = -k;
        //y = k*(x-Fx)+Fy
        Mat temp = erodemat.clone();
        while(endpoint == 0&&x > 0)
        {
            gray[0] = erodemat.at<uchar>(k*(x-Fx)+Fy,x);
            gray[1] = erodemat.at<uchar>(k*(x-1-Fx)+Fy,x-1);
            if(abs(gray[0]-gray[1])>100)
            {
                endpoint = 1;
            }
            else
            {
                x = x-1;
            }
            temp.at<uchar>(k*(x-Fx)+Fy,x)=255;
        }
        //ShowPicture("zzz",temp);
        cout<<"final point   ("<<x<<","<<k*(x-Fx)+Fy<<")"<<endl;
        imwrite("left180.jpg",temp);
        return Point(x,k*(x-Fx)+Fy);
    }


}

int GetLineLength(Vec4i line){
    return sqrt((line[0]-line[2])*(line[0]-line[2]) + (line[3]-line[1])*(line[3]-line[1]));
}
Mat AddOrDdecreaseBrightness(Mat matLight,Mat matDark,int deltaGrey){
    Mat absdiffResult2Grey;
    Mat delta(matLight.rows,matLight.cols,CV_8UC3,Scalar(deltaGrey,deltaGrey,deltaGrey));
    matLight=matLight+delta;
    matDark=matDark-delta;
    cvtColor(matLight-matDark, absdiffResult2Grey, CV_BGR2GRAY);

    Mat ThresholdMean;
    cv::adaptiveThreshold(absdiffResult2Grey, ThresholdMean, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY, 801, 7);
    imwrite("AddOrDdecreaseBrightness.jpg",ThresholdMean);

    //ȥ���ӵ�
    Mat elementDilate = getStructuringElement(MORPH_RECT,Size(5,5));
    dilate(ThresholdMean,ThresholdMean,elementDilate);
    Mat elementErode = getStructuringElement(MORPH_RECT,Size(7,7));
    erode(ThresholdMean,ThresholdMean,elementErode);
    imwrite("Aodb2dilate2erode.jpg",ThresholdMean);
    return ThresholdMean;
}
double getGrayAvg(Mat mat){
    Mat gray;
    cvtColor(mat,gray,CV_BGR2GRAY);
    cv::mean(gray);
    cv::Mat mean;
    cv::Mat stdDev;
    cv::meanStdDev(gray, mean, stdDev);
    return mean.ptr<double>(0)[0];
}
int ShowMode(Mat dark,Mat light)
{

    //Mat matLight = imread("321light_15.jpg");
    //Mat matDark = imread("321nolight_15.jpg");

    Mat matLight = light;
    Mat matDark = dark;
    Mat thresoldResult;
    cvtColor(matLight,thresoldResult,CV_BGR2GRAY);

    if(getGrayAvg(matLight) > 120.0&&getGrayAvg(matLight) < 255.0){
        thresoldResult = AddOrDdecreaseBrightness(matLight,matDark,80);
    }else if(getGrayAvg(matLight) > 0&&getGrayAvg(matLight) <= 120.0){
        thresoldResult = AddOrDdecreaseBrightness(matLight,matDark,120);
    }


    double t_total = (double)cvGetTickCount();
    double t1 = (double)cvGetTickCount();
    t1 = (double)cvGetTickCount() - t1;
    printf( "processing time ReduceAndGrey = %gms\n", t1/(cvGetTickFrequency()*1000) );//���ʱ��Ϊms
    double t2 = (double)cvGetTickCount();
    t2 = (double)cvGetTickCount() - t2;
    printf( "processing time AdaptiveThresholdALL = %gms\n", t2/(cvGetTickFrequency()*1000) );//���ʱ��Ϊms

 //����ROI���ڴ�ͼ�Ͻ�ȡԲ����Χ400*400�����������������������д������ռ����Ӱ�ӵĽǶȣ��ٷ��ش�ͼ
    int centre_ROI_weight=200;
    Mat centreROI(thresoldResult,Rect(Fx-centre_ROI_weight,Fy-centre_ROI_weight,2*centre_ROI_weight,2*centre_ROI_weight));
    //rectangle(centreROI,Rect(0,0,2*centre_ROI_weight,2*centre_ROI_weight),Scalar(255,255,255),1);
    imwrite("roi.jpg",centreROI);
/////////////////////////////////////////////////////////////////

    //��Ե��Ⲣ���л���ֱ����ȡ
    srand((unsigned)time(0));
    Mat midImage, dstImage;
    double t3 = (double)cvGetTickCount();
    Canny(centreROI, midImage, 50, 200, 3);//50 200 3
    t3 = (double)cvGetTickCount() - t3;
    printf( "processing time Canny = %gms\n", t3/(cvGetTickFrequency()*1000) );//���ʱ��Ϊms
    cvtColor(midImage, dstImage, CV_GRAY2BGR);

    vector<Vec4i> lines;
    double t4 = (double)cvGetTickCount();
    HoughLinesP(midImage, lines, 1, CV_PI / 180, 85, 5, 15);//...�ۼ�ƽ����ֵ������߶γ��ȣ���������룩80 5 25
    int a,b,c=0;
    for (size_t i = 0; i < lines.size(); i++)
    {
        a=rand()%255 ;
        b=rand()%255 ;
        c=rand()%255;
        Vec4i l = lines[i];
        line(dstImage, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(a, b, c), 1, CV_AA);
    }
    t4 = (double)cvGetTickCount() - t4;
    printf( "processing time Hough = %gms\n", t4/(cvGetTickFrequency()*1000) );//���ʱ��Ϊms
    imwrite("HoughResult.jpg",dstImage);
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    //ɸѡֱ�߲�����Ƕ�
    //1.�ȼ���������߶�������������ж�ֱ�ߵ���ʼ���Ƿ���һ����Բ�Ľӽ���2.������ʼ���Լ�һ��ˮƽ����ĵ��������Ӱ�ӽǶ�
    //�ж������ߵ���ʼ�㿴�Ǹ�����Ǹ�������жϽǶȵ�����
    double t5 = (double)cvGetTickCount();


    cout<<"lines count   "<<lines.size()<<endl;
    if(lines.size()<=0||lines.size()>100)
    {
        cout<<"�������޻��߳���һ�٣�"<<endl;
    }
//---->

    Vec4i final_lines;
    int relength = 0;
    for (int i = 0; i < lines.size(); i++)
    {
        Vec4i tempLines = lines[i];

        cout<<"x1:"<<tempLines[0]<<","<<"y1:"<<tempLines[1]<<endl;
        cout<<"x2:"<<tempLines[2]<<","<<"y2:"<<tempLines[3]<<"\n"<<endl;
        if((abs(tempLines[0]-fx)<100&&abs(tempLines[1]-fy)<100)||(abs(tempLines[2]-fx)<100&&abs(tempLines[3]-fy)<100))
        {
            if(GetLineLength(tempLines) > relength){
                relength = GetLineLength(tempLines);
                final_lines = tempLines;
            }
        }
    }

    if(final_lines[0]==0&&final_lines[1]==0)
    {
        cout<<"no choosen lines!"<<endl;
        return -1;
    }
//<---
    angleex(final_lines,anglework(final_lines));

    //�õ�Ӱ�ӵĽǶȺ��ٷ��ش�ͼ������Ƕ������һ���ߣ�Ѱ���������ϵı�Ե�㣬������ѡ����ߺͽǶȸ��̵�ԭͼ�У�ԭͼ�Ƚ��д���
    //
    //����ֵ�����ͼƬ��΢��ʴ�����ڽ��б�Ե���ж�
    t5 = (double)cvGetTickCount() - t5;
    printf( "processing time chooseAngel = %gms\n", t5/(cvGetTickFrequency()*1000) );//���ʱ��Ϊms
    /////////////////////////////////////////////////////////////////

    double t6 = (double)cvGetTickCount();


    Point p = TraversalTwo(final_lines,ShadowAngle,thresoldResult);

    double finaldis = sqrt((p.x-Fx)*(p.x-Fx)+(p.y-Fy)*(p.y-Fy));
    cout<<"Ӱ�ӵĳ�����  "<<finaldis<<endl;

    //Ȼ���ֱ�߹��ճ�������ѡ����ߵõ���ʼ�㣬���̵���ͼ�ϣ��ٻ����ֱ�ߣ�����ֱ���������޽Ƕȣ�ѡ�񷽰����б���
    //1.�Ƕ��ڣ�315��360����0��45������x+1������2.�Ƕ��ڣ�45��135������y+1������3.�Ƕ��ڣ�135��225������x-1������4.�Ƕ��ڣ�225��315������y-1����
    //��Ҫ�õ�ѡ����ֱ�ߵĽǶȺ���ʼ��
    /////////////////////////////////////////////////////////////////
    t6 = (double)cvGetTickCount() - t6;
    printf( "processing time FindShadows = %gms\n", t6/(cvGetTickFrequency()*1000) );//���ʱ��Ϊms
    double shadowDis= PXtoCM(finaldis);
    float pole_distance=calPole_distance(shadowDis);
    X=pole_distance*cos(ShadowAngle*PI/180.000);
    Y=pole_distance*sin(ShadowAngle*PI/180.000);
    double cosa=cos(ShadowAngle*PI/180.000);
    double sina=sin(ShadowAngle*PI/180.000);
    //sin cos��������û���
    cout<<"The shadow distance is: "<<finaldis<<","<<shadowDis<<endl;
    cout<<"The ShadowAngle is:   ("<<ShadowAngle<<"��)"<<endl;
    //cout<<"The cos ShadowAngle is: "<<cosa<<endl;
    //cout<<"The sin ShadowAngle is: "<<sina<<endl;
    cout<<"The pole's location(cm) is:  ("<<X<<","<<Y<<")"<<endl;
    t_total = (double)cvGetTickCount() - t_total;
    printf( "processing time Total = %gms\n", t_total/(cvGetTickFrequency()*1000) );//���ʱ��Ϊms
    while(1)
    {
        if(waitKey(100)==27)
            break;
    }
    destroyAllWindows();
    return 0;
}

int ManuialMode(int manu1)
{
    cv::VideoCapture Camera(0);
    Camera.set(CV_CAP_PROP_FRAME_WIDTH, 2048);//���
    Camera.set(CV_CAP_PROP_FRAME_HEIGHT, 1536);//�߶�
    if (!Camera.isOpened())
        return 0;

    int n=1;
    while (1)
    {
        cv::Mat SavePic;
        if (!Camera.read(SavePic))
            break;
        cv::imshow("video", SavePic);

        char filename1[100]="test.jpg";
        char filename2[100]="yiqi.jpg";

        char key = (char) waitKey(1); //delay N millis, usually long enough to display and capture input
        switch (key)
        {

        case 27: //escape key
            destroyAllWindows();
            return 0;
        case ' ': //Save an image
            if(n==1)
            {
                imwrite(filename1, SavePic);
                cout << "\n\t>������ " << filename1 <<" �ļ�������Ŀ¼��"<< endl;
            }
            else
            {
                imwrite(filename2, SavePic);
                cout << "\n\t>������ " << filename2 <<" �ļ�������Ŀ¼��"<< endl;
            }
            if(n==2)
                n=1;
            else
                n++;
            break;
        default:
            break;
        }

    }
    Camera.release();
    return 0;
}

int AutoMode()
{

    int i;
    while(1)
    {
        for(i=1; i++; i<3)
        {
            cout<< "���򼴽���10s������ͼƬ������������λ�ã�Ȼ��ȷ����λ���Ѿ��رա�"<<endl;
            for(int time=5; time--; time>0)
            {
                Sleep(1000);
                cout<<time<<endl;
            }
            cv::VideoCapture Camera(2);
            Camera.set(CV_CAP_PROP_FRAME_WIDTH, 2048);//���
            Camera.set(CV_CAP_PROP_FRAME_HEIGHT, 1536);//�߶�
            if (!Camera.isOpened())
            {
                cout<< "���δ�����򿪡�"<<endl;
                return 0;
            }
            Sleep(5000);
            Mat SavePic;
            if (!Camera.read(SavePic))
            {
                cout<< "δ�ܶ�ȡ���ͼ��"<<endl;
                break;
            }
            //cv::imshow("video", SavePic);


            cout<< "���������޶�λ����Ƭ..."<<endl;

            Mat photo1;
            Camera>>photo1;
            //SavePic.copyTo(photo1);
            imwrite("nolight.jpg",photo1);
            cout<< "������ϣ���򿪶�λ�ƣ�������10s�������һ��ڶ���ͼƬ��"<<endl;
            for(int time=5; time--; time>0)
            {
                Sleep(1000);
                cout<<time<<endl;
            }
            cout<< "���������ж�λ����Ƭ..."<<endl;
            Camera.release();
            cv::VideoCapture Camera1(0);
            Camera1.set(CV_CAP_PROP_FRAME_WIDTH, 2048);//���
            Camera1.set(CV_CAP_PROP_FRAME_HEIGHT, 1536);//�߶�
            Mat photo2;
            Sleep(5000);
            Camera1>>photo2;
            imwrite("light.jpg",photo2);
            cout<< "����������..."<<endl;
            ShowMode(photo1,photo2);
            cout<< "���������һ����Ƭ���㡣"<<endl;
            Camera1.release();
            break;
        }

        cout<< "������10s�������"<<endl;
        for(int time=10; time--; time>0)
        {
            Sleep(1000);
            cout<<time<<endl;
        }
        break;
    }

    return 0;
}

int main()
{

    while (1)
    {

        cout << "<---------->welcome to LED direct,please choose your direct mode��<---------->\n" << endl;
        cout << "1.AutoMode\n" << endl;
        cout << "2.ManuialMode��follow the instruction and take 3 pictures)\n" << endl;
        cout << "3.AutoMode��every 10 seconds take a picture in 30 seconds)\n" << endl;
        cout << "4.ParameterSetting��set the parameter such as LED height so on��\n" << endl;
        cout << "Please input your choosen number��\n" << endl;
        int choose ;//= 1;
        scanf("%d", &choose);
        switch (choose)
        {
        case 1:
            AutoMode();
            break;
        case 2:
            manu1 = ManuialMode(manu1);
            if (manu1 == 0)
            {
                break;
            }
            else
            {
                cout <<  "ManuialMode Error!" << endl;
                break;
            }
        case 3:
            printf("this mode is preparing...\n\n");
            break;
        case 4:
            printf("this mode is preparing...\n\n");
            break;
        default:
            printf("this is no a right number,please input again!\n\n");
            break;


        }

    }


    return 0;
}



