#include "opencv2/highgui/highgui.hpp"  
#include "opencv2/imgproc/imgproc.hpp"  
#include <iostream>  
#include <stdio.h>  
#include <stdlib.h>  
#include "epnp/epnp.h"
#include <Eigen/Dense>
#include <Eigen/Core>
#include "mysql.h"
#include <windows.h>
#include <process.h>

//djslfj
#include "MyLogger.h"

#include<iostream>
#include<sstream>
#include <fstream>
//#include <string>

using namespace cv;
using namespace std;
using Eigen::Matrix;
using Eigen::VectorXd;


const double uc = 406.37;
const double vc = 278.27;
const double fu = 626.807;
const double fv = 667.706;

const int n = 4;
const int len = 4;

Mat src; Mat src_gray;
int thresh = 90;//设定阈值为100
int max_thresh = 255;
RNG rng(12345);

MYSQL * con; //= mysql_init((MYSQL*) 0); 
MYSQL_RES *res;
MYSQL_ROW row;
char tmp[400];
//database configuartion
char dbuser[30] = "root";
char dbpasswd[30] = "admin"; // it must be    changed
char dbip[30] = "localhost";
char dbname[50] = "picdetect";
char tablename[4][20] = { "infodata1" ,"infodata2","infodata3","infodata4" };
char *query = NULL;
int rt;//return value  

float uu[len];
float vv[len];
float area[len];
string picpath;
/// Function header  
typedef struct Result
{
	float location[len][2];
	float totalarea[len];
}Result;

Result thresh_callback(int, void*);
void getU_V(float*, float*, float*, struct Result);

unsigned Counter = 0;

//日志初始化
MyLogger *MyLogger::my_logger = NULL;

MyLogger::MyLogger()
{
	log4cplus::initialize();
	PropertyConfigurator::doConfigure(LOG4CPLUS_TEXT(MY_LOG_FILE_PATH));
	logger = Logger::getRoot();
}

MyLogger * MyLogger::getInstance()
{
	if (my_logger == NULL)
	{
		my_logger = new MyLogger();
	}
	return my_logger;
}

MyLogger::~MyLogger()
{
	if (my_logger)
	{
		delete my_logger;
	}
}


unsigned __stdcall SecondThreadFunc(void* pArguments)
{
	MyLogger * myLoger = NULL;
	myLoger = MyLogger::getInstance();

	double R_true[3][3], t_true[3];
	float xx[4] = { 0,0,119,119 };
	float yy[4] = { 0,65.25,1,65.25 };
	float zz[4] = { 0,0,0,0 };
	
	con = mysql_init((MYSQL*)0);
	if (con != NULL && mysql_real_connect(con, dbip, dbuser, dbpasswd, dbname, 3306, NULL, 0)) {
		if (!mysql_select_db(con, dbname)) {
			printf("Select successfully the database!\n");
			LOG4CPLUS_DEBUG(myLoger->logger, " 数据库连接成功");
			con->reconnect = 1;
			query = "set names \'GBK\'";
			rt = mysql_real_query(con, query, strlen(query));
			if (rt) {
				printf("Error making query: %s !!!\n", mysql_error(con));
				}
			else {
				printf("query %s succeed!\n", query);
				}
			}
		}
		else {
			LOG4CPLUS_FATAL(myLoger->logger, "数据库连接有误[" << mysql_error(con) << "]");
			MessageBoxA(NULL, "Unable to connect the database,check your configuration!", "", NULL);
		}
		while (1)
		{
		for (int j = 0; j< 4; j++) {
			sprintf(tmp, "select id,path from %s where iscaculated =0", tablename[j]);
			rt = mysql_real_query(con, tmp, strlen(tmp));
			if (rt)
			{
				printf("Error making query: %s !!!\n", mysql_error(con));
			}
			else
			{
				LOG4CPLUS_FATAL(myLoger->logger, "数据库获取图片路径有误[" << mysql_error(con) << "]");
				printf("%s executed!!!\n", tmp);
			}

			res = mysql_store_result(con);//将结果保存在res结构体中
			struct Result myresult;

			while (row = mysql_fetch_row(res)) {
				//for (t = 0; t<mysql_num_fields(res); t++) {
				picpath = row[1];
				src = imread(picpath, 1);
				
				/// 转成灰度图并进行模糊降噪  
				cvtColor(src, src_gray, CV_BGR2GRAY);
				blur(src_gray, src_gray, Size(3, 3));


				//	createTrackbar(" Threshold:", "Source", &thresh, max_thresh, thresh_callback);
				myresult = thresh_callback(0, 0);

				getU_V(uu, vv, area, myresult);


				epnp PnP;

				srand(time(0));

				PnP.set_internal_parameters(uc, vc, fu, fv);
				PnP.set_maximum_number_of_correspondences(n);

				/*double R_true[3][3], t_true[3];
				float xx[4] = { 0,0,119,119 };
				float yy[4] = { 0,65.25,1,65.25 };
				float zz[4] = { 0,0,0,0 };*/

				PnP.reset_correspondences();
				for (int i = 0; i < n; i++) {
					double Xw, Yw, Zw, u, v;
					Xw = xx[i];
					Yw = yy[i];
					Zw = zz[i];
					u = uu[i];
					v = vv[i];

					PnP.add_correspondence(Xw, Yw, Zw, u, v);
				}

				double R_est[3][3], t_est[3];

				//开始使用Epnp
				double err2 = PnP.compute_pose(R_est, t_est);
				double rot_err, transl_err;

				PnP.relative_error(rot_err, transl_err, R_true, t_true, R_est, t_est);
				/*cout << ">>> Reprojection error: " << err2 << endl;
				cout << ">>> rot_err: " << rot_err << ", transl_err: " << transl_err << endl;
				cout << endl;*/
				
				//<< PnP.reprojection_error(R_true, t_true) << endl;*
			
				
			//	PnP.print_pose(R_est, t_est);
				//cout << R_est[0] << endl;

				PnP.findinv(R_est, t_est);

				sprintf(tmp, "update %s set xdistance = %f,ydistance = %f,zdistance = %f,iscaculated =1 where id = %d", tablename[j], t_est[0], t_est[1], t_est[2], stoi(row[0], 0, 10));

				rt = mysql_query(con, tmp);
				if (rt)
				{
					printf("Error making query: %s !!!\n", mysql_error(con));
					LOG4CPLUS_DEBUG(myLoger->logger, "数据库更新失败，可能没有新的数据[" << mysql_error(con) << "]");
				}
				else
				{
					printf("%s executed!!!\n", tmp);
					LOG4CPLUS_DEBUG(myLoger->logger, "新的数据库已计算，数据库更新成功");

				}


			}
		}
		Sleep(60000); //每隔6秒执行一次累加!
	}
}


/** @function main */
void main(int argc, char** argv)
{
	HANDLE hThread;
	unsigned threadID;

	printf("Creating second thread...\n");

	// Create the second thread.
	hThread = (HANDLE)_beginthreadex(NULL, 0, &SecondThreadFunc, NULL, 0, &threadID);

	//只是为了防止主线程退出!
	WaitForSingleObject(hThread, INFINITE);

	CloseHandle(hThread);

}

//余弦角计算
static double angle(Point pt1, Point pt2, Point pt0)
{
	double dx1 = pt1.x - pt0.x;
	double dy1 = pt1.y - pt0.y;
	double dx2 = pt2.x - pt0.x;
	double dy2 = pt2.y - pt0.y;
	return (dx1*dx2 + dy1*dy2) / sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
}

//面积排序，质心按序号排
static void BubbleSort(float  *p, int length, int * ind_diff)
{
	for (int m = 0; m < length; m++)
	{
		ind_diff[m] = m;
	}

	for (int i = 0; i < length; i++)
	{
		for (int j = 0; j < length - i - 1; j++)
		{
			if (p[j] > p[j + 1])
			{
				float temp = p[j];
				p[j] = p[j + 1];
				p[j + 1] = temp;

				int ind_temp = ind_diff[j];
				ind_diff[j] = ind_diff[j + 1];
				ind_diff[j + 1] = ind_temp;
			}
		}
	}
}

/** @function thresh_callback *///寻找轮廓
struct Result thresh_callback(int, void*)
{
	Mat src_copy = src.clone();
	Mat threshold_output;
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;

	//自定义的变量

	struct Result myresult;

	vector<vector<Point>> squares;
	squares.clear();

	int count = 0;

	vector<Point2f> mc1oc(10);

	/// 对图像进行二值化  
	threshold(src_gray, threshold_output, thresh, 255, THRESH_BINARY);

	/// 寻找轮廓  
	findContours(threshold_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
	/*Mat drawing = Mat::zeros(threshold_output.size(), CV_8UC3);*/

	//定义轮廓矩
	vector<Moments> mu(contours.size());
	//定义
	vector<Point2f> mc(contours.size());

	/* 对每个轮廓计算其凸包*/
	vector<Point> approx(contours.size());

	vector<vector<Point> >poly(contours.size());


	for (int i = 0; i < contours.size(); i++)
	{
		//拟合曲线，第一个参数是输入，也就是多个点集合，拟合之后，把这些点输出，也就是第二个参数
		////approxPolyDP(Mat(contours[i]), poly[i], 5, true);
		approxPolyDP(Mat(contours[i]), approx, arcLength(Mat(contours[i]), true)*0.02, true);
		if (approx.size() == 4 &&
			fabs(contourArea(Mat(approx))) >500 && fabs(contourArea(Mat(approx))) < 20000 &&
			isContourConvex(Mat(approx)))
		{
			double maxCosine = 0;

			for (int j = 2; j < 5; j++)
			{
				// 找到最大的余弦角，接近于0，即90°满足为直接保证矩形
				double cosine = fabs(angle(approx[j % 4], approx[j - 2], approx[j - 1]));
				maxCosine = MAX(maxCosine, cosine);

			}

			// if cosines of all angles are small
			// (all angles are ~90 degree) then write quandrange
			// vertices to resultant sequence
			if (maxCosine < 0.3) {
				squares.push_back(approx);

				//计算轮廓矩       
				mu[i] = moments(contours[i], false);
				//计算轮廓的质心     	
				mc[i] = Point2d(mu[i].m10 / mu[i].m00, mu[i].m01 / mu[i].m00);

				/*if (mu[i].m10 > 0 && mu[i].m01 > 0) {
				if (count < mc1oc.size()) {*/
				//	mc1oc[count] = Point2d(mu[i].m10 / mu[i].m00, mu[i].m01 / mu[i].m00);
				//赋值质心坐标和面积
				myresult.totalarea[count] = contourArea(Mat(approx));
				myresult.location[count][0] = mc[i].x;
				myresult.location[count][1] = mc[i].y;
				count = count + 1;

			}
		}
	}

	/* 绘出轮廓及其凸包*/
	//Mat drawing = Mat::zeros(threshold_output.size(), CV_8UC3);
	//for (int i = 0; i < squares.size(); i++)
	//{
	//	Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
	//	drawContours(drawing, squares, i, color, 1, 8, vector<Vec4i>(), 0, Point());
	//	//drawContours(drawing, contours, i, color, 1, 8, vector<Vec4i>(), 0, Point());
	//}

	/*vector<Point> poly;
	approxPolyDP(Mat(contours[3]), poly, 5, false);
	vector<Point>::const_iterator itp = poly.begin();
	while (itp != (poly.end() - 2))
	{
	line(drawing, *itp, *(itp + 1), Scalar(255), 2);
	++itp;
	}
	line(drawing, *itp, *(poly.begin()), Scalar(255), 2);*/
	/// 把结果显示在窗体  
	/*namedWindow("Hull demo", CV_WINDOW_AUTOSIZE);
	imshow("Hull demo", drawing);*/

	return myresult;
	/*waitKey(5);*/

}

//得到排序后的像素坐标
static void getU_V(float *uu, float *vv, float *area, struct Result myresult) {

	float Pi[len][2];

	for (int k = 0; k < len; k++) {
		Pi[k][0] = myresult.location[k][0];
		Pi[k][1] = myresult.location[k][1];
		area[k] = myresult.totalarea[k];

	}

	int ind[len] = { 0 };

	BubbleSort(area, len, ind);

	//cout << "排序开始" << endl;

	/*for (int i = 0; i < len; i++)
	{
		cout << "value: " << area[i] << "Index: " << ind[i] << endl;
	}*/

	for (int i = 0; i < len; i++)
	{
		uu[i] = Pi[ind[i]][0];
		vv[i] = Pi[ind[i]][1];
	//	cout << "排序后 " << uu[i] << "-" << vv[i] << endl;
	}

}

//
//static void getinv(const double R[3][3], const double t[3]) {
//
//	Matrix<double, 3, 3>  matrix_32;
//	Matrix<double, 3, 3> multiply_result;
//	Matrix<double, 3, 1>  T;
//	Matrix<double, 3, 1>  distance;
//
//	for (int i = 0; i < 3; i++) {
//
//		T(i) = t[i];
//		for (int j = 0; j < 3; j++)
//		{
//			matrix_32(i, j) = R[i][j];
//		}
//
//	}
//	multiply_result = matrix_32.inverse();
//	distance = multiply_result* T;
//
//	cout << R[0][0] << " " << R[0][1] << " " << R[0][2] << " " << distance(0) << endl;
//	cout << R[1][0] << " " << R[1][1] << " " << R[1][2] << " " << distance(1) << endl;
//	cout << R[2][0] << " " << R[2][1] << " " << R[2][2] << " " << distance(2) << endl;
//
//}




