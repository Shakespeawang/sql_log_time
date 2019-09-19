#include "opencv2/highgui/highgui.hpp"  
#include "opencv2/imgproc/imgproc.hpp"  
#include <iostream>  
#include <stdio.h>  
#include <stdlib.h>  
#include "epnp/epnp.h"
#include <Eigen/Dense>
#include <Eigen/Core>
//#include "mysql.h"
//#include <windows.h>
//#include<iostream>
//#include<sstream>
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
string path = NULL;
/// Function header  

typedef struct Result
{
	float location[len][2];
	float totalarea[len];
}Result;


Result thresh_callback(int, void*);
void getU_V(float*, float*, float*, struct Result);



/** @function main */
int main(int argc, char** argv)
{

	src = imread("c:/001y10.jpg", 1);

	struct Result myresult;

	bool flag;

	float uu[len];
	float vv[len];
	float area[len];

	/// 转成灰度图并进行模糊降噪  
	cvtColor(src, src_gray, CV_BGR2GRAY);
	blur(src_gray, src_gray, Size(3, 3));

	/// 创建窗体  
	char* source_window = "Source";
	namedWindow(source_window, CV_WINDOW_AUTOSIZE);
	imshow(source_window, src);

	//	createTrackbar(" Threshold:", "Source", &thresh, max_thresh, thresh_callback);
	myresult = thresh_callback(0, 0);

	getU_V(uu, vv, area, myresult);


	epnp PnP;

	srand(time(0));

	PnP.set_internal_parameters(uc, vc, fu, fv);
	PnP.set_maximum_number_of_correspondences(n);

	double R_true[3][3], t_true[3];
	//  random_pose(R_true, t_true);
	float xx[4] = { 0,0,119,119 };
	float yy[4] = { 0,65.25,1,65.25 };
	float zz[4] = { 0,0,0,0 };


	//float xx[4] = { 0,1,120,120 };
	//float yy[4] = { 0,66.25,1,66.25 };
	//float zz[4] = { 0,0,0,0 };

	PnP.reset_correspondences();
	for (int i = 0; i < n; i++) {
		double Xw, Yw, Zw, u, v;
		Xw = xx[i];
		Yw = yy[i];
		Zw = zz[i];
		u = uu[i];
		v = vv[i];

		//  project_with_noise(R_true, t_true, Xw, Yw, Zw, u, v);
		PnP.add_correspondence(Xw, Yw, Zw, u, v);
	}

	double R_est[3][3], t_est[3];
	/*cout << "没算之前" << endl;
	PnP.print_pose(R_est, t_est);*/

	//开始使用Epnp
	double err2 = PnP.compute_pose(R_est, t_est);
	double rot_err, transl_err;

	PnP.relative_error(rot_err, transl_err, R_true, t_true, R_est, t_est);
	cout << ">>> Reprojection error: " << err2 << endl;
	cout << ">>> rot_err: " << rot_err << ", transl_err: " << transl_err << endl;
	cout << endl;
	/*cout << "'True reprojection error':"
	<< PnP.reprojection_error(R_true, t_true) << endl;*/
	cout << endl;
	cout << "True pose:" << endl;
	//PnP.print_pose(R_true, t_true);
	//cout << endl;


	cout << "旋转矩阵:                     位移量：" << endl;

	PnP.print_pose(R_est, t_est);
	cout << R_est[0] << endl;

	PnP.findinv(R_est, t_est);

	waitKey(0);
	return(0);
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
	Mat drawing = Mat::zeros(threshold_output.size(), CV_8UC3);
	for (int i = 0; i < squares.size(); i++)
	{
		Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
		drawContours(drawing, squares, i, color, 1, 8, vector<Vec4i>(), 0, Point());
		//drawContours(drawing, contours, i, color, 1, 8, vector<Vec4i>(), 0, Point());
	}
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
	namedWindow("Hull demo", CV_WINDOW_AUTOSIZE);
	imshow("Hull demo", drawing);

	return myresult;
	waitKey(5);

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

	cout << "排序开始" << endl;

	for (int i = 0; i < len; i++)
	{
		cout << "value: " << area[i] << "Index: " << ind[i] << endl;
	}

	for (int i = 0; i < len; i++)
	{
		uu[i] = Pi[ind[i]][0];
		vv[i] = Pi[ind[i]][1];
		cout << "排序后 " << uu[i] << "-" << vv[i] << endl;
	}

}


static void getinv(const double R[3][3], const double t[3]) {

	Matrix<double, 3, 3>  matrix_32;
	Matrix<double, 3, 3> multiply_result;
	Matrix<double, 3, 1>  T;
	Matrix<double, 3, 1>  distance;

	for (int i = 0; i < 3; i++) {

		T(i) = t[i];
		for (int j = 0; j < 3; j++)
		{
			matrix_32(i, j) = R[i][j];
		}

	}
	multiply_result = matrix_32.inverse();
	distance = multiply_result* T;

	cout << R[0][0] << " " << R[0][1] << " " << R[0][2] << " " << distance(0) << endl;
	cout << R[1][0] << " " << R[1][1] << " " << R[1][2] << " " << distance(1) << endl;
	cout << R[2][0] << " " << R[2][1] << " " << R[2][2] << " " << distance(2) << endl;

}




