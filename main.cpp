//此版本省略显示和输出，提高执行速度
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include<iostream>
#include<windows.h>
#include <core/mat.hpp>
#include<fstream>
#include<sstream>
#include<io.h>
#include<time.h>
#include<algorithm>
using namespace std;
using namespace cv;

vector<string> getFilesList(string dir);
vector<string> getAllList(string dir);
cv::Point2f Pixel2World(cv::Point2f PixelCoordinate);
void drawline(Mat& img, vector<Vec2f> lines);
void writeFile(string filepath,float loc);
void getResult(string dir);
int judge_direction(Mat& src);
void set_ROI(Mat& src, Mat& gray, int dir, Mat& ROI, bool drawline);
//void single_process(std::string filePath);
void single_process2(Mat& src);
void salt_pepper(Mat& image, int n);

const int PW = 2448;//图像宽度
const int PH = 2048;//图像高度
const int PD = round(sqrt(PW * PW + PH * PH));
const float aspectRatio = 1.2;//像素宽高比
double D2R = CV_PI / 180;//角度转化为弧度的系数	
float WX = 117;//工件宽度
float WY = 194.27;//工件长度
float Dis_x = 117;//电机移动距离X
float Dis_y = 194.17;//电机移动距离Y
const double IA = 0;//粗测量的角度下限
const double SA = 2;//粗测量的角度上限

Point2f Pm;//测量点
double theta = 0.0;

int L;
const int dR = 400;
int Rs;

int debug = 1;//是否单个测试图像

bool drawEdge = false;
int thickness = 1;
cv::Scalar ROI_color = Scalar(255, 0, 0);
cv::Scalar edge_color = Scalar(0, 0, 255);
cv::Scalar OR_color = Scalar(0, 255, 0);
//clock_t startTime = clock();

int d = 381;//子块宽度
int k = 13;//子块数量
int error_time = 0;

int time1 = 0;
int time2 = 0;
int main()
{
	if (debug==1)
	{
		clock_t startTime = clock();
		string filePath = "C:\\Users\\wfs\\Desktop\\rebar.jpg";
		Mat src = imread(filePath);
		cvtColor(src, src, COLOR_BGR2GRAY);
		{
			clock_t lsd_start = clock();
			// 创建LSD检测类
			Ptr<LineSegmentDetector> ls = createLineSegmentDetector(LSD_REFINE_STD);
			vector<Vec4f> lines_std;
			// 线检测
			ls->detect(src, lines_std);
			clock_t lsd_end = clock();
			string mydst_path = "C:\\Users\\wfs\\Desktop\\rebar.jpg";
			Mat mydst = imread(mydst_path);
			for (auto line : lines_std)
			{
				int x0 = round(line[0]);
			
				int y0 = round(line[1]);
				int x1 = round(line[2]);
				int y1 = round(line[3]);

				cv::line(mydst, Point(x0, y0), Point(x1, y1), edge_color, thickness);
			}

			cout << "lsd_time:" << lsd_end - lsd_start << "ms" << endl;
			imwrite("C:\\Users\\wfs\\Desktop\\lsd_rebar.bmp", mydst);
		}
		//Mat src = imread(filePath);
		//single_process2(src);
		clock_t finishTime = clock();
		cout<<(finishTime - startTime) - 30<<"ms"<<endl;
	}
	else if(debug==2)
	{
		string filePath= "C:\\Users\\wfs\\Desktop\\thesis\\v3\\sim_res_2p.png";
		Mat src = imread(filePath);
		single_process2(src);
	}
	else if(debug==3)//灰度分布
	{
		string filePath = "C:\\Users\\wfs\\Desktop\\pingxi\\ImageProcessingTEMP\\10\\1\\1.bmp";
		Mat src = imread(filePath, 0);
		ofstream fout("gray.txt");
		int col_left = 1302;
		int col_mid = 1325;
		int col_right = 1340;
		for (int i = 0; i < src.rows; i++)
		{
			uchar* p = src.ptr<uchar>(i);
			fout << i << "\t" << (int)p[col_left] << "\t" << (int)p[col_mid]<<"\t" << (int)p[col_right] <<endl;
		}
		fout.close();
	}
	else if(debug==4)//算法误差
	{
		int s = 5;
		Mat fault = imread("C:\\Users\\wfs\\Desktop\\thesis\\edge0.0\\fault4.png");
		int noise_num = 12000*10;
		Mat lmat = imread("C:\\Users\\wfs\\Desktop\\thesis\\edge0.0\\edge0.0l.png");
		blur(lmat, lmat, Size(s, s));
		
		salt_pepper(lmat, noise_num);
		blur(lmat, lmat, Size(s, s));
		lmat = lmat + fault;
		imwrite("C:\\Users\\wfs\\Desktop\\thesis\\edge0.0\\siml.png", lmat);
		
		
		Mat rmat = imread("C:\\Users\\wfs\\Desktop\\thesis\\edge0.0\\edge0.0r.png");
		blur(rmat, rmat, Size(s, s));
		
		salt_pepper(rmat, noise_num);
		blur(rmat, rmat, Size(s, s));
		rmat = rmat + fault;
		imwrite("C:\\Users\\wfs\\Desktop\\thesis\\edge0.0\\simr.png", rmat);
		Size dst_size = lmat.size();
		Point2f center(lmat.rows / 2.0, lmat.cols / 2.0);


		ofstream fout("error.txt");
		for(double an = -0.0; an >=- SA+0.01; an-=0.1)
		{
			Mat l = lmat.clone();
			Mat r = rmat.clone();
			Mat rotation = getRotationMatrix2D(center,an, 1);
			warpAffine(l, l, rotation, dst_size, INTER_NEAREST, 1);
			single_process2(l);
			float mx1 = Pm.x;
			double theta1 = theta * 180 / CV_PI;

			warpAffine(r, r, rotation, dst_size, INTER_NEAREST, 1);
			single_process2(r);
			float mx2 = Pm.x;
			double theta2 = theta * 180 / CV_PI;

			float dis_error = mx2 - mx1 - 100;
			float ang1_error = theta1 - abs(an);
			float ang2_error = theta2 - abs(an);
			fout << abs(an) << "\t" << dis_error << "\t" << ang1_error << "\t" << ang2_error << endl;
		}
		fout.close();

	}
	else if (debug == 5)//错误率
	{
		string filename = "error_test.txt";
		//ofstream fout(filename);
		for (d = 641; d <= 801; d += 10)
		{
			int error_num = 0;
			float error_rate = 0.0;
			string directory = "C:\\Users\\wfs\\Desktop\\pingxi\\ImageProcessingTEMP";
			vector<string> fileList=getAllList(directory);
			for (auto x : fileList)
			{
				//cout << x << endl;
				Mat src = imread(x);

				single_process2(src);
				Point2f temp_pm1 = Pm;

				int temp = d;
				d = 401;
				single_process2(src);
				Point2f temp_pm2 = Pm;

				if (sqrt((temp_pm2.x - temp_pm1.x) * (temp_pm2.x - temp_pm1.x) + (temp_pm2.y - temp_pm1.y) * (temp_pm2.y - temp_pm1.y)) > 2)
				{
					error_num++;
				}
				d = temp;
			}
			error_rate = error_num *1.0/ 320*100;
			ofstream fout(filename, ios::out | ios::app);
			fout << d << "\t" << error_rate << endl;
			fout.close();
		}
	}
	else if (debug == 6)//算法耗时验证
	{
		string filename = "time.txt";
		ofstream fout(filename);
		d = 381;
		for (k = 3; k <= 103; k+=10)
		{
			error_time = 0;
			string directory = "C:\\Users\\wfs\\Desktop\\pingxi\\ImageProcessingTEMP\\10\\1";
			vector<string> fileList = getFilesList(directory);
			time1 = 0;
			time2 = 0;
			clock_t startTime = clock();
			for (auto x : fileList)
			{
				//cout <<k<< x << endl;
				//clock_t startTime = clock();
				Mat src = imread(x);
				single_process2(src);
				//clock_t finishTime = clock();
				//if ((finishTime - startTime) > max_time)
				//{
					//max_time = (finishTime - startTime);
				//}
			}
			clock_t finishTime = clock();
			time1 /= 16;
			time2 = (finishTime - startTime)/(16);
			ofstream fout(filename, ios::out | ios::app);
			fout << k << "\t" << time1 <<"\t"<<time2 <<endl;
			fout.close();
			
		}
	}
	else if (debug == 7)
	{
	int s = 5;
	Mat fault = imread("C:\\Users\\wfs\\Desktop\\thesis\\edge0.0\\fault4.png");
	int noise_num = 12000 * 10;
	Mat lmat = imread("C:\\Users\\wfs\\Desktop\\thesis\\edge0.0\\edge0.0l.png");
	Mat src = lmat.clone();
	blur(lmat, lmat, Size(s, s));

	salt_pepper(lmat, noise_num);
	blur(lmat, lmat, Size(s, s));
	lmat = lmat + fault;
	imwrite("C:\\Users\\wfs\\Desktop\\thesis\\edge0.0\\siml.png", lmat);

	Size dst_size = lmat.size();
	Point2f center(lmat.rows / 2.0, lmat.cols / 2.0);


	ofstream fout("error2.txt");
	for (double an = -0.0; an >= -SA + 0.01; an -= 0.1)
	{
		k = 13;
		Mat l = lmat.clone();
		Mat rotation = getRotationMatrix2D(center, an, 1);
		warpAffine(l, l, rotation, dst_size, INTER_NEAREST, 1);
		single_process2(l);
		float mx1 = Pm.x;
		double theta1 = theta * 180 / CV_PI;

		k = 88;
		Mat ls = src.clone();
		warpAffine(ls, ls, rotation, dst_size, INTER_NEAREST, 1);
		single_process2(ls);
		float mx2 = Pm.x;
		double theta2 = theta * 180 / CV_PI;

		float pos_error = mx1 - mx2;
		float ang_error = theta1 - theta2;
		fout << abs(an) << "\t" << ang_error << "\t" << pos_error<< endl;
	}
	fout.close();
 }
	else if (debug == 8)//不同算法对比效果
	{
		Mat src = imread("C:\\Users\\wfs\\Desktop\\thesis\\2.bmp");
		single_process2(src);

 }
	else if (debug == 9)
	{
	Mat cmp_mat = imread("C:\\Users\\wfs\\Desktop\\thesis\\V6\\img\\ROI_res_1p.bmp");
	int cut_w = 200;
	Mat mini_2 =cmp_mat(cv::Rect(150, 0, cut_w, cut_w));
	imwrite("C:\\Users\\wfs\\Desktop\\thesis\\my_mini.bmp", mini_2);
 }
	cout << "over!"<< endl;	
	system("pause");
}

void single_process2(Mat &src)
{
	//-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-图像预处理-*-*-*-*-*-*-*-*-*-*-*--*-*-*-*-*-*-*-**-*-*
	clock_t startTime = clock();
	if (src.empty())
	{
		cout << "An error occurred when reading the image!" << endl;
	}
	Mat gray;
	cv::cvtColor(src, gray, COLOR_BGR2GRAY);
	//30-40ms时间
	L = judge_direction(gray);
	
	if (L == -1)
	{
		std::cout << "can not judge the direction! " << std::endl;
	}
	Mat ROI;
	set_ROI(src, gray, L, ROI, false);
	//imwrite("C:\\Users\\wfs\\Desktop\\thesis\\2bmp_ROI.bmp", ROI);
	int mini_r = 400;
		if (L ==3 || L == 4)
		{
			Mat ROI_res = src(cv::Rect(Rs, 0, dR, gray.rows));
			Mat mini_res = ROI_res(cv::Rect(0, ROI_res.rows/2-mini_r, ROI_res.cols,  mini_r));
			//imwrite("C:\\Users\\wfs\\Desktop\\thesis\\2bmp_mini.bmp", mini_res);
			//cv::imwrite("C:\\Users\\wfs\\Desktop\\thesis\\6bmp_ROI.bmp", ROI_res);
			
			
		}
		else
		{
			Mat ROI_res = src(cv::Rect(0, Rs, gray.cols, dR));
			Mat mini_res = ROI_res(cv::Rect(ROI_res.cols/2-mini_r, 0, mini_r, ROI_res.rows));
			//imwrite("C:\\Users\\wfs\\Desktop\\thesis\\14bmp_mini.bmp", mini_res);
			//cv::imwrite("C:\\Users\\wfs\\Desktop\\thesis\\14bmp_ROI.bmp", ROI_res);
		}
	
	//-**-*-*--*-*-*-*-**-*-*-*-*-*-*-*-*-*-*-*-*-直线边检测*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-
	//----------------------------------边缘点定位与自动阈值计算---------------------------------------------
	

	int ds = 21;//统计器宽度
	int s = 1;//循环步长
	//int k = 9;//子块个数
	//int d = 161;//子块宽度

	vector<cv::Point2i> Q;//k个子块上的外边缘点
	vector<float> T;///k个子块上的分割阈值
	float Ta = 0;//自适应分割阈值
	Point Qa;//粗定位外边缘中心点
	int Ld = 0;//子块等距距离

	if (L == 3 || L == 4)//左到右
	{
		Ld = (PH - d) / k;
		for (int i = 0; i < k; i++)
		{
			int yi = PH / 2 + (i - k / 2) * Ld;
			int xi = 0;
			float Ti = 0;//子块的分割阈值
			float Sm = -1;//最大标准差
			Mat Bk = ROI(Rect(0, yi - d / 2, ROI.cols, d));
			for (int j = 0; j <= Bk.cols - 1 - ds; j = j + s)
			{
				Mat rec = Bk(Rect(j, 0, ds, Bk.rows));
				Scalar M, S;
				meanStdDev(rec, M, S);
				if (S[0] > Sm)
				{
					Sm = S[0];
					xi = j;
					Ti = M[0];
				}
			}
			xi = xi + ds / 2;
			Point2i Qi(xi, yi);//分块i中边缘点
			Q.push_back(Qi);
			T.push_back(Ti);
		}
	}
	else //
	{
		Ld = (PW - d) / k;
		for (int i = 0; i < k; i++)
		{
			int xi = PW / 2 + (i - k / 2) * Ld;
			int yi = 0;
			float Ti = 0;//子块的分割阈值
			float Sm = -1;
			Mat Bk = ROI(Rect(xi - d / 2, 0, d, ROI.rows));
			for (int j = 0; j <= Bk.rows - 1 - ds; j = j + s)
			{
				Mat rec = Bk(Rect(0, j, Bk.cols, ds));
				Scalar M, S;
				meanStdDev(rec, M, S);
				if (S[0] > Sm)
				{
					Sm = S[0];
					yi = j;
					Ti = M[0];
				}
			}
			yi = yi + ds / 2;
			Point2i Qi(xi, yi);
			Q.push_back(Qi);
			T.push_back(Ti);
		}
	}
	//-------------------边缘中点计算---------------------
	for (auto q : Q)
	{
		Qa.x += q.x;
		Qa.y += q.y;
	}
	Qa.x = round(Qa.x / k);
	Qa.y = round(Qa.y / k);
	//--------------------自适应阈值计算--------------------
	for (auto t : T)
	{
		Ta += t;
	}
	Ta = round(Ta / k);
	//---------------------粗测角度计算-----------------------
	float Ix = 0;
	float Iy = 0;
	for (auto q : Q)
	{
		if (q.x > Qa.x)
		{
			Ix += q.x - Qa.x;
			Iy += q.y - Qa.y;
		}
		else
		{
			Ix += Qa.x - q.x;
			Iy += Qa.y - q.y;
		}
	}
	float ang = Ix < k?-CV_PI/2:atan(Iy / Ix);
	float theta_a = ang + CV_PI / 2;

	float delta = 0.4 * D2R;//搜索范围
	float beta_min = theta_a - delta;
	beta_min = beta_min < 0 ? 0 : beta_min;
	float beta_max = theta_a + delta;
	beta_max = beta_max > CV_PI ? CV_PI : beta_max;
	int CR = 0;//目标区域半径
	Mat OR;//目标区域
	if (theta_a < CV_PI / 4 || theta_a >3 * CV_PI / 4)
	{
		CR = int(PH * max(sin(beta_min), sin(beta_max)) / 2);
	/*	if (CR > 40)
		{
			CR = 30;
			error_time++;
		}*/
		//{
	//	Mat ROI_3C = ROI.clone();
	//	cvtColor(ROI_3C, ROI_3C, COLOR_GRAY2BGR);
	//	if (debug == 4)
	//	{
	//		Point2f test1, test2;
	//		test1.x = Qa.x - PD / 2 * sin(theta_a);
	//		test1.y = Qa.y + PD / 2 * cos(theta_a);
	//		test2.x = Qa.x + PD / 2 * sin(theta_a);
	//		test2.y = Qa.y - PD / 2 * cos(theta_a);
	//		//line(ROI_3C, test1, test2, edge_color, thickness);
	//		for (auto q : Q)
	//		{
	//			circle(ROI_3C, q, 6, edge_color, 6);
	//			rectangle(ROI_3C, Rect(Qa.x - CR, 0, 2 * CR, ROI.rows), edge_color, thickness);
	//		}
	//		//imwrite("C:\\Users\\wfs\\Desktop\\thesis\\SIM_ROI_KPoints.bmp", ROI_3C);
	//	}
	//}
	//imwrite("C:\\Users\\wfs\\Desktop\\thesis\\OR.bmp", OR);
		OR = ROI(Rect(Qa.x - CR, 0, 2 * CR, ROI.rows));
	
	}
	else
	{
		CR = int(PW * max(abs(cos(beta_min)), abs(cos(beta_max))) / 2);
		/*if (CR > 40)
		{
			CR = 30;
			error_time++;
		}*/
		OR = ROI(Rect(0, Qa.y - CR, ROI.cols, 2 * CR));
	}

	//-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-边缘直线检测-*-*-*-*-*-*-*--*-*-*-*-*-*-*-*-*-*---*-*-**-*-*
	Mat TOR;
	cv::threshold(OR, TOR, Ta, 255, THRESH_BINARY);
	//imwrite("C:\\Users\\wfs\\Desktop\\thesis\\TOR.bmp", TOR);
	clock_t finishTime = clock();
	time1 += finishTime - startTime;
	Mat canny;
	cv::Canny(TOR, canny, 100, 200, 5);
	//imwrite("C:\\Users\\wfs\\Desktop\\thesis\\canny.bmp", canny);
	//------------------------------------------LS-----------------------------------------
	int offset = (Qa.x - CR);
	Mat ls_mat = ROI.clone();
	cvtColor(ROI, ls_mat, COLOR_GRAY2BGR);
	clock_t ls_start = clock();
	vector<Point> points;
	for (int row = 0; row < canny.rows; row++)
	{
		for (int col = 0; col < canny.cols; col++)
		{
			if (canny.at<uchar>(row, col) != 0)
			{
				points.push_back(Point(col, row));
			}
		}
	}
	const int np =2;
	Mat A = Mat::zeros(np, np, CV_64FC1);
	for (int row = 0; row < A.rows; row++)
	{
		for (int col = 0; col < A.cols; col++)
		{
			for (int k = 0; k < points.size(); k++)
			{
				A.at<double>(row, col) = A.at<double>(row, col) + pow(points[k].x, row + col);
			}
		}
	}
	//构建B矩阵
	Mat B = Mat::zeros(np, 1, CV_64FC1);
	for (int row = 0; row < B.rows; row++)
	{
		for (int k = 0; k < points.size(); k++)
		{
			B.at<double>(row, 0) = B.at<double>(row, 0) + pow(points[k].x,(1 - row)) * points[k].y;
		}
	}
	//A*X=B
	Mat X;
	//cout << A << endl << B << endl;
	solve(A, B, X, DECOMP_LU);
	clock_t ls_end = clock();
	cout << "ls_time:" << ls_end - ls_start << endl;
	//cout << X << endl;
	vector<Point>lines;
	for (int x = 0; x < ls_mat.size().width; x++)
	{				// y = b + ax;
		double y = X.at<double>(0, 0) + X.at<double>(1, 0) * x;
		//printf("(%d,%lf)\n", x, y);
		lines.push_back(Point(x+offset, y));
	}
	polylines(ls_mat, lines, false, Scalar(0, 0, 255), 1, 8);
	//imwrite("C:\\Users\\wfs\\Desktop\\thesis\\V6\\ls_mat_1p.bmp", ls_mat);
	//int y0 = 0;
	//int x0 = round((y0 - X.at<double>(0,0)) / X.at<double>(1,0))+offset;
	//int y1 = canny.rows;
	//int x1 = round((y1 - X.at<double>(0, 0)) / X.at<double>(1, 0)) +offset;
	//cv::line(ls_mat, Point(x0, y0), Point(x1, y1), edge_color, thickness);


	// ----------------------------------------LSD-------------------------------------------

	
	Mat lsd_mat = ROI.clone();
	cvtColor(ROI, lsd_mat, COLOR_GRAY2BGR);
	clock_t lsd_start = clock();
		// 创建LSD检测类
	Ptr<LineSegmentDetector> ls = createLineSegmentDetector(LSD_REFINE_STD);
	vector<Vec4f> lines_std;
	// 线检测
	ls->detect(canny, lines_std);
	clock_t lsd_end = clock();
	for (auto line : lines_std)
	{
		int x0 = round(line[0]);
		x0 = Qa.x - CR + x0;
		int y0 = round(line[1]);
		int x1 = round(line[2]);
		x1 = Qa.x - CR + x1;
		int y1 = round(line[3]);
		cv::line(lsd_mat, Point(x0, y0), Point(x1, y1),edge_color,thickness);
	}

	cout << "lsd_time:" << lsd_end - lsd_start << "ms" << endl;
	imwrite("C:\\Users\\wfs\\Desktop\\lsd_rebar.bmp", lsd_mat);
	//imwrite("C:\\Users\\wfs\\Desktop\\thesis\\V6\\lsd_mat_1p.bmp", lsd_mat);
	// 绘图线检测结果
	int break_point = 0;
	//------------------------------------Hough-----------------------------------------------
	Mat hough_mat;
	cvtColor(ROI, hough_mat, COLOR_GRAY2BGR);
	clock_t hough_start = clock();
	vector<Vec2f> hough_lines;
	HoughLines(canny, hough_lines, 1, 0.01, 230);
	clock_t hough_end = clock();
	for (auto line:hough_lines)
	{
		int offset =  (Qa.x - CR);
		Pm.y = src.rows / 2.0;
		Pm.x = (line[0] - Pm.y * sin(line[1])) / cos(line[1]);
		Pm.x = Pm.x + offset;
		Point2f worldPoint = Pixel2World(Pm);
		Point2f P1, P2;
		P1.x = Pm.x - PD / 2 * sin(line[1]);
		P1.y = Pm.y + PD / 2 * cos(line[1]);
		P2.x = Pm.x + PD / 2 * sin(line[1]);
		P2.y = Pm.y - PD / 2 * cos(line[1]);
		cv::line(hough_mat, P1, P2, edge_color, thickness);
	}

	cout << "hough_time:" << hough_end - hough_start << "ms" << endl;
	//imwrite("C:\\Users\\wfs\\Desktop\\thesis\\V6\\hough_mat_1p.bmp", hough_mat);
	break_point = 1;
	//-------------------------------------FLD-------------------------------------------
	
	//----------------------------基于二分法的Hough直线检测--------------------------------------
	clock_t my_start = clock();
	vector<Vec2f> HL;//霍夫变换检测的r和theta
	bool F = true;//没有找到合适的值终止迭代
	int	VL = 0;//二分法的左边界
	int VR = round(sqrt(ROI.rows * ROI.rows + ROI.cols * ROI.cols));
	double thetaH = 0.01 * CV_PI / 180;//离散化的角度
	double rH = 1;//离散化的距离
	double r = 0.0;//r的值
	//!!!!!
	theta = 0.0;//theta的值
	int EPSILON = 1;//二分法的最小区间长度
	int V = 0;
	//Mat	lines;
	while (F)
	{
		if ((VR - VL) <= EPSILON)
		{
			F = false;
			V = VL;//左边一定存在直线
		}
		else
		{
			V = (VL + VR) / 2;
		}
		HoughLines(canny, HL, rH, thetaH, V, 0, 0, beta_min, beta_max);
		int H = HL.size();
		if (H == 1)
		{
			F = false;
		}
		else if (H >= 1)
		{
			VL = V;
		}
		else
		{
			VR = V;
		}
	}
	for (int i = 0; i < HL.size(); i++)
	{
		r += HL[i][0];
		theta += HL[i][1];
	}
	r /= HL.size();
	theta /= HL.size();

	clock_t my_end = clock();
	cout << "my_time:" << my_end - my_start << "ms" << endl;
	//-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-显示直线边缘-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-

	if (L == 3 || L == 4)
	{
		int offset = Rs + (Qa.x - CR);
		Pm.y = src.rows / 2.0;
		Pm.x = (r - Pm.y * sin(theta)) / cos(theta);
		Pm.x = Pm.x + offset;
		Point2f worldPoint = Pixel2World(Pm);
	}
	else
	{
		int offset = Rs + (Qa.y - CR);
		Pm.x = src.cols / 2.0;
		Pm.y = (r - Pm.x * cos(theta)) / sin(theta);
		Pm.y = Pm.y + offset;
		Point2f worldPoint = Pixel2World(Pm);
	}

	//是否显示检测到的直线
	if (true)
	{
		Point2f P1, P2;
		P1.x = Pm.x - PD / 2 * sin(theta);
		P1.y = Pm.y + PD / 2 * cos(theta);
		P2.x = Pm.x + PD / 2 * sin(theta);
		P2.y = Pm.y - PD / 2 * cos(theta);
		line(src, P1, P2, edge_color, thickness);
	}
	//imwrite("C:\\Users\\wfs\\Desktop\\thesis\\result.bmp", src);
//-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*图像输出-**-*-*-*-*-*-*-*-*-*-*-*-*-*-**--*-
//imwrite("C:\\Users\\wfs\\Desktop\\thesis\\sim_res_2p.png", src);
//-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*图像输出-**-*-*-*-*-*-*-*-*-*-*-*-*-*-**--*-
	if (debug == 3)
	{
		//imwrite(filePath, src);
	}

}


int judge_direction(Mat& src)
{
	int dir;
	bool isvertical = true;//true==1
	int ROI_width = 600;
	int ROI_heigt = 500;
	int	ROI_y0 = (src.rows - ROI_heigt) / 2;
	int	ROI_x0 = (src.cols - ROI_width) / 2;

	Mat CROI = src(Range(ROI_y0, ROI_y0 + ROI_heigt), Range(ROI_x0, ROI_x0 + ROI_width)).clone();
	Mat x_kernel = (Mat_<float>(1, 2) << -1, 1);
	Mat y_kernel = (Mat_<float>(2, 1) << -1, 1);
	Mat HD, VD;
	filter2D(CROI, HD, CV_16S, x_kernel);
	filter2D(CROI, VD, CV_16S, y_kernel);
	convertScaleAbs(HD, HD);
	convertScaleAbs(VD, VD);

	Mat HD_mean, VD_mean;
	Mat  HD_stddev, VD_stddev;
	meanStdDev(HD, HD_mean, HD_stddev);
	meanStdDev(VD, VD_mean, VD_stddev);
	double HDS = HD_stddev.ptr<double>(0)[0];//不能用CV_64F
	double VDS = VD_stddev.ptr<double>(0)[0];
	double HDM = HD_mean.ptr<double>(0)[0];
	double VDM = VD_mean.ptr<double>(0)[0];
	double HD_ref = HDM + HDS;
	double VD_ref = VDM + VDS;


	if (HD_ref < VD_ref)
	{
		isvertical = false;
	}
	else if (HD_ref >= VD_ref)
	{
		isvertical = true;
	}

	if (isvertical)//垂直
	{
		int ROI_w = 400;
		Mat LROI = src(Range(0, src.rows), Range(0, ROI_width)).clone();
		Mat RROI = src(Range(0, src.rows), Range(src.cols - ROI_w, src.cols)).clone();
		Scalar LROI_mean = mean(LROI);
		Scalar RROI_mean = mean(RROI);
		if (LROI_mean[0] > RROI_mean[0])
		{
			dir = 3;
		}
		else if (LROI_mean[0] < RROI_mean[0])
		{
			dir = 4;
		}
		else
		{
			return -1;
		}
	}
	else//水平
	{
		int ROI_w = 400;
		Mat UROI = src(Range(0, ROI_w), Range(0, src.cols)).clone();
		Mat DROI = src(Range(src.rows - ROI_w, src.rows), Range(0, src.cols)).clone();
		Mat UROI_mean, DROI_mean;
		Mat UROI_stddev, DROI_stddev;
		meanStdDev(UROI, UROI_mean, UROI_stddev);
		meanStdDev(DROI, DROI_mean, DROI_stddev);
		double UROIS = UROI_stddev.ptr<double>(0)[0];//不能用CV_64F
		double DROIS = DROI_stddev.ptr<double>(0)[0];
		double UROIM = UROI_mean.ptr<double>(0)[0];
		double DROIM = DROI_mean.ptr<double>(0)[0];
		double UROI_ref = UROIM + 2 * UROIS;
		double DROI_ref = DROIM + 2 * DROIS;
		int threshold = 40;
		if ((UROIM - DROIM) > threshold)
		{
			dir = 1;
		}
		else if ((DROIM - UROIM) > threshold)
		{
			dir = 2;
		}
		else if (UROI_ref > DROI_ref)
		{
			dir = 1;
		}
		else if (UROI_ref < DROI_ref)
		{
			dir = 2;
		}
		else
		{
			return -1;
		}
	}
	return dir;
}
void set_ROI(Mat& src, Mat& gray, int dir, Mat& ROI, bool drawline)
{
	if (dir == 3 || dir == 4)	//左右
	{
		Rs = (gray.cols / 2) - (dR / 2);
		ROI = gray(cv::Rect(Rs, 0, dR, gray.rows));
		if (drawline)
			rectangle(src, cv::Rect(Rs, 0, dR, gray.rows),ROI_color, thickness, 8);
	}
	else//上下
	{
		Rs = (gray.rows / 2) - (dR / 2);
		ROI = gray(cv::Rect(0, Rs, gray.cols, dR));
		if (drawline)
			rectangle(src, cv::Rect(0, Rs, gray.cols, dR),ROI_color, thickness, 8);
	}
}
vector<string> getFilesList(string dir)
{
	vector<string> allPath;
	// 在目录后面加上"\\*.*"进行第一次搜索
	string dir2 = dir + "\\*.bmp";

	intptr_t handle;
	_finddata_t findData;

	handle = _findfirst(dir2.c_str(), &findData);
	if (handle == -1) {// 检查是否成功
		cout << "can not found the file ... " << endl;
		return allPath;
	}
	do
	{
		if (findData.attrib & _A_SUBDIR)
		{
			//若该子目录为"."或".."，则进行下一次循环，否则输出子目录名，并进入下一次搜索
			if (strcmp(findData.name, ".") == 0 || strcmp(findData.name, "..") == 0)
				continue;

			// 在目录后面加上"\\"和搜索到的目录名进行下一次搜索
			string dirNew = dir + "\\" + findData.name;
			vector<string> tempPath = getFilesList(dirNew);
			allPath.insert(allPath.end(), tempPath.begin(), tempPath.end());
		}
		else //不是子目录，即是文件，则输出文件名和文件的大小
		{
			string filePath = dir + "\\" + findData.name;
			allPath.push_back(filePath);
			//cout << filePath << "\t" << findData.size << " bytes.\n";
		}
	} while (_findnext(handle, &findData) == 0);
	_findclose(handle);    // 关闭搜索句柄
	return allPath;
}
vector<string> getAllList(string dir)
{
	vector<string> allPath;
	// 在目录后面加上"\\*.*"进行第一次搜索
	string dir2 = dir + "\\*.*";

	intptr_t handle;
	_finddata_t findData;

	handle = _findfirst(dir2.c_str(), &findData);
	if (handle == -1) {// 检查是否成功
		cout << "can not found the file ... " << endl;
		return allPath;
	}
	do
	{
		if (findData.attrib & _A_SUBDIR)
		{
			//若该子目录为"."或".."，则进行下一次循环，否则输出子目录名，并进入下一次搜索
			if (strcmp(findData.name, ".") == 0 || strcmp(findData.name, "..") == 0)
				continue;

			// 在目录后面加上"\\"和搜索到的目录名进行下一次搜索
			string dirNew = dir + "\\" + findData.name;
			vector<string> tempPath = getAllList(dirNew);
			allPath.insert(allPath.end(), tempPath.begin(), tempPath.end());
		}
		else //不是子目录，即是文件，则输出文件名和文件的大小
		{
			string filePath = dir + "\\" + findData.name;
			allPath.push_back(filePath);
			//cout << filePath << "\t" << findData.size << " bytes.\n";
		}
	} while (_findnext(handle, &findData) == 0);
	_findclose(handle);    // 关闭搜索句柄
	return allPath;
}
cv::Point2f Pixel2World(cv::Point2f PixelCoordinate)
{
	double a1 = -2.7608e-09;
	double a2 = -1.78913e-05;
	double a3 = 0.0261964;
	double a4 = 1.78974e-05;
	double a5 = 7.48454e-11;
	double a6 = -0.00160705;
	double a7 = 2.07205e-11;
	double a8 = 7.48693e-12;
	double a9 = 5.18507e-05;
	cv::Point2f WorldCoordinate;
	int xe = PixelCoordinate.x;//图像中点坐标x
	int ye = PixelCoordinate.y;//图像中点坐标y
	WorldCoordinate.y = (a1 * xe + a2 * ye + a3) / (a7 * xe + a8 * ye + a9) / 100;//世界坐标中x值
	WorldCoordinate.x = (a4 * xe + a5 * ye + a6) / (a7 * xe + a8 * ye + a9) / 100;//世界坐标中Y值
	return WorldCoordinate;
}
void writeFile(string filepath,float loc)
{
	string txtFile;
	for (int i = 0; i<filepath.size()-1; i++)
	{
		txtFile.push_back(filepath[i]);
		if (i == (filepath.size() - 5))
		{
			break;
		}
		
	}
	txtFile += ".txt";
	ofstream fout(txtFile);
	fout << loc << endl;
	fout.close();

}
void getResult(string dir)
{
	vector<string> txtList;
	vector<float> locList;
	ifstream fin;
	for (int i = 0; i < 16; i++)
	{
		string path = dir +"\\"+ to_string(i) + ".txt";
		fin.open(path);
		float loc;
		fin >> loc;
		locList.push_back(loc);
		fin.close();
	}
	float D0_12 = locList[12] - locList[0] + Dis_x;
	float D1_11= locList[11] - locList[1] + Dis_x;
	float D2_10 = locList[10] - locList[2] + Dis_x;
	float D3_9 = locList[9] - locList[3] + Dis_x;
	float D4_8 = locList[8] - locList[4] + Dis_x;
	float D5_15 = locList[5] - locList[15] + Dis_y;
	float D6_14 = locList[6] - locList[14] + Dis_y;
	float D7_13 = locList[7] - locList[13] + Dis_y;
	ofstream fout("result.txt");
	fout << D0_12 << endl;
	fout << D1_11 << endl;
	fout << D2_10 << endl;
	fout << D3_9 << endl;
	fout << D4_8 << endl;
	fout << D5_15 << endl;
	fout << D6_14 << endl;
	fout << D7_13 << endl;
	fout.close();

}
void drawline(Mat& img, vector<Vec2f> lines )
{
	int rows = img.rows ; int cols = img.cols; Scalar lineColor = Scalar(0, 0, 255); int thickness = 1;

	Point pt1, pt2;
	for (size_t i = 0; i < lines.size(); i++)
	{
		float rho = lines[i][0];
		float theta = lines[i][1];
		double a = cos(theta);
		double b = sin(theta);
		double x0 = a * rho, y0 = b * rho;//垂点坐标
		double length = max(rows, cols);
		pt1.x = cvRound(x0 + length * (-b));
		pt1.y= cvRound(y0 + length * (a));
		pt2.x = cvRound(x0 - length * (-b));
		pt2.y = cvRound(y0 - length * (a));
		cv::line(img,pt1, pt2, lineColor, thickness);
	}
}
void salt_pepper(Mat& image, int n)
{
	for (int k = 0; k < n ; k++)
	{
		int i, j;
		i = std::rand() % image.cols;
		j = std::rand() % image.rows;
		int write_black = std::rand() % 2;
		if (write_black == 0) //添加白色噪声
		{
			if (image.type() == CV_8UC1)//处理
			{
				image.at<uchar>(j, i) = 255;
			}
			else if (image.type() == CV_8UC3)
			{
				image.at<Vec3b>(j, i)[0] = 255;
				image.at<Vec3b>(j, i)[1] = 255;
				image.at<Vec3b>(j, i)[2] = 255;
			}
		}
		else//添加黑色噪声
		{
			if (image.type() == CV_8UC1)
			{
				image.at<uchar>(j, i) = 0;
			}
			else if (image.type() == CV_8UC3)
			{
				image.at<Vec3b>(j, i)[0] = 0;
				image.at<Vec3b>(j, i)[1] = 0;
				image.at<Vec3b>(j, i)[2] = 0;
			}

		}
	}
}