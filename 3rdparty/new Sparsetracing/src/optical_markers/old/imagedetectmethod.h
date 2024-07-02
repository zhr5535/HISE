#ifndef IMAGEDETECTMETHOD_H
#define IMAGEDETECTMETHOD_H

#include <QObject>
#include "../utils.h"
#include <opencv2/core.hpp>

using namespace cv;
using namespace std;

class ImageDetectMethod
{
public:
	struct image_wrapper_t
	{
		std::add_pointer_t<uint8_t> data{nullptr};
		size_t rows{};
		size_t cols{};
	};

	ImageDetectMethod(){};

	~ImageDetectMethod(){};

	//*********************编码点和非编码点检测************************
	// image - image wrapper to be detected.
	// ratio_k - 输入灰度准则的比例，非编码点外径与内径之比
	// ratio_k1 - 输入编码环内径与中间定位圆的内径之比
	// ratio_k2 - 输入编码环外径与中间定位圆的内径之比
	// min_radius - 圆最小半径
	// max_radius - 圆最大半径
	// color_type - 标志点白底黑点还是黑底白点
	// code_bites_type - 编码点编码位数， 一般默认常用 15_8 15段编码8个1
	// code_point_mat - 输出编码点信息矩阵  n*7 , id,x,y,quality, r_a,r_b,angle_in_pi
	// uncode_point_mat - 输出编码点信息矩阵 n*7 , id,x,y,quality, r_a,r_b,angle_in_pi
	static bool _General_circular_marker_detector(
		image_wrapper_t image,
		Mat &code_point_mat,
		Mat &uncode_point_mat,
		float min_radius, float max_radius,
		float radius_ratios[3],
		float ellipse_error_pixel = 0.5,
		MarkPointColorType color_type = BlackDownWhiteUp,
		CodePointBitesType code_bites_type = CodeBites15,
		DetectContoursMethod image_process_method = OTSU_Method,
		SubPixelPosMethod subpixel_pos_method = Gray_Centroid,
		bool enable_homofiler = false);

	//*******************圆心定位精度测试用**********************
	static bool TestCircleCenterPosAccuracy(
		QString image_file_name,
		Mat &code_point_mat,
		Mat &uncode_point_mat,
		float ratio_k, float ratio_k1, float ratio_k2,
		float min_radius, float max_radius,
		float ellipse_error_pixel = 0.5,
		MarkPointColorType color_type = BlackDownWhiteUp,
		CodePointBitesType code_bites_type = CodeBites15,
		DetectContoursMethod image_process_method = OTSU_Method,
		SubPixelPosMethod subpixel_pos_method = Gray_Centroid);

	// 标定板角点检测
	// corners-输出角点坐标，sign_list-输出点是否有效
	// 棋盘
	static bool FindChessGridOpencv(QString image_file_name,
									int h_num, int v_num,
									vector<Point2f> &corners,
									vector<uchar> &sign_list,
									int &useful_corner_num);

	// CSI标定板
	static bool FindCircleGrid(QString image_file_name, int h_num, int v_num, int h_offset, int v_offset, int h_mid_length,
							   int v_mid_length, vector<Point2f> &corners, vector<uchar> &sign_list, int &useful_corner_num,
							   float ratio_k = 2, float ratio_k1 = 2.4, float ratio_k2 = 4,
							   float min_radius = 5, float max_radius = 50, float ellipse_error_pixel = 0.5,
							   const vector<vector<Point2f>> &edge_points = vector<vector<Point2f>>(),
							   DetectContoursMethod image_process_method = OTSU_Method, SubPixelPosMethod subpixel_pos_method = Gray_Centroid);

	// 编码点自标定，特征点检测
	static bool FindCodePointsForSelfCalibrationByHartly(QStringList image_file_name_list, vector<vector<Point2f>> &code_points_list,
														 float ratio_k = 2, float ratio_k1 = 2.4, float ratio_k2 = 4,
														 float min_radius = 5, float max_radius = 50, float ellipse_error_pixel = 0.5,
														 MarkPointColorType color_type = BlackDownWhiteUp, CodePointBitesType code_bites_type = CodeBites15,
														 DetectContoursMethod image_process_method = OTSU_Method, SubPixelPosMethod subpixel_pos_method = Gray_Centroid);

	// 编码点标定板检测，特定十字编码标定班，以及自定义类型
	static bool FindCodePointsForPlaneCalibration(const QString image_file_name, const vector<int> &code_point_id,
												  vector<uchar> &sign_list, vector<Point2f> &code_points_list,
												  float ratio_k = 2, float ratio_k1 = 2.4, float ratio_k2 = 4,
												  float min_radius = 5, float max_radius = 50, float ellipse_error_pixel = 0.5,
												  MarkPointColorType color_type = BlackDownWhiteUp, CodePointBitesType code_bites_type = CodeBites15,
												  DetectContoursMethod image_process_method = OTSU_Method, SubPixelPosMethod subpixel_pos_method = Gray_Centroid);

private:
	// 图像预处理
	static bool ImagePreprocess(const Mat &ori_image_mat, Mat &processed_image_mat);
	// 检测闭合的轮廓线并存入序列
	static bool DetectClosedContours(const Mat &image_mat, vector<vector<Point>> &contours, DetectContoursMethod image_process_method);
	// 二值图，闭合边缘追踪
	static bool TraceEdge(int start_y, int start_x, int y, int x,
						  vector<Point> *edge_vector, Mat *cannyed_image_mat, Mat *result_mat, bool *is_contour_closed);
	// 自己写的Canny算子
	static bool SelfCannyMethod(const Mat &ori_image_mat, Mat &output_image_mat, float canny_low_thresh, float canny_high_thresh,
								vector<vector<Point>> &edge_point_list);
	// canny算子边缘追踪，高低阈值
	static bool FindCannyEdge(int start_y, int start_x, int y, int x,
							  Mat *low_threshold_mat, Mat *high_threshold_mat, Mat *search_sign_mat, vector<Point> *edge_point_vector, bool *is_contour_closed);

	// 条件准则约束，筛选边缘轮廓
	// ellipse_pars - n*6  center_x,center_y,r_a,r_b,angle_inPI,ellipse_error,contours_index,ID
	static bool FilterEllipseContours(const vector<vector<Point>> &contours, int min_radius_pixel, int max_radius_pixel, float ellipse_error_pixel,
									  QList<QList<float>> &ellipse_pars);

	static float ErrorDROfEllipseFit(float center_x, float center_y, float ellipse_a, float ellipse_b, float ellipse_angle,
									 float x, float y);
	static float LeastSquareErrorOfEllipseFit(float center_x, float center_y, float ellipse_a, float ellipse_b, float ellipse_angle,
											  float x, float y);

	// 进一步筛选出适用于编码点解码的圆点
	static bool FilterEllipseContoursForCodePoint(const Mat &image_mat, float ratio_k, float ratio_k1, float ratio_k2,
												  QList<QList<float>> &ellipse_pars);

	static bool EllipseGrayJudgeForCodePoint(const Mat &image_mat, float center_x, float center_y,
											 float ellipse_a, float ellipse_b, float angle_in_pi, float ratio_k, bool white_on_black = true,
											 float *out_foreground_mean = nullptr, float *out_background_mean = nullptr, float *out_foreground_stdDev = nullptr, float *out_background_stdDev = nullptr);

	// 获取编码值数组的指针，
	static int *ReturnDefualtIdArray(int &array_size, CodePointBitesType code_bites_type = CodeBites15);

	// 解码,编码点解码
	static int Decoding20140210(const Mat &image_mat, float center_x, float center_y, float ellipse_a, float ellipse_b, float angle_in_pi,
								float ratio_k1 = 2.4, float ratio_k2 = 4, MarkPointColorType color_type = BlackDownWhiteUp, CodePointBitesType code_bites_type = CodeBites15);

	static bool CalculateRealCodeID20140210(QList<int> in_put_code_list, QList<int> &out_put_code_list, int &out_put_code_ID);
	// 非编码点判断
	static bool UncodePointCheck(const Mat &image_mat, float center_x, float center_y, float ellipse_a, float ellipse_b, float angle_in_pi,
								 float ratio_k = 2, MarkPointColorType color_type = BlackDownWhiteUp, CodePointBitesType code_bites_type = CodeBites15);

	// 亚像素定位
	static bool FindSubPixelPosOfCircleCenter20140210(const Mat &image_mat, float center_x, float center_y, float ellipse_a, float ellipse_b,
													  float angle_in_pi, const vector<Point> &contour_points,
													  float &sub_pixel_center_x, float &sub_pixel_center_y, vector<Point2f> *subpixel_edge_points = nullptr, SubPixelPosMethod subPixel_method = NoSubPixel_Match, MarkPointColorType color_type = BlackDownWhiteUp);

	// 获取椭圆局部感兴趣区域
	static QRect GetEllipseROIRect(const Mat &image_mat, float center_x, float center_y, float ellipse_a, float ellipse_b, float angle_in_pi);

	//**********Otsu方法图像二值化，返回二值化阈值,支持高比特
	static float DICOTSU20140215(const Mat &ori_image, int total_level = 256);

	//******如果没有指定，黑底白点或白底黑点时，判断是哪种类型
	static MarkPointColorType JudgeTargetColorType(const Mat &sub_mat, float center_x_insubmat, float center_y_insubmat, float ellipse_a, float ellipse_b, float angle_in_pi);

	//******一种获得局部阈值的放法， 阈值 = (目标区域平均灰度 + 背景区域平局灰度)/2
	static float CalThresholdInSubsetMat(const Mat &sub_mat, float center_x_insubmat, float center_y_insubmat, float ellipse_a, float ellipse_b, float angle_in_pi);

	//*****一种获取局部阈值的方法，轮廓边缘灰度平均值
	static float CalThresholdInSubsetMat2(const Mat &image_mat, const vector<Point> &contour_points);

	//*********灰度重心法计算局部重心
	static bool CalCentriodBySubsetMat(const Mat &subset_mat, float thresh_old, float &sub_center_x, float &sub_center_y,
									   MarkPointColorType color_type = BlackDownWhiteUp, SubPixelPosMethod subPixel_method = Gray_Centroid);

	//**********获得重心法灰度权重
	static float CalWeightOfCentriod(float gray_value, float gray_threshold, MarkPointColorType color_type = BlackDownWhiteUp,
									 SubPixelPosMethod subPixel_method = Gray_Centroid);

	//*******计算点的灰度梯度幅值和角度（G）
	static void CalGrayGradientBySobel(const Mat &image_mat, int i, int j, float *gray_gradient, float *gradient_sita = nullptr);

	//*********曲率滤波，计算连续边缘点的曲率
	static void CalCurvatureFromEdgePoints(const vector<Point2f> &edge_points, vector<float> &curvature_vector);

	//************根据拟合误差，筛选部分点重新选点进行拟合
	static void ReduceBadEllipseFitPoints(vector<Point2f> &edge_points, float center_x, float center_y, float ellipse_a, float ellipse_b, float angle_in_pi);

	//**************平均值****
	static int AverageOfList(const QList<int> &list_value);

	//*****************canny算子自适应阈值Otsu法确定高阈值Th2，Th1= 0.5Th2；
	// 返回处于哪一等级时类间方差最大
	static double OTSUForCannyAdaptiveHighTh(Mat ori_image, int total_level = 256);

	//*********************获取由椭圆圆心发出射线与编码环两个交点之间的线段上的灰度值******************
	// image_mat-输入原始图像
	// point1-输入交点1坐标
	// point2-输入交点2坐标
	// 返回一系列数
	static QList<int> GetALineGrayList(Mat image_mat, QPoint point1, QPoint point2);

	//*************************获取一系列数的中值*********************************
	// value_list - 输入数组
	// 返回中值
	static int MIdValue(QList<int> value_list);

	//***********二进制转十进制，输入list_code2二进制数，输出return十进制值**********************
	static int Change2To10(QList<int> list_code2);

	//
	static void WirteMatToFile(Mat mat, QString file_name = "mat");

	// 筛选出CSI类圆标定板的 圆点方向点（外圆点）+ 普通点， vector<int> orident_point_index,3个方向点的index
	static bool FilterEllipseContoursForCSICalibrationPlane(const Mat &image_mat, float ratio_k, float ratio_k1, float ratio_k2,
															QList<QList<float>> &ellipse_pars, vector<int> &orident_point_index);

	// 计算直线方程Ax+By+C=0
	static void LineEquation(Point2f p1, Point2f p2, float &A, float &B, float &C);

	// 计算点p到直线Ax+By+C=0距离d
	static float PointTpLineDistance(Point2f p, float A, float B, float C);

	// 计算点到点之间距离
	static float PointToPointDistance(Point2f p1, Point2f p2);
};

#endif // IMAGEDETECTMETHOD_H
