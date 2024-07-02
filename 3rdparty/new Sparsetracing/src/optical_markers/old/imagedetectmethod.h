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

	//*********************�����ͷǱ������************************
	// image - image wrapper to be detected.
	// ratio_k - ����Ҷ�׼��ı������Ǳ�����⾶���ھ�֮��
	// ratio_k1 - ������뻷�ھ����м䶨λԲ���ھ�֮��
	// ratio_k2 - ������뻷�⾶���м䶨λԲ���ھ�֮��
	// min_radius - Բ��С�뾶
	// max_radius - Բ���뾶
	// color_type - ��־��׵׺ڵ㻹�Ǻڵװ׵�
	// code_bites_type - ��������λ���� һ��Ĭ�ϳ��� 15_8 15�α���8��1
	// code_point_mat - ����������Ϣ����  n*7 , id,x,y,quality, r_a,r_b,angle_in_pi
	// uncode_point_mat - ����������Ϣ���� n*7 , id,x,y,quality, r_a,r_b,angle_in_pi
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

	//*******************Բ�Ķ�λ���Ȳ�����**********************
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

	// �궨��ǵ���
	// corners-����ǵ����꣬sign_list-������Ƿ���Ч
	// ����
	static bool FindChessGridOpencv(QString image_file_name,
									int h_num, int v_num,
									vector<Point2f> &corners,
									vector<uchar> &sign_list,
									int &useful_corner_num);

	// CSI�궨��
	static bool FindCircleGrid(QString image_file_name, int h_num, int v_num, int h_offset, int v_offset, int h_mid_length,
							   int v_mid_length, vector<Point2f> &corners, vector<uchar> &sign_list, int &useful_corner_num,
							   float ratio_k = 2, float ratio_k1 = 2.4, float ratio_k2 = 4,
							   float min_radius = 5, float max_radius = 50, float ellipse_error_pixel = 0.5,
							   const vector<vector<Point2f>> &edge_points = vector<vector<Point2f>>(),
							   DetectContoursMethod image_process_method = OTSU_Method, SubPixelPosMethod subpixel_pos_method = Gray_Centroid);

	// ������Ա궨����������
	static bool FindCodePointsForSelfCalibrationByHartly(QStringList image_file_name_list, vector<vector<Point2f>> &code_points_list,
														 float ratio_k = 2, float ratio_k1 = 2.4, float ratio_k2 = 4,
														 float min_radius = 5, float max_radius = 50, float ellipse_error_pixel = 0.5,
														 MarkPointColorType color_type = BlackDownWhiteUp, CodePointBitesType code_bites_type = CodeBites15,
														 DetectContoursMethod image_process_method = OTSU_Method, SubPixelPosMethod subpixel_pos_method = Gray_Centroid);

	// �����궨���⣬�ض�ʮ�ֱ���궨�࣬�Լ��Զ�������
	static bool FindCodePointsForPlaneCalibration(const QString image_file_name, const vector<int> &code_point_id,
												  vector<uchar> &sign_list, vector<Point2f> &code_points_list,
												  float ratio_k = 2, float ratio_k1 = 2.4, float ratio_k2 = 4,
												  float min_radius = 5, float max_radius = 50, float ellipse_error_pixel = 0.5,
												  MarkPointColorType color_type = BlackDownWhiteUp, CodePointBitesType code_bites_type = CodeBites15,
												  DetectContoursMethod image_process_method = OTSU_Method, SubPixelPosMethod subpixel_pos_method = Gray_Centroid);

private:
	// ͼ��Ԥ����
	static bool ImagePreprocess(const Mat &ori_image_mat, Mat &processed_image_mat);
	// ���պϵ������߲���������
	static bool DetectClosedContours(const Mat &image_mat, vector<vector<Point>> &contours, DetectContoursMethod image_process_method);
	// ��ֵͼ���պϱ�Ե׷��
	static bool TraceEdge(int start_y, int start_x, int y, int x,
						  vector<Point> *edge_vector, Mat *cannyed_image_mat, Mat *result_mat, bool *is_contour_closed);
	// �Լ�д��Canny����
	static bool SelfCannyMethod(const Mat &ori_image_mat, Mat &output_image_mat, float canny_low_thresh, float canny_high_thresh,
								vector<vector<Point>> &edge_point_list);
	// canny���ӱ�Ե׷�٣��ߵ���ֵ
	static bool FindCannyEdge(int start_y, int start_x, int y, int x,
							  Mat *low_threshold_mat, Mat *high_threshold_mat, Mat *search_sign_mat, vector<Point> *edge_point_vector, bool *is_contour_closed);

	// ����׼��Լ����ɸѡ��Ե����
	// ellipse_pars - n*6  center_x,center_y,r_a,r_b,angle_inPI,ellipse_error,contours_index,ID
	static bool FilterEllipseContours(const vector<vector<Point>> &contours, int min_radius_pixel, int max_radius_pixel, float ellipse_error_pixel,
									  QList<QList<float>> &ellipse_pars);

	static float ErrorDROfEllipseFit(float center_x, float center_y, float ellipse_a, float ellipse_b, float ellipse_angle,
									 float x, float y);
	static float LeastSquareErrorOfEllipseFit(float center_x, float center_y, float ellipse_a, float ellipse_b, float ellipse_angle,
											  float x, float y);

	// ��һ��ɸѡ�������ڱ��������Բ��
	static bool FilterEllipseContoursForCodePoint(const Mat &image_mat, float ratio_k, float ratio_k1, float ratio_k2,
												  QList<QList<float>> &ellipse_pars);

	static bool EllipseGrayJudgeForCodePoint(const Mat &image_mat, float center_x, float center_y,
											 float ellipse_a, float ellipse_b, float angle_in_pi, float ratio_k, bool white_on_black = true,
											 float *out_foreground_mean = nullptr, float *out_background_mean = nullptr, float *out_foreground_stdDev = nullptr, float *out_background_stdDev = nullptr);

	// ��ȡ����ֵ�����ָ�룬
	static int *ReturnDefualtIdArray(int &array_size, CodePointBitesType code_bites_type = CodeBites15);

	// ����,��������
	static int Decoding20140210(const Mat &image_mat, float center_x, float center_y, float ellipse_a, float ellipse_b, float angle_in_pi,
								float ratio_k1 = 2.4, float ratio_k2 = 4, MarkPointColorType color_type = BlackDownWhiteUp, CodePointBitesType code_bites_type = CodeBites15);

	static bool CalculateRealCodeID20140210(QList<int> in_put_code_list, QList<int> &out_put_code_list, int &out_put_code_ID);
	// �Ǳ�����ж�
	static bool UncodePointCheck(const Mat &image_mat, float center_x, float center_y, float ellipse_a, float ellipse_b, float angle_in_pi,
								 float ratio_k = 2, MarkPointColorType color_type = BlackDownWhiteUp, CodePointBitesType code_bites_type = CodeBites15);

	// �����ض�λ
	static bool FindSubPixelPosOfCircleCenter20140210(const Mat &image_mat, float center_x, float center_y, float ellipse_a, float ellipse_b,
													  float angle_in_pi, const vector<Point> &contour_points,
													  float &sub_pixel_center_x, float &sub_pixel_center_y, vector<Point2f> *subpixel_edge_points = nullptr, SubPixelPosMethod subPixel_method = NoSubPixel_Match, MarkPointColorType color_type = BlackDownWhiteUp);

	// ��ȡ��Բ�ֲ�����Ȥ����
	static QRect GetEllipseROIRect(const Mat &image_mat, float center_x, float center_y, float ellipse_a, float ellipse_b, float angle_in_pi);

	//**********Otsu����ͼ���ֵ�������ض�ֵ����ֵ,֧�ָ߱���
	static float DICOTSU20140215(const Mat &ori_image, int total_level = 256);

	//******���û��ָ�����ڵװ׵��׵׺ڵ�ʱ���ж�����������
	static MarkPointColorType JudgeTargetColorType(const Mat &sub_mat, float center_x_insubmat, float center_y_insubmat, float ellipse_a, float ellipse_b, float angle_in_pi);

	//******һ�ֻ�þֲ���ֵ�ķŷ��� ��ֵ = (Ŀ������ƽ���Ҷ� + ��������ƽ�ֻҶ�)/2
	static float CalThresholdInSubsetMat(const Mat &sub_mat, float center_x_insubmat, float center_y_insubmat, float ellipse_a, float ellipse_b, float angle_in_pi);

	//*****һ�ֻ�ȡ�ֲ���ֵ�ķ�����������Ե�Ҷ�ƽ��ֵ
	static float CalThresholdInSubsetMat2(const Mat &image_mat, const vector<Point> &contour_points);

	//*********�Ҷ����ķ�����ֲ�����
	static bool CalCentriodBySubsetMat(const Mat &subset_mat, float thresh_old, float &sub_center_x, float &sub_center_y,
									   MarkPointColorType color_type = BlackDownWhiteUp, SubPixelPosMethod subPixel_method = Gray_Centroid);

	//**********������ķ��Ҷ�Ȩ��
	static float CalWeightOfCentriod(float gray_value, float gray_threshold, MarkPointColorType color_type = BlackDownWhiteUp,
									 SubPixelPosMethod subPixel_method = Gray_Centroid);

	//*******�����ĻҶ��ݶȷ�ֵ�ͽǶȣ�G��
	static void CalGrayGradientBySobel(const Mat &image_mat, int i, int j, float *gray_gradient, float *gradient_sita = nullptr);

	//*********�����˲�������������Ե�������
	static void CalCurvatureFromEdgePoints(const vector<Point2f> &edge_points, vector<float> &curvature_vector);

	//************���������ɸѡ���ֵ�����ѡ��������
	static void ReduceBadEllipseFitPoints(vector<Point2f> &edge_points, float center_x, float center_y, float ellipse_a, float ellipse_b, float angle_in_pi);

	//**************ƽ��ֵ****
	static int AverageOfList(const QList<int> &list_value);

	//*****************canny��������Ӧ��ֵOtsu��ȷ������ֵTh2��Th1= 0.5Th2��
	// ���ش�����һ�ȼ�ʱ��䷽�����
	static double OTSUForCannyAdaptiveHighTh(Mat ori_image, int total_level = 256);

	//*********************��ȡ����ԲԲ�ķ�����������뻷��������֮����߶��ϵĻҶ�ֵ******************
	// image_mat-����ԭʼͼ��
	// point1-���뽻��1����
	// point2-���뽻��2����
	// ����һϵ����
	static QList<int> GetALineGrayList(Mat image_mat, QPoint point1, QPoint point2);

	//*************************��ȡһϵ��������ֵ*********************************
	// value_list - ��������
	// ������ֵ
	static int MIdValue(QList<int> value_list);

	//***********������תʮ���ƣ�����list_code2�������������returnʮ����ֵ**********************
	static int Change2To10(QList<int> list_code2);

	//
	static void WirteMatToFile(Mat mat, QString file_name = "mat");

	// ɸѡ��CSI��Բ�궨��� Բ�㷽��㣨��Բ�㣩+ ��ͨ�㣬 vector<int> orident_point_index,3��������index
	static bool FilterEllipseContoursForCSICalibrationPlane(const Mat &image_mat, float ratio_k, float ratio_k1, float ratio_k2,
															QList<QList<float>> &ellipse_pars, vector<int> &orident_point_index);

	// ����ֱ�߷���Ax+By+C=0
	static void LineEquation(Point2f p1, Point2f p2, float &A, float &B, float &C);

	// �����p��ֱ��Ax+By+C=0����d
	static float PointTpLineDistance(Point2f p, float A, float B, float C);

	// ����㵽��֮�����
	static float PointToPointDistance(Point2f p1, Point2f p2);
};

#endif // IMAGEDETECTMETHOD_H
