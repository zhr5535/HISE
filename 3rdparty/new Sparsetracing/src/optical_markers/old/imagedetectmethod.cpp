#include "imagedetectmethod.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <QRect>
#include <QFileDialog>
#include <QTextStream>
#include <qalgorithms.h>
#include <execution>
#include <algorithm>
#include <oneapi/tbb/parallel_for.h>

#include "../internal/homomorphic_filter.h"
#include "../marker_functions.hpp"
namespace cexpr
{
	template <typename _Ty = float>
	inline static constexpr auto pi = _Ty(3.1415926);
}

//**********************************2021-6-22
bool ImageDetectMethod::_General_circular_marker_detector(
	image_wrapper_t image,
	Mat &code_point_mat, Mat &uncode_point_mat,
	float min_radius, float max_radius,
	float radius_ratios[3],
	float ellipse_error_pixel /*=0.5*/,
	MarkPointColorType color_type /*= BlackDownWhiteUp*/,
	CodePointBitesType code_bites_type /*=CodeBites15*/,
	DetectContoursMethod image_process_method /*= OTSU_Method*/,
	SubPixelPosMethod subpixel_pos_method /*=Gray_Centroid*/,
	bool enable_homofilter)
{
	if (!image.data)
		return false;

	/*@{*/ // 1.图像预处理
	Mat ori_image(image.rows, image.cols, dgelom::internal::make_dtype<uint8_t>, image.data);
	Mat processed_image_mat;
	ImagePreprocess(ori_image, processed_image_mat);

	if (enable_homofilter)
	{
		processed_image_mat = dgelom::homomorphic_filter(processed_image_mat);
	}
	/*@}*/

	/*@{*/ // 2.边缘检测, 存入闭合轮廓
	vector<vector<Point>> contours;
	DetectClosedContours(processed_image_mat, contours, image_process_method);

	/*@{*/							  // 3.轮廓筛选, 尺寸，形状等准则
	QList<QList<float>> ellipse_pars; // dim: Nx6  center_x,center_y,r_a,r_b,angle_inPI,ellipse_error,contours_index,ID,code_type(0- uncode,1- code)
	FilterEllipseContours(contours, min_radius, max_radius, ellipse_error_pixel, ellipse_pars);

	/*@{*/ // 4.进一步筛选 ，用于编码点和非编码
	const auto ratio_k = radius_ratios[0], ratio_k1 = radius_ratios[1], ratio_k2 = radius_ratios[2];
	FilterEllipseContoursForCodePoint(processed_image_mat, ratio_k, ratio_k1, ratio_k2, ellipse_pars);

	/*@{*/ // 5.编码点与非编码点进行区分,解码
	const auto _IDs = _Get_decode_id_b15();
	const auto default_id_array_size = _IDs.size();
	const auto default_id_array_ptr = _IDs.data();
	if (default_id_array_ptr == nullptr)
	{
		return false;
	}

	int uncodePoint_id = 0;
	for (int i = 0; i < ellipse_pars.size(); i++)
	{
		const auto _Code_id = Decoding20140210(
			processed_image_mat,
			ellipse_pars[i][0], ellipse_pars[i][1],
			ellipse_pars[i][2], ellipse_pars[i][3],
			ellipse_pars[i][4],
			ratio_k1, ratio_k2, color_type, code_bites_type);

		bool _Is_code_point = false;
		if (_Code_id >= 0)
		{
			auto _It = find(execution::par, _IDs.cbegin(), _IDs.cend(), _Code_id);
			if (_It != _IDs.cend())
			{
				_Is_code_point = true;
				ellipse_pars[i].append(_It - _IDs.cbegin());
				ellipse_pars[i].append(1);
			}
		}

		if (_Is_code_point == false)
		{
			const auto _Is_uncoded = UncodePointCheck(
				processed_image_mat, ellipse_pars[i][0],
				ellipse_pars[i][1], ellipse_pars[i][2],
				ellipse_pars[i][3], ellipse_pars[i][4],
				ratio_k, color_type, code_bites_type);

			if (_Is_uncoded == false)
			{
				ellipse_pars[i].append(uncodePoint_id);
				ellipse_pars[i].append(0);
				uncodePoint_id++;
			}
			else
			{
				ellipse_pars.removeAt(i);
				i--;
			}
		}
	}

	//************************亚像素定位
	// ellipse_pars - n*6:
	// center_x, center_y, r_a, r_b, angle_inPI, ellipse_error, contours_index, ID, code_type(0- uncode,1- code)
	vector<vector<Point2f>> subpixel_edge_contours(ellipse_pars.size());
#ifdef OM_USE_TBB
	tbb::parallel_for(size_t(0), size_t(ellipse_pars.size()), [&](auto i)
					  {
		float sub_pixel_x, sub_pixel_y;
		vector<Point2f> edge_contour;
		FindSubPixelPosOfCircleCenter20140210(
			processed_image_mat,
			ellipse_pars[i][0], ellipse_pars[i][1],
			ellipse_pars[i][2], ellipse_pars[i][3],
			ellipse_pars[i][4],
			contours[ellipse_pars[i][6]],
			sub_pixel_x, sub_pixel_y,
			&edge_contour, subpixel_pos_method, color_type);
		ellipse_pars[i][0] = sub_pixel_x;
		ellipse_pars[i][1] = sub_pixel_y;
		subpixel_edge_contours[i] = std::move(edge_contour); });
#else
	for (int i = 0; i < ellipse_pars.size(); i++)
	{
		float sub_pixel_x, sub_pixel_y;
		vector<Point2f> edge_contour;
		FindSubPixelPosOfCircleCenter20140210(
			processed_image_mat,
			ellipse_pars[i][0], ellipse_pars[i][1],
			ellipse_pars[i][2], ellipse_pars[i][3],
			ellipse_pars[i][4],
			contours[ellipse_pars[i][6]],
			sub_pixel_x, sub_pixel_y,
			&edge_contour, subpixel_pos_method, color_type);
		ellipse_pars[i][0] = sub_pixel_x;
		ellipse_pars[i][1] = sub_pixel_y;
		subpixel_edge_contours[i] = std::move(edge_contour);
	}
#endif

	// 输出  code_point_mat,  uncode_point_mat
	code_point_mat = Mat();
	code_point_mat.reserve(100);
	uncode_point_mat = Mat();
	uncode_point_mat.reserve(100);
	for (int i = 0; i < ellipse_pars.size(); i++)
	{
		// id, x, y, quality, r_a, r_b, angle_in_pi
		float a[7] = {
			ellipse_pars[i][7], ellipse_pars[i][0], ellipse_pars[i][1],
			ellipse_pars[i][5], ellipse_pars[i][2], ellipse_pars[i][3],
			ellipse_pars[i][4]};
		auto mat = Mat(1, 7, dgelom::internal::make_dtype<float>, a);
		if (ellipse_pars[i][8] > 0)
		{
			code_point_mat.push_back(mat);
		}
		else
		{
			uncode_point_mat.push_back(mat);
		}
	}

	return true;
}

bool ImageDetectMethod::ImagePreprocess(const Mat &ori_image_mat, Mat &processed_image_mat)
{
	constexpr auto sigma = 1.5; // Default is 1.5
	GaussianBlur(ori_image_mat, processed_image_mat, Size(3, 3), sigma, sigma);
	return true;
}

bool ImageDetectMethod::DetectClosedContours(const Mat &ori_image_mat, vector<vector<Point>> &contours, DetectContoursMethod image_process_method)
{
	// vector<vector<Point>> contours;
	vector<Vec4i> hierarchy;

	Mat threshold_image;
	if (image_process_method == OTSU_Method)
	{
		threshold(ori_image_mat, threshold_image, 0, 255.0, cv::THRESH_OTSU | cv::THRESH_BINARY_INV); // 对比自适应阈值化
		findContours(threshold_image, contours, hierarchy, cv::RETR_LIST, cv::CHAIN_APPROX_NONE, Point(0, 0));
	}
	else if (image_process_method == ADAPTIVE_THRESH_Method)
	{
		adaptiveThreshold(ori_image_mat, threshold_image, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY, 5, 0); // 自适应二值化
		findContours(threshold_image, contours, hierarchy, cv::RETR_LIST, cv::CHAIN_APPROX_NONE, Point(0, 0));
	}
	else if (image_process_method == CANNY_Method)
	{
		Canny(ori_image_mat, threshold_image, 20, 20 * 3);
		findContours(threshold_image, contours, hierarchy, cv::RETR_LIST, cv::CHAIN_APPROX_NONE, Point(0, 0));
	}
	else if (image_process_method == CANNY_Self_FindContours_Method)
	{
		Canny(ori_image_mat, threshold_image, 20, 20 * 3);

		// 边界追踪
		Mat edge_result_mat = Mat::zeros(ori_image_mat.size(), CV_8U);
		for (int i = 0; i < threshold_image.rows; i++)
		{
			auto _threshold_image_ptr = threshold_image.ptr<uchar>(i);
			auto _edge_result_mat_ptr = edge_result_mat.ptr<uchar>(i);

			for (int j = 0; j < threshold_image.cols; j++)
			{
				if (_threshold_image_ptr[j] > 0 && _edge_result_mat_ptr[j] == 0)
				{
					vector<Point> edge_point;
					// TraceEdge(i,j,&edge_point,&threshold_image,&edge_result_mat);

					bool is_contour_closed = false;
					_edge_result_mat_ptr[j] = 255;
					edge_point.push_back(Point(j, i));
					TraceEdge(i, j, i, j, &edge_point, &threshold_image, &edge_result_mat, &is_contour_closed);

					if (is_contour_closed == true)
					{
						contours.push_back(edge_point);
					}
				}
			}
		}
	}
	else if (image_process_method == Self_CANNY_Method)
	{
		SelfCannyMethod(ori_image_mat, threshold_image, 20, 20 * 3, contours);
	}
	return true;
}

bool ImageDetectMethod::TraceEdge(
	int start_y, int start_x, int y, int x,
	vector<Point> *edge_vector, Mat *cannyed_image_mat, Mat *result_mat,
	bool *is_contour_closed)
{
	// 对8邻域像素进行查询,搜索方案 直角边，斜对角边
	// 	int xNum[8] = {1,1,0,-1,-1,-1,0,1};
	// 	int yNum[8] = {0,1,1,1,0,-1,-1,-1};
	int xNum[8] = {1, 0, -1, 0, 1, -1, -1, 1};
	int yNum[8] = {0, 1, 0, -1, 1, 1, -1, -1};
	// 	int xNum[8] = {1,-1,-1, 1, 1, 0,-1, 0};
	// 	int yNum[8] = {1, 1,-1,-1, 0, 1, 0,-1 };

	int yy, xx;

	for (int k = 0; k < 8; k++)
	{
		yy = y + yNum[k];
		xx = x + xNum[k];

		if (xx < 1 || xx > cannyed_image_mat->cols - 1 || yy < 1 || yy > cannyed_image_mat->rows - 1)
		{
			continue;
		}

		//
		if (*is_contour_closed == true)
		{
			return true;
		}
		// 递归深度限制，不然会堆栈溢出
		if (edge_vector->size() > 2000)
		{
			return false;
		}

		uchar *_cannyed_image_mat_ptr = cannyed_image_mat->ptr<uchar>(yy);
		uchar *_result_mat_ptr = result_mat->ptr<uchar>(yy);

		if (_cannyed_image_mat_ptr[xx] > 0 && _result_mat_ptr[xx] == 0)
		{

			// 该点设为边界点
			_result_mat_ptr[xx] = 255;
			edge_vector->push_back(Point(xx, yy));

			// 判断是否不是中间有跳跃
			if (edge_vector->size() > 3)
			{
				if (abs(edge_vector->at(edge_vector->size() - 1).x - edge_vector->at(edge_vector->size() - 2).x) +
						abs(edge_vector->at(edge_vector->size() - 1).y - edge_vector->at(edge_vector->size() - 2).y) >
					2)
				{
					return false;
				}
			}

			// 以该点为中心再进行跟踪
			if (TraceEdge(start_y, start_x, yy, xx, edge_vector, cannyed_image_mat, result_mat, is_contour_closed) == false)
			{
				return false;
			}
		}
	}

	// 判断轮廓闭合
	if (edge_vector->size() > 2 && sqrt(double(edge_vector->at(edge_vector->size() - 1).x - start_x) * double(edge_vector->at(edge_vector->size() - 1).x - start_x) + double(edge_vector->at(edge_vector->size() - 1).y - start_y) * double(edge_vector->at(edge_vector->size() - 1).y - start_y)) < 1.5)
	{
		*is_contour_closed = true;
	}

	return true;
}

bool ImageDetectMethod::SelfCannyMethod(
	const Mat &ori_image_mat, Mat &output_image_mat,
	float canny_low_thresh, float canny_high_thresh,
	vector<vector<Point>> &edge_point_list)
{
	Mat src;
	ori_image_mat.convertTo(src, CV_32F);

	const int mat_rows = ori_image_mat.rows;
	const int mat_cols = ori_image_mat.cols;

	Mat M_gradient = Mat::zeros(mat_rows, mat_cols, CV_32F);
	Mat sita_mat = Mat::zeros(mat_rows, mat_cols, CV_32F);

	if (canny_low_thresh > canny_high_thresh)
		std::swap(canny_low_thresh, canny_high_thresh);
	const int low = cvFloor(canny_low_thresh);
	const int high = cvFloor(canny_high_thresh);

	cv::Mat dx(src.rows, src.cols, CV_32F);
	cv::Mat dy(src.rows, src.cols, CV_32F);

	cv::Sobel(src, dx, CV_32F, 1, 0, 3, 1, 0, cv::BORDER_REPLICATE);
	cv::Sobel(src, dy, CV_32F, 0, 1, 3, 1, 0, cv::BORDER_REPLICATE);

	for (int i = 1; i < src.rows - 1; i++)
	{
		float *_norm = M_gradient.ptr<float>(i);
		float *_sita = sita_mat.ptr<float>(i);
		float *_dx = dx.ptr<float>(i);
		float *_dy = dy.ptr<float>(i);

		for (int j = 1; j < src.cols - 1; j++)
		{
			_norm[j] = sqrt(_dx[j] * _dx[j] + _dy[j] * _dy[j]);
			_sita[j] = atan2(_dy[j], _dx[j]);
		}
	}

	Mat NMS_mat = Mat::zeros(mat_rows, mat_cols, CV_32F);
	for (int i = 1; i < mat_rows - 1; i++)
	{
		float *_NMS = NMS_mat.ptr<float>(i);
		float *_sita = sita_mat.ptr<float>(i);
		float *_norm = M_gradient.ptr<float>(i);
		float *_norm_up = M_gradient.ptr<float>(i - 1);
		float *_norm_down = M_gradient.ptr<float>(i + 1);
		for (int j = 1; j < mat_cols - 1; j++)
		{
			float m1, m2; // 梯度方向相邻的两个梯度值
			constexpr auto pi_d8 = cexpr::pi<> / 8;
			if (abs(_sita[j]) <= pi_d8 || abs(_sita[j]) >= 7 * pi_d8)
			{
				m1 = _norm[j + 1];
				m2 = _norm[j - 1];
			}
			else if ((_sita[j] >= pi_d8 && _sita[j] < 3 * pi_d8) ||
					 (_sita[j] > -7 * pi_d8 && _sita[j] < -5 * pi_d8))
			{
				m1 = _norm_down[j + 1];
				m2 = _norm_up[j - 1];
			}
			else if (abs(_sita[j]) >= 3 * pi_d8 && abs(_sita[j]) <= 5 * pi_d8)
			{
				m1 = _norm_down[j];
				m2 = _norm_up[j];
			}
			else
			{
				m1 = _norm_up[j + 1];
				m2 = _norm_down[j - 1];
			}
			if (_norm[j] >= m1 && _norm[j] >= m2)
			{
				_NMS[j] = float(_norm[j]);
			}
		}
	}

	///////////双阈值,指定阈值
	// 	Mat low_threshold_mat;
	// 	Mat high_threhold_mat;
	// 	threshold(NMS_mat, low_threshold_mat, low, 255, THRESH_BINARY);
	// 	threshold(NMS_mat, high_threhold_mat, high, 255, THRESH_BINARY);

	/////canny自适应阈值
	////otsu法，先将非极大值抑制后梯度的值进行，归一化（整数化），分成64等级，到高阈值th2，th1 = 0.5th2；
	int N = 64;
	Mat norm_NMS_mat = Mat(mat_rows, mat_cols, CV_8U);
	double max_value;
	minMaxLoc(NMS_mat, nullptr, &max_value);
	double per_d = (max_value + 0.000001) / N;

	for (int i = 0; i < mat_rows; i++)
	{
		float *_NMS_mat = NMS_mat.ptr<float>(i);
		uchar *_norm_NMS_mat = norm_NMS_mat.ptr<uchar>(i);
		for (int j = 0; j < mat_cols; j++)
		{
			_norm_NMS_mat[j] = floor(_NMS_mat[j] / per_d);
		}
	}

	double max_level = OTSUForCannyAdaptiveHighTh(norm_NMS_mat, N);
	double high_th = max_level * per_d;
	double low_th = 0.5 * high_th;
	Mat low_threshold_mat;
	Mat high_threhold_mat;
	threshold(NMS_mat, low_threshold_mat, low_th, 255, THRESH_BINARY);
	threshold(NMS_mat, high_threhold_mat, high_th, 255, THRESH_BINARY);

	Mat search_sign = Mat::zeros(mat_rows, mat_cols, CV_8U);
	for (int i = 0; i < mat_rows; i++)
	{
		for (int j = 0; j < mat_cols; j++)
		{
			if (search_sign.at<uchar>(i, j) == 0 && high_threhold_mat.at<float>(i, j) > 0)
			{
				vector<Point> edge_point;
				bool is_contour_closed = false;
				search_sign.at<uchar>(i, j) = 255;
				edge_point.push_back(Point(j, i));

				FindCannyEdge(i, j, i, j, &low_threshold_mat, &high_threhold_mat, &search_sign, &edge_point, &is_contour_closed);

				if (is_contour_closed == true)
				{
					edge_point_list.push_back(edge_point);
				}
			}
		}
	}
	output_image_mat = search_sign;
	return true;
}

bool ImageDetectMethod::FindCannyEdge(int start_y, int start_x, int y, int x,
									  Mat *low_threshold_mat, Mat *high_threshold_mat, Mat *search_sign_mat,
									  vector<Point> *edge_point_vector, bool *is_contour_closed)
{
	// 对8邻域像素进行查询,搜索方案 直角边，斜对角边
	//  	int xNum[8] = {1,1,0,-1,-1,-1,0,1};
	//  	int yNum[8] = {0,1,1,1,0,-1,-1,-1};
	int xNum[8] = {1, 0, -1, 0, 1, -1, -1, 1};
	int yNum[8] = {0, 1, 0, -1, 1, 1, -1, -1};
	// 	int xNum[8] = {1,-1,-1, 1, 1, 0,-1, 0};
	// 	int yNum[8] = {1, 1,-1,-1, 0, 1, 0,-1 };

	constexpr int edge_size_T = 2000;
	bool is_found_next_point = false;
	for (int k = 0; k < 8; k++)
	{
		const auto i = y + yNum[k], j = x + xNum[k];
		if (j < 0 || j > search_sign_mat->cols - 1 || i < 0 || i > search_sign_mat->rows - 1)
		{
			continue;
		}
		// 判断轮廓闭合
		if (*is_contour_closed == true)
		{
			return true;
		}
		if (edge_point_vector->size() > edge_size_T)
		{
			return false;
		}

		float *_high_threshold_mat_ptr = high_threshold_mat->ptr<float>(i);
		uchar *_search_sign_mat_ptr = search_sign_mat->ptr<uchar>(i);

		if (_high_threshold_mat_ptr[j] > 0 && _search_sign_mat_ptr[j] == 0)
		{
			if (edge_point_vector->size() > 3)
			{
				if (abs(edge_point_vector->at(edge_point_vector->size() - 1).x - edge_point_vector->at(edge_point_vector->size() - 2).x) +
						abs(edge_point_vector->at(edge_point_vector->size() - 1).y - edge_point_vector->at(edge_point_vector->size() - 2).y) >
					2)
				{
					return false;
				}
			}

			is_found_next_point = true;
			_search_sign_mat_ptr[j] = 255;
			edge_point_vector->push_back(Point(j, i));

			if (FindCannyEdge(start_y, start_x, i, j, low_threshold_mat, high_threshold_mat, search_sign_mat, edge_point_vector, is_contour_closed) == false)
			{
				return false;
			}
		}
	}

	if (is_found_next_point == false)
	{
		for (int k = 0; k < 8; k++)
		{
			const auto i = y + yNum[k], j = x + xNum[k];
			if (j < 0 || j > search_sign_mat->cols - 1 || i < 0 || i > search_sign_mat->rows - 1)
			{
				continue;
			}
			// 判断轮廓闭合
			if (*is_contour_closed == true)
			{
				return true;
			}
			if (edge_point_vector->size() > edge_size_T)
			{
				return false;
			}

			float *_low_threshold_mat_ptr = low_threshold_mat->ptr<float>(i);
			uchar *_search_sign_mat_ptr = search_sign_mat->ptr<uchar>(i);
			if (_search_sign_mat_ptr[j] > 0)
			{
				continue;
			}

			if (_low_threshold_mat_ptr[j] > 0 && _search_sign_mat_ptr[j] == 0)
			{
				if (edge_point_vector->size() > 3)
				{
					if (abs(edge_point_vector->at(edge_point_vector->size() - 1).x - edge_point_vector->at(edge_point_vector->size() - 2).x) +
							abs(edge_point_vector->at(edge_point_vector->size() - 1).y - edge_point_vector->at(edge_point_vector->size() - 2).y) >
						2)
					{
						return false;
					}
				}

				if (edge_point_vector->size() > edge_size_T)
				{
					break;
				}

				is_found_next_point = true;
				_search_sign_mat_ptr[j] = 255;
				edge_point_vector->push_back(Point(j, i));

				if (FindCannyEdge(start_y, start_x, i, j, low_threshold_mat, high_threshold_mat, search_sign_mat, edge_point_vector, is_contour_closed) == false)
				{
					return false;
				}
			}
		}
	}

	// 判断轮廓闭合
	if (edge_point_vector->size() > 2 && sqrt(double(edge_point_vector->at(edge_point_vector->size() - 1).x - start_x) * double(edge_point_vector->at(edge_point_vector->size() - 1).x - start_x) + double(edge_point_vector->at(edge_point_vector->size() - 1).y - start_y) * double(edge_point_vector->at(edge_point_vector->size() - 1).y - start_y)) < 1.5)
	{
		*is_contour_closed = true;
	}

	return true;
}

bool ImageDetectMethod::FilterEllipseContours(const vector<vector<Point>> &contours,
											  int min_radius_pixel, int max_radius_pixel, float ellipse_error_pixel,
											  QList<QList<float>> &ellipse_pars)
{
	ellipse_pars.clear();
#ifdef OM_USE_TBB
	auto is_radius_fit = [&](auto rect)
	{
		return 2 * min_radius_pixel < rect.size.width ||
			   2 * max_radius_pixel < rect.size.height;
	};
	tbb::parallel_for(size_t(0), contours.size(), [&](auto i)
					  {
		decltype(auto) contour = contours[i];
		if (const auto count = contour.size(); count >= 6) {//At least 6 pixels for ellipse fitting
			const auto rect = fitEllipse(contour);
			if (is_radius_fit(rect) && (rect.size.height < rect.size.width * 2)) {
				auto whole_error = float(0);
				for (int j = 0; j < count; j++) {
					whole_error += ErrorDROfEllipseFit(rect.center.x, rect.center.y,
						rect.size.width * 0.5, rect.size.height * 0.5,
						rect.angle * cexpr::pi<> / 180, contour[j].x, contour[j].y);
				}
				if (const auto aver_error = whole_error / count; aver_error <= ellipse_error_pixel) {
					ellipse_pars.append({ rect.center.x, rect.center.y,
						rect.size.width * 0.5f,rect.size.height * 0.5f,
						rect.angle * cexpr::pi<> / 180, aver_error, float(i) });
				}
			}
		} });
#else
	for (int i = 0; i < contours.size(); i++)
	{
		const int count = contours[i].size();
		if (count < 6)
		{ // At least 6 pixels for ellipse fitting
			continue;
		}

		// 圆度准则: 张维忠博士论文, C = P*P/A, 为圆时 C = 4pi， 应满足 1.0<= C/4/pi <=1.5
		/*const double length = arcLength(contours[i], true);
		const double area = contourArea(contours[i]);
		float circular_degree = length*length/area;
		if (circular_degree/4/cexpr::pi<><1.0||circular_degree/4/cexpr::pi<>>1.5) {
			continue;
		}*/

		// 椭圆拟合
		const auto rect = fitEllipse(contours[i]);
		// 半径大小判断
		if (2 * min_radius_pixel >= rect.size.width || 2 * max_radius_pixel <= rect.size.height)
		{
			continue;
		}

		// 长短轴比例判断
		if (rect.size.height / rect.size.width > 2)
		{
			continue;
		}

		// 最小二乘拟合误差判断，半径差值
		float whole_error = 0;
		for (int j = 0; j < count; j++)
		{
			whole_error += ErrorDROfEllipseFit(rect.center.x, rect.center.y,
											   rect.size.width * 0.5, rect.size.height * 0.5,
											   rect.angle * cexpr::pi<> / 180,
											   float(contours[i][j].x), float(contours[i][j].y));
		}
		const float aver_error = whole_error / count;
		if (aver_error < ellipse_error_pixel)
		{
			QList<float> ellipse_par_list;
			ellipse_par_list << rect.center.x << rect.center.y;
			ellipse_par_list << rect.size.width * 0.5 << rect.size.height * 0.5;
			ellipse_par_list << rect.angle * cexpr::pi<> / 180;
			ellipse_par_list << aver_error << i;
			ellipse_pars.append(ellipse_par_list);
		}
	}
#endif
	return true;
}

float ImageDetectMethod::ErrorDROfEllipseFit(float center_x, float center_y, float ellipse_a, float ellipse_b,
											 float ellipse_angle_in_pi, float x, float y)
{
	// 坐标转换，图像坐标系转椭圆坐标系      x^2/a^2 + y^2/b^2 =1
	const auto tr_x = (x - center_x) * cos(ellipse_angle_in_pi) + (y - center_y) * sin(ellipse_angle_in_pi);
	const auto tr_y = -(x - center_x) * sin(ellipse_angle_in_pi) + (y - center_y) * cos(ellipse_angle_in_pi);

	// 计算拟合误差，半径差作为拟合误差
	const auto alfa = atan2(tr_y, tr_x);

	float r = ellipse_a * ellipse_b / sqrt(ellipse_a * ellipse_a * sin(alfa) * sin(alfa) + ellipse_b * ellipse_b * cos(alfa) * cos(alfa));
	float delta_r = sqrt(tr_x * tr_x + tr_y * tr_y) - r;

	return abs(delta_r);
}

float ImageDetectMethod::LeastSquareErrorOfEllipseFit(float center_x, float center_y, float ellipse_a, float ellipse_b,
													  float ellipse_angle_in_pi, float x, float y)
{
	// 坐标转换，图像坐标系转椭圆坐标系          x^2/a^2 + y^2/b^2 =1
	float tr_x = (x - center_x) * cos(ellipse_angle_in_pi) + (y - center_y) * sin(ellipse_angle_in_pi);
	float tr_y = -(x - center_x) * sin(ellipse_angle_in_pi) + (y - center_y) * cos(ellipse_angle_in_pi);

	// 计算拟合误差，最小二乘拟合误差
	float e = tr_x * tr_x / ellipse_a / ellipse_a + tr_y * tr_y / ellipse_b / ellipse_b - 1;

	return e * e;
}

bool ImageDetectMethod::FilterEllipseContoursForCodePoint(const Mat &image_mat, float ratio_k, float ratio_k1, float ratio_k2,
														  QList<QList<float>> &ellipse_pars)
{
	// 进一步筛选出用编码点解码的点
	// 灰度准则
	for (int i = 0; i < ellipse_pars.size(); i++)
	{
		bool is_gray_judge = EllipseGrayJudgeForCodePoint(image_mat, ellipse_pars[i][0], ellipse_pars[i][1],
														  ellipse_pars[i][2], ellipse_pars[i][3], ellipse_pars[i][4], ratio_k);
		if (is_gray_judge == false)
		{
			ellipse_pars.removeAt(i);
			i--;
		}
	}

	// 剔除小的编码带影响: 筛选出标志点进行解码,位置关系，剔除外圆环和小编码带
	for (int i = 0; i < ellipse_pars.size() - 1; i++)
	{
		for (int j = i + 1; j < ellipse_pars.size(); j++)
		{
			const float x1 = ellipse_pars[i][0], y1 = ellipse_pars[i][1];
			const float x2 = ellipse_pars[j][0], y2 = ellipse_pars[j][1];
			const float distance = std::hypot(x1 - x2, y1 - y2);
			if (distance < min(ellipse_pars[i][3], ellipse_pars[j][3]))
			{
				if (ellipse_pars[i][3] > ellipse_pars[j][3])
				{
					ellipse_pars.removeAt(i);
					i--;
					break;
				}
				else
				{
					ellipse_pars.removeAt(j);
					j--;
				}
			}
			else if (distance < min(ellipse_pars[i][3], ellipse_pars[j][3]) * ratio_k2)
			{
				if (ellipse_pars[i][5] > ellipse_pars[j][5])
				{
					ellipse_pars.removeAt(i);
					i--;
					break;
				}
				else
				{
					ellipse_pars.removeAt(j);
					j--;
				}
			}
		}
	}
	return true;
}

bool ImageDetectMethod::EllipseGrayJudgeForCodePoint(const Mat &image_mat, float center_x, float center_y,
													 float ellipse_a, float ellipse_b, float angle_in_pi, float ratio_k, bool white_on_black,
													 float *out_foreground_mean, float *out_background_mean,
													 float *out_foreground_stdDev, float *out_background_stdDev)
{
	float ellipse_a2 = ellipse_a * ratio_k;
	float ellipse_b2 = ellipse_b * ratio_k;

	int i_top = 0;
	int i_bottom = 0;
	int j_left = 0;
	int j_right = 0;
	if (int(center_y - ellipse_b2) < 0)
	{
		i_top = 0;
	}
	else
		i_top = int(center_y - ellipse_b2);
	if (int(center_y + ellipse_b2) > image_mat.rows - 1)
	{
		i_bottom = 0;
	}
	else
		i_bottom = int(center_y + ellipse_b2);
	if (int(center_x - ellipse_b2) < 0)
	{
		j_left = 0;
	}
	else
		j_left = int(center_x - ellipse_b2);
	if (int(center_x + ellipse_b2) > image_mat.cols - 1)
	{
		j_right = 0;
	}
	else
		j_right = int(center_x + ellipse_b2);

	float foreground_mean = 0;
	float background_mean = 0;
	float whole_mean = 0;
	float foreground_stdDev = 0;
	float background_stdDev = 0;
	float whole_stdDev = 0;
	int forground_num = 0;
	int background_num = 0;
	int whole_num = 0;

	QList<float> background_value_list;
	QList<float> foreground_value_list;
	QList<float> whole_value_list;

	for (int i = i_top; i <= i_bottom; i++)
	{
		for (int j = j_left; j <= j_right; j++)
		{
			const auto _image_mat_ptr = image_mat.ptr<uchar>(i);
			// 坐标转换图像坐标系转椭圆坐标系
			float tr_x = (j - center_x) * cos(angle_in_pi) + (i - center_y) * sin(angle_in_pi);
			float tr_y = -(j - center_x) * sin(angle_in_pi) + (i - center_y) * cos(angle_in_pi);

			if (tr_x * tr_x / int(ellipse_a2) / int(ellipse_a2) + tr_y * tr_y / int(ellipse_b2) / int(ellipse_b2) < 1 &&
				tr_x * tr_x / ellipse_a / ellipse_a + tr_y * tr_y / ellipse_b / ellipse_b > 1)
			{
				background_mean += float(_image_mat_ptr[j]);
				background_value_list.append(float(_image_mat_ptr[j]));
				background_num++;

				whole_mean += float(_image_mat_ptr[j]);
				whole_value_list.append(float(_image_mat_ptr[j]));
				whole_num++;
			}
			else if (tr_x * tr_x / int(ellipse_a) / int(ellipse_a) + tr_y * tr_y / int(ellipse_b) / int(ellipse_b) < 1)
			{
				foreground_mean += float(_image_mat_ptr[j]);
				foreground_value_list.append(float(_image_mat_ptr[j]));
				forground_num++;

				whole_mean += float(_image_mat_ptr[j]);
				whole_value_list.append(float(_image_mat_ptr[j]));
				whole_num++;
			}
		}
	}
	foreground_mean = foreground_mean / forground_num;
	background_mean = background_mean / background_num;
	whole_mean = whole_mean / whole_num;

	for (int i = 0; i < background_value_list.size(); i++)
	{
		background_stdDev += (background_value_list[i] - background_mean) * (background_value_list[i] - background_mean);
	}
	for (int i = 0; i < foreground_value_list.size(); i++)
	{
		foreground_stdDev += (foreground_value_list[i] - foreground_mean) * (foreground_value_list[i] - foreground_mean);
	}
	for (int i = 0; i < whole_value_list.size(); i++)
	{
		whole_stdDev += (whole_value_list[i] - whole_mean) * (whole_value_list[i] - whole_mean);
	}
	foreground_stdDev = sqrt(foreground_stdDev / forground_num);
	background_stdDev = sqrt(background_stdDev / background_num);
	whole_stdDev = sqrt(whole_stdDev / whole_num);

	if (out_foreground_mean != nullptr)
	{
		*out_foreground_mean = foreground_mean;
	}
	if (out_background_mean != nullptr)
	{
		*out_background_mean = background_mean;
	}
	if (out_foreground_stdDev != nullptr)
	{
		*out_foreground_stdDev = foreground_stdDev;
	}
	if (out_background_stdDev != nullptr)
	{
		*out_background_stdDev = background_stdDev;
	}

	/// 阈值设置..................???
	float Mt = 90;
	float delta_Mt = 50;
	float fore_stdDev = 100;
	float back_stdDev = 100;

	// 	if (foreground_mean<Mt)
	// 	{
	// 		return false;
	// 	}
	// 	if (background_mean>Mt)
	// 	{
	// 		return false;
	// 	}
	if (abs(foreground_mean - background_mean) < delta_Mt)
	{
		return false;
	}
	if (foreground_stdDev > fore_stdDev)
	{
		return false;
	}
	if (background_stdDev > back_stdDev)
	{
		return false;
	}

	return true;
}

int *ImageDetectMethod::ReturnDefualtIdArray(int &array_size, CodePointBitesType code_bites_type /*=CodeBites15*/)
{
	switch (code_bites_type)
	{
	case CodeBites15:
		array_size = 429;
		return _Get_decode_ids_b15().data();
		break;
	}

	return nullptr;
}

int ImageDetectMethod::Decoding20140210(
	const Mat &image_mat,
	float center_x, float center_y,
	float ellipse_a, float ellipse_b, float angle_in_pi,
	float ratio_k1 /*=2.4*/, float ratio_k2 /*=4 */,
	MarkPointColorType color_type /*= BlackDownWhiteUp*/,
	CodePointBitesType code_bites_type /*=CodeBites15*/)
{
	float ellipse_a2 = ellipse_a * ratio_k1 /**1.2*/;
	float ellipse_b2 = ellipse_b * ratio_k1 /**1.2*/;
	float ellipse_a4 = ellipse_a * ratio_k2;
	float ellipse_b4 = ellipse_b * ratio_k2;

	int i_top = 0;
	int i_bottom = 0;
	int j_left = 0;
	int j_right = 0;
	if (int(center_y - ellipse_b4) < 0)
	{
		return false;
	}
	else
		i_top = int(center_y - ellipse_b4);
	if (int(center_y + ellipse_b4) > image_mat.rows - 1)
	{
		return false;
	}
	else
		i_bottom = int(center_y + ellipse_b4);
	if (int(center_x - ellipse_b4) < 0)
	{
		return false;
	}
	else
		j_left = int(center_x - ellipse_b4);
	if (int(center_x + ellipse_b4) > image_mat.cols - 1)
	{
		return false;
	}
	else
		j_right = int(center_x + ellipse_b4);

	int coed_bites_num;
	switch (code_bites_type)
	{
	case CodeBites15:
		coed_bites_num = 15;
		break;
	case CodeBites12:
		coed_bites_num = 12;
		break;
	}

	// 将圆环分成15段，每段取20个点的值
	constexpr int per_num_point = 20;
	QList<QPointF> code_ring_coords;
	QList<int> code_ring_values;
	for (int i = 0; i < coed_bites_num * per_num_point; i++)
	{
		const auto x = cos(float(i) / coed_bites_num / per_num_point * 2 * cexpr::pi<>);
		const auto y = sin(float(i) / coed_bites_num / per_num_point * 2 * cexpr::pi<>);

		const auto ell_cor_r2_x = ellipse_a2 * x;
		const auto ell_cor_r2_y = ellipse_b2 * y;
		const auto ell_cor_r3_x = ellipse_a4 * x;
		const auto ell_cor_r3_y = ellipse_b4 * y;

		auto car_cor_r2_x = ell_cor_r2_x * cos(angle_in_pi) - ell_cor_r2_y * sin(angle_in_pi) + center_x;
		auto car_cor_r2_y = ell_cor_r2_x * sin(angle_in_pi) + ell_cor_r2_y * cos(angle_in_pi) + center_y;
		auto car_cor_r3_x = ell_cor_r3_x * cos(angle_in_pi) - ell_cor_r3_y * sin(angle_in_pi) + center_x;
		auto car_cor_r3_y = ell_cor_r3_x * sin(angle_in_pi) + ell_cor_r3_y * cos(angle_in_pi) + center_y;

		const auto point1 = QPoint(int(car_cor_r2_x), int(car_cor_r2_y));
		const auto point2 = QPoint(int(car_cor_r3_x), int(car_cor_r3_y));
		const auto gray_list = GetALineGrayList(image_mat, point1, point2);

		code_ring_coords.append(QPointF(x, y));
		code_ring_values.append(MIdValue(gray_list));
	}

	////找单位圆编码的边界，一维卷积模板半宽为3
	int half_length = 5, edge_index = 0;
	float ans_max = 0;
	for (int i = 0; i < code_ring_coords.size() - 2 * half_length; i++)
	{
		float ans = 0;
		for (int j = 0; j < half_length; j++)
		{
			ans += code_ring_values.at(i + j + half_length) - code_ring_values.at(i + j);
		}
		if (ans > ans_max)
		{
			ans_max = ans;
			edge_index = i + half_length;
		}
	}
	if (ans_max / half_length < 20)
	{
		return false;
	}
	//////////解编码，24度顺时针
	double threshold_value = 0 /*= 150*/; // 阈值,要修改
	threshold_value = std::accumulate(code_ring_values.cbegin() + edge_index - half_length, code_ring_values.cbegin() + edge_index + half_length, 0.) / (half_length * 2);

	QList<int> code_in_2;
	for (int i = 0; i < coed_bites_num; i++)
	{
		double mean_gray = 0;
		for (int j = 0; j < per_num_point; j++)
		{
			int index = edge_index + i * per_num_point + j + 1;
			if (index >= code_ring_values.size())
			{
				index -= code_ring_values.size();
			}
			mean_gray += code_ring_values[index];
		}
		mean_gray /= per_num_point;

		if (color_type == BlackDownWhiteUp)
		{
			code_in_2.append(mean_gray > threshold_value ? 1 : 0);
		}
		else
		{
			code_in_2.append(mean_gray > threshold_value ? 0 : 1);
		}
	}
	QList<int> code_out_2;
	auto coded_id = -100;
	CalculateRealCodeID20140210(code_in_2, code_out_2, coded_id);
	return coded_id;
}

bool ImageDetectMethod::CalculateRealCodeID20140210(QList<int> in_put_code_list, QList<int> &out_put_code_list, int &out_put_code_ID)
{
	out_put_code_ID = Change2To10(in_put_code_list);
	out_put_code_list = in_put_code_list;

	int n = in_put_code_list.size();

	for (int i = 1; i <= n - 1; i++)
	{
		QList<int> new_code_list;
		int new_id;
		for (int j = 0; j < n; j++)
		{
			if (i + j <= n - 1)
			{
				new_code_list.append(in_put_code_list.at(i + j));
			}
			else
			{
				new_code_list.append(in_put_code_list.at(i + j - n));
			}
		}
		new_id = Change2To10(new_code_list);
		if (out_put_code_ID > new_id)
		{
			out_put_code_ID = new_id;
			out_put_code_list = new_code_list;
		}
	}
	return true;
}

bool ImageDetectMethod::UncodePointCheck(const Mat &image_mat, float center_x, float center_y, float ellipse_a, float ellipse_b,
										 float angle_in_pi, float ratio_k /*=2*/,
										 MarkPointColorType color_type /*= BlackDownWhiteUp*/, CodePointBitesType code_bites_type /*=CodeBites15*/)
{
	double ellipse_a1 = ellipse_a * 1.1;
	double ellipse_b1 = ellipse_b * 1.1;
	double ellipse_a2 = ellipse_a * (ratio_k - 0.1);
	double ellipse_b2 = ellipse_b * (ratio_k - 0.1);

	int i_top = 0;
	int i_bottom = 0;
	int j_left = 0;
	int j_right = 0;
	if (int(center_y - ellipse_b2) < 0)
	{
		return false;
	}
	else
		i_top = int(center_y - ellipse_b2);
	if (int(center_y + ellipse_b2) > image_mat.rows - 1)
	{
		return false;
	}
	else
		i_bottom = int(center_y + ellipse_b2);
	if (int(center_x - ellipse_b2) < 0)
	{
		return false;
	}
	else
		j_left = int(center_x - ellipse_b2);
	if (int(center_x + ellipse_b2) > image_mat.cols - 1)
	{
		return false;
	}
	else
		j_right = int(center_x + ellipse_b2);

	int coed_bites_num;
	switch (code_bites_type)
	{
	case CodeBites15:
		coed_bites_num = 15;
		break;
	case CodeBites12:
		coed_bites_num = 12;
		break;
	}

	QList<QPointF> ellipse_ring_points;
	QList<int> ellipse_ring_gray_value;

	// 将圆环分成15段，每段取20个点的值
	int per_num_point = 10;

	for (int i = 0; i < coed_bites_num * per_num_point; i++)
	{
		float x = cos(float(i) / coed_bites_num / per_num_point * 2 * cexpr::pi<>);
		float y = sin(float(i) / coed_bites_num / per_num_point * 2 * cexpr::pi<>);

		float ell_cor_r1_x = ellipse_a1 * x;
		float ell_cor_r1_y = ellipse_b1 * y;
		float ell_cor_r2_x = ellipse_a2 * x;
		float ell_cor_r2_y = ellipse_b2 * y;

		float car_cor_r1_x = ell_cor_r1_x * cos(angle_in_pi) - ell_cor_r1_y * sin(angle_in_pi) + center_x;
		float car_cor_r1_y = ell_cor_r1_x * sin(angle_in_pi) + ell_cor_r1_y * cos(angle_in_pi) + center_y;
		float car_cor_r2_x = ell_cor_r2_x * cos(angle_in_pi) - ell_cor_r2_y * sin(angle_in_pi) + center_x;
		float car_cor_r2_y = ell_cor_r2_x * sin(angle_in_pi) + ell_cor_r2_y * cos(angle_in_pi) + center_y;

		QPoint point1 = QPoint(int(car_cor_r1_x), int(car_cor_r1_y));
		QPoint point2 = QPoint(int(car_cor_r2_x), int(car_cor_r2_y));

		QList<int> gray_list = GetALineGrayList(image_mat, point1, point2);
		// int mid_value = MIdValue(gray_list);
		int mid_value = AverageOfList(gray_list);

		ellipse_ring_points.append(QPointF(x, y));
		ellipse_ring_gray_value.append(mid_value);
	}

	////找单位圆编码的边界，一维卷积模板半宽为3
	int half_length = 5;
	int edge_index = 0;
	float ans_max = 0;
	for (int i = 0; i < ellipse_ring_points.size() - 2 * half_length; i++)
	{
		float ans = 0;
		for (int j = 0; j < half_length; j++)
		{
			ans += -ellipse_ring_gray_value.at(i + j) + ellipse_ring_gray_value.at(i + j + half_length);
		}
		if (ans > ans_max)
		{
			ans_max = ans;
			edge_index = i + half_length;
		}
	}

	// 阈值
	int delta_M = 20;
	if (ans_max / half_length > delta_M)
	{
		return false;
	}
	else
		return true;
}

bool ImageDetectMethod::FindSubPixelPosOfCircleCenter20140210(const Mat &image_mat,
															  float center_x, float center_y,
															  float ellipse_a, float ellipse_b,
															  float angle_in_pi,
															  const vector<Point> &contour_points,
															  float &sub_pixel_center_x,
															  float &sub_pixel_center_y,
															  vector<Point2f> *subpixel_edge_points /*= nullptr*/,
															  SubPixelPosMethod subPixel_method /*= NoSubPixel_Match*/,
															  MarkPointColorType color_type /* = BlackDownWhiteUp*/)
{
	if (subPixel_method == NoSubPixel_Match)
	{
		sub_pixel_center_x = center_x;
		sub_pixel_center_y = center_y;

		if (subpixel_edge_points != nullptr)
		{
			// modified by Dgelom Su on 22/Nov/2016
			for (const auto &_pt : contour_points)
			{
				const auto pt2f = Point2f(_pt.x, _pt.y);
				subpixel_edge_points->push_back(pt2f);
			}
		}
		return true;
	}
	else if (subPixel_method == Binary_Centroid || subPixel_method == Gray_Centroid || subPixel_method == Squared_Gray_Centroid)
	{
		// 重心法,局部灰度阈值需要确认
		auto sub_rect = GetEllipseROIRect(image_mat,
										  center_x, center_y,
										  ellipse_a, ellipse_b, angle_in_pi);
		auto sub_mat = image_mat(
			Rect(sub_rect.x(), sub_rect.y(),
				 sub_rect.width() + 1, sub_rect.height() + 1));

		if (color_type == Uncertainty)
		{
			color_type = JudgeTargetColorType(
				sub_mat,
				center_x - sub_rect.x(),
				center_y - sub_rect.y(),
				ellipse_a, ellipse_b, angle_in_pi);
		}

		// 阈值选择方法，还有固定阈值 ，不加阈值0,255,暂时不知道哪种阈值方法较好，需要测试
		auto gray_threshold = DICOTSU20140215(sub_mat); // otsu算法确定阈值
														/*gray_threshold = CalThresholdInSubsetMat(sub_mat,center_x - sub_rect.x(),center_y -sub_rect.y(),
														   ellipse_a,ellipse_b,angle_in_pi);
													   gray_threshold= CalThresholdInSubsetMat2(image_mat,contour_points);*/

#pragma region
		// 计算圆心,选择二值化灰度重心，灰度重心法，灰度平方重心法
// 		float sub_x,sub_y;
// 		if (CalCentriodBySubsetMat(sub_mat,gray_threshold,sub_x,sub_y,color_type,subPixel_method))
// 		{
// 			sub_pixel_center_x = sub_x + sub_rect.x();
// 			sub_pixel_center_y = sub_y + sub_rect.y();
//
//
// 			if (subpixel_edge_points !=nullptr)
// 			{
// 				for (int i=0;i<contour_points.size();i++)
// 				{
// 					subpixel_edge_points->push_back(Point2f(contour_points.at(i).x,contour_points.at(i).y));
// 				}
// 			}
//
// 			return true;
// 		}else
// 			return false;
#pragma endregion
		//**************多阈值灰度形心法、灰度重心法，灰度平方重心法
		int k = 0;
		int d_gray = 4;
		float sub_x = 0, sub_y = 0;
		for (int i = -k; i <= k; ++i)
		{
			float threshold = gray_threshold + i * d_gray;
			float part_sub_x, part_sub_y;
			CalCentriodBySubsetMat(
				sub_mat, threshold,
				part_sub_x, part_sub_y,
				color_type, subPixel_method);
			sub_x += part_sub_x;
			sub_y += part_sub_y;
		}
		// modified by Dgelom Su on Jun/23/2021
		sub_x /= (k << 1 | 1);
		sub_y /= (k << 1 | 1);

		sub_pixel_center_x = sub_x + sub_rect.x();
		sub_pixel_center_y = sub_y + sub_rect.y();

		if (subpixel_edge_points != nullptr)
		{
			// modified by Dgelom Su on Jun/23/2021
			for (const auto &_pt : contour_points)
			{
				const auto pt2f = Point2f(_pt.x, _pt.y);
				subpixel_edge_points->push_back(pt2f);
			}
		}

		return true;
	}
	else if (subPixel_method == Interpolation_Ellipse_Match)
	{
		//<<亚像素边缘定位算法的稳定性分析>> 田原嫄,2010
		// x和y方向插值，效果不好，x-1,x,x+1的梯度fu值x处最大时较好
		// 二次多项式插值，再椭圆拟合
		vector<Point2f> new_contour_points;

		for (int n = 0; n < contour_points.size(); n++)
		{
			int y = contour_points[n].y;
			int x = contour_points[n].x;

			//    0  1  2
			//    3  4  5
			//    6  7  8
			float gray_gradient[9];
			float gradient_sita;
			CalGrayGradientBySobel(image_mat, y - 1, x, &gray_gradient[1]);
			CalGrayGradientBySobel(image_mat, y, x - 1, &gray_gradient[3]);
			CalGrayGradientBySobel(image_mat, y, x, &gray_gradient[4], &gradient_sita);
			CalGrayGradientBySobel(image_mat, y, x + 1, &gray_gradient[5]);
			CalGrayGradientBySobel(image_mat, y + 1, x, &gray_gradient[7]);

			float dx, dy;
			float dx_den = (gray_gradient[3] - 2 * gray_gradient[4] + gray_gradient[5]);
			float dy_den = (gray_gradient[1] - 2 * gray_gradient[4] + gray_gradient[7]);

			if (dx_den == 0 || dy_den == 0)
				continue;

			if (gray_gradient[4] > gray_gradient[1] && gray_gradient[4] > gray_gradient[7] && gray_gradient[4] > gray_gradient[3] && gray_gradient[4] > gray_gradient[5])
			{
				dx = (gray_gradient[3] - gray_gradient[5]) / 2 / dx_den;
				dy = (gray_gradient[1] - gray_gradient[7]) / 2 / dy_den;
			}
			else
				continue;

			new_contour_points.push_back(Point2f(x + dx, y + dy));
		}

		auto new_rRect = fitEllipse(new_contour_points); // fitEllipse只接受float和int类型

		// 根据误差剔除部分点
		ReduceBadEllipseFitPoints(new_contour_points, new_rRect.center.x, new_rRect.center.y,
								  new_rRect.size.width * 0.5, new_rRect.size.height * 0.5, new_rRect.angle * cexpr::pi<> / 180);
		if (new_contour_points.size() < 6)
		{
			return false;
			// 			sub_pixel_center_x = 0;
			// 			sub_pixel_center_y = 0;
			// 			return true;
		}
		new_rRect = fitEllipse(new_contour_points);

		sub_pixel_center_x = new_rRect.center.x;
		sub_pixel_center_y = new_rRect.center.y;

		if (subpixel_edge_points != nullptr)
		{
			*subpixel_edge_points = new_contour_points;
		}
		return true;
	}
	else if (subPixel_method == Interpolation_Rotaion_Ellipse_Match)
	{
		// 二次多项式插值，坐标转换后，只插值梯度方向，后进行坐标转换得到dx，dy
		vector<Point2f> new_contour_points;

		for (int n = 0; n < contour_points.size(); n++)
		{
			int y = contour_points[n].y;
			int x = contour_points[n].x;

			//    0  1  2
			//    3  4  5
			//    6  7  8
			float gray_gradient[9];
			float gradient_sita;
			CalGrayGradientBySobel(image_mat, y - 1, x, &gray_gradient[1]);
			CalGrayGradientBySobel(image_mat, y, x - 1, &gray_gradient[3]);
			CalGrayGradientBySobel(image_mat, y, x, &gray_gradient[4], &gradient_sita);
			CalGrayGradientBySobel(image_mat, y, x + 1, &gray_gradient[5]);
			CalGrayGradientBySobel(image_mat, y + 1, x, &gray_gradient[7]);

			//    2          | -->x
			// 1  0  3       |
			//    4           y
			float boder_gradient_value[5];
			boder_gradient_value[0] = gray_gradient[4];
			// 插值求算

			// 简单求算，与canny算子非极大值抑制
			if ((gradient_sita > -cexpr::pi<> / 8 && gradient_sita < -7 * cexpr::pi<> / 8) || (gradient_sita >= 7 * cexpr::pi<> / 8 && gradient_sita <= cexpr::pi<>))
			{
				boder_gradient_value[1] = gray_gradient[5];
				boder_gradient_value[2] = gray_gradient[3];
				boder_gradient_value[3] = gray_gradient[7];
				boder_gradient_value[4] = gray_gradient[1];
			}
			else if (gradient_sita >= -7 * cexpr::pi<> / 8 && gradient_sita < -5 * cexpr::pi<> / 8)
			{
				boder_gradient_value[1] = gray_gradient[8];
				boder_gradient_value[2] = gray_gradient[0];
				boder_gradient_value[3] = gray_gradient[6];
				boder_gradient_value[4] = gray_gradient[2];
			}
			else if (gradient_sita >= -5 * cexpr::pi<> / 8 && gradient_sita < -3 * cexpr::pi<> / 8)
			{
				boder_gradient_value[1] = gray_gradient[7];
				boder_gradient_value[2] = gray_gradient[1];
				boder_gradient_value[3] = gray_gradient[3];
				boder_gradient_value[4] = gray_gradient[5];
			}
			else if (gradient_sita >= -3 * cexpr::pi<> / 8 && gradient_sita < -cexpr::pi<> / 8)
			{
				boder_gradient_value[1] = gray_gradient[6];
				boder_gradient_value[2] = gray_gradient[2];
				boder_gradient_value[3] = gray_gradient[0];
				boder_gradient_value[4] = gray_gradient[8];
			}
			else if (gradient_sita >= -cexpr::pi<> / 8 && gradient_sita < cexpr::pi<> / 8)
			{
				boder_gradient_value[1] = gray_gradient[3];
				boder_gradient_value[2] = gray_gradient[5];
				boder_gradient_value[3] = gray_gradient[1];
				boder_gradient_value[4] = gray_gradient[7];
			}
			else if (gradient_sita >= cexpr::pi<> / 8 && gradient_sita < 3 * cexpr::pi<> / 8)
			{
				boder_gradient_value[1] = gray_gradient[0];
				boder_gradient_value[2] = gray_gradient[8];
				boder_gradient_value[3] = gray_gradient[2];
				boder_gradient_value[4] = gray_gradient[6];
			}
			else if (gradient_sita >= 3 * cexpr::pi<> / 8 && gradient_sita < 5 * cexpr::pi<> / 8)
			{
				boder_gradient_value[1] = gray_gradient[1];
				boder_gradient_value[2] = gray_gradient[7];
				boder_gradient_value[3] = gray_gradient[5];
				boder_gradient_value[4] = gray_gradient[3];
			}
			else
			{
				boder_gradient_value[1] = gray_gradient[2];
				boder_gradient_value[2] = gray_gradient[6];
				boder_gradient_value[3] = gray_gradient[8];
				boder_gradient_value[4] = gray_gradient[0];
			}

			// 将xy坐标系顺时针旋转sita角，将x正向指向梯度正向（及由圆心向外发射方向）
			float dx, dy;
			float dx_den = boder_gradient_value[1] - 2 * boder_gradient_value[0] + boder_gradient_value[3];
			float dy_den = boder_gradient_value[2] - 2 * boder_gradient_value[0] + boder_gradient_value[4];

			if (dx_den == 0 || dy_den == 0)
			{
				continue;
			}

			if (gray_gradient[4] > gray_gradient[1] && gray_gradient[4] > gray_gradient[7] && gray_gradient[4] > gray_gradient[3] && gray_gradient[4] > gray_gradient[8])
			{
				dx = (boder_gradient_value[1] - boder_gradient_value[3]) / 2 / dx_den;
				dy = (boder_gradient_value[2] - boder_gradient_value[4]) / 2 / dy_den;
			}
			else
				continue;

			// 坐标转换为正常坐标
			float dx_re = dx * cos(gradient_sita) + dy * sin(gradient_sita);
			float dy_re = -dx * sin(gradient_sita) + dy * cos(gradient_sita);

			new_contour_points.push_back(Point2f(x + dx_re, y + dy_re));
		}
		RotatedRect new_rRect = fitEllipse(new_contour_points); // fitEllipse只接受float和int类型

		// 根据误差剔除部分点
		ReduceBadEllipseFitPoints(new_contour_points, new_rRect.center.x, new_rRect.center.y,
								  new_rRect.size.width * 0.5, new_rRect.size.height * 0.5, new_rRect.angle * cexpr::pi<> / 180);
		if (new_contour_points.size() < 6)
		{
			return false;
		}
		new_rRect = fitEllipse(new_contour_points);

		sub_pixel_center_x = new_rRect.center.x;
		sub_pixel_center_y = new_rRect.center.y;

		return true;
	}
	else if (subPixel_method == Gauss_surface_fit_Ellipse_Match)
	{
		// 高斯曲面拟合法
		vector<Point2f> new_contour_points;

		for (int n = 0; n < contour_points.size(); n++)
		{
			int y = contour_points[n].y;
			int x = contour_points[n].x;

			Mat A = Mat(9, 5, CV_64F);
			Mat B = Mat(9, 1, CV_64F);
			for (int i = -1; i < 2; i++)
			{
				for (int j = -1; j < 2; j++)
				{
					A.at<double>((i + 1) * 3 + j + 1, 0) = j * j * double(image_mat.at<uchar>(y + i, x + j));
					A.at<double>((i + 1) * 3 + j + 1, 1) = i * i * double(image_mat.at<uchar>(y + i, x + j));
					A.at<double>((i + 1) * 3 + j + 1, 2) = j * double(image_mat.at<uchar>(y + i, x + j));
					A.at<double>((i + 1) * 3 + j + 1, 3) = i * double(image_mat.at<uchar>(y + i, x + j));
					A.at<double>((i + 1) * 3 + j + 1, 4) = double(image_mat.at<uchar>(y + i, x + j));

					B.at<double>((i + 1) * 3 + j + 1, 0) = double(image_mat.at<uchar>(y + i, x + j)) * log(double(image_mat.at<uchar>(y + i, x + j)));
				}
			}

			Mat X;
			solve(A.t() * A, A.t() * B, X);

			double dx = -X.at<double>(2, 0) / 2 / X.at<double>(0, 0);
			double dy = -X.at<double>(3, 0) / 2 / X.at<double>(1, 0);

			double a[9][5];
			for (int i = 0; i < 9; i++)
			{
				for (int j = 0; j < 5; j++)
				{
					a[i][j] = A.at<double>(i, j);
				}
			}
			double b[9];
			for (int i = 0; i < 9; i++)
			{
				b[i] = B.at<double>(i, 0);
			}
			double xx[5];
			for (int i = 0; i < 5; i++)
			{
				xx[i] = X.at<double>(i, 0);
			}

			if (__isnan(dx) || __isnan(dy))
			{
				continue;
			}

			new_contour_points.push_back(Point2f(x + dx, y + dy));
		}
		RotatedRect new_rRect = fitEllipse(new_contour_points); // fitEllipse只接受float和int类型

		sub_pixel_center_x = new_rRect.center.x;
		sub_pixel_center_y = new_rRect.center.y;

		return true;
	}
	else if (subPixel_method == Surface_fit_Ellipse_Match)
	{
		// <<圆形标志点的亚像素定位及其应用>>  殷永凯，2008
		// 曲面拟合法，对灰度进行曲面拟合，拟合公式 f(x,y)= k1+k2x+k3y+k4x^2+k5xy+k6y^2+k7x^3+k8x^2y+k9xy^2+k10y^3
		// 沿梯度方向求取二阶导为零值
		vector<Point2f> new_contour_points;

		for (int n = 0; n < contour_points.size(); n++)
		{
			int y = contour_points[n].y;
			int x = contour_points[n].x;

			if (x - 3 < 0 || x + 3 > image_mat.cols - 1 || y - 3 < 0 || y + 3 > image_mat.rows - 1)
			{
				continue;
			}

			int half_size = 2;
			int total_pixel = (2 * half_size + 1) * (2 * half_size + 1);
			Mat A = Mat(total_pixel, 10, CV_32F);
			Mat B = Mat(total_pixel, 1, CV_32F);

			for (int i = -half_size; i < half_size + 1; i++)
			{
				const uchar *_image_mat = image_mat.ptr<uchar>(i + y);
				for (int j = -half_size; j < half_size + 1; j++)
				{
					float *_A = A.ptr<float>((i + 2) * 5 + j + 2);
					float *_B = B.ptr<float>((i + 2) * 5 + j + 2);

					_A[0] = 1;
					_A[1] = j;
					_A[2] = i;
					_A[3] = j * j;
					_A[4] = j * i;
					_A[5] = i * i;
					_A[6] = j * j * j;
					_A[7] = j * j * i;
					_A[8] = j * i * i;
					_A[9] = i * i * i;

					_B[0] = float(_image_mat[j + x]);
				}
			}

			// X 10*1: [k1,k2,k3,k4,k5,k6,k7,k8,k9,k10]'
			Mat X;
			solve(A.t() * A, A.t() * B, X);

			float gray_gradient, gradient_sita;
			CalGrayGradientBySobel(image_mat, y, x, &gray_gradient, &gradient_sita);

			Mat XT = X.t();
			float *_XT_ptr = XT.ptr<float>(0);
			// f(x,y)二阶导=0，得到a*r+b=0,求r, d = sita
			// a = 6*(k7*sind^3 +k8*sind^2*cosd + k9sindcosd^2 +k10*cosd^3)
			//  b= 2*(k4*sind^2 + k5*sind*cosd + k6*cosd^2)
			float a = 6 * (_XT_ptr[6] * sin(gradient_sita) * sin(gradient_sita) * sin(gradient_sita) + _XT_ptr[7] * sin(gradient_sita) * sin(gradient_sita) * cos(gradient_sita) + _XT_ptr[8] * sin(gradient_sita) * sin(gradient_sita) * cos(gradient_sita) + _XT_ptr[9] * sin(gradient_sita) * sin(gradient_sita) * cos(gradient_sita));
			float b = 2 * (_XT_ptr[3] * sin(gradient_sita) * sin(gradient_sita) + _XT_ptr[4] * sin(gradient_sita) * cos(gradient_sita) + _XT_ptr[5] * cos(gradient_sita) * cos(gradient_sita));

			float r = -b / a;

			new_contour_points.push_back(Point2f(x + r * cos(gradient_sita), y + r * sin(gradient_sita)));
		}

		RotatedRect new_rRect = fitEllipse(new_contour_points); // fitEllipse只接受float和int类型

		sub_pixel_center_x = new_rRect.center.x;
		sub_pixel_center_y = new_rRect.center.y;

		return true;
	}
	else if (subPixel_method == Gauss_Curve_Fit)
	{
		//<<光学测量中椭圆圆心定位算法研究>>  张虎， 2008
		/////沿梯度方向对灰度梯度幅值进行高斯曲线拟合
		//////先插值出梯度方向的灰度值，再用
		vector<Point2f> new_contour_points;

		for (int n = 0; n < contour_points.size(); n++)
		{
			int y = contour_points[n].y;
			int x = contour_points[n].x;

			if (x - 3 < 0 || x + 3 > image_mat.cols - 1 || y - 3 < 0 || y + 3 > image_mat.rows - 1)
			{
				continue;
			}

			if (x - center_x == 0)
			{
				continue;
			}

			float sita = atan2(y - center_y, x - center_x);
			float k = tan(sita);
			// float k = (y-center_y)/(x-center_x);   //梯度方向
			//  			double sita = gray_gradient_sita.at<double>(y,x);
			//  			double k = tan(sita);
			// double f_a,f_b,f_c,f_d;
			float abs_k = abs(k);

			float positive_gray[3];
			float negative_gray[3];

			int invt = 1;
			if (k > 0)
			{
				invt = -1;
			}
			if (abs_k < 1)
			{

				for (int i = 1; i < 4; i++)
				{
					int a = int(i * abs_k) / 1;
					float lamd2 = i * abs_k - a;
					float lamd1 = 1 - lamd2;

					positive_gray[i - 1] = lamd1 * image_mat.at<uchar>(y - a, x + invt * i) + lamd2 * image_mat.at<uchar>(y - a - 1, x + invt * i);
					negative_gray[i - 1] = lamd1 * image_mat.at<uchar>(y + a, x - invt * i) + lamd2 * image_mat.at<uchar>(y + a + 1, x - invt * i);
				}
			}
			else
			{
				for (int i = 1; i < 4; i++)
				{
					int a = int(i / abs_k) / 1;
					float lamd2 = i / abs_k - a;
					float lamd1 = 1 - lamd2;

					positive_gray[i - 1] = lamd1 * image_mat.at<uchar>(y - i, x + invt * a) + lamd2 * image_mat.at<uchar>(y - i, x + invt * (a + 1));
					negative_gray[i - 1] = lamd1 * image_mat.at<uchar>(y + i, x - invt * a) + lamd2 * image_mat.at<uchar>(y + i, x - invt * (a + 1));
				}
			}

			float gray_value[7]; // 沿灰度梯度方向 ，向外
			if (y < center_y)
			{
				gray_value[0] = negative_gray[2];
				gray_value[1] = negative_gray[1];
				gray_value[2] = negative_gray[0];
				gray_value[3] = image_mat.at<uchar>(y, x);
				gray_value[4] = positive_gray[0];
				gray_value[5] = positive_gray[1];
				gray_value[6] = positive_gray[2];
			}
			else
			{
				gray_value[0] = positive_gray[2];
				gray_value[1] = positive_gray[1];
				gray_value[2] = positive_gray[0];
				gray_value[3] = image_mat.at<uchar>(y, x);
				gray_value[4] = negative_gray[0];
				gray_value[5] = negative_gray[1];
				gray_value[6] = negative_gray[2];
			}

			// 计算差分
			float f_difference[5];
			for (int i = 0; i < 5; i++)
			{
				f_difference[i] = abs(gray_value[i] - gray_value[i + 1]) / 2 + abs(gray_value[i + 1] - gray_value[i + 2]) / 2; // 向前插值和向后插值
																															   // f_difference[i] = abs(gray_value[i]-gray_value[i+2]);
			}

			// 高斯曲线拟合
			float delta = (0.1 * log(f_difference[0]) + 0.05 * log(f_difference[1]) - 0.05 * log(f_difference[3]) - 0.1 * log(f_difference[4])) /
						  (0.1429 * log(f_difference[0]) - 0.0714 * log(f_difference[1]) - 0.1429 * log(f_difference[2]) - 0.0714 * log(f_difference[3]) + 0.1429 * log(f_difference[4]));
			if (__isnan(delta))
			{
				delta = (0.5 * log(f_difference[1]) - 0.5 * log(f_difference[3])) / 2 /
						(0.5 * log(f_difference[1]) - log(f_difference[2]) + 0.5 * log(f_difference[3]));
			}
			if (__isnan(delta))
			{
				continue;
			}

			new_contour_points.push_back(Point2f(x + delta * cos(sita), y + delta * sin(sita)));
		}

		// 曲率滤波
		vector<float> curvature_vector;
		CalCurvatureFromEdgePoints(new_contour_points, curvature_vector);

		RotatedRect new_rRect = fitEllipse(new_contour_points); // fitEllipse只接受float和int类型

		// 尝试通过拟合误差剔除坏点
		// 		ReduceBadEllipseFitPoints( new_contour_points,new_rRect.center.x,new_rRect.center.y,
		// 			new_rRect.size.width*0.5,new_rRect.size.height*0.5,new_rRect.angle*cexpr::pi<>/180);
		// 		new_rRect =fitEllipse(new_contour_points);

		sub_pixel_center_x = new_rRect.center.x;
		sub_pixel_center_y = new_rRect.center.y;

		return true;
	}
	else if (subPixel_method == Gray_Moment)
	{
		/// 灰度矩方法，一维连续函数前3阶灰度矩
		vector<Point2f> new_contour_points;

		for (int n = 0; n < contour_points.size(); n++)
		{
			int y = contour_points[n].y;
			int x = contour_points[n].x;

			if (x - 3 < 0 || x + 3 > image_mat.cols - 1 || y - 3 < 0 || y + 3 > image_mat.rows - 1)
			{
				continue;
			}

			if (x - center_x == 0)
			{
				continue;
			}

			float sita = atan2(y - center_y, x - center_x);
			float k = tan(sita);
			// float k = (y-center_y)/(x-center_x);   //梯度方向
			//  			double sita = gray_gradient_sita.at<double>(y,x);
			//  			double k = tan(sita);
			// double f_a,f_b,f_c,f_d;
			float abs_k = abs(k);

			float positive_gray[3];
			float negative_gray[3];

			int invt = 1;
			if (k > 0)
			{
				invt = -1;
			}
			if (abs_k < 1)
			{

				for (int i = 1; i < 4; i++)
				{
					int a = int(i * abs_k) / 1;
					float lamd2 = i * abs_k - a;
					float lamd1 = 1 - lamd2;

					positive_gray[i - 1] = lamd1 * image_mat.at<uchar>(y - a, x + invt * i) + lamd2 * image_mat.at<uchar>(y - a - 1, x + invt * i);
					negative_gray[i - 1] = lamd1 * image_mat.at<uchar>(y + a, x - invt * i) + lamd2 * image_mat.at<uchar>(y + a + 1, x - invt * i);
				}
			}
			else
			{
				for (int i = 1; i < 4; i++)
				{
					int a = int(i / abs_k) / 1;
					float lamd2 = i / abs_k - a;
					float lamd1 = 1 - lamd2;

					positive_gray[i - 1] = lamd1 * image_mat.at<uchar>(y - i, x + invt * a) + lamd2 * image_mat.at<uchar>(y - i, x + invt * (a + 1));
					negative_gray[i - 1] = lamd1 * image_mat.at<uchar>(y + i, x - invt * a) + lamd2 * image_mat.at<uchar>(y + i, x - invt * (a + 1));
				}
			}

			float gray_value[7]; // 沿灰度梯度方向 ，向外
			if (y < center_y)
			{
				gray_value[0] = negative_gray[2];
				gray_value[1] = negative_gray[1];
				gray_value[2] = negative_gray[0];
				gray_value[3] = image_mat.at<uchar>(y, x);
				gray_value[4] = positive_gray[0];
				gray_value[5] = positive_gray[1];
				gray_value[6] = positive_gray[2];
			}
			else
			{
				gray_value[0] = positive_gray[2];
				gray_value[1] = positive_gray[1];
				gray_value[2] = positive_gray[0];
				gray_value[3] = image_mat.at<uchar>(y, x);
				gray_value[4] = negative_gray[0];
				gray_value[5] = negative_gray[1];
				gray_value[6] = negative_gray[2];
			}

			if (gray_value[0] > gray_value[6])
			{
				std::swap(gray_value[0], gray_value[6]);
				std::swap(gray_value[1], gray_value[5]);
				std::swap(gray_value[2], gray_value[4]);
				sita = sita + cexpr::pi<>;
			}

			////计算灰度矩
			int N = 7;
			float m1 = 0, m2 = 0, m3 = 0;
			for (int i = 0; i < N; i++)
			{
				m1 += gray_value[i];
				m2 += gray_value[i] * gray_value[i];
				m3 += gray_value[i] * gray_value[i] * gray_value[i];
			}
			m1 /= N;
			m2 /= N;
			m3 /= N;

			float sigm = sqrt(m2 - m1 * m1);
			float s = (m3 + 2 * m1 * m1 * m1 - 3 * m1 * m2) / (sigm * sigm * sigm);

			float delta = N * 0.5 * s * sqrt(1.0 / (4 + s * s)) + (N + 1) * 0.5 - (N / 2 + 1);
			if (__isnan(delta))
			{
				continue;
			}

			new_contour_points.push_back(Point2f(x + delta * cos(sita), y + delta * sin(sita)));
		}
		RotatedRect new_rRect = fitEllipse(new_contour_points); // fitEllipse只接受float和int类型

		// 尝试通过拟合误差剔除坏点
		ReduceBadEllipseFitPoints(new_contour_points, new_rRect.center.x, new_rRect.center.y,
								  new_rRect.size.width * 0.5, new_rRect.size.height * 0.5, new_rRect.angle * cexpr::pi<> / 180);
		new_rRect = fitEllipse(new_contour_points);

		sub_pixel_center_x = new_rRect.center.x;
		sub_pixel_center_y = new_rRect.center.y;

		return true;
	}
	return true;
}

QRect ImageDetectMethod::GetEllipseROIRect(const Mat &image_mat, float center_x, float center_y, float ellipse_a, float ellipse_b, float angle_in_pi)
{
	int delta = 0;

	int i_top = int(center_y - ellipse_b - delta);
	if (i_top < 0)
	{
		i_top = 0;
	}

	int i_bottom = ceil(center_y + ellipse_b + delta);
	if (i_bottom > image_mat.rows - 1)
	{
		i_bottom = image_mat.rows - 1;
	}
	int j_left = int(center_x - ellipse_b - delta);
	if (j_left < 0)
	{
		j_left = 0;
	}

	int j_right = ceil(center_x + ellipse_b + delta);
	if (j_right > image_mat.cols - 1)
	{
		j_right = image_mat.cols - 1;
	}

	return QRect(j_left, i_top, j_right - j_left, i_bottom - i_top);
}

float ImageDetectMethod::DICOTSU20140215(const Mat &ori_image, int total_level /*=256*/)
{
	Mat src;
	ori_image.convertTo(src, CV_16U); // CV_16U 对应ushort

	const int image_width = ori_image.cols;
	const int image_height = ori_image.rows;

	const auto N = total_level; // 整个图分的等级
	vector<int> pn(N, 0);		// 直方图的比例
	for (int i = 0; i < image_height; i++)
	{
		auto _src = src.ptr<ushort>(i);
		for (int j = 0; j < image_width; j++)
		{
			pn[int(_src[j])]++;
		}
	}

	const auto scale = 1.f / (image_width * image_height);
	float mean = 0;
	for (int i = 0; i < N; i++)
	{
		mean += i * (float)pn[i];
	}
	mean *= scale;

	float q1 = 0, mean1 = 0;
	float max_sigma = 0, max_val = 0;
	for (auto i = 0; i < N; i++)
	{
		const auto p_i = pn[i] * scale;
		mean1 *= q1;
		q1 += p_i;
		const auto q2 = 1 - q1;

		using std::max;
		using std::min;
		if (min(q1, q2) < FLT_EPSILON || max(q1, q2) > 1. - FLT_EPSILON)
			continue;

		mean1 = (mean1 + i * p_i) / q1;
		const auto mean2 = (mean - q1 * mean1) / q2;
		const auto sigma = q1 * q2 * (mean1 - mean2) * (mean1 - mean2);
		// sigma = q1*(mean1 - mean)*(mean1 - mean) + q2*(mean2 - mean)*(mean2 - mean);

		if (sigma > max_sigma)
		{
			max_sigma = sigma;
			max_val = i;
		}
	}

	return max_val;
}

bool ImageDetectMethod::CalCentriodBySubsetMat(const Mat &subset_mat, float gray_threshold, float &sub_center_x, float &sub_center_y,
											   MarkPointColorType color_type /*= BlackDownWhiteUp*/,
											   SubPixelPosMethod subPixel_method /*= Gray_Centroid*/)
{
	sub_center_x = 0;
	sub_center_y = 0;
	float all_weight = 0;

	for (int i = 0; i < subset_mat.rows; i++)
	{
		const auto _subset_mat = subset_mat.ptr<uchar>(i);
		for (int j = 0; j < subset_mat.cols; j++)
		{
			float weight_value = CalWeightOfCentriod(float(_subset_mat[j]), gray_threshold, color_type, subPixel_method);
			sub_center_x += j * weight_value;
			sub_center_y += i * weight_value;
			all_weight += weight_value;
		}
	}

	if (all_weight == 0)
	{
		sub_center_x = 0;
		sub_center_y = 0;
	}
	else
	{
		sub_center_x /= all_weight;
		sub_center_y /= all_weight;
	}

	return true;
}

float ImageDetectMethod::CalWeightOfCentriod(float gray_value, float gray_threshold,
											 MarkPointColorType color_type /*= BlackDownWhiteUp*/,
											 SubPixelPosMethod subPixel_method /*= Gray_Centroid*/)
{
	// gray_threshold = 100;
	float weight_value;
	switch (color_type)
	{
	case BlackDownWhiteUp:
	{
		switch (subPixel_method)
		{
		case Binary_Centroid:
			if (gray_value - gray_threshold > 0)
			{
				weight_value = 1.;
			}
			else
				weight_value = 0;
			break;

		case Gray_Centroid:
			if (gray_value - gray_threshold > 0)
			{
				weight_value = gray_value /*- gray_threshold*/;
			}
			else
				weight_value = 0;

			// 真实重心法
			// weight_value = gray_value;
			break;

		case Squared_Gray_Centroid:
			if (gray_value - gray_threshold > 0)
			{
				weight_value = (gray_value /*- gray_threshold*/) * (gray_value /*- gray_threshold*/);
			}
			else
				weight_value = 0;

			// 真实灰度平方重心法5
			// weight_value = gray_value*gray_value;
			break;
		}
		break;
	}

	case WhiteDownBlackUp:
	{
		switch (subPixel_method)
		{
		case Binary_Centroid:
			if (gray_threshold - gray_value > 0)
			{
				weight_value = 1.;
			}
			else
				weight_value = 0;
			break;

		case Gray_Centroid:
			if (gray_threshold - gray_value > 0)
			{
				weight_value = gray_threshold - gray_value;
			}
			else
				weight_value = 0;
			break;

		case Squared_Gray_Centroid:
			if (gray_threshold - gray_value > 0)
			{
				weight_value = (gray_threshold - gray_value) * (gray_threshold - gray_value);
			}
			else
				weight_value = 0;
			break;
		}
		break;
	}
	}

	return weight_value;
}

MarkPointColorType ImageDetectMethod::JudgeTargetColorType(const Mat &sub_mat, float center_x_insubmat, float center_y_insubmat,
														   float ellipse_a, float ellipse_b, float angle_in_pi)
{
	float target_gray = 0;
	float back_ground_gray = 0;
	int target_pixel_num = 0;
	int back_ground_pixel_num = 0;
	for (int i = 0; i < sub_mat.rows; i++)
	{
		const uchar *_sub_mat_ptr = sub_mat.ptr<uchar>(i);
		for (int j = 0; j < sub_mat.cols; j++)
		{
			// 坐标转换图像坐标系转椭圆坐标系
			float tr_x = (j - center_x_insubmat) * cos(angle_in_pi) + (i - center_y_insubmat) * sin(angle_in_pi);
			float tr_y = -(j - center_x_insubmat) * sin(angle_in_pi) + (i - center_y_insubmat) * cos(angle_in_pi);
			if (tr_x * tr_x / ellipse_a / ellipse_a + tr_y * tr_y / ellipse_b / ellipse_b < 1)
			{
				target_gray += _sub_mat_ptr[j];
				target_pixel_num++;
			}
			else
			{
				back_ground_gray += _sub_mat_ptr[j];
				back_ground_pixel_num++;
			}
		}
	}
	target_gray /= target_pixel_num;
	back_ground_gray /= back_ground_pixel_num;

	const auto gray_thresh = (target_gray + back_ground_gray) / 2;

	return target_gray > back_ground_gray ? BlackDownWhiteUp : WhiteDownBlackUp;
}

float ImageDetectMethod::CalThresholdInSubsetMat(const Mat &sub_mat, float center_x_insubmat, float center_y_insubmat,
												 float ellipse_a, float ellipse_b, float angle_in_pi)
{
	// 背景图和目标图灰度的均值
	float target_gray = 0;
	float back_ground_gray = 0;
	int target_pixel_num = 0;
	int back_ground_pixel_num = 0;
	for (int i = 0; i < sub_mat.rows; i++)
	{
		const uchar *_sub_mat_ptr = sub_mat.ptr<uchar>(i);
		for (int j = 0; j < sub_mat.cols; j++)
		{
			// 坐标转换图像坐标系转椭圆坐标系
			float tr_x = (j - center_x_insubmat) * cos(angle_in_pi) + (i - center_y_insubmat) * sin(angle_in_pi);
			float tr_y = -(j - center_x_insubmat) * sin(angle_in_pi) + (i - center_y_insubmat) * cos(angle_in_pi);
			if (tr_x * tr_x / ellipse_a / ellipse_a + tr_y * tr_y / ellipse_b / ellipse_b < 1)
			{
				target_gray += _sub_mat_ptr[j];
				target_pixel_num++;
			}
			else
			{
				back_ground_gray += _sub_mat_ptr[j];
				back_ground_pixel_num++;
			}
		}
	}
	target_gray /= target_pixel_num;
	back_ground_gray /= back_ground_pixel_num;

	float gray_thresh = (target_gray + back_ground_gray) / 2;

	return gray_thresh;
}

float ImageDetectMethod::CalThresholdInSubsetMat2(const Mat &image_mat, const vector<Point> &contour_points)
{
	float threshold_value = 0;
	for (int i = 0; i < contour_points.size(); i++)
	{
		threshold_value += image_mat.at<uchar>(contour_points[i].y, contour_points[i].x);
	}
	return threshold_value / contour_points.size();
}

void ImageDetectMethod::CalGrayGradientBySobel(const Mat &image_mat, int i, int j, float *gray_gradient, float *gradient_sita /*=nullptr*/)
{
	// 计算灰度梯度，这里通过sobel算子计算梯度幅值，和角度
	// sobel  x[-1, 0, 1          y[-1 -1 -1
	//          -1, 0, 1             0, 0, 0
	//          -1, 0, 1]            1, 1, 1]

	const uchar *_up_row_ptr = image_mat.ptr<uchar>(i - 1);
	const uchar *_mid_row_ptr = image_mat.ptr<uchar>(i);
	const uchar *_down_row_ptr = image_mat.ptr<uchar>(i + 1);
	float dx = (float(_up_row_ptr[j + 1] - _up_row_ptr[j - 1]) + 2 * float(_mid_row_ptr[j + 1] - _mid_row_ptr[j - 1]) + float(_down_row_ptr[j + 1] - _down_row_ptr[j - 1])) / 3;
	float dy = (float(_down_row_ptr[j - 1] - _up_row_ptr[j - 1]) + 2 * float(_down_row_ptr[j] - _up_row_ptr[j]) + float(_down_row_ptr[j + 1] - _up_row_ptr[j + 1])) / 3;

	*gray_gradient = sqrt(dx * dx + dy * dy);
	if (gradient_sita != nullptr)
	{
		*gradient_sita = atan2(dy, dx);
	}
}

/**
 * \brief Evaluate curvature from edge points
 * \ref 1. 光学测量中椭圆圆心定位算法研究[J], 张虎, 2008
 * \ref 2. The Pre-Processing of Data Points for Curve Fitting in Reverse[J], 2000
 * \param 'edge_points' [input] vector of edge points
 * \param 'curv_vector' [output] vector of edge curvature
 */
void ImageDetectMethod::CalCurvatureFromEdgePoints(const vector<Point2f> &edge_points,
												   vector<float> &curvature_vector)
{
	curvature_vector.clear();

	if (edge_points.size() < 3)
	{
		return;
	}

	float x1, y1, x2, y2, x3, y3;
	for (int i = 0; i < edge_points.size(); i++)
	{
		if (i == 0)
		{
			x1 = edge_points.at(edge_points.size() - 1).x;
			y1 = edge_points.at(edge_points.size() - 1).y;
			x2 = edge_points.at(i).x;
			y2 = edge_points.at(i).y;
			x3 = edge_points.at(i + 1).x;
			y3 = edge_points.at(i + 1).y;
		}
		else if (i == edge_points.size() - 1)
		{
			x1 = edge_points.at(i - 1).x;
			y1 = edge_points.at(i - 1).y;
			x2 = edge_points.at(i).x;
			y2 = edge_points.at(i).y;
			x3 = edge_points.at(0).x;
			y3 = edge_points.at(0).y;
		}
		else
		{
			x1 = edge_points.at(i - 1).x;
			y1 = edge_points.at(i - 1).y;
			x2 = edge_points.at(i).x;
			y2 = edge_points.at(i).y;
			x3 = edge_points.at(i + 1).x;
			y3 = edge_points.at(i + 1).y;
		}

		const auto a = (x1 + x2) * (x2 - x1) * (y3 - y2);
		const auto b = (x2 + x3) * (x3 - x2) * (y2 - y1);
		const auto c = (y1 - y3) * (y2 - y1) * (y3 - y2);
		const auto d = 2 * ((x2 - x1) * (y3 - y2) - (x3 - x2) * (y2 - y1));
		const auto e = (y1 + y2) * (y2 - y1) * (x3 - x2);
		const auto f = (y2 + y3) * (y3 - y2) * (x2 - x1);
		const auto g = (x1 - x3) * (x2 - x1) * (x3 - x2);
		const auto x0 = (a - b + c) / d;
		const auto y0 = -(e - f + g) / d;

		const auto k = 1. / sqrt((x0 - x2) * (x0 - x2) + (y0 - y2) * (y0 - y2));

		curvature_vector.push_back(k);
	}
}

void ImageDetectMethod::ReduceBadEllipseFitPoints(vector<Point2f> &edge_points,
												  float center_x, float center_y, float ellipse_a, float ellipse_b, float angle_in_pi)
{
	// 尝试通过拟合误差剔除坏点
	vector<float> error_vector;
	for (int i = 0; i < edge_points.size(); i++)
	{
		const auto error = ErrorDROfEllipseFit(center_x, center_y, ellipse_a, ellipse_b,
											   angle_in_pi, edge_points[i].x, edge_points[i].y);

		error_vector.push_back(error);

		if (error > 0.5)
		{
			auto it = edge_points.begin() + i;
			edge_points.erase(it);

			i--;
		}
	}
}

int ImageDetectMethod::AverageOfList(const QList<int> &list_value)
{
	const auto _Sum = std::accumulate(list_value.begin(), list_value.end(), 0);
	return (_Sum / list_value.size());
}

double ImageDetectMethod::OTSUForCannyAdaptiveHighTh(Mat ori_image, int total_level)
{
	Mat src;
	ori_image.convertTo(src, CV_16U); // CV_16U 对应ushort

	int image_width = ori_image.cols;
	int image_height = ori_image.rows;

	int N = total_level; // 整个图分的等级
	// int pn[N] ={0};                //直方图的比例
	vector<int> pn;
	for (int i = 0; i < N; i++)
	{
		pn.push_back(0);
	}

	for (int i = 0; i < image_height; i++)
	{
		ushort *_src = src.ptr<ushort>(i);
		for (int j = 0; j < image_width; j++)
		{
			pn[int(_src[j])]++;
		}
	}

	double scale = 1. / (image_width * image_height);
	double mean = 0;
	for (int i = 0; i < N; i++)
	{
		mean += i * (double)pn[i];
	}
	mean *= scale;

	double q1 = 0, mean1 = 0;
	double max_sigma = 0, max_val = 0;

	for (int i = 0; i < N; i++)
	{
		double p_i, q2, mean2, sigma;

		p_i = pn[i] * scale;
		mean1 *= q1;
		q1 += p_i;
		q2 = 1. - q1;

		if (std::min(q1, q2) < FLT_EPSILON || std::max(q1, q2) > 1. - FLT_EPSILON)
			continue;

		mean1 = (mean1 + i * p_i) / q1;
		mean2 = (mean - q1 * mean1) / q2;

		sigma = q1 * q2 * (mean1 - mean2) * (mean1 - mean2);
		// sigma = q1*(mean1 - mean)*(mean1 - mean) + q2*(mean2 - mean)*(mean2 - mean);

		if (i < N / 8)
		{
			continue;
		}
		if (sigma > max_sigma)
		{
			max_sigma = sigma;
			max_val = i;
		}
	}

	return max_val;
}

QList<int> ImageDetectMethod::GetALineGrayList(Mat image_mat, QPoint point1, QPoint point2)
{
	QList<int> gray_list;

	if (point1.x() == point2.x())
	{
		int j = point1.x();
		if (point1.y() < point2.y())
		{
			for (int i = point1.y(); i < point2.y(); i++)
			{
				gray_list.append(image_mat.at<uchar>(i, j));
			}
		}
		else
		{
			for (int i = point2.y(); i < point1.y(); i++)
			{
				gray_list.append(image_mat.at<uchar>(i, j));
			}
		}
	}
	else
	{
		auto k = double(point2.y() - point1.y()) / double(point2.x() - point1.x());
		if (abs(k) <= 1)
		{
			if (point1.x() < point2.x())
			{
				for (int j = point1.x(); j <= point2.x(); j++)
				{
					int i = int(k * (j - point1.x()) + point1.y());
					gray_list.append(image_mat.at<uchar>(i, j));
				}
			}
			else
			{
				// 				for (int j = point2.x();j<point1.x();j++)
				// 				{
				// 					int i = int(k*(j-point1.x())+point1.y());
				// 					gray_list.append(image_mat.at<uchar>(i,j));
				// 				}

				for (int j = point1.x(); j >= point2.x(); j--)
				{
					int i = int(k * (j - point1.x()) + point1.y());
					gray_list.append(image_mat.at<uchar>(i, j));
				}
			}
		}
		else
		{
			k = double(point2.x() - point1.x()) / double(point2.y() - point1.y());
			if (point1.y() < point2.y())
			{
				for (int i = point1.y(); i <= point2.y(); i++)
				{
					int j = int(k * (i - point1.y()) + point1.x());
					gray_list.append(image_mat.at<uchar>(i, j));
				}
			}
			else
			{
				// 				for (int i = point2.y();i<=point1.y();i++)
				// 				{
				// 					int j = int(k*(i-point1.y())+point1.x());
				// 					gray_list.append(image_mat.at<uchar>(i,j));
				// 				}
				for (int i = point1.y(); i >= point2.y(); i--)
				{
					int j = int(k * (i - point1.y()) + point1.x());
					gray_list.append(image_mat.at<uchar>(i, j));
				}
			}
		}
	}
	return gray_list;
}

int ImageDetectMethod::MIdValue(QList<int> value_list)
{
	int mid_value = 0;
	int n = value_list.size();
	for (int i = 0; i < n; i++)
	{
		for (int j = 0; j < n - 1 - i; j++)
		{
			if (value_list.value(i) > value_list.value(i + 1))
			{
#if QT_VERSION < QT_VERSION_CHECK(5, 15, 0)
				value_list.swap(j, j + 1);
#else
				value_list.swapItemsAt(j, j + 1);
#endif
			}
		}
	}
	if (n / 2 == 0)
	{
		mid_value = (value_list.value(n / 2 - 1) + value_list.value(n / 2)) / 2;
	}
	else
		mid_value = value_list.value(n / 2);

	return mid_value;
}

int ImageDetectMethod::Change2To10(QList<int> list_code2)
{
	int ans10 = 0;
	int n = list_code2.size();
	for (int i = 0; i < n; i++)
	{
		ans10 = ans10 + int(pow(2.0, n - i - 1) * list_code2.value(i));
	}
	return ans10;
}
//*************************************
bool ImageDetectMethod::TestCircleCenterPosAccuracy(QString image_file_name, Mat &code_point_mat, Mat &uncode_point_mat,
													float ratio_k, float ratio_k1, float ratio_k2, float min_radius, float max_radius, float ellipse_error_pixel /*=0.5*/,
													MarkPointColorType color_type /*= BlackDownWhiteUp*/, CodePointBitesType code_bites_type /*=CodeBites15*/,
													DetectContoursMethod image_process_method /*= OTSU_Method*/, SubPixelPosMethod subpixel_pos_method /*=Gray_Centroid*/)
{
	image_process_method = Self_CANNY_Method;
	subpixel_pos_method = Gray_Moment;
	/*ImageDetectMethod::CodeAndUncodePointDetect(image_file_name,code_point_mat,uncode_point_mat,
		ratio_k,ratio_k1,ratio_k2 ,
		min_radius,max_radius,ellipse_error_pixel,
		color_type,code_bites_type ,
		image_process_method,subpixel_pos_method);*/

	QFileInfo file_info(image_file_name);
	// WirteMatToFile(uncode_point_mat,file_info.completeBaseName());

	return true;
}

void ImageDetectMethod::WirteMatToFile(Mat mat, QString file_name)
{
	QFile save_file("E:\\" + file_name + ".csv");
	if (save_file.open(QIODevice::WriteOnly | QIODevice::Text))
	{
		QTextStream stream(&save_file);

		if (mat.depth() == CV_64F)
		{
			for (int i = 0; i < mat.rows; i++)
			{
				for (int j = 0; j < mat.cols; j++)
				{
					stream << mat.at<double>(i, j) << ",";
				}
				stream << "\n";
			}
		}
		else if (mat.depth() == CV_8U)
		{
			for (int i = 0; i < mat.rows; i++)
			{
				for (int j = 0; j < mat.cols; j++)
				{
					stream << mat.at<uchar>(i, j) << ",";
				}
				stream << "\n";
			}
		}
		else if (mat.depth() == CV_32F)
		{
			for (int i = 0; i < mat.rows; i++)
			{
				for (int j = 0; j < mat.cols; j++)
				{
					stream << mat.at<float>(i, j) << ",";
				}
				stream << "\n";
			}
		}
		else if (mat.depth() == CV_16SC1)
		{
			for (int i = 0; i < mat.rows; i++)
			{
				for (int j = 0; j < mat.cols; j++)
				{
					stream << mat.at<short int>(i, j) << ",";
				}
				stream << "\n";
			}
		}
		else if (mat.depth() == CV_32S)
		{
			for (int i = 0; i < mat.rows; i++)
			{
				for (int j = 0; j < mat.cols; j++)
				{
					stream << mat.at<int>(i, j) << ",";
				}
				stream << "\n";
			}
		}
	}

	save_file.close();
}
//***********************************************************
bool ImageDetectMethod::FindChessGridOpencv(
	QString image_file_name,
	int h_num, int v_num,
	vector<Point2f> &corners,
	vector<uchar> &sign_list,
	int &useful_corner_num)
{
	Mat ori_image = imread(image_file_name.toStdString(), 0);
	if (!ori_image.data) // 判断图片调入是否成功
		return false;	 // 调入图片失败则退出

	sign_list.clear();
	sign_list.resize(h_num * v_num);

	bool is_find = findChessboardCorners(ori_image,
										 Size(h_num, v_num), corners,
										 CALIB_CB_ADAPTIVE_THRESH +
											 CALIB_CB_NORMALIZE_IMAGE +
											 CALIB_CB_FAST_CHECK);

	if (is_find)
	{
		cornerSubPix(ori_image, corners,
					 Size(11, 11), Size(-1, -1),
					 TermCriteria(TermCriteria::EPS +
									  TermCriteria::MAX_ITER,
								  30, 0.1));

		for (int i = 0; i < h_num * v_num; i++)
		{
			sign_list.at(i) = 1;
		}
		useful_corner_num = h_num * v_num;
		return true;
	}
	else
		return false;
}

bool ImageDetectMethod::FindCircleGrid(
	QString image_file_name, int h_num, int v_num,
	int h_offset, int v_offset, int h_mid_length, int v_mid_length,
	vector<Point2f> &corners, vector<uchar> &sign_list,
	int &useful_corner_num,
	float ratio_k /*= 2*/, float ratio_k1 /*= 2.4*/,
	float ratio_k2 /*= 4*/,
	float min_radius /* = 5*/, float max_radius /*= 50*/,
	float ellipse_error_pixel /*=0.5*/,
	const vector<vector<Point2f>> &edge_points /*= vector<vector<Point2f>>()*/,
	DetectContoursMethod image_process_method /*= OTSU_Method*/,
	SubPixelPosMethod subpixel_pos_method /*=Gray_Centroid*/)
{
	Mat ori_image = imread(image_file_name.toStdString(), 0);
	if (!ori_image.data) // 判断图片调入是否成功
		return false;	 // 调入图片失败则退出

	corners.clear();
	sign_list.clear();
	corners.resize(h_num * v_num);
	sign_list.resize(h_num * v_num);
	useful_corner_num = 0;

	Mat processed_image_mat;
	vector<vector<Point>> contours;
	QList<QList<float>> ellipse_pars; // ellipse_pars - n*6  center_x,center_y,r_a,r_b,angle_inPI,ellipse_error,contours_index,ID,code_type(0- uncode,1- code)

	// 1.图像预处理
	ImagePreprocess(ori_image, processed_image_mat);

	// 2.边缘检测，存入闭合轮廓
	DetectClosedContours(processed_image_mat, contours, image_process_method);

	// 3.轮廓筛选，尺寸，形状等准则，圆
	FilterEllipseContours(contours, min_radius, max_radius,
						  ellipse_error_pixel, ellipse_pars);

	// 4.进一步筛选 ，用于CSI标定板，灰度和相对关系，判断3定向方向点
	vector<int> orident_point_index_list;
	FilterEllipseContoursForCSICalibrationPlane(processed_image_mat, ratio_k, ratio_k1, ratio_k2,
												ellipse_pars, orident_point_index_list);

	// 5.CSI圆形标定板排列
	// 5.1 方向点确定，直角边点 original_point_index, X边 X_axis_point_index,Y边 Y_axis_point_index
	int original_point_index = 0, X_axis_point_index = 0, Y_axis_point_index = 0;
	float d_max = 0;
	for (int i = 0; i < orident_point_index_list.size() - 1; i++)
	{
		for (int j = i + 1; j < orident_point_index_list.size(); j++)
		{
			float x1 = ellipse_pars[orident_point_index_list[i]][0];
			float y1 = ellipse_pars[orident_point_index_list[i]][1];
			float x2 = ellipse_pars[orident_point_index_list[j]][0];
			float y2 = ellipse_pars[orident_point_index_list[j]][1];
			float length = sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
			if (length > d_max)
			{
				d_max = length;
				X_axis_point_index = orident_point_index_list[i];
				Y_axis_point_index = orident_point_index_list[j];
			}
		}
	}
	for (int i = 0; i < orident_point_index_list.size(); i++)
	{
		if (orident_point_index_list[i] != X_axis_point_index && orident_point_index_list[i] != Y_axis_point_index)
		{
			original_point_index = orident_point_index_list[i];
		}
	}

	//***判断两个直角边上两方向点之间的圆点个数，确定正确的XY轴
	vector<Point2f> line_X_points;
	vector<Point2f> line_Y_points;
	Point2f original_point(ellipse_pars[original_point_index][0], ellipse_pars[original_point_index][1]);
	Point2f X_axis_point(ellipse_pars[X_axis_point_index][0], ellipse_pars[X_axis_point_index][1]);
	Point2f Y_axis_point(ellipse_pars[Y_axis_point_index][0], ellipse_pars[Y_axis_point_index][1]);
	float X_line_A, X_line_B, X_line_C, Y_line_A, Y_line_B, Y_line_C;
	LineEquation(original_point, X_axis_point, X_line_A, X_line_B, X_line_C);
	LineEquation(original_point, Y_axis_point, Y_line_A, Y_line_B, Y_line_C);

	for (int i = 0; i < ellipse_pars.size(); i++)
	{
		if (i == original_point_index || i == X_axis_point_index || i == Y_axis_point_index)
		{
			continue;
		}

		float d_max = ellipse_pars[i][2];

		Point2f p(ellipse_pars[i][0], ellipse_pars[i][1]);
		float dis_to_X_line = PointTpLineDistance(p, X_line_A, X_line_B, X_line_C);
		float dis_to_Y_line = PointTpLineDistance(p, Y_line_A, Y_line_B, Y_line_C);

		// 其实就是角度
		float dot_in_X_line = (p.x - original_point.x) * (p.x - X_axis_point.x) + (p.y - original_point.y) * (p.y - X_axis_point.y);
		float dot_in_Y_line = (p.x - original_point.x) * (p.x - Y_axis_point.x) + (p.y - original_point.y) * (p.y - Y_axis_point.y);

		if (dis_to_X_line < d_max && dot_in_X_line < 0)
		{
			line_X_points.push_back(p);
		}

		if (dis_to_Y_line < d_max && dot_in_Y_line < 0)
		{
			line_Y_points.push_back(p);
		}
	}

	if (line_X_points.size() == h_mid_length - 1 && line_Y_points.size() == v_mid_length - 1)
	{
	}
	else if (line_X_points.size() == v_mid_length - 1 && line_Y_points.size() == h_mid_length - 1)
	{
		std::swap(X_axis_point_index, Y_axis_point_index);
		std::swap(line_X_points, line_Y_points);
		std::swap(X_axis_point, Y_axis_point);
	}
	else
		return false;
	// 右手系判断，向量叉乘大于0
	Point2f vector1(X_axis_point.x - original_point.x, X_axis_point.y - original_point.y);
	Point2f vector2(Y_axis_point.x - original_point.x, Y_axis_point.y - original_point.y);
	if (vector1.x * vector2.y - vector1.y * vector2.x > 0)
	{
		return false;
	}
	// 5.3 计算单应矩阵H，平面到平面，opencv有函数，自己也写过最少4个点，重新排列点的位置
	vector<Point2f> src_points, dst_points;
	src_points.push_back(original_point);
	src_points.push_back(X_axis_point);
	src_points.push_back(Y_axis_point);
	dst_points.push_back(Point2f(h_offset, v_offset));
	dst_points.push_back(Point2f(h_offset + h_mid_length, v_offset));
	dst_points.push_back(Point2f(h_offset, v_offset + v_mid_length));
	for (int i = 0; i < line_X_points.size() - 1; i++)
	{
		for (int j = 0; j < line_X_points.size() - i - 1; j++)
		{
			float d1 = PointToPointDistance(line_X_points[j], original_point);
			float d2 = PointToPointDistance(line_X_points[j + 1], original_point);
			if (d1 > d2)
			{
				std::swap(line_X_points[j], line_X_points[j + 1]);
			}
		}
	}
	for (int i = 0; i < line_Y_points.size() - 1; i++)
	{
		for (int j = 0; j < line_Y_points.size() - i - 1; j++)
		{
			float d1 = PointToPointDistance(line_Y_points[j], original_point);
			float d2 = PointToPointDistance(line_Y_points[j + 1], original_point);
			if (d1 > d2)
			{
				std::swap(line_Y_points[j], line_Y_points[j + 1]);
			}
		}
	}
	for (int i = 0; i < line_X_points.size(); i++)
	{
		src_points.push_back(line_X_points[i]);
		dst_points.push_back(Point2f(h_offset + i + 1, v_offset));
	}
	for (int i = 0; i < line_Y_points.size(); i++)
	{
		src_points.push_back(line_Y_points[i]);
		dst_points.push_back(Point2f(h_offset, v_offset + i + 1));
	}

	Mat H_mat = findHomography(src_points, dst_points, cv::RANSAC);
	H_mat.convertTo(H_mat, CV_32F);

	// 5.4 重新按顺序排列
	vector<vector<Point2f>> subpixel_edge_contours;
	for (int n = 0; n < ellipse_pars.size(); n++)
	{
		Mat X = Mat(3, 1, CV_32F);
		X.at<float>(0, 0) = ellipse_pars[n][0];
		X.at<float>(1, 0) = ellipse_pars[n][1];
		X.at<float>(2, 0) = 1;
		Mat A = H_mat * X;
		float new_x = A.at<float>(0, 0) / A.at<float>(2, 0);
		float new_y = A.at<float>(1, 0) / A.at<float>(2, 0);

		int i = floor(new_y + 0.5);
		int j = floor(new_x + 0.5);
		float delta_x = abs(new_x - j);
		float delta_y = abs(new_y - i);

		if (i < 1 || i > v_num || j < 1 || j > h_num)
		{
			continue;
		}

		float sub_pixel_x, sub_pixel_y;
		vector<Point2f> edge_contour;
		FindSubPixelPosOfCircleCenter20140210(processed_image_mat, ellipse_pars[n][0], ellipse_pars[n][1], ellipse_pars[n][2],
											  ellipse_pars[n][3], ellipse_pars[n][4], contours[ellipse_pars[n][6]], sub_pixel_x, sub_pixel_y,
											  &edge_contour, subpixel_pos_method, Uncertainty);
		ellipse_pars[n][0] = sub_pixel_x;
		ellipse_pars[n][1] = sub_pixel_y;
		subpixel_edge_contours.push_back(edge_contour);

		corners[(i - 1) * h_num + j - 1] = Point2f(sub_pixel_x, sub_pixel_y);
		sign_list[(i - 1) * h_num + j - 1] = 1;

		useful_corner_num++;
	}

	return true;
}

bool ImageDetectMethod::FilterEllipseContoursForCSICalibrationPlane(const Mat &image_mat,
																	float ratio_k, float ratio_k1, float ratio_k2,
																	QList<QList<float>> &ellipse_pars,
																	vector<int> &orident_point_index)
{
	orident_point_index.clear();

	// 1.先利用相对关系准则，筛选外圆环方向点
	for (int i = 0; i < ellipse_pars.size() - 1; i++)
	{
		for (int j = i + 1; j < ellipse_pars.size(); j++)
		{
			float x1 = ellipse_pars[i][0];
			float y1 = ellipse_pars[i][1];
			float x2 = ellipse_pars[j][0];
			float y2 = ellipse_pars[j][1];
			float length_of_2_points = sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));

			if (length_of_2_points < min(ellipse_pars[i][3], ellipse_pars[j][3]))
			{
				if (ellipse_pars[i][3] > ellipse_pars[j][3])
				{
					// 外圆
					orident_point_index.push_back(i);

					ellipse_pars.removeAt(j);
					j--;

					break;
				}
				else
				{
					// 内圆
					ellipse_pars.removeAt(i);
					i--;

					orident_point_index.push_back(j - 1);
				}
			}
			else if (length_of_2_points < min(ellipse_pars[i][3], ellipse_pars[j][3]) * ratio_k2)
			{
				if (ellipse_pars[i][5] > ellipse_pars[j][5])
				{
					ellipse_pars.removeAt(i);
					i--;
					break;
				}
				else
				{
					ellipse_pars.removeAt(j);
					j--;
				}
			}
		}
	}

	if (orident_point_index.size() == 3)
	{
		// 3个定位方向点已经找到
		return true;
	}

	// 2.如果有部分中间圆点没有检测出来，需要利用灰度准则重新检测定位方向点
	QList<float> gray_value_std_list;
	for (int i = 0; i < ellipse_pars.size(); i++)
	{
		float out_foreground_stdDev; // 圆点灰度方差,
		EllipseGrayJudgeForCodePoint(image_mat, ellipse_pars[i][0], ellipse_pars[i][1],
									 ellipse_pars[i][2], ellipse_pars[i][3], ellipse_pars[i][4], ratio_k, true, nullptr, nullptr, &out_foreground_stdDev);
		gray_value_std_list.push_back(out_foreground_stdDev);
	}
	QList<float> ori_list = gray_value_std_list;
#if QT_VERSION < QT_VERSION_CHECK(5, 15, 0)
	qSort(gray_value_std_list.begin(), gray_value_std_list.end(), qGreater<float>());
#else
	std::sort(gray_value_std_list.begin(), gray_value_std_list.end(), std::greater<float>());
#endif
	if (gray_value_std_list.size() < 3)
	{
		return false;
	}
	orident_point_index.clear();
	orident_point_index.push_back(ori_list.indexOf(gray_value_std_list[0]));
	orident_point_index.push_back(ori_list.indexOf(gray_value_std_list[1]));
	orident_point_index.push_back(ori_list.indexOf(gray_value_std_list[2]));

	return true;
}

void ImageDetectMethod::LineEquation(Point2f p1, Point2f p2, float &A, float &B, float &C)
{
	float x1 = p1.x;
	float y1 = p1.y;
	float x2 = p2.x;
	float y2 = p2.y;

	if (x2 == x1)
	{
		A = 1;
		B = 0;
		C = -x1;

		return;
	}
	else
	{
		float k = (y2 - y1) / (x2 - x1);
		float b = y1 - k * x1;

		A = k;
		B = -1;
		C = b;
		return;
	}
}

float ImageDetectMethod::PointTpLineDistance(Point2f p, float A, float B, float C)
{
	float d = abs(A * p.x + B * p.y + C) / sqrt(A * A + B * B);
	return d;
}

float ImageDetectMethod::PointToPointDistance(Point2f p1, Point2f p2)
{
	return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
}

bool ImageDetectMethod::FindCodePointsForSelfCalibrationByHartly(QStringList image_file_name_list, vector<vector<Point2f>> &code_points_list,
																 float ratio_k /*= 2*/, float ratio_k1 /*= 2.4*/, float ratio_k2 /*= 4 */,
																 float min_radius, float max_radius, float ellipse_error_pixel /*=0.5*/,
																 MarkPointColorType color_type /*= BlackDownWhiteUp*/,
																 CodePointBitesType code_bites_type /*=CodeBites15*/,
																 DetectContoursMethod image_process_method /*= OTSU_Method*/,
																 SubPixelPosMethod subpixel_pos_method /*=Gray_Centroid*/)
{
	if (image_file_name_list.size() < 4)
	{
		return false;
	}

	QList<Mat> code_point_mat_list;
	for (int i = 0; i < image_file_name_list.size(); i++)
	{
		Mat code_point_mat, uncode_point_mat;
		auto _Image = imread(image_file_name_list[i].toStdString(), IMREAD_GRAYSCALE);
		float ratio[3] = {ratio_k, ratio_k1, ratio_k2};
		if (_General_circular_marker_detector(
				{_Image.ptr(), size_t(_Image.rows), size_t(_Image.cols)},
				code_point_mat, uncode_point_mat,
				min_radius, max_radius,
				ratio,
				ellipse_error_pixel,
				color_type, code_bites_type,
				image_process_method, subpixel_pos_method))
		{
			code_point_mat_list.append(code_point_mat);
		}
	}

	code_points_list.clear();
	code_points_list.resize(4);

	/*vector<vector<Point2f>> point_list(4);*/
	for (int i = 0; i < code_point_mat_list[0].rows; i++)
	{
		int match_index[4] = {-1, -1, -1, -1};
		match_index[0] = i;
		for (int j = 1; j < 4; j++)
		{
			for (int k = 0; k < code_point_mat_list[j].rows; k++)
			{
				if (code_point_mat_list[0].at<float>(i, 0) == code_point_mat_list[j].at<float>(k, 0))
				{
					match_index[j] = k;
					break;
				}
			}
			if (match_index[j] == -1)
			{
				break;
			}
			else if (j == 3)
			{
				code_points_list[0].push_back(Point2f(code_point_mat_list[0].at<float>(match_index[0], 1), code_point_mat_list[0].at<float>(match_index[0], 2)));
				code_points_list[1].push_back(Point2f(code_point_mat_list[1].at<float>(match_index[1], 1), code_point_mat_list[1].at<float>(match_index[1], 2)));
				code_points_list[2].push_back(Point2f(code_point_mat_list[2].at<float>(match_index[2], 1), code_point_mat_list[2].at<float>(match_index[2], 2)));
				code_points_list[3].push_back(Point2f(code_point_mat_list[3].at<float>(match_index[3], 1), code_point_mat_list[3].at<float>(match_index[3], 2)));
			}
		}
	}

	return (code_points_list[0].size() < 9) ? false : true;
}

bool ImageDetectMethod::FindCodePointsForPlaneCalibration(const QString image_file_name, const vector<int> &code_point_id,
														  vector<uchar> &sign_list, vector<Point2f> &code_points_list,
														  float ratio_k /*= 2*/, float ratio_k1 /*= 2.4*/, float ratio_k2 /*= 4 */,
														  float min_radius /*=5*/, float max_radius /*=50*/, float ellipse_error_pixel /*=0.5*/,
														  MarkPointColorType color_type /*= BlackDownWhiteUp*/, CodePointBitesType code_bites_type /*=CodeBites15*/,
														  DetectContoursMethod image_process_method /*= OTSU_Method*/, SubPixelPosMethod subpixel_pos_method /*= Gray_Centroid*/)
{
	Mat code_point_mat, uncode_point_mat;
	auto _Image = imread(image_file_name.toStdString(), IMREAD_GRAYSCALE);
	float ratio[3] = {ratio_k, ratio_k1, ratio_k2};
	const auto is_detect = _General_circular_marker_detector(
		{_Image.ptr(), size_t(_Image.rows), size_t(_Image.cols)},
		code_point_mat, uncode_point_mat,
		min_radius, max_radius,
		ratio,
		ellipse_error_pixel,
		color_type, code_bites_type,
		image_process_method, subpixel_pos_method);
	if (is_detect == false)
	{
		return false;
	}

	sign_list.clear();
	code_points_list.clear();
	for (int i = 0; i < code_point_id.size(); i++)
	{
		for (int j = 0; j < code_point_mat.rows; j++)
		{
			float *ptr_code_point_mat = code_point_mat.ptr<float>(j);
			if (code_point_id[i] == int(ptr_code_point_mat[0]))
			{
				sign_list.push_back(1);
				code_points_list.push_back(Point2f(ptr_code_point_mat[1], ptr_code_point_mat[2]));
				break;
			}
			if (j == code_point_mat.rows - 1)
			{
				sign_list.push_back(0);
				code_points_list.push_back(Point2f(0, 0));
			}
		}
	}

	return true;
}