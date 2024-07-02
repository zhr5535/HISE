#pragma once

#include <opencv2/core/core_c.h>
#include "sparse/__defs_ports__.hpp"

using namespace cv;

class DEFLOW_ALG _6x6UniformBicubicSplineInterpolationMethod
{
public:
	//\brief LUT element size
	static constexpr auto cell_size = 16;
	static constexpr auto start_indent = 2;
	static constexpr auto end_indent = start_indent | 1;

	_6x6UniformBicubicSplineInterpolationMethod();
	~_6x6UniformBicubicSplineInterpolationMethod();


	//以下方法可以参见maple推导--双三次样条插值
	static bool Build_Two_Dimension_Interpolation_6X6_Q_Matrix_4X6(
		CvMat* Q_mat
		);

	//查找表方法
	//计算插值系数查找表
	static bool Build_Two_Dimension_Interpolation_6X6_LUT(//首选
		CvMat* image,
		CvMat* Q_mat,
		int aoi_x_min,
		int aoi_x_max,//上闭区间，下开区间
		int aoi_y_min,
		int aoi_y_max,//左闭区间，右边开区间
		CvMat* lut_mat
		);

	//无校验的方法--慎用
	//首选--参考图像--整像素点灰度梯度
	static bool Build_Two_Dimension_Interpolation_6X6_Integer_Points_Gradient_Value_LUT(
		CvMat* reference_image,
		CvMat* Q_mat,
		int reference_image_gradient_mat_x_min,
		int reference_image_gradient_mat_x_max,
		int reference_image_gradient_mat_y_min,
		int reference_image_gradient_mat_y_max,
		CvMat* reference_image_gradient_x_mat,
		CvMat* reference_image_gradient_y_mat
		);

	//变形图
	static bool Deform_Value_One_Template_64F_20130617_Before(
		CvMat* image,//64F
		double reference_point_x,
		double reference_point_y,
		CvMat* displacement_input,//映射后应全部处于图像宽度和高度内，64F,6行1列
		int interpolation_area_x_min,
		int interpolation_area_x_max,//上闭区间，下开区间
		int interpolation_area_y_min,
		int interpolation_area_y_max,//左闭区间，右边开区间
		CvMat* interpolation_LUT,//32F
		CvMat* interpolation_LUT_valid_sign_mat,
		CvMat* uniform_bicubic_natrue_spline_interpolation_6x6_Q_mat,
		CvMat* g_mat
		);

	static bool Reference_Value_And_Gradient_One_Template_64F_20130531_Before(
		CvMat* image,//64F
		double reference_point_x,//参考图模板必须为方形
		double reference_point_y,
		int interpolation_area_x_min,
		int interpolation_area_x_max,//上闭区间，下开区间
		int interpolation_area_y_min,
		int interpolation_area_y_max,//左闭区间，右边开区间
		CvMat* interpolation_LUT,//32F
		CvMat* interpolation_LUT_valid_sign_mat,
		CvMat* uniform_bicubic_natrue_spline_interpolation_6x6_Q_mat,
		CvMat* g_mat,//64F
		CvMat* gx_mat,//64F
		CvMat* gy_mat//64F
		);

	//////////////////////////////////////////////////////////////////////////
	//20130617
	static bool Value_And_Gradient_One_Affine_Template_64F(
		CvMat* image,//64F
		double template_center_x,
		double template_center_y,
		double template_ux,
		double template_uy,
		double template_vx,
		double template_vy,
		int interpolation_area_x_min,
		int interpolation_area_x_max,//上闭区间，下开区间
		int interpolation_area_y_min,
		int interpolation_area_y_max,//左闭区间，右边开区间
		CvMat* interpolation_LUT,//32F
		CvMat* interpolation_LUT_valid_sign_mat,
		CvMat* uniform_bicubic_natrue_spline_interpolation_6x6_Q_mat,
		CvMat* g_mat,
		CvMat* gx_mat,//64F
		CvMat* gy_mat//64F
		);


	//变形图
	static bool Value_One_Affine_Template_64F(
		CvMat* image,//64F
		double template_center_x,
		double template_center_y,
		double template_ux,
		double template_uy,
		double template_vx,
		double template_vy,
		int interpolation_area_x_min,
		int interpolation_area_x_max,//上闭区间，下开区间
		int interpolation_area_y_min,
		int interpolation_area_y_max,//左闭区间，右边开区间
		CvMat* interpolation_LUT,//32F
		CvMat* interpolation_LUT_valid_sign_mat,
		CvMat* uniform_bicubic_natrue_spline_interpolation_6x6_Q_mat,
		CvMat* g_mat
		);
	// Overload of Value_One_Affine_Template_64F
	static auto value_one_affine_template_64f(
		cv::Mat& g_img, double x, double y, double disp_grad[4],
		int x_min, int x_max, int y_min, int y_max,
		cv::Mat& lut, cv::Mat& lut_mask, cv::Mat& Qmat,
		cv::Mat& g_wnd) {
		auto _G_img = cvMat(g_img), _G_wnd = cvMat(g_wnd);
		auto _Lut = cvMat(lut), _Lut_mask = cvMat(lut_mask), _Qmat = cvMat(Qmat);
		return Value_One_Affine_Template_64F(
			&_G_img, x, y, disp_grad[0], disp_grad[1], disp_grad[2], disp_grad[3],
			x_min, x_max, y_min, y_max,
			&_Lut, &_Lut_mask, &_Qmat, &_G_wnd);
	}

	//////////////////////////////////////////////////////////////////////////
	//引伸计模式
	//单模板方法
	static bool Integer_Points_Gradient_One_Template(
		CvMat* reference_image,//8U or 64F
		CvMat* Q_mat,
		int center_x_in_image,
		int center_y_in_image,
		CvMat* referece_template_gx_mat,
		CvMat* referece_template_gy_mat
		);
	static bool Value_One_Affine_Template_64F( 
		CvMat* image,//64F
		CvMat* template_transform,
		int interpolation_area_x_min,
		int interpolation_area_x_max,//上闭区间，下开区间
		int interpolation_area_y_min,
		int interpolation_area_y_max,//左闭区间，右边开区间
		CvMat* interpolation_LUT,//32F
		CvMat* interpolation_LUT_valid_sign_mat,
		CvMat* uniform_bicubic_natrue_spline_interpolation_6x6_Q_mat,
		CvMat* g_mat
		);


	//////////////////////////////////////////////////////////////////////////
	//12参数方法
	static bool Value_One_Affine_Template_64F_12_Parameter(
		CvMat* image,//64F
		CvMat* template_transform_mat,
		int interpolation_area_x_min,
		int interpolation_area_x_max,//上闭区间，下开区间
		int interpolation_area_y_min,
		int interpolation_area_y_max,//左闭区间，右边开区间
		CvMat* interpolation_LUT,//32F
		CvMat* interpolation_LUT_valid_sign_mat,
		CvMat* uniform_bicubic_natrue_spline_interpolation_6x6_Q_mat,
		CvMat* g_mat
		);

	static bool Value_And_Gradient_One_Affine_Template_64F_12_Parameter(
		CvMat* image,//64F
		CvMat* template_transform_mat,
		int interpolation_area_x_min,
		int interpolation_area_x_max,//上闭区间，下开区间
		int interpolation_area_y_min,
		int interpolation_area_y_max,//左闭区间，右边开区间
		CvMat* interpolation_LUT,//32F
		CvMat* interpolation_LUT_valid_sign_mat,
		CvMat* uniform_bicubic_natrue_spline_interpolation_6x6_Q_mat,
		CvMat* g_mat,//64F
		CvMat* gx_mat,//64F
		CvMat* gy_mat//64F
		);
	
	//////////////////////////////////////////////////////////////////////////

	//////////////////////////////////////////////////////////////////////////
	//20参数方法
	static bool Value_And_Gradient_One_Affine_Template_64F_20_Parameter(
		CvMat* image,//64F
		CvMat* template_transform_mat,
		int interpolation_area_x_min,
		int interpolation_area_x_max,//上闭区间，下开区间
		int interpolation_area_y_min,
		int interpolation_area_y_max,//左闭区间，右边开区间
		CvMat* interpolation_LUT,//32F
		CvMat* interpolation_LUT_valid_sign_mat,
		CvMat* uniform_bicubic_natrue_spline_interpolation_6x6_Q_mat,
		CvMat* g_mat,//64F
		CvMat* gx_mat,//64F
		CvMat* gy_mat//64F
		);
	//////////////////////////////////////////////////////////////////////////
	static const int Interpolation_LUT_Element_Size = 16;
	static const int Distance_To_Left_Top_Bound = 2;
	static const int Distance_To_Right_Bottom_Bound =3;
private:
	
};

using UBSplineInterp6x6_t = _6x6UniformBicubicSplineInterpolationMethod;
