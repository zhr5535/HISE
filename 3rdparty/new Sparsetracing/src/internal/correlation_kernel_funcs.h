#pragma once

#include "sparse/__defs_ports__.hpp"
#include <opencv2/core/core_c.h>

using namespace cv;

class DEFLOW_ALG CorrelationKernelMethods1394
{
public:
	//////////////////////////////////////////////////////////////////////////
	//无权重方法
	//计算位移
	static bool Calculate_Newton_Raphson_Get_Inverse_Hessian_Mat( 
		CvMat* reference_fx_value_mat, 
		CvMat* reference_fy_value_mat, 
		CvMat* invert_hessian_mat 
		);
	// Overload of Calculate_Newton_Raphson_Get_Inverse_Hessian_Mat
	static bool CalculateNewtonRaphsonInverseHessian(
		CvMat* reference_fx_value_mat,
		CvMat* reference_fy_value_mat,
		CvMat* invert_hessian_mat
	) {
		return Calculate_Newton_Raphson_Get_Inverse_Hessian_Mat(
			reference_fx_value_mat, reference_fy_value_mat,
			invert_hessian_mat
			);
	}

	static bool Calculate_Newton_Raphson_Get_Inverse_Displacement_Delta( 
		CvMat* reference_template_value, 
		CvMat* reference_template_gradient_x_value, 
		CvMat* reference_template_gradient_y_value, 
		CvMat* deform_template_value, 
		double* reference_template_mean, 
		double* deform_template_mean, 
		double* passd_a, 
		CvMat* invert_hessian_mat, 
		CvMat* dispalcement_delta_mat 
		);


	//计算相关系数
	static bool Calculate_Newton_Raphson_Get_ZNCC_Correlation_Coefficient( 
		CvMat* reference_f_value_mat, 
		CvMat* target_g_value_mat , 
		double* reference_mean, 
		double* reference_second_order_center_moment, 
		double* target_mean, 
		double* passd_a, 
		double* correlation_coefficient 
		);
	static bool Calculate_Newton_Raphson_Get_ZNCC_Correlation_Coefficient_No_Passd( 
		CvMat* reference_f_value_mat, /*64F */
		CvMat* target_g_value_mat ,/*64F */ 
		double* reference_mean, 
		double* reference_second_order_center_moment, 
		double* target_mean, 
		double* correlation_coefficient 
		);
	// Overload of Calculate_Newton_Raphson_Get_ZNCC_Correlation_Coefficient_No_Passd
	static bool eval_zncc(
		cv::Mat& f, cv::Mat& g, double* f_mean, double* f_ssd, double* g_mean,
		double* coef) {
		auto _F = cvMat(f), _G = cvMat(g);
		return Calculate_Newton_Raphson_Get_ZNCC_Correlation_Coefficient_No_Passd(
			&_F, &_G, f_mean, f_ssd, g_mean, coef
		);
	}


	//辅助方法
	static bool CalculatePassdScaleValue( 
		CvMat* f_value_mat, 
		CvMat* g_value_mat, 
		double* f_mean, 
		double* g_mean, 
		double* passd_a_value 
		);
	static bool CalculateSecondOrderCenterMoment( 
		CvMat* value_mat, 
		double* mean_value, 
		double* result 
		);
	
	//更新位移方法
	static bool Calculate_Newton_Raphson_Get_Inverse_Displacement( 
		CvMat* dispalcement_input_mat, 
		CvMat* dispalcement_delta_mat, 
		CvMat* displacement_output_mat 
		);

	//12参数型函数
	static bool Get_Inverse_Hessian_Mat_12_Parameter(
		CvMat* reference_fx_value_mat, 
		CvMat* reference_fy_value_mat, 
		CvMat* invert_hessian_mat 
		);

	static bool Get_Inverse_Displacement_Delta_12_Parameter( 
		CvMat* reference_template_value, 
		CvMat* reference_template_gradient_x_value, 
		CvMat* reference_template_gradient_y_value, 
		CvMat* deform_template_value, 
		double* reference_template_mean, 
		double* deform_template_mean, 
		double* passd_a, 
		CvMat* invert_hessian_mat, 
		CvMat* dispalcement_delta_mat 
		);

	static bool Get_Inverse_Displacement_Result_12_Parameter(
		CvMat* dispalcement_input_mat, 
		CvMat* dispalcement_delta_mat, 
		CvMat* displacement_output_mat 
		);
	//20参数型函数
	static bool Get_Inverse_Hessian_Mat_20_Parameter( 
		CvMat* reference_fx_value_mat, 
		CvMat* reference_fy_value_mat, 
		CvMat* invert_hessian_mat 
		);
	static bool Get_Inverse_Displacement_Delta_20_Parameter( 
		CvMat* reference_template_value, 
		CvMat* reference_template_gradient_x_value, 
		CvMat* reference_template_gradient_y_value, 
		CvMat* deform_template_value, 
		double* reference_template_mean, 
		double* deform_template_mean, 
		double* passd_a, 
		CvMat* invert_hessian_mat, 
		CvMat* dispalcement_delta_mat 
		);

	//其他类
	static bool Reproject3DPoints( 
		const CvMat* left_image_point_pos_mat,/*1XN CV_64FC2 */ 
		const CvMat* right_image_point_pos_mat,/*同上 */ 
		const CvMat* left_intrinsic_matrix, 
		const CvMat* right_intrinsic_matrix, 
		const CvMat* left_distortion_coeffs, 
		const CvMat* right_distortion_coeffs, 
		const CvMat* rotation_matrix, 
		const CvMat* translation_vector, 
		CvMat* fundamental_matrix, 
		CvMat* image_points_3d_X,/*64F */ 
		CvMat* image_points_3d_Y,/*64F */ 
		CvMat* image_points_3d_Z/*64F */ 
		);

	//高斯核函数
	static bool Get_Gauss_Core(
		CvMat* core_mat,//可不用
		CvMat* convolution_core_mat,
		double sigma
		);
	static bool Image_Gauss_Filter(
		CvMat* image_input, 
		CvMat* convolution_core_mat, 
		CvMat* image_output 
		);
private:
	
};

using CorrelateKernels1394_t = CorrelationKernelMethods1394;