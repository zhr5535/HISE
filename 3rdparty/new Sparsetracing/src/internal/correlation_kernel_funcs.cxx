#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc/imgproc_c.h>

#include "utils/utils.hpp"
#include "correlation_kernel_funcs.h"

//////////////////////////////////////////////////////////////////////////
//无权重方法
bool CorrelateKernels1394_t::Calculate_Newton_Raphson_Get_Inverse_Hessian_Mat(
	CvMat* reference_fx_value_mat, 
	CvMat* reference_fy_value_mat, 
	CvMat* invert_hessian_mat 
	)
{
	//Check the dimension
	int template_width =reference_fx_value_mat->cols;
	int template_height = reference_fx_value_mat->rows;
	if (template_width%2==0 || template_height%2==0)
	{
		return false;
	}
	int template_half_width = (template_width-1)/2;
	int template_half_height = (template_height-1)/2;
	int template_point_count = template_height* template_width;

	if (reference_fy_value_mat->cols!=template_width ||
		reference_fy_value_mat->rows!=template_height)
	{
		return false;
	}
	if (invert_hessian_mat->cols!=6||invert_hessian_mat->rows!=6)
	{
		return false;
	}


	//类型
	if ( CV_MAT_TYPE(reference_fx_value_mat->type) != CV_64F||
		CV_MAT_TYPE(reference_fy_value_mat->type) != CV_64F||
		CV_MAT_TYPE(invert_hessian_mat->type) != CV_64F)
	{
		return false;
	}

	//double* fx_ptr= reference_fx_value_mat->data.db;
	//double* fy_ptr = reference_fy_value_mat->data.db;

	double hessian_data[36]={0};
	CvMat hessian_mat = cvMat(6,6,CV_64F,hessian_data);
	cvZero(&hessian_mat);

	//////////////////////////////////////////////////////////////////////////
	//method 1
	for (int m = -template_half_height;m<=template_half_width;m++)
	{
		for (int n= -template_half_width;n<=template_half_width;n++)
		{
			//使用GetSubRect取得的子矩阵，用指针寻址会出问题
			//double fx = fx_ptr[(m+template_half_height)*template_width+n+template_half_width];
			//double fy = fy_ptr[(m+template_half_height)*template_width+n+template_half_width];


			double fx = CV_MAT_ELEM(*reference_fx_value_mat,double,m+template_half_height,n+template_half_width);
			double fy = CV_MAT_ELEM(*reference_fy_value_mat,double,m+template_half_height,n+template_half_width);


			hessian_data[0] += fx * fx;
			hessian_data[1] += fx * fx * (double)n ;
			hessian_data[2] += fx * fx * (double)m;
			hessian_data[3] += fx * fy ;
			hessian_data[4] += fx * fy * (double)n;
			hessian_data[5] += fx * fy * (double)m;

			hessian_data[6] += fx * (double)n * fx;
			hessian_data[7] += fx * (double)n * fx * (double)n;
			hessian_data[8] += fx * (double)n * fx * (double)m;
			hessian_data[9] += fx * (double)n * fy ;
			hessian_data[10] += fx * (double)n * fy *(double)n;
			hessian_data[11] += fx * (double)n *fy * (double)m;

			hessian_data[12] += fx * (double)m * fx;
			hessian_data[13] += fx * (double)m * fx * (double)n;
			hessian_data[14] += fx * (double)m * fx * (double)m;
			hessian_data[15] += fx * (double)m * fy ;
			hessian_data[16] += fx * (double)m * fy *(double)n;
			hessian_data[17] += fx * (double)m * fy *(double)m;


			hessian_data[18] += fy * fx;
			hessian_data[19] += fy * fx *(double)n;
			hessian_data[20] += fy * fx *(double)m;
			hessian_data[21] += fy * fy;
			hessian_data[22] += fy * fy *(double)n;
			hessian_data[23] += fy * fy *(double)m;

			hessian_data[24] += fy* (double)n * fx;
			hessian_data[25] += fy* (double)n * fx * (double)n;
			hessian_data[26] += fy* (double)n * fx * (double)m;
			hessian_data[27] += fy* (double)n * fy;
			hessian_data[28] += fy* (double)n * fy * (double)n;
			hessian_data[29] += fy* (double)n * fy * (double)m;

			hessian_data[30] += fy * (double)m * fx;
			hessian_data[31] += fy * (double)m * fx * (double)n;
			hessian_data[32] += fy * (double)m * fx * (double)m;
			hessian_data[33] += fy * (double)m * fy ;
			hessian_data[34] += fy * (double)m * fy * (double)n;
			hessian_data[35] += fy * (double)m * fy * (double)m;
		}
	}
	//////////////////////////////////////////////////////////////////////////


	//////////////////////////////////////////////////////////////////////////
	//method 2

	//CvMat* jacobi_mat = cvCreateMat(template_point_count,6,CV_64F);
	//CvMat* jacobi_T_mat = cvCreateMat(6,template_point_count,CV_64F);
	//for (int m = -template_half_height;m<=template_half_width;m++)
	//{
	//	for (int n= -template_half_width;n<=template_half_width;n++)
	//	{
	//		double fx = CV_MAT_ELEM(*reference_fx_value_mat,double,m+template_half_height,n+template_half_width);
	//		double fy = CV_MAT_ELEM(*reference_fy_value_mat,double,m+template_half_height,n+template_half_width);

	//		int index = (m+template_half_height) *template_width + n+template_half_width;

	//		CV_MAT_ELEM(*jacobi_mat,double,index,0) = fx;
	//		CV_MAT_ELEM(*jacobi_mat,double,index,1) = fx *(double)n;
	//		CV_MAT_ELEM(*jacobi_mat,double,index,2) = fx *(double)m;
	//		CV_MAT_ELEM(*jacobi_mat,double,index,3) = fy;
	//		CV_MAT_ELEM(*jacobi_mat,double,index,4) = fy *(double)n;
	//		CV_MAT_ELEM(*jacobi_mat,double,index,5) = fy *(double)m;


	//		CV_MAT_ELEM(*jacobi_T_mat,double,0,index) = fx;
	//		CV_MAT_ELEM(*jacobi_T_mat,double,1,index) = fx *(double)n;
	//		CV_MAT_ELEM(*jacobi_T_mat,double,2,index) = fx *(double)m;
	//		CV_MAT_ELEM(*jacobi_T_mat,double,3,index) = fy;
	//		CV_MAT_ELEM(*jacobi_T_mat,double,4,index) = fy *(double)n;
	//		CV_MAT_ELEM(*jacobi_T_mat,double,5,index) = fy *(double)m;
	//	}
	//}
	//cvMatMul(jacobi_T_mat,jacobi_mat,&hessian_mat);

	//cvReleaseMat(&jacobi_mat);
	//cvReleaseMat(&jacobi_T_mat);
	//////////////////////////////////////////////////////////////////////////


	double det = cvInvert(&hessian_mat,invert_hessian_mat);
	return true;

}

bool CorrelateKernels1394_t::Calculate_Newton_Raphson_Get_Inverse_Displacement_Delta(
	CvMat* reference_template_value, 
	CvMat* reference_template_gradient_x_value, 
	CvMat* reference_template_gradient_y_value, 
	CvMat* deform_template_value, 
	double* reference_template_mean,
	double* deform_template_mean,
	double* passd_a,
	CvMat* invert_hessian_mat,
	CvMat* dispalcement_delta_mat
	)
{
	//判断
	//大小
	int template_width =reference_template_value->cols;
	int template_height = reference_template_value->rows;
	if (template_width%2==0 || template_height%2==0)
	{
		return false;
	}
	int template_half_width = (template_width-1)/2;
	int template_half_height = (template_height-1)/2;

	if (reference_template_gradient_x_value->cols!=template_width ||
		reference_template_gradient_x_value->rows!=template_height||
		reference_template_gradient_y_value->cols!=template_width ||
		reference_template_gradient_y_value->rows!=template_height||
		deform_template_value->cols!=template_width ||
		deform_template_value->rows!=template_height
		)
	{
		return false;
	}
	if (/*dispalcement_input_mat->cols!=1||
		dispalcement_input_mat->rows!=6||*/
		dispalcement_delta_mat->cols!=1||
		dispalcement_delta_mat->rows!=6)
	{
		return false;
	}
	if (invert_hessian_mat->cols!=6||
		invert_hessian_mat->rows!=6)
	{
		return false;
	}


	//类型
	int reference_type = 0;
	if ( CV_MAT_TYPE(reference_template_value->type) ==CV_8U)
	{
		reference_type =1;
	}
	else if ( CV_MAT_TYPE(reference_template_value->type) == CV_64F)
	{
		reference_type =2;
	} 
	else
	{
		return false;
	}

	if (
		CV_MAT_TYPE(reference_template_gradient_x_value->type) != CV_64F||
		CV_MAT_TYPE(reference_template_gradient_y_value->type) != CV_64F||
		CV_MAT_TYPE(deform_template_value->type) != CV_64F||
		//CV_MAT_TYPE(dispalcement_input_mat->type) != CV_64F||
		CV_MAT_TYPE(dispalcement_delta_mat->type) != CV_64F||
		CV_MAT_TYPE(invert_hessian_mat->type) != CV_64F
		)
	{
		return false;
	}
	if (reference_template_mean==NULL ||
		deform_template_mean==NULL||
		passd_a == NULL)
	{
		return false;
	}

	//double* f_ptr = reference_template_value->data.db;
	//double* fx_ptr = reference_template_gradient_x_value->data.db;
	//double* fy_ptr = reference_template_gradient_y_value->data.db;
	double* g_ptr = deform_template_value->data.db;


	double f_mean = *reference_template_mean;
	double g_mean = *deform_template_mean;
	double passd_a_value = *passd_a;



	double patial_c_p_ptr[6] = {0};
	CvMat patial_c_p_mat = cvMat(6,1,CV_64F,patial_c_p_ptr);
	if (reference_type==1)
	{
		//uchar* f_ptr_uchar = reference_template_value->data.ptr;
		for (int m = -template_half_height;m<=template_half_width;m++)
		{
			for (int n= -template_half_width;n<=template_half_width;n++)
			{
				double f = CV_MAT_ELEM(*reference_template_value,uchar,m+template_half_height,n+template_half_width);
				double fx = CV_MAT_ELEM(*reference_template_gradient_x_value,double,m+template_half_height,n+template_half_width);
				double fy = CV_MAT_ELEM(*reference_template_gradient_y_value,double,m+template_half_height,n+template_half_width);
				double g = CV_MAT_ELEM(*deform_template_value,double,m+template_half_height,n+template_half_width);

				//第一种
				patial_c_p_ptr[0] += fx*(f - passd_a_value * g + passd_a_value*g_mean-f_mean);
				patial_c_p_ptr[1] += fx*n*(f - passd_a_value * g + passd_a_value*g_mean-f_mean);
				patial_c_p_ptr[2] += fx*m*(f - passd_a_value * g + passd_a_value*g_mean-f_mean);
				patial_c_p_ptr[3] += fy*(f - passd_a_value * g + passd_a_value*g_mean-f_mean);
				patial_c_p_ptr[4] += fy*n*(f - passd_a_value * g + passd_a_value*g_mean-f_mean);
				patial_c_p_ptr[5] += fy*m*(f - passd_a_value * g + passd_a_value*g_mean-f_mean);

			}
		}
	} 
	else
	{
		//double* f_ptr_double = reference_template_value->data.db;
		for (int m = -template_half_height;m<=template_half_width;m++)
		{
			for (int n= -template_half_width;n<=template_half_width;n++)
			{
				double f = CV_MAT_ELEM(*reference_template_value,double,m+template_half_height,n+template_half_width);
				double fx = CV_MAT_ELEM(*reference_template_gradient_x_value,double,m+template_half_height,n+template_half_width);
				double fy = CV_MAT_ELEM(*reference_template_gradient_y_value,double,m+template_half_height,n+template_half_width);
				double g = CV_MAT_ELEM(*deform_template_value,double,m+template_half_height,n+template_half_width);

				//第一种 PASSD
				patial_c_p_ptr[0] += fx*(f - passd_a_value * g + passd_a_value*g_mean-f_mean);
				patial_c_p_ptr[1] += fx*n*(f - passd_a_value * g + passd_a_value*g_mean-f_mean);
				patial_c_p_ptr[2] += fx*m*(f - passd_a_value * g + passd_a_value*g_mean-f_mean);
				patial_c_p_ptr[3] += fy*(f - passd_a_value * g + passd_a_value*g_mean-f_mean);
				patial_c_p_ptr[4] += fy*n*(f - passd_a_value * g + passd_a_value*g_mean-f_mean);
				patial_c_p_ptr[5] += fy*m*(f - passd_a_value * g + passd_a_value*g_mean-f_mean);
			}
		}
	}
	//double avg1 = cvAvg(reference_template_value).val[0];
	//double avg2 = cvAvg(reference_template_gradient_x_value).val[0];
	//double avg3 =cvAvg(reference_template_gradient_y_value).val[0];
	//double avg4 = cvAvg(&patial_c_p_mat).val[0];

	double* h_ptr = invert_hessian_mat->data.db;
	//double* p = dispalcement_input_mat->data.db;
	double * dp=  dispalcement_delta_mat->data.db;

	//double a[36];
	//CvMat a_mat = cvMat(6,6,CV_64F,a);
	//cvCopy(invert_hessian_mat,&a_mat);
	//CvScalar avg,std;
	//cvAvgSdv(reference_template_gradient_x_value,&avg,&std);
	//cvAvgSdv(reference_template_gradient_y_value,&avg,&std);

	
	dp[0] = -patial_c_p_ptr[0] * h_ptr[0] -patial_c_p_ptr[1] * h_ptr[1] -patial_c_p_ptr[2] * h_ptr[2] -patial_c_p_ptr[3] * h_ptr[3] -patial_c_p_ptr[4] * h_ptr[4] -patial_c_p_ptr[5] * h_ptr[5];
	dp[1] = -patial_c_p_ptr[0] * h_ptr[6] -patial_c_p_ptr[1] * h_ptr[7] -patial_c_p_ptr[2] * h_ptr[8] -patial_c_p_ptr[3] * h_ptr[9] -patial_c_p_ptr[4] * h_ptr[10] -patial_c_p_ptr[5] * h_ptr[11];
	dp[2] = -patial_c_p_ptr[0] * h_ptr[12] -patial_c_p_ptr[1] * h_ptr[13] -patial_c_p_ptr[2] * h_ptr[14] -patial_c_p_ptr[3] * h_ptr[15] -patial_c_p_ptr[4] * h_ptr[16] -patial_c_p_ptr[5] * h_ptr[17];
	dp[3] = -patial_c_p_ptr[0] * h_ptr[18] -patial_c_p_ptr[1] * h_ptr[19] -patial_c_p_ptr[2] * h_ptr[20] -patial_c_p_ptr[3] * h_ptr[21] -patial_c_p_ptr[4] * h_ptr[22] -patial_c_p_ptr[5] * h_ptr[23];
	dp[4] = -patial_c_p_ptr[0] * h_ptr[24] -patial_c_p_ptr[1] * h_ptr[25] -patial_c_p_ptr[2] * h_ptr[26] -patial_c_p_ptr[3] * h_ptr[27] -patial_c_p_ptr[4] * h_ptr[28] -patial_c_p_ptr[5] * h_ptr[29];
	dp[5] = -patial_c_p_ptr[0] * h_ptr[30] -patial_c_p_ptr[1] * h_ptr[31] -patial_c_p_ptr[2] * h_ptr[32] -patial_c_p_ptr[3] * h_ptr[33] -patial_c_p_ptr[4] * h_ptr[34] -patial_c_p_ptr[5] * h_ptr[35];


	//此处问题
	//dp[0] = (-patial_c_p_ptr[0] * h_ptr[0] -patial_c_p_ptr[1] * h_ptr[1] -patial_c_p_ptr[2] * h_ptr[2] -patial_c_p_ptr[3] * h_ptr[3] -patial_c_p_ptr[4] * h_ptr[4] -patial_c_p_ptr[5] * h_ptr[5])/passd_a_value;
	//dp[1] = -patial_c_p_ptr[0] * h_ptr[6] -patial_c_p_ptr[1] * h_ptr[7] -patial_c_p_ptr[2] * h_ptr[8] -patial_c_p_ptr[3] * h_ptr[9] -patial_c_p_ptr[4] * h_ptr[10] -patial_c_p_ptr[5] * h_ptr[11]/passd_a_value;
	//dp[2] = -patial_c_p_ptr[0] * h_ptr[12] -patial_c_p_ptr[1] * h_ptr[13] -patial_c_p_ptr[2] * h_ptr[14] -patial_c_p_ptr[3] * h_ptr[15] -patial_c_p_ptr[4] * h_ptr[16] -patial_c_p_ptr[5] * h_ptr[17]/passd_a_value;
	//dp[3] = -patial_c_p_ptr[0] * h_ptr[18] -patial_c_p_ptr[1] * h_ptr[19] -patial_c_p_ptr[2] * h_ptr[20] -patial_c_p_ptr[3] * h_ptr[21] -patial_c_p_ptr[4] * h_ptr[22] -patial_c_p_ptr[5] * h_ptr[23]/passd_a_value;
	//dp[4] = -patial_c_p_ptr[0] * h_ptr[24] -patial_c_p_ptr[1] * h_ptr[25] -patial_c_p_ptr[2] * h_ptr[26] -patial_c_p_ptr[3] * h_ptr[27] -patial_c_p_ptr[4] * h_ptr[28] -patial_c_p_ptr[5] * h_ptr[29]/passd_a_value;
	//dp[5] = -patial_c_p_ptr[0] * h_ptr[30] -patial_c_p_ptr[1] * h_ptr[31] -patial_c_p_ptr[2] * h_ptr[32] -patial_c_p_ptr[3] * h_ptr[33] -patial_c_p_ptr[4] * h_ptr[34] -patial_c_p_ptr[5] * h_ptr[35]/passd_a_value;


	return true;
}

bool CorrelateKernels1394_t::Calculate_Newton_Raphson_Get_Inverse_Displacement(
	CvMat* dispalcement_input_mat, 
	CvMat* dispalcement_delta_mat, 
	CvMat* displacement_output_mat 
	)
{

	if (dispalcement_input_mat->cols!=1||
		dispalcement_input_mat->rows!=6||
		dispalcement_delta_mat->cols!=1||
		dispalcement_delta_mat->rows!=6||
		displacement_output_mat->cols!=1||
		displacement_output_mat->rows!=6
		)
	{
		return false;
	}

	if (
		CV_MAT_TYPE(dispalcement_input_mat->type) != CV_64F||
		CV_MAT_TYPE(dispalcement_delta_mat->type) != CV_64F||
		CV_MAT_TYPE(displacement_output_mat->type) != CV_64F
		)
	{
		return false;
	}

	double* p = dispalcement_input_mat->data.db;
	double * dp = dispalcement_delta_mat->data.db;
	double* p_new = displacement_output_mat->data.db;


	double displacement_new_denominator = -dp[4]*dp[2] +1+dp[5] +dp[1] +dp[1]*dp[5];
	p_new[1] =((1+p[1]) * (1+dp[5]) - p[2] * dp[4])/displacement_new_denominator - 1.0;
	p_new[2] =(-(1+p[1])*dp[2] + p[2]*(1+dp[1]))/displacement_new_denominator;
	p_new[0] = (-(1+p[1])*(-dp[3]*dp[2] + dp[0] + dp[0] *dp[5]) + p[2] * (-dp[3] -dp[3]*dp[1] +dp[0]*dp[4]) )/displacement_new_denominator +p[0];
	p_new[4] = (p[4] *(1+dp[5]) -(1+p[5]) * dp[4])/displacement_new_denominator;
	p_new[5] = (-p[4] * dp[2] +(1+p[5]) * (1+dp[1]))/displacement_new_denominator -1.0;
	p_new[3] = (-p[4] *( -dp[3] * dp[2] + dp[0] +dp[0]*dp[5]) + ( 1+p[5] )* (-dp[3] -dp[3]*dp[1] +dp[0] *dp[4]) )/displacement_new_denominator +p[3];


	return true;
}

bool CorrelateKernels1394_t::Calculate_Newton_Raphson_Get_ZNCC_Correlation_Coefficient(
	CvMat* reference_f_value_mat, 
	CvMat* target_g_value_mat ,
	double* reference_mean,
	double* reference_second_order_center_moment,
	double* target_mean,
	double* passd_a,
	double* correlation_coefficient
	)
{

	//判断
	//大小
	int template_width =reference_f_value_mat->cols;
	int template_height = reference_f_value_mat->rows;

	if (
		target_g_value_mat->cols!=template_width ||
		target_g_value_mat->rows!=template_height
		)
	{
		return false;
	}
	if (reference_mean==NULL ||
		reference_second_order_center_moment ==NULL||
		target_mean==NULL||
		passd_a == NULL||
		correlation_coefficient == NULL)
	{
		return false;
	}
	double f_mean = *reference_mean;
	double f_second_order_center_moment= *reference_second_order_center_moment;
	double g_mean = *target_mean;
	double passd_a_value = *passd_a;


	//类型
	int referecnec_image_type = 0;
	if ( CV_MAT_TYPE(reference_f_value_mat->type) ==CV_8U)
	{
		referecnec_image_type =1;
	}
	else if ( CV_MAT_TYPE(reference_f_value_mat->type) == CV_64F)
	{
		referecnec_image_type =2;
	} 
	else
	{
		return false;
	}
	int target_image_type = 0;
	if ( CV_MAT_TYPE(target_g_value_mat->type) == CV_8U)
	{
		target_image_type=1;
	}
	else if (CV_MAT_TYPE(target_g_value_mat->type) == CV_64F)
	{
		target_image_type=2;
	} 
	else
	{
		return false;
	}
	double coe = 0.0;
	for (int i =0;i<template_height;i++)
	{
		for (int j = 0;j<template_width;j++)
		{
			double f_value,g_value;
			if (referecnec_image_type==1)
			{
				f_value =(double)CV_MAT_ELEM(*reference_f_value_mat,uchar,i,j);
			}
			else
			{
				f_value =CV_MAT_ELEM(*reference_f_value_mat,double,i,j);
			}

			if (target_image_type==1)
			{
				g_value =(double)CV_MAT_ELEM(*target_g_value_mat,uchar,i,j);
			}
			else
			{
				g_value= CV_MAT_ELEM(*target_g_value_mat,double,i,j);
			}
			coe+= ((f_value - f_mean )- passd_a_value* (g_value - g_mean)) *
				((f_value - f_mean )- passd_a_value* (g_mean - g_mean));
		}
	}
	*correlation_coefficient =sqrt(1.0-coe/f_second_order_center_moment) ;
	return true;
}

bool CorrelateKernels1394_t::Calculate_Newton_Raphson_Get_ZNCC_Correlation_Coefficient_No_Passd(
	CvMat* reference_f_value_mat, //64F
	CvMat* target_g_value_mat,//64F
	double* reference_mean,
	double* reference_second_order_center_moment,
	double* target_mean,
	double* correlation_coefficient
)
{
	const int template_width = reference_f_value_mat->cols;
	const int template_height = reference_f_value_mat->rows;
	const double f_mean = *reference_mean;
	const double f_second_order_center_moment = *reference_second_order_center_moment;
	const double g_mean = *target_mean;

	double deform_template_second_order_center_moment = 0.0;
	double cross_second_order_center_moment = 0.0;
	if (CV_32F == CV_MAT_TYPE(reference_f_value_mat->type)) {
		for (int i = 0; i < template_height; i++) {
			for (int j = 0; j < template_width; j++) {
				const double f_value = CV_MAT_ELEM(*reference_f_value_mat, float, i, j);
				const double g_value = CV_MAT_ELEM(*target_g_value_mat, float, i, j);
				deform_template_second_order_center_moment += (g_value - g_mean) * (g_value - g_mean);
				cross_second_order_center_moment += (f_value - f_mean) * (g_value - g_mean);
			}
		}
	}
	else if (CV_64F == CV_MAT_TYPE(reference_f_value_mat->type)) {
		for (int i = 0; i < template_height; i++) {
			for (int j = 0; j < template_width; j++) {
				const auto f_value = CV_MAT_ELEM(*reference_f_value_mat, double, i, j);
				const auto g_value = CV_MAT_ELEM(*target_g_value_mat, double, i, j);
				deform_template_second_order_center_moment += (g_value - g_mean) * (g_value - g_mean);
				cross_second_order_center_moment += (f_value - f_mean) * (g_value - g_mean);
			}
		}
	}
	else {
		for (int i = 0; i < template_height; i++) {
			for (int j = 0; j < template_width; j++) {
				const double f_value = CV_MAT_ELEM(*reference_f_value_mat, uchar, i, j);
				const double g_value = CV_MAT_ELEM(*target_g_value_mat, double, i, j);
				deform_template_second_order_center_moment += (g_value - g_mean) * (g_value - g_mean);
				cross_second_order_center_moment += (f_value - f_mean) * (g_value - g_mean);
			}
		}
	}

	double deno = sqrt(f_second_order_center_moment * deform_template_second_order_center_moment);
	if (deno > 0) {
		*correlation_coefficient = cross_second_order_center_moment / deno;
		return true;
	}
	else { *correlation_coefficient = -99; return false; }
}

bool CorrelateKernels1394_t::CalculatePassdScaleValue(
	CvMat* f_value_mat, 
	CvMat* g_value_mat, 
	double* f_mean, 
	double* g_mean, 
	double* passd_a_value 
	)
{
	//判断
	//大小
	int template_width =f_value_mat->cols;
	int template_height = f_value_mat->rows;

	if (
		g_value_mat->cols!=template_width ||
		g_value_mat->rows!=template_height
		)
	{
		return false;
	}
	if (f_mean==NULL ||
		g_mean==NULL||
		passd_a_value == NULL)
	{
		return false;
	}
	double fm = *f_mean;
	double gm = *g_mean;

	//类型
	int f_type = 0;
	if ( CV_MAT_TYPE(f_value_mat->type) ==CV_8U)
	{
		f_type =1;
	}
	else if ( CV_MAT_TYPE(f_value_mat->type) == CV_64F)
	{
		f_type =2;
	} 
	else
	{
		return false;
	}
	int g_type = 0;
	if ( CV_MAT_TYPE(g_value_mat->type) == CV_8U)
	{
		g_type=1;
	}
	else if (CV_MAT_TYPE(g_value_mat->type) == CV_64F)
	{
		g_type=2;
	} 
	else
	{
		return false;
	}
	double numerator = 0.0;
	double denominator =0.0;
	for (int i =0;i<template_height;i++)
	{
		for (int j =0;j<template_width;j++)
		{
			double f_value,g_value;
			if (f_type==1)
			{
				f_value =(double)CV_MAT_ELEM(*f_value_mat,uchar,i,j);
			}
			else
			{
				f_value= CV_MAT_ELEM(*f_value_mat,double,i,j);
			}

			if (g_type==1)
			{
				g_value =(double)CV_MAT_ELEM(*g_value_mat,uchar,i,j);
			}
			else
			{
				g_value= CV_MAT_ELEM(*g_value_mat,double,i,j);
			}
			numerator+= (f_value-fm) * (g_value-gm);
			denominator+= (g_value-gm) * (g_value-gm);
		}
	}
	*passd_a_value = numerator/denominator;
	return true;
}


bool CorrelateKernels1394_t::CalculateSecondOrderCenterMoment(
	CvMat* value_mat, double* mean_value, double* result)
{

	int mat_width = value_mat->cols;
	int mat_height =value_mat->rows;
	int value_mat_type = 0;//1--CV_8U,2--CV_64F
	if (CV_MAT_TYPE(value_mat->type) == CV_8U )
	{
		value_mat_type =1;
	}
	else if (CV_MAT_TYPE(value_mat->type) == CV_64F )
	{
		value_mat_type=2;
	}
	else 
	{
		return false;
	}

	double mean;
	if (mean_value==NULL)
	{
		mean = cvAvg(value_mat).val[0];
	}
	else
	{
		mean = *mean_value;
	}

	double second_order_center_moment =0.0;

	for (int i =0;i<mat_height;i++)
	{
		for (int j =0;j<mat_width;j++)
		{
			double f_val;
			if (value_mat_type==1)//CV_8U
			{
				f_val = (double)CV_MAT_ELEM(*value_mat,uchar,i,j);

			}
			else//CV_64F
			{
				f_val = CV_MAT_ELEM(*value_mat,double,i,j);
			}
			second_order_center_moment+=(f_val-mean)*(f_val-mean);
		}
	}		
	*result = second_order_center_moment;
	return true;
}

//////////////////////////////////////////////////////////////////////////

bool CorrelateKernels1394_t::Get_Inverse_Hessian_Mat_12_Parameter( 
	CvMat* reference_fx_value_mat, 
	CvMat* reference_fy_value_mat, 
	CvMat* invert_hessian_mat 
	)
{
	//判断
	//大小
	int template_width =reference_fx_value_mat->cols;
	int template_height = reference_fx_value_mat->rows;
	if (template_width%2==0 || template_height%2==0)
	{
		return false;
	}
	int template_half_width = (template_width-1)/2;
	int template_half_height = (template_height-1)/2;
	int template_mat_count = template_height*template_width;

	if (reference_fy_value_mat->cols!=template_width ||
		reference_fy_value_mat->rows!=template_height)
	{
		return false;
	}
	if (invert_hessian_mat->cols!=12||
		invert_hessian_mat->rows!=12)
	{
		return false;
	}


	//类型
	if ( CV_MAT_TYPE(reference_fx_value_mat->type) != CV_64F||
		CV_MAT_TYPE(reference_fy_value_mat->type) != CV_64F||
		CV_MAT_TYPE(invert_hessian_mat->type) != CV_64F)
	{
		return false;
	}

	//double* fx_ptr= reference_fx_value_mat->data.db;
	//double* fy_ptr = reference_fy_value_mat->data.db;

	//double hessian_data[36]={0};
	//for (int m = -template_half_height;m<=template_half_width;m++)
	//{
	//	for (int n= -template_half_width;n<=template_half_width;n++)
	//	{
	//		//使用GetSubRect取得的子矩阵，用指针寻址会出问题
	//		//double fx = fx_ptr[(m+template_half_height)*template_width+n+template_half_width];
	//		//double fy = fy_ptr[(m+template_half_height)*template_width+n+template_half_width];


	//		double fx = CV_MAT_ELEM(*reference_fx_value_mat,double,m+template_half_height,n+template_half_width);
	//		double fy = CV_MAT_ELEM(*reference_fy_value_mat,double,m+template_half_height,n+template_half_width);


	//		hessian_data[0] += fx * fx;
	//		hessian_data[1] += fx * fx * (double)n ;
	//		hessian_data[2] += fx * fx * (double)m;
	//		hessian_data[3] += fx * fy ;
	//		hessian_data[4] += fx * fy * (double)n;
	//		hessian_data[5] += fx * fy * (double)m;

	//		hessian_data[6] += fx * (double)n * fx;
	//		hessian_data[7] += fx * (double)n * fx * (double)n;
	//		hessian_data[8] += fx * (double)n * fx * (double)m;
	//		hessian_data[9] += fx * (double)n * fy ;
	//		hessian_data[10] += fx * (double)n * fy *(double)n;
	//		hessian_data[11] += fx * (double)n *fy * (double)m;

	//		hessian_data[12] += fx * (double)m * fx;
	//		hessian_data[13] += fx * (double)m * fx * (double)n;
	//		hessian_data[14] += fx * (double)m * fx * (double)m;
	//		hessian_data[15] += fx * (double)m * fy ;
	//		hessian_data[16] += fx * (double)m * fy *(double)n;
	//		hessian_data[17] += fx * (double)m * fy *(double)m;


	//		hessian_data[18] += fy * fx;
	//		hessian_data[19] += fy * fx *(double)n;
	//		hessian_data[20] += fy * fx *(double)m;
	//		hessian_data[21] += fy * fy;
	//		hessian_data[22] += fy * fy *(double)n;
	//		hessian_data[23] += fy * fy *(double)m;

	//		hessian_data[24] += fy* (double)n * fx;
	//		hessian_data[25] += fy* (double)n * fx * (double)n;
	//		hessian_data[26] += fy* (double)n * fx * (double)m;
	//		hessian_data[27] += fy* (double)n * fy;
	//		hessian_data[28] += fy* (double)n * fy * (double)n;
	//		hessian_data[29] += fy* (double)n * fy * (double)m;

	//		hessian_data[30] += fy * (double)m * fx;
	//		hessian_data[31] += fy * (double)m * fx * (double)n;
	//		hessian_data[32] += fy * (double)m * fx * (double)m;
	//		hessian_data[33] += fy * (double)m * fy ;
	//		hessian_data[34] += fy * (double)m * fy * (double)n;
	//		hessian_data[35] += fy * (double)m * fy * (double)m;
	//	}
	//}

	//test
	double hessian_data[144];
	CvMat hessian_mat = cvMat(12,12,CV_64F,hessian_data);
	cvZero(&hessian_mat);



	//////////////////////////////////////////////////////////////////////////
	//method 1
	//CvMat* vector_1 = cvCreateMat(12,1,CV_64F);
	//CvMat* vector_1_trans = cvCreateMat(1,12,CV_64F);
	//CvMat* vector_mul = cvCreateMat(12,12,CV_64F);
	//CvMat* cache =cvCreateMat(12,12,CV_64F);
	//for (int m = -template_half_height;m<=template_half_width;m++)
	//{
	//	for (int n= -template_half_width;n<=template_half_width;n++)
	//	{
	//		double fx = CV_MAT_ELEM(*reference_fx_value_mat,double,m+template_half_height,n+template_half_width);
	//		double fy = CV_MAT_ELEM(*reference_fy_value_mat,double,m+template_half_height,n+template_half_width);


	//		CV_MAT_ELEM(*vector_1,double,0,0) = fx;
	//		CV_MAT_ELEM(*vector_1,double,1,0) = fx *(double)n;
	//		CV_MAT_ELEM(*vector_1,double,2,0) = fx *(double)m;
	//		CV_MAT_ELEM(*vector_1,double,3,0) = fx * 0.5 *(double)n*(double)n;
	//		CV_MAT_ELEM(*vector_1,double,4,0) = fx * 0.5 *(double)m*(double)m;
	//		CV_MAT_ELEM(*vector_1,double,5,0) = fx *(double)n*(double)m;

	//		CV_MAT_ELEM(*vector_1,double,6,0) = fy;
	//		CV_MAT_ELEM(*vector_1,double,7,0) = fy *(double)n;
	//		CV_MAT_ELEM(*vector_1,double,8,0) = fy *(double)m;
	//		CV_MAT_ELEM(*vector_1,double,9,0) = fy * 0.5 *(double)n*(double)n;
	//		CV_MAT_ELEM(*vector_1,double,10,0) = fy * 0.5 *(double)m*(double)m;
	//		CV_MAT_ELEM(*vector_1,double,11,0) = fy *(double)n*(double)m;

	//		cvTranspose(vector_1,vector_1_trans);
	//		cvMatMul(vector_1,vector_1_trans,vector_mul);

	//		cvCopy(&hessian_mat,cache);
	//		cvAdd(vector_mul,cache,&hessian_mat);




	//	}
	//}

	//cvReleaseMat(&vector_1);
	//cvReleaseMat(&vector_1_trans);
	//cvReleaseMat(&vector_mul);
	//cvReleaseMat(&cache);
	//////////////////////////////////////////////////////////////////////////


	//////////////////////////////////////////////////////////////////////////
	//method 2
	CvMat* jacobi_mat = cvCreateMat(template_mat_count,12,CV_64F);
	CvMat* jacobi_T_mat = cvCreateMat(12,template_mat_count,CV_64F);
	for (int m = -template_half_height;m<=template_half_width;m++)
	{
		for (int n= -template_half_width;n<=template_half_width;n++)
		{
			double fx = CV_MAT_ELEM(*reference_fx_value_mat,double,m+template_half_height,n+template_half_width);
			double fy = CV_MAT_ELEM(*reference_fy_value_mat,double,m+template_half_height,n+template_half_width);

			int index = (m+template_half_height) *template_width + n+template_half_width;

			CV_MAT_ELEM(*jacobi_mat,double,index,0) = fx;
			CV_MAT_ELEM(*jacobi_mat,double,index,1) = fx *(double)n;
			CV_MAT_ELEM(*jacobi_mat,double,index,2) = fx *(double)m;
			CV_MAT_ELEM(*jacobi_mat,double,index,3) = fx * 0.5 *(double)n*(double)n;
			CV_MAT_ELEM(*jacobi_mat,double,index,4) = fx * 0.5 *(double)m*(double)m;
			CV_MAT_ELEM(*jacobi_mat,double,index,5) = fx *(double)n*(double)m;

			CV_MAT_ELEM(*jacobi_mat,double,index,6) = fy;
			CV_MAT_ELEM(*jacobi_mat,double,index,7) = fy *(double)n;
			CV_MAT_ELEM(*jacobi_mat,double,index,8) = fy *(double)m;
			CV_MAT_ELEM(*jacobi_mat,double,index,9) = fy * 0.5 *(double)n*(double)n;
			CV_MAT_ELEM(*jacobi_mat,double,index,10) = fy * 0.5 *(double)m*(double)m;
			CV_MAT_ELEM(*jacobi_mat,double,index,11) = fy *(double)n*(double)m;

			CV_MAT_ELEM(*jacobi_T_mat,double,0,index) = fx;
			CV_MAT_ELEM(*jacobi_T_mat,double,1,index) = fx *(double)n;
			CV_MAT_ELEM(*jacobi_T_mat,double,2,index) = fx *(double)m;
			CV_MAT_ELEM(*jacobi_T_mat,double,3,index) = fx * 0.5 *(double)n*(double)n;
			CV_MAT_ELEM(*jacobi_T_mat,double,4,index) = fx * 0.5 *(double)m*(double)m;
			CV_MAT_ELEM(*jacobi_T_mat,double,5,index) = fx *(double)n*(double)m;

			CV_MAT_ELEM(*jacobi_T_mat,double,6,index) = fy;
			CV_MAT_ELEM(*jacobi_T_mat,double,7,index) = fy *(double)n;
			CV_MAT_ELEM(*jacobi_T_mat,double,8,index) = fy *(double)m;
			CV_MAT_ELEM(*jacobi_T_mat,double,9,index) = fy * 0.5 *(double)n*(double)n;
			CV_MAT_ELEM(*jacobi_T_mat,double,10,index) = fy * 0.5 *(double)m*(double)m;
			CV_MAT_ELEM(*jacobi_T_mat,double,11,index) = fy *(double)n*(double)m;
		}
	}
	cvMatMul(jacobi_T_mat,jacobi_mat,&hessian_mat);

	cvReleaseMat(&jacobi_mat);
	cvReleaseMat(&jacobi_T_mat);
	

	//////////////////////////////////////////////////////////////////////////

	double det = cvInvert(&hessian_mat, invert_hessian_mat);
	return true;
}

bool CorrelateKernels1394_t::Get_Inverse_Displacement_Delta_12_Parameter(
	CvMat* reference_template_value,
	CvMat* reference_template_gradient_x_value, 
	CvMat* reference_template_gradient_y_value, 
	CvMat* deform_template_value, 
	double* reference_template_mean, 
	double* deform_template_mean, 
	double* passd_a, 
	CvMat* invert_hessian_mat, 
	CvMat* dispalcement_delta_mat 
	)
{
	//判断
	//大小
	int template_width =reference_template_value->cols;
	int template_height = reference_template_value->rows;
	if (template_width%2==0 || template_height%2==0)
	{
		return false;
	}
	int template_half_width = (template_width-1)/2;
	int template_half_height = (template_height-1)/2;

	if (reference_template_gradient_x_value->cols!=template_width ||
		reference_template_gradient_x_value->rows!=template_height||
		reference_template_gradient_y_value->cols!=template_width ||
		reference_template_gradient_y_value->rows!=template_height||
		deform_template_value->cols!=template_width ||
		deform_template_value->rows!=template_height
		)
	{
		return false;
	}
	if (/*dispalcement_input_mat->cols!=1||
		dispalcement_input_mat->rows!=6||*/
		dispalcement_delta_mat->cols!=1||
		dispalcement_delta_mat->rows!=12)
	{
		return false;
	}
	if (invert_hessian_mat->cols!=12||
		invert_hessian_mat->rows!=12)
	{
		return false;
	}


	//类型

	if ( CV_MAT_TYPE(reference_template_value->type) != CV_64F)
	{
		return false;
	} 
	if (
		CV_MAT_TYPE(reference_template_gradient_x_value->type) != CV_64F||
		CV_MAT_TYPE(reference_template_gradient_y_value->type) != CV_64F||
		CV_MAT_TYPE(deform_template_value->type) != CV_64F||
		//CV_MAT_TYPE(dispalcement_input_mat->type) != CV_64F||
		CV_MAT_TYPE(dispalcement_delta_mat->type) != CV_64F||
		CV_MAT_TYPE(invert_hessian_mat->type) != CV_64F
		)
	{
		return false;
	}
	if (reference_template_mean==NULL ||
		deform_template_mean==NULL||
		passd_a == NULL)
	{
		return false;
	}
	double* g_ptr = deform_template_value->data.db;


	double f_mean = *reference_template_mean;
	double g_mean = *deform_template_mean;
	double passd_a_value = *passd_a;



	double patial_c_p_ptr[12] = {0};
	CvMat patial_c_p_mat = cvMat(12,1,CV_64F,patial_c_p_ptr);
	for (int m = -template_half_height;m<=template_half_width;m++)
	{
		for (int n= -template_half_width;n<=template_half_width;n++)
		{
			//使用GetSubRect取得的子矩阵，用指针寻址会出问题
			//double f =  f_ptr_double[(m+template_half_height)*template_width+n+template_half_width];
			//double fx = fx_ptr[(m+template_half_height)*template_width+n+template_half_width];
			//double fy = fy_ptr[(m+template_half_height)*template_width+n+template_half_width];
			//double g =  g_ptr[(m+template_half_height)*template_width+n+template_half_width];

			double f = CV_MAT_ELEM(*reference_template_value,double,m+template_half_height,n+template_half_width);
			double fx = CV_MAT_ELEM(*reference_template_gradient_x_value,double,m+template_half_height,n+template_half_width);
			double fy = CV_MAT_ELEM(*reference_template_gradient_y_value,double,m+template_half_height,n+template_half_width);
			double g = CV_MAT_ELEM(*deform_template_value,double,m+template_half_height,n+template_half_width);
			patial_c_p_ptr[0] -= fx * (f - passd_a_value * g + passd_a_value*g_mean-f_mean);
			patial_c_p_ptr[1] -= fx * n * (f - passd_a_value * g + passd_a_value*g_mean-f_mean);
			patial_c_p_ptr[2] -= fx * m * (f - passd_a_value * g + passd_a_value*g_mean-f_mean);
			patial_c_p_ptr[3] -= fx * 0.5 * n * n * (f - passd_a_value * g + passd_a_value*g_mean-f_mean);
			patial_c_p_ptr[4] -= fx * 0.5 * m * m * (f - passd_a_value * g + passd_a_value*g_mean-f_mean);
			patial_c_p_ptr[5] -= fx * n * m * (f - passd_a_value * g + passd_a_value*g_mean-f_mean);

			patial_c_p_ptr[6] -= fy * (f - passd_a_value * g + passd_a_value*g_mean-f_mean);
			patial_c_p_ptr[7] -= fy * n * (f - passd_a_value * g + passd_a_value*g_mean-f_mean);
			patial_c_p_ptr[8] -= fy * m * (f - passd_a_value * g + passd_a_value*g_mean-f_mean);
			patial_c_p_ptr[9] -= fy * 0.5 * n * n * (f - passd_a_value * g + passd_a_value*g_mean-f_mean);
			patial_c_p_ptr[10] -= fy * 0.5 * m * m *(f - passd_a_value * g + passd_a_value*g_mean-f_mean);
			patial_c_p_ptr[11] -= fy * n * m * (f - passd_a_value * g + passd_a_value*g_mean-f_mean);
		}
	}




	double* h_ptr = invert_hessian_mat->data.db;
	//double* p = dispalcement_input_mat->data.db;
	double * dp=  dispalcement_delta_mat->data.db;

	cvMatMul(invert_hessian_mat,&patial_c_p_mat,dispalcement_delta_mat);




	
	//dp[0] = -patial_c_p_ptr[0] * h_ptr[0] -patial_c_p_ptr[1] * h_ptr[1] -patial_c_p_ptr[2] * h_ptr[2] -patial_c_p_ptr[3] * h_ptr[3] -patial_c_p_ptr[4] * h_ptr[4] -patial_c_p_ptr[5] * h_ptr[5];
	//dp[1] = -patial_c_p_ptr[0] * h_ptr[6] -patial_c_p_ptr[1] * h_ptr[7] -patial_c_p_ptr[2] * h_ptr[8] -patial_c_p_ptr[3] * h_ptr[9] -patial_c_p_ptr[4] * h_ptr[10] -patial_c_p_ptr[5] * h_ptr[11];
	//dp[2] = -patial_c_p_ptr[0] * h_ptr[12] -patial_c_p_ptr[1] * h_ptr[13] -patial_c_p_ptr[2] * h_ptr[14] -patial_c_p_ptr[3] * h_ptr[15] -patial_c_p_ptr[4] * h_ptr[16] -patial_c_p_ptr[5] * h_ptr[17];
	//dp[3] = -patial_c_p_ptr[0] * h_ptr[18] -patial_c_p_ptr[1] * h_ptr[19] -patial_c_p_ptr[2] * h_ptr[20] -patial_c_p_ptr[3] * h_ptr[21] -patial_c_p_ptr[4] * h_ptr[22] -patial_c_p_ptr[5] * h_ptr[23];
	//dp[4] = -patial_c_p_ptr[0] * h_ptr[24] -patial_c_p_ptr[1] * h_ptr[25] -patial_c_p_ptr[2] * h_ptr[26] -patial_c_p_ptr[3] * h_ptr[27] -patial_c_p_ptr[4] * h_ptr[28] -patial_c_p_ptr[5] * h_ptr[29];
	//dp[5] = -patial_c_p_ptr[0] * h_ptr[30] -patial_c_p_ptr[1] * h_ptr[31] -patial_c_p_ptr[2] * h_ptr[32] -patial_c_p_ptr[3] * h_ptr[33] -patial_c_p_ptr[4] * h_ptr[34] -patial_c_p_ptr[5] * h_ptr[35];


	return true;
}

bool CorrelateKernels1394_t::Get_Inverse_Hessian_Mat_20_Parameter( 
	CvMat* reference_fx_value_mat, 
	CvMat* reference_fy_value_mat, 
	CvMat* invert_hessian_mat 
	)
{
	//判断大小
	int template_width =reference_fx_value_mat->cols;
	int template_height = reference_fx_value_mat->rows;
	if (template_width%2==0 || template_height%2==0)
	{
		return false;
	}
	int template_half_width = (template_width-1)/2;
	int template_half_height = (template_height-1)/2;
	int template_mat_count = template_height*template_width;

	if (reference_fy_value_mat->cols!=template_width ||
		reference_fy_value_mat->rows!=template_height)
	{
		return false;
	}
	if (invert_hessian_mat->cols!=20||
		invert_hessian_mat->rows!=20)
	{
		return false;
	}


	//类型
	if ( CV_MAT_TYPE(reference_fx_value_mat->type) != CV_64F||
		CV_MAT_TYPE(reference_fy_value_mat->type) != CV_64F||
		CV_MAT_TYPE(invert_hessian_mat->type) != CV_64F)
	{
		return false;
	}

	//double* fx_ptr= reference_fx_value_mat->data.db;
	//double* fy_ptr = reference_fy_value_mat->data.db;

	//double hessian_data[36]={0};
	//for (int m = -template_half_height;m<=template_half_width;m++)
	//{
	//	for (int n= -template_half_width;n<=template_half_width;n++)
	//	{
	//		//使用GetSubRect取得的子矩阵，用指针寻址会出问题
	//		//double fx = fx_ptr[(m+template_half_height)*template_width+n+template_half_width];
	//		//double fy = fy_ptr[(m+template_half_height)*template_width+n+template_half_width];


	//		double fx = CV_MAT_ELEM(*reference_fx_value_mat,double,m+template_half_height,n+template_half_width);
	//		double fy = CV_MAT_ELEM(*reference_fy_value_mat,double,m+template_half_height,n+template_half_width);


	//		hessian_data[0] += fx * fx;
	//		hessian_data[1] += fx * fx * (double)n ;
	//		hessian_data[2] += fx * fx * (double)m;
	//		hessian_data[3] += fx * fy ;
	//		hessian_data[4] += fx * fy * (double)n;
	//		hessian_data[5] += fx * fy * (double)m;

	//		hessian_data[6] += fx * (double)n * fx;
	//		hessian_data[7] += fx * (double)n * fx * (double)n;
	//		hessian_data[8] += fx * (double)n * fx * (double)m;
	//		hessian_data[9] += fx * (double)n * fy ;
	//		hessian_data[10] += fx * (double)n * fy *(double)n;
	//		hessian_data[11] += fx * (double)n *fy * (double)m;

	//		hessian_data[12] += fx * (double)m * fx;
	//		hessian_data[13] += fx * (double)m * fx * (double)n;
	//		hessian_data[14] += fx * (double)m * fx * (double)m;
	//		hessian_data[15] += fx * (double)m * fy ;
	//		hessian_data[16] += fx * (double)m * fy *(double)n;
	//		hessian_data[17] += fx * (double)m * fy *(double)m;


	//		hessian_data[18] += fy * fx;
	//		hessian_data[19] += fy * fx *(double)n;
	//		hessian_data[20] += fy * fx *(double)m;
	//		hessian_data[21] += fy * fy;
	//		hessian_data[22] += fy * fy *(double)n;
	//		hessian_data[23] += fy * fy *(double)m;

	//		hessian_data[24] += fy* (double)n * fx;
	//		hessian_data[25] += fy* (double)n * fx * (double)n;
	//		hessian_data[26] += fy* (double)n * fx * (double)m;
	//		hessian_data[27] += fy* (double)n * fy;
	//		hessian_data[28] += fy* (double)n * fy * (double)n;
	//		hessian_data[29] += fy* (double)n * fy * (double)m;

	//		hessian_data[30] += fy * (double)m * fx;
	//		hessian_data[31] += fy * (double)m * fx * (double)n;
	//		hessian_data[32] += fy * (double)m * fx * (double)m;
	//		hessian_data[33] += fy * (double)m * fy ;
	//		hessian_data[34] += fy * (double)m * fy * (double)n;
	//		hessian_data[35] += fy * (double)m * fy * (double)m;
	//	}
	//}

	//test
	double hessian_data[400];
	CvMat hessian_mat = cvMat(20,20,CV_64F,hessian_data);
	cvZero(&hessian_mat);



	//////////////////////////////////////////////////////////////////////////
	//method 1
	//CvMat* vector_1 = cvCreateMat(12,1,CV_64F);
	//CvMat* vector_1_trans = cvCreateMat(1,12,CV_64F);
	//CvMat* vector_mul = cvCreateMat(12,12,CV_64F);
	//CvMat* cache =cvCreateMat(12,12,CV_64F);
	//for (int m = -template_half_height;m<=template_half_width;m++)
	//{
	//	for (int n= -template_half_width;n<=template_half_width;n++)
	//	{
	//		double fx = CV_MAT_ELEM(*reference_fx_value_mat,double,m+template_half_height,n+template_half_width);
	//		double fy = CV_MAT_ELEM(*reference_fy_value_mat,double,m+template_half_height,n+template_half_width);


	//		CV_MAT_ELEM(*vector_1,double,0,0) = fx;
	//		CV_MAT_ELEM(*vector_1,double,1,0) = fx *(double)n;
	//		CV_MAT_ELEM(*vector_1,double,2,0) = fx *(double)m;
	//		CV_MAT_ELEM(*vector_1,double,3,0) = fx * 0.5 *(double)n*(double)n;
	//		CV_MAT_ELEM(*vector_1,double,4,0) = fx * 0.5 *(double)m*(double)m;
	//		CV_MAT_ELEM(*vector_1,double,5,0) = fx *(double)n*(double)m;

	//		CV_MAT_ELEM(*vector_1,double,6,0) = fy;
	//		CV_MAT_ELEM(*vector_1,double,7,0) = fy *(double)n;
	//		CV_MAT_ELEM(*vector_1,double,8,0) = fy *(double)m;
	//		CV_MAT_ELEM(*vector_1,double,9,0) = fy * 0.5 *(double)n*(double)n;
	//		CV_MAT_ELEM(*vector_1,double,10,0) = fy * 0.5 *(double)m*(double)m;
	//		CV_MAT_ELEM(*vector_1,double,11,0) = fy *(double)n*(double)m;

	//		cvTranspose(vector_1,vector_1_trans);
	//		cvMatMul(vector_1,vector_1_trans,vector_mul);

	//		cvCopy(&hessian_mat,cache);
	//		cvAdd(vector_mul,cache,&hessian_mat);




	//	}
	//}

	//cvReleaseMat(&vector_1);
	//cvReleaseMat(&vector_1_trans);
	//cvReleaseMat(&vector_mul);
	//cvReleaseMat(&cache);
	//////////////////////////////////////////////////////////////////////////


	//////////////////////////////////////////////////////////////////////////
	//method 2
	CvMat* jacobi_mat = cvCreateMat(template_mat_count,20,CV_64F);
	CvMat* jacobi_T_mat = cvCreateMat(20,template_mat_count,CV_64F);
	for (int m = -template_half_height;m<=template_half_width;m++)
	{
		for (int n= -template_half_width;n<=template_half_width;n++)
		{
			double fx = CV_MAT_ELEM(*reference_fx_value_mat,double,m+template_half_height,n+template_half_width);
			double fy = CV_MAT_ELEM(*reference_fy_value_mat,double,m+template_half_height,n+template_half_width);

			int index = (m+template_half_height) *template_width + n+template_half_width;

			CV_MAT_ELEM(*jacobi_mat,double,index,0) = fx;
			CV_MAT_ELEM(*jacobi_mat,double,index,1) = fx *n;
			CV_MAT_ELEM(*jacobi_mat,double,index,2) = fx *m;
			CV_MAT_ELEM(*jacobi_mat,double,index,3) = fx * 0.5 *n*n;
			CV_MAT_ELEM(*jacobi_mat,double,index,4) = fx * 0.5 *m*m;
			CV_MAT_ELEM(*jacobi_mat,double,index,5) = fx *n*m;
			CV_MAT_ELEM(*jacobi_mat,double,index,6) = fx * 1.0 / 6.0 * n * n * n ;
			CV_MAT_ELEM(*jacobi_mat,double,index,7) = fx * 0.5 * n * n * m;
			CV_MAT_ELEM(*jacobi_mat,double,index,8) = fx * 0.5 * n * m * m;
			CV_MAT_ELEM(*jacobi_mat,double,index,9) = fx * 1.0 / 6.0 * m * m * m;

			CV_MAT_ELEM(*jacobi_mat,double,index,10) = fy;
			CV_MAT_ELEM(*jacobi_mat,double,index,11) = fy *n;
			CV_MAT_ELEM(*jacobi_mat,double,index,12) = fy *m;
			CV_MAT_ELEM(*jacobi_mat,double,index,13) = fy * 0.5 *n*n;
			CV_MAT_ELEM(*jacobi_mat,double,index,14) = fy * 0.5 *m*m;
			CV_MAT_ELEM(*jacobi_mat,double,index,15) = fy *n*m;
			CV_MAT_ELEM(*jacobi_mat,double,index,16) = fy * 1.0 / 6.0 * n * n * n ;
			CV_MAT_ELEM(*jacobi_mat,double,index,17) = fy * 0.5 * n * n * m;
			CV_MAT_ELEM(*jacobi_mat,double,index,18) = fy * 0.5 * n * m * m;
			CV_MAT_ELEM(*jacobi_mat,double,index,19) = fy * 1.0 / 6.0 * m * m * m;

			CV_MAT_ELEM(*jacobi_T_mat,double,0,index) = fx;
			CV_MAT_ELEM(*jacobi_T_mat,double,1,index) = fx *n;
			CV_MAT_ELEM(*jacobi_T_mat,double,2,index) = fx *m;
			CV_MAT_ELEM(*jacobi_T_mat,double,3,index) = fx * 0.5 *n*n;
			CV_MAT_ELEM(*jacobi_T_mat,double,4,index) = fx * 0.5 *m*m;
			CV_MAT_ELEM(*jacobi_T_mat,double,5,index) = fx *n*m;
			CV_MAT_ELEM(*jacobi_T_mat,double,6,index) = fx * 1.0 / 6.0 * n * n * n ;
			CV_MAT_ELEM(*jacobi_T_mat,double,7,index) = fx * 0.5 * n * n * m;
			CV_MAT_ELEM(*jacobi_T_mat,double,8,index) = fx * 0.5 * n * m * m;
			CV_MAT_ELEM(*jacobi_T_mat,double,9,index) = fx * 1.0 / 6.0 * m * m * m;

			CV_MAT_ELEM(*jacobi_T_mat,double,10,index) = fy;
			CV_MAT_ELEM(*jacobi_T_mat,double,11,index) = fy *n;
			CV_MAT_ELEM(*jacobi_T_mat,double,12,index) = fy *m;
			CV_MAT_ELEM(*jacobi_T_mat,double,13,index) = fy * 0.5 *n*n;
			CV_MAT_ELEM(*jacobi_T_mat,double,14,index) = fy * 0.5 *m*m;
			CV_MAT_ELEM(*jacobi_T_mat,double,15,index) = fy *n*m;
			CV_MAT_ELEM(*jacobi_T_mat,double,16,index) = fy * 1.0 / 6.0 * n * n * n ;
			CV_MAT_ELEM(*jacobi_T_mat,double,17,index) = fy * 0.5 * n * n * m;
			CV_MAT_ELEM(*jacobi_T_mat,double,18,index) = fy * 0.5 * n * m * m;
			CV_MAT_ELEM(*jacobi_T_mat,double,19,index) = fy * 1.0 / 6.0 * m * m * m;
		}
	}
	cvMatMul(jacobi_T_mat,jacobi_mat,&hessian_mat);

	cvReleaseMat(&jacobi_mat);
	cvReleaseMat(&jacobi_T_mat);
	
	//CvMat hessian_mat = cvMat(12,12,CV_64F,hessian_data);
	double det = cvInvert(&hessian_mat,invert_hessian_mat);
	return true;
}

bool CorrelateKernels1394_t::Get_Inverse_Displacement_Delta_20_Parameter(
	CvMat* reference_template_value,
	CvMat* reference_template_gradient_x_value, 
	CvMat* reference_template_gradient_y_value, 
	CvMat* deform_template_value, 
	double* reference_template_mean, 
	double* deform_template_mean, 
	double* passd_a, 
	CvMat* invert_hessian_mat, 
	CvMat* dispalcement_delta_mat 
	)
{
	//判断大小
	int template_width =reference_template_value->cols;
	int template_height = reference_template_value->rows;
	if (template_width%2==0 || template_height%2==0)
	{
		return false;
	}
	int template_half_width = (template_width-1)/2;
	int template_half_height = (template_height-1)/2;

	if (reference_template_gradient_x_value->cols!=template_width ||
		reference_template_gradient_x_value->rows!=template_height||
		reference_template_gradient_y_value->cols!=template_width ||
		reference_template_gradient_y_value->rows!=template_height||
		deform_template_value->cols!=template_width ||
		deform_template_value->rows!=template_height
		)
	{
		return false;
	}
	if (
		dispalcement_delta_mat->cols!=1||
		dispalcement_delta_mat->rows!=20)
	{
		return false;
	}
	if (invert_hessian_mat->cols!=20||
		invert_hessian_mat->rows!=20)
	{
		return false;
	}


	//类型
	if ( CV_MAT_TYPE(reference_template_value->type) != CV_64F)
	{
		return false;
	} 
	if (
		CV_MAT_TYPE(reference_template_gradient_x_value->type) != CV_64F||
		CV_MAT_TYPE(reference_template_gradient_y_value->type) != CV_64F||
		CV_MAT_TYPE(deform_template_value->type) != CV_64F||
		//CV_MAT_TYPE(dispalcement_input_mat->type) != CV_64F||
		CV_MAT_TYPE(dispalcement_delta_mat->type) != CV_64F||
		CV_MAT_TYPE(invert_hessian_mat->type) != CV_64F
		)
	{
		return false;
	}
	if (reference_template_mean==NULL ||
		deform_template_mean==NULL||
		passd_a == NULL)
	{
		return false;
	}

	double* g_ptr = deform_template_value->data.db;
	double f_mean = *reference_template_mean;
	double g_mean = *deform_template_mean;
	double passd_a_value = *passd_a;

	double patial_c_p_ptr[20] = {0};
	CvMat patial_c_p_mat = cvMat(20,1,CV_64F,patial_c_p_ptr);
	for (int m = -template_half_height;m<=template_half_width;m++)
	{
		for (int n= -template_half_width;n<=template_half_width;n++)
		{
			double f = CV_MAT_ELEM(*reference_template_value,double,m+template_half_height,n+template_half_width);
			double fx = CV_MAT_ELEM(*reference_template_gradient_x_value,double,m+template_half_height,n+template_half_width);
			double fy = CV_MAT_ELEM(*reference_template_gradient_y_value,double,m+template_half_height,n+template_half_width);
			double g = CV_MAT_ELEM(*deform_template_value,double,m+template_half_height,n+template_half_width);

			patial_c_p_ptr[0] -= fx * (f - passd_a_value * g + passd_a_value*g_mean-f_mean);
			patial_c_p_ptr[1] -= fx * n * (f - passd_a_value * g + passd_a_value*g_mean-f_mean);
			patial_c_p_ptr[2] -= fx * m * (f - passd_a_value * g + passd_a_value*g_mean-f_mean);
			patial_c_p_ptr[3] -= fx * 0.5 * n * n * (f - passd_a_value * g + passd_a_value*g_mean-f_mean);
			patial_c_p_ptr[4] -= fx * 0.5 * m * m * (f - passd_a_value * g + passd_a_value*g_mean-f_mean);
			patial_c_p_ptr[5] -= fx * n * m * (f - passd_a_value * g + passd_a_value*g_mean-f_mean);
			patial_c_p_ptr[6] -= fx * 1.0 / 6.0 * n * n * n * (f - passd_a_value * g + passd_a_value*g_mean-f_mean);
			patial_c_p_ptr[7] -= fx * 0.5 * n * n * m * (f - passd_a_value * g + passd_a_value*g_mean-f_mean);
			patial_c_p_ptr[8] -= fx * 0.5 * n * m * m * (f - passd_a_value * g + passd_a_value*g_mean-f_mean);
			patial_c_p_ptr[9] -= fx * 1.0 / 6.0 * m * m * m * (f - passd_a_value * g + passd_a_value*g_mean-f_mean);

			patial_c_p_ptr[10] -= fy * (f - passd_a_value * g + passd_a_value*g_mean-f_mean);
			patial_c_p_ptr[11] -= fy * n * (f - passd_a_value * g + passd_a_value*g_mean-f_mean);
			patial_c_p_ptr[12] -= fy * m * (f - passd_a_value * g + passd_a_value*g_mean-f_mean);
			patial_c_p_ptr[13] -= fy * 0.5 * n * n * (f - passd_a_value * g + passd_a_value*g_mean-f_mean);
			patial_c_p_ptr[14] -= fy * 0.5 * m * m *(f - passd_a_value * g + passd_a_value*g_mean-f_mean);
			patial_c_p_ptr[15] -= fy * n * m * (f - passd_a_value * g + passd_a_value*g_mean-f_mean);
			patial_c_p_ptr[16] -= fy * 1.0 / 6.0 * n * n * n * (f - passd_a_value * g + passd_a_value*g_mean-f_mean);
			patial_c_p_ptr[17] -= fy * 0.5 * n * n * m * (f - passd_a_value * g + passd_a_value*g_mean-f_mean);
			patial_c_p_ptr[18] -= fy * 0.5 * n * m * m * (f - passd_a_value * g + passd_a_value*g_mean-f_mean);
			patial_c_p_ptr[19] -= fy * 1.0 / 6.0 * m * m * m * (f - passd_a_value * g + passd_a_value*g_mean-f_mean);
		}
	}
	double* h_ptr = invert_hessian_mat->data.db;
	//double* p = dispalcement_input_mat->data.db;
	double * dp=  dispalcement_delta_mat->data.db;
	cvMatMul(invert_hessian_mat,&patial_c_p_mat,dispalcement_delta_mat);


	//dp[0] = -patial_c_p_ptr[0] * h_ptr[0] -patial_c_p_ptr[1] * h_ptr[1] -patial_c_p_ptr[2] * h_ptr[2] -patial_c_p_ptr[3] * h_ptr[3] -patial_c_p_ptr[4] * h_ptr[4] -patial_c_p_ptr[5] * h_ptr[5];
	//dp[1] = -patial_c_p_ptr[0] * h_ptr[6] -patial_c_p_ptr[1] * h_ptr[7] -patial_c_p_ptr[2] * h_ptr[8] -patial_c_p_ptr[3] * h_ptr[9] -patial_c_p_ptr[4] * h_ptr[10] -patial_c_p_ptr[5] * h_ptr[11];
	//dp[2] = -patial_c_p_ptr[0] * h_ptr[12] -patial_c_p_ptr[1] * h_ptr[13] -patial_c_p_ptr[2] * h_ptr[14] -patial_c_p_ptr[3] * h_ptr[15] -patial_c_p_ptr[4] * h_ptr[16] -patial_c_p_ptr[5] * h_ptr[17];
	//dp[3] = -patial_c_p_ptr[0] * h_ptr[18] -patial_c_p_ptr[1] * h_ptr[19] -patial_c_p_ptr[2] * h_ptr[20] -patial_c_p_ptr[3] * h_ptr[21] -patial_c_p_ptr[4] * h_ptr[22] -patial_c_p_ptr[5] * h_ptr[23];
	//dp[4] = -patial_c_p_ptr[0] * h_ptr[24] -patial_c_p_ptr[1] * h_ptr[25] -patial_c_p_ptr[2] * h_ptr[26] -patial_c_p_ptr[3] * h_ptr[27] -patial_c_p_ptr[4] * h_ptr[28] -patial_c_p_ptr[5] * h_ptr[29];
	//dp[5] = -patial_c_p_ptr[0] * h_ptr[30] -patial_c_p_ptr[1] * h_ptr[31] -patial_c_p_ptr[2] * h_ptr[32] -patial_c_p_ptr[3] * h_ptr[33] -patial_c_p_ptr[4] * h_ptr[34] -patial_c_p_ptr[5] * h_ptr[35];


	return true;
}

bool CorrelateKernels1394_t::Get_Inverse_Displacement_Result_12_Parameter(
	CvMat* dispalcement_input_mat, 
	CvMat* dispalcement_delta_mat, 
	CvMat* displacement_output_mat 
	)
{
	if (dispalcement_input_mat->cols!=1||
		dispalcement_input_mat->rows!=12||
		dispalcement_delta_mat->cols!=1||
		dispalcement_delta_mat->rows!=12||
		displacement_output_mat->cols!=1||
		displacement_output_mat->rows!=12
		)
	{
		return false;
	}

	if (
		CV_MAT_TYPE(dispalcement_input_mat->type) != CV_64F||
		CV_MAT_TYPE(dispalcement_delta_mat->type) != CV_64F||
		CV_MAT_TYPE(displacement_output_mat->type) != CV_64F
		)
	{
		return false;
	}

	//double* p = dispalcement_input_mat->data.db;
	//double * dp = dispalcement_delta_mat->data.db;
	//double* p_new = displacement_output_mat->data.db;

	CvMat* origin_W = cvCreateMat(6,6,CV_64F);
	CvMat* delta_origin_W = cvCreateMat(6,6,CV_64F);

	double U = CV_MAT_ELEM(*dispalcement_input_mat,double,0,0);
	double U_x = CV_MAT_ELEM(*dispalcement_input_mat,double,1,0);
	double U_y = CV_MAT_ELEM(*dispalcement_input_mat,double,2,0);
	double U_xx = CV_MAT_ELEM(*dispalcement_input_mat,double,3,0);	
	double U_yy = CV_MAT_ELEM(*dispalcement_input_mat,double,4,0);
	double U_xy = CV_MAT_ELEM(*dispalcement_input_mat,double,5,0);
	double V = CV_MAT_ELEM(*dispalcement_input_mat,double,6,0);
	double V_x = CV_MAT_ELEM(*dispalcement_input_mat,double,7,0);
	double V_y = CV_MAT_ELEM(*dispalcement_input_mat,double,8,0);
	double V_xx = CV_MAT_ELEM(*dispalcement_input_mat,double,9,0);
	double V_yy = CV_MAT_ELEM(*dispalcement_input_mat,double,10,0);
	double V_xy = CV_MAT_ELEM(*dispalcement_input_mat,double,11,0);
	

	CV_MAT_ELEM(*origin_W,double,0,0) = (1+U_x)*(1+U_x) + U*U_xx;
	CV_MAT_ELEM(*origin_W,double,0,1) = 2*U*U_xy+2*(1+U_x)*U_y;
	CV_MAT_ELEM(*origin_W,double,0,2) = (U_y*U_y)+U*U_yy;
	CV_MAT_ELEM(*origin_W,double,0,3) = 2*U*(1+U_x);
	CV_MAT_ELEM(*origin_W,double,0,4) = 2*U*U_y;
	CV_MAT_ELEM(*origin_W,double,0,5) = U*U;

	CV_MAT_ELEM(*origin_W,double,1,0) = 0.5*U*V_xx+V_x*(1+U_x)+0.5*V*U_xx;
	CV_MAT_ELEM(*origin_W,double,1,1) = U*V_xy+(1+U_x)*(1+V_y)+V_x*U_y+U_xy*V;
	CV_MAT_ELEM(*origin_W,double,1,2) = 0.5*U*V_yy+U_y*(1+V_y)+0.5*U_yy*V;
	CV_MAT_ELEM(*origin_W,double,1,3) = U*V_x + V*(1+U_x);
	CV_MAT_ELEM(*origin_W,double,1,4) = U*(1+V_y)+V*U_y;
	CV_MAT_ELEM(*origin_W,double,1,5) = U*V;

	CV_MAT_ELEM(*origin_W,double,2,0) = V_x*V_x + V_xx*V;
	CV_MAT_ELEM(*origin_W,double,2,1) = 2*V*V_xy+2*V_x*(1+V_y);
	CV_MAT_ELEM(*origin_W,double,2,2) = (1+V_y)*(1+V_y)+V*V_yy;
	CV_MAT_ELEM(*origin_W,double,2,3) = 2*V*V_x;
	CV_MAT_ELEM(*origin_W,double,2,4) = 2*V*(1+V_y);
	CV_MAT_ELEM(*origin_W,double,2,5) = V*V;

	CV_MAT_ELEM(*origin_W,double,3,0) = 0.5*U_xx;
	CV_MAT_ELEM(*origin_W,double,3,1) = U_xy;
	CV_MAT_ELEM(*origin_W,double,3,2) = 0.5*U_yy;
	CV_MAT_ELEM(*origin_W,double,3,3) = 1+U_x;
	CV_MAT_ELEM(*origin_W,double,3,4) = U_y;
	CV_MAT_ELEM(*origin_W,double,3,5) = U;

	CV_MAT_ELEM(*origin_W,double,4,0) = 0.5*V_xx;
	CV_MAT_ELEM(*origin_W,double,4,1) = V_xy;
	CV_MAT_ELEM(*origin_W,double,4,2) = 0.5*V_yy;
	CV_MAT_ELEM(*origin_W,double,4,3) = V_x;
	CV_MAT_ELEM(*origin_W,double,4,4) = V_y+1;
	CV_MAT_ELEM(*origin_W,double,4,5) = V;

	CV_MAT_ELEM(*origin_W,double,5,0) = 0;
	CV_MAT_ELEM(*origin_W,double,5,1) = 0;
	CV_MAT_ELEM(*origin_W,double,5,2) = 0;
	CV_MAT_ELEM(*origin_W,double,5,3) = 0;
	CV_MAT_ELEM(*origin_W,double,5,4) = 0;
	CV_MAT_ELEM(*origin_W,double,5,5) = 1.0;

	U = CV_MAT_ELEM(*dispalcement_delta_mat,double,0,0);
	U_x = CV_MAT_ELEM(*dispalcement_delta_mat,double,1,0);
	U_y = CV_MAT_ELEM(*dispalcement_delta_mat,double,2,0);
	U_xx = CV_MAT_ELEM(*dispalcement_delta_mat,double,3,0);	
	U_yy = CV_MAT_ELEM(*dispalcement_delta_mat,double,4,0);
	U_xy = CV_MAT_ELEM(*dispalcement_delta_mat,double,5,0);
	V = CV_MAT_ELEM(*dispalcement_delta_mat,double,6,0);
	V_x = CV_MAT_ELEM(*dispalcement_delta_mat,double,7,0);
	V_y = CV_MAT_ELEM(*dispalcement_delta_mat,double,8,0);
	V_xx = CV_MAT_ELEM(*dispalcement_delta_mat,double,9,0);
	V_yy = CV_MAT_ELEM(*dispalcement_delta_mat,double,10,0);
	V_xy = CV_MAT_ELEM(*dispalcement_delta_mat,double,11,0);
	

	CV_MAT_ELEM(*delta_origin_W,double,0,0) = (1+U_x)*(1+U_x) + U*U_xx;
	CV_MAT_ELEM(*delta_origin_W,double,0,1) = 2*U*U_xy+2*(1+U_x)*U_y;
	CV_MAT_ELEM(*delta_origin_W,double,0,2) = (U_y*U_y)+U*U_yy;
	CV_MAT_ELEM(*delta_origin_W,double,0,3) = 2*U*(1+U_x);
	CV_MAT_ELEM(*delta_origin_W,double,0,4) = 2*U*U_y;
	CV_MAT_ELEM(*delta_origin_W,double,0,5) = U*U;

	CV_MAT_ELEM(*delta_origin_W,double,1,0) = 0.5*U*V_xx+V_x*(1+U_x)+0.5*V*U_xx;
	CV_MAT_ELEM(*delta_origin_W,double,1,1) = U*V_xy+(1+U_x)*(1+V_y)+V_x*U_y+U_xy*V;
	CV_MAT_ELEM(*delta_origin_W,double,1,2) = 0.5*U*V_yy+U_y*(1+V_y)+0.5*U_yy*V;
	CV_MAT_ELEM(*delta_origin_W,double,1,3) = U*V_x + V*(1+U_x);
	CV_MAT_ELEM(*delta_origin_W,double,1,4) = U*(1+V_y)+V*U_y;
	CV_MAT_ELEM(*delta_origin_W,double,1,5) = U*V;

	CV_MAT_ELEM(*delta_origin_W,double,2,0) = V_x*V_x + V_xx*V;
	CV_MAT_ELEM(*delta_origin_W,double,2,1) = 2*V*V_xy+2*V_x*(1+V_y);
	CV_MAT_ELEM(*delta_origin_W,double,2,2) = (1+V_y)*(1+V_y)+V*V_yy;
	CV_MAT_ELEM(*delta_origin_W,double,2,3) = 2*V*V_x;
	CV_MAT_ELEM(*delta_origin_W,double,2,4) = 2*V*(1+V_y);
	CV_MAT_ELEM(*delta_origin_W,double,2,5) = V*V;

	CV_MAT_ELEM(*delta_origin_W,double,3,0) = 0.5*U_xx;
	CV_MAT_ELEM(*delta_origin_W,double,3,1) = U_xy;
	CV_MAT_ELEM(*delta_origin_W,double,3,2) = 0.5*U_yy;
	CV_MAT_ELEM(*delta_origin_W,double,3,3) = 1+U_x;
	CV_MAT_ELEM(*delta_origin_W,double,3,4) = U_y;
	CV_MAT_ELEM(*delta_origin_W,double,3,5) = U;

	CV_MAT_ELEM(*delta_origin_W,double,4,0) = 0.5*V_xx;
	CV_MAT_ELEM(*delta_origin_W,double,4,1) = V_xy;
	CV_MAT_ELEM(*delta_origin_W,double,4,2) = 0.5*V_yy;
	CV_MAT_ELEM(*delta_origin_W,double,4,3) = V_x;
	CV_MAT_ELEM(*delta_origin_W,double,4,4) = V_y+1;
	CV_MAT_ELEM(*delta_origin_W,double,4,5) = V;

	CV_MAT_ELEM(*delta_origin_W,double,5,0) = 0;
	CV_MAT_ELEM(*delta_origin_W,double,5,1) = 0;
	CV_MAT_ELEM(*delta_origin_W,double,5,2) = 0;
	CV_MAT_ELEM(*delta_origin_W,double,5,3) = 0;
	CV_MAT_ELEM(*delta_origin_W,double,5,4) = 0;
	CV_MAT_ELEM(*delta_origin_W,double,5,5) = 1.0;

	CvMat* inverse_delta_origin_W = cvCreateMat(6,6,CV_64F);
	CvMat* new_W = cvCreateMat(6,6,CV_64F);
	double det_delta_origin_W = cvInvert(delta_origin_W,inverse_delta_origin_W);
	cvMatMul(origin_W,inverse_delta_origin_W,new_W);

	CV_MAT_ELEM(*displacement_output_mat,double,0,0) = CV_MAT_ELEM(*new_W,double,3,5);
	CV_MAT_ELEM(*displacement_output_mat,double,1,0) = CV_MAT_ELEM(*new_W,double,3,3)-1.0;
	CV_MAT_ELEM(*displacement_output_mat,double,2,0) = CV_MAT_ELEM(*new_W,double,3,4);
	CV_MAT_ELEM(*displacement_output_mat,double,3,0) = CV_MAT_ELEM(*new_W,double,3,0)*2.0;
	CV_MAT_ELEM(*displacement_output_mat,double,4,0) = CV_MAT_ELEM(*new_W,double,3,2)*2.0;
	CV_MAT_ELEM(*displacement_output_mat,double,5,0) = CV_MAT_ELEM(*new_W,double,3,1);
	
	CV_MAT_ELEM(*displacement_output_mat,double,6,0) = CV_MAT_ELEM(*new_W,double,4,5);
	CV_MAT_ELEM(*displacement_output_mat,double,7,0) = CV_MAT_ELEM(*new_W,double,4,3);
	CV_MAT_ELEM(*displacement_output_mat,double,8,0) = CV_MAT_ELEM(*new_W,double,4,4)-1.0;
	CV_MAT_ELEM(*displacement_output_mat,double,9,0) = CV_MAT_ELEM(*new_W,double,4,0)*2.0;	
	CV_MAT_ELEM(*displacement_output_mat,double,10,0) = CV_MAT_ELEM(*new_W,double,4,2)*2.0;
	CV_MAT_ELEM(*displacement_output_mat,double,11,0) = CV_MAT_ELEM(*new_W,double,4,1);

	cvReleaseMat(&inverse_delta_origin_W);
	cvReleaseMat(&new_W);
	cvReleaseMat(&origin_W);
	cvReleaseMat(&delta_origin_W);

	return true;
}

bool CorrelateKernels1394_t::Reproject3DPoints(
	const CvMat* left_image_point_pos_mat,//1XN CV_64FC2
	const CvMat* right_image_point_pos_mat,//同上
	const CvMat* left_intrinsic_matrix,
	const CvMat* right_intrinsic_matrix,
	const CvMat* left_distortion_coeffs,
	const CvMat* right_distortion_coeffs,
	const CvMat* rotation_matrix,
	const CvMat* translation_vector,
	CvMat* fundamental_matrix,
	CvMat* image_points_3d_X,//64F
	CvMat* image_points_3d_Y,//64F
	CvMat* image_points_3d_Z//64F
	)
{
	//判断
	//大小
	int mat_col = image_points_3d_X->cols;
	int mat_row = image_points_3d_X->rows;
	if (image_points_3d_Y->cols != mat_col ||
		image_points_3d_Y->rows != mat_row ||
		image_points_3d_Z->cols != mat_col ||
		image_points_3d_Z->rows !=mat_row)
	{
		return false;
	}
	const auto point_count = mat_col*mat_row;
	if(
		left_image_point_pos_mat->rows!=1||
		right_image_point_pos_mat->rows!=1||
		left_image_point_pos_mat->cols!=right_image_point_pos_mat->cols||
		left_image_point_pos_mat->cols!=point_count
		)
	{
		return false;
	}
	if (left_intrinsic_matrix->cols!=3||
		left_intrinsic_matrix->rows!=3||
		right_intrinsic_matrix->cols!=3||
		right_intrinsic_matrix->rows!=3)
	{
		return false;
	}
	if (left_distortion_coeffs->cols!=4||
		left_distortion_coeffs->rows!=1||
		right_distortion_coeffs->cols!=4||
		right_distortion_coeffs->rows!=1
		)
	{
		return false;
	}
	if (rotation_matrix->cols!=3||rotation_matrix->rows!=3||
		translation_vector->cols!=1||translation_vector->rows!=3)
	{
		return false;
	}


	//类型
	if (
		CV_MAT_TYPE(left_image_point_pos_mat->type) !=CV_64FC2||
		CV_MAT_TYPE(right_image_point_pos_mat->type) !=CV_64FC2||
		CV_MAT_TYPE(image_points_3d_X->type) !=CV_64F||
		CV_MAT_TYPE(image_points_3d_Y->type) !=CV_64F||
		CV_MAT_TYPE(image_points_3d_Z->type) !=CV_64F
		)
	{
		return false;
	}

	if (
		(CV_MAT_TYPE( left_intrinsic_matrix->type )!=CV_32FC1)||
		(CV_MAT_TYPE(right_intrinsic_matrix->type) !=CV_32FC1) ||
		(CV_MAT_TYPE(right_distortion_coeffs->type) !=CV_32FC1)||
		(CV_MAT_TYPE(left_distortion_coeffs->type) !=CV_32FC1)|| 
		(CV_MAT_TYPE(translation_vector->type) !=CV_64FC1) || 
		(CV_MAT_TYPE( rotation_matrix->type) !=CV_64FC1)
		)		
	{
		return false;
	}

	CvMat* left_image_points_correct= cvCreateMat(1,point_count,CV_64FC2);
	CvMat* right_image_point_correct = cvCreateMat(1,point_count,CV_64FC2);
	if (fundamental_matrix == NULL)
	{
		//cvUndistortPoints(left_image_point_pos_mat, left_image_points_correct, left_intrinsic_matrix, left_distortion_coeffs, 0, left_intrinsic_matrix);
		//cvUndistortPoints(right_image_point_pos_mat, right_image_point_correct, right_intrinsic_matrix, right_distortion_coeffs, 0, right_intrinsic_matrix);
		cv::undistortPoints(::get_mat(left_image_point_pos_mat), ::get_mat(left_image_points_correct), ::get_mat(left_intrinsic_matrix), ::get_mat(left_distortion_coeffs));
		cv::undistortPoints(::get_mat(right_image_point_pos_mat), ::get_mat(right_image_point_correct), ::get_mat(right_intrinsic_matrix), ::get_mat(right_distortion_coeffs));
	} 
	else {
		//CvMat* left_image_points_cache= cvCreateMat(1,point_count,CV_64FC2);
		//CvMat* right_image_point_cache = cvCreateMat(1,point_count,CV_64FC2);
		auto left_image_points_cache = cv::Mat(1, point_count, CV_64FC2);
		auto right_image_point_cache = cv::Mat(1, point_count, CV_64FC2);

		//cvUndistortPoints(left_image_point_pos_mat, left_image_points_cache, left_intrinsic_matrix, left_distortion_coeffs, 0, left_intrinsic_matrix);
		//cvUndistortPoints(right_image_point_pos_mat, right_image_point_cache, right_intrinsic_matrix, right_distortion_coeffs, 0, right_intrinsic_matrix);
		//cvCorrectMatches(fundamental_matrix, left_image_points_cache, right_image_point_cache, left_image_points_correct, right_image_point_correct);
		cv::undistortPoints(::get_mat(left_image_point_pos_mat), left_image_points_cache, ::get_mat(left_intrinsic_matrix), ::get_mat(left_distortion_coeffs));
		cv::undistortPoints(::get_mat(right_image_point_pos_mat), right_image_point_cache, ::get_mat(right_intrinsic_matrix), ::get_mat(right_distortion_coeffs));
		cv::correctMatches(::get_mat(fundamental_matrix), left_image_points_cache, right_image_point_cache, ::get_mat(left_image_points_correct), ::get_mat(right_image_point_correct));

		//cvReleaseMat(&left_image_points_cache);
		//cvReleaseMat(&right_image_point_cache);
	}

	//
	double r0,r1,r2,r3,r4,r5,r6,r7,r8;
	double tx,ty,tz;
	double fxl,fyl,cxl,cyl,fxr,fyr,cxr,cyr;

	r0 = CV_MAT_ELEM(*rotation_matrix,double,0,0);
	r1 = CV_MAT_ELEM(*rotation_matrix,double,0,1);
	r2 = CV_MAT_ELEM(*rotation_matrix,double,0,2);
	r3 = CV_MAT_ELEM(*rotation_matrix,double,1,0);
	r4 = CV_MAT_ELEM(*rotation_matrix,double,1,1);
	r5 = CV_MAT_ELEM(*rotation_matrix,double,1,2);
	r6 = CV_MAT_ELEM(*rotation_matrix,double,2,0);
	r7 = CV_MAT_ELEM(*rotation_matrix,double,2,1);
	r8 = CV_MAT_ELEM(*rotation_matrix,double,2,2);

	tx = CV_MAT_ELEM(*translation_vector,double,0,0);
	ty = CV_MAT_ELEM(*translation_vector,double,1,0);
	tz = CV_MAT_ELEM(*translation_vector,double,2,0);

	fxl = CV_MAT_ELEM(*left_intrinsic_matrix,float,0,0);
	fyl = CV_MAT_ELEM(*left_intrinsic_matrix,float,1,1);
	cxl = CV_MAT_ELEM(*left_intrinsic_matrix,float,0,2);
	cyl = CV_MAT_ELEM(*left_intrinsic_matrix,float,1,2);

	fxr = CV_MAT_ELEM(*right_intrinsic_matrix,float,0,0);
	fyr = CV_MAT_ELEM(*right_intrinsic_matrix,float,1,1);
	cxr = CV_MAT_ELEM(*right_intrinsic_matrix,float,0,2);
	cyr = CV_MAT_ELEM(*right_intrinsic_matrix,float,1,2);


	//CvMat* left_image_pointx = cvCreateMat(1,point_count,CV_32FC1);
	//CvMat* left_image_pointy = cvCreateMat(1,point_count,CV_32FC1);
	//CvMat* right_image_pointx = cvCreateMat(1,point_count,CV_32FC1);
	//CvMat* right_imgae_pointy = cvCreateMat(1,point_count,CV_32FC1);
	//cvSplit(left_image_points_undistort,left_image_pointx,left_image_pointy,NULL,NULL);
	//cvSplit(right_image_point_undistort,right_image_pointx,right_imgae_pointy,NULL,NULL);

	double* left_image_points_undistort_ptr = left_image_points_correct->data.db;
	double* right_image_point_undistort_ptr =right_image_point_correct->data.db;


	/*	CvMat* X_equation2output = cvCreateMat(pointsCountTotal,3,CV_32FC1);*/

	//#pragma omp parallel for
	for (int i =0;i<mat_row;i++)
	{
		for (int j =0;j<mat_col;j++)
		{
			CvMat* A_equation = cvCreateMat(4,3,CV_64F);
			CvMat* B_equation = cvCreateMat(4,1,CV_64F);
			CvMat* X_equation = cvCreateMat(3,1,CV_64F);

			CV_MAT_ELEM(*A_equation,double,0,0) = -fxl;
			CV_MAT_ELEM(*A_equation,double,0,1) = 0.0f;
			CV_MAT_ELEM(*A_equation,double,0,2) = left_image_points_undistort_ptr[(i*mat_col+j)*2] - cxl;
			CV_MAT_ELEM(*A_equation,double,1,0) = 0.0f;
			CV_MAT_ELEM(*A_equation,double,1,1) = -fyl;
			CV_MAT_ELEM(*A_equation,double,1,2) = left_image_points_undistort_ptr[(i*mat_col+j)*2+1] - cyl;
			CV_MAT_ELEM(*A_equation,double,2,0) = (right_image_point_undistort_ptr[(i*mat_col+j)*2]-cxr)*r6 - fxr*r0;
			CV_MAT_ELEM(*A_equation,double,2,1) = (right_image_point_undistort_ptr[(i*mat_col+j)*2]-cxr)*r7 - fxr*r1;
			CV_MAT_ELEM(*A_equation,double,2,2) = (right_image_point_undistort_ptr[(i*mat_col+j)*2]-cxr)*r8 - fxr*r2;
			CV_MAT_ELEM(*A_equation,double,3,0) = (right_image_point_undistort_ptr[(i*mat_col+j)*2+1]-cyr)*r6 - fyr*r3;
			CV_MAT_ELEM(*A_equation,double,3,1) = (right_image_point_undistort_ptr[(i*mat_col+j)*2+1]-cyr)*r7 - fyr*r4;
			CV_MAT_ELEM(*A_equation,double,3,2) = (right_image_point_undistort_ptr[(i*mat_col+j)*2+1]-cyr)*r8 - fyr*r5;

			CV_MAT_ELEM(*B_equation,double,0,0) = 0.0f;
			CV_MAT_ELEM(*B_equation,double,1,0) = 0.0f;
			CV_MAT_ELEM(*B_equation,double,2,0) = -((right_image_point_undistort_ptr[(i*mat_col+j)*2]-cxr)*tz-fxr*tx);
			CV_MAT_ELEM(*B_equation,double,3,0) = -((right_image_point_undistort_ptr[(i*mat_col+j)*2+1]-cyr)*tz-fyr*ty);

			cvSolve(A_equation,B_equation,X_equation);

			//与CSI同样，Y,Z反向
			CV_MAT_ELEM(*image_points_3d_X,double,i,j) = CV_MAT_ELEM(*X_equation,double,0,0);
			CV_MAT_ELEM(*image_points_3d_Y,double,i,j) = -CV_MAT_ELEM(*X_equation,double,1,0);
			CV_MAT_ELEM(*image_points_3d_Z,double,i,j) = -CV_MAT_ELEM(*X_equation,double,2,0);

			cvReleaseMat(&A_equation);
			cvReleaseMat(&B_equation);
			cvReleaseMat(&X_equation);
		}
	}
	cvReleaseMat(&left_image_points_correct);
	cvReleaseMat(&right_image_point_correct);
	return true;
}

bool CorrelateKernels1394_t::Get_Gauss_Core(
	CvMat* core_mat, CvMat* convolution_core_mat, double sigma)
{
	int core_width = core_mat->width;
	int core_height = core_mat->height;
	if (core_width % 2 == 0 || core_height % 2 == 0)
	{
		return false;
	}
	int core_half_width = (core_width - 1) / 2;
	int core_half_height = (core_height - 1) / 2;

	double total_weight = 0.0;
	for (int m = -core_half_height; m <= core_half_height; m++)
	{
		for (int n = -core_half_width; n <= core_half_width; n++)
		{
			double weight = exp(-(m * m + n * n) / 2.0 / sigma / sigma);
			CV_MAT_ELEM(*core_mat, double, m + core_half_height, n + core_half_width) = weight;
			total_weight += weight;
		}
	}
	total_weight = ((double)core_height * (double)core_width) / total_weight;

	for (int m = -core_half_height; m <= core_half_height; m++)
	{
		for (int n = -core_half_width; n <= core_half_width; n++)
		{
			const auto weight_new = total_weight * CV_MAT_ELEM(*core_mat, double, m + core_half_height, n + core_half_width);
			CV_MAT_ELEM(*core_mat, double, m + core_half_height, n + core_half_width) = weight_new;
			CV_MAT_ELEM(*convolution_core_mat, double, m + core_half_height, n + core_half_width) = weight_new / ((double)core_height * (double)core_width);
		}
	}
	return true;
}

bool CorrelateKernels1394_t::Image_Gauss_Filter(
	CvMat* image_input, CvMat* convolution_core_mat, CvMat* image_output )
{
	int core_width  = convolution_core_mat->width;
	int core_height = convolution_core_mat->height;
	if (core_width %2==0|| core_height%2==0)
	{
		return false;
	}
	int core_half_width = (core_width-1)/2;
	int core_half_height = (core_height-1)/2;

	int image_height = image_input->rows;
	int image_width = image_input->cols;

	for (int i =0 ;i<image_height;i++)
	{
		for (int j =0;j<image_width;j++)
		{			
			double val_out;
			if (i<core_half_height || i>image_height-1-core_half_height || 
				j<core_half_width || j>image_width -1 -core_half_width)
			{
				val_out = CV_MAT_ELEM(*image_input,double,i,j);
			} 
			else
			{
				val_out = 0.0;
				for (int m =-core_half_height;m<=core_half_height;m++)
				{
					for (int n =-core_half_width;n<=core_half_width;n++)
					{
						double val_in = CV_MAT_ELEM(*image_input,double,i+m,j+n);
						double weight = CV_MAT_ELEM(*convolution_core_mat,double,m+core_half_height,n+core_half_width);
						val_out += weight*val_in;
					}
				}				
			}
			CV_MAT_ELEM(*image_output,double,i,j) = val_out;
		}
	}

	return true;
}