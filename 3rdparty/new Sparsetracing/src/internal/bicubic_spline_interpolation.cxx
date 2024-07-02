#include "cubic_spline_interpolation.h"
#include "bicubic_spline_interpolation.h"

_6x6UniformBicubicSplineInterpolationMethod::_6x6UniformBicubicSplineInterpolationMethod()
{

}

_6x6UniformBicubicSplineInterpolationMethod::~_6x6UniformBicubicSplineInterpolationMethod()
{

}
bool _6x6UniformBicubicSplineInterpolationMethod::Build_Two_Dimension_Interpolation_6X6_Q_Matrix_4X6( CvMat* Q_mat )
{
	//判断
	//大小
	if (Q_mat->cols!=6||Q_mat->rows!=4)
	{
		return false;
	}

	//类型
	if (CV_MAT_TYPE(Q_mat->type) != CV_64F)
	{
		return false;
	}

	double* Q_mat__4X8_ptr = Q_mat->data.db;

	//q3
	double q3_de = 11.0;
	Q_mat__4X8_ptr[0] = 1.0/q3_de;
	Q_mat__4X8_ptr[1] = -6.0/q3_de;
	Q_mat__4X8_ptr[2] = 13.0/q3_de;
	Q_mat__4X8_ptr[3] = -13.0/q3_de;
	Q_mat__4X8_ptr[4] = 6.0/q3_de;
	Q_mat__4X8_ptr[5] = -1.0/q3_de;


	//q2
	double q2_de = 209.0;
	Q_mat__4X8_ptr[6] = -45.0/q2_de;
	Q_mat__4X8_ptr[7] = 270.0/q2_de;
	Q_mat__4X8_ptr[8] = -453.0/q2_de;
	Q_mat__4X8_ptr[9] = 288.0/q2_de;
	Q_mat__4X8_ptr[10] = -72.0/q2_de;
	Q_mat__4X8_ptr[11] = 12.0/q2_de;


	//q1
	double q1_de = q2_de;;
	Q_mat__4X8_ptr[12] = 26.0/q1_de;
	Q_mat__4X8_ptr[13] = -156.0/q1_de;
	Q_mat__4X8_ptr[14] = -3.0/q1_de;
	Q_mat__4X8_ptr[15] = 168.0/q1_de;
	Q_mat__4X8_ptr[16] = -42.0/q1_de;
	Q_mat__4X8_ptr[17] = 7.0/q1_de;

	//q0
	Q_mat__4X8_ptr[18] = 0;
	Q_mat__4X8_ptr[19] = 0;
	Q_mat__4X8_ptr[20] = 1.0;
	Q_mat__4X8_ptr[21] = 0;
	Q_mat__4X8_ptr[22] = 0;
	Q_mat__4X8_ptr[23] = 0;
	return true;
}
bool _6x6UniformBicubicSplineInterpolationMethod::Build_Two_Dimension_Interpolation_6X6_LUT(
	CvMat* image,
	CvMat* Q_mat,
	int aoi_x_min,
	int aoi_x_max,//上闭区间，下开区间
	int aoi_y_min,
	int aoi_y_max,//左闭区间，右边开区间
	CvMat* lut_mat
	)
{

	//判断
	//大小

	int image_width= image->width;
	int image_height = image->height;

	if (Q_mat->cols!=6||Q_mat->rows!=4)
	{
		return false;
	}


	//类型
	int image_type = 0;//1--CV_8U,2--CV_64F
	if (CV_MAT_TYPE(image->type) ==CV_8U )
	{
		image_type = 1;
	}
	else if (CV_MAT_TYPE(image->type) ==CV_64F)
	{
		image_type = 2;
	} 
	else
	{
		return false;
	}
	int lut_type  =0;//1--CV_32F,2--CV_64F
	if (CV_MAT_TYPE(lut_mat->type) == CV_32F)
	{
		lut_type =1;
	}
	else if (CV_MAT_TYPE(lut_mat->type) == CV_64F)
	{
		lut_type =2;
	} 
	else
	{
		return false;
	}

	if (CV_MAT_TYPE(Q_mat->type) != CV_64F)
	{
		return false;
	}



	if (aoi_x_min<2 || aoi_x_max>image_width-3||
		aoi_y_min<2 || aoi_y_max>image_height-3)
	{
		return false;
	}
	int aoi_width = aoi_x_max-aoi_x_min;
	int aoi_height = aoi_y_max-aoi_y_min;

	if (lut_mat->cols != aoi_width*UniformCubicNatureSplineInterpolationMethod::lut_element_size_||
		lut_mat->rows !=aoi_height)
	{
		return false;
	}



	double* Q_mat_ptr = Q_mat->data.db;
	if (lut_type==1)
	{
		if (image_type==1)
		{
			for (int i = 0;i<aoi_height;i++)
			{
				for (int j=0;j<aoi_width;j++)
				{
					float* lut_element_ptr = lut_mat->data.fl+(i*aoi_width+j)*UniformCubicNatureSplineInterpolationMethod::lut_element_size_;
					int index_i_min = aoi_y_min+i -2;
					int index_j_min = aoi_x_min+j-2;
					uchar* y = image->data.ptr + index_i_min*image_width+index_j_min;

					lut_element_ptr[0] = (Q_mat_ptr[0] * y[0] + Q_mat_ptr[1] * y[image_width] + Q_mat_ptr[2] * y[2 * image_width] + Q_mat_ptr[3] * y[3 * image_width] + Q_mat_ptr[4] * y[4 * image_width] + Q_mat_ptr[5] * y[5 * image_width]) * Q_mat_ptr[0] + (Q_mat_ptr[0] * y[1] + Q_mat_ptr[1] * y[1 + image_width] + Q_mat_ptr[2] * y[1 + 2 * image_width] + Q_mat_ptr[3] * y[1 + 3 * image_width] + Q_mat_ptr[4] * y[1 + 4 * image_width] + Q_mat_ptr[5] * y[1 + 5 * image_width]) * Q_mat_ptr[1] + (Q_mat_ptr[0] * y[2] + Q_mat_ptr[1] * y[2 + image_width] + Q_mat_ptr[2] * y[2 + 2 * image_width] + Q_mat_ptr[3] * y[2 + 3 * image_width] + Q_mat_ptr[4] * y[2 + 4 * image_width] + Q_mat_ptr[5] * y[2 + 5 * image_width]) * Q_mat_ptr[2] + (Q_mat_ptr[0] * y[3] + Q_mat_ptr[1] * y[3 + image_width] + Q_mat_ptr[2] * y[3 + 2 * image_width] + Q_mat_ptr[3] * y[3 + 3 * image_width] + Q_mat_ptr[4] * y[3 + 4 * image_width] + Q_mat_ptr[5] * y[3 + 5 * image_width]) * Q_mat_ptr[3] + (Q_mat_ptr[0] * y[4] + Q_mat_ptr[1] * y[4 + image_width] + Q_mat_ptr[2] * y[4 + 2 * image_width] + Q_mat_ptr[3] * y[4 + 3 * image_width] + Q_mat_ptr[4] * y[4 + 4 * image_width] + Q_mat_ptr[5] * y[4 + 5 * image_width]) * Q_mat_ptr[4] + (Q_mat_ptr[0] * y[5] + Q_mat_ptr[1] * y[5 + image_width] + Q_mat_ptr[2] * y[5 + 2 * image_width] + Q_mat_ptr[3] * y[5 + 3 * image_width] + Q_mat_ptr[4] * y[5 + 4 * image_width] + Q_mat_ptr[5] * y[5 + 5 * image_width]) * Q_mat_ptr[5];
					lut_element_ptr[1] = (Q_mat_ptr[0] * y[0] + Q_mat_ptr[1] * y[image_width] + Q_mat_ptr[2] * y[2 * image_width] + Q_mat_ptr[3] * y[3 * image_width] + Q_mat_ptr[4] * y[4 * image_width] + Q_mat_ptr[5] * y[5 * image_width]) * Q_mat_ptr[6] + (Q_mat_ptr[0] * y[1] + Q_mat_ptr[1] * y[1 + image_width] + Q_mat_ptr[2] * y[1 + 2 * image_width] + Q_mat_ptr[3] * y[1 + 3 * image_width] + Q_mat_ptr[4] * y[1 + 4 * image_width] + Q_mat_ptr[5] * y[1 + 5 * image_width]) * Q_mat_ptr[7] + (Q_mat_ptr[0] * y[2] + Q_mat_ptr[1] * y[2 + image_width] + Q_mat_ptr[2] * y[2 + 2 * image_width] + Q_mat_ptr[3] * y[2 + 3 * image_width] + Q_mat_ptr[4] * y[2 + 4 * image_width] + Q_mat_ptr[5] * y[2 + 5 * image_width]) * Q_mat_ptr[8] + (Q_mat_ptr[0] * y[3] + Q_mat_ptr[1] * y[3 + image_width] + Q_mat_ptr[2] * y[3 + 2 * image_width] + Q_mat_ptr[3] * y[3 + 3 * image_width] + Q_mat_ptr[4] * y[3 + 4 * image_width] + Q_mat_ptr[5] * y[3 + 5 * image_width]) * Q_mat_ptr[9] + (Q_mat_ptr[0] * y[4] + Q_mat_ptr[1] * y[4 + image_width] + Q_mat_ptr[2] * y[4 + 2 * image_width] + Q_mat_ptr[3] * y[4 + 3 * image_width] + Q_mat_ptr[4] * y[4 + 4 * image_width] + Q_mat_ptr[5] * y[4 + 5 * image_width]) * Q_mat_ptr[10] + (Q_mat_ptr[0] * y[5] + Q_mat_ptr[1] * y[5 + image_width] + Q_mat_ptr[2] * y[5 + 2 * image_width] + Q_mat_ptr[3] * y[5 + 3 * image_width] + Q_mat_ptr[4] * y[5 + 4 * image_width] + Q_mat_ptr[5] * y[5 + 5 * image_width]) * Q_mat_ptr[11];
					lut_element_ptr[2] = (Q_mat_ptr[0] * y[0] + Q_mat_ptr[1] * y[image_width] + Q_mat_ptr[2] * y[2 * image_width] + Q_mat_ptr[3] * y[3 * image_width] + Q_mat_ptr[4] * y[4 * image_width] + Q_mat_ptr[5] * y[5 * image_width]) * Q_mat_ptr[12] + (Q_mat_ptr[0] * y[1] + Q_mat_ptr[1] * y[1 + image_width] + Q_mat_ptr[2] * y[1 + 2 * image_width] + Q_mat_ptr[3] * y[1 + 3 * image_width] + Q_mat_ptr[4] * y[1 + 4 * image_width] + Q_mat_ptr[5] * y[1 + 5 * image_width]) * Q_mat_ptr[13] + (Q_mat_ptr[0] * y[2] + Q_mat_ptr[1] * y[2 + image_width] + Q_mat_ptr[2] * y[2 + 2 * image_width] + Q_mat_ptr[3] * y[2 + 3 * image_width] + Q_mat_ptr[4] * y[2 + 4 * image_width] + Q_mat_ptr[5] * y[2 + 5 * image_width]) * Q_mat_ptr[14] + (Q_mat_ptr[0] * y[3] + Q_mat_ptr[1] * y[3 + image_width] + Q_mat_ptr[2] * y[3 + 2 * image_width] + Q_mat_ptr[3] * y[3 + 3 * image_width] + Q_mat_ptr[4] * y[3 + 4 * image_width] + Q_mat_ptr[5] * y[3 + 5 * image_width]) * Q_mat_ptr[15] + (Q_mat_ptr[0] * y[4] + Q_mat_ptr[1] * y[4 + image_width] + Q_mat_ptr[2] * y[4 + 2 * image_width] + Q_mat_ptr[3] * y[4 + 3 * image_width] + Q_mat_ptr[4] * y[4 + 4 * image_width] + Q_mat_ptr[5] * y[4 + 5 * image_width]) * Q_mat_ptr[16] + (Q_mat_ptr[0] * y[5] + Q_mat_ptr[1] * y[5 + image_width] + Q_mat_ptr[2] * y[5 + 2 * image_width] + Q_mat_ptr[3] * y[5 + 3 * image_width] + Q_mat_ptr[4] * y[5 + 4 * image_width] + Q_mat_ptr[5] * y[5 + 5 * image_width]) * Q_mat_ptr[17];
					lut_element_ptr[3] = Q_mat_ptr[0] * y[2] + Q_mat_ptr[1] * y[2 + image_width] + Q_mat_ptr[2] * y[2 + 2 * image_width] + Q_mat_ptr[3] * y[2 + 3 * image_width] + Q_mat_ptr[4] * y[2 + 4 * image_width] + Q_mat_ptr[5] * y[2 + 5 * image_width];
					lut_element_ptr[4] = (Q_mat_ptr[6] * y[0] + Q_mat_ptr[7] * y[image_width] + Q_mat_ptr[8] * y[2 * image_width] + Q_mat_ptr[9] * y[3 * image_width] + Q_mat_ptr[10] * y[4 * image_width] + Q_mat_ptr[11] * y[5 * image_width]) * Q_mat_ptr[0] + (Q_mat_ptr[6] * y[1] + Q_mat_ptr[7] * y[1 + image_width] + Q_mat_ptr[8] * y[1 + 2 * image_width] + Q_mat_ptr[9] * y[1 + 3 * image_width] + Q_mat_ptr[10] * y[1 + 4 * image_width] + Q_mat_ptr[11] * y[1 + 5 * image_width]) * Q_mat_ptr[1] + (Q_mat_ptr[6] * y[2] + Q_mat_ptr[7] * y[2 + image_width] + Q_mat_ptr[8] * y[2 + 2 * image_width] + Q_mat_ptr[9] * y[2 + 3 * image_width] + Q_mat_ptr[10] * y[2 + 4 * image_width] + Q_mat_ptr[11] * y[2 + 5 * image_width]) * Q_mat_ptr[2] + (Q_mat_ptr[6] * y[3] + Q_mat_ptr[7] * y[3 + image_width] + Q_mat_ptr[8] * y[3 + 2 * image_width] + Q_mat_ptr[9] * y[3 + 3 * image_width] + Q_mat_ptr[10] * y[3 + 4 * image_width] + Q_mat_ptr[11] * y[3 + 5 * image_width]) * Q_mat_ptr[3] + (Q_mat_ptr[6] * y[4] + Q_mat_ptr[7] * y[4 + image_width] + Q_mat_ptr[8] * y[4 + 2 * image_width] + Q_mat_ptr[9] * y[4 + 3 * image_width] + Q_mat_ptr[10] * y[4 + 4 * image_width] + Q_mat_ptr[11] * y[4 + 5 * image_width]) * Q_mat_ptr[4] + (Q_mat_ptr[6] * y[5] + Q_mat_ptr[7] * y[5 + image_width] + Q_mat_ptr[8] * y[5 + 2 * image_width] + Q_mat_ptr[9] * y[5 + 3 * image_width] + Q_mat_ptr[10] * y[5 + 4 * image_width] + Q_mat_ptr[11] * y[5 + 5 * image_width]) * Q_mat_ptr[5];
					lut_element_ptr[5] = (Q_mat_ptr[6] * y[0] + Q_mat_ptr[7] * y[image_width] + Q_mat_ptr[8] * y[2 * image_width] + Q_mat_ptr[9] * y[3 * image_width] + Q_mat_ptr[10] * y[4 * image_width] + Q_mat_ptr[11] * y[5 * image_width]) * Q_mat_ptr[6] + (Q_mat_ptr[6] * y[1] + Q_mat_ptr[7] * y[1 + image_width] + Q_mat_ptr[8] * y[1 + 2 * image_width] + Q_mat_ptr[9] * y[1 + 3 * image_width] + Q_mat_ptr[10] * y[1 + 4 * image_width] + Q_mat_ptr[11] * y[1 + 5 * image_width]) * Q_mat_ptr[7] + (Q_mat_ptr[6] * y[2] + Q_mat_ptr[7] * y[2 + image_width] + Q_mat_ptr[8] * y[2 + 2 * image_width] + Q_mat_ptr[9] * y[2 + 3 * image_width] + Q_mat_ptr[10] * y[2 + 4 * image_width] + Q_mat_ptr[11] * y[2 + 5 * image_width]) * Q_mat_ptr[8] + (Q_mat_ptr[6] * y[3] + Q_mat_ptr[7] * y[3 + image_width] + Q_mat_ptr[8] * y[3 + 2 * image_width] + Q_mat_ptr[9] * y[3 + 3 * image_width] + Q_mat_ptr[10] * y[3 + 4 * image_width] + Q_mat_ptr[11] * y[3 + 5 * image_width]) * Q_mat_ptr[9] + (Q_mat_ptr[6] * y[4] + Q_mat_ptr[7] * y[4 + image_width] + Q_mat_ptr[8] * y[4 + 2 * image_width] + Q_mat_ptr[9] * y[4 + 3 * image_width] + Q_mat_ptr[10] * y[4 + 4 * image_width] + Q_mat_ptr[11] * y[4 + 5 * image_width]) * Q_mat_ptr[10] + (Q_mat_ptr[6] * y[5] + Q_mat_ptr[7] * y[5 + image_width] + Q_mat_ptr[8] * y[5 + 2 * image_width] + Q_mat_ptr[9] * y[5 + 3 * image_width] + Q_mat_ptr[10] * y[5 + 4 * image_width] + Q_mat_ptr[11] * y[5 + 5 * image_width]) * Q_mat_ptr[11];
					lut_element_ptr[6] = (Q_mat_ptr[6] * y[0] + Q_mat_ptr[7] * y[image_width] + Q_mat_ptr[8] * y[2 * image_width] + Q_mat_ptr[9] * y[3 * image_width] + Q_mat_ptr[10] * y[4 * image_width] + Q_mat_ptr[11] * y[5 * image_width]) * Q_mat_ptr[12] + (Q_mat_ptr[6] * y[1] + Q_mat_ptr[7] * y[1 + image_width] + Q_mat_ptr[8] * y[1 + 2 * image_width] + Q_mat_ptr[9] * y[1 + 3 * image_width] + Q_mat_ptr[10] * y[1 + 4 * image_width] + Q_mat_ptr[11] * y[1 + 5 * image_width]) * Q_mat_ptr[13] + (Q_mat_ptr[6] * y[2] + Q_mat_ptr[7] * y[2 + image_width] + Q_mat_ptr[8] * y[2 + 2 * image_width] + Q_mat_ptr[9] * y[2 + 3 * image_width] + Q_mat_ptr[10] * y[2 + 4 * image_width] + Q_mat_ptr[11] * y[2 + 5 * image_width]) * Q_mat_ptr[14] + (Q_mat_ptr[6] * y[3] + Q_mat_ptr[7] * y[3 + image_width] + Q_mat_ptr[8] * y[3 + 2 * image_width] + Q_mat_ptr[9] * y[3 + 3 * image_width] + Q_mat_ptr[10] * y[3 + 4 * image_width] + Q_mat_ptr[11] * y[3 + 5 * image_width]) * Q_mat_ptr[15] + (Q_mat_ptr[6] * y[4] + Q_mat_ptr[7] * y[4 + image_width] + Q_mat_ptr[8] * y[4 + 2 * image_width] + Q_mat_ptr[9] * y[4 + 3 * image_width] + Q_mat_ptr[10] * y[4 + 4 * image_width] + Q_mat_ptr[11] * y[4 + 5 * image_width]) * Q_mat_ptr[16] + (Q_mat_ptr[6] * y[5] + Q_mat_ptr[7] * y[5 + image_width] + Q_mat_ptr[8] * y[5 + 2 * image_width] + Q_mat_ptr[9] * y[5 + 3 * image_width] + Q_mat_ptr[10] * y[5 + 4 * image_width] + Q_mat_ptr[11] * y[5 + 5 * image_width]) * Q_mat_ptr[17];
					lut_element_ptr[7] = Q_mat_ptr[6] * y[2] + Q_mat_ptr[7] * y[2 + image_width] + Q_mat_ptr[8] * y[2 + 2 * image_width] + Q_mat_ptr[9] * y[2 + 3 * image_width] + Q_mat_ptr[10] * y[2 + 4 * image_width] + Q_mat_ptr[11] * y[2 + 5 * image_width];
					lut_element_ptr[8] = (Q_mat_ptr[12] * y[0] + Q_mat_ptr[13] * y[image_width] + Q_mat_ptr[14] * y[2 * image_width] + Q_mat_ptr[15] * y[3 * image_width] + Q_mat_ptr[16] * y[4 * image_width] + Q_mat_ptr[17] * y[5 * image_width]) * Q_mat_ptr[0] + (Q_mat_ptr[12] * y[1] + Q_mat_ptr[13] * y[1 + image_width] + Q_mat_ptr[14] * y[1 + 2 * image_width] + Q_mat_ptr[15] * y[1 + 3 * image_width] + Q_mat_ptr[16] * y[1 + 4 * image_width] + Q_mat_ptr[17] * y[1 + 5 * image_width]) * Q_mat_ptr[1] + (Q_mat_ptr[12] * y[2] + Q_mat_ptr[13] * y[2 + image_width] + Q_mat_ptr[14] * y[2 + 2 * image_width] + Q_mat_ptr[15] * y[2 + 3 * image_width] + Q_mat_ptr[16] * y[2 + 4 * image_width] + Q_mat_ptr[17] * y[2 + 5 * image_width]) * Q_mat_ptr[2] + (Q_mat_ptr[12] * y[3] + Q_mat_ptr[13] * y[3 + image_width] + Q_mat_ptr[14] * y[3 + 2 * image_width] + Q_mat_ptr[15] * y[3 + 3 * image_width] + Q_mat_ptr[16] * y[3 + 4 * image_width] + Q_mat_ptr[17] * y[3 + 5 * image_width]) * Q_mat_ptr[3] + (Q_mat_ptr[12] * y[4] + Q_mat_ptr[13] * y[4 + image_width] + Q_mat_ptr[14] * y[4 + 2 * image_width] + Q_mat_ptr[15] * y[4 + 3 * image_width] + Q_mat_ptr[16] * y[4 + 4 * image_width] + Q_mat_ptr[17] * y[4 + 5 * image_width]) * Q_mat_ptr[4] + (Q_mat_ptr[12] * y[5] + Q_mat_ptr[13] * y[5 + image_width] + Q_mat_ptr[14] * y[5 + 2 * image_width] + Q_mat_ptr[15] * y[5 + 3 * image_width] + Q_mat_ptr[16] * y[5 + 4 * image_width] + Q_mat_ptr[17] * y[5 + 5 * image_width]) * Q_mat_ptr[5];
					lut_element_ptr[9] = (Q_mat_ptr[12] * y[0] + Q_mat_ptr[13] * y[image_width] + Q_mat_ptr[14] * y[2 * image_width] + Q_mat_ptr[15] * y[3 * image_width] + Q_mat_ptr[16] * y[4 * image_width] + Q_mat_ptr[17] * y[5 * image_width]) * Q_mat_ptr[6] + (Q_mat_ptr[12] * y[1] + Q_mat_ptr[13] * y[1 + image_width] + Q_mat_ptr[14] * y[1 + 2 * image_width] + Q_mat_ptr[15] * y[1 + 3 * image_width] + Q_mat_ptr[16] * y[1 + 4 * image_width] + Q_mat_ptr[17] * y[1 + 5 * image_width]) * Q_mat_ptr[7] + (Q_mat_ptr[12] * y[2] + Q_mat_ptr[13] * y[2 + image_width] + Q_mat_ptr[14] * y[2 + 2 * image_width] + Q_mat_ptr[15] * y[2 + 3 * image_width] + Q_mat_ptr[16] * y[2 + 4 * image_width] + Q_mat_ptr[17] * y[2 + 5 * image_width]) * Q_mat_ptr[8] + (Q_mat_ptr[12] * y[3] + Q_mat_ptr[13] * y[3 + image_width] + Q_mat_ptr[14] * y[3 + 2 * image_width] + Q_mat_ptr[15] * y[3 + 3 * image_width] + Q_mat_ptr[16] * y[3 + 4 * image_width] + Q_mat_ptr[17] * y[3 + 5 * image_width]) * Q_mat_ptr[9] + (Q_mat_ptr[12] * y[4] + Q_mat_ptr[13] * y[4 + image_width] + Q_mat_ptr[14] * y[4 + 2 * image_width] + Q_mat_ptr[15] * y[4 + 3 * image_width] + Q_mat_ptr[16] * y[4 + 4 * image_width] + Q_mat_ptr[17] * y[4 + 5 * image_width]) * Q_mat_ptr[10] + (Q_mat_ptr[12] * y[5] + Q_mat_ptr[13] * y[5 + image_width] + Q_mat_ptr[14] * y[5 + 2 * image_width] + Q_mat_ptr[15] * y[5 + 3 * image_width] + Q_mat_ptr[16] * y[5 + 4 * image_width] + Q_mat_ptr[17] * y[5 + 5 * image_width]) * Q_mat_ptr[11];
					lut_element_ptr[10] = (Q_mat_ptr[12] * y[0] + Q_mat_ptr[13] * y[image_width] + Q_mat_ptr[14] * y[2 * image_width] + Q_mat_ptr[15] * y[3 * image_width] + Q_mat_ptr[16] * y[4 * image_width] + Q_mat_ptr[17] * y[5 * image_width]) * Q_mat_ptr[12] + (Q_mat_ptr[12] * y[1] + Q_mat_ptr[13] * y[1 + image_width] + Q_mat_ptr[14] * y[1 + 2 * image_width] + Q_mat_ptr[15] * y[1 + 3 * image_width] + Q_mat_ptr[16] * y[1 + 4 * image_width] + Q_mat_ptr[17] * y[1 + 5 * image_width]) * Q_mat_ptr[13] + (Q_mat_ptr[12] * y[2] + Q_mat_ptr[13] * y[2 + image_width] + Q_mat_ptr[14] * y[2 + 2 * image_width] + Q_mat_ptr[15] * y[2 + 3 * image_width] + Q_mat_ptr[16] * y[2 + 4 * image_width] + Q_mat_ptr[17] * y[2 + 5 * image_width]) * Q_mat_ptr[14] + (Q_mat_ptr[12] * y[3] + Q_mat_ptr[13] * y[3 + image_width] + Q_mat_ptr[14] * y[3 + 2 * image_width] + Q_mat_ptr[15] * y[3 + 3 * image_width] + Q_mat_ptr[16] * y[3 + 4 * image_width] + Q_mat_ptr[17] * y[3 + 5 * image_width]) * Q_mat_ptr[15] + (Q_mat_ptr[12] * y[4] + Q_mat_ptr[13] * y[4 + image_width] + Q_mat_ptr[14] * y[4 + 2 * image_width] + Q_mat_ptr[15] * y[4 + 3 * image_width] + Q_mat_ptr[16] * y[4 + 4 * image_width] + Q_mat_ptr[17] * y[4 + 5 * image_width]) * Q_mat_ptr[16] + (Q_mat_ptr[12] * y[5] + Q_mat_ptr[13] * y[5 + image_width] + Q_mat_ptr[14] * y[5 + 2 * image_width] + Q_mat_ptr[15] * y[5 + 3 * image_width] + Q_mat_ptr[16] * y[5 + 4 * image_width] + Q_mat_ptr[17] * y[5 + 5 * image_width]) * Q_mat_ptr[17];
					lut_element_ptr[11] = Q_mat_ptr[12] * y[2] + Q_mat_ptr[13] * y[2 + image_width] + Q_mat_ptr[14] * y[2 + 2 * image_width] + Q_mat_ptr[15] * y[2 + 3 * image_width] + Q_mat_ptr[16] * y[2 + 4 * image_width] + Q_mat_ptr[17] * y[2 + 5 * image_width];
					lut_element_ptr[12] = y[2 * image_width] * Q_mat_ptr[0] + y[1 + 2 * image_width] * Q_mat_ptr[1] + Q_mat_ptr[2] * y[2 + 2 * image_width] + y[3 + 2 * image_width] * Q_mat_ptr[3] + y[4 + 2 * image_width] * Q_mat_ptr[4] + y[5 + 2 * image_width] * Q_mat_ptr[5];
					lut_element_ptr[13] = y[2 * image_width] * Q_mat_ptr[6] + y[1 + 2 * image_width] * Q_mat_ptr[7] + Q_mat_ptr[8] * y[2 + 2 * image_width] + y[3 + 2 * image_width] * Q_mat_ptr[9] + y[4 + 2 * image_width] * Q_mat_ptr[10] + y[5 + 2 * image_width] * Q_mat_ptr[11];
					lut_element_ptr[14] = y[2 * image_width] * Q_mat_ptr[12] + y[1 + 2 * image_width] * Q_mat_ptr[13] + Q_mat_ptr[14] * y[2 + 2 * image_width] + y[3 + 2 * image_width] * Q_mat_ptr[15] + y[4 + 2 * image_width] * Q_mat_ptr[16] + y[5 + 2 * image_width] * Q_mat_ptr[17];
					lut_element_ptr[15] = y[2 + 2 * image_width];



					//lut_element_ptr[0] =(Q_mat_ptr[0]*y[0]+Q_mat_ptr[1]*y[image_width]+Q_mat_ptr[2]*y[image_width*2]+Q_mat_ptr[3]*y[image_width*3]+Q_mat_ptr[4]*y[image_width*4]+Q_mat_ptr[5]*y[image_width*5])*Q_mat_ptr[0]+(Q_mat_ptr[0]*y[1]+Q_mat_ptr[1]*y[image_width+1]+Q_mat_ptr[2]*y[image_width*2+1]+Q_mat_ptr[3]*y[image_width*3+1]+Q_mat_ptr[4]*y[image_width*4+1]+Q_mat_ptr[5]*y[image_width*5+1])*Q_mat_ptr[1]+(Q_mat_ptr[0]*y[2]+Q_mat_ptr[1]*y[image_width+2]+Q_mat_ptr[2]*y[image_width*2+2]+Q_mat_ptr[3]*y[image_width*3+2]+Q_mat_ptr[4]*y[image_width*4+2]+Q_mat_ptr[5]*y[image_width*5+2])*Q_mat_ptr[2]+(Q_mat_ptr[0]*y[3]+Q_mat_ptr[1]*y[image_width+3]+Q_mat_ptr[2]*y[image_width*2+3]+Q_mat_ptr[3]*y[image_width*3+3]+Q_mat_ptr[4]*y[image_width*4+3]+Q_mat_ptr[5]*y[image_width*5+3])*Q_mat_ptr[3]+(Q_mat_ptr[0]*y[4]+Q_mat_ptr[1]*y[image_width+4]+Q_mat_ptr[2]*y[image_width*2+4]+Q_mat_ptr[3]*y[image_width*3+4]+Q_mat_ptr[4]*y[image_width*4+4]+Q_mat_ptr[5]*y[image_width*5+4])*Q_mat_ptr[4]+(Q_mat_ptr[0]*y[5]+Q_mat_ptr[1]*y[image_width+5]+Q_mat_ptr[2]*y[image_width*2+5]+Q_mat_ptr[3]*y[image_width*3+5]+Q_mat_ptr[4]*y[image_width*4+5]+Q_mat_ptr[5]*y[image_width*5+5])*Q_mat_ptr[5];
					//lut_element_ptr[1] = (Q_mat_ptr[0]*y[0]+Q_mat_ptr[1]*y[image_width]+Q_mat_ptr[2]*y[image_width*2]+Q_mat_ptr[3]*y[image_width*3]+Q_mat_ptr[4]*y[image_width*4]+Q_mat_ptr[5]*y[image_width*5])*Q_mat_ptr[6]+(Q_mat_ptr[0]*y[1]+Q_mat_ptr[1]*y[image_width+1]+Q_mat_ptr[2]*y[image_width*2+1]+Q_mat_ptr[3]*y[image_width*3+1]+Q_mat_ptr[4]*y[image_width*4+1]+Q_mat_ptr[5]*y[image_width*5+1])*Q_mat_ptr[7]+(Q_mat_ptr[0]*y[2]+Q_mat_ptr[1]*y[image_width+2]+Q_mat_ptr[2]*y[image_width*2+2]+Q_mat_ptr[3]*y[image_width*3+2]+Q_mat_ptr[4]*y[image_width*4+2]+Q_mat_ptr[5]*y[image_width*5+2])*Q_mat_ptr[8]+(Q_mat_ptr[0]*y[3]+Q_mat_ptr[1]*y[image_width+3]+Q_mat_ptr[2]*y[image_width*2+3]+Q_mat_ptr[3]*y[image_width*3+3]+Q_mat_ptr[4]*y[image_width*4+3]+Q_mat_ptr[5]*y[image_width*5+3])*Q_mat_ptr[9]+(Q_mat_ptr[0]*y[4]+Q_mat_ptr[1]*y[image_width+4]+Q_mat_ptr[2]*y[image_width*2+4]+Q_mat_ptr[3]*y[image_width*3+4]+Q_mat_ptr[4]*y[image_width*4+4]+Q_mat_ptr[5]*y[image_width*5+4])*Q_mat_ptr[10]+(Q_mat_ptr[0]*y[5]+Q_mat_ptr[1]*y[image_width+5]+Q_mat_ptr[2]*y[image_width*2+5]+Q_mat_ptr[3]*y[image_width*3+5]+Q_mat_ptr[4]*y[image_width*4+5]+Q_mat_ptr[5]*y[image_width*5+5])*Q_mat_ptr[11];
					//lut_element_ptr[2] = (Q_mat_ptr[0]*y[0]+Q_mat_ptr[1]*y[image_width]+Q_mat_ptr[2]*y[image_width*2]+Q_mat_ptr[3]*y[image_width*3]+Q_mat_ptr[4]*y[image_width*4]+Q_mat_ptr[5]*y[image_width*5])*Q_mat_ptr[12]+(Q_mat_ptr[0]*y[1]+Q_mat_ptr[1]*y[image_width+1]+Q_mat_ptr[2]*y[image_width*2+1]+Q_mat_ptr[3]*y[image_width*3+1]+Q_mat_ptr[4]*y[image_width*4+1]+Q_mat_ptr[5]*y[image_width*5+1])*Q_mat_ptr[13]+(Q_mat_ptr[0]*y[2]+Q_mat_ptr[1]*y[image_width+2]+Q_mat_ptr[2]*y[image_width*2+2]+Q_mat_ptr[3]*y[image_width*3+2]+Q_mat_ptr[4]*y[image_width*4+2]+Q_mat_ptr[5]*y[image_width*5+2])*Q_mat_ptr[14]+(Q_mat_ptr[0]*y[3]+Q_mat_ptr[1]*y[image_width+3]+Q_mat_ptr[2]*y[image_width*2+3]+Q_mat_ptr[3]*y[image_width*3+3]+Q_mat_ptr[4]*y[image_width*4+3]+Q_mat_ptr[5]*y[image_width*5+3])*Q_mat_ptr[15]+(Q_mat_ptr[0]*y[4]+Q_mat_ptr[1]*y[image_width+4]+Q_mat_ptr[2]*y[image_width*2+4]+Q_mat_ptr[3]*y[image_width*3+4]+Q_mat_ptr[4]*y[image_width*4+4]+Q_mat_ptr[5]*y[image_width*5+4])*Q_mat_ptr[16]+(Q_mat_ptr[0]*y[5]+Q_mat_ptr[1]*y[image_width+5]+Q_mat_ptr[2]*y[image_width*2+5]+Q_mat_ptr[3]*y[image_width*3+5]+Q_mat_ptr[4]*y[image_width*4+5]+Q_mat_ptr[5]*y[image_width*5+5])*Q_mat_ptr[17];
					//lut_element_ptr[3] = (Q_mat_ptr[0]*y[0]+Q_mat_ptr[1]*y[image_width]+Q_mat_ptr[2]*y[image_width*2]+Q_mat_ptr[3]*y[image_width*3]+Q_mat_ptr[4]*y[image_width*4]+Q_mat_ptr[5]*y[image_width*5])*Q_mat_ptr[18]+(Q_mat_ptr[0]*y[1]+Q_mat_ptr[1]*y[image_width+1]+Q_mat_ptr[2]*y[image_width*2+1]+Q_mat_ptr[3]*y[image_width*3+1]+Q_mat_ptr[4]*y[image_width*4+1]+Q_mat_ptr[5]*y[image_width*5+1])*Q_mat_ptr[19]+(Q_mat_ptr[0]*y[2]+Q_mat_ptr[1]*y[image_width+2]+Q_mat_ptr[2]*y[image_width*2+2]+Q_mat_ptr[3]*y[image_width*3+2]+Q_mat_ptr[4]*y[image_width*4+2]+Q_mat_ptr[5]*y[image_width*5+2])*Q_mat_ptr[20]+(Q_mat_ptr[0]*y[3]+Q_mat_ptr[1]*y[image_width+3]+Q_mat_ptr[2]*y[image_width*2+3]+Q_mat_ptr[3]*y[image_width*3+3]+Q_mat_ptr[4]*y[image_width*4+3]+Q_mat_ptr[5]*y[image_width*5+3])*Q_mat_ptr[21]+(Q_mat_ptr[0]*y[4]+Q_mat_ptr[1]*y[image_width+4]+Q_mat_ptr[2]*y[image_width*2+4]+Q_mat_ptr[3]*y[image_width*3+4]+Q_mat_ptr[4]*y[image_width*4+4]+Q_mat_ptr[5]*y[image_width*5+4])*Q_mat_ptr[22]+(Q_mat_ptr[0]*y[5]+Q_mat_ptr[1]*y[image_width+5]+Q_mat_ptr[2]*y[image_width*2+5]+Q_mat_ptr[3]*y[image_width*3+5]+Q_mat_ptr[4]*y[image_width*4+5]+Q_mat_ptr[5]*y[image_width*5+5])*Q_mat_ptr[23];
					//lut_element_ptr[4] = (Q_mat_ptr[6]*y[0]+Q_mat_ptr[7]*y[image_width]+Q_mat_ptr[8]*y[image_width*2]+Q_mat_ptr[9]*y[image_width*3]+Q_mat_ptr[10]*y[image_width*4]+Q_mat_ptr[11]*y[image_width*5])*Q_mat_ptr[0]+(Q_mat_ptr[6]*y[1]+Q_mat_ptr[7]*y[image_width+1]+Q_mat_ptr[8]*y[image_width*2+1]+Q_mat_ptr[9]*y[image_width*3+1]+Q_mat_ptr[10]*y[image_width*4+1]+Q_mat_ptr[11]*y[image_width*5+1])*Q_mat_ptr[1]+(Q_mat_ptr[6]*y[2]+Q_mat_ptr[7]*y[image_width+2]+Q_mat_ptr[8]*y[image_width*2+2]+Q_mat_ptr[9]*y[image_width*3+2]+Q_mat_ptr[10]*y[image_width*4+2]+Q_mat_ptr[11]*y[image_width*5+2])*Q_mat_ptr[2]+(Q_mat_ptr[6]*y[3]+Q_mat_ptr[7]*y[image_width+3]+Q_mat_ptr[8]*y[image_width*2+3]+Q_mat_ptr[9]*y[image_width*3+3]+Q_mat_ptr[10]*y[image_width*4+3]+Q_mat_ptr[11]*y[image_width*5+3])*Q_mat_ptr[3]+(Q_mat_ptr[6]*y[4]+Q_mat_ptr[7]*y[image_width+4]+Q_mat_ptr[8]*y[image_width*2+4]+Q_mat_ptr[9]*y[image_width*3+4]+Q_mat_ptr[10]*y[image_width*4+4]+Q_mat_ptr[11]*y[image_width*5+4])*Q_mat_ptr[4]+(Q_mat_ptr[6]*y[5]+Q_mat_ptr[7]*y[image_width+5]+Q_mat_ptr[8]*y[image_width*2+5]+Q_mat_ptr[9]*y[image_width*3+5]+Q_mat_ptr[10]*y[image_width*4+5]+Q_mat_ptr[11]*y[image_width*5+5])*Q_mat_ptr[5];
					//lut_element_ptr[5] = (Q_mat_ptr[6]*y[0]+Q_mat_ptr[7]*y[image_width]+Q_mat_ptr[8]*y[image_width*2]+Q_mat_ptr[9]*y[image_width*3]+Q_mat_ptr[10]*y[image_width*4]+Q_mat_ptr[11]*y[image_width*5])*Q_mat_ptr[6]+(Q_mat_ptr[6]*y[1]+Q_mat_ptr[7]*y[image_width+1]+Q_mat_ptr[8]*y[image_width*2+1]+Q_mat_ptr[9]*y[image_width*3+1]+Q_mat_ptr[10]*y[image_width*4+1]+Q_mat_ptr[11]*y[image_width*5+1])*Q_mat_ptr[7]+(Q_mat_ptr[6]*y[2]+Q_mat_ptr[7]*y[image_width+2]+Q_mat_ptr[8]*y[image_width*2+2]+Q_mat_ptr[9]*y[image_width*3+2]+Q_mat_ptr[10]*y[image_width*4+2]+Q_mat_ptr[11]*y[image_width*5+2])*Q_mat_ptr[8]+(Q_mat_ptr[6]*y[3]+Q_mat_ptr[7]*y[image_width+3]+Q_mat_ptr[8]*y[image_width*2+3]+Q_mat_ptr[9]*y[image_width*3+3]+Q_mat_ptr[10]*y[image_width*4+3]+Q_mat_ptr[11]*y[image_width*5+3])*Q_mat_ptr[9]+(Q_mat_ptr[6]*y[4]+Q_mat_ptr[7]*y[image_width+4]+Q_mat_ptr[8]*y[image_width*2+4]+Q_mat_ptr[9]*y[image_width*3+4]+Q_mat_ptr[10]*y[image_width*4+4]+Q_mat_ptr[11]*y[image_width*5+4])*Q_mat_ptr[10]+(Q_mat_ptr[6]*y[5]+Q_mat_ptr[7]*y[image_width+5]+Q_mat_ptr[8]*y[image_width*2+5]+Q_mat_ptr[9]*y[image_width*3+5]+Q_mat_ptr[10]*y[image_width*4+5]+Q_mat_ptr[11]*y[image_width*5+5])*Q_mat_ptr[11];
					//lut_element_ptr[6] = (Q_mat_ptr[6]*y[0]+Q_mat_ptr[7]*y[image_width]+Q_mat_ptr[8]*y[image_width*2]+Q_mat_ptr[9]*y[image_width*3]+Q_mat_ptr[10]*y[image_width*4]+Q_mat_ptr[11]*y[image_width*5])*Q_mat_ptr[12]+(Q_mat_ptr[6]*y[1]+Q_mat_ptr[7]*y[image_width+1]+Q_mat_ptr[8]*y[image_width*2+1]+Q_mat_ptr[9]*y[image_width*3+1]+Q_mat_ptr[10]*y[image_width*4+1]+Q_mat_ptr[11]*y[image_width*5+1])*Q_mat_ptr[13]+(Q_mat_ptr[6]*y[2]+Q_mat_ptr[7]*y[image_width+2]+Q_mat_ptr[8]*y[image_width*2+2]+Q_mat_ptr[9]*y[image_width*3+2]+Q_mat_ptr[10]*y[image_width*4+2]+Q_mat_ptr[11]*y[image_width*5+2])*Q_mat_ptr[14]+(Q_mat_ptr[6]*y[3]+Q_mat_ptr[7]*y[image_width+3]+Q_mat_ptr[8]*y[image_width*2+3]+Q_mat_ptr[9]*y[image_width*3+3]+Q_mat_ptr[10]*y[image_width*4+3]+Q_mat_ptr[11]*y[image_width*5+3])*Q_mat_ptr[15]+(Q_mat_ptr[6]*y[4]+Q_mat_ptr[7]*y[image_width+4]+Q_mat_ptr[8]*y[image_width*2+4]+Q_mat_ptr[9]*y[image_width*3+4]+Q_mat_ptr[10]*y[image_width*4+4]+Q_mat_ptr[11]*y[image_width*5+4])*Q_mat_ptr[16]+(Q_mat_ptr[6]*y[5]+Q_mat_ptr[7]*y[image_width+5]+Q_mat_ptr[8]*y[image_width*2+5]+Q_mat_ptr[9]*y[image_width*3+5]+Q_mat_ptr[10]*y[image_width*4+5]+Q_mat_ptr[11]*y[image_width*5+5])*Q_mat_ptr[17];
					//lut_element_ptr[7] = (Q_mat_ptr[6]*y[0]+Q_mat_ptr[7]*y[image_width]+Q_mat_ptr[8]*y[image_width*2]+Q_mat_ptr[9]*y[image_width*3]+Q_mat_ptr[10]*y[image_width*4]+Q_mat_ptr[11]*y[image_width*5])*Q_mat_ptr[18]+(Q_mat_ptr[6]*y[1]+Q_mat_ptr[7]*y[image_width+1]+Q_mat_ptr[8]*y[image_width*2+1]+Q_mat_ptr[9]*y[image_width*3+1]+Q_mat_ptr[10]*y[image_width*4+1]+Q_mat_ptr[11]*y[image_width*5+1])*Q_mat_ptr[19]+(Q_mat_ptr[6]*y[2]+Q_mat_ptr[7]*y[image_width+2]+Q_mat_ptr[8]*y[image_width*2+2]+Q_mat_ptr[9]*y[image_width*3+2]+Q_mat_ptr[10]*y[image_width*4+2]+Q_mat_ptr[11]*y[image_width*5+2])*Q_mat_ptr[20]+(Q_mat_ptr[6]*y[3]+Q_mat_ptr[7]*y[image_width+3]+Q_mat_ptr[8]*y[image_width*2+3]+Q_mat_ptr[9]*y[image_width*3+3]+Q_mat_ptr[10]*y[image_width*4+3]+Q_mat_ptr[11]*y[image_width*5+3])*Q_mat_ptr[21]+(Q_mat_ptr[6]*y[4]+Q_mat_ptr[7]*y[image_width+4]+Q_mat_ptr[8]*y[image_width*2+4]+Q_mat_ptr[9]*y[image_width*3+4]+Q_mat_ptr[10]*y[image_width*4+4]+Q_mat_ptr[11]*y[image_width*5+4])*Q_mat_ptr[22]+(Q_mat_ptr[6]*y[5]+Q_mat_ptr[7]*y[image_width+5]+Q_mat_ptr[8]*y[image_width*2+5]+Q_mat_ptr[9]*y[image_width*3+5]+Q_mat_ptr[10]*y[image_width*4+5]+Q_mat_ptr[11]*y[image_width*5+5])*Q_mat_ptr[23]; 
					//lut_element_ptr[8] = (Q_mat_ptr[12]*y[0]+Q_mat_ptr[13]*y[image_width]+Q_mat_ptr[14]*y[image_width*2]+Q_mat_ptr[15]*y[image_width*3]+Q_mat_ptr[16]*y[image_width*4]+Q_mat_ptr[17]*y[image_width*5])*Q_mat_ptr[0]+(Q_mat_ptr[12]*y[1]+Q_mat_ptr[13]*y[image_width+1]+Q_mat_ptr[14]*y[image_width*2+1]+Q_mat_ptr[15]*y[image_width*3+1]+Q_mat_ptr[16]*y[image_width*4+1]+Q_mat_ptr[17]*y[image_width*5+1])*Q_mat_ptr[1]+(Q_mat_ptr[12]*y[2]+Q_mat_ptr[13]*y[image_width+2]+Q_mat_ptr[14]*y[image_width*2+2]+Q_mat_ptr[15]*y[image_width*3+2]+Q_mat_ptr[16]*y[image_width*4+2]+Q_mat_ptr[17]*y[image_width*5+2])*Q_mat_ptr[2]+(Q_mat_ptr[12]*y[3]+Q_mat_ptr[13]*y[image_width+3]+Q_mat_ptr[14]*y[image_width*2+3]+Q_mat_ptr[15]*y[image_width*3+3]+Q_mat_ptr[16]*y[image_width*4+3]+Q_mat_ptr[17]*y[image_width*5+3])*Q_mat_ptr[3]+(Q_mat_ptr[12]*y[4]+Q_mat_ptr[13]*y[image_width+4]+Q_mat_ptr[14]*y[image_width*2+4]+Q_mat_ptr[15]*y[image_width*3+4]+Q_mat_ptr[16]*y[image_width*4+4]+Q_mat_ptr[17]*y[image_width*5+4])*Q_mat_ptr[4]+(Q_mat_ptr[12]*y[5]+Q_mat_ptr[13]*y[image_width+5]+Q_mat_ptr[14]*y[image_width*2+5]+Q_mat_ptr[15]*y[image_width*3+5]+Q_mat_ptr[16]*y[image_width*4+5]+Q_mat_ptr[17]*y[image_width*5+5])*Q_mat_ptr[5];
					//lut_element_ptr[9] = (Q_mat_ptr[12]*y[0]+Q_mat_ptr[13]*y[image_width]+Q_mat_ptr[14]*y[image_width*2]+Q_mat_ptr[15]*y[image_width*3]+Q_mat_ptr[16]*y[image_width*4]+Q_mat_ptr[17]*y[image_width*5])*Q_mat_ptr[6]+(Q_mat_ptr[12]*y[1]+Q_mat_ptr[13]*y[image_width+1]+Q_mat_ptr[14]*y[image_width*2+1]+Q_mat_ptr[15]*y[image_width*3+1]+Q_mat_ptr[16]*y[image_width*4+1]+Q_mat_ptr[17]*y[image_width*5+1])*Q_mat_ptr[7]+(Q_mat_ptr[12]*y[2]+Q_mat_ptr[13]*y[image_width+2]+Q_mat_ptr[14]*y[image_width*2+2]+Q_mat_ptr[15]*y[image_width*3+2]+Q_mat_ptr[16]*y[image_width*4+2]+Q_mat_ptr[17]*y[image_width*5+2])*Q_mat_ptr[8]+(Q_mat_ptr[12]*y[3]+Q_mat_ptr[13]*y[image_width+3]+Q_mat_ptr[14]*y[image_width*2+3]+Q_mat_ptr[15]*y[image_width*3+3]+Q_mat_ptr[16]*y[image_width*4+3]+Q_mat_ptr[17]*y[image_width*5+3])*Q_mat_ptr[9]+(Q_mat_ptr[12]*y[4]+Q_mat_ptr[13]*y[image_width+4]+Q_mat_ptr[14]*y[image_width*2+4]+Q_mat_ptr[15]*y[image_width*3+4]+Q_mat_ptr[16]*y[image_width*4+4]+Q_mat_ptr[17]*y[image_width*5+4])*Q_mat_ptr[10]+(Q_mat_ptr[12]*y[5]+Q_mat_ptr[13]*y[image_width+5]+Q_mat_ptr[14]*y[image_width*2+5]+Q_mat_ptr[15]*y[image_width*3+5]+Q_mat_ptr[16]*y[image_width*4+5]+Q_mat_ptr[17]*y[image_width*5+5])*Q_mat_ptr[11];
					//lut_element_ptr[10] = (Q_mat_ptr[12]*y[0]+Q_mat_ptr[13]*y[image_width]+Q_mat_ptr[14]*y[image_width*2]+Q_mat_ptr[15]*y[image_width*3]+Q_mat_ptr[16]*y[image_width*4]+Q_mat_ptr[17]*y[image_width*5])*Q_mat_ptr[12]+(Q_mat_ptr[12]*y[1]+Q_mat_ptr[13]*y[image_width+1]+Q_mat_ptr[14]*y[image_width*2+1]+Q_mat_ptr[15]*y[image_width*3+1]+Q_mat_ptr[16]*y[image_width*4+1]+Q_mat_ptr[17]*y[image_width*5+1])*Q_mat_ptr[13]+(Q_mat_ptr[12]*y[2]+Q_mat_ptr[13]*y[image_width+2]+Q_mat_ptr[14]*y[image_width*2+2]+Q_mat_ptr[15]*y[image_width*3+2]+Q_mat_ptr[16]*y[image_width*4+2]+Q_mat_ptr[17]*y[image_width*5+2])*Q_mat_ptr[14]+(Q_mat_ptr[12]*y[3]+Q_mat_ptr[13]*y[image_width+3]+Q_mat_ptr[14]*y[image_width*2+3]+Q_mat_ptr[15]*y[image_width*3+3]+Q_mat_ptr[16]*y[image_width*4+3]+Q_mat_ptr[17]*y[image_width*5+3])*Q_mat_ptr[15]+(Q_mat_ptr[12]*y[4]+Q_mat_ptr[13]*y[image_width+4]+Q_mat_ptr[14]*y[image_width*2+4]+Q_mat_ptr[15]*y[image_width*3+4]+Q_mat_ptr[16]*y[image_width*4+4]+Q_mat_ptr[17]*y[image_width*5+4])*Q_mat_ptr[16]+(Q_mat_ptr[12]*y[5]+Q_mat_ptr[13]*y[image_width+5]+Q_mat_ptr[14]*y[image_width*2+5]+Q_mat_ptr[15]*y[image_width*3+5]+Q_mat_ptr[16]*y[image_width*4+5]+Q_mat_ptr[17]*y[image_width*5+5])*Q_mat_ptr[17];
					//lut_element_ptr[11] = (Q_mat_ptr[12]*y[0]+Q_mat_ptr[13]*y[image_width]+Q_mat_ptr[14]*y[image_width*2]+Q_mat_ptr[15]*y[image_width*3]+Q_mat_ptr[16]*y[image_width*4]+Q_mat_ptr[17]*y[image_width*5])*Q_mat_ptr[18]+(Q_mat_ptr[12]*y[1]+Q_mat_ptr[13]*y[image_width+1]+Q_mat_ptr[14]*y[image_width*2+1]+Q_mat_ptr[15]*y[image_width*3+1]+Q_mat_ptr[16]*y[image_width*4+1]+Q_mat_ptr[17]*y[image_width*5+1])*Q_mat_ptr[19]+(Q_mat_ptr[12]*y[2]+Q_mat_ptr[13]*y[image_width+2]+Q_mat_ptr[14]*y[image_width*2+2]+Q_mat_ptr[15]*y[image_width*3+2]+Q_mat_ptr[16]*y[image_width*4+2]+Q_mat_ptr[17]*y[image_width*5+2])*Q_mat_ptr[20]+(Q_mat_ptr[12]*y[3]+Q_mat_ptr[13]*y[image_width+3]+Q_mat_ptr[14]*y[image_width*2+3]+Q_mat_ptr[15]*y[image_width*3+3]+Q_mat_ptr[16]*y[image_width*4+3]+Q_mat_ptr[17]*y[image_width*5+3])*Q_mat_ptr[21]+(Q_mat_ptr[12]*y[4]+Q_mat_ptr[13]*y[image_width+4]+Q_mat_ptr[14]*y[image_width*2+4]+Q_mat_ptr[15]*y[image_width*3+4]+Q_mat_ptr[16]*y[image_width*4+4]+Q_mat_ptr[17]*y[image_width*5+4])*Q_mat_ptr[22]+(Q_mat_ptr[12]*y[5]+Q_mat_ptr[13]*y[image_width+5]+Q_mat_ptr[14]*y[image_width*2+5]+Q_mat_ptr[15]*y[image_width*3+5]+Q_mat_ptr[16]*y[image_width*4+5]+Q_mat_ptr[17]*y[image_width*5+5])*Q_mat_ptr[23]; 
					//lut_element_ptr[12] = (Q_mat_ptr[18]*y[0]+Q_mat_ptr[19]*y[image_width]+Q_mat_ptr[20]*y[image_width*2]+Q_mat_ptr[21]*y[image_width*3]+Q_mat_ptr[22]*y[image_width*4]+Q_mat_ptr[23]*y[image_width*5])*Q_mat_ptr[0]+(Q_mat_ptr[18]*y[1]+Q_mat_ptr[19]*y[image_width+1]+Q_mat_ptr[20]*y[image_width*2+1]+Q_mat_ptr[21]*y[image_width*3+1]+Q_mat_ptr[22]*y[image_width*4+1]+Q_mat_ptr[23]*y[image_width*5+1])*Q_mat_ptr[1]+(Q_mat_ptr[18]*y[2]+Q_mat_ptr[19]*y[image_width+2]+Q_mat_ptr[20]*y[image_width*2+2]+Q_mat_ptr[21]*y[image_width*3+2]+Q_mat_ptr[22]*y[image_width*4+2]+Q_mat_ptr[23]*y[image_width*5+2])*Q_mat_ptr[2]+(Q_mat_ptr[18]*y[3]+Q_mat_ptr[19]*y[image_width+3]+Q_mat_ptr[20]*y[image_width*2+3]+Q_mat_ptr[21]*y[image_width*3+3]+Q_mat_ptr[22]*y[image_width*4+3]+Q_mat_ptr[23]*y[image_width*5+3])*Q_mat_ptr[3]+(Q_mat_ptr[18]*y[4]+Q_mat_ptr[19]*y[image_width+4]+Q_mat_ptr[20]*y[image_width*2+4]+Q_mat_ptr[21]*y[image_width*3+4]+Q_mat_ptr[22]*y[image_width*4+4]+Q_mat_ptr[23]*y[image_width*5+4])*Q_mat_ptr[4]+(Q_mat_ptr[18]*y[5]+Q_mat_ptr[19]*y[image_width+5]+Q_mat_ptr[20]*y[image_width*2+5]+Q_mat_ptr[21]*y[image_width*3+5]+Q_mat_ptr[22]*y[image_width*4+5]+Q_mat_ptr[23]*y[image_width*5+5])*Q_mat_ptr[5];
					//lut_element_ptr[13] = (Q_mat_ptr[18]*y[0]+Q_mat_ptr[19]*y[image_width]+Q_mat_ptr[20]*y[image_width*2]+Q_mat_ptr[21]*y[image_width*3]+Q_mat_ptr[22]*y[image_width*4]+Q_mat_ptr[23]*y[image_width*5])*Q_mat_ptr[6]+(Q_mat_ptr[18]*y[1]+Q_mat_ptr[19]*y[image_width+1]+Q_mat_ptr[20]*y[image_width*2+1]+Q_mat_ptr[21]*y[image_width*3+1]+Q_mat_ptr[22]*y[image_width*4+1]+Q_mat_ptr[23]*y[image_width*5+1])*Q_mat_ptr[7]+(Q_mat_ptr[18]*y[2]+Q_mat_ptr[19]*y[image_width+2]+Q_mat_ptr[20]*y[image_width*2+2]+Q_mat_ptr[21]*y[image_width*3+2]+Q_mat_ptr[22]*y[image_width*4+2]+Q_mat_ptr[23]*y[image_width*5+2])*Q_mat_ptr[8]+(Q_mat_ptr[18]*y[3]+Q_mat_ptr[19]*y[image_width+3]+Q_mat_ptr[20]*y[image_width*2+3]+Q_mat_ptr[21]*y[image_width*3+3]+Q_mat_ptr[22]*y[image_width*4+3]+Q_mat_ptr[23]*y[image_width*5+3])*Q_mat_ptr[9]+(Q_mat_ptr[18]*y[4]+Q_mat_ptr[19]*y[image_width+4]+Q_mat_ptr[20]*y[image_width*2+4]+Q_mat_ptr[21]*y[image_width*3+4]+Q_mat_ptr[22]*y[image_width*4+4]+Q_mat_ptr[23]*y[image_width*5+4])*Q_mat_ptr[10]+(Q_mat_ptr[18]*y[5]+Q_mat_ptr[19]*y[image_width+5]+Q_mat_ptr[20]*y[image_width*2+5]+Q_mat_ptr[21]*y[image_width*3+5]+Q_mat_ptr[22]*y[image_width*4+5]+Q_mat_ptr[23]*y[image_width*5+5])*Q_mat_ptr[11];
					//lut_element_ptr[14]= (Q_mat_ptr[18]*y[0]+Q_mat_ptr[19]*y[image_width]+Q_mat_ptr[20]*y[image_width*2]+Q_mat_ptr[21]*y[image_width*3]+Q_mat_ptr[22]*y[image_width*4]+Q_mat_ptr[23]*y[image_width*5])*Q_mat_ptr[12]+(Q_mat_ptr[18]*y[1]+Q_mat_ptr[19]*y[image_width+1]+Q_mat_ptr[20]*y[image_width*2+1]+Q_mat_ptr[21]*y[image_width*3+1]+Q_mat_ptr[22]*y[image_width*4+1]+Q_mat_ptr[23]*y[image_width*5+1])*Q_mat_ptr[13]+(Q_mat_ptr[18]*y[2]+Q_mat_ptr[19]*y[image_width+2]+Q_mat_ptr[20]*y[image_width*2+2]+Q_mat_ptr[21]*y[image_width*3+2]+Q_mat_ptr[22]*y[image_width*4+2]+Q_mat_ptr[23]*y[image_width*5+2])*Q_mat_ptr[14]+(Q_mat_ptr[18]*y[3]+Q_mat_ptr[19]*y[image_width+3]+Q_mat_ptr[20]*y[image_width*2+3]+Q_mat_ptr[21]*y[image_width*3+3]+Q_mat_ptr[22]*y[image_width*4+3]+Q_mat_ptr[23]*y[image_width*5+3])*Q_mat_ptr[15]+(Q_mat_ptr[18]*y[4]+Q_mat_ptr[19]*y[image_width+4]+Q_mat_ptr[20]*y[image_width*2+4]+Q_mat_ptr[21]*y[image_width*3+4]+Q_mat_ptr[22]*y[image_width*4+4]+Q_mat_ptr[23]*y[image_width*5+4])*Q_mat_ptr[16]+(Q_mat_ptr[18]*y[5]+Q_mat_ptr[19]*y[image_width+5]+Q_mat_ptr[20]*y[image_width*2+5]+Q_mat_ptr[21]*y[image_width*3+5]+Q_mat_ptr[22]*y[image_width*4+5]+Q_mat_ptr[23]*y[image_width*5+5])*Q_mat_ptr[17];
					//lut_element_ptr[15] = (Q_mat_ptr[18]*y[0]+Q_mat_ptr[19]*y[image_width]+Q_mat_ptr[20]*y[image_width*2]+Q_mat_ptr[21]*y[image_width*3]+Q_mat_ptr[22]*y[image_width*4]+Q_mat_ptr[23]*y[image_width*5])*Q_mat_ptr[18]+(Q_mat_ptr[18]*y[1]+Q_mat_ptr[19]*y[image_width+1]+Q_mat_ptr[20]*y[image_width*2+1]+Q_mat_ptr[21]*y[image_width*3+1]+Q_mat_ptr[22]*y[image_width*4+1]+Q_mat_ptr[23]*y[image_width*5+1])*Q_mat_ptr[19]+(Q_mat_ptr[18]*y[2]+Q_mat_ptr[19]*y[image_width+2]+Q_mat_ptr[20]*y[image_width*2+2]+Q_mat_ptr[21]*y[image_width*3+2]+Q_mat_ptr[22]*y[image_width*4+2]+Q_mat_ptr[23]*y[image_width*5+2])*Q_mat_ptr[20]+(Q_mat_ptr[18]*y[3]+Q_mat_ptr[19]*y[image_width+3]+Q_mat_ptr[20]*y[image_width*2+3]+Q_mat_ptr[21]*y[image_width*3+3]+Q_mat_ptr[22]*y[image_width*4+3]+Q_mat_ptr[23]*y[image_width*5+3])*Q_mat_ptr[21]+(Q_mat_ptr[18]*y[4]+Q_mat_ptr[19]*y[image_width+4]+Q_mat_ptr[20]*y[image_width*2+4]+Q_mat_ptr[21]*y[image_width*3+4]+Q_mat_ptr[22]*y[image_width*4+4]+Q_mat_ptr[23]*y[image_width*5+4])*Q_mat_ptr[22]+(Q_mat_ptr[18]*y[5]+Q_mat_ptr[19]*y[image_width+5]+Q_mat_ptr[20]*y[image_width*2+5]+Q_mat_ptr[21]*y[image_width*3+5]+Q_mat_ptr[22]*y[image_width*4+5]+Q_mat_ptr[23]*y[image_width*5+5])*Q_mat_ptr[23];
				}
			}
		}
		else 
		{
			for (int i = 0;i<aoi_height;i++)
			{
				for (int j=0;j<aoi_width;j++)
				{
					float* lut_element_ptr = lut_mat->data.fl+(i*aoi_width+j)*UniformCubicNatureSplineInterpolationMethod::lut_element_size_;
					int index_i_min = aoi_y_min+i -2;
					int index_j_min = aoi_x_min+j-2;

					double* y = image->data.db + index_i_min*image_width+index_j_min;
					lut_element_ptr[0] = (Q_mat_ptr[0] * y[0] + Q_mat_ptr[1] * y[image_width] + Q_mat_ptr[2] * y[2 * image_width] + Q_mat_ptr[3] * y[3 * image_width] + Q_mat_ptr[4] * y[4 * image_width] + Q_mat_ptr[5] * y[5 * image_width]) * Q_mat_ptr[0] + (Q_mat_ptr[0] * y[1] + Q_mat_ptr[1] * y[1 + image_width] + Q_mat_ptr[2] * y[1 + 2 * image_width] + Q_mat_ptr[3] * y[1 + 3 * image_width] + Q_mat_ptr[4] * y[1 + 4 * image_width] + Q_mat_ptr[5] * y[1 + 5 * image_width]) * Q_mat_ptr[1] + (Q_mat_ptr[0] * y[2] + Q_mat_ptr[1] * y[2 + image_width] + Q_mat_ptr[2] * y[2 + 2 * image_width] + Q_mat_ptr[3] * y[2 + 3 * image_width] + Q_mat_ptr[4] * y[2 + 4 * image_width] + Q_mat_ptr[5] * y[2 + 5 * image_width]) * Q_mat_ptr[2] + (Q_mat_ptr[0] * y[3] + Q_mat_ptr[1] * y[3 + image_width] + Q_mat_ptr[2] * y[3 + 2 * image_width] + Q_mat_ptr[3] * y[3 + 3 * image_width] + Q_mat_ptr[4] * y[3 + 4 * image_width] + Q_mat_ptr[5] * y[3 + 5 * image_width]) * Q_mat_ptr[3] + (Q_mat_ptr[0] * y[4] + Q_mat_ptr[1] * y[4 + image_width] + Q_mat_ptr[2] * y[4 + 2 * image_width] + Q_mat_ptr[3] * y[4 + 3 * image_width] + Q_mat_ptr[4] * y[4 + 4 * image_width] + Q_mat_ptr[5] * y[4 + 5 * image_width]) * Q_mat_ptr[4] + (Q_mat_ptr[0] * y[5] + Q_mat_ptr[1] * y[5 + image_width] + Q_mat_ptr[2] * y[5 + 2 * image_width] + Q_mat_ptr[3] * y[5 + 3 * image_width] + Q_mat_ptr[4] * y[5 + 4 * image_width] + Q_mat_ptr[5] * y[5 + 5 * image_width]) * Q_mat_ptr[5];
					lut_element_ptr[1] = (Q_mat_ptr[0] * y[0] + Q_mat_ptr[1] * y[image_width] + Q_mat_ptr[2] * y[2 * image_width] + Q_mat_ptr[3] * y[3 * image_width] + Q_mat_ptr[4] * y[4 * image_width] + Q_mat_ptr[5] * y[5 * image_width]) * Q_mat_ptr[6] + (Q_mat_ptr[0] * y[1] + Q_mat_ptr[1] * y[1 + image_width] + Q_mat_ptr[2] * y[1 + 2 * image_width] + Q_mat_ptr[3] * y[1 + 3 * image_width] + Q_mat_ptr[4] * y[1 + 4 * image_width] + Q_mat_ptr[5] * y[1 + 5 * image_width]) * Q_mat_ptr[7] + (Q_mat_ptr[0] * y[2] + Q_mat_ptr[1] * y[2 + image_width] + Q_mat_ptr[2] * y[2 + 2 * image_width] + Q_mat_ptr[3] * y[2 + 3 * image_width] + Q_mat_ptr[4] * y[2 + 4 * image_width] + Q_mat_ptr[5] * y[2 + 5 * image_width]) * Q_mat_ptr[8] + (Q_mat_ptr[0] * y[3] + Q_mat_ptr[1] * y[3 + image_width] + Q_mat_ptr[2] * y[3 + 2 * image_width] + Q_mat_ptr[3] * y[3 + 3 * image_width] + Q_mat_ptr[4] * y[3 + 4 * image_width] + Q_mat_ptr[5] * y[3 + 5 * image_width]) * Q_mat_ptr[9] + (Q_mat_ptr[0] * y[4] + Q_mat_ptr[1] * y[4 + image_width] + Q_mat_ptr[2] * y[4 + 2 * image_width] + Q_mat_ptr[3] * y[4 + 3 * image_width] + Q_mat_ptr[4] * y[4 + 4 * image_width] + Q_mat_ptr[5] * y[4 + 5 * image_width]) * Q_mat_ptr[10] + (Q_mat_ptr[0] * y[5] + Q_mat_ptr[1] * y[5 + image_width] + Q_mat_ptr[2] * y[5 + 2 * image_width] + Q_mat_ptr[3] * y[5 + 3 * image_width] + Q_mat_ptr[4] * y[5 + 4 * image_width] + Q_mat_ptr[5] * y[5 + 5 * image_width]) * Q_mat_ptr[11];
					lut_element_ptr[2] = (Q_mat_ptr[0] * y[0] + Q_mat_ptr[1] * y[image_width] + Q_mat_ptr[2] * y[2 * image_width] + Q_mat_ptr[3] * y[3 * image_width] + Q_mat_ptr[4] * y[4 * image_width] + Q_mat_ptr[5] * y[5 * image_width]) * Q_mat_ptr[12] + (Q_mat_ptr[0] * y[1] + Q_mat_ptr[1] * y[1 + image_width] + Q_mat_ptr[2] * y[1 + 2 * image_width] + Q_mat_ptr[3] * y[1 + 3 * image_width] + Q_mat_ptr[4] * y[1 + 4 * image_width] + Q_mat_ptr[5] * y[1 + 5 * image_width]) * Q_mat_ptr[13] + (Q_mat_ptr[0] * y[2] + Q_mat_ptr[1] * y[2 + image_width] + Q_mat_ptr[2] * y[2 + 2 * image_width] + Q_mat_ptr[3] * y[2 + 3 * image_width] + Q_mat_ptr[4] * y[2 + 4 * image_width] + Q_mat_ptr[5] * y[2 + 5 * image_width]) * Q_mat_ptr[14] + (Q_mat_ptr[0] * y[3] + Q_mat_ptr[1] * y[3 + image_width] + Q_mat_ptr[2] * y[3 + 2 * image_width] + Q_mat_ptr[3] * y[3 + 3 * image_width] + Q_mat_ptr[4] * y[3 + 4 * image_width] + Q_mat_ptr[5] * y[3 + 5 * image_width]) * Q_mat_ptr[15] + (Q_mat_ptr[0] * y[4] + Q_mat_ptr[1] * y[4 + image_width] + Q_mat_ptr[2] * y[4 + 2 * image_width] + Q_mat_ptr[3] * y[4 + 3 * image_width] + Q_mat_ptr[4] * y[4 + 4 * image_width] + Q_mat_ptr[5] * y[4 + 5 * image_width]) * Q_mat_ptr[16] + (Q_mat_ptr[0] * y[5] + Q_mat_ptr[1] * y[5 + image_width] + Q_mat_ptr[2] * y[5 + 2 * image_width] + Q_mat_ptr[3] * y[5 + 3 * image_width] + Q_mat_ptr[4] * y[5 + 4 * image_width] + Q_mat_ptr[5] * y[5 + 5 * image_width]) * Q_mat_ptr[17];
					lut_element_ptr[3] = Q_mat_ptr[0] * y[2] + Q_mat_ptr[1] * y[2 + image_width] + Q_mat_ptr[2] * y[2 + 2 * image_width] + Q_mat_ptr[3] * y[2 + 3 * image_width] + Q_mat_ptr[4] * y[2 + 4 * image_width] + Q_mat_ptr[5] * y[2 + 5 * image_width];
					lut_element_ptr[4] = (Q_mat_ptr[6] * y[0] + Q_mat_ptr[7] * y[image_width] + Q_mat_ptr[8] * y[2 * image_width] + Q_mat_ptr[9] * y[3 * image_width] + Q_mat_ptr[10] * y[4 * image_width] + Q_mat_ptr[11] * y[5 * image_width]) * Q_mat_ptr[0] + (Q_mat_ptr[6] * y[1] + Q_mat_ptr[7] * y[1 + image_width] + Q_mat_ptr[8] * y[1 + 2 * image_width] + Q_mat_ptr[9] * y[1 + 3 * image_width] + Q_mat_ptr[10] * y[1 + 4 * image_width] + Q_mat_ptr[11] * y[1 + 5 * image_width]) * Q_mat_ptr[1] + (Q_mat_ptr[6] * y[2] + Q_mat_ptr[7] * y[2 + image_width] + Q_mat_ptr[8] * y[2 + 2 * image_width] + Q_mat_ptr[9] * y[2 + 3 * image_width] + Q_mat_ptr[10] * y[2 + 4 * image_width] + Q_mat_ptr[11] * y[2 + 5 * image_width]) * Q_mat_ptr[2] + (Q_mat_ptr[6] * y[3] + Q_mat_ptr[7] * y[3 + image_width] + Q_mat_ptr[8] * y[3 + 2 * image_width] + Q_mat_ptr[9] * y[3 + 3 * image_width] + Q_mat_ptr[10] * y[3 + 4 * image_width] + Q_mat_ptr[11] * y[3 + 5 * image_width]) * Q_mat_ptr[3] + (Q_mat_ptr[6] * y[4] + Q_mat_ptr[7] * y[4 + image_width] + Q_mat_ptr[8] * y[4 + 2 * image_width] + Q_mat_ptr[9] * y[4 + 3 * image_width] + Q_mat_ptr[10] * y[4 + 4 * image_width] + Q_mat_ptr[11] * y[4 + 5 * image_width]) * Q_mat_ptr[4] + (Q_mat_ptr[6] * y[5] + Q_mat_ptr[7] * y[5 + image_width] + Q_mat_ptr[8] * y[5 + 2 * image_width] + Q_mat_ptr[9] * y[5 + 3 * image_width] + Q_mat_ptr[10] * y[5 + 4 * image_width] + Q_mat_ptr[11] * y[5 + 5 * image_width]) * Q_mat_ptr[5];
					lut_element_ptr[5] = (Q_mat_ptr[6] * y[0] + Q_mat_ptr[7] * y[image_width] + Q_mat_ptr[8] * y[2 * image_width] + Q_mat_ptr[9] * y[3 * image_width] + Q_mat_ptr[10] * y[4 * image_width] + Q_mat_ptr[11] * y[5 * image_width]) * Q_mat_ptr[6] + (Q_mat_ptr[6] * y[1] + Q_mat_ptr[7] * y[1 + image_width] + Q_mat_ptr[8] * y[1 + 2 * image_width] + Q_mat_ptr[9] * y[1 + 3 * image_width] + Q_mat_ptr[10] * y[1 + 4 * image_width] + Q_mat_ptr[11] * y[1 + 5 * image_width]) * Q_mat_ptr[7] + (Q_mat_ptr[6] * y[2] + Q_mat_ptr[7] * y[2 + image_width] + Q_mat_ptr[8] * y[2 + 2 * image_width] + Q_mat_ptr[9] * y[2 + 3 * image_width] + Q_mat_ptr[10] * y[2 + 4 * image_width] + Q_mat_ptr[11] * y[2 + 5 * image_width]) * Q_mat_ptr[8] + (Q_mat_ptr[6] * y[3] + Q_mat_ptr[7] * y[3 + image_width] + Q_mat_ptr[8] * y[3 + 2 * image_width] + Q_mat_ptr[9] * y[3 + 3 * image_width] + Q_mat_ptr[10] * y[3 + 4 * image_width] + Q_mat_ptr[11] * y[3 + 5 * image_width]) * Q_mat_ptr[9] + (Q_mat_ptr[6] * y[4] + Q_mat_ptr[7] * y[4 + image_width] + Q_mat_ptr[8] * y[4 + 2 * image_width] + Q_mat_ptr[9] * y[4 + 3 * image_width] + Q_mat_ptr[10] * y[4 + 4 * image_width] + Q_mat_ptr[11] * y[4 + 5 * image_width]) * Q_mat_ptr[10] + (Q_mat_ptr[6] * y[5] + Q_mat_ptr[7] * y[5 + image_width] + Q_mat_ptr[8] * y[5 + 2 * image_width] + Q_mat_ptr[9] * y[5 + 3 * image_width] + Q_mat_ptr[10] * y[5 + 4 * image_width] + Q_mat_ptr[11] * y[5 + 5 * image_width]) * Q_mat_ptr[11];
					lut_element_ptr[6] = (Q_mat_ptr[6] * y[0] + Q_mat_ptr[7] * y[image_width] + Q_mat_ptr[8] * y[2 * image_width] + Q_mat_ptr[9] * y[3 * image_width] + Q_mat_ptr[10] * y[4 * image_width] + Q_mat_ptr[11] * y[5 * image_width]) * Q_mat_ptr[12] + (Q_mat_ptr[6] * y[1] + Q_mat_ptr[7] * y[1 + image_width] + Q_mat_ptr[8] * y[1 + 2 * image_width] + Q_mat_ptr[9] * y[1 + 3 * image_width] + Q_mat_ptr[10] * y[1 + 4 * image_width] + Q_mat_ptr[11] * y[1 + 5 * image_width]) * Q_mat_ptr[13] + (Q_mat_ptr[6] * y[2] + Q_mat_ptr[7] * y[2 + image_width] + Q_mat_ptr[8] * y[2 + 2 * image_width] + Q_mat_ptr[9] * y[2 + 3 * image_width] + Q_mat_ptr[10] * y[2 + 4 * image_width] + Q_mat_ptr[11] * y[2 + 5 * image_width]) * Q_mat_ptr[14] + (Q_mat_ptr[6] * y[3] + Q_mat_ptr[7] * y[3 + image_width] + Q_mat_ptr[8] * y[3 + 2 * image_width] + Q_mat_ptr[9] * y[3 + 3 * image_width] + Q_mat_ptr[10] * y[3 + 4 * image_width] + Q_mat_ptr[11] * y[3 + 5 * image_width]) * Q_mat_ptr[15] + (Q_mat_ptr[6] * y[4] + Q_mat_ptr[7] * y[4 + image_width] + Q_mat_ptr[8] * y[4 + 2 * image_width] + Q_mat_ptr[9] * y[4 + 3 * image_width] + Q_mat_ptr[10] * y[4 + 4 * image_width] + Q_mat_ptr[11] * y[4 + 5 * image_width]) * Q_mat_ptr[16] + (Q_mat_ptr[6] * y[5] + Q_mat_ptr[7] * y[5 + image_width] + Q_mat_ptr[8] * y[5 + 2 * image_width] + Q_mat_ptr[9] * y[5 + 3 * image_width] + Q_mat_ptr[10] * y[5 + 4 * image_width] + Q_mat_ptr[11] * y[5 + 5 * image_width]) * Q_mat_ptr[17];
					lut_element_ptr[7] = Q_mat_ptr[6] * y[2] + Q_mat_ptr[7] * y[2 + image_width] + Q_mat_ptr[8] * y[2 + 2 * image_width] + Q_mat_ptr[9] * y[2 + 3 * image_width] + Q_mat_ptr[10] * y[2 + 4 * image_width] + Q_mat_ptr[11] * y[2 + 5 * image_width];
					lut_element_ptr[8] = (Q_mat_ptr[12] * y[0] + Q_mat_ptr[13] * y[image_width] + Q_mat_ptr[14] * y[2 * image_width] + Q_mat_ptr[15] * y[3 * image_width] + Q_mat_ptr[16] * y[4 * image_width] + Q_mat_ptr[17] * y[5 * image_width]) * Q_mat_ptr[0] + (Q_mat_ptr[12] * y[1] + Q_mat_ptr[13] * y[1 + image_width] + Q_mat_ptr[14] * y[1 + 2 * image_width] + Q_mat_ptr[15] * y[1 + 3 * image_width] + Q_mat_ptr[16] * y[1 + 4 * image_width] + Q_mat_ptr[17] * y[1 + 5 * image_width]) * Q_mat_ptr[1] + (Q_mat_ptr[12] * y[2] + Q_mat_ptr[13] * y[2 + image_width] + Q_mat_ptr[14] * y[2 + 2 * image_width] + Q_mat_ptr[15] * y[2 + 3 * image_width] + Q_mat_ptr[16] * y[2 + 4 * image_width] + Q_mat_ptr[17] * y[2 + 5 * image_width]) * Q_mat_ptr[2] + (Q_mat_ptr[12] * y[3] + Q_mat_ptr[13] * y[3 + image_width] + Q_mat_ptr[14] * y[3 + 2 * image_width] + Q_mat_ptr[15] * y[3 + 3 * image_width] + Q_mat_ptr[16] * y[3 + 4 * image_width] + Q_mat_ptr[17] * y[3 + 5 * image_width]) * Q_mat_ptr[3] + (Q_mat_ptr[12] * y[4] + Q_mat_ptr[13] * y[4 + image_width] + Q_mat_ptr[14] * y[4 + 2 * image_width] + Q_mat_ptr[15] * y[4 + 3 * image_width] + Q_mat_ptr[16] * y[4 + 4 * image_width] + Q_mat_ptr[17] * y[4 + 5 * image_width]) * Q_mat_ptr[4] + (Q_mat_ptr[12] * y[5] + Q_mat_ptr[13] * y[5 + image_width] + Q_mat_ptr[14] * y[5 + 2 * image_width] + Q_mat_ptr[15] * y[5 + 3 * image_width] + Q_mat_ptr[16] * y[5 + 4 * image_width] + Q_mat_ptr[17] * y[5 + 5 * image_width]) * Q_mat_ptr[5];
					lut_element_ptr[9] = (Q_mat_ptr[12] * y[0] + Q_mat_ptr[13] * y[image_width] + Q_mat_ptr[14] * y[2 * image_width] + Q_mat_ptr[15] * y[3 * image_width] + Q_mat_ptr[16] * y[4 * image_width] + Q_mat_ptr[17] * y[5 * image_width]) * Q_mat_ptr[6] + (Q_mat_ptr[12] * y[1] + Q_mat_ptr[13] * y[1 + image_width] + Q_mat_ptr[14] * y[1 + 2 * image_width] + Q_mat_ptr[15] * y[1 + 3 * image_width] + Q_mat_ptr[16] * y[1 + 4 * image_width] + Q_mat_ptr[17] * y[1 + 5 * image_width]) * Q_mat_ptr[7] + (Q_mat_ptr[12] * y[2] + Q_mat_ptr[13] * y[2 + image_width] + Q_mat_ptr[14] * y[2 + 2 * image_width] + Q_mat_ptr[15] * y[2 + 3 * image_width] + Q_mat_ptr[16] * y[2 + 4 * image_width] + Q_mat_ptr[17] * y[2 + 5 * image_width]) * Q_mat_ptr[8] + (Q_mat_ptr[12] * y[3] + Q_mat_ptr[13] * y[3 + image_width] + Q_mat_ptr[14] * y[3 + 2 * image_width] + Q_mat_ptr[15] * y[3 + 3 * image_width] + Q_mat_ptr[16] * y[3 + 4 * image_width] + Q_mat_ptr[17] * y[3 + 5 * image_width]) * Q_mat_ptr[9] + (Q_mat_ptr[12] * y[4] + Q_mat_ptr[13] * y[4 + image_width] + Q_mat_ptr[14] * y[4 + 2 * image_width] + Q_mat_ptr[15] * y[4 + 3 * image_width] + Q_mat_ptr[16] * y[4 + 4 * image_width] + Q_mat_ptr[17] * y[4 + 5 * image_width]) * Q_mat_ptr[10] + (Q_mat_ptr[12] * y[5] + Q_mat_ptr[13] * y[5 + image_width] + Q_mat_ptr[14] * y[5 + 2 * image_width] + Q_mat_ptr[15] * y[5 + 3 * image_width] + Q_mat_ptr[16] * y[5 + 4 * image_width] + Q_mat_ptr[17] * y[5 + 5 * image_width]) * Q_mat_ptr[11];
					lut_element_ptr[10] = (Q_mat_ptr[12] * y[0] + Q_mat_ptr[13] * y[image_width] + Q_mat_ptr[14] * y[2 * image_width] + Q_mat_ptr[15] * y[3 * image_width] + Q_mat_ptr[16] * y[4 * image_width] + Q_mat_ptr[17] * y[5 * image_width]) * Q_mat_ptr[12] + (Q_mat_ptr[12] * y[1] + Q_mat_ptr[13] * y[1 + image_width] + Q_mat_ptr[14] * y[1 + 2 * image_width] + Q_mat_ptr[15] * y[1 + 3 * image_width] + Q_mat_ptr[16] * y[1 + 4 * image_width] + Q_mat_ptr[17] * y[1 + 5 * image_width]) * Q_mat_ptr[13] + (Q_mat_ptr[12] * y[2] + Q_mat_ptr[13] * y[2 + image_width] + Q_mat_ptr[14] * y[2 + 2 * image_width] + Q_mat_ptr[15] * y[2 + 3 * image_width] + Q_mat_ptr[16] * y[2 + 4 * image_width] + Q_mat_ptr[17] * y[2 + 5 * image_width]) * Q_mat_ptr[14] + (Q_mat_ptr[12] * y[3] + Q_mat_ptr[13] * y[3 + image_width] + Q_mat_ptr[14] * y[3 + 2 * image_width] + Q_mat_ptr[15] * y[3 + 3 * image_width] + Q_mat_ptr[16] * y[3 + 4 * image_width] + Q_mat_ptr[17] * y[3 + 5 * image_width]) * Q_mat_ptr[15] + (Q_mat_ptr[12] * y[4] + Q_mat_ptr[13] * y[4 + image_width] + Q_mat_ptr[14] * y[4 + 2 * image_width] + Q_mat_ptr[15] * y[4 + 3 * image_width] + Q_mat_ptr[16] * y[4 + 4 * image_width] + Q_mat_ptr[17] * y[4 + 5 * image_width]) * Q_mat_ptr[16] + (Q_mat_ptr[12] * y[5] + Q_mat_ptr[13] * y[5 + image_width] + Q_mat_ptr[14] * y[5 + 2 * image_width] + Q_mat_ptr[15] * y[5 + 3 * image_width] + Q_mat_ptr[16] * y[5 + 4 * image_width] + Q_mat_ptr[17] * y[5 + 5 * image_width]) * Q_mat_ptr[17];
					lut_element_ptr[11] = Q_mat_ptr[12] * y[2] + Q_mat_ptr[13] * y[2 + image_width] + Q_mat_ptr[14] * y[2 + 2 * image_width] + Q_mat_ptr[15] * y[2 + 3 * image_width] + Q_mat_ptr[16] * y[2 + 4 * image_width] + Q_mat_ptr[17] * y[2 + 5 * image_width];
					lut_element_ptr[12] = y[2 * image_width] * Q_mat_ptr[0] + y[1 + 2 * image_width] * Q_mat_ptr[1] + Q_mat_ptr[2] * y[2 + 2 * image_width] + y[3 + 2 * image_width] * Q_mat_ptr[3] + y[4 + 2 * image_width] * Q_mat_ptr[4] + y[5 + 2 * image_width] * Q_mat_ptr[5];
					lut_element_ptr[13] = y[2 * image_width] * Q_mat_ptr[6] + y[1 + 2 * image_width] * Q_mat_ptr[7] + Q_mat_ptr[8] * y[2 + 2 * image_width] + y[3 + 2 * image_width] * Q_mat_ptr[9] + y[4 + 2 * image_width] * Q_mat_ptr[10] + y[5 + 2 * image_width] * Q_mat_ptr[11];
					lut_element_ptr[14] = y[2 * image_width] * Q_mat_ptr[12] + y[1 + 2 * image_width] * Q_mat_ptr[13] + Q_mat_ptr[14] * y[2 + 2 * image_width] + y[3 + 2 * image_width] * Q_mat_ptr[15] + y[4 + 2 * image_width] * Q_mat_ptr[16] + y[5 + 2 * image_width] * Q_mat_ptr[17];
					lut_element_ptr[15] = y[2 + 2 * image_width];
				}
			}
		}
	}
	else
	{
		if (image_type==1)
		{
			for (int i = 0;i<aoi_height;i++)
			{
				for (int j=0;j<aoi_width;j++)
				{
					double* lut_element_ptr = lut_mat->data.db+(i*aoi_width+j)*UniformCubicNatureSplineInterpolationMethod::lut_element_size_;
					int index_i_min = aoi_y_min+i -2;
					int index_j_min = aoi_x_min+j-2;
					uchar* y = image->data.ptr + index_i_min*image_width+index_j_min;
					lut_element_ptr[0] = (Q_mat_ptr[0] * y[0] + Q_mat_ptr[1] * y[image_width] + Q_mat_ptr[2] * y[2 * image_width] + Q_mat_ptr[3] * y[3 * image_width] + Q_mat_ptr[4] * y[4 * image_width] + Q_mat_ptr[5] * y[5 * image_width]) * Q_mat_ptr[0] + (Q_mat_ptr[0] * y[1] + Q_mat_ptr[1] * y[1 + image_width] + Q_mat_ptr[2] * y[1 + 2 * image_width] + Q_mat_ptr[3] * y[1 + 3 * image_width] + Q_mat_ptr[4] * y[1 + 4 * image_width] + Q_mat_ptr[5] * y[1 + 5 * image_width]) * Q_mat_ptr[1] + (Q_mat_ptr[0] * y[2] + Q_mat_ptr[1] * y[2 + image_width] + Q_mat_ptr[2] * y[2 + 2 * image_width] + Q_mat_ptr[3] * y[2 + 3 * image_width] + Q_mat_ptr[4] * y[2 + 4 * image_width] + Q_mat_ptr[5] * y[2 + 5 * image_width]) * Q_mat_ptr[2] + (Q_mat_ptr[0] * y[3] + Q_mat_ptr[1] * y[3 + image_width] + Q_mat_ptr[2] * y[3 + 2 * image_width] + Q_mat_ptr[3] * y[3 + 3 * image_width] + Q_mat_ptr[4] * y[3 + 4 * image_width] + Q_mat_ptr[5] * y[3 + 5 * image_width]) * Q_mat_ptr[3] + (Q_mat_ptr[0] * y[4] + Q_mat_ptr[1] * y[4 + image_width] + Q_mat_ptr[2] * y[4 + 2 * image_width] + Q_mat_ptr[3] * y[4 + 3 * image_width] + Q_mat_ptr[4] * y[4 + 4 * image_width] + Q_mat_ptr[5] * y[4 + 5 * image_width]) * Q_mat_ptr[4] + (Q_mat_ptr[0] * y[5] + Q_mat_ptr[1] * y[5 + image_width] + Q_mat_ptr[2] * y[5 + 2 * image_width] + Q_mat_ptr[3] * y[5 + 3 * image_width] + Q_mat_ptr[4] * y[5 + 4 * image_width] + Q_mat_ptr[5] * y[5 + 5 * image_width]) * Q_mat_ptr[5];
					lut_element_ptr[1] = (Q_mat_ptr[0] * y[0] + Q_mat_ptr[1] * y[image_width] + Q_mat_ptr[2] * y[2 * image_width] + Q_mat_ptr[3] * y[3 * image_width] + Q_mat_ptr[4] * y[4 * image_width] + Q_mat_ptr[5] * y[5 * image_width]) * Q_mat_ptr[6] + (Q_mat_ptr[0] * y[1] + Q_mat_ptr[1] * y[1 + image_width] + Q_mat_ptr[2] * y[1 + 2 * image_width] + Q_mat_ptr[3] * y[1 + 3 * image_width] + Q_mat_ptr[4] * y[1 + 4 * image_width] + Q_mat_ptr[5] * y[1 + 5 * image_width]) * Q_mat_ptr[7] + (Q_mat_ptr[0] * y[2] + Q_mat_ptr[1] * y[2 + image_width] + Q_mat_ptr[2] * y[2 + 2 * image_width] + Q_mat_ptr[3] * y[2 + 3 * image_width] + Q_mat_ptr[4] * y[2 + 4 * image_width] + Q_mat_ptr[5] * y[2 + 5 * image_width]) * Q_mat_ptr[8] + (Q_mat_ptr[0] * y[3] + Q_mat_ptr[1] * y[3 + image_width] + Q_mat_ptr[2] * y[3 + 2 * image_width] + Q_mat_ptr[3] * y[3 + 3 * image_width] + Q_mat_ptr[4] * y[3 + 4 * image_width] + Q_mat_ptr[5] * y[3 + 5 * image_width]) * Q_mat_ptr[9] + (Q_mat_ptr[0] * y[4] + Q_mat_ptr[1] * y[4 + image_width] + Q_mat_ptr[2] * y[4 + 2 * image_width] + Q_mat_ptr[3] * y[4 + 3 * image_width] + Q_mat_ptr[4] * y[4 + 4 * image_width] + Q_mat_ptr[5] * y[4 + 5 * image_width]) * Q_mat_ptr[10] + (Q_mat_ptr[0] * y[5] + Q_mat_ptr[1] * y[5 + image_width] + Q_mat_ptr[2] * y[5 + 2 * image_width] + Q_mat_ptr[3] * y[5 + 3 * image_width] + Q_mat_ptr[4] * y[5 + 4 * image_width] + Q_mat_ptr[5] * y[5 + 5 * image_width]) * Q_mat_ptr[11];
					lut_element_ptr[2] = (Q_mat_ptr[0] * y[0] + Q_mat_ptr[1] * y[image_width] + Q_mat_ptr[2] * y[2 * image_width] + Q_mat_ptr[3] * y[3 * image_width] + Q_mat_ptr[4] * y[4 * image_width] + Q_mat_ptr[5] * y[5 * image_width]) * Q_mat_ptr[12] + (Q_mat_ptr[0] * y[1] + Q_mat_ptr[1] * y[1 + image_width] + Q_mat_ptr[2] * y[1 + 2 * image_width] + Q_mat_ptr[3] * y[1 + 3 * image_width] + Q_mat_ptr[4] * y[1 + 4 * image_width] + Q_mat_ptr[5] * y[1 + 5 * image_width]) * Q_mat_ptr[13] + (Q_mat_ptr[0] * y[2] + Q_mat_ptr[1] * y[2 + image_width] + Q_mat_ptr[2] * y[2 + 2 * image_width] + Q_mat_ptr[3] * y[2 + 3 * image_width] + Q_mat_ptr[4] * y[2 + 4 * image_width] + Q_mat_ptr[5] * y[2 + 5 * image_width]) * Q_mat_ptr[14] + (Q_mat_ptr[0] * y[3] + Q_mat_ptr[1] * y[3 + image_width] + Q_mat_ptr[2] * y[3 + 2 * image_width] + Q_mat_ptr[3] * y[3 + 3 * image_width] + Q_mat_ptr[4] * y[3 + 4 * image_width] + Q_mat_ptr[5] * y[3 + 5 * image_width]) * Q_mat_ptr[15] + (Q_mat_ptr[0] * y[4] + Q_mat_ptr[1] * y[4 + image_width] + Q_mat_ptr[2] * y[4 + 2 * image_width] + Q_mat_ptr[3] * y[4 + 3 * image_width] + Q_mat_ptr[4] * y[4 + 4 * image_width] + Q_mat_ptr[5] * y[4 + 5 * image_width]) * Q_mat_ptr[16] + (Q_mat_ptr[0] * y[5] + Q_mat_ptr[1] * y[5 + image_width] + Q_mat_ptr[2] * y[5 + 2 * image_width] + Q_mat_ptr[3] * y[5 + 3 * image_width] + Q_mat_ptr[4] * y[5 + 4 * image_width] + Q_mat_ptr[5] * y[5 + 5 * image_width]) * Q_mat_ptr[17];
					lut_element_ptr[3] = Q_mat_ptr[0] * y[2] + Q_mat_ptr[1] * y[2 + image_width] + Q_mat_ptr[2] * y[2 + 2 * image_width] + Q_mat_ptr[3] * y[2 + 3 * image_width] + Q_mat_ptr[4] * y[2 + 4 * image_width] + Q_mat_ptr[5] * y[2 + 5 * image_width];
					lut_element_ptr[4] = (Q_mat_ptr[6] * y[0] + Q_mat_ptr[7] * y[image_width] + Q_mat_ptr[8] * y[2 * image_width] + Q_mat_ptr[9] * y[3 * image_width] + Q_mat_ptr[10] * y[4 * image_width] + Q_mat_ptr[11] * y[5 * image_width]) * Q_mat_ptr[0] + (Q_mat_ptr[6] * y[1] + Q_mat_ptr[7] * y[1 + image_width] + Q_mat_ptr[8] * y[1 + 2 * image_width] + Q_mat_ptr[9] * y[1 + 3 * image_width] + Q_mat_ptr[10] * y[1 + 4 * image_width] + Q_mat_ptr[11] * y[1 + 5 * image_width]) * Q_mat_ptr[1] + (Q_mat_ptr[6] * y[2] + Q_mat_ptr[7] * y[2 + image_width] + Q_mat_ptr[8] * y[2 + 2 * image_width] + Q_mat_ptr[9] * y[2 + 3 * image_width] + Q_mat_ptr[10] * y[2 + 4 * image_width] + Q_mat_ptr[11] * y[2 + 5 * image_width]) * Q_mat_ptr[2] + (Q_mat_ptr[6] * y[3] + Q_mat_ptr[7] * y[3 + image_width] + Q_mat_ptr[8] * y[3 + 2 * image_width] + Q_mat_ptr[9] * y[3 + 3 * image_width] + Q_mat_ptr[10] * y[3 + 4 * image_width] + Q_mat_ptr[11] * y[3 + 5 * image_width]) * Q_mat_ptr[3] + (Q_mat_ptr[6] * y[4] + Q_mat_ptr[7] * y[4 + image_width] + Q_mat_ptr[8] * y[4 + 2 * image_width] + Q_mat_ptr[9] * y[4 + 3 * image_width] + Q_mat_ptr[10] * y[4 + 4 * image_width] + Q_mat_ptr[11] * y[4 + 5 * image_width]) * Q_mat_ptr[4] + (Q_mat_ptr[6] * y[5] + Q_mat_ptr[7] * y[5 + image_width] + Q_mat_ptr[8] * y[5 + 2 * image_width] + Q_mat_ptr[9] * y[5 + 3 * image_width] + Q_mat_ptr[10] * y[5 + 4 * image_width] + Q_mat_ptr[11] * y[5 + 5 * image_width]) * Q_mat_ptr[5];
					lut_element_ptr[5] = (Q_mat_ptr[6] * y[0] + Q_mat_ptr[7] * y[image_width] + Q_mat_ptr[8] * y[2 * image_width] + Q_mat_ptr[9] * y[3 * image_width] + Q_mat_ptr[10] * y[4 * image_width] + Q_mat_ptr[11] * y[5 * image_width]) * Q_mat_ptr[6] + (Q_mat_ptr[6] * y[1] + Q_mat_ptr[7] * y[1 + image_width] + Q_mat_ptr[8] * y[1 + 2 * image_width] + Q_mat_ptr[9] * y[1 + 3 * image_width] + Q_mat_ptr[10] * y[1 + 4 * image_width] + Q_mat_ptr[11] * y[1 + 5 * image_width]) * Q_mat_ptr[7] + (Q_mat_ptr[6] * y[2] + Q_mat_ptr[7] * y[2 + image_width] + Q_mat_ptr[8] * y[2 + 2 * image_width] + Q_mat_ptr[9] * y[2 + 3 * image_width] + Q_mat_ptr[10] * y[2 + 4 * image_width] + Q_mat_ptr[11] * y[2 + 5 * image_width]) * Q_mat_ptr[8] + (Q_mat_ptr[6] * y[3] + Q_mat_ptr[7] * y[3 + image_width] + Q_mat_ptr[8] * y[3 + 2 * image_width] + Q_mat_ptr[9] * y[3 + 3 * image_width] + Q_mat_ptr[10] * y[3 + 4 * image_width] + Q_mat_ptr[11] * y[3 + 5 * image_width]) * Q_mat_ptr[9] + (Q_mat_ptr[6] * y[4] + Q_mat_ptr[7] * y[4 + image_width] + Q_mat_ptr[8] * y[4 + 2 * image_width] + Q_mat_ptr[9] * y[4 + 3 * image_width] + Q_mat_ptr[10] * y[4 + 4 * image_width] + Q_mat_ptr[11] * y[4 + 5 * image_width]) * Q_mat_ptr[10] + (Q_mat_ptr[6] * y[5] + Q_mat_ptr[7] * y[5 + image_width] + Q_mat_ptr[8] * y[5 + 2 * image_width] + Q_mat_ptr[9] * y[5 + 3 * image_width] + Q_mat_ptr[10] * y[5 + 4 * image_width] + Q_mat_ptr[11] * y[5 + 5 * image_width]) * Q_mat_ptr[11];
					lut_element_ptr[6] = (Q_mat_ptr[6] * y[0] + Q_mat_ptr[7] * y[image_width] + Q_mat_ptr[8] * y[2 * image_width] + Q_mat_ptr[9] * y[3 * image_width] + Q_mat_ptr[10] * y[4 * image_width] + Q_mat_ptr[11] * y[5 * image_width]) * Q_mat_ptr[12] + (Q_mat_ptr[6] * y[1] + Q_mat_ptr[7] * y[1 + image_width] + Q_mat_ptr[8] * y[1 + 2 * image_width] + Q_mat_ptr[9] * y[1 + 3 * image_width] + Q_mat_ptr[10] * y[1 + 4 * image_width] + Q_mat_ptr[11] * y[1 + 5 * image_width]) * Q_mat_ptr[13] + (Q_mat_ptr[6] * y[2] + Q_mat_ptr[7] * y[2 + image_width] + Q_mat_ptr[8] * y[2 + 2 * image_width] + Q_mat_ptr[9] * y[2 + 3 * image_width] + Q_mat_ptr[10] * y[2 + 4 * image_width] + Q_mat_ptr[11] * y[2 + 5 * image_width]) * Q_mat_ptr[14] + (Q_mat_ptr[6] * y[3] + Q_mat_ptr[7] * y[3 + image_width] + Q_mat_ptr[8] * y[3 + 2 * image_width] + Q_mat_ptr[9] * y[3 + 3 * image_width] + Q_mat_ptr[10] * y[3 + 4 * image_width] + Q_mat_ptr[11] * y[3 + 5 * image_width]) * Q_mat_ptr[15] + (Q_mat_ptr[6] * y[4] + Q_mat_ptr[7] * y[4 + image_width] + Q_mat_ptr[8] * y[4 + 2 * image_width] + Q_mat_ptr[9] * y[4 + 3 * image_width] + Q_mat_ptr[10] * y[4 + 4 * image_width] + Q_mat_ptr[11] * y[4 + 5 * image_width]) * Q_mat_ptr[16] + (Q_mat_ptr[6] * y[5] + Q_mat_ptr[7] * y[5 + image_width] + Q_mat_ptr[8] * y[5 + 2 * image_width] + Q_mat_ptr[9] * y[5 + 3 * image_width] + Q_mat_ptr[10] * y[5 + 4 * image_width] + Q_mat_ptr[11] * y[5 + 5 * image_width]) * Q_mat_ptr[17];
					lut_element_ptr[7] = Q_mat_ptr[6] * y[2] + Q_mat_ptr[7] * y[2 + image_width] + Q_mat_ptr[8] * y[2 + 2 * image_width] + Q_mat_ptr[9] * y[2 + 3 * image_width] + Q_mat_ptr[10] * y[2 + 4 * image_width] + Q_mat_ptr[11] * y[2 + 5 * image_width];
					lut_element_ptr[8] = (Q_mat_ptr[12] * y[0] + Q_mat_ptr[13] * y[image_width] + Q_mat_ptr[14] * y[2 * image_width] + Q_mat_ptr[15] * y[3 * image_width] + Q_mat_ptr[16] * y[4 * image_width] + Q_mat_ptr[17] * y[5 * image_width]) * Q_mat_ptr[0] + (Q_mat_ptr[12] * y[1] + Q_mat_ptr[13] * y[1 + image_width] + Q_mat_ptr[14] * y[1 + 2 * image_width] + Q_mat_ptr[15] * y[1 + 3 * image_width] + Q_mat_ptr[16] * y[1 + 4 * image_width] + Q_mat_ptr[17] * y[1 + 5 * image_width]) * Q_mat_ptr[1] + (Q_mat_ptr[12] * y[2] + Q_mat_ptr[13] * y[2 + image_width] + Q_mat_ptr[14] * y[2 + 2 * image_width] + Q_mat_ptr[15] * y[2 + 3 * image_width] + Q_mat_ptr[16] * y[2 + 4 * image_width] + Q_mat_ptr[17] * y[2 + 5 * image_width]) * Q_mat_ptr[2] + (Q_mat_ptr[12] * y[3] + Q_mat_ptr[13] * y[3 + image_width] + Q_mat_ptr[14] * y[3 + 2 * image_width] + Q_mat_ptr[15] * y[3 + 3 * image_width] + Q_mat_ptr[16] * y[3 + 4 * image_width] + Q_mat_ptr[17] * y[3 + 5 * image_width]) * Q_mat_ptr[3] + (Q_mat_ptr[12] * y[4] + Q_mat_ptr[13] * y[4 + image_width] + Q_mat_ptr[14] * y[4 + 2 * image_width] + Q_mat_ptr[15] * y[4 + 3 * image_width] + Q_mat_ptr[16] * y[4 + 4 * image_width] + Q_mat_ptr[17] * y[4 + 5 * image_width]) * Q_mat_ptr[4] + (Q_mat_ptr[12] * y[5] + Q_mat_ptr[13] * y[5 + image_width] + Q_mat_ptr[14] * y[5 + 2 * image_width] + Q_mat_ptr[15] * y[5 + 3 * image_width] + Q_mat_ptr[16] * y[5 + 4 * image_width] + Q_mat_ptr[17] * y[5 + 5 * image_width]) * Q_mat_ptr[5];
					lut_element_ptr[9] = (Q_mat_ptr[12] * y[0] + Q_mat_ptr[13] * y[image_width] + Q_mat_ptr[14] * y[2 * image_width] + Q_mat_ptr[15] * y[3 * image_width] + Q_mat_ptr[16] * y[4 * image_width] + Q_mat_ptr[17] * y[5 * image_width]) * Q_mat_ptr[6] + (Q_mat_ptr[12] * y[1] + Q_mat_ptr[13] * y[1 + image_width] + Q_mat_ptr[14] * y[1 + 2 * image_width] + Q_mat_ptr[15] * y[1 + 3 * image_width] + Q_mat_ptr[16] * y[1 + 4 * image_width] + Q_mat_ptr[17] * y[1 + 5 * image_width]) * Q_mat_ptr[7] + (Q_mat_ptr[12] * y[2] + Q_mat_ptr[13] * y[2 + image_width] + Q_mat_ptr[14] * y[2 + 2 * image_width] + Q_mat_ptr[15] * y[2 + 3 * image_width] + Q_mat_ptr[16] * y[2 + 4 * image_width] + Q_mat_ptr[17] * y[2 + 5 * image_width]) * Q_mat_ptr[8] + (Q_mat_ptr[12] * y[3] + Q_mat_ptr[13] * y[3 + image_width] + Q_mat_ptr[14] * y[3 + 2 * image_width] + Q_mat_ptr[15] * y[3 + 3 * image_width] + Q_mat_ptr[16] * y[3 + 4 * image_width] + Q_mat_ptr[17] * y[3 + 5 * image_width]) * Q_mat_ptr[9] + (Q_mat_ptr[12] * y[4] + Q_mat_ptr[13] * y[4 + image_width] + Q_mat_ptr[14] * y[4 + 2 * image_width] + Q_mat_ptr[15] * y[4 + 3 * image_width] + Q_mat_ptr[16] * y[4 + 4 * image_width] + Q_mat_ptr[17] * y[4 + 5 * image_width]) * Q_mat_ptr[10] + (Q_mat_ptr[12] * y[5] + Q_mat_ptr[13] * y[5 + image_width] + Q_mat_ptr[14] * y[5 + 2 * image_width] + Q_mat_ptr[15] * y[5 + 3 * image_width] + Q_mat_ptr[16] * y[5 + 4 * image_width] + Q_mat_ptr[17] * y[5 + 5 * image_width]) * Q_mat_ptr[11];
					lut_element_ptr[10] = (Q_mat_ptr[12] * y[0] + Q_mat_ptr[13] * y[image_width] + Q_mat_ptr[14] * y[2 * image_width] + Q_mat_ptr[15] * y[3 * image_width] + Q_mat_ptr[16] * y[4 * image_width] + Q_mat_ptr[17] * y[5 * image_width]) * Q_mat_ptr[12] + (Q_mat_ptr[12] * y[1] + Q_mat_ptr[13] * y[1 + image_width] + Q_mat_ptr[14] * y[1 + 2 * image_width] + Q_mat_ptr[15] * y[1 + 3 * image_width] + Q_mat_ptr[16] * y[1 + 4 * image_width] + Q_mat_ptr[17] * y[1 + 5 * image_width]) * Q_mat_ptr[13] + (Q_mat_ptr[12] * y[2] + Q_mat_ptr[13] * y[2 + image_width] + Q_mat_ptr[14] * y[2 + 2 * image_width] + Q_mat_ptr[15] * y[2 + 3 * image_width] + Q_mat_ptr[16] * y[2 + 4 * image_width] + Q_mat_ptr[17] * y[2 + 5 * image_width]) * Q_mat_ptr[14] + (Q_mat_ptr[12] * y[3] + Q_mat_ptr[13] * y[3 + image_width] + Q_mat_ptr[14] * y[3 + 2 * image_width] + Q_mat_ptr[15] * y[3 + 3 * image_width] + Q_mat_ptr[16] * y[3 + 4 * image_width] + Q_mat_ptr[17] * y[3 + 5 * image_width]) * Q_mat_ptr[15] + (Q_mat_ptr[12] * y[4] + Q_mat_ptr[13] * y[4 + image_width] + Q_mat_ptr[14] * y[4 + 2 * image_width] + Q_mat_ptr[15] * y[4 + 3 * image_width] + Q_mat_ptr[16] * y[4 + 4 * image_width] + Q_mat_ptr[17] * y[4 + 5 * image_width]) * Q_mat_ptr[16] + (Q_mat_ptr[12] * y[5] + Q_mat_ptr[13] * y[5 + image_width] + Q_mat_ptr[14] * y[5 + 2 * image_width] + Q_mat_ptr[15] * y[5 + 3 * image_width] + Q_mat_ptr[16] * y[5 + 4 * image_width] + Q_mat_ptr[17] * y[5 + 5 * image_width]) * Q_mat_ptr[17];
					lut_element_ptr[11] = Q_mat_ptr[12] * y[2] + Q_mat_ptr[13] * y[2 + image_width] + Q_mat_ptr[14] * y[2 + 2 * image_width] + Q_mat_ptr[15] * y[2 + 3 * image_width] + Q_mat_ptr[16] * y[2 + 4 * image_width] + Q_mat_ptr[17] * y[2 + 5 * image_width];
					lut_element_ptr[12] = y[2 * image_width] * Q_mat_ptr[0] + y[1 + 2 * image_width] * Q_mat_ptr[1] + Q_mat_ptr[2] * y[2 + 2 * image_width] + y[3 + 2 * image_width] * Q_mat_ptr[3] + y[4 + 2 * image_width] * Q_mat_ptr[4] + y[5 + 2 * image_width] * Q_mat_ptr[5];
					lut_element_ptr[13] = y[2 * image_width] * Q_mat_ptr[6] + y[1 + 2 * image_width] * Q_mat_ptr[7] + Q_mat_ptr[8] * y[2 + 2 * image_width] + y[3 + 2 * image_width] * Q_mat_ptr[9] + y[4 + 2 * image_width] * Q_mat_ptr[10] + y[5 + 2 * image_width] * Q_mat_ptr[11];
					lut_element_ptr[14] = y[2 * image_width] * Q_mat_ptr[12] + y[1 + 2 * image_width] * Q_mat_ptr[13] + Q_mat_ptr[14] * y[2 + 2 * image_width] + y[3 + 2 * image_width] * Q_mat_ptr[15] + y[4 + 2 * image_width] * Q_mat_ptr[16] + y[5 + 2 * image_width] * Q_mat_ptr[17];
					lut_element_ptr[15] = y[2 + 2 * image_width];
				}
			}
		}
		else 
		{
			for (int i = 0;i<aoi_height;i++)
			{
				for (int j=0;j<aoi_width;j++)
				{
					double* lut_element_ptr = lut_mat->data.db+(i*aoi_width+j)*UniformCubicNatureSplineInterpolationMethod::lut_element_size_;
					int index_i_min = aoi_y_min+i -2;
					int index_j_min = aoi_x_min+j-2;

					double* y = image->data.db + index_i_min*image_width+index_j_min;
					lut_element_ptr[0] = (Q_mat_ptr[0] * y[0] + Q_mat_ptr[1] * y[image_width] + Q_mat_ptr[2] * y[2 * image_width] + Q_mat_ptr[3] * y[3 * image_width] + Q_mat_ptr[4] * y[4 * image_width] + Q_mat_ptr[5] * y[5 * image_width]) * Q_mat_ptr[0] + (Q_mat_ptr[0] * y[1] + Q_mat_ptr[1] * y[1 + image_width] + Q_mat_ptr[2] * y[1 + 2 * image_width] + Q_mat_ptr[3] * y[1 + 3 * image_width] + Q_mat_ptr[4] * y[1 + 4 * image_width] + Q_mat_ptr[5] * y[1 + 5 * image_width]) * Q_mat_ptr[1] + (Q_mat_ptr[0] * y[2] + Q_mat_ptr[1] * y[2 + image_width] + Q_mat_ptr[2] * y[2 + 2 * image_width] + Q_mat_ptr[3] * y[2 + 3 * image_width] + Q_mat_ptr[4] * y[2 + 4 * image_width] + Q_mat_ptr[5] * y[2 + 5 * image_width]) * Q_mat_ptr[2] + (Q_mat_ptr[0] * y[3] + Q_mat_ptr[1] * y[3 + image_width] + Q_mat_ptr[2] * y[3 + 2 * image_width] + Q_mat_ptr[3] * y[3 + 3 * image_width] + Q_mat_ptr[4] * y[3 + 4 * image_width] + Q_mat_ptr[5] * y[3 + 5 * image_width]) * Q_mat_ptr[3] + (Q_mat_ptr[0] * y[4] + Q_mat_ptr[1] * y[4 + image_width] + Q_mat_ptr[2] * y[4 + 2 * image_width] + Q_mat_ptr[3] * y[4 + 3 * image_width] + Q_mat_ptr[4] * y[4 + 4 * image_width] + Q_mat_ptr[5] * y[4 + 5 * image_width]) * Q_mat_ptr[4] + (Q_mat_ptr[0] * y[5] + Q_mat_ptr[1] * y[5 + image_width] + Q_mat_ptr[2] * y[5 + 2 * image_width] + Q_mat_ptr[3] * y[5 + 3 * image_width] + Q_mat_ptr[4] * y[5 + 4 * image_width] + Q_mat_ptr[5] * y[5 + 5 * image_width]) * Q_mat_ptr[5];
					lut_element_ptr[1] = (Q_mat_ptr[0] * y[0] + Q_mat_ptr[1] * y[image_width] + Q_mat_ptr[2] * y[2 * image_width] + Q_mat_ptr[3] * y[3 * image_width] + Q_mat_ptr[4] * y[4 * image_width] + Q_mat_ptr[5] * y[5 * image_width]) * Q_mat_ptr[6] + (Q_mat_ptr[0] * y[1] + Q_mat_ptr[1] * y[1 + image_width] + Q_mat_ptr[2] * y[1 + 2 * image_width] + Q_mat_ptr[3] * y[1 + 3 * image_width] + Q_mat_ptr[4] * y[1 + 4 * image_width] + Q_mat_ptr[5] * y[1 + 5 * image_width]) * Q_mat_ptr[7] + (Q_mat_ptr[0] * y[2] + Q_mat_ptr[1] * y[2 + image_width] + Q_mat_ptr[2] * y[2 + 2 * image_width] + Q_mat_ptr[3] * y[2 + 3 * image_width] + Q_mat_ptr[4] * y[2 + 4 * image_width] + Q_mat_ptr[5] * y[2 + 5 * image_width]) * Q_mat_ptr[8] + (Q_mat_ptr[0] * y[3] + Q_mat_ptr[1] * y[3 + image_width] + Q_mat_ptr[2] * y[3 + 2 * image_width] + Q_mat_ptr[3] * y[3 + 3 * image_width] + Q_mat_ptr[4] * y[3 + 4 * image_width] + Q_mat_ptr[5] * y[3 + 5 * image_width]) * Q_mat_ptr[9] + (Q_mat_ptr[0] * y[4] + Q_mat_ptr[1] * y[4 + image_width] + Q_mat_ptr[2] * y[4 + 2 * image_width] + Q_mat_ptr[3] * y[4 + 3 * image_width] + Q_mat_ptr[4] * y[4 + 4 * image_width] + Q_mat_ptr[5] * y[4 + 5 * image_width]) * Q_mat_ptr[10] + (Q_mat_ptr[0] * y[5] + Q_mat_ptr[1] * y[5 + image_width] + Q_mat_ptr[2] * y[5 + 2 * image_width] + Q_mat_ptr[3] * y[5 + 3 * image_width] + Q_mat_ptr[4] * y[5 + 4 * image_width] + Q_mat_ptr[5] * y[5 + 5 * image_width]) * Q_mat_ptr[11];
					lut_element_ptr[2] = (Q_mat_ptr[0] * y[0] + Q_mat_ptr[1] * y[image_width] + Q_mat_ptr[2] * y[2 * image_width] + Q_mat_ptr[3] * y[3 * image_width] + Q_mat_ptr[4] * y[4 * image_width] + Q_mat_ptr[5] * y[5 * image_width]) * Q_mat_ptr[12] + (Q_mat_ptr[0] * y[1] + Q_mat_ptr[1] * y[1 + image_width] + Q_mat_ptr[2] * y[1 + 2 * image_width] + Q_mat_ptr[3] * y[1 + 3 * image_width] + Q_mat_ptr[4] * y[1 + 4 * image_width] + Q_mat_ptr[5] * y[1 + 5 * image_width]) * Q_mat_ptr[13] + (Q_mat_ptr[0] * y[2] + Q_mat_ptr[1] * y[2 + image_width] + Q_mat_ptr[2] * y[2 + 2 * image_width] + Q_mat_ptr[3] * y[2 + 3 * image_width] + Q_mat_ptr[4] * y[2 + 4 * image_width] + Q_mat_ptr[5] * y[2 + 5 * image_width]) * Q_mat_ptr[14] + (Q_mat_ptr[0] * y[3] + Q_mat_ptr[1] * y[3 + image_width] + Q_mat_ptr[2] * y[3 + 2 * image_width] + Q_mat_ptr[3] * y[3 + 3 * image_width] + Q_mat_ptr[4] * y[3 + 4 * image_width] + Q_mat_ptr[5] * y[3 + 5 * image_width]) * Q_mat_ptr[15] + (Q_mat_ptr[0] * y[4] + Q_mat_ptr[1] * y[4 + image_width] + Q_mat_ptr[2] * y[4 + 2 * image_width] + Q_mat_ptr[3] * y[4 + 3 * image_width] + Q_mat_ptr[4] * y[4 + 4 * image_width] + Q_mat_ptr[5] * y[4 + 5 * image_width]) * Q_mat_ptr[16] + (Q_mat_ptr[0] * y[5] + Q_mat_ptr[1] * y[5 + image_width] + Q_mat_ptr[2] * y[5 + 2 * image_width] + Q_mat_ptr[3] * y[5 + 3 * image_width] + Q_mat_ptr[4] * y[5 + 4 * image_width] + Q_mat_ptr[5] * y[5 + 5 * image_width]) * Q_mat_ptr[17];
					lut_element_ptr[3] = Q_mat_ptr[0] * y[2] + Q_mat_ptr[1] * y[2 + image_width] + Q_mat_ptr[2] * y[2 + 2 * image_width] + Q_mat_ptr[3] * y[2 + 3 * image_width] + Q_mat_ptr[4] * y[2 + 4 * image_width] + Q_mat_ptr[5] * y[2 + 5 * image_width];
					lut_element_ptr[4] = (Q_mat_ptr[6] * y[0] + Q_mat_ptr[7] * y[image_width] + Q_mat_ptr[8] * y[2 * image_width] + Q_mat_ptr[9] * y[3 * image_width] + Q_mat_ptr[10] * y[4 * image_width] + Q_mat_ptr[11] * y[5 * image_width]) * Q_mat_ptr[0] + (Q_mat_ptr[6] * y[1] + Q_mat_ptr[7] * y[1 + image_width] + Q_mat_ptr[8] * y[1 + 2 * image_width] + Q_mat_ptr[9] * y[1 + 3 * image_width] + Q_mat_ptr[10] * y[1 + 4 * image_width] + Q_mat_ptr[11] * y[1 + 5 * image_width]) * Q_mat_ptr[1] + (Q_mat_ptr[6] * y[2] + Q_mat_ptr[7] * y[2 + image_width] + Q_mat_ptr[8] * y[2 + 2 * image_width] + Q_mat_ptr[9] * y[2 + 3 * image_width] + Q_mat_ptr[10] * y[2 + 4 * image_width] + Q_mat_ptr[11] * y[2 + 5 * image_width]) * Q_mat_ptr[2] + (Q_mat_ptr[6] * y[3] + Q_mat_ptr[7] * y[3 + image_width] + Q_mat_ptr[8] * y[3 + 2 * image_width] + Q_mat_ptr[9] * y[3 + 3 * image_width] + Q_mat_ptr[10] * y[3 + 4 * image_width] + Q_mat_ptr[11] * y[3 + 5 * image_width]) * Q_mat_ptr[3] + (Q_mat_ptr[6] * y[4] + Q_mat_ptr[7] * y[4 + image_width] + Q_mat_ptr[8] * y[4 + 2 * image_width] + Q_mat_ptr[9] * y[4 + 3 * image_width] + Q_mat_ptr[10] * y[4 + 4 * image_width] + Q_mat_ptr[11] * y[4 + 5 * image_width]) * Q_mat_ptr[4] + (Q_mat_ptr[6] * y[5] + Q_mat_ptr[7] * y[5 + image_width] + Q_mat_ptr[8] * y[5 + 2 * image_width] + Q_mat_ptr[9] * y[5 + 3 * image_width] + Q_mat_ptr[10] * y[5 + 4 * image_width] + Q_mat_ptr[11] * y[5 + 5 * image_width]) * Q_mat_ptr[5];
					lut_element_ptr[5] = (Q_mat_ptr[6] * y[0] + Q_mat_ptr[7] * y[image_width] + Q_mat_ptr[8] * y[2 * image_width] + Q_mat_ptr[9] * y[3 * image_width] + Q_mat_ptr[10] * y[4 * image_width] + Q_mat_ptr[11] * y[5 * image_width]) * Q_mat_ptr[6] + (Q_mat_ptr[6] * y[1] + Q_mat_ptr[7] * y[1 + image_width] + Q_mat_ptr[8] * y[1 + 2 * image_width] + Q_mat_ptr[9] * y[1 + 3 * image_width] + Q_mat_ptr[10] * y[1 + 4 * image_width] + Q_mat_ptr[11] * y[1 + 5 * image_width]) * Q_mat_ptr[7] + (Q_mat_ptr[6] * y[2] + Q_mat_ptr[7] * y[2 + image_width] + Q_mat_ptr[8] * y[2 + 2 * image_width] + Q_mat_ptr[9] * y[2 + 3 * image_width] + Q_mat_ptr[10] * y[2 + 4 * image_width] + Q_mat_ptr[11] * y[2 + 5 * image_width]) * Q_mat_ptr[8] + (Q_mat_ptr[6] * y[3] + Q_mat_ptr[7] * y[3 + image_width] + Q_mat_ptr[8] * y[3 + 2 * image_width] + Q_mat_ptr[9] * y[3 + 3 * image_width] + Q_mat_ptr[10] * y[3 + 4 * image_width] + Q_mat_ptr[11] * y[3 + 5 * image_width]) * Q_mat_ptr[9] + (Q_mat_ptr[6] * y[4] + Q_mat_ptr[7] * y[4 + image_width] + Q_mat_ptr[8] * y[4 + 2 * image_width] + Q_mat_ptr[9] * y[4 + 3 * image_width] + Q_mat_ptr[10] * y[4 + 4 * image_width] + Q_mat_ptr[11] * y[4 + 5 * image_width]) * Q_mat_ptr[10] + (Q_mat_ptr[6] * y[5] + Q_mat_ptr[7] * y[5 + image_width] + Q_mat_ptr[8] * y[5 + 2 * image_width] + Q_mat_ptr[9] * y[5 + 3 * image_width] + Q_mat_ptr[10] * y[5 + 4 * image_width] + Q_mat_ptr[11] * y[5 + 5 * image_width]) * Q_mat_ptr[11];
					lut_element_ptr[6] = (Q_mat_ptr[6] * y[0] + Q_mat_ptr[7] * y[image_width] + Q_mat_ptr[8] * y[2 * image_width] + Q_mat_ptr[9] * y[3 * image_width] + Q_mat_ptr[10] * y[4 * image_width] + Q_mat_ptr[11] * y[5 * image_width]) * Q_mat_ptr[12] + (Q_mat_ptr[6] * y[1] + Q_mat_ptr[7] * y[1 + image_width] + Q_mat_ptr[8] * y[1 + 2 * image_width] + Q_mat_ptr[9] * y[1 + 3 * image_width] + Q_mat_ptr[10] * y[1 + 4 * image_width] + Q_mat_ptr[11] * y[1 + 5 * image_width]) * Q_mat_ptr[13] + (Q_mat_ptr[6] * y[2] + Q_mat_ptr[7] * y[2 + image_width] + Q_mat_ptr[8] * y[2 + 2 * image_width] + Q_mat_ptr[9] * y[2 + 3 * image_width] + Q_mat_ptr[10] * y[2 + 4 * image_width] + Q_mat_ptr[11] * y[2 + 5 * image_width]) * Q_mat_ptr[14] + (Q_mat_ptr[6] * y[3] + Q_mat_ptr[7] * y[3 + image_width] + Q_mat_ptr[8] * y[3 + 2 * image_width] + Q_mat_ptr[9] * y[3 + 3 * image_width] + Q_mat_ptr[10] * y[3 + 4 * image_width] + Q_mat_ptr[11] * y[3 + 5 * image_width]) * Q_mat_ptr[15] + (Q_mat_ptr[6] * y[4] + Q_mat_ptr[7] * y[4 + image_width] + Q_mat_ptr[8] * y[4 + 2 * image_width] + Q_mat_ptr[9] * y[4 + 3 * image_width] + Q_mat_ptr[10] * y[4 + 4 * image_width] + Q_mat_ptr[11] * y[4 + 5 * image_width]) * Q_mat_ptr[16] + (Q_mat_ptr[6] * y[5] + Q_mat_ptr[7] * y[5 + image_width] + Q_mat_ptr[8] * y[5 + 2 * image_width] + Q_mat_ptr[9] * y[5 + 3 * image_width] + Q_mat_ptr[10] * y[5 + 4 * image_width] + Q_mat_ptr[11] * y[5 + 5 * image_width]) * Q_mat_ptr[17];
					lut_element_ptr[7] = Q_mat_ptr[6] * y[2] + Q_mat_ptr[7] * y[2 + image_width] + Q_mat_ptr[8] * y[2 + 2 * image_width] + Q_mat_ptr[9] * y[2 + 3 * image_width] + Q_mat_ptr[10] * y[2 + 4 * image_width] + Q_mat_ptr[11] * y[2 + 5 * image_width];
					lut_element_ptr[8] = (Q_mat_ptr[12] * y[0] + Q_mat_ptr[13] * y[image_width] + Q_mat_ptr[14] * y[2 * image_width] + Q_mat_ptr[15] * y[3 * image_width] + Q_mat_ptr[16] * y[4 * image_width] + Q_mat_ptr[17] * y[5 * image_width]) * Q_mat_ptr[0] + (Q_mat_ptr[12] * y[1] + Q_mat_ptr[13] * y[1 + image_width] + Q_mat_ptr[14] * y[1 + 2 * image_width] + Q_mat_ptr[15] * y[1 + 3 * image_width] + Q_mat_ptr[16] * y[1 + 4 * image_width] + Q_mat_ptr[17] * y[1 + 5 * image_width]) * Q_mat_ptr[1] + (Q_mat_ptr[12] * y[2] + Q_mat_ptr[13] * y[2 + image_width] + Q_mat_ptr[14] * y[2 + 2 * image_width] + Q_mat_ptr[15] * y[2 + 3 * image_width] + Q_mat_ptr[16] * y[2 + 4 * image_width] + Q_mat_ptr[17] * y[2 + 5 * image_width]) * Q_mat_ptr[2] + (Q_mat_ptr[12] * y[3] + Q_mat_ptr[13] * y[3 + image_width] + Q_mat_ptr[14] * y[3 + 2 * image_width] + Q_mat_ptr[15] * y[3 + 3 * image_width] + Q_mat_ptr[16] * y[3 + 4 * image_width] + Q_mat_ptr[17] * y[3 + 5 * image_width]) * Q_mat_ptr[3] + (Q_mat_ptr[12] * y[4] + Q_mat_ptr[13] * y[4 + image_width] + Q_mat_ptr[14] * y[4 + 2 * image_width] + Q_mat_ptr[15] * y[4 + 3 * image_width] + Q_mat_ptr[16] * y[4 + 4 * image_width] + Q_mat_ptr[17] * y[4 + 5 * image_width]) * Q_mat_ptr[4] + (Q_mat_ptr[12] * y[5] + Q_mat_ptr[13] * y[5 + image_width] + Q_mat_ptr[14] * y[5 + 2 * image_width] + Q_mat_ptr[15] * y[5 + 3 * image_width] + Q_mat_ptr[16] * y[5 + 4 * image_width] + Q_mat_ptr[17] * y[5 + 5 * image_width]) * Q_mat_ptr[5];
					lut_element_ptr[9] = (Q_mat_ptr[12] * y[0] + Q_mat_ptr[13] * y[image_width] + Q_mat_ptr[14] * y[2 * image_width] + Q_mat_ptr[15] * y[3 * image_width] + Q_mat_ptr[16] * y[4 * image_width] + Q_mat_ptr[17] * y[5 * image_width]) * Q_mat_ptr[6] + (Q_mat_ptr[12] * y[1] + Q_mat_ptr[13] * y[1 + image_width] + Q_mat_ptr[14] * y[1 + 2 * image_width] + Q_mat_ptr[15] * y[1 + 3 * image_width] + Q_mat_ptr[16] * y[1 + 4 * image_width] + Q_mat_ptr[17] * y[1 + 5 * image_width]) * Q_mat_ptr[7] + (Q_mat_ptr[12] * y[2] + Q_mat_ptr[13] * y[2 + image_width] + Q_mat_ptr[14] * y[2 + 2 * image_width] + Q_mat_ptr[15] * y[2 + 3 * image_width] + Q_mat_ptr[16] * y[2 + 4 * image_width] + Q_mat_ptr[17] * y[2 + 5 * image_width]) * Q_mat_ptr[8] + (Q_mat_ptr[12] * y[3] + Q_mat_ptr[13] * y[3 + image_width] + Q_mat_ptr[14] * y[3 + 2 * image_width] + Q_mat_ptr[15] * y[3 + 3 * image_width] + Q_mat_ptr[16] * y[3 + 4 * image_width] + Q_mat_ptr[17] * y[3 + 5 * image_width]) * Q_mat_ptr[9] + (Q_mat_ptr[12] * y[4] + Q_mat_ptr[13] * y[4 + image_width] + Q_mat_ptr[14] * y[4 + 2 * image_width] + Q_mat_ptr[15] * y[4 + 3 * image_width] + Q_mat_ptr[16] * y[4 + 4 * image_width] + Q_mat_ptr[17] * y[4 + 5 * image_width]) * Q_mat_ptr[10] + (Q_mat_ptr[12] * y[5] + Q_mat_ptr[13] * y[5 + image_width] + Q_mat_ptr[14] * y[5 + 2 * image_width] + Q_mat_ptr[15] * y[5 + 3 * image_width] + Q_mat_ptr[16] * y[5 + 4 * image_width] + Q_mat_ptr[17] * y[5 + 5 * image_width]) * Q_mat_ptr[11];
					lut_element_ptr[10] = (Q_mat_ptr[12] * y[0] + Q_mat_ptr[13] * y[image_width] + Q_mat_ptr[14] * y[2 * image_width] + Q_mat_ptr[15] * y[3 * image_width] + Q_mat_ptr[16] * y[4 * image_width] + Q_mat_ptr[17] * y[5 * image_width]) * Q_mat_ptr[12] + (Q_mat_ptr[12] * y[1] + Q_mat_ptr[13] * y[1 + image_width] + Q_mat_ptr[14] * y[1 + 2 * image_width] + Q_mat_ptr[15] * y[1 + 3 * image_width] + Q_mat_ptr[16] * y[1 + 4 * image_width] + Q_mat_ptr[17] * y[1 + 5 * image_width]) * Q_mat_ptr[13] + (Q_mat_ptr[12] * y[2] + Q_mat_ptr[13] * y[2 + image_width] + Q_mat_ptr[14] * y[2 + 2 * image_width] + Q_mat_ptr[15] * y[2 + 3 * image_width] + Q_mat_ptr[16] * y[2 + 4 * image_width] + Q_mat_ptr[17] * y[2 + 5 * image_width]) * Q_mat_ptr[14] + (Q_mat_ptr[12] * y[3] + Q_mat_ptr[13] * y[3 + image_width] + Q_mat_ptr[14] * y[3 + 2 * image_width] + Q_mat_ptr[15] * y[3 + 3 * image_width] + Q_mat_ptr[16] * y[3 + 4 * image_width] + Q_mat_ptr[17] * y[3 + 5 * image_width]) * Q_mat_ptr[15] + (Q_mat_ptr[12] * y[4] + Q_mat_ptr[13] * y[4 + image_width] + Q_mat_ptr[14] * y[4 + 2 * image_width] + Q_mat_ptr[15] * y[4 + 3 * image_width] + Q_mat_ptr[16] * y[4 + 4 * image_width] + Q_mat_ptr[17] * y[4 + 5 * image_width]) * Q_mat_ptr[16] + (Q_mat_ptr[12] * y[5] + Q_mat_ptr[13] * y[5 + image_width] + Q_mat_ptr[14] * y[5 + 2 * image_width] + Q_mat_ptr[15] * y[5 + 3 * image_width] + Q_mat_ptr[16] * y[5 + 4 * image_width] + Q_mat_ptr[17] * y[5 + 5 * image_width]) * Q_mat_ptr[17];
					lut_element_ptr[11] = Q_mat_ptr[12] * y[2] + Q_mat_ptr[13] * y[2 + image_width] + Q_mat_ptr[14] * y[2 + 2 * image_width] + Q_mat_ptr[15] * y[2 + 3 * image_width] + Q_mat_ptr[16] * y[2 + 4 * image_width] + Q_mat_ptr[17] * y[2 + 5 * image_width];
					lut_element_ptr[12] = y[2 * image_width] * Q_mat_ptr[0] + y[1 + 2 * image_width] * Q_mat_ptr[1] + Q_mat_ptr[2] * y[2 + 2 * image_width] + y[3 + 2 * image_width] * Q_mat_ptr[3] + y[4 + 2 * image_width] * Q_mat_ptr[4] + y[5 + 2 * image_width] * Q_mat_ptr[5];
					lut_element_ptr[13] = y[2 * image_width] * Q_mat_ptr[6] + y[1 + 2 * image_width] * Q_mat_ptr[7] + Q_mat_ptr[8] * y[2 + 2 * image_width] + y[3 + 2 * image_width] * Q_mat_ptr[9] + y[4 + 2 * image_width] * Q_mat_ptr[10] + y[5 + 2 * image_width] * Q_mat_ptr[11];
					lut_element_ptr[14] = y[2 * image_width] * Q_mat_ptr[12] + y[1 + 2 * image_width] * Q_mat_ptr[13] + Q_mat_ptr[14] * y[2 + 2 * image_width] + y[3 + 2 * image_width] * Q_mat_ptr[15] + y[4 + 2 * image_width] * Q_mat_ptr[16] + y[5 + 2 * image_width] * Q_mat_ptr[17];
					lut_element_ptr[15] = y[2 + 2 * image_width];
				}
			}
		}
	}
	return true;

}







bool _6x6UniformBicubicSplineInterpolationMethod::Build_Two_Dimension_Interpolation_6X6_Integer_Points_Gradient_Value_LUT( 
	CvMat* reference_image,
	CvMat* Q_mat,
	int reference_image_gradient_mat_x_min,
	int reference_image_gradient_mat_x_max,
	int reference_image_gradient_mat_y_min,
	int reference_image_gradient_mat_y_max,
	CvMat* reference_image_gradient_x_mat,
	CvMat* reference_image_gradient_y_mat
	)
{
	//判断
	//大小
	int image_width= reference_image->width;
	int image_height = reference_image->height;



	int mat_width = reference_image_gradient_x_mat->cols;
	int mat_height = reference_image_gradient_y_mat->rows;
	if (reference_image_gradient_y_mat->cols!=mat_width||
		reference_image_gradient_y_mat->rows!=mat_height)
	{
		return false;
	}
	if (reference_image_gradient_mat_x_max-reference_image_gradient_mat_x_min+1 != mat_width||
		reference_image_gradient_mat_y_max-reference_image_gradient_mat_y_min+1 !=mat_height)
	{
		return false;
	}

	if (
		reference_image_gradient_mat_x_min<2 ||
		reference_image_gradient_mat_x_max>=image_width-3 ||
		reference_image_gradient_mat_y_min<2 ||
		reference_image_gradient_mat_y_max>=image_height-3 
		)
	{
		return false;
	}

	if (Q_mat->rows!=4||Q_mat->cols!=6)
	{
		return false;
	}


	//类型
	int image_type = 0;//1--CV_8U,2--CV_64F
	if (CV_MAT_TYPE(reference_image->type) ==CV_8U )
	{
		image_type = 1;
	}
	else if (CV_MAT_TYPE(reference_image->type) ==CV_64F)
	{
		image_type = 2;
	} 
	else
	{
		return false;
	}
	if (CV_MAT_TYPE(Q_mat->type) !=CV_64F )
	{
		return false;
	}


	if (
		CV_MAT_TYPE(reference_image_gradient_x_mat->type) != CV_64F||
		CV_MAT_TYPE(reference_image_gradient_y_mat->type) != CV_64F
		)
	{
		return false;
	}



	double* Q_mat_ptr = Q_mat->data.db;
	//double* reference_gradient_x_mat_ptr = reference_gradient_x_mat->data.db;
	//double* reference_gradient_y_mat_ptr = reference_gradient_y_mat->data.db;


	if (image_type==1)
	{
		for (int i =0;i<mat_height;i++)
		{
			for (int j =0;j<mat_width;j++)
			{
				int index_i_min = reference_image_gradient_mat_y_min+i-2;
				int index_j_min = reference_image_gradient_mat_x_min+j-2;
				uchar* y=reference_image->data.ptr +index_i_min*image_width+index_j_min;
				CV_MAT_ELEM(*reference_image_gradient_x_mat,double,i,j) = (double)(
					y[2 * image_width] * Q_mat_ptr[12] + 
					y[1 + 2 * image_width] * Q_mat_ptr[13] +
					Q_mat_ptr[14] * y[2 + 2 * image_width] +
					y[3 + 2 * image_width] * Q_mat_ptr[15] +
					y[4 + 2 * image_width] * Q_mat_ptr[16] +
					y[5 + 2 * image_width] * Q_mat_ptr[17]
				);
				CV_MAT_ELEM(*reference_image_gradient_y_mat,double,i,j) = (double)(
					Q_mat_ptr[12] * y[2] + 
					Q_mat_ptr[13] * y[2 + image_width] + 
					Q_mat_ptr[14] * y[2 + 2 * image_width] + 
					Q_mat_ptr[15] * y[2 + 3 * image_width] + 
					Q_mat_ptr[16] * y[2 + 4 * image_width] + 
					Q_mat_ptr[17] * y[2 + 5 * image_width]
				);
			}
		}
	}
	else
	{
		for (int i =0;i<mat_height;i++)
		{
			for (int j =0;j<mat_width;j++)
			{
				int index_i_min = reference_image_gradient_mat_y_min+i-2;
				int index_j_min = reference_image_gradient_mat_x_min+j-2;
				double* y=reference_image->data.db +index_i_min*image_width+index_j_min;
				CV_MAT_ELEM(*reference_image_gradient_x_mat,double,i,j) = (double)(
					y[2 * image_width] * Q_mat_ptr[12] + 
					y[1 + 2 * image_width] * Q_mat_ptr[13] + 
					Q_mat_ptr[14] * y[2 + 2 * image_width] + 
					y[3 + 2 * image_width] * Q_mat_ptr[15] + 
					y[4 + 2 * image_width] * Q_mat_ptr[16] + 
					y[5 + 2 * image_width] * Q_mat_ptr[17]
				);
				CV_MAT_ELEM(*reference_image_gradient_y_mat,double,i,j) = (double)(
					Q_mat_ptr[12] * y[2] + 
					Q_mat_ptr[13] * y[2 + image_width] +
					Q_mat_ptr[14] * y[2 + 2 * image_width] + 
					Q_mat_ptr[15] * y[2 + 3 * image_width] + 
					Q_mat_ptr[16] * y[2 + 4 * image_width] + 
					Q_mat_ptr[17] * y[2 + 5 * image_width]
				);
			}
		}
	}



	return true;
}


//bool _6x6UniformBicubicSplineInterpolationMethod::Reference_Value_And_Gradient_LUT_64F(
//														CvMat* image,//64F
//														CvMat* center_point_position_x_mat,//64F
//														CvMat* center_point_position_y_mat,//64F
//														CvMat* mask_valid_sign_mat,//8U
//														int horizontal_step,
//														int vertical_step,
//														int template_half_width,
//														int template_half_height,
//														int interpolation_area_x_min,
//														int interpolation_area_x_max ,
//														int interpolation_area_y_min ,
//														int interpolation_area_y_max ,
//														CvMat* interpolation_LUT,//32F
//														CvMat* interpolation_LUT_valid_sign_mat,//8U
//														CvMat* uniform_bicubic_natrue_spline_interpolation_6x6_Q_mat,//64F
//														CvMat* value_LUT,//64F
//														CvMat* gradient_x_LUT,//64F
//														CvMat* gradient_y_LUT//64F
//															)//可以根据有效位指示，根据需要计算查找表
//{
//
//	//省略校验
//
//	//int interpolation_LUT_element_size = 16;
//	int image_width= image->width;
//	int image_height = image->height;
//
//
//	int template_width = template_half_width*2+1;
//	int template_height= template_half_height*2+1;
//	int template_point_count = template_width*template_height;
//
//
//
//	int mat_col_count  = mask_valid_sign_mat->cols;
//	int mat_row_count = mask_valid_sign_mat->rows;
//	//float* value_LUT_ptr = value_LUT->data.fl;
//	//float* gradient_x_LUT_ptr = value_LUT->data.fl;
//	//float* gradient_y_LUT_ptr = value_LUT->data.fl;
//	//float* pos_x_LUT_ptr = center_point_position_x_mat->data.fl;
//	//float* pos_y_LUT_ptr = center_point_position_y_mat->data.fl;
//
//
//
//
//	double* Q_mat_ptr = uniform_bicubic_natrue_spline_interpolation_6x6_Q_mat->data.db;
//
//	for (int i =0;i<mat_row_count;i++)
//	{
//		for (int j = 0;j<mat_col_count;j++)
//		{
//			uchar sign = CV_MAT_ELEM(*mask_valid_sign_mat,uchar,i,j);
//			if (sign==1)
//			{
//				//double u = CV_MAT_ELEM(*displacement_u_mat,double,i,j);
//				//double ux = CV_MAT_ELEM(*displacement_ux_mat,double,i,j);
//				//double uy = CV_MAT_ELEM(*displacement_uy_mat,double,i,j);
//				//double v = CV_MAT_ELEM(*displacement_v_mat,double,i,j);
//				//double vx = CV_MAT_ELEM(*displacement_vx_mat,double,i,j);
//				//double vy = CV_MAT_ELEM(*displacement_vy_mat,double,i,j);
//
//				//int center_x = j*horizontal_step+mask_left_top_x;
//				//int center_y = i*vertical_step+mask_left_top_y;
//
//				double* value_ptr = value_LUT->data.db+(i*mat_col_count+j)*template_point_count;
//				double* gradient_x_ptr = gradient_x_LUT->data.db+(i*mat_col_count+j)*template_point_count;
//				double* gradient_y_ptr = gradient_y_LUT->data.db+(i*mat_col_count+j)*template_point_count;
//
//				//double center_pos_x = CV_MAT_ELEM(*center_point_position_x_mat,double,i,j);
//				//double center_pos_y = CV_MAT_ELEM(*center_point_position_y_mat,double,i,j);
//
//				//开始插值
//
//				for (int m =-template_half_height;m<=template_half_height;m++)
//				{
//					for (int n =-template_half_width;n<=template_half_width;n++)
//					{
//						//参考图只能为正方形
//						double pos_x = CV_MAT_ELEM(*center_point_position_x_mat,double,i,j) + (double)n;
//						double pos_y = CV_MAT_ELEM(*center_point_position_y_mat,double,i,j) + (double)m;
//
//
//						if (pos_x<interpolation_area_x_min || pos_x>=interpolation_area_x_max||
//							pos_y<interpolation_area_y_min || pos_y>=interpolation_area_y_max)//右侧为开区间
//						{
//							return false;
//						}
//						int index_x = cvFloor(pos_x);
//						int index_y = cvFloor(pos_y);
//
//						double delta_x = pos_x - index_x;
//						double delta_y = pos_y - index_y;
//
//						uchar sign = CV_MAT_ELEM(*interpolation_LUT_valid_sign_mat,uchar,index_y - interpolation_area_y_min,index_x - interpolation_area_x_min);
//						float* S_mat_ptr = interpolation_LUT->data.fl+ ((index_y - interpolation_area_y_min) * (interpolation_area_x_max-interpolation_area_x_min)+index_x - interpolation_area_x_min) * Interpolation_LUT_Element_Size;
//
//						if (sign!=1)
//						{							
//							double* y = image->data.db + (index_y-2)*image_width+index_x-2;
//							S_mat_ptr[0] = (Q_mat_ptr[0] * y[0] + Q_mat_ptr[1] * y[image_width] + Q_mat_ptr[2] * y[2 * image_width] + Q_mat_ptr[3] * y[3 * image_width] + Q_mat_ptr[4] * y[4 * image_width] + Q_mat_ptr[5] * y[5 * image_width]) * Q_mat_ptr[0] + (Q_mat_ptr[0] * y[1] + Q_mat_ptr[1] * y[1 + image_width] + Q_mat_ptr[2] * y[1 + 2 * image_width] + Q_mat_ptr[3] * y[1 + 3 * image_width] + Q_mat_ptr[4] * y[1 + 4 * image_width] + Q_mat_ptr[5] * y[1 + 5 * image_width]) * Q_mat_ptr[1] + (Q_mat_ptr[0] * y[2] + Q_mat_ptr[1] * y[2 + image_width] + Q_mat_ptr[2] * y[2 + 2 * image_width] + Q_mat_ptr[3] * y[2 + 3 * image_width] + Q_mat_ptr[4] * y[2 + 4 * image_width] + Q_mat_ptr[5] * y[2 + 5 * image_width]) * Q_mat_ptr[2] + (Q_mat_ptr[0] * y[3] + Q_mat_ptr[1] * y[3 + image_width] + Q_mat_ptr[2] * y[3 + 2 * image_width] + Q_mat_ptr[3] * y[3 + 3 * image_width] + Q_mat_ptr[4] * y[3 + 4 * image_width] + Q_mat_ptr[5] * y[3 + 5 * image_width]) * Q_mat_ptr[3] + (Q_mat_ptr[0] * y[4] + Q_mat_ptr[1] * y[4 + image_width] + Q_mat_ptr[2] * y[4 + 2 * image_width] + Q_mat_ptr[3] * y[4 + 3 * image_width] + Q_mat_ptr[4] * y[4 + 4 * image_width] + Q_mat_ptr[5] * y[4 + 5 * image_width]) * Q_mat_ptr[4] + (Q_mat_ptr[0] * y[5] + Q_mat_ptr[1] * y[5 + image_width] + Q_mat_ptr[2] * y[5 + 2 * image_width] + Q_mat_ptr[3] * y[5 + 3 * image_width] + Q_mat_ptr[4] * y[5 + 4 * image_width] + Q_mat_ptr[5] * y[5 + 5 * image_width]) * Q_mat_ptr[5];
//							S_mat_ptr[1] = (Q_mat_ptr[0] * y[0] + Q_mat_ptr[1] * y[image_width] + Q_mat_ptr[2] * y[2 * image_width] + Q_mat_ptr[3] * y[3 * image_width] + Q_mat_ptr[4] * y[4 * image_width] + Q_mat_ptr[5] * y[5 * image_width]) * Q_mat_ptr[6] + (Q_mat_ptr[0] * y[1] + Q_mat_ptr[1] * y[1 + image_width] + Q_mat_ptr[2] * y[1 + 2 * image_width] + Q_mat_ptr[3] * y[1 + 3 * image_width] + Q_mat_ptr[4] * y[1 + 4 * image_width] + Q_mat_ptr[5] * y[1 + 5 * image_width]) * Q_mat_ptr[7] + (Q_mat_ptr[0] * y[2] + Q_mat_ptr[1] * y[2 + image_width] + Q_mat_ptr[2] * y[2 + 2 * image_width] + Q_mat_ptr[3] * y[2 + 3 * image_width] + Q_mat_ptr[4] * y[2 + 4 * image_width] + Q_mat_ptr[5] * y[2 + 5 * image_width]) * Q_mat_ptr[8] + (Q_mat_ptr[0] * y[3] + Q_mat_ptr[1] * y[3 + image_width] + Q_mat_ptr[2] * y[3 + 2 * image_width] + Q_mat_ptr[3] * y[3 + 3 * image_width] + Q_mat_ptr[4] * y[3 + 4 * image_width] + Q_mat_ptr[5] * y[3 + 5 * image_width]) * Q_mat_ptr[9] + (Q_mat_ptr[0] * y[4] + Q_mat_ptr[1] * y[4 + image_width] + Q_mat_ptr[2] * y[4 + 2 * image_width] + Q_mat_ptr[3] * y[4 + 3 * image_width] + Q_mat_ptr[4] * y[4 + 4 * image_width] + Q_mat_ptr[5] * y[4 + 5 * image_width]) * Q_mat_ptr[10] + (Q_mat_ptr[0] * y[5] + Q_mat_ptr[1] * y[5 + image_width] + Q_mat_ptr[2] * y[5 + 2 * image_width] + Q_mat_ptr[3] * y[5 + 3 * image_width] + Q_mat_ptr[4] * y[5 + 4 * image_width] + Q_mat_ptr[5] * y[5 + 5 * image_width]) * Q_mat_ptr[11];
//							S_mat_ptr[2] = (Q_mat_ptr[0] * y[0] + Q_mat_ptr[1] * y[image_width] + Q_mat_ptr[2] * y[2 * image_width] + Q_mat_ptr[3] * y[3 * image_width] + Q_mat_ptr[4] * y[4 * image_width] + Q_mat_ptr[5] * y[5 * image_width]) * Q_mat_ptr[12] + (Q_mat_ptr[0] * y[1] + Q_mat_ptr[1] * y[1 + image_width] + Q_mat_ptr[2] * y[1 + 2 * image_width] + Q_mat_ptr[3] * y[1 + 3 * image_width] + Q_mat_ptr[4] * y[1 + 4 * image_width] + Q_mat_ptr[5] * y[1 + 5 * image_width]) * Q_mat_ptr[13] + (Q_mat_ptr[0] * y[2] + Q_mat_ptr[1] * y[2 + image_width] + Q_mat_ptr[2] * y[2 + 2 * image_width] + Q_mat_ptr[3] * y[2 + 3 * image_width] + Q_mat_ptr[4] * y[2 + 4 * image_width] + Q_mat_ptr[5] * y[2 + 5 * image_width]) * Q_mat_ptr[14] + (Q_mat_ptr[0] * y[3] + Q_mat_ptr[1] * y[3 + image_width] + Q_mat_ptr[2] * y[3 + 2 * image_width] + Q_mat_ptr[3] * y[3 + 3 * image_width] + Q_mat_ptr[4] * y[3 + 4 * image_width] + Q_mat_ptr[5] * y[3 + 5 * image_width]) * Q_mat_ptr[15] + (Q_mat_ptr[0] * y[4] + Q_mat_ptr[1] * y[4 + image_width] + Q_mat_ptr[2] * y[4 + 2 * image_width] + Q_mat_ptr[3] * y[4 + 3 * image_width] + Q_mat_ptr[4] * y[4 + 4 * image_width] + Q_mat_ptr[5] * y[4 + 5 * image_width]) * Q_mat_ptr[16] + (Q_mat_ptr[0] * y[5] + Q_mat_ptr[1] * y[5 + image_width] + Q_mat_ptr[2] * y[5 + 2 * image_width] + Q_mat_ptr[3] * y[5 + 3 * image_width] + Q_mat_ptr[4] * y[5 + 4 * image_width] + Q_mat_ptr[5] * y[5 + 5 * image_width]) * Q_mat_ptr[17];
//							S_mat_ptr[3] = Q_mat_ptr[0] * y[2] + Q_mat_ptr[1] * y[2 + image_width] + Q_mat_ptr[2] * y[2 + 2 * image_width] + Q_mat_ptr[3] * y[2 + 3 * image_width] + Q_mat_ptr[4] * y[2 + 4 * image_width] + Q_mat_ptr[5] * y[2 + 5 * image_width];
//							S_mat_ptr[4] = (Q_mat_ptr[6] * y[0] + Q_mat_ptr[7] * y[image_width] + Q_mat_ptr[8] * y[2 * image_width] + Q_mat_ptr[9] * y[3 * image_width] + Q_mat_ptr[10] * y[4 * image_width] + Q_mat_ptr[11] * y[5 * image_width]) * Q_mat_ptr[0] + (Q_mat_ptr[6] * y[1] + Q_mat_ptr[7] * y[1 + image_width] + Q_mat_ptr[8] * y[1 + 2 * image_width] + Q_mat_ptr[9] * y[1 + 3 * image_width] + Q_mat_ptr[10] * y[1 + 4 * image_width] + Q_mat_ptr[11] * y[1 + 5 * image_width]) * Q_mat_ptr[1] + (Q_mat_ptr[6] * y[2] + Q_mat_ptr[7] * y[2 + image_width] + Q_mat_ptr[8] * y[2 + 2 * image_width] + Q_mat_ptr[9] * y[2 + 3 * image_width] + Q_mat_ptr[10] * y[2 + 4 * image_width] + Q_mat_ptr[11] * y[2 + 5 * image_width]) * Q_mat_ptr[2] + (Q_mat_ptr[6] * y[3] + Q_mat_ptr[7] * y[3 + image_width] + Q_mat_ptr[8] * y[3 + 2 * image_width] + Q_mat_ptr[9] * y[3 + 3 * image_width] + Q_mat_ptr[10] * y[3 + 4 * image_width] + Q_mat_ptr[11] * y[3 + 5 * image_width]) * Q_mat_ptr[3] + (Q_mat_ptr[6] * y[4] + Q_mat_ptr[7] * y[4 + image_width] + Q_mat_ptr[8] * y[4 + 2 * image_width] + Q_mat_ptr[9] * y[4 + 3 * image_width] + Q_mat_ptr[10] * y[4 + 4 * image_width] + Q_mat_ptr[11] * y[4 + 5 * image_width]) * Q_mat_ptr[4] + (Q_mat_ptr[6] * y[5] + Q_mat_ptr[7] * y[5 + image_width] + Q_mat_ptr[8] * y[5 + 2 * image_width] + Q_mat_ptr[9] * y[5 + 3 * image_width] + Q_mat_ptr[10] * y[5 + 4 * image_width] + Q_mat_ptr[11] * y[5 + 5 * image_width]) * Q_mat_ptr[5];
//							S_mat_ptr[5] = (Q_mat_ptr[6] * y[0] + Q_mat_ptr[7] * y[image_width] + Q_mat_ptr[8] * y[2 * image_width] + Q_mat_ptr[9] * y[3 * image_width] + Q_mat_ptr[10] * y[4 * image_width] + Q_mat_ptr[11] * y[5 * image_width]) * Q_mat_ptr[6] + (Q_mat_ptr[6] * y[1] + Q_mat_ptr[7] * y[1 + image_width] + Q_mat_ptr[8] * y[1 + 2 * image_width] + Q_mat_ptr[9] * y[1 + 3 * image_width] + Q_mat_ptr[10] * y[1 + 4 * image_width] + Q_mat_ptr[11] * y[1 + 5 * image_width]) * Q_mat_ptr[7] + (Q_mat_ptr[6] * y[2] + Q_mat_ptr[7] * y[2 + image_width] + Q_mat_ptr[8] * y[2 + 2 * image_width] + Q_mat_ptr[9] * y[2 + 3 * image_width] + Q_mat_ptr[10] * y[2 + 4 * image_width] + Q_mat_ptr[11] * y[2 + 5 * image_width]) * Q_mat_ptr[8] + (Q_mat_ptr[6] * y[3] + Q_mat_ptr[7] * y[3 + image_width] + Q_mat_ptr[8] * y[3 + 2 * image_width] + Q_mat_ptr[9] * y[3 + 3 * image_width] + Q_mat_ptr[10] * y[3 + 4 * image_width] + Q_mat_ptr[11] * y[3 + 5 * image_width]) * Q_mat_ptr[9] + (Q_mat_ptr[6] * y[4] + Q_mat_ptr[7] * y[4 + image_width] + Q_mat_ptr[8] * y[4 + 2 * image_width] + Q_mat_ptr[9] * y[4 + 3 * image_width] + Q_mat_ptr[10] * y[4 + 4 * image_width] + Q_mat_ptr[11] * y[4 + 5 * image_width]) * Q_mat_ptr[10] + (Q_mat_ptr[6] * y[5] + Q_mat_ptr[7] * y[5 + image_width] + Q_mat_ptr[8] * y[5 + 2 * image_width] + Q_mat_ptr[9] * y[5 + 3 * image_width] + Q_mat_ptr[10] * y[5 + 4 * image_width] + Q_mat_ptr[11] * y[5 + 5 * image_width]) * Q_mat_ptr[11];
//							S_mat_ptr[6] = (Q_mat_ptr[6] * y[0] + Q_mat_ptr[7] * y[image_width] + Q_mat_ptr[8] * y[2 * image_width] + Q_mat_ptr[9] * y[3 * image_width] + Q_mat_ptr[10] * y[4 * image_width] + Q_mat_ptr[11] * y[5 * image_width]) * Q_mat_ptr[12] + (Q_mat_ptr[6] * y[1] + Q_mat_ptr[7] * y[1 + image_width] + Q_mat_ptr[8] * y[1 + 2 * image_width] + Q_mat_ptr[9] * y[1 + 3 * image_width] + Q_mat_ptr[10] * y[1 + 4 * image_width] + Q_mat_ptr[11] * y[1 + 5 * image_width]) * Q_mat_ptr[13] + (Q_mat_ptr[6] * y[2] + Q_mat_ptr[7] * y[2 + image_width] + Q_mat_ptr[8] * y[2 + 2 * image_width] + Q_mat_ptr[9] * y[2 + 3 * image_width] + Q_mat_ptr[10] * y[2 + 4 * image_width] + Q_mat_ptr[11] * y[2 + 5 * image_width]) * Q_mat_ptr[14] + (Q_mat_ptr[6] * y[3] + Q_mat_ptr[7] * y[3 + image_width] + Q_mat_ptr[8] * y[3 + 2 * image_width] + Q_mat_ptr[9] * y[3 + 3 * image_width] + Q_mat_ptr[10] * y[3 + 4 * image_width] + Q_mat_ptr[11] * y[3 + 5 * image_width]) * Q_mat_ptr[15] + (Q_mat_ptr[6] * y[4] + Q_mat_ptr[7] * y[4 + image_width] + Q_mat_ptr[8] * y[4 + 2 * image_width] + Q_mat_ptr[9] * y[4 + 3 * image_width] + Q_mat_ptr[10] * y[4 + 4 * image_width] + Q_mat_ptr[11] * y[4 + 5 * image_width]) * Q_mat_ptr[16] + (Q_mat_ptr[6] * y[5] + Q_mat_ptr[7] * y[5 + image_width] + Q_mat_ptr[8] * y[5 + 2 * image_width] + Q_mat_ptr[9] * y[5 + 3 * image_width] + Q_mat_ptr[10] * y[5 + 4 * image_width] + Q_mat_ptr[11] * y[5 + 5 * image_width]) * Q_mat_ptr[17];
//							S_mat_ptr[7] = Q_mat_ptr[6] * y[2] + Q_mat_ptr[7] * y[2 + image_width] + Q_mat_ptr[8] * y[2 + 2 * image_width] + Q_mat_ptr[9] * y[2 + 3 * image_width] + Q_mat_ptr[10] * y[2 + 4 * image_width] + Q_mat_ptr[11] * y[2 + 5 * image_width];
//							S_mat_ptr[8] = (Q_mat_ptr[12] * y[0] + Q_mat_ptr[13] * y[image_width] + Q_mat_ptr[14] * y[2 * image_width] + Q_mat_ptr[15] * y[3 * image_width] + Q_mat_ptr[16] * y[4 * image_width] + Q_mat_ptr[17] * y[5 * image_width]) * Q_mat_ptr[0] + (Q_mat_ptr[12] * y[1] + Q_mat_ptr[13] * y[1 + image_width] + Q_mat_ptr[14] * y[1 + 2 * image_width] + Q_mat_ptr[15] * y[1 + 3 * image_width] + Q_mat_ptr[16] * y[1 + 4 * image_width] + Q_mat_ptr[17] * y[1 + 5 * image_width]) * Q_mat_ptr[1] + (Q_mat_ptr[12] * y[2] + Q_mat_ptr[13] * y[2 + image_width] + Q_mat_ptr[14] * y[2 + 2 * image_width] + Q_mat_ptr[15] * y[2 + 3 * image_width] + Q_mat_ptr[16] * y[2 + 4 * image_width] + Q_mat_ptr[17] * y[2 + 5 * image_width]) * Q_mat_ptr[2] + (Q_mat_ptr[12] * y[3] + Q_mat_ptr[13] * y[3 + image_width] + Q_mat_ptr[14] * y[3 + 2 * image_width] + Q_mat_ptr[15] * y[3 + 3 * image_width] + Q_mat_ptr[16] * y[3 + 4 * image_width] + Q_mat_ptr[17] * y[3 + 5 * image_width]) * Q_mat_ptr[3] + (Q_mat_ptr[12] * y[4] + Q_mat_ptr[13] * y[4 + image_width] + Q_mat_ptr[14] * y[4 + 2 * image_width] + Q_mat_ptr[15] * y[4 + 3 * image_width] + Q_mat_ptr[16] * y[4 + 4 * image_width] + Q_mat_ptr[17] * y[4 + 5 * image_width]) * Q_mat_ptr[4] + (Q_mat_ptr[12] * y[5] + Q_mat_ptr[13] * y[5 + image_width] + Q_mat_ptr[14] * y[5 + 2 * image_width] + Q_mat_ptr[15] * y[5 + 3 * image_width] + Q_mat_ptr[16] * y[5 + 4 * image_width] + Q_mat_ptr[17] * y[5 + 5 * image_width]) * Q_mat_ptr[5];
//							S_mat_ptr[9] = (Q_mat_ptr[12] * y[0] + Q_mat_ptr[13] * y[image_width] + Q_mat_ptr[14] * y[2 * image_width] + Q_mat_ptr[15] * y[3 * image_width] + Q_mat_ptr[16] * y[4 * image_width] + Q_mat_ptr[17] * y[5 * image_width]) * Q_mat_ptr[6] + (Q_mat_ptr[12] * y[1] + Q_mat_ptr[13] * y[1 + image_width] + Q_mat_ptr[14] * y[1 + 2 * image_width] + Q_mat_ptr[15] * y[1 + 3 * image_width] + Q_mat_ptr[16] * y[1 + 4 * image_width] + Q_mat_ptr[17] * y[1 + 5 * image_width]) * Q_mat_ptr[7] + (Q_mat_ptr[12] * y[2] + Q_mat_ptr[13] * y[2 + image_width] + Q_mat_ptr[14] * y[2 + 2 * image_width] + Q_mat_ptr[15] * y[2 + 3 * image_width] + Q_mat_ptr[16] * y[2 + 4 * image_width] + Q_mat_ptr[17] * y[2 + 5 * image_width]) * Q_mat_ptr[8] + (Q_mat_ptr[12] * y[3] + Q_mat_ptr[13] * y[3 + image_width] + Q_mat_ptr[14] * y[3 + 2 * image_width] + Q_mat_ptr[15] * y[3 + 3 * image_width] + Q_mat_ptr[16] * y[3 + 4 * image_width] + Q_mat_ptr[17] * y[3 + 5 * image_width]) * Q_mat_ptr[9] + (Q_mat_ptr[12] * y[4] + Q_mat_ptr[13] * y[4 + image_width] + Q_mat_ptr[14] * y[4 + 2 * image_width] + Q_mat_ptr[15] * y[4 + 3 * image_width] + Q_mat_ptr[16] * y[4 + 4 * image_width] + Q_mat_ptr[17] * y[4 + 5 * image_width]) * Q_mat_ptr[10] + (Q_mat_ptr[12] * y[5] + Q_mat_ptr[13] * y[5 + image_width] + Q_mat_ptr[14] * y[5 + 2 * image_width] + Q_mat_ptr[15] * y[5 + 3 * image_width] + Q_mat_ptr[16] * y[5 + 4 * image_width] + Q_mat_ptr[17] * y[5 + 5 * image_width]) * Q_mat_ptr[11];
//							S_mat_ptr[10] = (Q_mat_ptr[12] * y[0] + Q_mat_ptr[13] * y[image_width] + Q_mat_ptr[14] * y[2 * image_width] + Q_mat_ptr[15] * y[3 * image_width] + Q_mat_ptr[16] * y[4 * image_width] + Q_mat_ptr[17] * y[5 * image_width]) * Q_mat_ptr[12] + (Q_mat_ptr[12] * y[1] + Q_mat_ptr[13] * y[1 + image_width] + Q_mat_ptr[14] * y[1 + 2 * image_width] + Q_mat_ptr[15] * y[1 + 3 * image_width] + Q_mat_ptr[16] * y[1 + 4 * image_width] + Q_mat_ptr[17] * y[1 + 5 * image_width]) * Q_mat_ptr[13] + (Q_mat_ptr[12] * y[2] + Q_mat_ptr[13] * y[2 + image_width] + Q_mat_ptr[14] * y[2 + 2 * image_width] + Q_mat_ptr[15] * y[2 + 3 * image_width] + Q_mat_ptr[16] * y[2 + 4 * image_width] + Q_mat_ptr[17] * y[2 + 5 * image_width]) * Q_mat_ptr[14] + (Q_mat_ptr[12] * y[3] + Q_mat_ptr[13] * y[3 + image_width] + Q_mat_ptr[14] * y[3 + 2 * image_width] + Q_mat_ptr[15] * y[3 + 3 * image_width] + Q_mat_ptr[16] * y[3 + 4 * image_width] + Q_mat_ptr[17] * y[3 + 5 * image_width]) * Q_mat_ptr[15] + (Q_mat_ptr[12] * y[4] + Q_mat_ptr[13] * y[4 + image_width] + Q_mat_ptr[14] * y[4 + 2 * image_width] + Q_mat_ptr[15] * y[4 + 3 * image_width] + Q_mat_ptr[16] * y[4 + 4 * image_width] + Q_mat_ptr[17] * y[4 + 5 * image_width]) * Q_mat_ptr[16] + (Q_mat_ptr[12] * y[5] + Q_mat_ptr[13] * y[5 + image_width] + Q_mat_ptr[14] * y[5 + 2 * image_width] + Q_mat_ptr[15] * y[5 + 3 * image_width] + Q_mat_ptr[16] * y[5 + 4 * image_width] + Q_mat_ptr[17] * y[5 + 5 * image_width]) * Q_mat_ptr[17];
//							S_mat_ptr[11] = Q_mat_ptr[12] * y[2] + Q_mat_ptr[13] * y[2 + image_width] + Q_mat_ptr[14] * y[2 + 2 * image_width] + Q_mat_ptr[15] * y[2 + 3 * image_width] + Q_mat_ptr[16] * y[2 + 4 * image_width] + Q_mat_ptr[17] * y[2 + 5 * image_width];
//							S_mat_ptr[12] = y[2 * image_width] * Q_mat_ptr[0] + y[1 + 2 * image_width] * Q_mat_ptr[1] + Q_mat_ptr[2] * y[2 + 2 * image_width] + y[3 + 2 * image_width] * Q_mat_ptr[3] + y[4 + 2 * image_width] * Q_mat_ptr[4] + y[5 + 2 * image_width] * Q_mat_ptr[5];
//							S_mat_ptr[13] = y[2 * image_width] * Q_mat_ptr[6] + y[1 + 2 * image_width] * Q_mat_ptr[7] + Q_mat_ptr[8] * y[2 + 2 * image_width] + y[3 + 2 * image_width] * Q_mat_ptr[9] + y[4 + 2 * image_width] * Q_mat_ptr[10] + y[5 + 2 * image_width] * Q_mat_ptr[11];
//							S_mat_ptr[14] = y[2 * image_width] * Q_mat_ptr[12] + y[1 + 2 * image_width] * Q_mat_ptr[13] + Q_mat_ptr[14] * y[2 + 2 * image_width] + y[3 + 2 * image_width] * Q_mat_ptr[15] + y[4 + 2 * image_width] * Q_mat_ptr[16] + y[5 + 2 * image_width] * Q_mat_ptr[17];
//							S_mat_ptr[15] = y[2 + 2 * image_width];
//							CV_MAT_ELEM(*interpolation_LUT_valid_sign_mat,uchar,index_y - interpolation_area_y_min,index_x - interpolation_area_x_min) =1;		
//						}
//						//g
//						value_ptr[(m+template_half_height)*template_width+n+template_half_width]=
//							delta_x*delta_x*delta_x * (delta_y*delta_y*delta_y * S_mat_ptr[0] + delta_y*delta_y * S_mat_ptr[4] + delta_y * S_mat_ptr[8] + S_mat_ptr[12] ) +
//							delta_x*delta_x * (delta_y*delta_y*delta_y * S_mat_ptr[1] + delta_y*delta_y * S_mat_ptr[5] + delta_y * S_mat_ptr[9] + S_mat_ptr[13] ) +
//							delta_x  * (delta_y*delta_y*delta_y * S_mat_ptr[2] + delta_y*delta_y * S_mat_ptr[6] + delta_y * S_mat_ptr[10] + S_mat_ptr[14] ) +
//							(delta_y*delta_y*delta_y * S_mat_ptr[3] + delta_y*delta_y * S_mat_ptr[7] + delta_y * S_mat_ptr[11] + S_mat_ptr[15] );
//
//						gradient_x_ptr[(m+template_half_height)*template_width+n+template_half_width]=
//							3.0 * delta_x*delta_x * (delta_y*delta_y*delta_y * S_mat_ptr[0] + delta_y*delta_y * S_mat_ptr[4] + delta_y * S_mat_ptr[8] + S_mat_ptr[12] ) +
//							2.0 * delta_x  * (delta_y*delta_y*delta_y * S_mat_ptr[1] + delta_y*delta_y * S_mat_ptr[5] + delta_y * S_mat_ptr[9] + S_mat_ptr[13] ) +
//							(delta_y*delta_y*delta_y * S_mat_ptr[2] + delta_y*delta_y * S_mat_ptr[6] + delta_y * S_mat_ptr[10] + S_mat_ptr[14] );
//
//						gradient_y_ptr [(m+template_half_height)*template_width+n+template_half_width]=
//							delta_x*delta_x*delta_x * (3.0 * delta_y*delta_y * S_mat_ptr[0] + 2.0 * delta_y * S_mat_ptr[4] + S_mat_ptr[8] ) +
//							delta_x*delta_x * (3.0 * delta_y*delta_y * S_mat_ptr[1] +  2.0 * delta_y * S_mat_ptr[5] + S_mat_ptr[9] ) +
//							delta_x  * (3.0 * delta_y*delta_y * S_mat_ptr[2] +  2.0 * delta_y * S_mat_ptr[6] + S_mat_ptr[10] ) +
//							(3.0 * delta_y*delta_y * S_mat_ptr[3] +  2.0 * delta_y * S_mat_ptr[7] + S_mat_ptr[11] );
//
//					}
//				}
//
//
//				
//
//			}
//		}
//	}
//
//
//	return true;
//}
bool _6x6UniformBicubicSplineInterpolationMethod::Deform_Value_One_Template_64F_20130617_Before(
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
	)
{

	//判断
	//大小
	int image_width= image->width;
	int image_height = image->height;
	int template_width = g_mat->cols;
	int template_height = g_mat->cols;
	int lut_width = interpolation_LUT->cols;
	int lut_height = interpolation_LUT->rows;

	if (displacement_input->rows!=6 && displacement_input->cols!=1)
	{
		return false;
	} 

	if (interpolation_area_x_min<0 || interpolation_area_x_max>=image_width||
		interpolation_area_y_min<0 || interpolation_area_y_max>=image_height)
	{
		return false;
	}
	int interpolation_area_width = interpolation_area_x_max-interpolation_area_x_min;
	int interpolation_area_height = interpolation_area_y_max-interpolation_area_y_min;

	if (interpolation_LUT->cols != interpolation_area_width*Interpolation_LUT_Element_Size||
		interpolation_LUT->rows !=interpolation_area_height||
		interpolation_LUT_valid_sign_mat->cols!=interpolation_area_width||
		interpolation_LUT_valid_sign_mat->rows!= interpolation_area_height)
	{
		return false;
	}

	if (uniform_bicubic_natrue_spline_interpolation_6x6_Q_mat->rows!=4 || uniform_bicubic_natrue_spline_interpolation_6x6_Q_mat->cols!=6)
	{
		return false;
	}



	//类型
	if (CV_MAT_TYPE(image->type) !=CV_64F)
	{
		return false;
	}

	if (CV_MAT_TYPE(interpolation_LUT->type) != CV_32F)
	{
		return false;
	}


	if ( CV_MAT_TYPE(displacement_input->type) != CV_64F)
	{
		return false;
	}
	if (CV_MAT_TYPE(g_mat->type) != CV_64F || CV_MAT_TYPE(uniform_bicubic_natrue_spline_interpolation_6x6_Q_mat->type) != CV_64F || CV_MAT_TYPE(interpolation_LUT_valid_sign_mat->type)!=CV_8U)
	{
		return false;
	}

	double u = CV_MAT_ELEM(*displacement_input,double,0,0);
	double ux = CV_MAT_ELEM(*displacement_input,double,1,0);
	double uy = CV_MAT_ELEM(*displacement_input,double,2,0);
	double v = CV_MAT_ELEM(*displacement_input,double,3,0) ;
	double vx = CV_MAT_ELEM(*displacement_input,double,4,0);
	double vy = CV_MAT_ELEM(*displacement_input,double,5,0);

	int template_half_width = (template_width-1)/2;
	int template_half_height = (template_height-1)/2;
	double* Q_mat_ptr = uniform_bicubic_natrue_spline_interpolation_6x6_Q_mat->data.db;




	for (int m =-template_half_height;m<=template_half_height;m++)
	{
		for (int n =-template_half_width;n<=template_half_width;n++)
		{
			//第一种
			double pos_x =reference_point_x + u + n + ux * n + uy * m;
			double pos_y =reference_point_y + v + m + vx * n + vy * m;

			int index_x = cvFloor(pos_x);
			int index_y = cvFloor(pos_y);
			if (index_y - interpolation_area_y_min <0 || index_y - interpolation_area_y_min >= interpolation_LUT_valid_sign_mat->rows-1 ||
				index_x - interpolation_area_x_min <0 || index_x - interpolation_area_x_min >= interpolation_LUT_valid_sign_mat->cols-1)
			{
				return false;
			}

			double delta_x = pos_x - index_x;
			double delta_y = pos_y - index_y;

			uchar sign = CV_MAT_ELEM(*interpolation_LUT_valid_sign_mat,uchar,index_y - interpolation_area_y_min,index_x - interpolation_area_x_min);
			float* S_mat_ptr = interpolation_LUT->data.fl+ ((index_y - interpolation_area_y_min) * (interpolation_area_x_max-interpolation_area_x_min)+index_x - interpolation_area_x_min) * Interpolation_LUT_Element_Size;
			if (sign!=1)
			{							
				double* y = image->data.db + (index_y-2)*image_width+index_x-2;
				S_mat_ptr[0] = (Q_mat_ptr[0] * y[0] + Q_mat_ptr[1] * y[image_width] + Q_mat_ptr[2] * y[2 * image_width] + Q_mat_ptr[3] * y[3 * image_width] + Q_mat_ptr[4] * y[4 * image_width] + Q_mat_ptr[5] * y[5 * image_width]) * Q_mat_ptr[0] + (Q_mat_ptr[0] * y[1] + Q_mat_ptr[1] * y[1 + image_width] + Q_mat_ptr[2] * y[1 + 2 * image_width] + Q_mat_ptr[3] * y[1 + 3 * image_width] + Q_mat_ptr[4] * y[1 + 4 * image_width] + Q_mat_ptr[5] * y[1 + 5 * image_width]) * Q_mat_ptr[1] + (Q_mat_ptr[0] * y[2] + Q_mat_ptr[1] * y[2 + image_width] + Q_mat_ptr[2] * y[2 + 2 * image_width] + Q_mat_ptr[3] * y[2 + 3 * image_width] + Q_mat_ptr[4] * y[2 + 4 * image_width] + Q_mat_ptr[5] * y[2 + 5 * image_width]) * Q_mat_ptr[2] + (Q_mat_ptr[0] * y[3] + Q_mat_ptr[1] * y[3 + image_width] + Q_mat_ptr[2] * y[3 + 2 * image_width] + Q_mat_ptr[3] * y[3 + 3 * image_width] + Q_mat_ptr[4] * y[3 + 4 * image_width] + Q_mat_ptr[5] * y[3 + 5 * image_width]) * Q_mat_ptr[3] + (Q_mat_ptr[0] * y[4] + Q_mat_ptr[1] * y[4 + image_width] + Q_mat_ptr[2] * y[4 + 2 * image_width] + Q_mat_ptr[3] * y[4 + 3 * image_width] + Q_mat_ptr[4] * y[4 + 4 * image_width] + Q_mat_ptr[5] * y[4 + 5 * image_width]) * Q_mat_ptr[4] + (Q_mat_ptr[0] * y[5] + Q_mat_ptr[1] * y[5 + image_width] + Q_mat_ptr[2] * y[5 + 2 * image_width] + Q_mat_ptr[3] * y[5 + 3 * image_width] + Q_mat_ptr[4] * y[5 + 4 * image_width] + Q_mat_ptr[5] * y[5 + 5 * image_width]) * Q_mat_ptr[5];
				S_mat_ptr[1] = (Q_mat_ptr[0] * y[0] + Q_mat_ptr[1] * y[image_width] + Q_mat_ptr[2] * y[2 * image_width] + Q_mat_ptr[3] * y[3 * image_width] + Q_mat_ptr[4] * y[4 * image_width] + Q_mat_ptr[5] * y[5 * image_width]) * Q_mat_ptr[6] + (Q_mat_ptr[0] * y[1] + Q_mat_ptr[1] * y[1 + image_width] + Q_mat_ptr[2] * y[1 + 2 * image_width] + Q_mat_ptr[3] * y[1 + 3 * image_width] + Q_mat_ptr[4] * y[1 + 4 * image_width] + Q_mat_ptr[5] * y[1 + 5 * image_width]) * Q_mat_ptr[7] + (Q_mat_ptr[0] * y[2] + Q_mat_ptr[1] * y[2 + image_width] + Q_mat_ptr[2] * y[2 + 2 * image_width] + Q_mat_ptr[3] * y[2 + 3 * image_width] + Q_mat_ptr[4] * y[2 + 4 * image_width] + Q_mat_ptr[5] * y[2 + 5 * image_width]) * Q_mat_ptr[8] + (Q_mat_ptr[0] * y[3] + Q_mat_ptr[1] * y[3 + image_width] + Q_mat_ptr[2] * y[3 + 2 * image_width] + Q_mat_ptr[3] * y[3 + 3 * image_width] + Q_mat_ptr[4] * y[3 + 4 * image_width] + Q_mat_ptr[5] * y[3 + 5 * image_width]) * Q_mat_ptr[9] + (Q_mat_ptr[0] * y[4] + Q_mat_ptr[1] * y[4 + image_width] + Q_mat_ptr[2] * y[4 + 2 * image_width] + Q_mat_ptr[3] * y[4 + 3 * image_width] + Q_mat_ptr[4] * y[4 + 4 * image_width] + Q_mat_ptr[5] * y[4 + 5 * image_width]) * Q_mat_ptr[10] + (Q_mat_ptr[0] * y[5] + Q_mat_ptr[1] * y[5 + image_width] + Q_mat_ptr[2] * y[5 + 2 * image_width] + Q_mat_ptr[3] * y[5 + 3 * image_width] + Q_mat_ptr[4] * y[5 + 4 * image_width] + Q_mat_ptr[5] * y[5 + 5 * image_width]) * Q_mat_ptr[11];
				S_mat_ptr[2] = (Q_mat_ptr[0] * y[0] + Q_mat_ptr[1] * y[image_width] + Q_mat_ptr[2] * y[2 * image_width] + Q_mat_ptr[3] * y[3 * image_width] + Q_mat_ptr[4] * y[4 * image_width] + Q_mat_ptr[5] * y[5 * image_width]) * Q_mat_ptr[12] + (Q_mat_ptr[0] * y[1] + Q_mat_ptr[1] * y[1 + image_width] + Q_mat_ptr[2] * y[1 + 2 * image_width] + Q_mat_ptr[3] * y[1 + 3 * image_width] + Q_mat_ptr[4] * y[1 + 4 * image_width] + Q_mat_ptr[5] * y[1 + 5 * image_width]) * Q_mat_ptr[13] + (Q_mat_ptr[0] * y[2] + Q_mat_ptr[1] * y[2 + image_width] + Q_mat_ptr[2] * y[2 + 2 * image_width] + Q_mat_ptr[3] * y[2 + 3 * image_width] + Q_mat_ptr[4] * y[2 + 4 * image_width] + Q_mat_ptr[5] * y[2 + 5 * image_width]) * Q_mat_ptr[14] + (Q_mat_ptr[0] * y[3] + Q_mat_ptr[1] * y[3 + image_width] + Q_mat_ptr[2] * y[3 + 2 * image_width] + Q_mat_ptr[3] * y[3 + 3 * image_width] + Q_mat_ptr[4] * y[3 + 4 * image_width] + Q_mat_ptr[5] * y[3 + 5 * image_width]) * Q_mat_ptr[15] + (Q_mat_ptr[0] * y[4] + Q_mat_ptr[1] * y[4 + image_width] + Q_mat_ptr[2] * y[4 + 2 * image_width] + Q_mat_ptr[3] * y[4 + 3 * image_width] + Q_mat_ptr[4] * y[4 + 4 * image_width] + Q_mat_ptr[5] * y[4 + 5 * image_width]) * Q_mat_ptr[16] + (Q_mat_ptr[0] * y[5] + Q_mat_ptr[1] * y[5 + image_width] + Q_mat_ptr[2] * y[5 + 2 * image_width] + Q_mat_ptr[3] * y[5 + 3 * image_width] + Q_mat_ptr[4] * y[5 + 4 * image_width] + Q_mat_ptr[5] * y[5 + 5 * image_width]) * Q_mat_ptr[17];
				S_mat_ptr[3] = Q_mat_ptr[0] * y[2] + Q_mat_ptr[1] * y[2 + image_width] + Q_mat_ptr[2] * y[2 + 2 * image_width] + Q_mat_ptr[3] * y[2 + 3 * image_width] + Q_mat_ptr[4] * y[2 + 4 * image_width] + Q_mat_ptr[5] * y[2 + 5 * image_width];
				S_mat_ptr[4] = (Q_mat_ptr[6] * y[0] + Q_mat_ptr[7] * y[image_width] + Q_mat_ptr[8] * y[2 * image_width] + Q_mat_ptr[9] * y[3 * image_width] + Q_mat_ptr[10] * y[4 * image_width] + Q_mat_ptr[11] * y[5 * image_width]) * Q_mat_ptr[0] + (Q_mat_ptr[6] * y[1] + Q_mat_ptr[7] * y[1 + image_width] + Q_mat_ptr[8] * y[1 + 2 * image_width] + Q_mat_ptr[9] * y[1 + 3 * image_width] + Q_mat_ptr[10] * y[1 + 4 * image_width] + Q_mat_ptr[11] * y[1 + 5 * image_width]) * Q_mat_ptr[1] + (Q_mat_ptr[6] * y[2] + Q_mat_ptr[7] * y[2 + image_width] + Q_mat_ptr[8] * y[2 + 2 * image_width] + Q_mat_ptr[9] * y[2 + 3 * image_width] + Q_mat_ptr[10] * y[2 + 4 * image_width] + Q_mat_ptr[11] * y[2 + 5 * image_width]) * Q_mat_ptr[2] + (Q_mat_ptr[6] * y[3] + Q_mat_ptr[7] * y[3 + image_width] + Q_mat_ptr[8] * y[3 + 2 * image_width] + Q_mat_ptr[9] * y[3 + 3 * image_width] + Q_mat_ptr[10] * y[3 + 4 * image_width] + Q_mat_ptr[11] * y[3 + 5 * image_width]) * Q_mat_ptr[3] + (Q_mat_ptr[6] * y[4] + Q_mat_ptr[7] * y[4 + image_width] + Q_mat_ptr[8] * y[4 + 2 * image_width] + Q_mat_ptr[9] * y[4 + 3 * image_width] + Q_mat_ptr[10] * y[4 + 4 * image_width] + Q_mat_ptr[11] * y[4 + 5 * image_width]) * Q_mat_ptr[4] + (Q_mat_ptr[6] * y[5] + Q_mat_ptr[7] * y[5 + image_width] + Q_mat_ptr[8] * y[5 + 2 * image_width] + Q_mat_ptr[9] * y[5 + 3 * image_width] + Q_mat_ptr[10] * y[5 + 4 * image_width] + Q_mat_ptr[11] * y[5 + 5 * image_width]) * Q_mat_ptr[5];
				S_mat_ptr[5] = (Q_mat_ptr[6] * y[0] + Q_mat_ptr[7] * y[image_width] + Q_mat_ptr[8] * y[2 * image_width] + Q_mat_ptr[9] * y[3 * image_width] + Q_mat_ptr[10] * y[4 * image_width] + Q_mat_ptr[11] * y[5 * image_width]) * Q_mat_ptr[6] + (Q_mat_ptr[6] * y[1] + Q_mat_ptr[7] * y[1 + image_width] + Q_mat_ptr[8] * y[1 + 2 * image_width] + Q_mat_ptr[9] * y[1 + 3 * image_width] + Q_mat_ptr[10] * y[1 + 4 * image_width] + Q_mat_ptr[11] * y[1 + 5 * image_width]) * Q_mat_ptr[7] + (Q_mat_ptr[6] * y[2] + Q_mat_ptr[7] * y[2 + image_width] + Q_mat_ptr[8] * y[2 + 2 * image_width] + Q_mat_ptr[9] * y[2 + 3 * image_width] + Q_mat_ptr[10] * y[2 + 4 * image_width] + Q_mat_ptr[11] * y[2 + 5 * image_width]) * Q_mat_ptr[8] + (Q_mat_ptr[6] * y[3] + Q_mat_ptr[7] * y[3 + image_width] + Q_mat_ptr[8] * y[3 + 2 * image_width] + Q_mat_ptr[9] * y[3 + 3 * image_width] + Q_mat_ptr[10] * y[3 + 4 * image_width] + Q_mat_ptr[11] * y[3 + 5 * image_width]) * Q_mat_ptr[9] + (Q_mat_ptr[6] * y[4] + Q_mat_ptr[7] * y[4 + image_width] + Q_mat_ptr[8] * y[4 + 2 * image_width] + Q_mat_ptr[9] * y[4 + 3 * image_width] + Q_mat_ptr[10] * y[4 + 4 * image_width] + Q_mat_ptr[11] * y[4 + 5 * image_width]) * Q_mat_ptr[10] + (Q_mat_ptr[6] * y[5] + Q_mat_ptr[7] * y[5 + image_width] + Q_mat_ptr[8] * y[5 + 2 * image_width] + Q_mat_ptr[9] * y[5 + 3 * image_width] + Q_mat_ptr[10] * y[5 + 4 * image_width] + Q_mat_ptr[11] * y[5 + 5 * image_width]) * Q_mat_ptr[11];
				S_mat_ptr[6] = (Q_mat_ptr[6] * y[0] + Q_mat_ptr[7] * y[image_width] + Q_mat_ptr[8] * y[2 * image_width] + Q_mat_ptr[9] * y[3 * image_width] + Q_mat_ptr[10] * y[4 * image_width] + Q_mat_ptr[11] * y[5 * image_width]) * Q_mat_ptr[12] + (Q_mat_ptr[6] * y[1] + Q_mat_ptr[7] * y[1 + image_width] + Q_mat_ptr[8] * y[1 + 2 * image_width] + Q_mat_ptr[9] * y[1 + 3 * image_width] + Q_mat_ptr[10] * y[1 + 4 * image_width] + Q_mat_ptr[11] * y[1 + 5 * image_width]) * Q_mat_ptr[13] + (Q_mat_ptr[6] * y[2] + Q_mat_ptr[7] * y[2 + image_width] + Q_mat_ptr[8] * y[2 + 2 * image_width] + Q_mat_ptr[9] * y[2 + 3 * image_width] + Q_mat_ptr[10] * y[2 + 4 * image_width] + Q_mat_ptr[11] * y[2 + 5 * image_width]) * Q_mat_ptr[14] + (Q_mat_ptr[6] * y[3] + Q_mat_ptr[7] * y[3 + image_width] + Q_mat_ptr[8] * y[3 + 2 * image_width] + Q_mat_ptr[9] * y[3 + 3 * image_width] + Q_mat_ptr[10] * y[3 + 4 * image_width] + Q_mat_ptr[11] * y[3 + 5 * image_width]) * Q_mat_ptr[15] + (Q_mat_ptr[6] * y[4] + Q_mat_ptr[7] * y[4 + image_width] + Q_mat_ptr[8] * y[4 + 2 * image_width] + Q_mat_ptr[9] * y[4 + 3 * image_width] + Q_mat_ptr[10] * y[4 + 4 * image_width] + Q_mat_ptr[11] * y[4 + 5 * image_width]) * Q_mat_ptr[16] + (Q_mat_ptr[6] * y[5] + Q_mat_ptr[7] * y[5 + image_width] + Q_mat_ptr[8] * y[5 + 2 * image_width] + Q_mat_ptr[9] * y[5 + 3 * image_width] + Q_mat_ptr[10] * y[5 + 4 * image_width] + Q_mat_ptr[11] * y[5 + 5 * image_width]) * Q_mat_ptr[17];
				S_mat_ptr[7] = Q_mat_ptr[6] * y[2] + Q_mat_ptr[7] * y[2 + image_width] + Q_mat_ptr[8] * y[2 + 2 * image_width] + Q_mat_ptr[9] * y[2 + 3 * image_width] + Q_mat_ptr[10] * y[2 + 4 * image_width] + Q_mat_ptr[11] * y[2 + 5 * image_width];
				S_mat_ptr[8] = (Q_mat_ptr[12] * y[0] + Q_mat_ptr[13] * y[image_width] + Q_mat_ptr[14] * y[2 * image_width] + Q_mat_ptr[15] * y[3 * image_width] + Q_mat_ptr[16] * y[4 * image_width] + Q_mat_ptr[17] * y[5 * image_width]) * Q_mat_ptr[0] + (Q_mat_ptr[12] * y[1] + Q_mat_ptr[13] * y[1 + image_width] + Q_mat_ptr[14] * y[1 + 2 * image_width] + Q_mat_ptr[15] * y[1 + 3 * image_width] + Q_mat_ptr[16] * y[1 + 4 * image_width] + Q_mat_ptr[17] * y[1 + 5 * image_width]) * Q_mat_ptr[1] + (Q_mat_ptr[12] * y[2] + Q_mat_ptr[13] * y[2 + image_width] + Q_mat_ptr[14] * y[2 + 2 * image_width] + Q_mat_ptr[15] * y[2 + 3 * image_width] + Q_mat_ptr[16] * y[2 + 4 * image_width] + Q_mat_ptr[17] * y[2 + 5 * image_width]) * Q_mat_ptr[2] + (Q_mat_ptr[12] * y[3] + Q_mat_ptr[13] * y[3 + image_width] + Q_mat_ptr[14] * y[3 + 2 * image_width] + Q_mat_ptr[15] * y[3 + 3 * image_width] + Q_mat_ptr[16] * y[3 + 4 * image_width] + Q_mat_ptr[17] * y[3 + 5 * image_width]) * Q_mat_ptr[3] + (Q_mat_ptr[12] * y[4] + Q_mat_ptr[13] * y[4 + image_width] + Q_mat_ptr[14] * y[4 + 2 * image_width] + Q_mat_ptr[15] * y[4 + 3 * image_width] + Q_mat_ptr[16] * y[4 + 4 * image_width] + Q_mat_ptr[17] * y[4 + 5 * image_width]) * Q_mat_ptr[4] + (Q_mat_ptr[12] * y[5] + Q_mat_ptr[13] * y[5 + image_width] + Q_mat_ptr[14] * y[5 + 2 * image_width] + Q_mat_ptr[15] * y[5 + 3 * image_width] + Q_mat_ptr[16] * y[5 + 4 * image_width] + Q_mat_ptr[17] * y[5 + 5 * image_width]) * Q_mat_ptr[5];
				S_mat_ptr[9] = (Q_mat_ptr[12] * y[0] + Q_mat_ptr[13] * y[image_width] + Q_mat_ptr[14] * y[2 * image_width] + Q_mat_ptr[15] * y[3 * image_width] + Q_mat_ptr[16] * y[4 * image_width] + Q_mat_ptr[17] * y[5 * image_width]) * Q_mat_ptr[6] + (Q_mat_ptr[12] * y[1] + Q_mat_ptr[13] * y[1 + image_width] + Q_mat_ptr[14] * y[1 + 2 * image_width] + Q_mat_ptr[15] * y[1 + 3 * image_width] + Q_mat_ptr[16] * y[1 + 4 * image_width] + Q_mat_ptr[17] * y[1 + 5 * image_width]) * Q_mat_ptr[7] + (Q_mat_ptr[12] * y[2] + Q_mat_ptr[13] * y[2 + image_width] + Q_mat_ptr[14] * y[2 + 2 * image_width] + Q_mat_ptr[15] * y[2 + 3 * image_width] + Q_mat_ptr[16] * y[2 + 4 * image_width] + Q_mat_ptr[17] * y[2 + 5 * image_width]) * Q_mat_ptr[8] + (Q_mat_ptr[12] * y[3] + Q_mat_ptr[13] * y[3 + image_width] + Q_mat_ptr[14] * y[3 + 2 * image_width] + Q_mat_ptr[15] * y[3 + 3 * image_width] + Q_mat_ptr[16] * y[3 + 4 * image_width] + Q_mat_ptr[17] * y[3 + 5 * image_width]) * Q_mat_ptr[9] + (Q_mat_ptr[12] * y[4] + Q_mat_ptr[13] * y[4 + image_width] + Q_mat_ptr[14] * y[4 + 2 * image_width] + Q_mat_ptr[15] * y[4 + 3 * image_width] + Q_mat_ptr[16] * y[4 + 4 * image_width] + Q_mat_ptr[17] * y[4 + 5 * image_width]) * Q_mat_ptr[10] + (Q_mat_ptr[12] * y[5] + Q_mat_ptr[13] * y[5 + image_width] + Q_mat_ptr[14] * y[5 + 2 * image_width] + Q_mat_ptr[15] * y[5 + 3 * image_width] + Q_mat_ptr[16] * y[5 + 4 * image_width] + Q_mat_ptr[17] * y[5 + 5 * image_width]) * Q_mat_ptr[11];
				S_mat_ptr[10] = (Q_mat_ptr[12] * y[0] + Q_mat_ptr[13] * y[image_width] + Q_mat_ptr[14] * y[2 * image_width] + Q_mat_ptr[15] * y[3 * image_width] + Q_mat_ptr[16] * y[4 * image_width] + Q_mat_ptr[17] * y[5 * image_width]) * Q_mat_ptr[12] + (Q_mat_ptr[12] * y[1] + Q_mat_ptr[13] * y[1 + image_width] + Q_mat_ptr[14] * y[1 + 2 * image_width] + Q_mat_ptr[15] * y[1 + 3 * image_width] + Q_mat_ptr[16] * y[1 + 4 * image_width] + Q_mat_ptr[17] * y[1 + 5 * image_width]) * Q_mat_ptr[13] + (Q_mat_ptr[12] * y[2] + Q_mat_ptr[13] * y[2 + image_width] + Q_mat_ptr[14] * y[2 + 2 * image_width] + Q_mat_ptr[15] * y[2 + 3 * image_width] + Q_mat_ptr[16] * y[2 + 4 * image_width] + Q_mat_ptr[17] * y[2 + 5 * image_width]) * Q_mat_ptr[14] + (Q_mat_ptr[12] * y[3] + Q_mat_ptr[13] * y[3 + image_width] + Q_mat_ptr[14] * y[3 + 2 * image_width] + Q_mat_ptr[15] * y[3 + 3 * image_width] + Q_mat_ptr[16] * y[3 + 4 * image_width] + Q_mat_ptr[17] * y[3 + 5 * image_width]) * Q_mat_ptr[15] + (Q_mat_ptr[12] * y[4] + Q_mat_ptr[13] * y[4 + image_width] + Q_mat_ptr[14] * y[4 + 2 * image_width] + Q_mat_ptr[15] * y[4 + 3 * image_width] + Q_mat_ptr[16] * y[4 + 4 * image_width] + Q_mat_ptr[17] * y[4 + 5 * image_width]) * Q_mat_ptr[16] + (Q_mat_ptr[12] * y[5] + Q_mat_ptr[13] * y[5 + image_width] + Q_mat_ptr[14] * y[5 + 2 * image_width] + Q_mat_ptr[15] * y[5 + 3 * image_width] + Q_mat_ptr[16] * y[5 + 4 * image_width] + Q_mat_ptr[17] * y[5 + 5 * image_width]) * Q_mat_ptr[17];
				S_mat_ptr[11] = Q_mat_ptr[12] * y[2] + Q_mat_ptr[13] * y[2 + image_width] + Q_mat_ptr[14] * y[2 + 2 * image_width] + Q_mat_ptr[15] * y[2 + 3 * image_width] + Q_mat_ptr[16] * y[2 + 4 * image_width] + Q_mat_ptr[17] * y[2 + 5 * image_width];
				S_mat_ptr[12] = y[2 * image_width] * Q_mat_ptr[0] + y[1 + 2 * image_width] * Q_mat_ptr[1] + Q_mat_ptr[2] * y[2 + 2 * image_width] + y[3 + 2 * image_width] * Q_mat_ptr[3] + y[4 + 2 * image_width] * Q_mat_ptr[4] + y[5 + 2 * image_width] * Q_mat_ptr[5];
				S_mat_ptr[13] = y[2 * image_width] * Q_mat_ptr[6] + y[1 + 2 * image_width] * Q_mat_ptr[7] + Q_mat_ptr[8] * y[2 + 2 * image_width] + y[3 + 2 * image_width] * Q_mat_ptr[9] + y[4 + 2 * image_width] * Q_mat_ptr[10] + y[5 + 2 * image_width] * Q_mat_ptr[11];
				S_mat_ptr[14] = y[2 * image_width] * Q_mat_ptr[12] + y[1 + 2 * image_width] * Q_mat_ptr[13] + Q_mat_ptr[14] * y[2 + 2 * image_width] + y[3 + 2 * image_width] * Q_mat_ptr[15] + y[4 + 2 * image_width] * Q_mat_ptr[16] + y[5 + 2 * image_width] * Q_mat_ptr[17];
				S_mat_ptr[15] = y[2 + 2 * image_width];
				CV_MAT_ELEM(*interpolation_LUT_valid_sign_mat,uchar,index_y - interpolation_area_y_min,index_x - interpolation_area_x_min) =1;		
			}
			//g
			CV_MAT_ELEM(*g_mat,double,m+template_half_height,n+template_half_width) =
				delta_x*delta_x*delta_x * (delta_y*delta_y*delta_y * S_mat_ptr[0] + delta_y*delta_y * S_mat_ptr[4] + delta_y * S_mat_ptr[8] + S_mat_ptr[12] ) +
				delta_x*delta_x * (delta_y*delta_y*delta_y * S_mat_ptr[1] + delta_y*delta_y * S_mat_ptr[5] + delta_y * S_mat_ptr[9] + S_mat_ptr[13] ) +
				delta_x  * (delta_y*delta_y*delta_y * S_mat_ptr[2] + delta_y*delta_y * S_mat_ptr[6] + delta_y * S_mat_ptr[10] + S_mat_ptr[14] ) +
				(delta_y*delta_y*delta_y * S_mat_ptr[3] + delta_y*delta_y * S_mat_ptr[7] + delta_y * S_mat_ptr[11] + S_mat_ptr[15] );

			//CV_MAT_ELEM(*g_mat,double,m+template_half_height,n+template_half_width) =
			//	3.0 * delta_x*delta_x * (delta_y*delta_y*delta_y * S_mat_ptr[0] + delta_y*delta_y * S_mat_ptr[4] + delta_y * S_mat_ptr[8] + S_mat_ptr[12] ) +
			//	2.0 * delta_x  * (delta_y*delta_y*delta_y * S_mat_ptr[1] + delta_y*delta_y * S_mat_ptr[5] + delta_y * S_mat_ptr[9] + S_mat_ptr[13] ) +
			//	(delta_y*delta_y*delta_y * S_mat_ptr[2] + delta_y*delta_y * S_mat_ptr[6] + delta_y * S_mat_ptr[10] + S_mat_ptr[14] );
			//CV_MAT_ELEM(*g_mat,double,m+template_half_height,n+template_half_width) =
			//	delta_x*delta_x*delta_x * (3.0 * delta_y*delta_y * S_mat_ptr[0] + 2.0 * delta_y * S_mat_ptr[4] + S_mat_ptr[8] ) +
			//	delta_x*delta_x * (3.0 * delta_y*delta_y * S_mat_ptr[1] +  2.0 * delta_y * S_mat_ptr[5] + S_mat_ptr[9] ) +
			//	delta_x  * (3.0 * delta_y*delta_y * S_mat_ptr[2] +  2.0 * delta_y * S_mat_ptr[6] + S_mat_ptr[10] ) +
			//	(3.0 * delta_y*delta_y * S_mat_ptr[3] +  2.0 * delta_y * S_mat_ptr[7] + S_mat_ptr[11] );
		}
	}
	

	return true;
}


bool _6x6UniformBicubicSplineInterpolationMethod::Reference_Value_And_Gradient_One_Template_64F_20130531_Before(
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
	)
{
	//判断省略
	//大小
	int image_width= image->width;
	int image_height = image->height;
	int template_width = g_mat->cols;
	int template_height = g_mat->cols;
	int lut_width = interpolation_LUT->cols;
	int lut_height = interpolation_LUT->rows;

	//if (displacement_input->rows!=6 && displacement_input->cols!=1)
	//{
	//	return false;
	//} 

	//if (interpolation_area_x_min<0 || interpolation_area_x_max>=image_width||
	//	interpolation_area_y_min<0 || interpolation_area_y_max>=image_height)
	//{
	//	return false;
	//}
	int interpolation_area_width = interpolation_area_x_max-interpolation_area_x_min;
	int interpolation_area_height = interpolation_area_y_max-interpolation_area_y_min;

	//if (interpolation_LUT->cols != interpolation_area_width*Interpolation_LUT_Element_Size||
	//	interpolation_LUT->rows !=interpolation_area_height||
	//	interpolation_LUT_valid_sign_mat->cols!=interpolation_area_width||
	//	interpolation_LUT_valid_sign_mat->rows!= interpolation_area_height)
	//{
	//	return false;
	//}

	//if (uniform_bicubic_natrue_spline_interpolation_6x6_Q_mat->rows!=4 || uniform_bicubic_natrue_spline_interpolation_6x6_Q_mat->cols!=6)
	//{
	//	return false;
	//}



	////类型
	//if (CV_MAT_TYPE(image->type) !=CV_64F)
	//{
	//	return false;
	//}

	//if (CV_MAT_TYPE(interpolation_LUT->type) != CV_32F)
	//{
	//	return false;
	//}


	//if ( CV_MAT_TYPE(displacement_input->type) != CV_64F)
	//{
	//	return false;
	//}
	//if (CV_MAT_TYPE(g_mat->type) != CV_64F || CV_MAT_TYPE(uniform_bicubic_natrue_spline_interpolation_6x6_Q_mat->type) != CV_64F || CV_MAT_TYPE(interpolation_LUT_valid_sign_mat->type)!=CV_8U)
	//{
	//	return false;
	//}

	int template_half_width = (template_width-1)/2;
	int template_half_height = (template_height-1)/2;
	double* Q_mat_ptr = uniform_bicubic_natrue_spline_interpolation_6x6_Q_mat->data.db;




	for (int m =-template_half_height;m<=template_half_height;m++)
	{
		for (int n =-template_half_width;n<=template_half_width;n++)
		{
			//第一种
			double pos_x =reference_point_x + (double)n;
			double pos_y =reference_point_y + (double)m;


			int index_x = cvFloor(pos_x);
			int index_y = cvFloor(pos_y);
			if (index_y - interpolation_area_y_min <0 || index_y - interpolation_area_y_min >= interpolation_LUT_valid_sign_mat->rows-1 ||
				index_x - interpolation_area_x_min <0 || index_x - interpolation_area_x_min >= interpolation_LUT_valid_sign_mat->cols-1)
			{
				return false;
			}

			double delta_x = pos_x - index_x;
			double delta_y = pos_y - index_y;

			uchar sign = CV_MAT_ELEM(*interpolation_LUT_valid_sign_mat,uchar,index_y - interpolation_area_y_min,index_x - interpolation_area_x_min);
			float* S_mat_ptr = interpolation_LUT->data.fl+ ((index_y - interpolation_area_y_min) * (interpolation_area_x_max-interpolation_area_x_min)+index_x - interpolation_area_x_min) * Interpolation_LUT_Element_Size;
			if (sign!=1)
			{							
				double* y = image->data.db + (index_y-2)*image_width+index_x-2;
				S_mat_ptr[0] = (Q_mat_ptr[0] * y[0] + Q_mat_ptr[1] * y[image_width] + Q_mat_ptr[2] * y[2 * image_width] + Q_mat_ptr[3] * y[3 * image_width] + Q_mat_ptr[4] * y[4 * image_width] + Q_mat_ptr[5] * y[5 * image_width]) * Q_mat_ptr[0] + (Q_mat_ptr[0] * y[1] + Q_mat_ptr[1] * y[1 + image_width] + Q_mat_ptr[2] * y[1 + 2 * image_width] + Q_mat_ptr[3] * y[1 + 3 * image_width] + Q_mat_ptr[4] * y[1 + 4 * image_width] + Q_mat_ptr[5] * y[1 + 5 * image_width]) * Q_mat_ptr[1] + (Q_mat_ptr[0] * y[2] + Q_mat_ptr[1] * y[2 + image_width] + Q_mat_ptr[2] * y[2 + 2 * image_width] + Q_mat_ptr[3] * y[2 + 3 * image_width] + Q_mat_ptr[4] * y[2 + 4 * image_width] + Q_mat_ptr[5] * y[2 + 5 * image_width]) * Q_mat_ptr[2] + (Q_mat_ptr[0] * y[3] + Q_mat_ptr[1] * y[3 + image_width] + Q_mat_ptr[2] * y[3 + 2 * image_width] + Q_mat_ptr[3] * y[3 + 3 * image_width] + Q_mat_ptr[4] * y[3 + 4 * image_width] + Q_mat_ptr[5] * y[3 + 5 * image_width]) * Q_mat_ptr[3] + (Q_mat_ptr[0] * y[4] + Q_mat_ptr[1] * y[4 + image_width] + Q_mat_ptr[2] * y[4 + 2 * image_width] + Q_mat_ptr[3] * y[4 + 3 * image_width] + Q_mat_ptr[4] * y[4 + 4 * image_width] + Q_mat_ptr[5] * y[4 + 5 * image_width]) * Q_mat_ptr[4] + (Q_mat_ptr[0] * y[5] + Q_mat_ptr[1] * y[5 + image_width] + Q_mat_ptr[2] * y[5 + 2 * image_width] + Q_mat_ptr[3] * y[5 + 3 * image_width] + Q_mat_ptr[4] * y[5 + 4 * image_width] + Q_mat_ptr[5] * y[5 + 5 * image_width]) * Q_mat_ptr[5];
				S_mat_ptr[1] = (Q_mat_ptr[0] * y[0] + Q_mat_ptr[1] * y[image_width] + Q_mat_ptr[2] * y[2 * image_width] + Q_mat_ptr[3] * y[3 * image_width] + Q_mat_ptr[4] * y[4 * image_width] + Q_mat_ptr[5] * y[5 * image_width]) * Q_mat_ptr[6] + (Q_mat_ptr[0] * y[1] + Q_mat_ptr[1] * y[1 + image_width] + Q_mat_ptr[2] * y[1 + 2 * image_width] + Q_mat_ptr[3] * y[1 + 3 * image_width] + Q_mat_ptr[4] * y[1 + 4 * image_width] + Q_mat_ptr[5] * y[1 + 5 * image_width]) * Q_mat_ptr[7] + (Q_mat_ptr[0] * y[2] + Q_mat_ptr[1] * y[2 + image_width] + Q_mat_ptr[2] * y[2 + 2 * image_width] + Q_mat_ptr[3] * y[2 + 3 * image_width] + Q_mat_ptr[4] * y[2 + 4 * image_width] + Q_mat_ptr[5] * y[2 + 5 * image_width]) * Q_mat_ptr[8] + (Q_mat_ptr[0] * y[3] + Q_mat_ptr[1] * y[3 + image_width] + Q_mat_ptr[2] * y[3 + 2 * image_width] + Q_mat_ptr[3] * y[3 + 3 * image_width] + Q_mat_ptr[4] * y[3 + 4 * image_width] + Q_mat_ptr[5] * y[3 + 5 * image_width]) * Q_mat_ptr[9] + (Q_mat_ptr[0] * y[4] + Q_mat_ptr[1] * y[4 + image_width] + Q_mat_ptr[2] * y[4 + 2 * image_width] + Q_mat_ptr[3] * y[4 + 3 * image_width] + Q_mat_ptr[4] * y[4 + 4 * image_width] + Q_mat_ptr[5] * y[4 + 5 * image_width]) * Q_mat_ptr[10] + (Q_mat_ptr[0] * y[5] + Q_mat_ptr[1] * y[5 + image_width] + Q_mat_ptr[2] * y[5 + 2 * image_width] + Q_mat_ptr[3] * y[5 + 3 * image_width] + Q_mat_ptr[4] * y[5 + 4 * image_width] + Q_mat_ptr[5] * y[5 + 5 * image_width]) * Q_mat_ptr[11];
				S_mat_ptr[2] = (Q_mat_ptr[0] * y[0] + Q_mat_ptr[1] * y[image_width] + Q_mat_ptr[2] * y[2 * image_width] + Q_mat_ptr[3] * y[3 * image_width] + Q_mat_ptr[4] * y[4 * image_width] + Q_mat_ptr[5] * y[5 * image_width]) * Q_mat_ptr[12] + (Q_mat_ptr[0] * y[1] + Q_mat_ptr[1] * y[1 + image_width] + Q_mat_ptr[2] * y[1 + 2 * image_width] + Q_mat_ptr[3] * y[1 + 3 * image_width] + Q_mat_ptr[4] * y[1 + 4 * image_width] + Q_mat_ptr[5] * y[1 + 5 * image_width]) * Q_mat_ptr[13] + (Q_mat_ptr[0] * y[2] + Q_mat_ptr[1] * y[2 + image_width] + Q_mat_ptr[2] * y[2 + 2 * image_width] + Q_mat_ptr[3] * y[2 + 3 * image_width] + Q_mat_ptr[4] * y[2 + 4 * image_width] + Q_mat_ptr[5] * y[2 + 5 * image_width]) * Q_mat_ptr[14] + (Q_mat_ptr[0] * y[3] + Q_mat_ptr[1] * y[3 + image_width] + Q_mat_ptr[2] * y[3 + 2 * image_width] + Q_mat_ptr[3] * y[3 + 3 * image_width] + Q_mat_ptr[4] * y[3 + 4 * image_width] + Q_mat_ptr[5] * y[3 + 5 * image_width]) * Q_mat_ptr[15] + (Q_mat_ptr[0] * y[4] + Q_mat_ptr[1] * y[4 + image_width] + Q_mat_ptr[2] * y[4 + 2 * image_width] + Q_mat_ptr[3] * y[4 + 3 * image_width] + Q_mat_ptr[4] * y[4 + 4 * image_width] + Q_mat_ptr[5] * y[4 + 5 * image_width]) * Q_mat_ptr[16] + (Q_mat_ptr[0] * y[5] + Q_mat_ptr[1] * y[5 + image_width] + Q_mat_ptr[2] * y[5 + 2 * image_width] + Q_mat_ptr[3] * y[5 + 3 * image_width] + Q_mat_ptr[4] * y[5 + 4 * image_width] + Q_mat_ptr[5] * y[5 + 5 * image_width]) * Q_mat_ptr[17];
				S_mat_ptr[3] = Q_mat_ptr[0] * y[2] + Q_mat_ptr[1] * y[2 + image_width] + Q_mat_ptr[2] * y[2 + 2 * image_width] + Q_mat_ptr[3] * y[2 + 3 * image_width] + Q_mat_ptr[4] * y[2 + 4 * image_width] + Q_mat_ptr[5] * y[2 + 5 * image_width];
				S_mat_ptr[4] = (Q_mat_ptr[6] * y[0] + Q_mat_ptr[7] * y[image_width] + Q_mat_ptr[8] * y[2 * image_width] + Q_mat_ptr[9] * y[3 * image_width] + Q_mat_ptr[10] * y[4 * image_width] + Q_mat_ptr[11] * y[5 * image_width]) * Q_mat_ptr[0] + (Q_mat_ptr[6] * y[1] + Q_mat_ptr[7] * y[1 + image_width] + Q_mat_ptr[8] * y[1 + 2 * image_width] + Q_mat_ptr[9] * y[1 + 3 * image_width] + Q_mat_ptr[10] * y[1 + 4 * image_width] + Q_mat_ptr[11] * y[1 + 5 * image_width]) * Q_mat_ptr[1] + (Q_mat_ptr[6] * y[2] + Q_mat_ptr[7] * y[2 + image_width] + Q_mat_ptr[8] * y[2 + 2 * image_width] + Q_mat_ptr[9] * y[2 + 3 * image_width] + Q_mat_ptr[10] * y[2 + 4 * image_width] + Q_mat_ptr[11] * y[2 + 5 * image_width]) * Q_mat_ptr[2] + (Q_mat_ptr[6] * y[3] + Q_mat_ptr[7] * y[3 + image_width] + Q_mat_ptr[8] * y[3 + 2 * image_width] + Q_mat_ptr[9] * y[3 + 3 * image_width] + Q_mat_ptr[10] * y[3 + 4 * image_width] + Q_mat_ptr[11] * y[3 + 5 * image_width]) * Q_mat_ptr[3] + (Q_mat_ptr[6] * y[4] + Q_mat_ptr[7] * y[4 + image_width] + Q_mat_ptr[8] * y[4 + 2 * image_width] + Q_mat_ptr[9] * y[4 + 3 * image_width] + Q_mat_ptr[10] * y[4 + 4 * image_width] + Q_mat_ptr[11] * y[4 + 5 * image_width]) * Q_mat_ptr[4] + (Q_mat_ptr[6] * y[5] + Q_mat_ptr[7] * y[5 + image_width] + Q_mat_ptr[8] * y[5 + 2 * image_width] + Q_mat_ptr[9] * y[5 + 3 * image_width] + Q_mat_ptr[10] * y[5 + 4 * image_width] + Q_mat_ptr[11] * y[5 + 5 * image_width]) * Q_mat_ptr[5];
				S_mat_ptr[5] = (Q_mat_ptr[6] * y[0] + Q_mat_ptr[7] * y[image_width] + Q_mat_ptr[8] * y[2 * image_width] + Q_mat_ptr[9] * y[3 * image_width] + Q_mat_ptr[10] * y[4 * image_width] + Q_mat_ptr[11] * y[5 * image_width]) * Q_mat_ptr[6] + (Q_mat_ptr[6] * y[1] + Q_mat_ptr[7] * y[1 + image_width] + Q_mat_ptr[8] * y[1 + 2 * image_width] + Q_mat_ptr[9] * y[1 + 3 * image_width] + Q_mat_ptr[10] * y[1 + 4 * image_width] + Q_mat_ptr[11] * y[1 + 5 * image_width]) * Q_mat_ptr[7] + (Q_mat_ptr[6] * y[2] + Q_mat_ptr[7] * y[2 + image_width] + Q_mat_ptr[8] * y[2 + 2 * image_width] + Q_mat_ptr[9] * y[2 + 3 * image_width] + Q_mat_ptr[10] * y[2 + 4 * image_width] + Q_mat_ptr[11] * y[2 + 5 * image_width]) * Q_mat_ptr[8] + (Q_mat_ptr[6] * y[3] + Q_mat_ptr[7] * y[3 + image_width] + Q_mat_ptr[8] * y[3 + 2 * image_width] + Q_mat_ptr[9] * y[3 + 3 * image_width] + Q_mat_ptr[10] * y[3 + 4 * image_width] + Q_mat_ptr[11] * y[3 + 5 * image_width]) * Q_mat_ptr[9] + (Q_mat_ptr[6] * y[4] + Q_mat_ptr[7] * y[4 + image_width] + Q_mat_ptr[8] * y[4 + 2 * image_width] + Q_mat_ptr[9] * y[4 + 3 * image_width] + Q_mat_ptr[10] * y[4 + 4 * image_width] + Q_mat_ptr[11] * y[4 + 5 * image_width]) * Q_mat_ptr[10] + (Q_mat_ptr[6] * y[5] + Q_mat_ptr[7] * y[5 + image_width] + Q_mat_ptr[8] * y[5 + 2 * image_width] + Q_mat_ptr[9] * y[5 + 3 * image_width] + Q_mat_ptr[10] * y[5 + 4 * image_width] + Q_mat_ptr[11] * y[5 + 5 * image_width]) * Q_mat_ptr[11];
				S_mat_ptr[6] = (Q_mat_ptr[6] * y[0] + Q_mat_ptr[7] * y[image_width] + Q_mat_ptr[8] * y[2 * image_width] + Q_mat_ptr[9] * y[3 * image_width] + Q_mat_ptr[10] * y[4 * image_width] + Q_mat_ptr[11] * y[5 * image_width]) * Q_mat_ptr[12] + (Q_mat_ptr[6] * y[1] + Q_mat_ptr[7] * y[1 + image_width] + Q_mat_ptr[8] * y[1 + 2 * image_width] + Q_mat_ptr[9] * y[1 + 3 * image_width] + Q_mat_ptr[10] * y[1 + 4 * image_width] + Q_mat_ptr[11] * y[1 + 5 * image_width]) * Q_mat_ptr[13] + (Q_mat_ptr[6] * y[2] + Q_mat_ptr[7] * y[2 + image_width] + Q_mat_ptr[8] * y[2 + 2 * image_width] + Q_mat_ptr[9] * y[2 + 3 * image_width] + Q_mat_ptr[10] * y[2 + 4 * image_width] + Q_mat_ptr[11] * y[2 + 5 * image_width]) * Q_mat_ptr[14] + (Q_mat_ptr[6] * y[3] + Q_mat_ptr[7] * y[3 + image_width] + Q_mat_ptr[8] * y[3 + 2 * image_width] + Q_mat_ptr[9] * y[3 + 3 * image_width] + Q_mat_ptr[10] * y[3 + 4 * image_width] + Q_mat_ptr[11] * y[3 + 5 * image_width]) * Q_mat_ptr[15] + (Q_mat_ptr[6] * y[4] + Q_mat_ptr[7] * y[4 + image_width] + Q_mat_ptr[8] * y[4 + 2 * image_width] + Q_mat_ptr[9] * y[4 + 3 * image_width] + Q_mat_ptr[10] * y[4 + 4 * image_width] + Q_mat_ptr[11] * y[4 + 5 * image_width]) * Q_mat_ptr[16] + (Q_mat_ptr[6] * y[5] + Q_mat_ptr[7] * y[5 + image_width] + Q_mat_ptr[8] * y[5 + 2 * image_width] + Q_mat_ptr[9] * y[5 + 3 * image_width] + Q_mat_ptr[10] * y[5 + 4 * image_width] + Q_mat_ptr[11] * y[5 + 5 * image_width]) * Q_mat_ptr[17];
				S_mat_ptr[7] = Q_mat_ptr[6] * y[2] + Q_mat_ptr[7] * y[2 + image_width] + Q_mat_ptr[8] * y[2 + 2 * image_width] + Q_mat_ptr[9] * y[2 + 3 * image_width] + Q_mat_ptr[10] * y[2 + 4 * image_width] + Q_mat_ptr[11] * y[2 + 5 * image_width];
				S_mat_ptr[8] = (Q_mat_ptr[12] * y[0] + Q_mat_ptr[13] * y[image_width] + Q_mat_ptr[14] * y[2 * image_width] + Q_mat_ptr[15] * y[3 * image_width] + Q_mat_ptr[16] * y[4 * image_width] + Q_mat_ptr[17] * y[5 * image_width]) * Q_mat_ptr[0] + (Q_mat_ptr[12] * y[1] + Q_mat_ptr[13] * y[1 + image_width] + Q_mat_ptr[14] * y[1 + 2 * image_width] + Q_mat_ptr[15] * y[1 + 3 * image_width] + Q_mat_ptr[16] * y[1 + 4 * image_width] + Q_mat_ptr[17] * y[1 + 5 * image_width]) * Q_mat_ptr[1] + (Q_mat_ptr[12] * y[2] + Q_mat_ptr[13] * y[2 + image_width] + Q_mat_ptr[14] * y[2 + 2 * image_width] + Q_mat_ptr[15] * y[2 + 3 * image_width] + Q_mat_ptr[16] * y[2 + 4 * image_width] + Q_mat_ptr[17] * y[2 + 5 * image_width]) * Q_mat_ptr[2] + (Q_mat_ptr[12] * y[3] + Q_mat_ptr[13] * y[3 + image_width] + Q_mat_ptr[14] * y[3 + 2 * image_width] + Q_mat_ptr[15] * y[3 + 3 * image_width] + Q_mat_ptr[16] * y[3 + 4 * image_width] + Q_mat_ptr[17] * y[3 + 5 * image_width]) * Q_mat_ptr[3] + (Q_mat_ptr[12] * y[4] + Q_mat_ptr[13] * y[4 + image_width] + Q_mat_ptr[14] * y[4 + 2 * image_width] + Q_mat_ptr[15] * y[4 + 3 * image_width] + Q_mat_ptr[16] * y[4 + 4 * image_width] + Q_mat_ptr[17] * y[4 + 5 * image_width]) * Q_mat_ptr[4] + (Q_mat_ptr[12] * y[5] + Q_mat_ptr[13] * y[5 + image_width] + Q_mat_ptr[14] * y[5 + 2 * image_width] + Q_mat_ptr[15] * y[5 + 3 * image_width] + Q_mat_ptr[16] * y[5 + 4 * image_width] + Q_mat_ptr[17] * y[5 + 5 * image_width]) * Q_mat_ptr[5];
				S_mat_ptr[9] = (Q_mat_ptr[12] * y[0] + Q_mat_ptr[13] * y[image_width] + Q_mat_ptr[14] * y[2 * image_width] + Q_mat_ptr[15] * y[3 * image_width] + Q_mat_ptr[16] * y[4 * image_width] + Q_mat_ptr[17] * y[5 * image_width]) * Q_mat_ptr[6] + (Q_mat_ptr[12] * y[1] + Q_mat_ptr[13] * y[1 + image_width] + Q_mat_ptr[14] * y[1 + 2 * image_width] + Q_mat_ptr[15] * y[1 + 3 * image_width] + Q_mat_ptr[16] * y[1 + 4 * image_width] + Q_mat_ptr[17] * y[1 + 5 * image_width]) * Q_mat_ptr[7] + (Q_mat_ptr[12] * y[2] + Q_mat_ptr[13] * y[2 + image_width] + Q_mat_ptr[14] * y[2 + 2 * image_width] + Q_mat_ptr[15] * y[2 + 3 * image_width] + Q_mat_ptr[16] * y[2 + 4 * image_width] + Q_mat_ptr[17] * y[2 + 5 * image_width]) * Q_mat_ptr[8] + (Q_mat_ptr[12] * y[3] + Q_mat_ptr[13] * y[3 + image_width] + Q_mat_ptr[14] * y[3 + 2 * image_width] + Q_mat_ptr[15] * y[3 + 3 * image_width] + Q_mat_ptr[16] * y[3 + 4 * image_width] + Q_mat_ptr[17] * y[3 + 5 * image_width]) * Q_mat_ptr[9] + (Q_mat_ptr[12] * y[4] + Q_mat_ptr[13] * y[4 + image_width] + Q_mat_ptr[14] * y[4 + 2 * image_width] + Q_mat_ptr[15] * y[4 + 3 * image_width] + Q_mat_ptr[16] * y[4 + 4 * image_width] + Q_mat_ptr[17] * y[4 + 5 * image_width]) * Q_mat_ptr[10] + (Q_mat_ptr[12] * y[5] + Q_mat_ptr[13] * y[5 + image_width] + Q_mat_ptr[14] * y[5 + 2 * image_width] + Q_mat_ptr[15] * y[5 + 3 * image_width] + Q_mat_ptr[16] * y[5 + 4 * image_width] + Q_mat_ptr[17] * y[5 + 5 * image_width]) * Q_mat_ptr[11];
				S_mat_ptr[10] = (Q_mat_ptr[12] * y[0] + Q_mat_ptr[13] * y[image_width] + Q_mat_ptr[14] * y[2 * image_width] + Q_mat_ptr[15] * y[3 * image_width] + Q_mat_ptr[16] * y[4 * image_width] + Q_mat_ptr[17] * y[5 * image_width]) * Q_mat_ptr[12] + (Q_mat_ptr[12] * y[1] + Q_mat_ptr[13] * y[1 + image_width] + Q_mat_ptr[14] * y[1 + 2 * image_width] + Q_mat_ptr[15] * y[1 + 3 * image_width] + Q_mat_ptr[16] * y[1 + 4 * image_width] + Q_mat_ptr[17] * y[1 + 5 * image_width]) * Q_mat_ptr[13] + (Q_mat_ptr[12] * y[2] + Q_mat_ptr[13] * y[2 + image_width] + Q_mat_ptr[14] * y[2 + 2 * image_width] + Q_mat_ptr[15] * y[2 + 3 * image_width] + Q_mat_ptr[16] * y[2 + 4 * image_width] + Q_mat_ptr[17] * y[2 + 5 * image_width]) * Q_mat_ptr[14] + (Q_mat_ptr[12] * y[3] + Q_mat_ptr[13] * y[3 + image_width] + Q_mat_ptr[14] * y[3 + 2 * image_width] + Q_mat_ptr[15] * y[3 + 3 * image_width] + Q_mat_ptr[16] * y[3 + 4 * image_width] + Q_mat_ptr[17] * y[3 + 5 * image_width]) * Q_mat_ptr[15] + (Q_mat_ptr[12] * y[4] + Q_mat_ptr[13] * y[4 + image_width] + Q_mat_ptr[14] * y[4 + 2 * image_width] + Q_mat_ptr[15] * y[4 + 3 * image_width] + Q_mat_ptr[16] * y[4 + 4 * image_width] + Q_mat_ptr[17] * y[4 + 5 * image_width]) * Q_mat_ptr[16] + (Q_mat_ptr[12] * y[5] + Q_mat_ptr[13] * y[5 + image_width] + Q_mat_ptr[14] * y[5 + 2 * image_width] + Q_mat_ptr[15] * y[5 + 3 * image_width] + Q_mat_ptr[16] * y[5 + 4 * image_width] + Q_mat_ptr[17] * y[5 + 5 * image_width]) * Q_mat_ptr[17];
				S_mat_ptr[11] = Q_mat_ptr[12] * y[2] + Q_mat_ptr[13] * y[2 + image_width] + Q_mat_ptr[14] * y[2 + 2 * image_width] + Q_mat_ptr[15] * y[2 + 3 * image_width] + Q_mat_ptr[16] * y[2 + 4 * image_width] + Q_mat_ptr[17] * y[2 + 5 * image_width];
				S_mat_ptr[12] = y[2 * image_width] * Q_mat_ptr[0] + y[1 + 2 * image_width] * Q_mat_ptr[1] + Q_mat_ptr[2] * y[2 + 2 * image_width] + y[3 + 2 * image_width] * Q_mat_ptr[3] + y[4 + 2 * image_width] * Q_mat_ptr[4] + y[5 + 2 * image_width] * Q_mat_ptr[5];
				S_mat_ptr[13] = y[2 * image_width] * Q_mat_ptr[6] + y[1 + 2 * image_width] * Q_mat_ptr[7] + Q_mat_ptr[8] * y[2 + 2 * image_width] + y[3 + 2 * image_width] * Q_mat_ptr[9] + y[4 + 2 * image_width] * Q_mat_ptr[10] + y[5 + 2 * image_width] * Q_mat_ptr[11];
				S_mat_ptr[14] = y[2 * image_width] * Q_mat_ptr[12] + y[1 + 2 * image_width] * Q_mat_ptr[13] + Q_mat_ptr[14] * y[2 + 2 * image_width] + y[3 + 2 * image_width] * Q_mat_ptr[15] + y[4 + 2 * image_width] * Q_mat_ptr[16] + y[5 + 2 * image_width] * Q_mat_ptr[17];
				S_mat_ptr[15] = y[2 + 2 * image_width];
				CV_MAT_ELEM(*interpolation_LUT_valid_sign_mat,uchar,index_y - interpolation_area_y_min,index_x - interpolation_area_x_min) =1;		
			}
			//g
			CV_MAT_ELEM(*g_mat,double,m+template_half_height,n+template_half_width) =
				delta_x*delta_x*delta_x * (delta_y*delta_y*delta_y * S_mat_ptr[0] + delta_y*delta_y * S_mat_ptr[4] + delta_y * S_mat_ptr[8] + S_mat_ptr[12] ) +
				delta_x*delta_x * (delta_y*delta_y*delta_y * S_mat_ptr[1] + delta_y*delta_y * S_mat_ptr[5] + delta_y * S_mat_ptr[9] + S_mat_ptr[13] ) +
				delta_x  * (delta_y*delta_y*delta_y * S_mat_ptr[2] + delta_y*delta_y * S_mat_ptr[6] + delta_y * S_mat_ptr[10] + S_mat_ptr[14] ) +
				(delta_y*delta_y*delta_y * S_mat_ptr[3] + delta_y*delta_y * S_mat_ptr[7] + delta_y * S_mat_ptr[11] + S_mat_ptr[15] );

			CV_MAT_ELEM(*gx_mat,double,m+template_half_height,n+template_half_width) =
				3.0 * delta_x*delta_x * (delta_y*delta_y*delta_y * S_mat_ptr[0] + delta_y*delta_y * S_mat_ptr[4] + delta_y * S_mat_ptr[8] + S_mat_ptr[12] ) +
				2.0 * delta_x  * (delta_y*delta_y*delta_y * S_mat_ptr[1] + delta_y*delta_y * S_mat_ptr[5] + delta_y * S_mat_ptr[9] + S_mat_ptr[13] ) +
				(delta_y*delta_y*delta_y * S_mat_ptr[2] + delta_y*delta_y * S_mat_ptr[6] + delta_y * S_mat_ptr[10] + S_mat_ptr[14] );
			CV_MAT_ELEM(*gy_mat,double,m+template_half_height,n+template_half_width) =
				delta_x*delta_x*delta_x * (3.0 * delta_y*delta_y * S_mat_ptr[0] + 2.0 * delta_y * S_mat_ptr[4] + S_mat_ptr[8] ) +
				delta_x*delta_x * (3.0 * delta_y*delta_y * S_mat_ptr[1] +  2.0 * delta_y * S_mat_ptr[5] + S_mat_ptr[9] ) +
				delta_x  * (3.0 * delta_y*delta_y * S_mat_ptr[2] +  2.0 * delta_y * S_mat_ptr[6] + S_mat_ptr[10] ) +
				(3.0 * delta_y*delta_y * S_mat_ptr[3] +  2.0 * delta_y * S_mat_ptr[7] + S_mat_ptr[11] );
		}
	}


	return true;
}

bool _6x6UniformBicubicSplineInterpolationMethod::Value_And_Gradient_One_Affine_Template_64F(
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
	)
{
	//判断
	//大小
	int image_width= image->width;
	int image_height = image->height;
	int template_width = g_mat->cols;
	int template_height = g_mat->cols;
	int lut_width = interpolation_LUT->cols;
	int lut_height = interpolation_LUT->rows;


	if (interpolation_area_x_min<0 || interpolation_area_x_max>=image_width||
		interpolation_area_y_min<0 || interpolation_area_y_max>=image_height)
	{
		return false;
	}
	int interpolation_area_width = interpolation_area_x_max-interpolation_area_x_min;
	int interpolation_area_height = interpolation_area_y_max-interpolation_area_y_min;

	if (interpolation_LUT->cols != interpolation_area_width*Interpolation_LUT_Element_Size||
		interpolation_LUT->rows !=interpolation_area_height||
		interpolation_LUT_valid_sign_mat->cols!=interpolation_area_width||
		interpolation_LUT_valid_sign_mat->rows!= interpolation_area_height)
	{
		return false;
	}

	if (uniform_bicubic_natrue_spline_interpolation_6x6_Q_mat->rows!=4 || uniform_bicubic_natrue_spline_interpolation_6x6_Q_mat->cols!=6)
	{
		return false;
	}



	//类型
	if (CV_MAT_TYPE(image->type) !=CV_64F)
	{
		return false;
	}

	if (CV_MAT_TYPE(interpolation_LUT->type) != CV_32F)
	{
		return false;
	}

	if (CV_MAT_TYPE(g_mat->type) != CV_64F || CV_MAT_TYPE(uniform_bicubic_natrue_spline_interpolation_6x6_Q_mat->type) != CV_64F || CV_MAT_TYPE(interpolation_LUT_valid_sign_mat->type)!=CV_8U)
	{
		return false;
	}

	int template_half_width = (template_width-1)/2;
	int template_half_height = (template_height-1)/2;
	double* Q_mat_ptr = uniform_bicubic_natrue_spline_interpolation_6x6_Q_mat->data.db;




	for (int m =-template_half_height;m<=template_half_height;m++)
	{
		for (int n =-template_half_width;n<=template_half_width;n++)
		{
			//第一种
			double pos_x =template_center_x + n + template_ux * n + template_uy * m;
			double pos_y =template_center_y + m + template_vx * n + template_vy * m;


			//以下与相对位移方法相同
			int index_x = cvFloor(pos_x);
			int index_y = cvFloor(pos_y);

			if (index_y - interpolation_area_y_min <0 || index_y - interpolation_area_y_min >= interpolation_LUT_valid_sign_mat->rows-1 ||
				index_x - interpolation_area_x_min <0 || index_x - interpolation_area_x_min >= interpolation_LUT_valid_sign_mat->cols-1)
			{
				return false;
			}

			double delta_x = pos_x - index_x;
			double delta_y = pos_y - index_y;


			uchar sign = CV_MAT_ELEM(*interpolation_LUT_valid_sign_mat,uchar,index_y - interpolation_area_y_min,index_x - interpolation_area_x_min);
			float* S_mat_ptr = interpolation_LUT->data.fl+ ((index_y - interpolation_area_y_min) * (interpolation_area_x_max-interpolation_area_x_min)+index_x - interpolation_area_x_min) * Interpolation_LUT_Element_Size;
			if (sign!=1)
			{							
				double* y = image->data.db + (index_y-2)*image_width+index_x-2;
				S_mat_ptr[0] = (Q_mat_ptr[0] * y[0] + Q_mat_ptr[1] * y[image_width] + Q_mat_ptr[2] * y[2 * image_width] + Q_mat_ptr[3] * y[3 * image_width] + Q_mat_ptr[4] * y[4 * image_width] + Q_mat_ptr[5] * y[5 * image_width]) * Q_mat_ptr[0] + (Q_mat_ptr[0] * y[1] + Q_mat_ptr[1] * y[1 + image_width] + Q_mat_ptr[2] * y[1 + 2 * image_width] + Q_mat_ptr[3] * y[1 + 3 * image_width] + Q_mat_ptr[4] * y[1 + 4 * image_width] + Q_mat_ptr[5] * y[1 + 5 * image_width]) * Q_mat_ptr[1] + (Q_mat_ptr[0] * y[2] + Q_mat_ptr[1] * y[2 + image_width] + Q_mat_ptr[2] * y[2 + 2 * image_width] + Q_mat_ptr[3] * y[2 + 3 * image_width] + Q_mat_ptr[4] * y[2 + 4 * image_width] + Q_mat_ptr[5] * y[2 + 5 * image_width]) * Q_mat_ptr[2] + (Q_mat_ptr[0] * y[3] + Q_mat_ptr[1] * y[3 + image_width] + Q_mat_ptr[2] * y[3 + 2 * image_width] + Q_mat_ptr[3] * y[3 + 3 * image_width] + Q_mat_ptr[4] * y[3 + 4 * image_width] + Q_mat_ptr[5] * y[3 + 5 * image_width]) * Q_mat_ptr[3] + (Q_mat_ptr[0] * y[4] + Q_mat_ptr[1] * y[4 + image_width] + Q_mat_ptr[2] * y[4 + 2 * image_width] + Q_mat_ptr[3] * y[4 + 3 * image_width] + Q_mat_ptr[4] * y[4 + 4 * image_width] + Q_mat_ptr[5] * y[4 + 5 * image_width]) * Q_mat_ptr[4] + (Q_mat_ptr[0] * y[5] + Q_mat_ptr[1] * y[5 + image_width] + Q_mat_ptr[2] * y[5 + 2 * image_width] + Q_mat_ptr[3] * y[5 + 3 * image_width] + Q_mat_ptr[4] * y[5 + 4 * image_width] + Q_mat_ptr[5] * y[5 + 5 * image_width]) * Q_mat_ptr[5];
				S_mat_ptr[1] = (Q_mat_ptr[0] * y[0] + Q_mat_ptr[1] * y[image_width] + Q_mat_ptr[2] * y[2 * image_width] + Q_mat_ptr[3] * y[3 * image_width] + Q_mat_ptr[4] * y[4 * image_width] + Q_mat_ptr[5] * y[5 * image_width]) * Q_mat_ptr[6] + (Q_mat_ptr[0] * y[1] + Q_mat_ptr[1] * y[1 + image_width] + Q_mat_ptr[2] * y[1 + 2 * image_width] + Q_mat_ptr[3] * y[1 + 3 * image_width] + Q_mat_ptr[4] * y[1 + 4 * image_width] + Q_mat_ptr[5] * y[1 + 5 * image_width]) * Q_mat_ptr[7] + (Q_mat_ptr[0] * y[2] + Q_mat_ptr[1] * y[2 + image_width] + Q_mat_ptr[2] * y[2 + 2 * image_width] + Q_mat_ptr[3] * y[2 + 3 * image_width] + Q_mat_ptr[4] * y[2 + 4 * image_width] + Q_mat_ptr[5] * y[2 + 5 * image_width]) * Q_mat_ptr[8] + (Q_mat_ptr[0] * y[3] + Q_mat_ptr[1] * y[3 + image_width] + Q_mat_ptr[2] * y[3 + 2 * image_width] + Q_mat_ptr[3] * y[3 + 3 * image_width] + Q_mat_ptr[4] * y[3 + 4 * image_width] + Q_mat_ptr[5] * y[3 + 5 * image_width]) * Q_mat_ptr[9] + (Q_mat_ptr[0] * y[4] + Q_mat_ptr[1] * y[4 + image_width] + Q_mat_ptr[2] * y[4 + 2 * image_width] + Q_mat_ptr[3] * y[4 + 3 * image_width] + Q_mat_ptr[4] * y[4 + 4 * image_width] + Q_mat_ptr[5] * y[4 + 5 * image_width]) * Q_mat_ptr[10] + (Q_mat_ptr[0] * y[5] + Q_mat_ptr[1] * y[5 + image_width] + Q_mat_ptr[2] * y[5 + 2 * image_width] + Q_mat_ptr[3] * y[5 + 3 * image_width] + Q_mat_ptr[4] * y[5 + 4 * image_width] + Q_mat_ptr[5] * y[5 + 5 * image_width]) * Q_mat_ptr[11];
				S_mat_ptr[2] = (Q_mat_ptr[0] * y[0] + Q_mat_ptr[1] * y[image_width] + Q_mat_ptr[2] * y[2 * image_width] + Q_mat_ptr[3] * y[3 * image_width] + Q_mat_ptr[4] * y[4 * image_width] + Q_mat_ptr[5] * y[5 * image_width]) * Q_mat_ptr[12] + (Q_mat_ptr[0] * y[1] + Q_mat_ptr[1] * y[1 + image_width] + Q_mat_ptr[2] * y[1 + 2 * image_width] + Q_mat_ptr[3] * y[1 + 3 * image_width] + Q_mat_ptr[4] * y[1 + 4 * image_width] + Q_mat_ptr[5] * y[1 + 5 * image_width]) * Q_mat_ptr[13] + (Q_mat_ptr[0] * y[2] + Q_mat_ptr[1] * y[2 + image_width] + Q_mat_ptr[2] * y[2 + 2 * image_width] + Q_mat_ptr[3] * y[2 + 3 * image_width] + Q_mat_ptr[4] * y[2 + 4 * image_width] + Q_mat_ptr[5] * y[2 + 5 * image_width]) * Q_mat_ptr[14] + (Q_mat_ptr[0] * y[3] + Q_mat_ptr[1] * y[3 + image_width] + Q_mat_ptr[2] * y[3 + 2 * image_width] + Q_mat_ptr[3] * y[3 + 3 * image_width] + Q_mat_ptr[4] * y[3 + 4 * image_width] + Q_mat_ptr[5] * y[3 + 5 * image_width]) * Q_mat_ptr[15] + (Q_mat_ptr[0] * y[4] + Q_mat_ptr[1] * y[4 + image_width] + Q_mat_ptr[2] * y[4 + 2 * image_width] + Q_mat_ptr[3] * y[4 + 3 * image_width] + Q_mat_ptr[4] * y[4 + 4 * image_width] + Q_mat_ptr[5] * y[4 + 5 * image_width]) * Q_mat_ptr[16] + (Q_mat_ptr[0] * y[5] + Q_mat_ptr[1] * y[5 + image_width] + Q_mat_ptr[2] * y[5 + 2 * image_width] + Q_mat_ptr[3] * y[5 + 3 * image_width] + Q_mat_ptr[4] * y[5 + 4 * image_width] + Q_mat_ptr[5] * y[5 + 5 * image_width]) * Q_mat_ptr[17];
				S_mat_ptr[3] = Q_mat_ptr[0] * y[2] + Q_mat_ptr[1] * y[2 + image_width] + Q_mat_ptr[2] * y[2 + 2 * image_width] + Q_mat_ptr[3] * y[2 + 3 * image_width] + Q_mat_ptr[4] * y[2 + 4 * image_width] + Q_mat_ptr[5] * y[2 + 5 * image_width];
				S_mat_ptr[4] = (Q_mat_ptr[6] * y[0] + Q_mat_ptr[7] * y[image_width] + Q_mat_ptr[8] * y[2 * image_width] + Q_mat_ptr[9] * y[3 * image_width] + Q_mat_ptr[10] * y[4 * image_width] + Q_mat_ptr[11] * y[5 * image_width]) * Q_mat_ptr[0] + (Q_mat_ptr[6] * y[1] + Q_mat_ptr[7] * y[1 + image_width] + Q_mat_ptr[8] * y[1 + 2 * image_width] + Q_mat_ptr[9] * y[1 + 3 * image_width] + Q_mat_ptr[10] * y[1 + 4 * image_width] + Q_mat_ptr[11] * y[1 + 5 * image_width]) * Q_mat_ptr[1] + (Q_mat_ptr[6] * y[2] + Q_mat_ptr[7] * y[2 + image_width] + Q_mat_ptr[8] * y[2 + 2 * image_width] + Q_mat_ptr[9] * y[2 + 3 * image_width] + Q_mat_ptr[10] * y[2 + 4 * image_width] + Q_mat_ptr[11] * y[2 + 5 * image_width]) * Q_mat_ptr[2] + (Q_mat_ptr[6] * y[3] + Q_mat_ptr[7] * y[3 + image_width] + Q_mat_ptr[8] * y[3 + 2 * image_width] + Q_mat_ptr[9] * y[3 + 3 * image_width] + Q_mat_ptr[10] * y[3 + 4 * image_width] + Q_mat_ptr[11] * y[3 + 5 * image_width]) * Q_mat_ptr[3] + (Q_mat_ptr[6] * y[4] + Q_mat_ptr[7] * y[4 + image_width] + Q_mat_ptr[8] * y[4 + 2 * image_width] + Q_mat_ptr[9] * y[4 + 3 * image_width] + Q_mat_ptr[10] * y[4 + 4 * image_width] + Q_mat_ptr[11] * y[4 + 5 * image_width]) * Q_mat_ptr[4] + (Q_mat_ptr[6] * y[5] + Q_mat_ptr[7] * y[5 + image_width] + Q_mat_ptr[8] * y[5 + 2 * image_width] + Q_mat_ptr[9] * y[5 + 3 * image_width] + Q_mat_ptr[10] * y[5 + 4 * image_width] + Q_mat_ptr[11] * y[5 + 5 * image_width]) * Q_mat_ptr[5];
				S_mat_ptr[5] = (Q_mat_ptr[6] * y[0] + Q_mat_ptr[7] * y[image_width] + Q_mat_ptr[8] * y[2 * image_width] + Q_mat_ptr[9] * y[3 * image_width] + Q_mat_ptr[10] * y[4 * image_width] + Q_mat_ptr[11] * y[5 * image_width]) * Q_mat_ptr[6] + (Q_mat_ptr[6] * y[1] + Q_mat_ptr[7] * y[1 + image_width] + Q_mat_ptr[8] * y[1 + 2 * image_width] + Q_mat_ptr[9] * y[1 + 3 * image_width] + Q_mat_ptr[10] * y[1 + 4 * image_width] + Q_mat_ptr[11] * y[1 + 5 * image_width]) * Q_mat_ptr[7] + (Q_mat_ptr[6] * y[2] + Q_mat_ptr[7] * y[2 + image_width] + Q_mat_ptr[8] * y[2 + 2 * image_width] + Q_mat_ptr[9] * y[2 + 3 * image_width] + Q_mat_ptr[10] * y[2 + 4 * image_width] + Q_mat_ptr[11] * y[2 + 5 * image_width]) * Q_mat_ptr[8] + (Q_mat_ptr[6] * y[3] + Q_mat_ptr[7] * y[3 + image_width] + Q_mat_ptr[8] * y[3 + 2 * image_width] + Q_mat_ptr[9] * y[3 + 3 * image_width] + Q_mat_ptr[10] * y[3 + 4 * image_width] + Q_mat_ptr[11] * y[3 + 5 * image_width]) * Q_mat_ptr[9] + (Q_mat_ptr[6] * y[4] + Q_mat_ptr[7] * y[4 + image_width] + Q_mat_ptr[8] * y[4 + 2 * image_width] + Q_mat_ptr[9] * y[4 + 3 * image_width] + Q_mat_ptr[10] * y[4 + 4 * image_width] + Q_mat_ptr[11] * y[4 + 5 * image_width]) * Q_mat_ptr[10] + (Q_mat_ptr[6] * y[5] + Q_mat_ptr[7] * y[5 + image_width] + Q_mat_ptr[8] * y[5 + 2 * image_width] + Q_mat_ptr[9] * y[5 + 3 * image_width] + Q_mat_ptr[10] * y[5 + 4 * image_width] + Q_mat_ptr[11] * y[5 + 5 * image_width]) * Q_mat_ptr[11];
				S_mat_ptr[6] = (Q_mat_ptr[6] * y[0] + Q_mat_ptr[7] * y[image_width] + Q_mat_ptr[8] * y[2 * image_width] + Q_mat_ptr[9] * y[3 * image_width] + Q_mat_ptr[10] * y[4 * image_width] + Q_mat_ptr[11] * y[5 * image_width]) * Q_mat_ptr[12] + (Q_mat_ptr[6] * y[1] + Q_mat_ptr[7] * y[1 + image_width] + Q_mat_ptr[8] * y[1 + 2 * image_width] + Q_mat_ptr[9] * y[1 + 3 * image_width] + Q_mat_ptr[10] * y[1 + 4 * image_width] + Q_mat_ptr[11] * y[1 + 5 * image_width]) * Q_mat_ptr[13] + (Q_mat_ptr[6] * y[2] + Q_mat_ptr[7] * y[2 + image_width] + Q_mat_ptr[8] * y[2 + 2 * image_width] + Q_mat_ptr[9] * y[2 + 3 * image_width] + Q_mat_ptr[10] * y[2 + 4 * image_width] + Q_mat_ptr[11] * y[2 + 5 * image_width]) * Q_mat_ptr[14] + (Q_mat_ptr[6] * y[3] + Q_mat_ptr[7] * y[3 + image_width] + Q_mat_ptr[8] * y[3 + 2 * image_width] + Q_mat_ptr[9] * y[3 + 3 * image_width] + Q_mat_ptr[10] * y[3 + 4 * image_width] + Q_mat_ptr[11] * y[3 + 5 * image_width]) * Q_mat_ptr[15] + (Q_mat_ptr[6] * y[4] + Q_mat_ptr[7] * y[4 + image_width] + Q_mat_ptr[8] * y[4 + 2 * image_width] + Q_mat_ptr[9] * y[4 + 3 * image_width] + Q_mat_ptr[10] * y[4 + 4 * image_width] + Q_mat_ptr[11] * y[4 + 5 * image_width]) * Q_mat_ptr[16] + (Q_mat_ptr[6] * y[5] + Q_mat_ptr[7] * y[5 + image_width] + Q_mat_ptr[8] * y[5 + 2 * image_width] + Q_mat_ptr[9] * y[5 + 3 * image_width] + Q_mat_ptr[10] * y[5 + 4 * image_width] + Q_mat_ptr[11] * y[5 + 5 * image_width]) * Q_mat_ptr[17];
				S_mat_ptr[7] = Q_mat_ptr[6] * y[2] + Q_mat_ptr[7] * y[2 + image_width] + Q_mat_ptr[8] * y[2 + 2 * image_width] + Q_mat_ptr[9] * y[2 + 3 * image_width] + Q_mat_ptr[10] * y[2 + 4 * image_width] + Q_mat_ptr[11] * y[2 + 5 * image_width];
				S_mat_ptr[8] = (Q_mat_ptr[12] * y[0] + Q_mat_ptr[13] * y[image_width] + Q_mat_ptr[14] * y[2 * image_width] + Q_mat_ptr[15] * y[3 * image_width] + Q_mat_ptr[16] * y[4 * image_width] + Q_mat_ptr[17] * y[5 * image_width]) * Q_mat_ptr[0] + (Q_mat_ptr[12] * y[1] + Q_mat_ptr[13] * y[1 + image_width] + Q_mat_ptr[14] * y[1 + 2 * image_width] + Q_mat_ptr[15] * y[1 + 3 * image_width] + Q_mat_ptr[16] * y[1 + 4 * image_width] + Q_mat_ptr[17] * y[1 + 5 * image_width]) * Q_mat_ptr[1] + (Q_mat_ptr[12] * y[2] + Q_mat_ptr[13] * y[2 + image_width] + Q_mat_ptr[14] * y[2 + 2 * image_width] + Q_mat_ptr[15] * y[2 + 3 * image_width] + Q_mat_ptr[16] * y[2 + 4 * image_width] + Q_mat_ptr[17] * y[2 + 5 * image_width]) * Q_mat_ptr[2] + (Q_mat_ptr[12] * y[3] + Q_mat_ptr[13] * y[3 + image_width] + Q_mat_ptr[14] * y[3 + 2 * image_width] + Q_mat_ptr[15] * y[3 + 3 * image_width] + Q_mat_ptr[16] * y[3 + 4 * image_width] + Q_mat_ptr[17] * y[3 + 5 * image_width]) * Q_mat_ptr[3] + (Q_mat_ptr[12] * y[4] + Q_mat_ptr[13] * y[4 + image_width] + Q_mat_ptr[14] * y[4 + 2 * image_width] + Q_mat_ptr[15] * y[4 + 3 * image_width] + Q_mat_ptr[16] * y[4 + 4 * image_width] + Q_mat_ptr[17] * y[4 + 5 * image_width]) * Q_mat_ptr[4] + (Q_mat_ptr[12] * y[5] + Q_mat_ptr[13] * y[5 + image_width] + Q_mat_ptr[14] * y[5 + 2 * image_width] + Q_mat_ptr[15] * y[5 + 3 * image_width] + Q_mat_ptr[16] * y[5 + 4 * image_width] + Q_mat_ptr[17] * y[5 + 5 * image_width]) * Q_mat_ptr[5];
				S_mat_ptr[9] = (Q_mat_ptr[12] * y[0] + Q_mat_ptr[13] * y[image_width] + Q_mat_ptr[14] * y[2 * image_width] + Q_mat_ptr[15] * y[3 * image_width] + Q_mat_ptr[16] * y[4 * image_width] + Q_mat_ptr[17] * y[5 * image_width]) * Q_mat_ptr[6] + (Q_mat_ptr[12] * y[1] + Q_mat_ptr[13] * y[1 + image_width] + Q_mat_ptr[14] * y[1 + 2 * image_width] + Q_mat_ptr[15] * y[1 + 3 * image_width] + Q_mat_ptr[16] * y[1 + 4 * image_width] + Q_mat_ptr[17] * y[1 + 5 * image_width]) * Q_mat_ptr[7] + (Q_mat_ptr[12] * y[2] + Q_mat_ptr[13] * y[2 + image_width] + Q_mat_ptr[14] * y[2 + 2 * image_width] + Q_mat_ptr[15] * y[2 + 3 * image_width] + Q_mat_ptr[16] * y[2 + 4 * image_width] + Q_mat_ptr[17] * y[2 + 5 * image_width]) * Q_mat_ptr[8] + (Q_mat_ptr[12] * y[3] + Q_mat_ptr[13] * y[3 + image_width] + Q_mat_ptr[14] * y[3 + 2 * image_width] + Q_mat_ptr[15] * y[3 + 3 * image_width] + Q_mat_ptr[16] * y[3 + 4 * image_width] + Q_mat_ptr[17] * y[3 + 5 * image_width]) * Q_mat_ptr[9] + (Q_mat_ptr[12] * y[4] + Q_mat_ptr[13] * y[4 + image_width] + Q_mat_ptr[14] * y[4 + 2 * image_width] + Q_mat_ptr[15] * y[4 + 3 * image_width] + Q_mat_ptr[16] * y[4 + 4 * image_width] + Q_mat_ptr[17] * y[4 + 5 * image_width]) * Q_mat_ptr[10] + (Q_mat_ptr[12] * y[5] + Q_mat_ptr[13] * y[5 + image_width] + Q_mat_ptr[14] * y[5 + 2 * image_width] + Q_mat_ptr[15] * y[5 + 3 * image_width] + Q_mat_ptr[16] * y[5 + 4 * image_width] + Q_mat_ptr[17] * y[5 + 5 * image_width]) * Q_mat_ptr[11];
				S_mat_ptr[10] = (Q_mat_ptr[12] * y[0] + Q_mat_ptr[13] * y[image_width] + Q_mat_ptr[14] * y[2 * image_width] + Q_mat_ptr[15] * y[3 * image_width] + Q_mat_ptr[16] * y[4 * image_width] + Q_mat_ptr[17] * y[5 * image_width]) * Q_mat_ptr[12] + (Q_mat_ptr[12] * y[1] + Q_mat_ptr[13] * y[1 + image_width] + Q_mat_ptr[14] * y[1 + 2 * image_width] + Q_mat_ptr[15] * y[1 + 3 * image_width] + Q_mat_ptr[16] * y[1 + 4 * image_width] + Q_mat_ptr[17] * y[1 + 5 * image_width]) * Q_mat_ptr[13] + (Q_mat_ptr[12] * y[2] + Q_mat_ptr[13] * y[2 + image_width] + Q_mat_ptr[14] * y[2 + 2 * image_width] + Q_mat_ptr[15] * y[2 + 3 * image_width] + Q_mat_ptr[16] * y[2 + 4 * image_width] + Q_mat_ptr[17] * y[2 + 5 * image_width]) * Q_mat_ptr[14] + (Q_mat_ptr[12] * y[3] + Q_mat_ptr[13] * y[3 + image_width] + Q_mat_ptr[14] * y[3 + 2 * image_width] + Q_mat_ptr[15] * y[3 + 3 * image_width] + Q_mat_ptr[16] * y[3 + 4 * image_width] + Q_mat_ptr[17] * y[3 + 5 * image_width]) * Q_mat_ptr[15] + (Q_mat_ptr[12] * y[4] + Q_mat_ptr[13] * y[4 + image_width] + Q_mat_ptr[14] * y[4 + 2 * image_width] + Q_mat_ptr[15] * y[4 + 3 * image_width] + Q_mat_ptr[16] * y[4 + 4 * image_width] + Q_mat_ptr[17] * y[4 + 5 * image_width]) * Q_mat_ptr[16] + (Q_mat_ptr[12] * y[5] + Q_mat_ptr[13] * y[5 + image_width] + Q_mat_ptr[14] * y[5 + 2 * image_width] + Q_mat_ptr[15] * y[5 + 3 * image_width] + Q_mat_ptr[16] * y[5 + 4 * image_width] + Q_mat_ptr[17] * y[5 + 5 * image_width]) * Q_mat_ptr[17];
				S_mat_ptr[11] = Q_mat_ptr[12] * y[2] + Q_mat_ptr[13] * y[2 + image_width] + Q_mat_ptr[14] * y[2 + 2 * image_width] + Q_mat_ptr[15] * y[2 + 3 * image_width] + Q_mat_ptr[16] * y[2 + 4 * image_width] + Q_mat_ptr[17] * y[2 + 5 * image_width];
				S_mat_ptr[12] = y[2 * image_width] * Q_mat_ptr[0] + y[1 + 2 * image_width] * Q_mat_ptr[1] + Q_mat_ptr[2] * y[2 + 2 * image_width] + y[3 + 2 * image_width] * Q_mat_ptr[3] + y[4 + 2 * image_width] * Q_mat_ptr[4] + y[5 + 2 * image_width] * Q_mat_ptr[5];
				S_mat_ptr[13] = y[2 * image_width] * Q_mat_ptr[6] + y[1 + 2 * image_width] * Q_mat_ptr[7] + Q_mat_ptr[8] * y[2 + 2 * image_width] + y[3 + 2 * image_width] * Q_mat_ptr[9] + y[4 + 2 * image_width] * Q_mat_ptr[10] + y[5 + 2 * image_width] * Q_mat_ptr[11];
				S_mat_ptr[14] = y[2 * image_width] * Q_mat_ptr[12] + y[1 + 2 * image_width] * Q_mat_ptr[13] + Q_mat_ptr[14] * y[2 + 2 * image_width] + y[3 + 2 * image_width] * Q_mat_ptr[15] + y[4 + 2 * image_width] * Q_mat_ptr[16] + y[5 + 2 * image_width] * Q_mat_ptr[17];
				S_mat_ptr[15] = y[2 + 2 * image_width];
				CV_MAT_ELEM(*interpolation_LUT_valid_sign_mat,uchar,index_y - interpolation_area_y_min,index_x - interpolation_area_x_min) =1;		
			}
			//g
			CV_MAT_ELEM(*g_mat,double,m+template_half_height,n+template_half_width) =
				delta_x*delta_x*delta_x * (delta_y*delta_y*delta_y * S_mat_ptr[0] + delta_y*delta_y * S_mat_ptr[4] + delta_y * S_mat_ptr[8] + S_mat_ptr[12] ) +
				delta_x*delta_x * (delta_y*delta_y*delta_y * S_mat_ptr[1] + delta_y*delta_y * S_mat_ptr[5] + delta_y * S_mat_ptr[9] + S_mat_ptr[13] ) +
				delta_x  * (delta_y*delta_y*delta_y * S_mat_ptr[2] + delta_y*delta_y * S_mat_ptr[6] + delta_y * S_mat_ptr[10] + S_mat_ptr[14] ) +
				(delta_y*delta_y*delta_y * S_mat_ptr[3] + delta_y*delta_y * S_mat_ptr[7] + delta_y * S_mat_ptr[11] + S_mat_ptr[15] );

			//gx
			CV_MAT_ELEM(*gx_mat,double,m+template_half_height,n+template_half_width) =
				3.0 * delta_x*delta_x * (delta_y*delta_y*delta_y * S_mat_ptr[0] + delta_y*delta_y * S_mat_ptr[4] + delta_y * S_mat_ptr[8] + S_mat_ptr[12] ) +
				2.0 * delta_x  * (delta_y*delta_y*delta_y * S_mat_ptr[1] + delta_y*delta_y * S_mat_ptr[5] + delta_y * S_mat_ptr[9] + S_mat_ptr[13] ) +
				(delta_y*delta_y*delta_y * S_mat_ptr[2] + delta_y*delta_y * S_mat_ptr[6] + delta_y * S_mat_ptr[10] + S_mat_ptr[14] );
			//gy
			CV_MAT_ELEM(*gy_mat,double,m+template_half_height,n+template_half_width) =
				delta_x*delta_x*delta_x * (3.0 * delta_y*delta_y * S_mat_ptr[0] + 2.0 * delta_y * S_mat_ptr[4] + S_mat_ptr[8] ) +
				delta_x*delta_x * (3.0 * delta_y*delta_y * S_mat_ptr[1] +  2.0 * delta_y * S_mat_ptr[5] + S_mat_ptr[9] ) +
				delta_x  * (3.0 * delta_y*delta_y * S_mat_ptr[2] +  2.0 * delta_y * S_mat_ptr[6] + S_mat_ptr[10] ) +
				(3.0 * delta_y*delta_y * S_mat_ptr[3] +  2.0 * delta_y * S_mat_ptr[7] + S_mat_ptr[11] );
		}
	}


	return true;
}
bool _6x6UniformBicubicSplineInterpolationMethod::Value_One_Affine_Template_64F(
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
	)
{
	const int image_width= image->width;
	const int image_height = image->height;
	const int template_width = g_mat->cols;
	const int template_height = g_mat->cols;
	const int lut_width = interpolation_LUT->cols;
	const int lut_height = interpolation_LUT->rows;

	// Shape consistency check
	if (interpolation_area_x_min<0 || interpolation_area_x_max>=image_width||
		interpolation_area_y_min<0 || interpolation_area_y_max>=image_height)
	{
		return false;
	}
	const int interpolation_area_width = interpolation_area_x_max-interpolation_area_x_min;
	const int interpolation_area_height = interpolation_area_y_max-interpolation_area_y_min;

	if (interpolation_LUT->cols != interpolation_area_width*Interpolation_LUT_Element_Size||
		interpolation_LUT->rows !=interpolation_area_height||
		interpolation_LUT_valid_sign_mat->cols!=interpolation_area_width||
		interpolation_LUT_valid_sign_mat->rows!= interpolation_area_height)
	{
		return false;
	}

	if (uniform_bicubic_natrue_spline_interpolation_6x6_Q_mat->rows!=4 || uniform_bicubic_natrue_spline_interpolation_6x6_Q_mat->cols!=6)
	{
		return false;
	}

	// Value type check
	if (CV_MAT_TYPE(image->type) !=CV_64F) {
		return false;
	}
	if (CV_MAT_TYPE(interpolation_LUT->type) != CV_32F) {
		return false;
	}
	if (CV_MAT_TYPE(g_mat->type) != CV_64F || CV_MAT_TYPE(uniform_bicubic_natrue_spline_interpolation_6x6_Q_mat->type) != CV_64F || CV_MAT_TYPE(interpolation_LUT_valid_sign_mat->type)!=CV_8U)
	{
		return false;
	}

	const int template_half_width = template_width >> 1;
	const int template_half_height = template_height >> 1;

	double* Q_mat_ptr = uniform_bicubic_natrue_spline_interpolation_6x6_Q_mat->data.db;
	for (int m = -template_half_height; m <= template_half_height; m++)
	{
		for (int n = -template_half_width; n <= template_half_width; n++)
		{
			//第一种
			double pos_x =template_center_x + n + template_ux * n + template_uy * m;
			double pos_y =template_center_y + m + template_vx * n + template_vy * m;


			//以下与相对位移方法相同
			int index_x = cvFloor(pos_x), index_y = cvFloor(pos_y);

			if (index_y - interpolation_area_y_min <0 || index_y - interpolation_area_y_min >= interpolation_LUT_valid_sign_mat->rows-1 ||
				index_x - interpolation_area_x_min <0 || index_x - interpolation_area_x_min >= interpolation_LUT_valid_sign_mat->cols-1)
			{
				return false;
			}

			double delta_x = pos_x - index_x, delta_y = pos_y - index_y;

			uchar sign = CV_MAT_ELEM(*interpolation_LUT_valid_sign_mat,uchar,index_y - interpolation_area_y_min,index_x - interpolation_area_x_min);
			float* S_mat_ptr = interpolation_LUT->data.fl+ ((index_y - interpolation_area_y_min) * (interpolation_area_x_max-interpolation_area_x_min)+index_x - interpolation_area_x_min) * Interpolation_LUT_Element_Size;
			if (sign != 1)
			{							
				double* y = image->data.db + (index_y-2)*image_width+index_x-2;
				S_mat_ptr[0] = (Q_mat_ptr[0] * y[0] + Q_mat_ptr[1] * y[image_width] + Q_mat_ptr[2] * y[2 * image_width] + Q_mat_ptr[3] * y[3 * image_width] + Q_mat_ptr[4] * y[4 * image_width] + Q_mat_ptr[5] * y[5 * image_width]) * Q_mat_ptr[0] + (Q_mat_ptr[0] * y[1] + Q_mat_ptr[1] * y[1 + image_width] + Q_mat_ptr[2] * y[1 + 2 * image_width] + Q_mat_ptr[3] * y[1 + 3 * image_width] + Q_mat_ptr[4] * y[1 + 4 * image_width] + Q_mat_ptr[5] * y[1 + 5 * image_width]) * Q_mat_ptr[1] + (Q_mat_ptr[0] * y[2] + Q_mat_ptr[1] * y[2 + image_width] + Q_mat_ptr[2] * y[2 + 2 * image_width] + Q_mat_ptr[3] * y[2 + 3 * image_width] + Q_mat_ptr[4] * y[2 + 4 * image_width] + Q_mat_ptr[5] * y[2 + 5 * image_width]) * Q_mat_ptr[2] + (Q_mat_ptr[0] * y[3] + Q_mat_ptr[1] * y[3 + image_width] + Q_mat_ptr[2] * y[3 + 2 * image_width] + Q_mat_ptr[3] * y[3 + 3 * image_width] + Q_mat_ptr[4] * y[3 + 4 * image_width] + Q_mat_ptr[5] * y[3 + 5 * image_width]) * Q_mat_ptr[3] + (Q_mat_ptr[0] * y[4] + Q_mat_ptr[1] * y[4 + image_width] + Q_mat_ptr[2] * y[4 + 2 * image_width] + Q_mat_ptr[3] * y[4 + 3 * image_width] + Q_mat_ptr[4] * y[4 + 4 * image_width] + Q_mat_ptr[5] * y[4 + 5 * image_width]) * Q_mat_ptr[4] + (Q_mat_ptr[0] * y[5] + Q_mat_ptr[1] * y[5 + image_width] + Q_mat_ptr[2] * y[5 + 2 * image_width] + Q_mat_ptr[3] * y[5 + 3 * image_width] + Q_mat_ptr[4] * y[5 + 4 * image_width] + Q_mat_ptr[5] * y[5 + 5 * image_width]) * Q_mat_ptr[5];
				S_mat_ptr[1] = (Q_mat_ptr[0] * y[0] + Q_mat_ptr[1] * y[image_width] + Q_mat_ptr[2] * y[2 * image_width] + Q_mat_ptr[3] * y[3 * image_width] + Q_mat_ptr[4] * y[4 * image_width] + Q_mat_ptr[5] * y[5 * image_width]) * Q_mat_ptr[6] + (Q_mat_ptr[0] * y[1] + Q_mat_ptr[1] * y[1 + image_width] + Q_mat_ptr[2] * y[1 + 2 * image_width] + Q_mat_ptr[3] * y[1 + 3 * image_width] + Q_mat_ptr[4] * y[1 + 4 * image_width] + Q_mat_ptr[5] * y[1 + 5 * image_width]) * Q_mat_ptr[7] + (Q_mat_ptr[0] * y[2] + Q_mat_ptr[1] * y[2 + image_width] + Q_mat_ptr[2] * y[2 + 2 * image_width] + Q_mat_ptr[3] * y[2 + 3 * image_width] + Q_mat_ptr[4] * y[2 + 4 * image_width] + Q_mat_ptr[5] * y[2 + 5 * image_width]) * Q_mat_ptr[8] + (Q_mat_ptr[0] * y[3] + Q_mat_ptr[1] * y[3 + image_width] + Q_mat_ptr[2] * y[3 + 2 * image_width] + Q_mat_ptr[3] * y[3 + 3 * image_width] + Q_mat_ptr[4] * y[3 + 4 * image_width] + Q_mat_ptr[5] * y[3 + 5 * image_width]) * Q_mat_ptr[9] + (Q_mat_ptr[0] * y[4] + Q_mat_ptr[1] * y[4 + image_width] + Q_mat_ptr[2] * y[4 + 2 * image_width] + Q_mat_ptr[3] * y[4 + 3 * image_width] + Q_mat_ptr[4] * y[4 + 4 * image_width] + Q_mat_ptr[5] * y[4 + 5 * image_width]) * Q_mat_ptr[10] + (Q_mat_ptr[0] * y[5] + Q_mat_ptr[1] * y[5 + image_width] + Q_mat_ptr[2] * y[5 + 2 * image_width] + Q_mat_ptr[3] * y[5 + 3 * image_width] + Q_mat_ptr[4] * y[5 + 4 * image_width] + Q_mat_ptr[5] * y[5 + 5 * image_width]) * Q_mat_ptr[11];
				S_mat_ptr[2] = (Q_mat_ptr[0] * y[0] + Q_mat_ptr[1] * y[image_width] + Q_mat_ptr[2] * y[2 * image_width] + Q_mat_ptr[3] * y[3 * image_width] + Q_mat_ptr[4] * y[4 * image_width] + Q_mat_ptr[5] * y[5 * image_width]) * Q_mat_ptr[12] + (Q_mat_ptr[0] * y[1] + Q_mat_ptr[1] * y[1 + image_width] + Q_mat_ptr[2] * y[1 + 2 * image_width] + Q_mat_ptr[3] * y[1 + 3 * image_width] + Q_mat_ptr[4] * y[1 + 4 * image_width] + Q_mat_ptr[5] * y[1 + 5 * image_width]) * Q_mat_ptr[13] + (Q_mat_ptr[0] * y[2] + Q_mat_ptr[1] * y[2 + image_width] + Q_mat_ptr[2] * y[2 + 2 * image_width] + Q_mat_ptr[3] * y[2 + 3 * image_width] + Q_mat_ptr[4] * y[2 + 4 * image_width] + Q_mat_ptr[5] * y[2 + 5 * image_width]) * Q_mat_ptr[14] + (Q_mat_ptr[0] * y[3] + Q_mat_ptr[1] * y[3 + image_width] + Q_mat_ptr[2] * y[3 + 2 * image_width] + Q_mat_ptr[3] * y[3 + 3 * image_width] + Q_mat_ptr[4] * y[3 + 4 * image_width] + Q_mat_ptr[5] * y[3 + 5 * image_width]) * Q_mat_ptr[15] + (Q_mat_ptr[0] * y[4] + Q_mat_ptr[1] * y[4 + image_width] + Q_mat_ptr[2] * y[4 + 2 * image_width] + Q_mat_ptr[3] * y[4 + 3 * image_width] + Q_mat_ptr[4] * y[4 + 4 * image_width] + Q_mat_ptr[5] * y[4 + 5 * image_width]) * Q_mat_ptr[16] + (Q_mat_ptr[0] * y[5] + Q_mat_ptr[1] * y[5 + image_width] + Q_mat_ptr[2] * y[5 + 2 * image_width] + Q_mat_ptr[3] * y[5 + 3 * image_width] + Q_mat_ptr[4] * y[5 + 4 * image_width] + Q_mat_ptr[5] * y[5 + 5 * image_width]) * Q_mat_ptr[17];
				S_mat_ptr[3] = Q_mat_ptr[0] * y[2] + Q_mat_ptr[1] * y[2 + image_width] + Q_mat_ptr[2] * y[2 + 2 * image_width] + Q_mat_ptr[3] * y[2 + 3 * image_width] + Q_mat_ptr[4] * y[2 + 4 * image_width] + Q_mat_ptr[5] * y[2 + 5 * image_width];
				S_mat_ptr[4] = (Q_mat_ptr[6] * y[0] + Q_mat_ptr[7] * y[image_width] + Q_mat_ptr[8] * y[2 * image_width] + Q_mat_ptr[9] * y[3 * image_width] + Q_mat_ptr[10] * y[4 * image_width] + Q_mat_ptr[11] * y[5 * image_width]) * Q_mat_ptr[0] + (Q_mat_ptr[6] * y[1] + Q_mat_ptr[7] * y[1 + image_width] + Q_mat_ptr[8] * y[1 + 2 * image_width] + Q_mat_ptr[9] * y[1 + 3 * image_width] + Q_mat_ptr[10] * y[1 + 4 * image_width] + Q_mat_ptr[11] * y[1 + 5 * image_width]) * Q_mat_ptr[1] + (Q_mat_ptr[6] * y[2] + Q_mat_ptr[7] * y[2 + image_width] + Q_mat_ptr[8] * y[2 + 2 * image_width] + Q_mat_ptr[9] * y[2 + 3 * image_width] + Q_mat_ptr[10] * y[2 + 4 * image_width] + Q_mat_ptr[11] * y[2 + 5 * image_width]) * Q_mat_ptr[2] + (Q_mat_ptr[6] * y[3] + Q_mat_ptr[7] * y[3 + image_width] + Q_mat_ptr[8] * y[3 + 2 * image_width] + Q_mat_ptr[9] * y[3 + 3 * image_width] + Q_mat_ptr[10] * y[3 + 4 * image_width] + Q_mat_ptr[11] * y[3 + 5 * image_width]) * Q_mat_ptr[3] + (Q_mat_ptr[6] * y[4] + Q_mat_ptr[7] * y[4 + image_width] + Q_mat_ptr[8] * y[4 + 2 * image_width] + Q_mat_ptr[9] * y[4 + 3 * image_width] + Q_mat_ptr[10] * y[4 + 4 * image_width] + Q_mat_ptr[11] * y[4 + 5 * image_width]) * Q_mat_ptr[4] + (Q_mat_ptr[6] * y[5] + Q_mat_ptr[7] * y[5 + image_width] + Q_mat_ptr[8] * y[5 + 2 * image_width] + Q_mat_ptr[9] * y[5 + 3 * image_width] + Q_mat_ptr[10] * y[5 + 4 * image_width] + Q_mat_ptr[11] * y[5 + 5 * image_width]) * Q_mat_ptr[5];
				S_mat_ptr[5] = (Q_mat_ptr[6] * y[0] + Q_mat_ptr[7] * y[image_width] + Q_mat_ptr[8] * y[2 * image_width] + Q_mat_ptr[9] * y[3 * image_width] + Q_mat_ptr[10] * y[4 * image_width] + Q_mat_ptr[11] * y[5 * image_width]) * Q_mat_ptr[6] + (Q_mat_ptr[6] * y[1] + Q_mat_ptr[7] * y[1 + image_width] + Q_mat_ptr[8] * y[1 + 2 * image_width] + Q_mat_ptr[9] * y[1 + 3 * image_width] + Q_mat_ptr[10] * y[1 + 4 * image_width] + Q_mat_ptr[11] * y[1 + 5 * image_width]) * Q_mat_ptr[7] + (Q_mat_ptr[6] * y[2] + Q_mat_ptr[7] * y[2 + image_width] + Q_mat_ptr[8] * y[2 + 2 * image_width] + Q_mat_ptr[9] * y[2 + 3 * image_width] + Q_mat_ptr[10] * y[2 + 4 * image_width] + Q_mat_ptr[11] * y[2 + 5 * image_width]) * Q_mat_ptr[8] + (Q_mat_ptr[6] * y[3] + Q_mat_ptr[7] * y[3 + image_width] + Q_mat_ptr[8] * y[3 + 2 * image_width] + Q_mat_ptr[9] * y[3 + 3 * image_width] + Q_mat_ptr[10] * y[3 + 4 * image_width] + Q_mat_ptr[11] * y[3 + 5 * image_width]) * Q_mat_ptr[9] + (Q_mat_ptr[6] * y[4] + Q_mat_ptr[7] * y[4 + image_width] + Q_mat_ptr[8] * y[4 + 2 * image_width] + Q_mat_ptr[9] * y[4 + 3 * image_width] + Q_mat_ptr[10] * y[4 + 4 * image_width] + Q_mat_ptr[11] * y[4 + 5 * image_width]) * Q_mat_ptr[10] + (Q_mat_ptr[6] * y[5] + Q_mat_ptr[7] * y[5 + image_width] + Q_mat_ptr[8] * y[5 + 2 * image_width] + Q_mat_ptr[9] * y[5 + 3 * image_width] + Q_mat_ptr[10] * y[5 + 4 * image_width] + Q_mat_ptr[11] * y[5 + 5 * image_width]) * Q_mat_ptr[11];
				S_mat_ptr[6] = (Q_mat_ptr[6] * y[0] + Q_mat_ptr[7] * y[image_width] + Q_mat_ptr[8] * y[2 * image_width] + Q_mat_ptr[9] * y[3 * image_width] + Q_mat_ptr[10] * y[4 * image_width] + Q_mat_ptr[11] * y[5 * image_width]) * Q_mat_ptr[12] + (Q_mat_ptr[6] * y[1] + Q_mat_ptr[7] * y[1 + image_width] + Q_mat_ptr[8] * y[1 + 2 * image_width] + Q_mat_ptr[9] * y[1 + 3 * image_width] + Q_mat_ptr[10] * y[1 + 4 * image_width] + Q_mat_ptr[11] * y[1 + 5 * image_width]) * Q_mat_ptr[13] + (Q_mat_ptr[6] * y[2] + Q_mat_ptr[7] * y[2 + image_width] + Q_mat_ptr[8] * y[2 + 2 * image_width] + Q_mat_ptr[9] * y[2 + 3 * image_width] + Q_mat_ptr[10] * y[2 + 4 * image_width] + Q_mat_ptr[11] * y[2 + 5 * image_width]) * Q_mat_ptr[14] + (Q_mat_ptr[6] * y[3] + Q_mat_ptr[7] * y[3 + image_width] + Q_mat_ptr[8] * y[3 + 2 * image_width] + Q_mat_ptr[9] * y[3 + 3 * image_width] + Q_mat_ptr[10] * y[3 + 4 * image_width] + Q_mat_ptr[11] * y[3 + 5 * image_width]) * Q_mat_ptr[15] + (Q_mat_ptr[6] * y[4] + Q_mat_ptr[7] * y[4 + image_width] + Q_mat_ptr[8] * y[4 + 2 * image_width] + Q_mat_ptr[9] * y[4 + 3 * image_width] + Q_mat_ptr[10] * y[4 + 4 * image_width] + Q_mat_ptr[11] * y[4 + 5 * image_width]) * Q_mat_ptr[16] + (Q_mat_ptr[6] * y[5] + Q_mat_ptr[7] * y[5 + image_width] + Q_mat_ptr[8] * y[5 + 2 * image_width] + Q_mat_ptr[9] * y[5 + 3 * image_width] + Q_mat_ptr[10] * y[5 + 4 * image_width] + Q_mat_ptr[11] * y[5 + 5 * image_width]) * Q_mat_ptr[17];
				S_mat_ptr[7] = Q_mat_ptr[6] * y[2] + Q_mat_ptr[7] * y[2 + image_width] + Q_mat_ptr[8] * y[2 + 2 * image_width] + Q_mat_ptr[9] * y[2 + 3 * image_width] + Q_mat_ptr[10] * y[2 + 4 * image_width] + Q_mat_ptr[11] * y[2 + 5 * image_width];
				S_mat_ptr[8] = (Q_mat_ptr[12] * y[0] + Q_mat_ptr[13] * y[image_width] + Q_mat_ptr[14] * y[2 * image_width] + Q_mat_ptr[15] * y[3 * image_width] + Q_mat_ptr[16] * y[4 * image_width] + Q_mat_ptr[17] * y[5 * image_width]) * Q_mat_ptr[0] + (Q_mat_ptr[12] * y[1] + Q_mat_ptr[13] * y[1 + image_width] + Q_mat_ptr[14] * y[1 + 2 * image_width] + Q_mat_ptr[15] * y[1 + 3 * image_width] + Q_mat_ptr[16] * y[1 + 4 * image_width] + Q_mat_ptr[17] * y[1 + 5 * image_width]) * Q_mat_ptr[1] + (Q_mat_ptr[12] * y[2] + Q_mat_ptr[13] * y[2 + image_width] + Q_mat_ptr[14] * y[2 + 2 * image_width] + Q_mat_ptr[15] * y[2 + 3 * image_width] + Q_mat_ptr[16] * y[2 + 4 * image_width] + Q_mat_ptr[17] * y[2 + 5 * image_width]) * Q_mat_ptr[2] + (Q_mat_ptr[12] * y[3] + Q_mat_ptr[13] * y[3 + image_width] + Q_mat_ptr[14] * y[3 + 2 * image_width] + Q_mat_ptr[15] * y[3 + 3 * image_width] + Q_mat_ptr[16] * y[3 + 4 * image_width] + Q_mat_ptr[17] * y[3 + 5 * image_width]) * Q_mat_ptr[3] + (Q_mat_ptr[12] * y[4] + Q_mat_ptr[13] * y[4 + image_width] + Q_mat_ptr[14] * y[4 + 2 * image_width] + Q_mat_ptr[15] * y[4 + 3 * image_width] + Q_mat_ptr[16] * y[4 + 4 * image_width] + Q_mat_ptr[17] * y[4 + 5 * image_width]) * Q_mat_ptr[4] + (Q_mat_ptr[12] * y[5] + Q_mat_ptr[13] * y[5 + image_width] + Q_mat_ptr[14] * y[5 + 2 * image_width] + Q_mat_ptr[15] * y[5 + 3 * image_width] + Q_mat_ptr[16] * y[5 + 4 * image_width] + Q_mat_ptr[17] * y[5 + 5 * image_width]) * Q_mat_ptr[5];
				S_mat_ptr[9] = (Q_mat_ptr[12] * y[0] + Q_mat_ptr[13] * y[image_width] + Q_mat_ptr[14] * y[2 * image_width] + Q_mat_ptr[15] * y[3 * image_width] + Q_mat_ptr[16] * y[4 * image_width] + Q_mat_ptr[17] * y[5 * image_width]) * Q_mat_ptr[6] + (Q_mat_ptr[12] * y[1] + Q_mat_ptr[13] * y[1 + image_width] + Q_mat_ptr[14] * y[1 + 2 * image_width] + Q_mat_ptr[15] * y[1 + 3 * image_width] + Q_mat_ptr[16] * y[1 + 4 * image_width] + Q_mat_ptr[17] * y[1 + 5 * image_width]) * Q_mat_ptr[7] + (Q_mat_ptr[12] * y[2] + Q_mat_ptr[13] * y[2 + image_width] + Q_mat_ptr[14] * y[2 + 2 * image_width] + Q_mat_ptr[15] * y[2 + 3 * image_width] + Q_mat_ptr[16] * y[2 + 4 * image_width] + Q_mat_ptr[17] * y[2 + 5 * image_width]) * Q_mat_ptr[8] + (Q_mat_ptr[12] * y[3] + Q_mat_ptr[13] * y[3 + image_width] + Q_mat_ptr[14] * y[3 + 2 * image_width] + Q_mat_ptr[15] * y[3 + 3 * image_width] + Q_mat_ptr[16] * y[3 + 4 * image_width] + Q_mat_ptr[17] * y[3 + 5 * image_width]) * Q_mat_ptr[9] + (Q_mat_ptr[12] * y[4] + Q_mat_ptr[13] * y[4 + image_width] + Q_mat_ptr[14] * y[4 + 2 * image_width] + Q_mat_ptr[15] * y[4 + 3 * image_width] + Q_mat_ptr[16] * y[4 + 4 * image_width] + Q_mat_ptr[17] * y[4 + 5 * image_width]) * Q_mat_ptr[10] + (Q_mat_ptr[12] * y[5] + Q_mat_ptr[13] * y[5 + image_width] + Q_mat_ptr[14] * y[5 + 2 * image_width] + Q_mat_ptr[15] * y[5 + 3 * image_width] + Q_mat_ptr[16] * y[5 + 4 * image_width] + Q_mat_ptr[17] * y[5 + 5 * image_width]) * Q_mat_ptr[11];
				S_mat_ptr[10] = (Q_mat_ptr[12] * y[0] + Q_mat_ptr[13] * y[image_width] + Q_mat_ptr[14] * y[2 * image_width] + Q_mat_ptr[15] * y[3 * image_width] + Q_mat_ptr[16] * y[4 * image_width] + Q_mat_ptr[17] * y[5 * image_width]) * Q_mat_ptr[12] + (Q_mat_ptr[12] * y[1] + Q_mat_ptr[13] * y[1 + image_width] + Q_mat_ptr[14] * y[1 + 2 * image_width] + Q_mat_ptr[15] * y[1 + 3 * image_width] + Q_mat_ptr[16] * y[1 + 4 * image_width] + Q_mat_ptr[17] * y[1 + 5 * image_width]) * Q_mat_ptr[13] + (Q_mat_ptr[12] * y[2] + Q_mat_ptr[13] * y[2 + image_width] + Q_mat_ptr[14] * y[2 + 2 * image_width] + Q_mat_ptr[15] * y[2 + 3 * image_width] + Q_mat_ptr[16] * y[2 + 4 * image_width] + Q_mat_ptr[17] * y[2 + 5 * image_width]) * Q_mat_ptr[14] + (Q_mat_ptr[12] * y[3] + Q_mat_ptr[13] * y[3 + image_width] + Q_mat_ptr[14] * y[3 + 2 * image_width] + Q_mat_ptr[15] * y[3 + 3 * image_width] + Q_mat_ptr[16] * y[3 + 4 * image_width] + Q_mat_ptr[17] * y[3 + 5 * image_width]) * Q_mat_ptr[15] + (Q_mat_ptr[12] * y[4] + Q_mat_ptr[13] * y[4 + image_width] + Q_mat_ptr[14] * y[4 + 2 * image_width] + Q_mat_ptr[15] * y[4 + 3 * image_width] + Q_mat_ptr[16] * y[4 + 4 * image_width] + Q_mat_ptr[17] * y[4 + 5 * image_width]) * Q_mat_ptr[16] + (Q_mat_ptr[12] * y[5] + Q_mat_ptr[13] * y[5 + image_width] + Q_mat_ptr[14] * y[5 + 2 * image_width] + Q_mat_ptr[15] * y[5 + 3 * image_width] + Q_mat_ptr[16] * y[5 + 4 * image_width] + Q_mat_ptr[17] * y[5 + 5 * image_width]) * Q_mat_ptr[17];
				S_mat_ptr[11] = Q_mat_ptr[12] * y[2] + Q_mat_ptr[13] * y[2 + image_width] + Q_mat_ptr[14] * y[2 + 2 * image_width] + Q_mat_ptr[15] * y[2 + 3 * image_width] + Q_mat_ptr[16] * y[2 + 4 * image_width] + Q_mat_ptr[17] * y[2 + 5 * image_width];
				S_mat_ptr[12] = y[2 * image_width] * Q_mat_ptr[0] + y[1 + 2 * image_width] * Q_mat_ptr[1] + Q_mat_ptr[2] * y[2 + 2 * image_width] + y[3 + 2 * image_width] * Q_mat_ptr[3] + y[4 + 2 * image_width] * Q_mat_ptr[4] + y[5 + 2 * image_width] * Q_mat_ptr[5];
				S_mat_ptr[13] = y[2 * image_width] * Q_mat_ptr[6] + y[1 + 2 * image_width] * Q_mat_ptr[7] + Q_mat_ptr[8] * y[2 + 2 * image_width] + y[3 + 2 * image_width] * Q_mat_ptr[9] + y[4 + 2 * image_width] * Q_mat_ptr[10] + y[5 + 2 * image_width] * Q_mat_ptr[11];
				S_mat_ptr[14] = y[2 * image_width] * Q_mat_ptr[12] + y[1 + 2 * image_width] * Q_mat_ptr[13] + Q_mat_ptr[14] * y[2 + 2 * image_width] + y[3 + 2 * image_width] * Q_mat_ptr[15] + y[4 + 2 * image_width] * Q_mat_ptr[16] + y[5 + 2 * image_width] * Q_mat_ptr[17];
				S_mat_ptr[15] = y[2 + 2 * image_width];
				CV_MAT_ELEM(*interpolation_LUT_valid_sign_mat,uchar,index_y - interpolation_area_y_min,index_x - interpolation_area_x_min) =1;		
			}
			//g
			CV_MAT_ELEM(*g_mat,double,m+template_half_height,n+template_half_width) =
				delta_x*delta_x*delta_x * (delta_y*delta_y*delta_y * S_mat_ptr[0] + delta_y*delta_y * S_mat_ptr[4] + delta_y * S_mat_ptr[8] + S_mat_ptr[12] ) +
				delta_x*delta_x * (delta_y*delta_y*delta_y * S_mat_ptr[1] + delta_y*delta_y * S_mat_ptr[5] + delta_y * S_mat_ptr[9] + S_mat_ptr[13] ) +
				delta_x  * (delta_y*delta_y*delta_y * S_mat_ptr[2] + delta_y*delta_y * S_mat_ptr[6] + delta_y * S_mat_ptr[10] + S_mat_ptr[14] ) +
				(delta_y*delta_y*delta_y * S_mat_ptr[3] + delta_y*delta_y * S_mat_ptr[7] + delta_y * S_mat_ptr[11] + S_mat_ptr[15] );

			//CV_MAT_ELEM(*g_mat,double,m+template_half_height,n+template_half_width) =
			//	3.0 * delta_x*delta_x * (delta_y*delta_y*delta_y * S_mat_ptr[0] + delta_y*delta_y * S_mat_ptr[4] + delta_y * S_mat_ptr[8] + S_mat_ptr[12] ) +
			//	2.0 * delta_x  * (delta_y*delta_y*delta_y * S_mat_ptr[1] + delta_y*delta_y * S_mat_ptr[5] + delta_y * S_mat_ptr[9] + S_mat_ptr[13] ) +
			//	(delta_y*delta_y*delta_y * S_mat_ptr[2] + delta_y*delta_y * S_mat_ptr[6] + delta_y * S_mat_ptr[10] + S_mat_ptr[14] );
			//CV_MAT_ELEM(*g_mat,double,m+template_half_height,n+template_half_width) =
			//	delta_x*delta_x*delta_x * (3.0 * delta_y*delta_y * S_mat_ptr[0] + 2.0 * delta_y * S_mat_ptr[4] + S_mat_ptr[8] ) +
			//	delta_x*delta_x * (3.0 * delta_y*delta_y * S_mat_ptr[1] +  2.0 * delta_y * S_mat_ptr[5] + S_mat_ptr[9] ) +
			//	delta_x  * (3.0 * delta_y*delta_y * S_mat_ptr[2] +  2.0 * delta_y * S_mat_ptr[6] + S_mat_ptr[10] ) +
			//	(3.0 * delta_y*delta_y * S_mat_ptr[3] +  2.0 * delta_y * S_mat_ptr[7] + S_mat_ptr[11] );
		}
	}
	

	return true;
}



bool _6x6UniformBicubicSplineInterpolationMethod::Value_One_Affine_Template_64F(
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
	)
{
	return Value_One_Affine_Template_64F(
		image,
		CV_MAT_ELEM(*template_transform,double,0,0),
		CV_MAT_ELEM(*template_transform,double,3,0),
		CV_MAT_ELEM(*template_transform,double,1,0),
		CV_MAT_ELEM(*template_transform,double,2,0),
		CV_MAT_ELEM(*template_transform,double,4,0),
		CV_MAT_ELEM(*template_transform,double,5,0),
		interpolation_area_x_min,
		interpolation_area_x_max,//上闭区间，下开区间
		interpolation_area_y_min,
		interpolation_area_y_max,//左闭区间，右边开区间
		interpolation_LUT,//32F
		interpolation_LUT_valid_sign_mat,
		uniform_bicubic_natrue_spline_interpolation_6x6_Q_mat,
		g_mat
		);
	////判断
	////大小
	//int image_width= image->width;
	//int image_height = image->height;
	//int template_width = g_mat->cols;
	//int template_height = g_mat->cols;
	//int lut_width = interpolation_LUT->cols;
	//int lut_height = interpolation_LUT->rows;


	//if (interpolation_area_x_min<0 || interpolation_area_x_max>=image_width||
	//	interpolation_area_y_min<0 || interpolation_area_y_max>=image_height)
	//{
	//	return false;
	//}
	//int interpolation_area_width = interpolation_area_x_max-interpolation_area_x_min;
	//int interpolation_area_height = interpolation_area_y_max-interpolation_area_y_min;

	//if (interpolation_LUT->cols != interpolation_area_width*Interpolation_LUT_Element_Size||
	//	interpolation_LUT->rows !=interpolation_area_height||
	//	interpolation_LUT_valid_sign_mat->cols!=interpolation_area_width||
	//	interpolation_LUT_valid_sign_mat->rows!= interpolation_area_height)
	//{
	//	return false;
	//}

	//if (uniform_bicubic_natrue_spline_interpolation_6x6_Q_mat->rows!=4 || uniform_bicubic_natrue_spline_interpolation_6x6_Q_mat->cols!=6)
	//{
	//	return false;
	//}



	////类型
	//if (CV_MAT_TYPE(image->type) !=CV_64F)
	//{
	//	return false;
	//}

	//if (CV_MAT_TYPE(interpolation_LUT->type) != CV_32F)
	//{
	//	return false;
	//}

	//if (CV_MAT_TYPE(g_mat->type) != CV_64F || CV_MAT_TYPE(uniform_bicubic_natrue_spline_interpolation_6x6_Q_mat->type) != CV_64F || CV_MAT_TYPE(interpolation_LUT_valid_sign_mat->type)!=CV_8U)
	//{
	//	return false;
	//}

	//int template_half_width = (template_width-1)/2;
	//int template_half_height = (template_height-1)/2;
	//double* Q_mat_ptr = uniform_bicubic_natrue_spline_interpolation_6x6_Q_mat->data.db;


	//double template_center_x = CV_MAT_ELEM(*template_transform,double,0,0);
	//double template_ux = CV_MAT_ELEM(*template_transform,double,1,0);
	//double template_uy = CV_MAT_ELEM(*template_transform,double,2,0);
	//double template_center_y = CV_MAT_ELEM(*template_transform,double,3,0);
	//double template_vx = CV_MAT_ELEM(*template_transform,double,4,0);
	//double template_vy = CV_MAT_ELEM(*template_transform,double,5,0);


	//for (int m =-template_half_height;m<=template_half_height;m++)
	//{
	//	for (int n =-template_half_width;n<=template_half_width;n++)
	//	{
	//		//第一种
	//		//double pos_x = template_center_x + n + template_ux * n + template_uy * m;
	//		//double pos_y =template_center_y + m + template_vx * n + template_vy * m;
	//		double pos_x = 
	//			CV_MAT_ELEM(*template_transform,double,0,0) + n +
	//			CV_MAT_ELEM(*template_transform,double,1,0) * n +
	//			CV_MAT_ELEM(*template_transform,double,2,0) * m;
	//		double pos_y = 
	//			CV_MAT_ELEM(*template_transform,double,3,0) + m + 
	//			CV_MAT_ELEM(*template_transform,double,4,0) * n + 
	//			CV_MAT_ELEM(*template_transform,double,5,0) * m;


	//		//以下与相对位移方法相同
	//		int index_x = cvFloor(pos_x);
	//		int index_y = cvFloor(pos_y);
	//		if (index_y - interpolation_area_y_min <0 || index_y - interpolation_area_y_min >= interpolation_LUT_valid_sign_mat->rows-1 ||
	//			index_x - interpolation_area_x_min <0 || index_x - interpolation_area_x_min >= interpolation_LUT_valid_sign_mat->cols-1)
	//		{
	//			return false;
	//		}

	//		double delta_x = pos_x - index_x;
	//		double delta_y = pos_y - index_y;

	//		uchar sign = CV_MAT_ELEM(*interpolation_LUT_valid_sign_mat,uchar,index_y - interpolation_area_y_min,index_x - interpolation_area_x_min);
	//		float* S_mat_ptr = interpolation_LUT->data.fl+ ((index_y - interpolation_area_y_min) * (interpolation_area_x_max-interpolation_area_x_min)+index_x - interpolation_area_x_min) * Interpolation_LUT_Element_Size;
	//		if (sign!=1)
	//		{							
	//			double* y = image->data.db + (index_y-2)*image_width+index_x-2;
	//			S_mat_ptr[0] = (Q_mat_ptr[0] * y[0] + Q_mat_ptr[1] * y[image_width] + Q_mat_ptr[2] * y[2 * image_width] + Q_mat_ptr[3] * y[3 * image_width] + Q_mat_ptr[4] * y[4 * image_width] + Q_mat_ptr[5] * y[5 * image_width]) * Q_mat_ptr[0] + (Q_mat_ptr[0] * y[1] + Q_mat_ptr[1] * y[1 + image_width] + Q_mat_ptr[2] * y[1 + 2 * image_width] + Q_mat_ptr[3] * y[1 + 3 * image_width] + Q_mat_ptr[4] * y[1 + 4 * image_width] + Q_mat_ptr[5] * y[1 + 5 * image_width]) * Q_mat_ptr[1] + (Q_mat_ptr[0] * y[2] + Q_mat_ptr[1] * y[2 + image_width] + Q_mat_ptr[2] * y[2 + 2 * image_width] + Q_mat_ptr[3] * y[2 + 3 * image_width] + Q_mat_ptr[4] * y[2 + 4 * image_width] + Q_mat_ptr[5] * y[2 + 5 * image_width]) * Q_mat_ptr[2] + (Q_mat_ptr[0] * y[3] + Q_mat_ptr[1] * y[3 + image_width] + Q_mat_ptr[2] * y[3 + 2 * image_width] + Q_mat_ptr[3] * y[3 + 3 * image_width] + Q_mat_ptr[4] * y[3 + 4 * image_width] + Q_mat_ptr[5] * y[3 + 5 * image_width]) * Q_mat_ptr[3] + (Q_mat_ptr[0] * y[4] + Q_mat_ptr[1] * y[4 + image_width] + Q_mat_ptr[2] * y[4 + 2 * image_width] + Q_mat_ptr[3] * y[4 + 3 * image_width] + Q_mat_ptr[4] * y[4 + 4 * image_width] + Q_mat_ptr[5] * y[4 + 5 * image_width]) * Q_mat_ptr[4] + (Q_mat_ptr[0] * y[5] + Q_mat_ptr[1] * y[5 + image_width] + Q_mat_ptr[2] * y[5 + 2 * image_width] + Q_mat_ptr[3] * y[5 + 3 * image_width] + Q_mat_ptr[4] * y[5 + 4 * image_width] + Q_mat_ptr[5] * y[5 + 5 * image_width]) * Q_mat_ptr[5];
	//			S_mat_ptr[1] = (Q_mat_ptr[0] * y[0] + Q_mat_ptr[1] * y[image_width] + Q_mat_ptr[2] * y[2 * image_width] + Q_mat_ptr[3] * y[3 * image_width] + Q_mat_ptr[4] * y[4 * image_width] + Q_mat_ptr[5] * y[5 * image_width]) * Q_mat_ptr[6] + (Q_mat_ptr[0] * y[1] + Q_mat_ptr[1] * y[1 + image_width] + Q_mat_ptr[2] * y[1 + 2 * image_width] + Q_mat_ptr[3] * y[1 + 3 * image_width] + Q_mat_ptr[4] * y[1 + 4 * image_width] + Q_mat_ptr[5] * y[1 + 5 * image_width]) * Q_mat_ptr[7] + (Q_mat_ptr[0] * y[2] + Q_mat_ptr[1] * y[2 + image_width] + Q_mat_ptr[2] * y[2 + 2 * image_width] + Q_mat_ptr[3] * y[2 + 3 * image_width] + Q_mat_ptr[4] * y[2 + 4 * image_width] + Q_mat_ptr[5] * y[2 + 5 * image_width]) * Q_mat_ptr[8] + (Q_mat_ptr[0] * y[3] + Q_mat_ptr[1] * y[3 + image_width] + Q_mat_ptr[2] * y[3 + 2 * image_width] + Q_mat_ptr[3] * y[3 + 3 * image_width] + Q_mat_ptr[4] * y[3 + 4 * image_width] + Q_mat_ptr[5] * y[3 + 5 * image_width]) * Q_mat_ptr[9] + (Q_mat_ptr[0] * y[4] + Q_mat_ptr[1] * y[4 + image_width] + Q_mat_ptr[2] * y[4 + 2 * image_width] + Q_mat_ptr[3] * y[4 + 3 * image_width] + Q_mat_ptr[4] * y[4 + 4 * image_width] + Q_mat_ptr[5] * y[4 + 5 * image_width]) * Q_mat_ptr[10] + (Q_mat_ptr[0] * y[5] + Q_mat_ptr[1] * y[5 + image_width] + Q_mat_ptr[2] * y[5 + 2 * image_width] + Q_mat_ptr[3] * y[5 + 3 * image_width] + Q_mat_ptr[4] * y[5 + 4 * image_width] + Q_mat_ptr[5] * y[5 + 5 * image_width]) * Q_mat_ptr[11];
	//			S_mat_ptr[2] = (Q_mat_ptr[0] * y[0] + Q_mat_ptr[1] * y[image_width] + Q_mat_ptr[2] * y[2 * image_width] + Q_mat_ptr[3] * y[3 * image_width] + Q_mat_ptr[4] * y[4 * image_width] + Q_mat_ptr[5] * y[5 * image_width]) * Q_mat_ptr[12] + (Q_mat_ptr[0] * y[1] + Q_mat_ptr[1] * y[1 + image_width] + Q_mat_ptr[2] * y[1 + 2 * image_width] + Q_mat_ptr[3] * y[1 + 3 * image_width] + Q_mat_ptr[4] * y[1 + 4 * image_width] + Q_mat_ptr[5] * y[1 + 5 * image_width]) * Q_mat_ptr[13] + (Q_mat_ptr[0] * y[2] + Q_mat_ptr[1] * y[2 + image_width] + Q_mat_ptr[2] * y[2 + 2 * image_width] + Q_mat_ptr[3] * y[2 + 3 * image_width] + Q_mat_ptr[4] * y[2 + 4 * image_width] + Q_mat_ptr[5] * y[2 + 5 * image_width]) * Q_mat_ptr[14] + (Q_mat_ptr[0] * y[3] + Q_mat_ptr[1] * y[3 + image_width] + Q_mat_ptr[2] * y[3 + 2 * image_width] + Q_mat_ptr[3] * y[3 + 3 * image_width] + Q_mat_ptr[4] * y[3 + 4 * image_width] + Q_mat_ptr[5] * y[3 + 5 * image_width]) * Q_mat_ptr[15] + (Q_mat_ptr[0] * y[4] + Q_mat_ptr[1] * y[4 + image_width] + Q_mat_ptr[2] * y[4 + 2 * image_width] + Q_mat_ptr[3] * y[4 + 3 * image_width] + Q_mat_ptr[4] * y[4 + 4 * image_width] + Q_mat_ptr[5] * y[4 + 5 * image_width]) * Q_mat_ptr[16] + (Q_mat_ptr[0] * y[5] + Q_mat_ptr[1] * y[5 + image_width] + Q_mat_ptr[2] * y[5 + 2 * image_width] + Q_mat_ptr[3] * y[5 + 3 * image_width] + Q_mat_ptr[4] * y[5 + 4 * image_width] + Q_mat_ptr[5] * y[5 + 5 * image_width]) * Q_mat_ptr[17];
	//			S_mat_ptr[3] = Q_mat_ptr[0] * y[2] + Q_mat_ptr[1] * y[2 + image_width] + Q_mat_ptr[2] * y[2 + 2 * image_width] + Q_mat_ptr[3] * y[2 + 3 * image_width] + Q_mat_ptr[4] * y[2 + 4 * image_width] + Q_mat_ptr[5] * y[2 + 5 * image_width];
	//			S_mat_ptr[4] = (Q_mat_ptr[6] * y[0] + Q_mat_ptr[7] * y[image_width] + Q_mat_ptr[8] * y[2 * image_width] + Q_mat_ptr[9] * y[3 * image_width] + Q_mat_ptr[10] * y[4 * image_width] + Q_mat_ptr[11] * y[5 * image_width]) * Q_mat_ptr[0] + (Q_mat_ptr[6] * y[1] + Q_mat_ptr[7] * y[1 + image_width] + Q_mat_ptr[8] * y[1 + 2 * image_width] + Q_mat_ptr[9] * y[1 + 3 * image_width] + Q_mat_ptr[10] * y[1 + 4 * image_width] + Q_mat_ptr[11] * y[1 + 5 * image_width]) * Q_mat_ptr[1] + (Q_mat_ptr[6] * y[2] + Q_mat_ptr[7] * y[2 + image_width] + Q_mat_ptr[8] * y[2 + 2 * image_width] + Q_mat_ptr[9] * y[2 + 3 * image_width] + Q_mat_ptr[10] * y[2 + 4 * image_width] + Q_mat_ptr[11] * y[2 + 5 * image_width]) * Q_mat_ptr[2] + (Q_mat_ptr[6] * y[3] + Q_mat_ptr[7] * y[3 + image_width] + Q_mat_ptr[8] * y[3 + 2 * image_width] + Q_mat_ptr[9] * y[3 + 3 * image_width] + Q_mat_ptr[10] * y[3 + 4 * image_width] + Q_mat_ptr[11] * y[3 + 5 * image_width]) * Q_mat_ptr[3] + (Q_mat_ptr[6] * y[4] + Q_mat_ptr[7] * y[4 + image_width] + Q_mat_ptr[8] * y[4 + 2 * image_width] + Q_mat_ptr[9] * y[4 + 3 * image_width] + Q_mat_ptr[10] * y[4 + 4 * image_width] + Q_mat_ptr[11] * y[4 + 5 * image_width]) * Q_mat_ptr[4] + (Q_mat_ptr[6] * y[5] + Q_mat_ptr[7] * y[5 + image_width] + Q_mat_ptr[8] * y[5 + 2 * image_width] + Q_mat_ptr[9] * y[5 + 3 * image_width] + Q_mat_ptr[10] * y[5 + 4 * image_width] + Q_mat_ptr[11] * y[5 + 5 * image_width]) * Q_mat_ptr[5];
	//			S_mat_ptr[5] = (Q_mat_ptr[6] * y[0] + Q_mat_ptr[7] * y[image_width] + Q_mat_ptr[8] * y[2 * image_width] + Q_mat_ptr[9] * y[3 * image_width] + Q_mat_ptr[10] * y[4 * image_width] + Q_mat_ptr[11] * y[5 * image_width]) * Q_mat_ptr[6] + (Q_mat_ptr[6] * y[1] + Q_mat_ptr[7] * y[1 + image_width] + Q_mat_ptr[8] * y[1 + 2 * image_width] + Q_mat_ptr[9] * y[1 + 3 * image_width] + Q_mat_ptr[10] * y[1 + 4 * image_width] + Q_mat_ptr[11] * y[1 + 5 * image_width]) * Q_mat_ptr[7] + (Q_mat_ptr[6] * y[2] + Q_mat_ptr[7] * y[2 + image_width] + Q_mat_ptr[8] * y[2 + 2 * image_width] + Q_mat_ptr[9] * y[2 + 3 * image_width] + Q_mat_ptr[10] * y[2 + 4 * image_width] + Q_mat_ptr[11] * y[2 + 5 * image_width]) * Q_mat_ptr[8] + (Q_mat_ptr[6] * y[3] + Q_mat_ptr[7] * y[3 + image_width] + Q_mat_ptr[8] * y[3 + 2 * image_width] + Q_mat_ptr[9] * y[3 + 3 * image_width] + Q_mat_ptr[10] * y[3 + 4 * image_width] + Q_mat_ptr[11] * y[3 + 5 * image_width]) * Q_mat_ptr[9] + (Q_mat_ptr[6] * y[4] + Q_mat_ptr[7] * y[4 + image_width] + Q_mat_ptr[8] * y[4 + 2 * image_width] + Q_mat_ptr[9] * y[4 + 3 * image_width] + Q_mat_ptr[10] * y[4 + 4 * image_width] + Q_mat_ptr[11] * y[4 + 5 * image_width]) * Q_mat_ptr[10] + (Q_mat_ptr[6] * y[5] + Q_mat_ptr[7] * y[5 + image_width] + Q_mat_ptr[8] * y[5 + 2 * image_width] + Q_mat_ptr[9] * y[5 + 3 * image_width] + Q_mat_ptr[10] * y[5 + 4 * image_width] + Q_mat_ptr[11] * y[5 + 5 * image_width]) * Q_mat_ptr[11];
	//			S_mat_ptr[6] = (Q_mat_ptr[6] * y[0] + Q_mat_ptr[7] * y[image_width] + Q_mat_ptr[8] * y[2 * image_width] + Q_mat_ptr[9] * y[3 * image_width] + Q_mat_ptr[10] * y[4 * image_width] + Q_mat_ptr[11] * y[5 * image_width]) * Q_mat_ptr[12] + (Q_mat_ptr[6] * y[1] + Q_mat_ptr[7] * y[1 + image_width] + Q_mat_ptr[8] * y[1 + 2 * image_width] + Q_mat_ptr[9] * y[1 + 3 * image_width] + Q_mat_ptr[10] * y[1 + 4 * image_width] + Q_mat_ptr[11] * y[1 + 5 * image_width]) * Q_mat_ptr[13] + (Q_mat_ptr[6] * y[2] + Q_mat_ptr[7] * y[2 + image_width] + Q_mat_ptr[8] * y[2 + 2 * image_width] + Q_mat_ptr[9] * y[2 + 3 * image_width] + Q_mat_ptr[10] * y[2 + 4 * image_width] + Q_mat_ptr[11] * y[2 + 5 * image_width]) * Q_mat_ptr[14] + (Q_mat_ptr[6] * y[3] + Q_mat_ptr[7] * y[3 + image_width] + Q_mat_ptr[8] * y[3 + 2 * image_width] + Q_mat_ptr[9] * y[3 + 3 * image_width] + Q_mat_ptr[10] * y[3 + 4 * image_width] + Q_mat_ptr[11] * y[3 + 5 * image_width]) * Q_mat_ptr[15] + (Q_mat_ptr[6] * y[4] + Q_mat_ptr[7] * y[4 + image_width] + Q_mat_ptr[8] * y[4 + 2 * image_width] + Q_mat_ptr[9] * y[4 + 3 * image_width] + Q_mat_ptr[10] * y[4 + 4 * image_width] + Q_mat_ptr[11] * y[4 + 5 * image_width]) * Q_mat_ptr[16] + (Q_mat_ptr[6] * y[5] + Q_mat_ptr[7] * y[5 + image_width] + Q_mat_ptr[8] * y[5 + 2 * image_width] + Q_mat_ptr[9] * y[5 + 3 * image_width] + Q_mat_ptr[10] * y[5 + 4 * image_width] + Q_mat_ptr[11] * y[5 + 5 * image_width]) * Q_mat_ptr[17];
	//			S_mat_ptr[7] = Q_mat_ptr[6] * y[2] + Q_mat_ptr[7] * y[2 + image_width] + Q_mat_ptr[8] * y[2 + 2 * image_width] + Q_mat_ptr[9] * y[2 + 3 * image_width] + Q_mat_ptr[10] * y[2 + 4 * image_width] + Q_mat_ptr[11] * y[2 + 5 * image_width];
	//			S_mat_ptr[8] = (Q_mat_ptr[12] * y[0] + Q_mat_ptr[13] * y[image_width] + Q_mat_ptr[14] * y[2 * image_width] + Q_mat_ptr[15] * y[3 * image_width] + Q_mat_ptr[16] * y[4 * image_width] + Q_mat_ptr[17] * y[5 * image_width]) * Q_mat_ptr[0] + (Q_mat_ptr[12] * y[1] + Q_mat_ptr[13] * y[1 + image_width] + Q_mat_ptr[14] * y[1 + 2 * image_width] + Q_mat_ptr[15] * y[1 + 3 * image_width] + Q_mat_ptr[16] * y[1 + 4 * image_width] + Q_mat_ptr[17] * y[1 + 5 * image_width]) * Q_mat_ptr[1] + (Q_mat_ptr[12] * y[2] + Q_mat_ptr[13] * y[2 + image_width] + Q_mat_ptr[14] * y[2 + 2 * image_width] + Q_mat_ptr[15] * y[2 + 3 * image_width] + Q_mat_ptr[16] * y[2 + 4 * image_width] + Q_mat_ptr[17] * y[2 + 5 * image_width]) * Q_mat_ptr[2] + (Q_mat_ptr[12] * y[3] + Q_mat_ptr[13] * y[3 + image_width] + Q_mat_ptr[14] * y[3 + 2 * image_width] + Q_mat_ptr[15] * y[3 + 3 * image_width] + Q_mat_ptr[16] * y[3 + 4 * image_width] + Q_mat_ptr[17] * y[3 + 5 * image_width]) * Q_mat_ptr[3] + (Q_mat_ptr[12] * y[4] + Q_mat_ptr[13] * y[4 + image_width] + Q_mat_ptr[14] * y[4 + 2 * image_width] + Q_mat_ptr[15] * y[4 + 3 * image_width] + Q_mat_ptr[16] * y[4 + 4 * image_width] + Q_mat_ptr[17] * y[4 + 5 * image_width]) * Q_mat_ptr[4] + (Q_mat_ptr[12] * y[5] + Q_mat_ptr[13] * y[5 + image_width] + Q_mat_ptr[14] * y[5 + 2 * image_width] + Q_mat_ptr[15] * y[5 + 3 * image_width] + Q_mat_ptr[16] * y[5 + 4 * image_width] + Q_mat_ptr[17] * y[5 + 5 * image_width]) * Q_mat_ptr[5];
	//			S_mat_ptr[9] = (Q_mat_ptr[12] * y[0] + Q_mat_ptr[13] * y[image_width] + Q_mat_ptr[14] * y[2 * image_width] + Q_mat_ptr[15] * y[3 * image_width] + Q_mat_ptr[16] * y[4 * image_width] + Q_mat_ptr[17] * y[5 * image_width]) * Q_mat_ptr[6] + (Q_mat_ptr[12] * y[1] + Q_mat_ptr[13] * y[1 + image_width] + Q_mat_ptr[14] * y[1 + 2 * image_width] + Q_mat_ptr[15] * y[1 + 3 * image_width] + Q_mat_ptr[16] * y[1 + 4 * image_width] + Q_mat_ptr[17] * y[1 + 5 * image_width]) * Q_mat_ptr[7] + (Q_mat_ptr[12] * y[2] + Q_mat_ptr[13] * y[2 + image_width] + Q_mat_ptr[14] * y[2 + 2 * image_width] + Q_mat_ptr[15] * y[2 + 3 * image_width] + Q_mat_ptr[16] * y[2 + 4 * image_width] + Q_mat_ptr[17] * y[2 + 5 * image_width]) * Q_mat_ptr[8] + (Q_mat_ptr[12] * y[3] + Q_mat_ptr[13] * y[3 + image_width] + Q_mat_ptr[14] * y[3 + 2 * image_width] + Q_mat_ptr[15] * y[3 + 3 * image_width] + Q_mat_ptr[16] * y[3 + 4 * image_width] + Q_mat_ptr[17] * y[3 + 5 * image_width]) * Q_mat_ptr[9] + (Q_mat_ptr[12] * y[4] + Q_mat_ptr[13] * y[4 + image_width] + Q_mat_ptr[14] * y[4 + 2 * image_width] + Q_mat_ptr[15] * y[4 + 3 * image_width] + Q_mat_ptr[16] * y[4 + 4 * image_width] + Q_mat_ptr[17] * y[4 + 5 * image_width]) * Q_mat_ptr[10] + (Q_mat_ptr[12] * y[5] + Q_mat_ptr[13] * y[5 + image_width] + Q_mat_ptr[14] * y[5 + 2 * image_width] + Q_mat_ptr[15] * y[5 + 3 * image_width] + Q_mat_ptr[16] * y[5 + 4 * image_width] + Q_mat_ptr[17] * y[5 + 5 * image_width]) * Q_mat_ptr[11];
	//			S_mat_ptr[10] = (Q_mat_ptr[12] * y[0] + Q_mat_ptr[13] * y[image_width] + Q_mat_ptr[14] * y[2 * image_width] + Q_mat_ptr[15] * y[3 * image_width] + Q_mat_ptr[16] * y[4 * image_width] + Q_mat_ptr[17] * y[5 * image_width]) * Q_mat_ptr[12] + (Q_mat_ptr[12] * y[1] + Q_mat_ptr[13] * y[1 + image_width] + Q_mat_ptr[14] * y[1 + 2 * image_width] + Q_mat_ptr[15] * y[1 + 3 * image_width] + Q_mat_ptr[16] * y[1 + 4 * image_width] + Q_mat_ptr[17] * y[1 + 5 * image_width]) * Q_mat_ptr[13] + (Q_mat_ptr[12] * y[2] + Q_mat_ptr[13] * y[2 + image_width] + Q_mat_ptr[14] * y[2 + 2 * image_width] + Q_mat_ptr[15] * y[2 + 3 * image_width] + Q_mat_ptr[16] * y[2 + 4 * image_width] + Q_mat_ptr[17] * y[2 + 5 * image_width]) * Q_mat_ptr[14] + (Q_mat_ptr[12] * y[3] + Q_mat_ptr[13] * y[3 + image_width] + Q_mat_ptr[14] * y[3 + 2 * image_width] + Q_mat_ptr[15] * y[3 + 3 * image_width] + Q_mat_ptr[16] * y[3 + 4 * image_width] + Q_mat_ptr[17] * y[3 + 5 * image_width]) * Q_mat_ptr[15] + (Q_mat_ptr[12] * y[4] + Q_mat_ptr[13] * y[4 + image_width] + Q_mat_ptr[14] * y[4 + 2 * image_width] + Q_mat_ptr[15] * y[4 + 3 * image_width] + Q_mat_ptr[16] * y[4 + 4 * image_width] + Q_mat_ptr[17] * y[4 + 5 * image_width]) * Q_mat_ptr[16] + (Q_mat_ptr[12] * y[5] + Q_mat_ptr[13] * y[5 + image_width] + Q_mat_ptr[14] * y[5 + 2 * image_width] + Q_mat_ptr[15] * y[5 + 3 * image_width] + Q_mat_ptr[16] * y[5 + 4 * image_width] + Q_mat_ptr[17] * y[5 + 5 * image_width]) * Q_mat_ptr[17];
	//			S_mat_ptr[11] = Q_mat_ptr[12] * y[2] + Q_mat_ptr[13] * y[2 + image_width] + Q_mat_ptr[14] * y[2 + 2 * image_width] + Q_mat_ptr[15] * y[2 + 3 * image_width] + Q_mat_ptr[16] * y[2 + 4 * image_width] + Q_mat_ptr[17] * y[2 + 5 * image_width];
	//			S_mat_ptr[12] = y[2 * image_width] * Q_mat_ptr[0] + y[1 + 2 * image_width] * Q_mat_ptr[1] + Q_mat_ptr[2] * y[2 + 2 * image_width] + y[3 + 2 * image_width] * Q_mat_ptr[3] + y[4 + 2 * image_width] * Q_mat_ptr[4] + y[5 + 2 * image_width] * Q_mat_ptr[5];
	//			S_mat_ptr[13] = y[2 * image_width] * Q_mat_ptr[6] + y[1 + 2 * image_width] * Q_mat_ptr[7] + Q_mat_ptr[8] * y[2 + 2 * image_width] + y[3 + 2 * image_width] * Q_mat_ptr[9] + y[4 + 2 * image_width] * Q_mat_ptr[10] + y[5 + 2 * image_width] * Q_mat_ptr[11];
	//			S_mat_ptr[14] = y[2 * image_width] * Q_mat_ptr[12] + y[1 + 2 * image_width] * Q_mat_ptr[13] + Q_mat_ptr[14] * y[2 + 2 * image_width] + y[3 + 2 * image_width] * Q_mat_ptr[15] + y[4 + 2 * image_width] * Q_mat_ptr[16] + y[5 + 2 * image_width] * Q_mat_ptr[17];
	//			S_mat_ptr[15] = y[2 + 2 * image_width];
	//			CV_MAT_ELEM(*interpolation_LUT_valid_sign_mat,uchar,index_y - interpolation_area_y_min,index_x - interpolation_area_x_min) =1;		
	//		}
	//		//g
	//		CV_MAT_ELEM(*g_mat,double,m+template_half_height,n+template_half_width) =
	//			delta_x*delta_x*delta_x * (delta_y*delta_y*delta_y * S_mat_ptr[0] + delta_y*delta_y * S_mat_ptr[4] + delta_y * S_mat_ptr[8] + S_mat_ptr[12] ) +
	//			delta_x*delta_x * (delta_y*delta_y*delta_y * S_mat_ptr[1] + delta_y*delta_y * S_mat_ptr[5] + delta_y * S_mat_ptr[9] + S_mat_ptr[13] ) +
	//			delta_x  * (delta_y*delta_y*delta_y * S_mat_ptr[2] + delta_y*delta_y * S_mat_ptr[6] + delta_y * S_mat_ptr[10] + S_mat_ptr[14] ) +
	//			(delta_y*delta_y*delta_y * S_mat_ptr[3] + delta_y*delta_y * S_mat_ptr[7] + delta_y * S_mat_ptr[11] + S_mat_ptr[15] );

	//		//CV_MAT_ELEM(*g_mat,double,m+template_half_height,n+template_half_width) =
	//		//	3.0 * delta_x*delta_x * (delta_y*delta_y*delta_y * S_mat_ptr[0] + delta_y*delta_y * S_mat_ptr[4] + delta_y * S_mat_ptr[8] + S_mat_ptr[12] ) +
	//		//	2.0 * delta_x  * (delta_y*delta_y*delta_y * S_mat_ptr[1] + delta_y*delta_y * S_mat_ptr[5] + delta_y * S_mat_ptr[9] + S_mat_ptr[13] ) +
	//		//	(delta_y*delta_y*delta_y * S_mat_ptr[2] + delta_y*delta_y * S_mat_ptr[6] + delta_y * S_mat_ptr[10] + S_mat_ptr[14] );
	//		//CV_MAT_ELEM(*g_mat,double,m+template_half_height,n+template_half_width) =
	//		//	delta_x*delta_x*delta_x * (3.0 * delta_y*delta_y * S_mat_ptr[0] + 2.0 * delta_y * S_mat_ptr[4] + S_mat_ptr[8] ) +
	//		//	delta_x*delta_x * (3.0 * delta_y*delta_y * S_mat_ptr[1] +  2.0 * delta_y * S_mat_ptr[5] + S_mat_ptr[9] ) +
	//		//	delta_x  * (3.0 * delta_y*delta_y * S_mat_ptr[2] +  2.0 * delta_y * S_mat_ptr[6] + S_mat_ptr[10] ) +
	//		//	(3.0 * delta_y*delta_y * S_mat_ptr[3] +  2.0 * delta_y * S_mat_ptr[7] + S_mat_ptr[11] );
	//	}
	//}


	//return true;
}
bool _6x6UniformBicubicSplineInterpolationMethod::Integer_Points_Gradient_One_Template( 
	CvMat* image, 
	CvMat* Q_mat, 
	int center_x_in_image, 
	int center_y_in_image, 
	CvMat* referece_template_gx_mat,
	CvMat* referece_template_gy_mat 
	)
{
	//判断 大小
	const int image_width= image->width, image_height = image->height;
	const int template_width = referece_template_gx_mat->cols;
	const int template_height = referece_template_gx_mat->rows;
	const int templ_radius = (template_width-1)/2;
	const int templ_radius_h = (template_height-1)/2;

	if (Q_mat->rows!=4||Q_mat->cols!=6) {
		return false;
	}

	//类型
	int image_type = 0;//1--CV_8U,2--CV_64F
	if (CV_MAT_TYPE(image->type) ==CV_8U ) {
		image_type = 1;
	}
	else if (CV_MAT_TYPE(image->type) ==CV_64F) {
		image_type = 2;
	} 
	else {
		return false;
	}
	if (CV_MAT_TYPE(Q_mat->type) !=CV_64F ) {
		return false;
	}

	if (
		CV_MAT_TYPE(referece_template_gx_mat->type) != CV_64F||
		CV_MAT_TYPE(referece_template_gy_mat->type) != CV_64F
		) {
		return false;
	}

	double* _Qptr = Q_mat->data.db;

	if (image_type==1)
	{
		for (int m =-templ_radius_h;m<=templ_radius_h;m++)
		{
			for (int n =-templ_radius;n<=templ_radius;n++)
			{
				int index_i_min = center_y_in_image+m-2;
				int index_j_min = center_x_in_image+n-2;

				if (index_i_min  <0 || index_i_min+5 > image_height-1 ||
					index_j_min  <0 || index_j_min+5 > image_width-1)
				{
					return false;
				}

				const auto y=image->data.ptr +index_i_min*image_width+index_j_min;
				CV_MAT_ELEM(*referece_template_gx_mat,double,m+templ_radius_h,n+templ_radius) = 
					(double)(
						_Qptr[12]*y[2*image_width]   + 
						_Qptr[13]*y[1+2*image_width] +
						_Qptr[14]*y[2+2*image_width] + 
						_Qptr[15]*y[3+2*image_width] +
						_Qptr[16]*y[4+2*image_width] +
						_Qptr[17]*y[5+2*image_width]
				);
				CV_MAT_ELEM(*referece_template_gy_mat,double,m+templ_radius_h,n+templ_radius) = (double)(
					_Qptr[12] * y[2] + 
					_Qptr[13] * y[2 + image_width] + 
					_Qptr[14] * y[2 + 2 * image_width] + 
					_Qptr[15] * y[2 + 3 * image_width] + 
					_Qptr[16] * y[2 + 4 * image_width] + 
					_Qptr[17] * y[2 + 5 * image_width]
				);
			}
		}
	}
	else
	{
		for (int m =-templ_radius_h;m<=templ_radius_h;m++)
		{
			for (int n =-templ_radius;n<=templ_radius;n++)
			{
				const int index_i_min = center_y_in_image+m-2;
				const int index_j_min = center_x_in_image+n-2;
				if (index_i_min  <0 || index_i_min + 5 > image_height-1 ||
					index_j_min  <0 || index_j_min + 5> image_width-1)
				{
					return false;
				}

				const auto y = image->data.db + index_i_min * image_width + index_j_min;
				CV_MAT_ELEM(*referece_template_gx_mat,double,m+templ_radius_h,n+templ_radius) = (double)(
					y[2 * image_width] * _Qptr[12] + 
					y[1 + 2 * image_width] * _Qptr[13] +
					_Qptr[14] * y[2 + 2 * image_width] +
					y[3 + 2 * image_width] * _Qptr[15] +
					y[4 + 2 * image_width] * _Qptr[16] +
					y[5 + 2 * image_width] * _Qptr[17]
				);
				CV_MAT_ELEM(*referece_template_gy_mat,double,m+templ_radius_h,n+templ_radius) = (double)(
					_Qptr[12] * y[2] + 
					_Qptr[13] * y[2 + image_width] + 
					_Qptr[14] * y[2 + 2 * image_width] + 
					_Qptr[15] * y[2 + 3 * image_width] + 
					_Qptr[16] * y[2 + 4 * image_width] + 
					_Qptr[17] * y[2 + 5 * image_width]
				);
			}
		}
	}

	return true;
}

//////////////////////////////////////////////////////////////////////////
//12参数
bool _6x6UniformBicubicSplineInterpolationMethod::Value_One_Affine_Template_64F_12_Parameter(
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
	)
{
	//判断
	//大小
	int image_width= image->width;
	int image_height = image->height;
	int template_width = g_mat->cols;
	int template_height = g_mat->cols;
	int lut_width = interpolation_LUT->cols;
	int lut_height = interpolation_LUT->rows;


	if (interpolation_area_x_min<0 || interpolation_area_x_max>=image_width||
		interpolation_area_y_min<0 || interpolation_area_y_max>=image_height)
	{
		return false;
	}
	int interpolation_area_width = interpolation_area_x_max-interpolation_area_x_min;
	int interpolation_area_height = interpolation_area_y_max-interpolation_area_y_min;

	if (interpolation_LUT->cols != interpolation_area_width*Interpolation_LUT_Element_Size||
		interpolation_LUT->rows !=interpolation_area_height||
		interpolation_LUT_valid_sign_mat->cols!=interpolation_area_width||
		interpolation_LUT_valid_sign_mat->rows!= interpolation_area_height)
	{
		return false;
	}

	if (uniform_bicubic_natrue_spline_interpolation_6x6_Q_mat->rows!=4 || uniform_bicubic_natrue_spline_interpolation_6x6_Q_mat->cols!=6)
	{
		return false;
	}



	//类型
	if (CV_MAT_TYPE(image->type) !=CV_64F)
	{
		return false;
	}

	if (CV_MAT_TYPE(interpolation_LUT->type) != CV_32F)
	{
		return false;
	}

	if (CV_MAT_TYPE(g_mat->type) != CV_64F || CV_MAT_TYPE(uniform_bicubic_natrue_spline_interpolation_6x6_Q_mat->type) != CV_64F || CV_MAT_TYPE(interpolation_LUT_valid_sign_mat->type)!=CV_8U)
	{
		return false;
	}

	int template_half_width = (template_width-1)/2;
	int template_half_height = (template_height-1)/2;
	double* Q_mat_ptr = uniform_bicubic_natrue_spline_interpolation_6x6_Q_mat->data.db;


	double U = CV_MAT_ELEM(*template_transform_mat,double,0,0);
	double U_x = CV_MAT_ELEM(*template_transform_mat,double,1,0);
	double U_y = CV_MAT_ELEM(*template_transform_mat,double,2,0);
	double U_xx = CV_MAT_ELEM(*template_transform_mat,double,3,0);	
	double U_yy = CV_MAT_ELEM(*template_transform_mat,double,4,0);
	double U_xy = CV_MAT_ELEM(*template_transform_mat,double,5,0);
	double V = CV_MAT_ELEM(*template_transform_mat,double,6,0);
	double V_x = CV_MAT_ELEM(*template_transform_mat,double,7,0);
	double V_y = CV_MAT_ELEM(*template_transform_mat,double,8,0);
	double V_xx = CV_MAT_ELEM(*template_transform_mat,double,9,0);
	double V_yy = CV_MAT_ELEM(*template_transform_mat,double,10,0);
	double V_xy = CV_MAT_ELEM(*template_transform_mat,double,11,0);



	for (int m =-template_half_height;m<=template_half_height;m++)
	{
		for (int n =-template_half_width;n<=template_half_width;n++)
		{
			double pos_x = 
				//double(x_pos) + 
				U +
				n +
				U_x * n + 
				U_y * m + 				
				0.5 * U_xx * n * n + 
				0.5 * U_yy * m * m + 
				U_xy * n * m;
			double pos_y = 
				//double(y_pos) +
				V + 
				m + 
				V_x * n + 
				V_y * m + 				
				0.5 * V_xx * n * n + 
				0.5 * V_yy * m * m + 
				V_xy * n * m;


			//第一种
			//double pos_x =template_center_x + n + template_ux * n + template_uy * m;
			//double pos_y =template_center_y + m + template_vx * n + template_vy * m;


			//以下与相对位移方法相同
			int index_x = cvFloor(pos_x);
			int index_y = cvFloor(pos_y);

			if (index_y - interpolation_area_y_min <0 || index_y - interpolation_area_y_min >= interpolation_LUT_valid_sign_mat->rows-1 ||
				index_x - interpolation_area_x_min <0 || index_x - interpolation_area_x_min >= interpolation_LUT_valid_sign_mat->cols-1)
			{
				return false;
			}

			double delta_x = pos_x - index_x;
			double delta_y = pos_y - index_y;


			uchar sign = CV_MAT_ELEM(*interpolation_LUT_valid_sign_mat,uchar,index_y - interpolation_area_y_min,index_x - interpolation_area_x_min);
			float* S_mat_ptr = interpolation_LUT->data.fl+ ((index_y - interpolation_area_y_min) * (interpolation_area_x_max-interpolation_area_x_min)+index_x - interpolation_area_x_min) * Interpolation_LUT_Element_Size;
			if (sign!=1)
			{							
				double* y = image->data.db + (index_y-2)*image_width+index_x-2;
				S_mat_ptr[0] = (Q_mat_ptr[0] * y[0] + Q_mat_ptr[1] * y[image_width] + Q_mat_ptr[2] * y[2 * image_width] + Q_mat_ptr[3] * y[3 * image_width] + Q_mat_ptr[4] * y[4 * image_width] + Q_mat_ptr[5] * y[5 * image_width]) * Q_mat_ptr[0] + (Q_mat_ptr[0] * y[1] + Q_mat_ptr[1] * y[1 + image_width] + Q_mat_ptr[2] * y[1 + 2 * image_width] + Q_mat_ptr[3] * y[1 + 3 * image_width] + Q_mat_ptr[4] * y[1 + 4 * image_width] + Q_mat_ptr[5] * y[1 + 5 * image_width]) * Q_mat_ptr[1] + (Q_mat_ptr[0] * y[2] + Q_mat_ptr[1] * y[2 + image_width] + Q_mat_ptr[2] * y[2 + 2 * image_width] + Q_mat_ptr[3] * y[2 + 3 * image_width] + Q_mat_ptr[4] * y[2 + 4 * image_width] + Q_mat_ptr[5] * y[2 + 5 * image_width]) * Q_mat_ptr[2] + (Q_mat_ptr[0] * y[3] + Q_mat_ptr[1] * y[3 + image_width] + Q_mat_ptr[2] * y[3 + 2 * image_width] + Q_mat_ptr[3] * y[3 + 3 * image_width] + Q_mat_ptr[4] * y[3 + 4 * image_width] + Q_mat_ptr[5] * y[3 + 5 * image_width]) * Q_mat_ptr[3] + (Q_mat_ptr[0] * y[4] + Q_mat_ptr[1] * y[4 + image_width] + Q_mat_ptr[2] * y[4 + 2 * image_width] + Q_mat_ptr[3] * y[4 + 3 * image_width] + Q_mat_ptr[4] * y[4 + 4 * image_width] + Q_mat_ptr[5] * y[4 + 5 * image_width]) * Q_mat_ptr[4] + (Q_mat_ptr[0] * y[5] + Q_mat_ptr[1] * y[5 + image_width] + Q_mat_ptr[2] * y[5 + 2 * image_width] + Q_mat_ptr[3] * y[5 + 3 * image_width] + Q_mat_ptr[4] * y[5 + 4 * image_width] + Q_mat_ptr[5] * y[5 + 5 * image_width]) * Q_mat_ptr[5];
				S_mat_ptr[1] = (Q_mat_ptr[0] * y[0] + Q_mat_ptr[1] * y[image_width] + Q_mat_ptr[2] * y[2 * image_width] + Q_mat_ptr[3] * y[3 * image_width] + Q_mat_ptr[4] * y[4 * image_width] + Q_mat_ptr[5] * y[5 * image_width]) * Q_mat_ptr[6] + (Q_mat_ptr[0] * y[1] + Q_mat_ptr[1] * y[1 + image_width] + Q_mat_ptr[2] * y[1 + 2 * image_width] + Q_mat_ptr[3] * y[1 + 3 * image_width] + Q_mat_ptr[4] * y[1 + 4 * image_width] + Q_mat_ptr[5] * y[1 + 5 * image_width]) * Q_mat_ptr[7] + (Q_mat_ptr[0] * y[2] + Q_mat_ptr[1] * y[2 + image_width] + Q_mat_ptr[2] * y[2 + 2 * image_width] + Q_mat_ptr[3] * y[2 + 3 * image_width] + Q_mat_ptr[4] * y[2 + 4 * image_width] + Q_mat_ptr[5] * y[2 + 5 * image_width]) * Q_mat_ptr[8] + (Q_mat_ptr[0] * y[3] + Q_mat_ptr[1] * y[3 + image_width] + Q_mat_ptr[2] * y[3 + 2 * image_width] + Q_mat_ptr[3] * y[3 + 3 * image_width] + Q_mat_ptr[4] * y[3 + 4 * image_width] + Q_mat_ptr[5] * y[3 + 5 * image_width]) * Q_mat_ptr[9] + (Q_mat_ptr[0] * y[4] + Q_mat_ptr[1] * y[4 + image_width] + Q_mat_ptr[2] * y[4 + 2 * image_width] + Q_mat_ptr[3] * y[4 + 3 * image_width] + Q_mat_ptr[4] * y[4 + 4 * image_width] + Q_mat_ptr[5] * y[4 + 5 * image_width]) * Q_mat_ptr[10] + (Q_mat_ptr[0] * y[5] + Q_mat_ptr[1] * y[5 + image_width] + Q_mat_ptr[2] * y[5 + 2 * image_width] + Q_mat_ptr[3] * y[5 + 3 * image_width] + Q_mat_ptr[4] * y[5 + 4 * image_width] + Q_mat_ptr[5] * y[5 + 5 * image_width]) * Q_mat_ptr[11];
				S_mat_ptr[2] = (Q_mat_ptr[0] * y[0] + Q_mat_ptr[1] * y[image_width] + Q_mat_ptr[2] * y[2 * image_width] + Q_mat_ptr[3] * y[3 * image_width] + Q_mat_ptr[4] * y[4 * image_width] + Q_mat_ptr[5] * y[5 * image_width]) * Q_mat_ptr[12] + (Q_mat_ptr[0] * y[1] + Q_mat_ptr[1] * y[1 + image_width] + Q_mat_ptr[2] * y[1 + 2 * image_width] + Q_mat_ptr[3] * y[1 + 3 * image_width] + Q_mat_ptr[4] * y[1 + 4 * image_width] + Q_mat_ptr[5] * y[1 + 5 * image_width]) * Q_mat_ptr[13] + (Q_mat_ptr[0] * y[2] + Q_mat_ptr[1] * y[2 + image_width] + Q_mat_ptr[2] * y[2 + 2 * image_width] + Q_mat_ptr[3] * y[2 + 3 * image_width] + Q_mat_ptr[4] * y[2 + 4 * image_width] + Q_mat_ptr[5] * y[2 + 5 * image_width]) * Q_mat_ptr[14] + (Q_mat_ptr[0] * y[3] + Q_mat_ptr[1] * y[3 + image_width] + Q_mat_ptr[2] * y[3 + 2 * image_width] + Q_mat_ptr[3] * y[3 + 3 * image_width] + Q_mat_ptr[4] * y[3 + 4 * image_width] + Q_mat_ptr[5] * y[3 + 5 * image_width]) * Q_mat_ptr[15] + (Q_mat_ptr[0] * y[4] + Q_mat_ptr[1] * y[4 + image_width] + Q_mat_ptr[2] * y[4 + 2 * image_width] + Q_mat_ptr[3] * y[4 + 3 * image_width] + Q_mat_ptr[4] * y[4 + 4 * image_width] + Q_mat_ptr[5] * y[4 + 5 * image_width]) * Q_mat_ptr[16] + (Q_mat_ptr[0] * y[5] + Q_mat_ptr[1] * y[5 + image_width] + Q_mat_ptr[2] * y[5 + 2 * image_width] + Q_mat_ptr[3] * y[5 + 3 * image_width] + Q_mat_ptr[4] * y[5 + 4 * image_width] + Q_mat_ptr[5] * y[5 + 5 * image_width]) * Q_mat_ptr[17];
				S_mat_ptr[3] = Q_mat_ptr[0] * y[2] + Q_mat_ptr[1] * y[2 + image_width] + Q_mat_ptr[2] * y[2 + 2 * image_width] + Q_mat_ptr[3] * y[2 + 3 * image_width] + Q_mat_ptr[4] * y[2 + 4 * image_width] + Q_mat_ptr[5] * y[2 + 5 * image_width];
				S_mat_ptr[4] = (Q_mat_ptr[6] * y[0] + Q_mat_ptr[7] * y[image_width] + Q_mat_ptr[8] * y[2 * image_width] + Q_mat_ptr[9] * y[3 * image_width] + Q_mat_ptr[10] * y[4 * image_width] + Q_mat_ptr[11] * y[5 * image_width]) * Q_mat_ptr[0] + (Q_mat_ptr[6] * y[1] + Q_mat_ptr[7] * y[1 + image_width] + Q_mat_ptr[8] * y[1 + 2 * image_width] + Q_mat_ptr[9] * y[1 + 3 * image_width] + Q_mat_ptr[10] * y[1 + 4 * image_width] + Q_mat_ptr[11] * y[1 + 5 * image_width]) * Q_mat_ptr[1] + (Q_mat_ptr[6] * y[2] + Q_mat_ptr[7] * y[2 + image_width] + Q_mat_ptr[8] * y[2 + 2 * image_width] + Q_mat_ptr[9] * y[2 + 3 * image_width] + Q_mat_ptr[10] * y[2 + 4 * image_width] + Q_mat_ptr[11] * y[2 + 5 * image_width]) * Q_mat_ptr[2] + (Q_mat_ptr[6] * y[3] + Q_mat_ptr[7] * y[3 + image_width] + Q_mat_ptr[8] * y[3 + 2 * image_width] + Q_mat_ptr[9] * y[3 + 3 * image_width] + Q_mat_ptr[10] * y[3 + 4 * image_width] + Q_mat_ptr[11] * y[3 + 5 * image_width]) * Q_mat_ptr[3] + (Q_mat_ptr[6] * y[4] + Q_mat_ptr[7] * y[4 + image_width] + Q_mat_ptr[8] * y[4 + 2 * image_width] + Q_mat_ptr[9] * y[4 + 3 * image_width] + Q_mat_ptr[10] * y[4 + 4 * image_width] + Q_mat_ptr[11] * y[4 + 5 * image_width]) * Q_mat_ptr[4] + (Q_mat_ptr[6] * y[5] + Q_mat_ptr[7] * y[5 + image_width] + Q_mat_ptr[8] * y[5 + 2 * image_width] + Q_mat_ptr[9] * y[5 + 3 * image_width] + Q_mat_ptr[10] * y[5 + 4 * image_width] + Q_mat_ptr[11] * y[5 + 5 * image_width]) * Q_mat_ptr[5];
				S_mat_ptr[5] = (Q_mat_ptr[6] * y[0] + Q_mat_ptr[7] * y[image_width] + Q_mat_ptr[8] * y[2 * image_width] + Q_mat_ptr[9] * y[3 * image_width] + Q_mat_ptr[10] * y[4 * image_width] + Q_mat_ptr[11] * y[5 * image_width]) * Q_mat_ptr[6] + (Q_mat_ptr[6] * y[1] + Q_mat_ptr[7] * y[1 + image_width] + Q_mat_ptr[8] * y[1 + 2 * image_width] + Q_mat_ptr[9] * y[1 + 3 * image_width] + Q_mat_ptr[10] * y[1 + 4 * image_width] + Q_mat_ptr[11] * y[1 + 5 * image_width]) * Q_mat_ptr[7] + (Q_mat_ptr[6] * y[2] + Q_mat_ptr[7] * y[2 + image_width] + Q_mat_ptr[8] * y[2 + 2 * image_width] + Q_mat_ptr[9] * y[2 + 3 * image_width] + Q_mat_ptr[10] * y[2 + 4 * image_width] + Q_mat_ptr[11] * y[2 + 5 * image_width]) * Q_mat_ptr[8] + (Q_mat_ptr[6] * y[3] + Q_mat_ptr[7] * y[3 + image_width] + Q_mat_ptr[8] * y[3 + 2 * image_width] + Q_mat_ptr[9] * y[3 + 3 * image_width] + Q_mat_ptr[10] * y[3 + 4 * image_width] + Q_mat_ptr[11] * y[3 + 5 * image_width]) * Q_mat_ptr[9] + (Q_mat_ptr[6] * y[4] + Q_mat_ptr[7] * y[4 + image_width] + Q_mat_ptr[8] * y[4 + 2 * image_width] + Q_mat_ptr[9] * y[4 + 3 * image_width] + Q_mat_ptr[10] * y[4 + 4 * image_width] + Q_mat_ptr[11] * y[4 + 5 * image_width]) * Q_mat_ptr[10] + (Q_mat_ptr[6] * y[5] + Q_mat_ptr[7] * y[5 + image_width] + Q_mat_ptr[8] * y[5 + 2 * image_width] + Q_mat_ptr[9] * y[5 + 3 * image_width] + Q_mat_ptr[10] * y[5 + 4 * image_width] + Q_mat_ptr[11] * y[5 + 5 * image_width]) * Q_mat_ptr[11];
				S_mat_ptr[6] = (Q_mat_ptr[6] * y[0] + Q_mat_ptr[7] * y[image_width] + Q_mat_ptr[8] * y[2 * image_width] + Q_mat_ptr[9] * y[3 * image_width] + Q_mat_ptr[10] * y[4 * image_width] + Q_mat_ptr[11] * y[5 * image_width]) * Q_mat_ptr[12] + (Q_mat_ptr[6] * y[1] + Q_mat_ptr[7] * y[1 + image_width] + Q_mat_ptr[8] * y[1 + 2 * image_width] + Q_mat_ptr[9] * y[1 + 3 * image_width] + Q_mat_ptr[10] * y[1 + 4 * image_width] + Q_mat_ptr[11] * y[1 + 5 * image_width]) * Q_mat_ptr[13] + (Q_mat_ptr[6] * y[2] + Q_mat_ptr[7] * y[2 + image_width] + Q_mat_ptr[8] * y[2 + 2 * image_width] + Q_mat_ptr[9] * y[2 + 3 * image_width] + Q_mat_ptr[10] * y[2 + 4 * image_width] + Q_mat_ptr[11] * y[2 + 5 * image_width]) * Q_mat_ptr[14] + (Q_mat_ptr[6] * y[3] + Q_mat_ptr[7] * y[3 + image_width] + Q_mat_ptr[8] * y[3 + 2 * image_width] + Q_mat_ptr[9] * y[3 + 3 * image_width] + Q_mat_ptr[10] * y[3 + 4 * image_width] + Q_mat_ptr[11] * y[3 + 5 * image_width]) * Q_mat_ptr[15] + (Q_mat_ptr[6] * y[4] + Q_mat_ptr[7] * y[4 + image_width] + Q_mat_ptr[8] * y[4 + 2 * image_width] + Q_mat_ptr[9] * y[4 + 3 * image_width] + Q_mat_ptr[10] * y[4 + 4 * image_width] + Q_mat_ptr[11] * y[4 + 5 * image_width]) * Q_mat_ptr[16] + (Q_mat_ptr[6] * y[5] + Q_mat_ptr[7] * y[5 + image_width] + Q_mat_ptr[8] * y[5 + 2 * image_width] + Q_mat_ptr[9] * y[5 + 3 * image_width] + Q_mat_ptr[10] * y[5 + 4 * image_width] + Q_mat_ptr[11] * y[5 + 5 * image_width]) * Q_mat_ptr[17];
				S_mat_ptr[7] = Q_mat_ptr[6] * y[2] + Q_mat_ptr[7] * y[2 + image_width] + Q_mat_ptr[8] * y[2 + 2 * image_width] + Q_mat_ptr[9] * y[2 + 3 * image_width] + Q_mat_ptr[10] * y[2 + 4 * image_width] + Q_mat_ptr[11] * y[2 + 5 * image_width];
				S_mat_ptr[8] = (Q_mat_ptr[12] * y[0] + Q_mat_ptr[13] * y[image_width] + Q_mat_ptr[14] * y[2 * image_width] + Q_mat_ptr[15] * y[3 * image_width] + Q_mat_ptr[16] * y[4 * image_width] + Q_mat_ptr[17] * y[5 * image_width]) * Q_mat_ptr[0] + (Q_mat_ptr[12] * y[1] + Q_mat_ptr[13] * y[1 + image_width] + Q_mat_ptr[14] * y[1 + 2 * image_width] + Q_mat_ptr[15] * y[1 + 3 * image_width] + Q_mat_ptr[16] * y[1 + 4 * image_width] + Q_mat_ptr[17] * y[1 + 5 * image_width]) * Q_mat_ptr[1] + (Q_mat_ptr[12] * y[2] + Q_mat_ptr[13] * y[2 + image_width] + Q_mat_ptr[14] * y[2 + 2 * image_width] + Q_mat_ptr[15] * y[2 + 3 * image_width] + Q_mat_ptr[16] * y[2 + 4 * image_width] + Q_mat_ptr[17] * y[2 + 5 * image_width]) * Q_mat_ptr[2] + (Q_mat_ptr[12] * y[3] + Q_mat_ptr[13] * y[3 + image_width] + Q_mat_ptr[14] * y[3 + 2 * image_width] + Q_mat_ptr[15] * y[3 + 3 * image_width] + Q_mat_ptr[16] * y[3 + 4 * image_width] + Q_mat_ptr[17] * y[3 + 5 * image_width]) * Q_mat_ptr[3] + (Q_mat_ptr[12] * y[4] + Q_mat_ptr[13] * y[4 + image_width] + Q_mat_ptr[14] * y[4 + 2 * image_width] + Q_mat_ptr[15] * y[4 + 3 * image_width] + Q_mat_ptr[16] * y[4 + 4 * image_width] + Q_mat_ptr[17] * y[4 + 5 * image_width]) * Q_mat_ptr[4] + (Q_mat_ptr[12] * y[5] + Q_mat_ptr[13] * y[5 + image_width] + Q_mat_ptr[14] * y[5 + 2 * image_width] + Q_mat_ptr[15] * y[5 + 3 * image_width] + Q_mat_ptr[16] * y[5 + 4 * image_width] + Q_mat_ptr[17] * y[5 + 5 * image_width]) * Q_mat_ptr[5];
				S_mat_ptr[9] = (Q_mat_ptr[12] * y[0] + Q_mat_ptr[13] * y[image_width] + Q_mat_ptr[14] * y[2 * image_width] + Q_mat_ptr[15] * y[3 * image_width] + Q_mat_ptr[16] * y[4 * image_width] + Q_mat_ptr[17] * y[5 * image_width]) * Q_mat_ptr[6] + (Q_mat_ptr[12] * y[1] + Q_mat_ptr[13] * y[1 + image_width] + Q_mat_ptr[14] * y[1 + 2 * image_width] + Q_mat_ptr[15] * y[1 + 3 * image_width] + Q_mat_ptr[16] * y[1 + 4 * image_width] + Q_mat_ptr[17] * y[1 + 5 * image_width]) * Q_mat_ptr[7] + (Q_mat_ptr[12] * y[2] + Q_mat_ptr[13] * y[2 + image_width] + Q_mat_ptr[14] * y[2 + 2 * image_width] + Q_mat_ptr[15] * y[2 + 3 * image_width] + Q_mat_ptr[16] * y[2 + 4 * image_width] + Q_mat_ptr[17] * y[2 + 5 * image_width]) * Q_mat_ptr[8] + (Q_mat_ptr[12] * y[3] + Q_mat_ptr[13] * y[3 + image_width] + Q_mat_ptr[14] * y[3 + 2 * image_width] + Q_mat_ptr[15] * y[3 + 3 * image_width] + Q_mat_ptr[16] * y[3 + 4 * image_width] + Q_mat_ptr[17] * y[3 + 5 * image_width]) * Q_mat_ptr[9] + (Q_mat_ptr[12] * y[4] + Q_mat_ptr[13] * y[4 + image_width] + Q_mat_ptr[14] * y[4 + 2 * image_width] + Q_mat_ptr[15] * y[4 + 3 * image_width] + Q_mat_ptr[16] * y[4 + 4 * image_width] + Q_mat_ptr[17] * y[4 + 5 * image_width]) * Q_mat_ptr[10] + (Q_mat_ptr[12] * y[5] + Q_mat_ptr[13] * y[5 + image_width] + Q_mat_ptr[14] * y[5 + 2 * image_width] + Q_mat_ptr[15] * y[5 + 3 * image_width] + Q_mat_ptr[16] * y[5 + 4 * image_width] + Q_mat_ptr[17] * y[5 + 5 * image_width]) * Q_mat_ptr[11];
				S_mat_ptr[10] = (Q_mat_ptr[12] * y[0] + Q_mat_ptr[13] * y[image_width] + Q_mat_ptr[14] * y[2 * image_width] + Q_mat_ptr[15] * y[3 * image_width] + Q_mat_ptr[16] * y[4 * image_width] + Q_mat_ptr[17] * y[5 * image_width]) * Q_mat_ptr[12] + (Q_mat_ptr[12] * y[1] + Q_mat_ptr[13] * y[1 + image_width] + Q_mat_ptr[14] * y[1 + 2 * image_width] + Q_mat_ptr[15] * y[1 + 3 * image_width] + Q_mat_ptr[16] * y[1 + 4 * image_width] + Q_mat_ptr[17] * y[1 + 5 * image_width]) * Q_mat_ptr[13] + (Q_mat_ptr[12] * y[2] + Q_mat_ptr[13] * y[2 + image_width] + Q_mat_ptr[14] * y[2 + 2 * image_width] + Q_mat_ptr[15] * y[2 + 3 * image_width] + Q_mat_ptr[16] * y[2 + 4 * image_width] + Q_mat_ptr[17] * y[2 + 5 * image_width]) * Q_mat_ptr[14] + (Q_mat_ptr[12] * y[3] + Q_mat_ptr[13] * y[3 + image_width] + Q_mat_ptr[14] * y[3 + 2 * image_width] + Q_mat_ptr[15] * y[3 + 3 * image_width] + Q_mat_ptr[16] * y[3 + 4 * image_width] + Q_mat_ptr[17] * y[3 + 5 * image_width]) * Q_mat_ptr[15] + (Q_mat_ptr[12] * y[4] + Q_mat_ptr[13] * y[4 + image_width] + Q_mat_ptr[14] * y[4 + 2 * image_width] + Q_mat_ptr[15] * y[4 + 3 * image_width] + Q_mat_ptr[16] * y[4 + 4 * image_width] + Q_mat_ptr[17] * y[4 + 5 * image_width]) * Q_mat_ptr[16] + (Q_mat_ptr[12] * y[5] + Q_mat_ptr[13] * y[5 + image_width] + Q_mat_ptr[14] * y[5 + 2 * image_width] + Q_mat_ptr[15] * y[5 + 3 * image_width] + Q_mat_ptr[16] * y[5 + 4 * image_width] + Q_mat_ptr[17] * y[5 + 5 * image_width]) * Q_mat_ptr[17];
				S_mat_ptr[11] = Q_mat_ptr[12] * y[2] + Q_mat_ptr[13] * y[2 + image_width] + Q_mat_ptr[14] * y[2 + 2 * image_width] + Q_mat_ptr[15] * y[2 + 3 * image_width] + Q_mat_ptr[16] * y[2 + 4 * image_width] + Q_mat_ptr[17] * y[2 + 5 * image_width];
				S_mat_ptr[12] = y[2 * image_width] * Q_mat_ptr[0] + y[1 + 2 * image_width] * Q_mat_ptr[1] + Q_mat_ptr[2] * y[2 + 2 * image_width] + y[3 + 2 * image_width] * Q_mat_ptr[3] + y[4 + 2 * image_width] * Q_mat_ptr[4] + y[5 + 2 * image_width] * Q_mat_ptr[5];
				S_mat_ptr[13] = y[2 * image_width] * Q_mat_ptr[6] + y[1 + 2 * image_width] * Q_mat_ptr[7] + Q_mat_ptr[8] * y[2 + 2 * image_width] + y[3 + 2 * image_width] * Q_mat_ptr[9] + y[4 + 2 * image_width] * Q_mat_ptr[10] + y[5 + 2 * image_width] * Q_mat_ptr[11];
				S_mat_ptr[14] = y[2 * image_width] * Q_mat_ptr[12] + y[1 + 2 * image_width] * Q_mat_ptr[13] + Q_mat_ptr[14] * y[2 + 2 * image_width] + y[3 + 2 * image_width] * Q_mat_ptr[15] + y[4 + 2 * image_width] * Q_mat_ptr[16] + y[5 + 2 * image_width] * Q_mat_ptr[17];
				S_mat_ptr[15] = y[2 + 2 * image_width];
				CV_MAT_ELEM(*interpolation_LUT_valid_sign_mat,uchar,index_y - interpolation_area_y_min,index_x - interpolation_area_x_min) =1;		
			}
			//g
			CV_MAT_ELEM(*g_mat,double,m+template_half_height,n+template_half_width) =
				delta_x*delta_x*delta_x * (delta_y*delta_y*delta_y * S_mat_ptr[0] + delta_y*delta_y * S_mat_ptr[4] + delta_y * S_mat_ptr[8] + S_mat_ptr[12] ) +
				delta_x*delta_x * (delta_y*delta_y*delta_y * S_mat_ptr[1] + delta_y*delta_y * S_mat_ptr[5] + delta_y * S_mat_ptr[9] + S_mat_ptr[13] ) +
				delta_x  * (delta_y*delta_y*delta_y * S_mat_ptr[2] + delta_y*delta_y * S_mat_ptr[6] + delta_y * S_mat_ptr[10] + S_mat_ptr[14] ) +
				(delta_y*delta_y*delta_y * S_mat_ptr[3] + delta_y*delta_y * S_mat_ptr[7] + delta_y * S_mat_ptr[11] + S_mat_ptr[15] );

			//CV_MAT_ELEM(*g_mat,double,m+template_half_height,n+template_half_width) =
			//	3.0 * delta_x*delta_x * (delta_y*delta_y*delta_y * S_mat_ptr[0] + delta_y*delta_y * S_mat_ptr[4] + delta_y * S_mat_ptr[8] + S_mat_ptr[12] ) +
			//	2.0 * delta_x  * (delta_y*delta_y*delta_y * S_mat_ptr[1] + delta_y*delta_y * S_mat_ptr[5] + delta_y * S_mat_ptr[9] + S_mat_ptr[13] ) +
			//	(delta_y*delta_y*delta_y * S_mat_ptr[2] + delta_y*delta_y * S_mat_ptr[6] + delta_y * S_mat_ptr[10] + S_mat_ptr[14] );
			//CV_MAT_ELEM(*g_mat,double,m+template_half_height,n+template_half_width) =
			//	delta_x*delta_x*delta_x * (3.0 * delta_y*delta_y * S_mat_ptr[0] + 2.0 * delta_y * S_mat_ptr[4] + S_mat_ptr[8] ) +
			//	delta_x*delta_x * (3.0 * delta_y*delta_y * S_mat_ptr[1] +  2.0 * delta_y * S_mat_ptr[5] + S_mat_ptr[9] ) +
			//	delta_x  * (3.0 * delta_y*delta_y * S_mat_ptr[2] +  2.0 * delta_y * S_mat_ptr[6] + S_mat_ptr[10] ) +
			//	(3.0 * delta_y*delta_y * S_mat_ptr[3] +  2.0 * delta_y * S_mat_ptr[7] + S_mat_ptr[11] );
		}
	}


	return true;
}

bool _6x6UniformBicubicSplineInterpolationMethod::Value_And_Gradient_One_Affine_Template_64F_12_Parameter(
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
	)
{
	//判断省略
	//大小
	int image_width= image->width;
	int image_height = image->height;
	int template_width = g_mat->cols;
	int template_height = g_mat->cols;
	int lut_width = interpolation_LUT->cols;
	int lut_height = interpolation_LUT->rows;


	int interpolation_area_width = interpolation_area_x_max-interpolation_area_x_min;
	int interpolation_area_height = interpolation_area_y_max-interpolation_area_y_min;

	int template_half_width = (template_width-1)/2;
	int template_half_height = (template_height-1)/2;
	double* Q_mat_ptr = uniform_bicubic_natrue_spline_interpolation_6x6_Q_mat->data.db;

	double U = CV_MAT_ELEM(*template_transform_mat,double,0,0);
	double U_x = CV_MAT_ELEM(*template_transform_mat,double,1,0);
	double U_y = CV_MAT_ELEM(*template_transform_mat,double,2,0);
	double U_xx = CV_MAT_ELEM(*template_transform_mat,double,3,0);	
	double U_yy = CV_MAT_ELEM(*template_transform_mat,double,4,0);
	double U_xy = CV_MAT_ELEM(*template_transform_mat,double,5,0);
	double V = CV_MAT_ELEM(*template_transform_mat,double,6,0);
	double V_x = CV_MAT_ELEM(*template_transform_mat,double,7,0);
	double V_y = CV_MAT_ELEM(*template_transform_mat,double,8,0);
	double V_xx = CV_MAT_ELEM(*template_transform_mat,double,9,0);
	double V_yy = CV_MAT_ELEM(*template_transform_mat,double,10,0);
	double V_xy = CV_MAT_ELEM(*template_transform_mat,double,11,0);


	for (int m =-template_half_height;m<=template_half_height;m++)
	{
		for (int n =-template_half_width;n<=template_half_width;n++)
		{
			double pos_x = 
				//double(x_pos) + 
				U +
				n +
				U_x * n + 
				U_y * m + 				
				0.5 * U_xx * n * n + 
				0.5 * U_yy * m * m + 
				U_xy * n * m;
			double pos_y = 
				//double(y_pos) +
				V + 
				m + 
				V_x * n + 
				V_y * m + 				
				0.5 * V_xx * n * n + 
				0.5 * V_yy * m * m + 
				V_xy * n * m;

			//x' = x0 + u +dx + dx*ux+dy*uy,  x = x0+u
			//double pos_x =reference_center_point_x + n + n* reference_ux + m * reference_vy;
			//double pos_y =reference_center_point_y + m + n* reference_vx + m * reference_vy;
			//下面方法与方形模板的相同

			int index_x = cvFloor(pos_x);
			int index_y = cvFloor(pos_y);
			if (index_y - interpolation_area_y_min <0 || index_y - interpolation_area_y_min >= interpolation_LUT_valid_sign_mat->rows-1 ||
				index_x - interpolation_area_x_min <0 || index_x - interpolation_area_x_min >= interpolation_LUT_valid_sign_mat->cols-1)
			{
				return false;
			}

			double delta_x = pos_x - index_x;
			double delta_y = pos_y - index_y;

			uchar sign = CV_MAT_ELEM(*interpolation_LUT_valid_sign_mat,uchar,index_y - interpolation_area_y_min,index_x - interpolation_area_x_min);
			float* S_mat_ptr = interpolation_LUT->data.fl+ ((index_y - interpolation_area_y_min) * (interpolation_area_x_max-interpolation_area_x_min)+index_x - interpolation_area_x_min) * Interpolation_LUT_Element_Size;
			if (sign!=1)
			{							
				double* y = image->data.db + (index_y-2)*image_width+index_x-2;
				S_mat_ptr[0] = (Q_mat_ptr[0] * y[0] + Q_mat_ptr[1] * y[image_width] + Q_mat_ptr[2] * y[2 * image_width] + Q_mat_ptr[3] * y[3 * image_width] + Q_mat_ptr[4] * y[4 * image_width] + Q_mat_ptr[5] * y[5 * image_width]) * Q_mat_ptr[0] + (Q_mat_ptr[0] * y[1] + Q_mat_ptr[1] * y[1 + image_width] + Q_mat_ptr[2] * y[1 + 2 * image_width] + Q_mat_ptr[3] * y[1 + 3 * image_width] + Q_mat_ptr[4] * y[1 + 4 * image_width] + Q_mat_ptr[5] * y[1 + 5 * image_width]) * Q_mat_ptr[1] + (Q_mat_ptr[0] * y[2] + Q_mat_ptr[1] * y[2 + image_width] + Q_mat_ptr[2] * y[2 + 2 * image_width] + Q_mat_ptr[3] * y[2 + 3 * image_width] + Q_mat_ptr[4] * y[2 + 4 * image_width] + Q_mat_ptr[5] * y[2 + 5 * image_width]) * Q_mat_ptr[2] + (Q_mat_ptr[0] * y[3] + Q_mat_ptr[1] * y[3 + image_width] + Q_mat_ptr[2] * y[3 + 2 * image_width] + Q_mat_ptr[3] * y[3 + 3 * image_width] + Q_mat_ptr[4] * y[3 + 4 * image_width] + Q_mat_ptr[5] * y[3 + 5 * image_width]) * Q_mat_ptr[3] + (Q_mat_ptr[0] * y[4] + Q_mat_ptr[1] * y[4 + image_width] + Q_mat_ptr[2] * y[4 + 2 * image_width] + Q_mat_ptr[3] * y[4 + 3 * image_width] + Q_mat_ptr[4] * y[4 + 4 * image_width] + Q_mat_ptr[5] * y[4 + 5 * image_width]) * Q_mat_ptr[4] + (Q_mat_ptr[0] * y[5] + Q_mat_ptr[1] * y[5 + image_width] + Q_mat_ptr[2] * y[5 + 2 * image_width] + Q_mat_ptr[3] * y[5 + 3 * image_width] + Q_mat_ptr[4] * y[5 + 4 * image_width] + Q_mat_ptr[5] * y[5 + 5 * image_width]) * Q_mat_ptr[5];
				S_mat_ptr[1] = (Q_mat_ptr[0] * y[0] + Q_mat_ptr[1] * y[image_width] + Q_mat_ptr[2] * y[2 * image_width] + Q_mat_ptr[3] * y[3 * image_width] + Q_mat_ptr[4] * y[4 * image_width] + Q_mat_ptr[5] * y[5 * image_width]) * Q_mat_ptr[6] + (Q_mat_ptr[0] * y[1] + Q_mat_ptr[1] * y[1 + image_width] + Q_mat_ptr[2] * y[1 + 2 * image_width] + Q_mat_ptr[3] * y[1 + 3 * image_width] + Q_mat_ptr[4] * y[1 + 4 * image_width] + Q_mat_ptr[5] * y[1 + 5 * image_width]) * Q_mat_ptr[7] + (Q_mat_ptr[0] * y[2] + Q_mat_ptr[1] * y[2 + image_width] + Q_mat_ptr[2] * y[2 + 2 * image_width] + Q_mat_ptr[3] * y[2 + 3 * image_width] + Q_mat_ptr[4] * y[2 + 4 * image_width] + Q_mat_ptr[5] * y[2 + 5 * image_width]) * Q_mat_ptr[8] + (Q_mat_ptr[0] * y[3] + Q_mat_ptr[1] * y[3 + image_width] + Q_mat_ptr[2] * y[3 + 2 * image_width] + Q_mat_ptr[3] * y[3 + 3 * image_width] + Q_mat_ptr[4] * y[3 + 4 * image_width] + Q_mat_ptr[5] * y[3 + 5 * image_width]) * Q_mat_ptr[9] + (Q_mat_ptr[0] * y[4] + Q_mat_ptr[1] * y[4 + image_width] + Q_mat_ptr[2] * y[4 + 2 * image_width] + Q_mat_ptr[3] * y[4 + 3 * image_width] + Q_mat_ptr[4] * y[4 + 4 * image_width] + Q_mat_ptr[5] * y[4 + 5 * image_width]) * Q_mat_ptr[10] + (Q_mat_ptr[0] * y[5] + Q_mat_ptr[1] * y[5 + image_width] + Q_mat_ptr[2] * y[5 + 2 * image_width] + Q_mat_ptr[3] * y[5 + 3 * image_width] + Q_mat_ptr[4] * y[5 + 4 * image_width] + Q_mat_ptr[5] * y[5 + 5 * image_width]) * Q_mat_ptr[11];
				S_mat_ptr[2] = (Q_mat_ptr[0] * y[0] + Q_mat_ptr[1] * y[image_width] + Q_mat_ptr[2] * y[2 * image_width] + Q_mat_ptr[3] * y[3 * image_width] + Q_mat_ptr[4] * y[4 * image_width] + Q_mat_ptr[5] * y[5 * image_width]) * Q_mat_ptr[12] + (Q_mat_ptr[0] * y[1] + Q_mat_ptr[1] * y[1 + image_width] + Q_mat_ptr[2] * y[1 + 2 * image_width] + Q_mat_ptr[3] * y[1 + 3 * image_width] + Q_mat_ptr[4] * y[1 + 4 * image_width] + Q_mat_ptr[5] * y[1 + 5 * image_width]) * Q_mat_ptr[13] + (Q_mat_ptr[0] * y[2] + Q_mat_ptr[1] * y[2 + image_width] + Q_mat_ptr[2] * y[2 + 2 * image_width] + Q_mat_ptr[3] * y[2 + 3 * image_width] + Q_mat_ptr[4] * y[2 + 4 * image_width] + Q_mat_ptr[5] * y[2 + 5 * image_width]) * Q_mat_ptr[14] + (Q_mat_ptr[0] * y[3] + Q_mat_ptr[1] * y[3 + image_width] + Q_mat_ptr[2] * y[3 + 2 * image_width] + Q_mat_ptr[3] * y[3 + 3 * image_width] + Q_mat_ptr[4] * y[3 + 4 * image_width] + Q_mat_ptr[5] * y[3 + 5 * image_width]) * Q_mat_ptr[15] + (Q_mat_ptr[0] * y[4] + Q_mat_ptr[1] * y[4 + image_width] + Q_mat_ptr[2] * y[4 + 2 * image_width] + Q_mat_ptr[3] * y[4 + 3 * image_width] + Q_mat_ptr[4] * y[4 + 4 * image_width] + Q_mat_ptr[5] * y[4 + 5 * image_width]) * Q_mat_ptr[16] + (Q_mat_ptr[0] * y[5] + Q_mat_ptr[1] * y[5 + image_width] + Q_mat_ptr[2] * y[5 + 2 * image_width] + Q_mat_ptr[3] * y[5 + 3 * image_width] + Q_mat_ptr[4] * y[5 + 4 * image_width] + Q_mat_ptr[5] * y[5 + 5 * image_width]) * Q_mat_ptr[17];
				S_mat_ptr[3] = Q_mat_ptr[0] * y[2] + Q_mat_ptr[1] * y[2 + image_width] + Q_mat_ptr[2] * y[2 + 2 * image_width] + Q_mat_ptr[3] * y[2 + 3 * image_width] + Q_mat_ptr[4] * y[2 + 4 * image_width] + Q_mat_ptr[5] * y[2 + 5 * image_width];
				S_mat_ptr[4] = (Q_mat_ptr[6] * y[0] + Q_mat_ptr[7] * y[image_width] + Q_mat_ptr[8] * y[2 * image_width] + Q_mat_ptr[9] * y[3 * image_width] + Q_mat_ptr[10] * y[4 * image_width] + Q_mat_ptr[11] * y[5 * image_width]) * Q_mat_ptr[0] + (Q_mat_ptr[6] * y[1] + Q_mat_ptr[7] * y[1 + image_width] + Q_mat_ptr[8] * y[1 + 2 * image_width] + Q_mat_ptr[9] * y[1 + 3 * image_width] + Q_mat_ptr[10] * y[1 + 4 * image_width] + Q_mat_ptr[11] * y[1 + 5 * image_width]) * Q_mat_ptr[1] + (Q_mat_ptr[6] * y[2] + Q_mat_ptr[7] * y[2 + image_width] + Q_mat_ptr[8] * y[2 + 2 * image_width] + Q_mat_ptr[9] * y[2 + 3 * image_width] + Q_mat_ptr[10] * y[2 + 4 * image_width] + Q_mat_ptr[11] * y[2 + 5 * image_width]) * Q_mat_ptr[2] + (Q_mat_ptr[6] * y[3] + Q_mat_ptr[7] * y[3 + image_width] + Q_mat_ptr[8] * y[3 + 2 * image_width] + Q_mat_ptr[9] * y[3 + 3 * image_width] + Q_mat_ptr[10] * y[3 + 4 * image_width] + Q_mat_ptr[11] * y[3 + 5 * image_width]) * Q_mat_ptr[3] + (Q_mat_ptr[6] * y[4] + Q_mat_ptr[7] * y[4 + image_width] + Q_mat_ptr[8] * y[4 + 2 * image_width] + Q_mat_ptr[9] * y[4 + 3 * image_width] + Q_mat_ptr[10] * y[4 + 4 * image_width] + Q_mat_ptr[11] * y[4 + 5 * image_width]) * Q_mat_ptr[4] + (Q_mat_ptr[6] * y[5] + Q_mat_ptr[7] * y[5 + image_width] + Q_mat_ptr[8] * y[5 + 2 * image_width] + Q_mat_ptr[9] * y[5 + 3 * image_width] + Q_mat_ptr[10] * y[5 + 4 * image_width] + Q_mat_ptr[11] * y[5 + 5 * image_width]) * Q_mat_ptr[5];
				S_mat_ptr[5] = (Q_mat_ptr[6] * y[0] + Q_mat_ptr[7] * y[image_width] + Q_mat_ptr[8] * y[2 * image_width] + Q_mat_ptr[9] * y[3 * image_width] + Q_mat_ptr[10] * y[4 * image_width] + Q_mat_ptr[11] * y[5 * image_width]) * Q_mat_ptr[6] + (Q_mat_ptr[6] * y[1] + Q_mat_ptr[7] * y[1 + image_width] + Q_mat_ptr[8] * y[1 + 2 * image_width] + Q_mat_ptr[9] * y[1 + 3 * image_width] + Q_mat_ptr[10] * y[1 + 4 * image_width] + Q_mat_ptr[11] * y[1 + 5 * image_width]) * Q_mat_ptr[7] + (Q_mat_ptr[6] * y[2] + Q_mat_ptr[7] * y[2 + image_width] + Q_mat_ptr[8] * y[2 + 2 * image_width] + Q_mat_ptr[9] * y[2 + 3 * image_width] + Q_mat_ptr[10] * y[2 + 4 * image_width] + Q_mat_ptr[11] * y[2 + 5 * image_width]) * Q_mat_ptr[8] + (Q_mat_ptr[6] * y[3] + Q_mat_ptr[7] * y[3 + image_width] + Q_mat_ptr[8] * y[3 + 2 * image_width] + Q_mat_ptr[9] * y[3 + 3 * image_width] + Q_mat_ptr[10] * y[3 + 4 * image_width] + Q_mat_ptr[11] * y[3 + 5 * image_width]) * Q_mat_ptr[9] + (Q_mat_ptr[6] * y[4] + Q_mat_ptr[7] * y[4 + image_width] + Q_mat_ptr[8] * y[4 + 2 * image_width] + Q_mat_ptr[9] * y[4 + 3 * image_width] + Q_mat_ptr[10] * y[4 + 4 * image_width] + Q_mat_ptr[11] * y[4 + 5 * image_width]) * Q_mat_ptr[10] + (Q_mat_ptr[6] * y[5] + Q_mat_ptr[7] * y[5 + image_width] + Q_mat_ptr[8] * y[5 + 2 * image_width] + Q_mat_ptr[9] * y[5 + 3 * image_width] + Q_mat_ptr[10] * y[5 + 4 * image_width] + Q_mat_ptr[11] * y[5 + 5 * image_width]) * Q_mat_ptr[11];
				S_mat_ptr[6] = (Q_mat_ptr[6] * y[0] + Q_mat_ptr[7] * y[image_width] + Q_mat_ptr[8] * y[2 * image_width] + Q_mat_ptr[9] * y[3 * image_width] + Q_mat_ptr[10] * y[4 * image_width] + Q_mat_ptr[11] * y[5 * image_width]) * Q_mat_ptr[12] + (Q_mat_ptr[6] * y[1] + Q_mat_ptr[7] * y[1 + image_width] + Q_mat_ptr[8] * y[1 + 2 * image_width] + Q_mat_ptr[9] * y[1 + 3 * image_width] + Q_mat_ptr[10] * y[1 + 4 * image_width] + Q_mat_ptr[11] * y[1 + 5 * image_width]) * Q_mat_ptr[13] + (Q_mat_ptr[6] * y[2] + Q_mat_ptr[7] * y[2 + image_width] + Q_mat_ptr[8] * y[2 + 2 * image_width] + Q_mat_ptr[9] * y[2 + 3 * image_width] + Q_mat_ptr[10] * y[2 + 4 * image_width] + Q_mat_ptr[11] * y[2 + 5 * image_width]) * Q_mat_ptr[14] + (Q_mat_ptr[6] * y[3] + Q_mat_ptr[7] * y[3 + image_width] + Q_mat_ptr[8] * y[3 + 2 * image_width] + Q_mat_ptr[9] * y[3 + 3 * image_width] + Q_mat_ptr[10] * y[3 + 4 * image_width] + Q_mat_ptr[11] * y[3 + 5 * image_width]) * Q_mat_ptr[15] + (Q_mat_ptr[6] * y[4] + Q_mat_ptr[7] * y[4 + image_width] + Q_mat_ptr[8] * y[4 + 2 * image_width] + Q_mat_ptr[9] * y[4 + 3 * image_width] + Q_mat_ptr[10] * y[4 + 4 * image_width] + Q_mat_ptr[11] * y[4 + 5 * image_width]) * Q_mat_ptr[16] + (Q_mat_ptr[6] * y[5] + Q_mat_ptr[7] * y[5 + image_width] + Q_mat_ptr[8] * y[5 + 2 * image_width] + Q_mat_ptr[9] * y[5 + 3 * image_width] + Q_mat_ptr[10] * y[5 + 4 * image_width] + Q_mat_ptr[11] * y[5 + 5 * image_width]) * Q_mat_ptr[17];
				S_mat_ptr[7] = Q_mat_ptr[6] * y[2] + Q_mat_ptr[7] * y[2 + image_width] + Q_mat_ptr[8] * y[2 + 2 * image_width] + Q_mat_ptr[9] * y[2 + 3 * image_width] + Q_mat_ptr[10] * y[2 + 4 * image_width] + Q_mat_ptr[11] * y[2 + 5 * image_width];
				S_mat_ptr[8] = (Q_mat_ptr[12] * y[0] + Q_mat_ptr[13] * y[image_width] + Q_mat_ptr[14] * y[2 * image_width] + Q_mat_ptr[15] * y[3 * image_width] + Q_mat_ptr[16] * y[4 * image_width] + Q_mat_ptr[17] * y[5 * image_width]) * Q_mat_ptr[0] + (Q_mat_ptr[12] * y[1] + Q_mat_ptr[13] * y[1 + image_width] + Q_mat_ptr[14] * y[1 + 2 * image_width] + Q_mat_ptr[15] * y[1 + 3 * image_width] + Q_mat_ptr[16] * y[1 + 4 * image_width] + Q_mat_ptr[17] * y[1 + 5 * image_width]) * Q_mat_ptr[1] + (Q_mat_ptr[12] * y[2] + Q_mat_ptr[13] * y[2 + image_width] + Q_mat_ptr[14] * y[2 + 2 * image_width] + Q_mat_ptr[15] * y[2 + 3 * image_width] + Q_mat_ptr[16] * y[2 + 4 * image_width] + Q_mat_ptr[17] * y[2 + 5 * image_width]) * Q_mat_ptr[2] + (Q_mat_ptr[12] * y[3] + Q_mat_ptr[13] * y[3 + image_width] + Q_mat_ptr[14] * y[3 + 2 * image_width] + Q_mat_ptr[15] * y[3 + 3 * image_width] + Q_mat_ptr[16] * y[3 + 4 * image_width] + Q_mat_ptr[17] * y[3 + 5 * image_width]) * Q_mat_ptr[3] + (Q_mat_ptr[12] * y[4] + Q_mat_ptr[13] * y[4 + image_width] + Q_mat_ptr[14] * y[4 + 2 * image_width] + Q_mat_ptr[15] * y[4 + 3 * image_width] + Q_mat_ptr[16] * y[4 + 4 * image_width] + Q_mat_ptr[17] * y[4 + 5 * image_width]) * Q_mat_ptr[4] + (Q_mat_ptr[12] * y[5] + Q_mat_ptr[13] * y[5 + image_width] + Q_mat_ptr[14] * y[5 + 2 * image_width] + Q_mat_ptr[15] * y[5 + 3 * image_width] + Q_mat_ptr[16] * y[5 + 4 * image_width] + Q_mat_ptr[17] * y[5 + 5 * image_width]) * Q_mat_ptr[5];
				S_mat_ptr[9] = (Q_mat_ptr[12] * y[0] + Q_mat_ptr[13] * y[image_width] + Q_mat_ptr[14] * y[2 * image_width] + Q_mat_ptr[15] * y[3 * image_width] + Q_mat_ptr[16] * y[4 * image_width] + Q_mat_ptr[17] * y[5 * image_width]) * Q_mat_ptr[6] + (Q_mat_ptr[12] * y[1] + Q_mat_ptr[13] * y[1 + image_width] + Q_mat_ptr[14] * y[1 + 2 * image_width] + Q_mat_ptr[15] * y[1 + 3 * image_width] + Q_mat_ptr[16] * y[1 + 4 * image_width] + Q_mat_ptr[17] * y[1 + 5 * image_width]) * Q_mat_ptr[7] + (Q_mat_ptr[12] * y[2] + Q_mat_ptr[13] * y[2 + image_width] + Q_mat_ptr[14] * y[2 + 2 * image_width] + Q_mat_ptr[15] * y[2 + 3 * image_width] + Q_mat_ptr[16] * y[2 + 4 * image_width] + Q_mat_ptr[17] * y[2 + 5 * image_width]) * Q_mat_ptr[8] + (Q_mat_ptr[12] * y[3] + Q_mat_ptr[13] * y[3 + image_width] + Q_mat_ptr[14] * y[3 + 2 * image_width] + Q_mat_ptr[15] * y[3 + 3 * image_width] + Q_mat_ptr[16] * y[3 + 4 * image_width] + Q_mat_ptr[17] * y[3 + 5 * image_width]) * Q_mat_ptr[9] + (Q_mat_ptr[12] * y[4] + Q_mat_ptr[13] * y[4 + image_width] + Q_mat_ptr[14] * y[4 + 2 * image_width] + Q_mat_ptr[15] * y[4 + 3 * image_width] + Q_mat_ptr[16] * y[4 + 4 * image_width] + Q_mat_ptr[17] * y[4 + 5 * image_width]) * Q_mat_ptr[10] + (Q_mat_ptr[12] * y[5] + Q_mat_ptr[13] * y[5 + image_width] + Q_mat_ptr[14] * y[5 + 2 * image_width] + Q_mat_ptr[15] * y[5 + 3 * image_width] + Q_mat_ptr[16] * y[5 + 4 * image_width] + Q_mat_ptr[17] * y[5 + 5 * image_width]) * Q_mat_ptr[11];
				S_mat_ptr[10] = (Q_mat_ptr[12] * y[0] + Q_mat_ptr[13] * y[image_width] + Q_mat_ptr[14] * y[2 * image_width] + Q_mat_ptr[15] * y[3 * image_width] + Q_mat_ptr[16] * y[4 * image_width] + Q_mat_ptr[17] * y[5 * image_width]) * Q_mat_ptr[12] + (Q_mat_ptr[12] * y[1] + Q_mat_ptr[13] * y[1 + image_width] + Q_mat_ptr[14] * y[1 + 2 * image_width] + Q_mat_ptr[15] * y[1 + 3 * image_width] + Q_mat_ptr[16] * y[1 + 4 * image_width] + Q_mat_ptr[17] * y[1 + 5 * image_width]) * Q_mat_ptr[13] + (Q_mat_ptr[12] * y[2] + Q_mat_ptr[13] * y[2 + image_width] + Q_mat_ptr[14] * y[2 + 2 * image_width] + Q_mat_ptr[15] * y[2 + 3 * image_width] + Q_mat_ptr[16] * y[2 + 4 * image_width] + Q_mat_ptr[17] * y[2 + 5 * image_width]) * Q_mat_ptr[14] + (Q_mat_ptr[12] * y[3] + Q_mat_ptr[13] * y[3 + image_width] + Q_mat_ptr[14] * y[3 + 2 * image_width] + Q_mat_ptr[15] * y[3 + 3 * image_width] + Q_mat_ptr[16] * y[3 + 4 * image_width] + Q_mat_ptr[17] * y[3 + 5 * image_width]) * Q_mat_ptr[15] + (Q_mat_ptr[12] * y[4] + Q_mat_ptr[13] * y[4 + image_width] + Q_mat_ptr[14] * y[4 + 2 * image_width] + Q_mat_ptr[15] * y[4 + 3 * image_width] + Q_mat_ptr[16] * y[4 + 4 * image_width] + Q_mat_ptr[17] * y[4 + 5 * image_width]) * Q_mat_ptr[16] + (Q_mat_ptr[12] * y[5] + Q_mat_ptr[13] * y[5 + image_width] + Q_mat_ptr[14] * y[5 + 2 * image_width] + Q_mat_ptr[15] * y[5 + 3 * image_width] + Q_mat_ptr[16] * y[5 + 4 * image_width] + Q_mat_ptr[17] * y[5 + 5 * image_width]) * Q_mat_ptr[17];
				S_mat_ptr[11] = Q_mat_ptr[12] * y[2] + Q_mat_ptr[13] * y[2 + image_width] + Q_mat_ptr[14] * y[2 + 2 * image_width] + Q_mat_ptr[15] * y[2 + 3 * image_width] + Q_mat_ptr[16] * y[2 + 4 * image_width] + Q_mat_ptr[17] * y[2 + 5 * image_width];
				S_mat_ptr[12] = y[2 * image_width] * Q_mat_ptr[0] + y[1 + 2 * image_width] * Q_mat_ptr[1] + Q_mat_ptr[2] * y[2 + 2 * image_width] + y[3 + 2 * image_width] * Q_mat_ptr[3] + y[4 + 2 * image_width] * Q_mat_ptr[4] + y[5 + 2 * image_width] * Q_mat_ptr[5];
				S_mat_ptr[13] = y[2 * image_width] * Q_mat_ptr[6] + y[1 + 2 * image_width] * Q_mat_ptr[7] + Q_mat_ptr[8] * y[2 + 2 * image_width] + y[3 + 2 * image_width] * Q_mat_ptr[9] + y[4 + 2 * image_width] * Q_mat_ptr[10] + y[5 + 2 * image_width] * Q_mat_ptr[11];
				S_mat_ptr[14] = y[2 * image_width] * Q_mat_ptr[12] + y[1 + 2 * image_width] * Q_mat_ptr[13] + Q_mat_ptr[14] * y[2 + 2 * image_width] + y[3 + 2 * image_width] * Q_mat_ptr[15] + y[4 + 2 * image_width] * Q_mat_ptr[16] + y[5 + 2 * image_width] * Q_mat_ptr[17];
				S_mat_ptr[15] = y[2 + 2 * image_width];
				CV_MAT_ELEM(*interpolation_LUT_valid_sign_mat,uchar,index_y - interpolation_area_y_min,index_x - interpolation_area_x_min) =1;		
			}
			//g
			CV_MAT_ELEM(*g_mat,double,m+template_half_height,n+template_half_width) =
				delta_x*delta_x*delta_x * (delta_y*delta_y*delta_y * S_mat_ptr[0] + delta_y*delta_y * S_mat_ptr[4] + delta_y * S_mat_ptr[8] + S_mat_ptr[12] ) +
				delta_x*delta_x * (delta_y*delta_y*delta_y * S_mat_ptr[1] + delta_y*delta_y * S_mat_ptr[5] + delta_y * S_mat_ptr[9] + S_mat_ptr[13] ) +
				delta_x  * (delta_y*delta_y*delta_y * S_mat_ptr[2] + delta_y*delta_y * S_mat_ptr[6] + delta_y * S_mat_ptr[10] + S_mat_ptr[14] ) +
				(delta_y*delta_y*delta_y * S_mat_ptr[3] + delta_y*delta_y * S_mat_ptr[7] + delta_y * S_mat_ptr[11] + S_mat_ptr[15] );

			CV_MAT_ELEM(*gx_mat,double,m+template_half_height,n+template_half_width) =
				3.0 * delta_x*delta_x * (delta_y*delta_y*delta_y * S_mat_ptr[0] + delta_y*delta_y * S_mat_ptr[4] + delta_y * S_mat_ptr[8] + S_mat_ptr[12] ) +
				2.0 * delta_x  * (delta_y*delta_y*delta_y * S_mat_ptr[1] + delta_y*delta_y * S_mat_ptr[5] + delta_y * S_mat_ptr[9] + S_mat_ptr[13] ) +
				(delta_y*delta_y*delta_y * S_mat_ptr[2] + delta_y*delta_y * S_mat_ptr[6] + delta_y * S_mat_ptr[10] + S_mat_ptr[14] );
			CV_MAT_ELEM(*gy_mat,double,m+template_half_height,n+template_half_width) =
				delta_x*delta_x*delta_x * (3.0 * delta_y*delta_y * S_mat_ptr[0] + 2.0 * delta_y * S_mat_ptr[4] + S_mat_ptr[8] ) +
				delta_x*delta_x * (3.0 * delta_y*delta_y * S_mat_ptr[1] +  2.0 * delta_y * S_mat_ptr[5] + S_mat_ptr[9] ) +
				delta_x  * (3.0 * delta_y*delta_y * S_mat_ptr[2] +  2.0 * delta_y * S_mat_ptr[6] + S_mat_ptr[10] ) +
				(3.0 * delta_y*delta_y * S_mat_ptr[3] +  2.0 * delta_y * S_mat_ptr[7] + S_mat_ptr[11] );
		}
	}


	return true;
}


bool _6x6UniformBicubicSplineInterpolationMethod::Value_And_Gradient_One_Affine_Template_64F_20_Parameter(
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
	)
{
	//判断省略
	//大小
	int image_width= image->width;
	int image_height = image->height;
	int template_width = g_mat->cols;
	int template_height = g_mat->cols;
	int lut_width = interpolation_LUT->cols;
	int lut_height = interpolation_LUT->rows;


	int interpolation_area_width = interpolation_area_x_max-interpolation_area_x_min;
	int interpolation_area_height = interpolation_area_y_max-interpolation_area_y_min;

	int template_half_width = (template_width-1)/2;
	int template_half_height = (template_height-1)/2;
	double* Q_mat_ptr = uniform_bicubic_natrue_spline_interpolation_6x6_Q_mat->data.db;

	double U = CV_MAT_ELEM(*template_transform_mat,double,0,0);
	double U_x = CV_MAT_ELEM(*template_transform_mat,double,1,0);
	double U_y = CV_MAT_ELEM(*template_transform_mat,double,2,0);
	double U_xx = CV_MAT_ELEM(*template_transform_mat,double,3,0);	
	double U_yy = CV_MAT_ELEM(*template_transform_mat,double,4,0);
	double U_xy = CV_MAT_ELEM(*template_transform_mat,double,5,0);
	double U_xxx = CV_MAT_ELEM(*template_transform_mat,double,6,0);
	double U_xxy = CV_MAT_ELEM(*template_transform_mat,double,7,0);
	double U_xyy = CV_MAT_ELEM(*template_transform_mat,double,8,0);
	double U_yyy = CV_MAT_ELEM(*template_transform_mat,double,9,0);
	double V = CV_MAT_ELEM(*template_transform_mat,double,10,0);
	double V_x = CV_MAT_ELEM(*template_transform_mat,double,11,0);
	double V_y = CV_MAT_ELEM(*template_transform_mat,double,12,0);
	double V_xx = CV_MAT_ELEM(*template_transform_mat,double,13,0);
	double V_yy = CV_MAT_ELEM(*template_transform_mat,double,14,0);
	double V_xy = CV_MAT_ELEM(*template_transform_mat,double,15,0);
	double V_xxx = CV_MAT_ELEM(*template_transform_mat,double,16,0);
	double V_xxy = CV_MAT_ELEM(*template_transform_mat,double,17,0);
	double V_xyy = CV_MAT_ELEM(*template_transform_mat,double,18,0);
	double V_yyy = CV_MAT_ELEM(*template_transform_mat,double,19,0);


	for (int m =-template_half_height;m<=template_half_height;m++)
	{
		for (int n =-template_half_width;n<=template_half_width;n++)
		{
			double pos_x = 
				//double(x_pos) + 
				U +
				n +
				U_x * n + 
				U_y * m + 				
				0.5 * U_xx * n * n + 
				0.5 * U_yy * m * m + 
				U_xy * n * m +
				1.0/6.0 * U_xxx * n * n * n +
				0.5 * U_xxy * n * n * m +
				0.5 * U_xyy * n * m * m +
				1.0/6.0 * U_yyy * m * m * m;
			double pos_y = 
				//double(y_pos) +
				V + 
				m + 
				V_x * n + 
				V_y * m + 				
				0.5 * V_xx * n * n + 
				0.5 * V_yy * m * m + 
				V_xy * n * m +
				1.0/6.0 * V_xxx * n * n * n +
				0.5 * V_xxy * n * n * m +
				0.5 * V_xyy * n * m * m +
				1.0/6.0 * V_yyy * m * m * m;


			//x' = x0 + u +dx + dx*ux+dy*uy,  x = x0+u
			//double pos_x =reference_center_point_x + n + n* reference_ux + m * reference_vy;
			//double pos_y =reference_center_point_y + m + n* reference_vx + m * reference_vy;
			//下面方法与方形模板的相同

			int index_x = cvFloor(pos_x);
			int index_y = cvFloor(pos_y);
			if (index_y - interpolation_area_y_min <0 || index_y - interpolation_area_y_min >= interpolation_LUT_valid_sign_mat->rows-1 ||
				index_x - interpolation_area_x_min <0 || index_x - interpolation_area_x_min >= interpolation_LUT_valid_sign_mat->cols-1)
			{
				return false;
			}

			double delta_x = pos_x - index_x;
			double delta_y = pos_y - index_y;

			uchar sign = CV_MAT_ELEM(*interpolation_LUT_valid_sign_mat,uchar,index_y - interpolation_area_y_min,index_x - interpolation_area_x_min);
			float* S_mat_ptr = interpolation_LUT->data.fl+ ((index_y - interpolation_area_y_min) * (interpolation_area_x_max-interpolation_area_x_min)+index_x - interpolation_area_x_min) * Interpolation_LUT_Element_Size;
			if (sign!=1)
			{							
				double* y = image->data.db + (index_y-2)*image_width+index_x-2;
				S_mat_ptr[0] = (Q_mat_ptr[0] * y[0] + Q_mat_ptr[1] * y[image_width] + Q_mat_ptr[2] * y[2 * image_width] + Q_mat_ptr[3] * y[3 * image_width] + Q_mat_ptr[4] * y[4 * image_width] + Q_mat_ptr[5] * y[5 * image_width]) * Q_mat_ptr[0] + (Q_mat_ptr[0] * y[1] + Q_mat_ptr[1] * y[1 + image_width] + Q_mat_ptr[2] * y[1 + 2 * image_width] + Q_mat_ptr[3] * y[1 + 3 * image_width] + Q_mat_ptr[4] * y[1 + 4 * image_width] + Q_mat_ptr[5] * y[1 + 5 * image_width]) * Q_mat_ptr[1] + (Q_mat_ptr[0] * y[2] + Q_mat_ptr[1] * y[2 + image_width] + Q_mat_ptr[2] * y[2 + 2 * image_width] + Q_mat_ptr[3] * y[2 + 3 * image_width] + Q_mat_ptr[4] * y[2 + 4 * image_width] + Q_mat_ptr[5] * y[2 + 5 * image_width]) * Q_mat_ptr[2] + (Q_mat_ptr[0] * y[3] + Q_mat_ptr[1] * y[3 + image_width] + Q_mat_ptr[2] * y[3 + 2 * image_width] + Q_mat_ptr[3] * y[3 + 3 * image_width] + Q_mat_ptr[4] * y[3 + 4 * image_width] + Q_mat_ptr[5] * y[3 + 5 * image_width]) * Q_mat_ptr[3] + (Q_mat_ptr[0] * y[4] + Q_mat_ptr[1] * y[4 + image_width] + Q_mat_ptr[2] * y[4 + 2 * image_width] + Q_mat_ptr[3] * y[4 + 3 * image_width] + Q_mat_ptr[4] * y[4 + 4 * image_width] + Q_mat_ptr[5] * y[4 + 5 * image_width]) * Q_mat_ptr[4] + (Q_mat_ptr[0] * y[5] + Q_mat_ptr[1] * y[5 + image_width] + Q_mat_ptr[2] * y[5 + 2 * image_width] + Q_mat_ptr[3] * y[5 + 3 * image_width] + Q_mat_ptr[4] * y[5 + 4 * image_width] + Q_mat_ptr[5] * y[5 + 5 * image_width]) * Q_mat_ptr[5];
				S_mat_ptr[1] = (Q_mat_ptr[0] * y[0] + Q_mat_ptr[1] * y[image_width] + Q_mat_ptr[2] * y[2 * image_width] + Q_mat_ptr[3] * y[3 * image_width] + Q_mat_ptr[4] * y[4 * image_width] + Q_mat_ptr[5] * y[5 * image_width]) * Q_mat_ptr[6] + (Q_mat_ptr[0] * y[1] + Q_mat_ptr[1] * y[1 + image_width] + Q_mat_ptr[2] * y[1 + 2 * image_width] + Q_mat_ptr[3] * y[1 + 3 * image_width] + Q_mat_ptr[4] * y[1 + 4 * image_width] + Q_mat_ptr[5] * y[1 + 5 * image_width]) * Q_mat_ptr[7] + (Q_mat_ptr[0] * y[2] + Q_mat_ptr[1] * y[2 + image_width] + Q_mat_ptr[2] * y[2 + 2 * image_width] + Q_mat_ptr[3] * y[2 + 3 * image_width] + Q_mat_ptr[4] * y[2 + 4 * image_width] + Q_mat_ptr[5] * y[2 + 5 * image_width]) * Q_mat_ptr[8] + (Q_mat_ptr[0] * y[3] + Q_mat_ptr[1] * y[3 + image_width] + Q_mat_ptr[2] * y[3 + 2 * image_width] + Q_mat_ptr[3] * y[3 + 3 * image_width] + Q_mat_ptr[4] * y[3 + 4 * image_width] + Q_mat_ptr[5] * y[3 + 5 * image_width]) * Q_mat_ptr[9] + (Q_mat_ptr[0] * y[4] + Q_mat_ptr[1] * y[4 + image_width] + Q_mat_ptr[2] * y[4 + 2 * image_width] + Q_mat_ptr[3] * y[4 + 3 * image_width] + Q_mat_ptr[4] * y[4 + 4 * image_width] + Q_mat_ptr[5] * y[4 + 5 * image_width]) * Q_mat_ptr[10] + (Q_mat_ptr[0] * y[5] + Q_mat_ptr[1] * y[5 + image_width] + Q_mat_ptr[2] * y[5 + 2 * image_width] + Q_mat_ptr[3] * y[5 + 3 * image_width] + Q_mat_ptr[4] * y[5 + 4 * image_width] + Q_mat_ptr[5] * y[5 + 5 * image_width]) * Q_mat_ptr[11];
				S_mat_ptr[2] = (Q_mat_ptr[0] * y[0] + Q_mat_ptr[1] * y[image_width] + Q_mat_ptr[2] * y[2 * image_width] + Q_mat_ptr[3] * y[3 * image_width] + Q_mat_ptr[4] * y[4 * image_width] + Q_mat_ptr[5] * y[5 * image_width]) * Q_mat_ptr[12] + (Q_mat_ptr[0] * y[1] + Q_mat_ptr[1] * y[1 + image_width] + Q_mat_ptr[2] * y[1 + 2 * image_width] + Q_mat_ptr[3] * y[1 + 3 * image_width] + Q_mat_ptr[4] * y[1 + 4 * image_width] + Q_mat_ptr[5] * y[1 + 5 * image_width]) * Q_mat_ptr[13] + (Q_mat_ptr[0] * y[2] + Q_mat_ptr[1] * y[2 + image_width] + Q_mat_ptr[2] * y[2 + 2 * image_width] + Q_mat_ptr[3] * y[2 + 3 * image_width] + Q_mat_ptr[4] * y[2 + 4 * image_width] + Q_mat_ptr[5] * y[2 + 5 * image_width]) * Q_mat_ptr[14] + (Q_mat_ptr[0] * y[3] + Q_mat_ptr[1] * y[3 + image_width] + Q_mat_ptr[2] * y[3 + 2 * image_width] + Q_mat_ptr[3] * y[3 + 3 * image_width] + Q_mat_ptr[4] * y[3 + 4 * image_width] + Q_mat_ptr[5] * y[3 + 5 * image_width]) * Q_mat_ptr[15] + (Q_mat_ptr[0] * y[4] + Q_mat_ptr[1] * y[4 + image_width] + Q_mat_ptr[2] * y[4 + 2 * image_width] + Q_mat_ptr[3] * y[4 + 3 * image_width] + Q_mat_ptr[4] * y[4 + 4 * image_width] + Q_mat_ptr[5] * y[4 + 5 * image_width]) * Q_mat_ptr[16] + (Q_mat_ptr[0] * y[5] + Q_mat_ptr[1] * y[5 + image_width] + Q_mat_ptr[2] * y[5 + 2 * image_width] + Q_mat_ptr[3] * y[5 + 3 * image_width] + Q_mat_ptr[4] * y[5 + 4 * image_width] + Q_mat_ptr[5] * y[5 + 5 * image_width]) * Q_mat_ptr[17];
				S_mat_ptr[3] = Q_mat_ptr[0] * y[2] + Q_mat_ptr[1] * y[2 + image_width] + Q_mat_ptr[2] * y[2 + 2 * image_width] + Q_mat_ptr[3] * y[2 + 3 * image_width] + Q_mat_ptr[4] * y[2 + 4 * image_width] + Q_mat_ptr[5] * y[2 + 5 * image_width];
				S_mat_ptr[4] = (Q_mat_ptr[6] * y[0] + Q_mat_ptr[7] * y[image_width] + Q_mat_ptr[8] * y[2 * image_width] + Q_mat_ptr[9] * y[3 * image_width] + Q_mat_ptr[10] * y[4 * image_width] + Q_mat_ptr[11] * y[5 * image_width]) * Q_mat_ptr[0] + (Q_mat_ptr[6] * y[1] + Q_mat_ptr[7] * y[1 + image_width] + Q_mat_ptr[8] * y[1 + 2 * image_width] + Q_mat_ptr[9] * y[1 + 3 * image_width] + Q_mat_ptr[10] * y[1 + 4 * image_width] + Q_mat_ptr[11] * y[1 + 5 * image_width]) * Q_mat_ptr[1] + (Q_mat_ptr[6] * y[2] + Q_mat_ptr[7] * y[2 + image_width] + Q_mat_ptr[8] * y[2 + 2 * image_width] + Q_mat_ptr[9] * y[2 + 3 * image_width] + Q_mat_ptr[10] * y[2 + 4 * image_width] + Q_mat_ptr[11] * y[2 + 5 * image_width]) * Q_mat_ptr[2] + (Q_mat_ptr[6] * y[3] + Q_mat_ptr[7] * y[3 + image_width] + Q_mat_ptr[8] * y[3 + 2 * image_width] + Q_mat_ptr[9] * y[3 + 3 * image_width] + Q_mat_ptr[10] * y[3 + 4 * image_width] + Q_mat_ptr[11] * y[3 + 5 * image_width]) * Q_mat_ptr[3] + (Q_mat_ptr[6] * y[4] + Q_mat_ptr[7] * y[4 + image_width] + Q_mat_ptr[8] * y[4 + 2 * image_width] + Q_mat_ptr[9] * y[4 + 3 * image_width] + Q_mat_ptr[10] * y[4 + 4 * image_width] + Q_mat_ptr[11] * y[4 + 5 * image_width]) * Q_mat_ptr[4] + (Q_mat_ptr[6] * y[5] + Q_mat_ptr[7] * y[5 + image_width] + Q_mat_ptr[8] * y[5 + 2 * image_width] + Q_mat_ptr[9] * y[5 + 3 * image_width] + Q_mat_ptr[10] * y[5 + 4 * image_width] + Q_mat_ptr[11] * y[5 + 5 * image_width]) * Q_mat_ptr[5];
				S_mat_ptr[5] = (Q_mat_ptr[6] * y[0] + Q_mat_ptr[7] * y[image_width] + Q_mat_ptr[8] * y[2 * image_width] + Q_mat_ptr[9] * y[3 * image_width] + Q_mat_ptr[10] * y[4 * image_width] + Q_mat_ptr[11] * y[5 * image_width]) * Q_mat_ptr[6] + (Q_mat_ptr[6] * y[1] + Q_mat_ptr[7] * y[1 + image_width] + Q_mat_ptr[8] * y[1 + 2 * image_width] + Q_mat_ptr[9] * y[1 + 3 * image_width] + Q_mat_ptr[10] * y[1 + 4 * image_width] + Q_mat_ptr[11] * y[1 + 5 * image_width]) * Q_mat_ptr[7] + (Q_mat_ptr[6] * y[2] + Q_mat_ptr[7] * y[2 + image_width] + Q_mat_ptr[8] * y[2 + 2 * image_width] + Q_mat_ptr[9] * y[2 + 3 * image_width] + Q_mat_ptr[10] * y[2 + 4 * image_width] + Q_mat_ptr[11] * y[2 + 5 * image_width]) * Q_mat_ptr[8] + (Q_mat_ptr[6] * y[3] + Q_mat_ptr[7] * y[3 + image_width] + Q_mat_ptr[8] * y[3 + 2 * image_width] + Q_mat_ptr[9] * y[3 + 3 * image_width] + Q_mat_ptr[10] * y[3 + 4 * image_width] + Q_mat_ptr[11] * y[3 + 5 * image_width]) * Q_mat_ptr[9] + (Q_mat_ptr[6] * y[4] + Q_mat_ptr[7] * y[4 + image_width] + Q_mat_ptr[8] * y[4 + 2 * image_width] + Q_mat_ptr[9] * y[4 + 3 * image_width] + Q_mat_ptr[10] * y[4 + 4 * image_width] + Q_mat_ptr[11] * y[4 + 5 * image_width]) * Q_mat_ptr[10] + (Q_mat_ptr[6] * y[5] + Q_mat_ptr[7] * y[5 + image_width] + Q_mat_ptr[8] * y[5 + 2 * image_width] + Q_mat_ptr[9] * y[5 + 3 * image_width] + Q_mat_ptr[10] * y[5 + 4 * image_width] + Q_mat_ptr[11] * y[5 + 5 * image_width]) * Q_mat_ptr[11];
				S_mat_ptr[6] = (Q_mat_ptr[6] * y[0] + Q_mat_ptr[7] * y[image_width] + Q_mat_ptr[8] * y[2 * image_width] + Q_mat_ptr[9] * y[3 * image_width] + Q_mat_ptr[10] * y[4 * image_width] + Q_mat_ptr[11] * y[5 * image_width]) * Q_mat_ptr[12] + (Q_mat_ptr[6] * y[1] + Q_mat_ptr[7] * y[1 + image_width] + Q_mat_ptr[8] * y[1 + 2 * image_width] + Q_mat_ptr[9] * y[1 + 3 * image_width] + Q_mat_ptr[10] * y[1 + 4 * image_width] + Q_mat_ptr[11] * y[1 + 5 * image_width]) * Q_mat_ptr[13] + (Q_mat_ptr[6] * y[2] + Q_mat_ptr[7] * y[2 + image_width] + Q_mat_ptr[8] * y[2 + 2 * image_width] + Q_mat_ptr[9] * y[2 + 3 * image_width] + Q_mat_ptr[10] * y[2 + 4 * image_width] + Q_mat_ptr[11] * y[2 + 5 * image_width]) * Q_mat_ptr[14] + (Q_mat_ptr[6] * y[3] + Q_mat_ptr[7] * y[3 + image_width] + Q_mat_ptr[8] * y[3 + 2 * image_width] + Q_mat_ptr[9] * y[3 + 3 * image_width] + Q_mat_ptr[10] * y[3 + 4 * image_width] + Q_mat_ptr[11] * y[3 + 5 * image_width]) * Q_mat_ptr[15] + (Q_mat_ptr[6] * y[4] + Q_mat_ptr[7] * y[4 + image_width] + Q_mat_ptr[8] * y[4 + 2 * image_width] + Q_mat_ptr[9] * y[4 + 3 * image_width] + Q_mat_ptr[10] * y[4 + 4 * image_width] + Q_mat_ptr[11] * y[4 + 5 * image_width]) * Q_mat_ptr[16] + (Q_mat_ptr[6] * y[5] + Q_mat_ptr[7] * y[5 + image_width] + Q_mat_ptr[8] * y[5 + 2 * image_width] + Q_mat_ptr[9] * y[5 + 3 * image_width] + Q_mat_ptr[10] * y[5 + 4 * image_width] + Q_mat_ptr[11] * y[5 + 5 * image_width]) * Q_mat_ptr[17];
				S_mat_ptr[7] = Q_mat_ptr[6] * y[2] + Q_mat_ptr[7] * y[2 + image_width] + Q_mat_ptr[8] * y[2 + 2 * image_width] + Q_mat_ptr[9] * y[2 + 3 * image_width] + Q_mat_ptr[10] * y[2 + 4 * image_width] + Q_mat_ptr[11] * y[2 + 5 * image_width];
				S_mat_ptr[8] = (Q_mat_ptr[12] * y[0] + Q_mat_ptr[13] * y[image_width] + Q_mat_ptr[14] * y[2 * image_width] + Q_mat_ptr[15] * y[3 * image_width] + Q_mat_ptr[16] * y[4 * image_width] + Q_mat_ptr[17] * y[5 * image_width]) * Q_mat_ptr[0] + (Q_mat_ptr[12] * y[1] + Q_mat_ptr[13] * y[1 + image_width] + Q_mat_ptr[14] * y[1 + 2 * image_width] + Q_mat_ptr[15] * y[1 + 3 * image_width] + Q_mat_ptr[16] * y[1 + 4 * image_width] + Q_mat_ptr[17] * y[1 + 5 * image_width]) * Q_mat_ptr[1] + (Q_mat_ptr[12] * y[2] + Q_mat_ptr[13] * y[2 + image_width] + Q_mat_ptr[14] * y[2 + 2 * image_width] + Q_mat_ptr[15] * y[2 + 3 * image_width] + Q_mat_ptr[16] * y[2 + 4 * image_width] + Q_mat_ptr[17] * y[2 + 5 * image_width]) * Q_mat_ptr[2] + (Q_mat_ptr[12] * y[3] + Q_mat_ptr[13] * y[3 + image_width] + Q_mat_ptr[14] * y[3 + 2 * image_width] + Q_mat_ptr[15] * y[3 + 3 * image_width] + Q_mat_ptr[16] * y[3 + 4 * image_width] + Q_mat_ptr[17] * y[3 + 5 * image_width]) * Q_mat_ptr[3] + (Q_mat_ptr[12] * y[4] + Q_mat_ptr[13] * y[4 + image_width] + Q_mat_ptr[14] * y[4 + 2 * image_width] + Q_mat_ptr[15] * y[4 + 3 * image_width] + Q_mat_ptr[16] * y[4 + 4 * image_width] + Q_mat_ptr[17] * y[4 + 5 * image_width]) * Q_mat_ptr[4] + (Q_mat_ptr[12] * y[5] + Q_mat_ptr[13] * y[5 + image_width] + Q_mat_ptr[14] * y[5 + 2 * image_width] + Q_mat_ptr[15] * y[5 + 3 * image_width] + Q_mat_ptr[16] * y[5 + 4 * image_width] + Q_mat_ptr[17] * y[5 + 5 * image_width]) * Q_mat_ptr[5];
				S_mat_ptr[9] = (Q_mat_ptr[12] * y[0] + Q_mat_ptr[13] * y[image_width] + Q_mat_ptr[14] * y[2 * image_width] + Q_mat_ptr[15] * y[3 * image_width] + Q_mat_ptr[16] * y[4 * image_width] + Q_mat_ptr[17] * y[5 * image_width]) * Q_mat_ptr[6] + (Q_mat_ptr[12] * y[1] + Q_mat_ptr[13] * y[1 + image_width] + Q_mat_ptr[14] * y[1 + 2 * image_width] + Q_mat_ptr[15] * y[1 + 3 * image_width] + Q_mat_ptr[16] * y[1 + 4 * image_width] + Q_mat_ptr[17] * y[1 + 5 * image_width]) * Q_mat_ptr[7] + (Q_mat_ptr[12] * y[2] + Q_mat_ptr[13] * y[2 + image_width] + Q_mat_ptr[14] * y[2 + 2 * image_width] + Q_mat_ptr[15] * y[2 + 3 * image_width] + Q_mat_ptr[16] * y[2 + 4 * image_width] + Q_mat_ptr[17] * y[2 + 5 * image_width]) * Q_mat_ptr[8] + (Q_mat_ptr[12] * y[3] + Q_mat_ptr[13] * y[3 + image_width] + Q_mat_ptr[14] * y[3 + 2 * image_width] + Q_mat_ptr[15] * y[3 + 3 * image_width] + Q_mat_ptr[16] * y[3 + 4 * image_width] + Q_mat_ptr[17] * y[3 + 5 * image_width]) * Q_mat_ptr[9] + (Q_mat_ptr[12] * y[4] + Q_mat_ptr[13] * y[4 + image_width] + Q_mat_ptr[14] * y[4 + 2 * image_width] + Q_mat_ptr[15] * y[4 + 3 * image_width] + Q_mat_ptr[16] * y[4 + 4 * image_width] + Q_mat_ptr[17] * y[4 + 5 * image_width]) * Q_mat_ptr[10] + (Q_mat_ptr[12] * y[5] + Q_mat_ptr[13] * y[5 + image_width] + Q_mat_ptr[14] * y[5 + 2 * image_width] + Q_mat_ptr[15] * y[5 + 3 * image_width] + Q_mat_ptr[16] * y[5 + 4 * image_width] + Q_mat_ptr[17] * y[5 + 5 * image_width]) * Q_mat_ptr[11];
				S_mat_ptr[10] = (Q_mat_ptr[12] * y[0] + Q_mat_ptr[13] * y[image_width] + Q_mat_ptr[14] * y[2 * image_width] + Q_mat_ptr[15] * y[3 * image_width] + Q_mat_ptr[16] * y[4 * image_width] + Q_mat_ptr[17] * y[5 * image_width]) * Q_mat_ptr[12] + (Q_mat_ptr[12] * y[1] + Q_mat_ptr[13] * y[1 + image_width] + Q_mat_ptr[14] * y[1 + 2 * image_width] + Q_mat_ptr[15] * y[1 + 3 * image_width] + Q_mat_ptr[16] * y[1 + 4 * image_width] + Q_mat_ptr[17] * y[1 + 5 * image_width]) * Q_mat_ptr[13] + (Q_mat_ptr[12] * y[2] + Q_mat_ptr[13] * y[2 + image_width] + Q_mat_ptr[14] * y[2 + 2 * image_width] + Q_mat_ptr[15] * y[2 + 3 * image_width] + Q_mat_ptr[16] * y[2 + 4 * image_width] + Q_mat_ptr[17] * y[2 + 5 * image_width]) * Q_mat_ptr[14] + (Q_mat_ptr[12] * y[3] + Q_mat_ptr[13] * y[3 + image_width] + Q_mat_ptr[14] * y[3 + 2 * image_width] + Q_mat_ptr[15] * y[3 + 3 * image_width] + Q_mat_ptr[16] * y[3 + 4 * image_width] + Q_mat_ptr[17] * y[3 + 5 * image_width]) * Q_mat_ptr[15] + (Q_mat_ptr[12] * y[4] + Q_mat_ptr[13] * y[4 + image_width] + Q_mat_ptr[14] * y[4 + 2 * image_width] + Q_mat_ptr[15] * y[4 + 3 * image_width] + Q_mat_ptr[16] * y[4 + 4 * image_width] + Q_mat_ptr[17] * y[4 + 5 * image_width]) * Q_mat_ptr[16] + (Q_mat_ptr[12] * y[5] + Q_mat_ptr[13] * y[5 + image_width] + Q_mat_ptr[14] * y[5 + 2 * image_width] + Q_mat_ptr[15] * y[5 + 3 * image_width] + Q_mat_ptr[16] * y[5 + 4 * image_width] + Q_mat_ptr[17] * y[5 + 5 * image_width]) * Q_mat_ptr[17];
				S_mat_ptr[11] = Q_mat_ptr[12] * y[2] + Q_mat_ptr[13] * y[2 + image_width] + Q_mat_ptr[14] * y[2 + 2 * image_width] + Q_mat_ptr[15] * y[2 + 3 * image_width] + Q_mat_ptr[16] * y[2 + 4 * image_width] + Q_mat_ptr[17] * y[2 + 5 * image_width];
				S_mat_ptr[12] = y[2 * image_width] * Q_mat_ptr[0] + y[1 + 2 * image_width] * Q_mat_ptr[1] + Q_mat_ptr[2] * y[2 + 2 * image_width] + y[3 + 2 * image_width] * Q_mat_ptr[3] + y[4 + 2 * image_width] * Q_mat_ptr[4] + y[5 + 2 * image_width] * Q_mat_ptr[5];
				S_mat_ptr[13] = y[2 * image_width] * Q_mat_ptr[6] + y[1 + 2 * image_width] * Q_mat_ptr[7] + Q_mat_ptr[8] * y[2 + 2 * image_width] + y[3 + 2 * image_width] * Q_mat_ptr[9] + y[4 + 2 * image_width] * Q_mat_ptr[10] + y[5 + 2 * image_width] * Q_mat_ptr[11];
				S_mat_ptr[14] = y[2 * image_width] * Q_mat_ptr[12] + y[1 + 2 * image_width] * Q_mat_ptr[13] + Q_mat_ptr[14] * y[2 + 2 * image_width] + y[3 + 2 * image_width] * Q_mat_ptr[15] + y[4 + 2 * image_width] * Q_mat_ptr[16] + y[5 + 2 * image_width] * Q_mat_ptr[17];
				S_mat_ptr[15] = y[2 + 2 * image_width];
				CV_MAT_ELEM(*interpolation_LUT_valid_sign_mat,uchar,index_y - interpolation_area_y_min,index_x - interpolation_area_x_min) =1;		
			}
			//g
			CV_MAT_ELEM(*g_mat,double,m+template_half_height,n+template_half_width) =
				delta_x*delta_x*delta_x * (delta_y*delta_y*delta_y * S_mat_ptr[0] + delta_y*delta_y * S_mat_ptr[4] + delta_y * S_mat_ptr[8] + S_mat_ptr[12] ) +
				delta_x*delta_x * (delta_y*delta_y*delta_y * S_mat_ptr[1] + delta_y*delta_y * S_mat_ptr[5] + delta_y * S_mat_ptr[9] + S_mat_ptr[13] ) +
				delta_x  * (delta_y*delta_y*delta_y * S_mat_ptr[2] + delta_y*delta_y * S_mat_ptr[6] + delta_y * S_mat_ptr[10] + S_mat_ptr[14] ) +
				(delta_y*delta_y*delta_y * S_mat_ptr[3] + delta_y*delta_y * S_mat_ptr[7] + delta_y * S_mat_ptr[11] + S_mat_ptr[15] );

			if (gx_mat !=NULL)
			{
				CV_MAT_ELEM(*gx_mat,double,m+template_half_height,n+template_half_width) =
					3.0 * delta_x*delta_x * (delta_y*delta_y*delta_y * S_mat_ptr[0] + delta_y*delta_y * S_mat_ptr[4] + delta_y * S_mat_ptr[8] + S_mat_ptr[12] ) +
					2.0 * delta_x  * (delta_y*delta_y*delta_y * S_mat_ptr[1] + delta_y*delta_y * S_mat_ptr[5] + delta_y * S_mat_ptr[9] + S_mat_ptr[13] ) +
					(delta_y*delta_y*delta_y * S_mat_ptr[2] + delta_y*delta_y * S_mat_ptr[6] + delta_y * S_mat_ptr[10] + S_mat_ptr[14] );
			}
			if (gy_mat !=NULL)
			{
				CV_MAT_ELEM(*gy_mat,double,m+template_half_height,n+template_half_width) =
					delta_x*delta_x*delta_x * (3.0 * delta_y*delta_y * S_mat_ptr[0] + 2.0 * delta_y * S_mat_ptr[4] + S_mat_ptr[8] ) +
					delta_x*delta_x * (3.0 * delta_y*delta_y * S_mat_ptr[1] +  2.0 * delta_y * S_mat_ptr[5] + S_mat_ptr[9] ) +
					delta_x  * (3.0 * delta_y*delta_y * S_mat_ptr[2] +  2.0 * delta_y * S_mat_ptr[6] + S_mat_ptr[10] ) +
					(3.0 * delta_y*delta_y * S_mat_ptr[3] +  2.0 * delta_y * S_mat_ptr[7] + S_mat_ptr[11] );
			}
		}
	}


	return true;

}
//////////////////////////////////////////////////////////////////////////