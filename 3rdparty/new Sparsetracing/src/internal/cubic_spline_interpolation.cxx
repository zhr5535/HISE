#include "cubic_spline_interpolation.h"

using UniCubicSplineInterp = unicubic_spline_interp_t;

bool UniCubicSplineInterp::Two_Dimension_Interpolation_One_Point(
	CvMat* image, 
	double inter_point_pos_x, double inter_point_pos_y, 
	CvMat* Q_mat, 
	double* g, double* gx, double* gy 
	)
{
	//判断大小
	const int image_width= image->width, image_height = image->height;

	if (Q_mat->cols!=8||Q_mat->rows!=4)
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

	if (CV_MAT_TYPE(Q_mat->type) != CV_64F)
	{
		return false;
	}

	if (inter_point_pos_x<3 || inter_point_pos_x>=image_width-4 ||
		inter_point_pos_y<3 || inter_point_pos_y>=image_height-4 )
	{
		return false;
	}
	int left = cvFloor(inter_point_pos_x);
	int top = cvFloor(inter_point_pos_y);
	double delta_x = inter_point_pos_x - (double)left;
	double delta_y = inter_point_pos_y - (double)top;
	double delta_x3 = pow(delta_x,3.0);
	double delta_x2 = pow(delta_x,2.0);
	double delta_y3=  pow(delta_y,3.0);
	double delta_y2 = pow(delta_y,2.0);



	int index_i_min = top-3;
	int index_j_min = left-3;
	int index_i_max = top+4;
	int index_j_max = left+4;
	CvMat* Q_mat_4X8_Transpose = cvCreateMat(8,4,CV_64F);
	cvTranspose(Q_mat,Q_mat_4X8_Transpose);
	CvMat* Y_mat = cvCreateMat(8,8,CV_64F);
	for (int i=index_i_min;i<=index_i_max;i++)
	{
		for (int j=index_j_min;j<=index_j_max;j++)
		{
			int s = i-index_i_min;
			int t = j-index_j_min;
			double val;
			if (image_type==1)
			{
				val =(double) CV_MAT_ELEM(*image,uchar,i,j);
			} 
			else
			{
				val = CV_MAT_ELEM(*image,double,i,j);
			}
			CV_MAT_ELEM(*Y_mat,double,s,t) = val;
		}
	}

	CvMat* delta_y_vector = cvCreateMat(1,4,CV_64F);
	double* delta_y_vector_ptr = delta_y_vector->data.db;
	delta_y_vector_ptr[0] = delta_y3;
	delta_y_vector_ptr[1] = delta_y2;
	delta_y_vector_ptr[2] = delta_y;
	delta_y_vector_ptr[3] = 1;

	CvMat* delta_dy_vector = cvCreateMat(1,4,CV_64F);
	double* delta_dy_vector_ptr = delta_dy_vector->data.db;
	delta_dy_vector_ptr[0] = 3.0*delta_y2;
	delta_dy_vector_ptr[1] = 2.0*delta_y;
	delta_dy_vector_ptr[2] = 1.0;
	delta_dy_vector_ptr[3] = 0;



	CvMat* delta_x_vector = cvCreateMat(4,1,CV_64F);
	double* delta_x_vector_ptr = delta_x_vector->data.db;
	delta_x_vector_ptr[0] = delta_x3;
	delta_x_vector_ptr[1] = delta_x2;
	delta_x_vector_ptr[2] = delta_x;
	delta_x_vector_ptr[3] = 1.0;

	CvMat* delta_dx_vector = cvCreateMat(4,1,CV_64F);
	double* delta_dx_vector_ptr=  delta_dx_vector->data.db;
	delta_dx_vector_ptr[0] = 3.0*delta_x2;
	delta_dx_vector_ptr[1] = 2.0*delta_x;
	delta_dx_vector_ptr[2] = 1.0;
	delta_dx_vector_ptr[3] = 0;



	//S= Q* Y* QT
	CvMat* Q_mat_4X8_Y_mat_cache_1 = cvCreateMat(4,8,CV_64F);
	cvMatMul(Q_mat,Y_mat,Q_mat_4X8_Y_mat_cache_1);
	CvMat* S_mat = cvCreateMat(4,4,CV_64F);
	cvMatMul(Q_mat_4X8_Y_mat_cache_1,Q_mat_4X8_Transpose,S_mat);

	//g
	CvMat* result_g=  cvCreateMat(1,1,CV_64F);
	CvMat* delta_y_vector_S_mat_cache_2= cvCreateMat(1,4,CV_64F);
	cvMatMul(delta_y_vector,S_mat,delta_y_vector_S_mat_cache_2);
	cvMatMul(delta_y_vector_S_mat_cache_2,delta_x_vector,result_g);


	//gx
	CvMat* result_gx = cvCreateMat(1,1,CV_64F);
	cvMatMul(delta_y_vector_S_mat_cache_2,delta_dx_vector,result_gx);


	//gy
	CvMat* result_gy = cvCreateMat(1,1,CV_64F);
	CvMat* delta_dy_vector_S_mat_cache_3 = cvCreateMat(1,4,CV_64F);
	cvMatMul(delta_dy_vector,S_mat,delta_dy_vector_S_mat_cache_3);
	cvMatMul(delta_dy_vector_S_mat_cache_3,delta_x_vector,result_gy);


	if (g!=NULL)
	{
		*g = CV_MAT_ELEM(*result_g,double,0,0);
	}
	if (gx!=NULL)
	{
		*gx = CV_MAT_ELEM(*result_gx,double,0,0);
	}
	if (gy!=NULL)
	{
		*gy = CV_MAT_ELEM(*result_gy,double,0,0);
	}

	cvReleaseMat(&Q_mat_4X8_Transpose);
	cvReleaseMat(&Y_mat);
	cvReleaseMat(&delta_y_vector);
	cvReleaseMat(&delta_dy_vector);
	cvReleaseMat(&delta_x_vector);
	cvReleaseMat(&delta_dx_vector);
	cvReleaseMat(&Q_mat_4X8_Y_mat_cache_1);
	cvReleaseMat(&S_mat);
	cvReleaseMat(&result_g);
	cvReleaseMat(&delta_y_vector_S_mat_cache_2);
	cvReleaseMat(&result_gx);
	cvReleaseMat(&result_gy);
	cvReleaseMat(&delta_dy_vector_S_mat_cache_3);

	return true;
}

bool UniformCubicNatureSplineInterpolationMethod::Two_Dimension_Interpolation_Many_Points(
	CvMat* image, 
	CvMat* pos_x_mat, 
	CvMat* pos_y_mat, 
	CvMat* Q_mat, 
	CvMat* g_mat, 
	CvMat* gx_mat, 
	CvMat* gy_mat 
	)
{

	//判断
	//大小
	int image_width= image->width;
	int image_height = image->height;

	if (Q_mat->cols!=8||Q_mat->rows!=4)
	{
		return false;
	}
	int mat_width = pos_x_mat->cols;
	int mat_height = pos_x_mat->cols;

	if (pos_y_mat->cols != mat_width ||
		pos_y_mat->rows != mat_height ||
		g_mat->cols != mat_width ||
		g_mat->rows != mat_height ||
		gx_mat->cols != mat_width ||
		gx_mat->rows != mat_height ||
		gy_mat->cols != mat_width ||
		gy_mat->rows != mat_height 
		)
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

	if (CV_MAT_TYPE(Q_mat->type) != CV_64F)
	{
		return false;
	}
	if (CV_MAT_TYPE(pos_x_mat->type) != CV_64F ||
		CV_MAT_TYPE(pos_y_mat->type) != CV_64F ||
		CV_MAT_TYPE(g_mat->type) != CV_64F ||
		CV_MAT_TYPE(gx_mat->type) != CV_64F ||
		CV_MAT_TYPE(gy_mat->type) != CV_64F 
		)
	{
		return false;
	}


	double pos_x_min,pos_y_min,pos_x_max,pos_y_max;
	cvMinMaxLoc(pos_x_mat,&pos_x_min,&pos_x_max);
	cvMinMaxLoc(pos_y_mat,&pos_y_min,&pos_y_max);

	int x_min = cvFloor(pos_x_min)-3;
	int x_max = cvFloor(pos_x_max)+4;
	int y_min = cvFloor(pos_y_min)-3;
	int y_max = cvFloor(pos_y_max)+4;


	if (x_min<0 || x_max>=image_width||
		y_min<0 || x_max>=image_height)
	{
		return false;
	}


	for (int m =0;m<mat_height;m++)
	{
		for (int n =0;n<mat_width;n++)
		{
			double pos_x = CV_MAT_ELEM(*pos_x_mat,double,m,n);
			double pos_y = CV_MAT_ELEM(*pos_y_mat,double,m,n);

			int left = cvFloor(pos_x);
			int top = cvFloor(pos_y);
			double delta_x = pos_x - (double)left;
			double delta_y = pos_y - (double)top;
			double delta_x3 = pow(delta_x,3.0);
			double delta_x2 = pow(delta_x,2.0);
			double delta_y3=  pow(delta_y,3.0);
			double delta_y2 = pow(delta_y,2.0);

			int index_i_min = top-3;
			int index_j_min = left-3;
			int index_i_max = top+4;
			int index_j_max = left+4;

			CvMat* Q_mat_4X8_Transpose = cvCreateMat(8,4,CV_64F);
			cvTranspose(Q_mat,Q_mat_4X8_Transpose);
			CvMat* Y_mat = cvCreateMat(8,8,CV_64F);
			for (int i=index_i_min;i<=index_i_max;i++)
			{
				for (int j=index_j_min;j<=index_j_max;j++)
				{
					int s = i-index_i_min;
					int t = j-index_j_min;
					double val;
					if (image_type==1)
					{
						val =(double) CV_MAT_ELEM(*image,uchar,i,j);
					} 
					else
					{
						val = CV_MAT_ELEM(*image,double,i,j);
					}
					CV_MAT_ELEM(*Y_mat,double,s,t) = val;
				}
			}


			//S= Q* Y* QT
			CvMat* Q_mat_4X8_Y_mat_cache_1 = cvCreateMat(4,8,CV_64F);
			cvMatMul(Q_mat,Y_mat,Q_mat_4X8_Y_mat_cache_1);
			CvMat* S_mat = cvCreateMat(4,4,CV_64F);
			cvMatMul(Q_mat_4X8_Y_mat_cache_1,Q_mat_4X8_Transpose,S_mat);





			double g,gx,gy;




			//方法1
			//CvMat* delta_y_vector = cvCreateMat(1,4,CV_64F);
			//double* delta_y_vector_ptr = delta_y_vector->data.db;
			//delta_y_vector_ptr[0] = delta_y3;
			//delta_y_vector_ptr[1] = delta_y2;
			//delta_y_vector_ptr[2] = delta_y;
			//delta_y_vector_ptr[3] = 1;

			//CvMat* delta_dy_vector = cvCreateMat(1,4,CV_64F);
			//double* delta_dy_vector_ptr = delta_dy_vector->data.db;
			//delta_dy_vector_ptr[0] = 3.0*delta_y2;
			//delta_dy_vector_ptr[1] = 2.0*delta_y;
			//delta_dy_vector_ptr[2] = 1.0;
			//delta_dy_vector_ptr[3] = 0;



			//CvMat* delta_x_vector = cvCreateMat(4,1,CV_64F);
			//double* delta_x_vector_ptr = delta_x_vector->data.db;
			//delta_x_vector_ptr[0] = delta_x3;
			//delta_x_vector_ptr[1] = delta_x2;
			//delta_x_vector_ptr[2] = delta_x;
			//delta_x_vector_ptr[3] = 1.0;

			//CvMat* delta_dx_vector = cvCreateMat(4,1,CV_64F);
			//double* delta_dx_vector_ptr=  delta_dx_vector->data.db;
			//delta_dx_vector_ptr[0] = 3.0*delta_x2;
			//delta_dx_vector_ptr[1] = 2.0*delta_x;
			//delta_dx_vector_ptr[2] = 1.0;
			//delta_dx_vector_ptr[3] = 0;


			////g
			//CvMat* result_g=  cvCreateMat(1,1,CV_64F);
			//CvMat* delta_y_vector_S_mat_cache_2= cvCreateMat(1,4,CV_64F);
			//cvMatMul(delta_y_vector,S_mat,delta_y_vector_S_mat_cache_2);
			//cvMatMul(delta_y_vector_S_mat_cache_2,delta_x_vector,result_g);


			////gx
			//CvMat* result_gx = cvCreateMat(1,1,CV_64F);
			//cvMatMul(delta_y_vector_S_mat_cache_2,delta_dx_vector,result_gx);


			////gy
			//CvMat* result_gy = cvCreateMat(1,1,CV_64F);
			//CvMat* delta_dy_vector_S_mat_cache_3 = cvCreateMat(1,4,CV_64F);
			//cvMatMul(delta_dy_vector,S_mat,delta_dy_vector_S_mat_cache_3);
			//cvMatMul(delta_dy_vector_S_mat_cache_3,delta_x_vector,result_gy);

			//g = CV_MAT_ELEM(*result_g,double,0,0);
			//gx = CV_MAT_ELEM(*result_gx,double,0,0);
			//gy = CV_MAT_ELEM(*result_gy,double,0,0);




			//cvReleaseMat(&delta_y_vector);
			//cvReleaseMat(&delta_dy_vector);
			//cvReleaseMat(&delta_x_vector);
			//cvReleaseMat(&delta_dx_vector);
			//cvReleaseMat(&result_g);
			//cvReleaseMat(&delta_y_vector_S_mat_cache_2);
			//cvReleaseMat(&result_gx);
			//cvReleaseMat(&result_gy);
			//cvReleaseMat(&delta_dy_vector_S_mat_cache_3);


			//方法2--速度为方法1的三倍

			double* S_mat_ptr = S_mat->data.db;
			g = delta_x3 * (delta_y3 * S_mat_ptr[0] + delta_y2 * S_mat_ptr[4] + delta_y * S_mat_ptr[8] + S_mat_ptr[12] ) +
				delta_x2 * (delta_y3 * S_mat_ptr[1] + delta_y2 * S_mat_ptr[5] + delta_y * S_mat_ptr[9] + S_mat_ptr[13] ) +
				delta_x  * (delta_y3 * S_mat_ptr[2] + delta_y2 * S_mat_ptr[6] + delta_y * S_mat_ptr[10] + S_mat_ptr[14] ) +
				(delta_y3 * S_mat_ptr[3] + delta_y2 * S_mat_ptr[7] + delta_y * S_mat_ptr[11] + S_mat_ptr[15] );
			gx= 3.0 * delta_x2 * (delta_y3 * S_mat_ptr[0] + delta_y2 * S_mat_ptr[4] + delta_y * S_mat_ptr[8] + S_mat_ptr[12] ) +
				2.0 * delta_x  * (delta_y3 * S_mat_ptr[1] + delta_y2 * S_mat_ptr[5] + delta_y * S_mat_ptr[9] + S_mat_ptr[13] ) +
				(delta_y3 * S_mat_ptr[2] + delta_y2 * S_mat_ptr[6] + delta_y * S_mat_ptr[10] + S_mat_ptr[14] );
			gy = delta_x3 * (3.0 * delta_y2 * S_mat_ptr[0] + 2.0 * delta_y * S_mat_ptr[4] + S_mat_ptr[8] ) +
				delta_x2 * (3.0 * delta_y2 * S_mat_ptr[1] +  2.0 * delta_y * S_mat_ptr[5] + S_mat_ptr[9] ) +
				delta_x  * (3.0 * delta_y2 * S_mat_ptr[2] +  2.0 * delta_y * S_mat_ptr[6] + S_mat_ptr[10] ) +
				(3.0 * delta_y2 * S_mat_ptr[3] +  2.0 * delta_y * S_mat_ptr[7] + S_mat_ptr[11] );



			CV_MAT_ELEM(*g_mat,double,m,n) = g;
			CV_MAT_ELEM(*gx_mat,double,m,n) =gx;
			CV_MAT_ELEM(*gy_mat,double,m,n) = gy;
			cvReleaseMat(&Q_mat_4X8_Transpose);
			cvReleaseMat(&Y_mat);
			cvReleaseMat(&Q_mat_4X8_Y_mat_cache_1);
			cvReleaseMat(&S_mat);
		}
	}









	return true;
}



bool UniformCubicNatureSplineInterpolationMethod::Two_Dimension_Interpolation_Many_Points(
	CvMat* image, 
	CvMat* pos_x_mat, 
	CvMat* pos_y_mat, 
	int aoi_x_min, 
	int aoi_x_max,/*上闭区间，下开区间 */
	int aoi_y_min,
	int aoi_y_max,/*左闭区间，右边开区间 */ 
	CvMat* lut_mat,
	CvMat* g_mat,
	CvMat* gx_mat,
	CvMat* gy_mat 
	)
{
	//判断
	//大小
	int image_width= image->width;
	int image_height = image->height;
	int mat_width = pos_x_mat->cols;
	int mat_height = pos_x_mat->cols;
	int lut_width = lut_mat->cols;
	int lut_height = lut_mat->rows;

	if (pos_y_mat->cols != mat_width ||
		pos_y_mat->rows != mat_height ||
		g_mat->cols != mat_width ||
		g_mat->rows != mat_height ||
		gx_mat->cols != mat_width ||
		gx_mat->rows != mat_height ||
		gy_mat->cols != mat_width ||
		gy_mat->rows != mat_height 
		)
	{
		return false;
	}


	if (aoi_x_min<0 || aoi_x_max>=image_width||
		aoi_y_min<0 || aoi_y_max>=image_height)
	{
		return false;
	}
	int aoi_width = aoi_x_max-aoi_x_min;
	int aoi_height = aoi_y_max-aoi_y_min;

	if (lut_mat->cols != aoi_width*lut_element_size_||
		lut_mat->rows !=aoi_height)
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
	if (CV_MAT_TYPE(pos_x_mat->type) != CV_64F ||
		CV_MAT_TYPE(pos_y_mat->type) != CV_64F ||
		CV_MAT_TYPE(g_mat->type) != CV_64F ||
		CV_MAT_TYPE(gx_mat->type) != CV_64F ||
		CV_MAT_TYPE(gy_mat->type) != CV_64F 
		)
	{
		return false;
	}


	double pos_x_min,pos_y_min,pos_x_max,pos_y_max;
	cvMinMaxLoc(pos_x_mat,&pos_x_min,&pos_x_max);
	cvMinMaxLoc(pos_y_mat,&pos_y_min,&pos_y_max);

	if (pos_x_min<aoi_x_min || pos_x_max>=aoi_x_max||
		pos_y_min<aoi_y_min || pos_y_max>=aoi_y_max)//右侧为开区间
	{
		return false;
	}




	if (lut_type==1)
	{
		for (int i =0;i<mat_height;i++)
		{
			for (int j =0;j<mat_width;j++)
			{
				double pos_x = CV_MAT_ELEM(*pos_x_mat,double,i,j);
				double pos_y = CV_MAT_ELEM(*pos_y_mat,double,i,j);


				double delta_x = pos_x - cvFloor(pos_x);
				double delta_y = pos_y - cvFloor(pos_y);

				float* S_mat_ptr = lut_mat->data.fl+ ((cvFloor(pos_y) - aoi_y_min) * aoi_width+cvFloor(pos_x) - aoi_x_min) * lut_element_size_;

				CV_MAT_ELEM(*g_mat,double,i,j) = 
					delta_x*delta_x*delta_x * (delta_y*delta_y*delta_y * S_mat_ptr[0] + delta_y*delta_y * S_mat_ptr[4] + delta_y * S_mat_ptr[8] + S_mat_ptr[12] ) +
					delta_x*delta_x * (delta_y*delta_y*delta_y * S_mat_ptr[1] + delta_y*delta_y * S_mat_ptr[5] + delta_y * S_mat_ptr[9] + S_mat_ptr[13] ) +
					delta_x  * (delta_y*delta_y*delta_y * S_mat_ptr[2] + delta_y*delta_y * S_mat_ptr[6] + delta_y * S_mat_ptr[10] + S_mat_ptr[14] ) +
					(delta_y*delta_y*delta_y * S_mat_ptr[3] + delta_y*delta_y * S_mat_ptr[7] + delta_y * S_mat_ptr[11] + S_mat_ptr[15] );
				CV_MAT_ELEM(*gx_mat,double,i,j) =
					3.0 * delta_x*delta_x * (delta_y*delta_y*delta_y * S_mat_ptr[0] + delta_y*delta_y * S_mat_ptr[4] + delta_y * S_mat_ptr[8] + S_mat_ptr[12] ) +
					2.0 * delta_x  * (delta_y*delta_y*delta_y * S_mat_ptr[1] + delta_y*delta_y * S_mat_ptr[5] + delta_y * S_mat_ptr[9] + S_mat_ptr[13] ) +
					(delta_y*delta_y*delta_y * S_mat_ptr[2] + delta_y*delta_y * S_mat_ptr[6] + delta_y * S_mat_ptr[10] + S_mat_ptr[14] );
				CV_MAT_ELEM(*gy_mat,double,i,j) =
					delta_x*delta_x*delta_x * (3.0 * delta_y*delta_y * S_mat_ptr[0] + 2.0 * delta_y * S_mat_ptr[4] + S_mat_ptr[8] ) +
					delta_x*delta_x * (3.0 * delta_y*delta_y * S_mat_ptr[1] +  2.0 * delta_y * S_mat_ptr[5] + S_mat_ptr[9] ) +
					delta_x  * (3.0 * delta_y*delta_y * S_mat_ptr[2] +  2.0 * delta_y * S_mat_ptr[6] + S_mat_ptr[10] ) +
					(3.0 * delta_y*delta_y * S_mat_ptr[3] +  2.0 * delta_y * S_mat_ptr[7] + S_mat_ptr[11] );
			}
		}
	} 
	else
	{
		for (int i =0;i<mat_height;i++)
		{
			for (int j =0;j<mat_width;j++)
			{
				double pos_x = CV_MAT_ELEM(*pos_x_mat,double,i,j);
				double pos_y = CV_MAT_ELEM(*pos_y_mat,double,i,j);


				double delta_x = pos_x - cvFloor(pos_x);
				double delta_y = pos_y - cvFloor(pos_y);

				double* S_mat_ptr = lut_mat->data.db+ ((cvFloor(pos_y) - aoi_y_min) * aoi_width+cvFloor(pos_x) - aoi_x_min) * lut_element_size_;

				CV_MAT_ELEM(*g_mat,double,i,j) = 
					delta_x*delta_x*delta_x * (delta_y*delta_y*delta_y * S_mat_ptr[0] + delta_y*delta_y * S_mat_ptr[4] + delta_y * S_mat_ptr[8] + S_mat_ptr[12] ) +
					delta_x*delta_x * (delta_y*delta_y*delta_y * S_mat_ptr[1] + delta_y*delta_y * S_mat_ptr[5] + delta_y * S_mat_ptr[9] + S_mat_ptr[13] ) +
					delta_x  * (delta_y*delta_y*delta_y * S_mat_ptr[2] + delta_y*delta_y * S_mat_ptr[6] + delta_y * S_mat_ptr[10] + S_mat_ptr[14] ) +
					(delta_y*delta_y*delta_y * S_mat_ptr[3] + delta_y*delta_y * S_mat_ptr[7] + delta_y * S_mat_ptr[11] + S_mat_ptr[15] );
				CV_MAT_ELEM(*gx_mat,double,i,j) =
					3.0 * delta_x*delta_x * (delta_y*delta_y*delta_y * S_mat_ptr[0] + delta_y*delta_y * S_mat_ptr[4] + delta_y * S_mat_ptr[8] + S_mat_ptr[12] ) +
					2.0 * delta_x  * (delta_y*delta_y*delta_y * S_mat_ptr[1] + delta_y*delta_y * S_mat_ptr[5] + delta_y * S_mat_ptr[9] + S_mat_ptr[13] ) +
					(delta_y*delta_y*delta_y * S_mat_ptr[2] + delta_y*delta_y * S_mat_ptr[6] + delta_y * S_mat_ptr[10] + S_mat_ptr[14] );
				CV_MAT_ELEM(*gy_mat,double,i,j) =
					delta_x*delta_x*delta_x * (3.0 * delta_y*delta_y * S_mat_ptr[0] + 2.0 * delta_y * S_mat_ptr[4] + S_mat_ptr[8] ) +
					delta_x*delta_x * (3.0 * delta_y*delta_y * S_mat_ptr[1] +  2.0 * delta_y * S_mat_ptr[5] + S_mat_ptr[9] ) +
					delta_x  * (3.0 * delta_y*delta_y * S_mat_ptr[2] +  2.0 * delta_y * S_mat_ptr[6] + S_mat_ptr[10] ) +
					(3.0 * delta_y*delta_y * S_mat_ptr[3] +  2.0 * delta_y * S_mat_ptr[7] + S_mat_ptr[11] );
			}
		}
	}
	



	//时间测试
	//////////////////////////////////////////////////////////////////////////
	//QTime timer = QTime::currentTime();
	//int a[10]= {0};
	//int count_a = 10000;



	//

	//timer.start();
	//for (int k =0;k<count_a;k++)
	//{

	//}
	//a[0] = timer.restart();




	//int image_width= image->width;
	//int image_height = image->height;
	//int mat_width = pos_x_mat->cols;
	//int mat_height = pos_x_mat->cols;
	//int lut_width = lut_mat->cols;
	//int lut_height = lut_mat->rows;
	//int aoi_width = aoi_x_max-aoi_x_min;
	//int aoi_height = aoi_y_max-aoi_y_min;

	//double pos_x_min,pos_y_min,pos_x_max,pos_y_max;

	//for (int k =0;k<count_a;k++)
	//{
	//	cvMinMaxLoc(pos_x_mat,&pos_x_min,&pos_x_max);
	//	cvMinMaxLoc(pos_y_mat,&pos_y_min,&pos_y_max);
	//}

	//a[1] = timer.restart();


	//for (int k =0;k<count_a;k++)
	//{
	//	if (pos_y_mat->cols != mat_width ||
	//		pos_y_mat->rows != mat_height ||
	//		g_mat->cols != mat_width ||
	//		g_mat->rows != mat_height ||
	//		gx_mat->cols != mat_width ||
	//		gx_mat->rows != mat_height ||
	//		gy_mat->cols != mat_width ||
	//		gy_mat->rows != mat_height 
	//		)
	//	{
	//		return false;
	//	}
	//	if (aoi_x_min<3 || aoi_x_max>image_width-4||
	//		aoi_y_min<3 || aoi_y_max>image_height-4)
	//	{
	//		return false;
	//	}


	//	if (lut_mat->cols != aoi_width*lut_element_size_||
	//		lut_mat->rows !=aoi_height)
	//	{
	//		return false;
	//	}

	//	int image_type = 0;//1--CV_8U,2--CV_64F
	//	if (CV_MAT_TYPE(image->type) ==CV_8U )
	//	{
	//		image_type = 1;
	//	}
	//	else if (CV_MAT_TYPE(image->type) ==CV_64F)
	//	{
	//		image_type = 2;
	//	} 
	//	else
	//	{
	//		return false;
	//	}

	//	if (CV_MAT_TYPE(lut_mat->type) != CV_64F)
	//	{
	//		return false;
	//	}
	//	if (CV_MAT_TYPE(pos_x_mat->type) != CV_64F ||
	//		CV_MAT_TYPE(pos_y_mat->type) != CV_64F ||
	//		CV_MAT_TYPE(g_mat->type) != CV_64F ||
	//		CV_MAT_TYPE(gx_mat->type) != CV_64F ||
	//		CV_MAT_TYPE(gy_mat->type) != CV_64F 
	//		)
	//	{
	//		return false;
	//	}




	//	if (pos_x_min<aoi_x_min || pos_x_max>=aoi_x_max||
	//		pos_y_min<aoi_y_min || pos_y_max>=aoi_y_max)//右侧为开区间
	//	{
	//		return false;
	//	}
	//}

	//a[2] = timer.restart();

	//


	//for (int k =0;k<count_a;k++)
	//{
	//	for (int i =0;i<mat_height;i++)
	//	{
	//		for (int j =0;j<mat_width;j++)
	//		{
	//			//double pos_x = CV_MAT_ELEM(*pos_x_mat,double,i,j);
	//			//double pos_y = CV_MAT_ELEM(*pos_y_mat,double,i,j);

	//			//int left = cvFloor(pos_x);
	//			//int top = cvFloor(pos_y);

	//			//double delta_x = pos_x - (double)left;
	//			//double delta_y = pos_y - (double)top;
	//			//double delta_x3 = pow(delta_x,3.0);
	//			//double delta_x2 = pow(delta_x,2.0);
	//			//double delta_y3=  pow(delta_y,3.0);
	//			//double delta_y2 = pow(delta_y,2.0);

	//			//int lut_index_j = left - aoi_x_min;
	//			//int lut_index_i = top - aoi_y_min;
	//			//double* S_mat_ptr = lut_mat->data.db+ (lut_index_i * aoi_width+lut_index_j) * lut_element_size_;

	//			//CV_MAT_ELEM(*g_mat,double,i,j) = 
	//			//	delta_x3 * (delta_y3 * S_mat_ptr[0] + delta_y2 * S_mat_ptr[4] + delta_y * S_mat_ptr[8] + S_mat_ptr[12] ) +
	//			//	delta_x2 * (delta_y3 * S_mat_ptr[1] + delta_y2 * S_mat_ptr[5] + delta_y * S_mat_ptr[9] + S_mat_ptr[13] ) +
	//			//	delta_x  * (delta_y3 * S_mat_ptr[2] + delta_y2 * S_mat_ptr[6] + delta_y * S_mat_ptr[10] + S_mat_ptr[14] ) +
	//			//	(delta_y3 * S_mat_ptr[3] + delta_y2 * S_mat_ptr[7] + delta_y * S_mat_ptr[11] + S_mat_ptr[15] );
	//			//CV_MAT_ELEM(*gx_mat,double,i,j) =
	//			//	3.0 * delta_x2 * (delta_y3 * S_mat_ptr[0] + delta_y2 * S_mat_ptr[4] + delta_y * S_mat_ptr[8] + S_mat_ptr[12] ) +
	//			//	2.0 * delta_x  * (delta_y3 * S_mat_ptr[1] + delta_y2 * S_mat_ptr[5] + delta_y * S_mat_ptr[9] + S_mat_ptr[13] ) +
	//			//	(delta_y3 * S_mat_ptr[2] + delta_y2 * S_mat_ptr[6] + delta_y * S_mat_ptr[10] + S_mat_ptr[14] );
	//			//CV_MAT_ELEM(*gy_mat,double,i,j) =
	//			//	delta_x3 * (3.0 * delta_y2 * S_mat_ptr[0] + 2.0 * delta_y * S_mat_ptr[4] + S_mat_ptr[8] ) +
	//			//	delta_x2 * (3.0 * delta_y2 * S_mat_ptr[1] +  2.0 * delta_y * S_mat_ptr[5] + S_mat_ptr[9] ) +
	//			//	delta_x  * (3.0 * delta_y2 * S_mat_ptr[2] +  2.0 * delta_y * S_mat_ptr[6] + S_mat_ptr[10] ) +
	//			//	(3.0 * delta_y2 * S_mat_ptr[3] +  2.0 * delta_y * S_mat_ptr[7] + S_mat_ptr[11] );


	//			double pos_x = CV_MAT_ELEM(*pos_x_mat,double,i,j);
	//			double pos_y = CV_MAT_ELEM(*pos_y_mat,double,i,j);


	//			double delta_x = pos_x - cvFloor(pos_x);
	//			double delta_y = pos_y - cvFloor(pos_y);

	//			double* S_mat_ptr = lut_mat->data.db+ ((cvFloor(pos_y) - aoi_y_min) * aoi_width+cvFloor(pos_x) - aoi_x_min) * lut_element_size_;

	//			CV_MAT_ELEM(*g_mat,double,i,j) = 
	//				delta_x*delta_x*delta_x * (delta_y*delta_y*delta_y * S_mat_ptr[0] + delta_y*delta_y * S_mat_ptr[4] + delta_y * S_mat_ptr[8] + S_mat_ptr[12] ) +
	//				delta_x*delta_x * (delta_y*delta_y*delta_y * S_mat_ptr[1] + delta_y*delta_y * S_mat_ptr[5] + delta_y * S_mat_ptr[9] + S_mat_ptr[13] ) +
	//				delta_x  * (delta_y*delta_y*delta_y * S_mat_ptr[2] + delta_y*delta_y * S_mat_ptr[6] + delta_y * S_mat_ptr[10] + S_mat_ptr[14] ) +
	//				(delta_y*delta_y*delta_y * S_mat_ptr[3] + delta_y*delta_y * S_mat_ptr[7] + delta_y * S_mat_ptr[11] + S_mat_ptr[15] );
	//			CV_MAT_ELEM(*gx_mat,double,i,j) =
	//				3.0 * delta_x*delta_x * (delta_y*delta_y*delta_y * S_mat_ptr[0] + delta_y*delta_y * S_mat_ptr[4] + delta_y * S_mat_ptr[8] + S_mat_ptr[12] ) +
	//				2.0 * delta_x  * (delta_y*delta_y*delta_y * S_mat_ptr[1] + delta_y*delta_y * S_mat_ptr[5] + delta_y * S_mat_ptr[9] + S_mat_ptr[13] ) +
	//				(delta_y*delta_y*delta_y * S_mat_ptr[2] + delta_y*delta_y * S_mat_ptr[6] + delta_y * S_mat_ptr[10] + S_mat_ptr[14] );
	//			CV_MAT_ELEM(*gy_mat,double,i,j) =
	//				delta_x*delta_x*delta_x * (3.0 * delta_y*delta_y * S_mat_ptr[0] + 2.0 * delta_y * S_mat_ptr[4] + S_mat_ptr[8] ) +
	//				delta_x*delta_x * (3.0 * delta_y*delta_y * S_mat_ptr[1] +  2.0 * delta_y * S_mat_ptr[5] + S_mat_ptr[9] ) +
	//				delta_x  * (3.0 * delta_y*delta_y * S_mat_ptr[2] +  2.0 * delta_y * S_mat_ptr[6] + S_mat_ptr[10] ) +
	//				(3.0 * delta_y*delta_y * S_mat_ptr[3] +  2.0 * delta_y * S_mat_ptr[7] + S_mat_ptr[11] );

	//		}
	//	}
	//}
	//a[3] = timer.restart();



	return true;

}
bool UniformCubicNatureSplineInterpolationMethod::Two_Dimension_Interpolation_Many_Points( 
	CvMat* image,
	int reference_point_x,
	int reference_point_y,
	CvMat* displacement_input,//映射后应全部处于图像宽度和高度内，64F,6行1列
	int aoi_x_min,
	int aoi_x_max,//上闭区间，下开区间
	int aoi_y_min,
	int aoi_y_max,//左闭区间，右边开区间
	CvMat* lut_mat,
	CvMat* g_mat
	)
{
	//判断
	//大小
	int image_width= image->width;
	int image_height = image->height;
	int template_width = g_mat->cols;
	int template_height = g_mat->cols;
	int lut_width = lut_mat->cols;
	int lut_height = lut_mat->rows;

	if (displacement_input->rows!=6 && displacement_input->cols!=1)
	{
		return false;
	} 

	if (aoi_x_min<0 || aoi_x_max>=image_width||
		aoi_y_min<0 || aoi_y_max>=image_height)
	{
		return false;
	}
	int aoi_width = aoi_x_max-aoi_x_min;
	int aoi_height = aoi_y_max-aoi_y_min;

	if (lut_mat->cols != aoi_width*lut_element_size_||
		lut_mat->rows !=aoi_height)
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
	if ( CV_MAT_TYPE(displacement_input->type) != CV_64F)
	{
		return false;
	}
	if (
		CV_MAT_TYPE(g_mat->type) != CV_64F
		)
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


	if (lut_type==1)
	{
		for (int i =0;i<template_height;i++)
		{
			for (int j =0;j<template_width;j++)
			{
				//第一种
				double pos_x =reference_point_x + j - template_half_width + u + ux * (j - template_half_width) + uy * (i - template_half_height);
				double pos_y =reference_point_y + i - template_half_height + v + vx * (j - template_half_width) + vy * (i - template_half_height);

				if (pos_x<aoi_x_min || pos_x>=aoi_x_max||
					pos_y<aoi_y_min || pos_y>=aoi_y_max)//右侧为开区间
				{
					return false;
				}

				double delta_x = pos_x - cvFloor(pos_x);
				double delta_y = pos_y - cvFloor(pos_y);
				float* S_mat_ptr = lut_mat->data.fl+ ((cvFloor(pos_y) - aoi_y_min) * aoi_width+cvFloor(pos_x) - aoi_x_min) * lut_element_size_;
				CV_MAT_ELEM(*g_mat,double,i,j) = 
					delta_x*delta_x*delta_x * (delta_y*delta_y*delta_y * S_mat_ptr[0] + delta_y*delta_y * S_mat_ptr[4] + delta_y * S_mat_ptr[8] + S_mat_ptr[12] ) +
					delta_x*delta_x * (delta_y*delta_y*delta_y * S_mat_ptr[1] + delta_y*delta_y * S_mat_ptr[5] + delta_y * S_mat_ptr[9] + S_mat_ptr[13] ) +
					delta_x  * (delta_y*delta_y*delta_y * S_mat_ptr[2] + delta_y*delta_y * S_mat_ptr[6] + delta_y * S_mat_ptr[10] + S_mat_ptr[14] ) +
					(delta_y*delta_y*delta_y * S_mat_ptr[3] + delta_y*delta_y * S_mat_ptr[7] + delta_y * S_mat_ptr[11] + S_mat_ptr[15] );



				//第二种--没第一种快
				//double delta_x = reference_point_x + j - model_half_width + u + ux * (j - model_half_width) + uy * (i - model_half_height) - cvFloor(reference_point_x + j - model_half_width + u + ux * (j - model_half_width) + uy * (i - model_half_height));
				//double delta_y = reference_point_y + i - model_half_height + v + vx * (j - model_half_width) + vy * (i - model_half_height) - cvFloor(reference_point_y + i - model_half_height + v + vx * (j - model_half_width) + vy * (i - model_half_height));
				//int lut_index =((reference_point_y + i - model_half_height + v + vx * (j - model_half_width) + vy * (i - model_half_height) -aoi_y_min)*aoi_width +reference_point_x + j - model_half_width + u + ux * (j - model_half_width) + uy * (i - model_half_height) -aoi_x_min )*lut_element_size_;
				//double* S_mat_ptr = lut_mat->data.db+ lut_index;
				//CV_MAT_ELEM(*g_mat,double,i,j) = 
				//	delta_x*delta_x*delta_x * (delta_y*delta_y*delta_y * S_mat_ptr[0] + delta_y*delta_y * S_mat_ptr[4] + delta_y * S_mat_ptr[8] + S_mat_ptr[12] ) +
				//	delta_x*delta_x * (delta_y*delta_y*delta_y * S_mat_ptr[1] + delta_y*delta_y * S_mat_ptr[5] + delta_y * S_mat_ptr[9] + S_mat_ptr[13] ) +
				//	delta_x  * (delta_y*delta_y*delta_y * S_mat_ptr[2] + delta_y*delta_y * S_mat_ptr[6] + delta_y * S_mat_ptr[10] + S_mat_ptr[14] ) +
				//	(delta_y*delta_y*delta_y * S_mat_ptr[3] + delta_y*delta_y * S_mat_ptr[7] + delta_y * S_mat_ptr[11] + S_mat_ptr[15] );


			}
		}
	}
	else
	{
		for (int i =0;i<template_height;i++)
		{
			for (int j =0;j<template_width;j++)
			{
				//第一种
				double pos_x =reference_point_x + j - template_half_width + u + ux * (j - template_half_width) + uy * (i - template_half_height);
				double pos_y =reference_point_y + i - template_half_height + v + vx * (j - template_half_width) + vy * (i - template_half_height);

				if (pos_x<aoi_x_min || pos_x>=aoi_x_max||
					pos_y<aoi_y_min || pos_y>=aoi_y_max)//右侧为开区间
				{
					return false;
				}

				double delta_x = pos_x - cvFloor(pos_x);
				double delta_y = pos_y - cvFloor(pos_y);
				double* S_mat_ptr = lut_mat->data.db+ ((cvFloor(pos_y) - aoi_y_min) * aoi_width+cvFloor(pos_x) - aoi_x_min) * lut_element_size_;
				CV_MAT_ELEM(*g_mat,double,i,j) = 
					delta_x*delta_x*delta_x * (delta_y*delta_y*delta_y * S_mat_ptr[0] + delta_y*delta_y * S_mat_ptr[4] + delta_y * S_mat_ptr[8] + S_mat_ptr[12] ) +
					delta_x*delta_x * (delta_y*delta_y*delta_y * S_mat_ptr[1] + delta_y*delta_y * S_mat_ptr[5] + delta_y * S_mat_ptr[9] + S_mat_ptr[13] ) +
					delta_x  * (delta_y*delta_y*delta_y * S_mat_ptr[2] + delta_y*delta_y * S_mat_ptr[6] + delta_y * S_mat_ptr[10] + S_mat_ptr[14] ) +
					(delta_y*delta_y*delta_y * S_mat_ptr[3] + delta_y*delta_y * S_mat_ptr[7] + delta_y * S_mat_ptr[11] + S_mat_ptr[15] );



				//第二种--没第一种快
				//double delta_x = reference_point_x + j - model_half_width + u + ux * (j - model_half_width) + uy * (i - model_half_height) - cvFloor(reference_point_x + j - model_half_width + u + ux * (j - model_half_width) + uy * (i - model_half_height));
				//double delta_y = reference_point_y + i - model_half_height + v + vx * (j - model_half_width) + vy * (i - model_half_height) - cvFloor(reference_point_y + i - model_half_height + v + vx * (j - model_half_width) + vy * (i - model_half_height));
				//int lut_index =((reference_point_y + i - model_half_height + v + vx * (j - model_half_width) + vy * (i - model_half_height) -aoi_y_min)*aoi_width +reference_point_x + j - model_half_width + u + ux * (j - model_half_width) + uy * (i - model_half_height) -aoi_x_min )*lut_element_size_;
				//double* S_mat_ptr = lut_mat->data.db+ lut_index;
				//CV_MAT_ELEM(*g_mat,double,i,j) = 
				//	delta_x*delta_x*delta_x * (delta_y*delta_y*delta_y * S_mat_ptr[0] + delta_y*delta_y * S_mat_ptr[4] + delta_y * S_mat_ptr[8] + S_mat_ptr[12] ) +
				//	delta_x*delta_x * (delta_y*delta_y*delta_y * S_mat_ptr[1] + delta_y*delta_y * S_mat_ptr[5] + delta_y * S_mat_ptr[9] + S_mat_ptr[13] ) +
				//	delta_x  * (delta_y*delta_y*delta_y * S_mat_ptr[2] + delta_y*delta_y * S_mat_ptr[6] + delta_y * S_mat_ptr[10] + S_mat_ptr[14] ) +
				//	(delta_y*delta_y*delta_y * S_mat_ptr[3] + delta_y*delta_y * S_mat_ptr[7] + delta_y * S_mat_ptr[11] + S_mat_ptr[15] );


			}
		}
	}

	return true;

	
}

bool UniformCubicNatureSplineInterpolationMethod::Two_Dimension_Interpolation_Many_Points_For_Image_8U(
	CvMat* image,
	int reference_point_x,
	int reference_point_y,
	CvMat* displacement_input,//映射后应全部处于图像宽度和高度内，64F,6行1列
	int aoi_x_min,
	int aoi_x_max,//上闭区间，下开区间
	int aoi_y_min,
	int aoi_y_max,//左闭区间，右边开区间
	CvMat* lut_mat,
	CvMat* lut_valid_sign_mat,
	CvMat* Q_mat,
	CvMat* g_mat
	)
{
	//判断
	//大小
	int image_width= image->width;
	int image_height = image->height;
	int template_width = g_mat->cols;
	int template_height = g_mat->cols;
	int lut_width = lut_mat->cols;
	int lut_height = lut_mat->rows;

	if (displacement_input->rows!=6 && displacement_input->cols!=1)
	{
		return false;
	} 

	if (aoi_x_min<0 || aoi_x_max>=image_width||
		aoi_y_min<0 || aoi_y_max>=image_height)
	{
		return false;
	}
	int aoi_width = aoi_x_max-aoi_x_min;
	int aoi_height = aoi_y_max-aoi_y_min;

	if (lut_mat->cols != aoi_width*lut_element_size_||
		lut_mat->rows !=aoi_height||
		lut_valid_sign_mat->cols!=aoi_width||
		lut_valid_sign_mat->rows!= aoi_height)
	{
		return false;
	}
	
	int Q_mat_type =0;//1--4x6,2--4x8
	if (Q_mat->rows==4 && Q_mat->cols==6)
	{
		Q_mat_type =1;
	}
	else if (Q_mat->rows==4 && Q_mat->cols==8)
	{
		Q_mat_type=2;
	} 
	else
	{
		return false;
	}



	//类型
	int image_type = 0;//1--CV_8U,2--CV_64F
	if (CV_MAT_TYPE(image->type) ==CV_8U )
	{
		image_type = 1;
	}
	//else if (CV_MAT_TYPE(image->type) ==CV_64F)
	//{
	//	image_type = 2;
	//} 
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
	if ( CV_MAT_TYPE(displacement_input->type) != CV_64F)
	{
		return false;
	}
	if (CV_MAT_TYPE(g_mat->type) != CV_64F || CV_MAT_TYPE(Q_mat->type) != CV_64F || CV_MAT_TYPE(lut_valid_sign_mat->type)!=CV_8U)
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
	double* Q_mat_ptr = Q_mat->data.db;


	if (lut_type==1)
	{
		for (int i =0;i<template_height;i++)
		{
			for (int j =0;j<template_width;j++)
			{
				//第一种
				double pos_x =reference_point_x + j - template_half_width + u + ux * (j - template_half_width) + uy * (i - template_half_height);
				double pos_y =reference_point_y + i - template_half_height + v + vx * (j - template_half_width) + vy * (i - template_half_height);

				if (pos_x<aoi_x_min || pos_x>=aoi_x_max||
					pos_y<aoi_y_min || pos_y>=aoi_y_max)//右侧为开区间
				{
					return false;
				}
				int index_x = cvFloor(pos_x);
				int index_y = cvFloor(pos_y);

				double delta_x = pos_x - index_x;
				double delta_y = pos_y - index_y;
				uchar sign = CV_MAT_ELEM(*lut_valid_sign_mat,uchar,index_y - aoi_y_min,index_x - aoi_x_min);
				float* S_mat_ptr = lut_mat->data.fl+ ((index_y - aoi_y_min) * aoi_width+index_x - aoi_x_min) * lut_element_size_;
				if (sign!=1)
				{
					uchar* y;
					if (Q_mat_type==1)
					{
						y = image->data.ptr + (index_y-2)*image_width+index_x-2;
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

					} 
					else
					{
						y = image->data.ptr + (index_y-3)*image_width+index_x-3;
						S_mat_ptr[0] = (Q_mat_ptr[0] * y[0] + Q_mat_ptr[1] * y[image_width] + Q_mat_ptr[2] * y[2 * image_width] + Q_mat_ptr[3] * y[3 * image_width] + Q_mat_ptr[4] * y[4 * image_width] + Q_mat_ptr[5] * y[5 * image_width] + Q_mat_ptr[6] * y[6 * image_width] + Q_mat_ptr[7] * y[7 * image_width]) * Q_mat_ptr[0] + (Q_mat_ptr[0] * y[1] + Q_mat_ptr[1] * y[1 + image_width] + Q_mat_ptr[2] * y[1 + 2 * image_width] + Q_mat_ptr[3] * y[1 + 3 * image_width] + Q_mat_ptr[4] * y[1 + 4 * image_width] + Q_mat_ptr[5] * y[1 + 5 * image_width] + Q_mat_ptr[6] * y[1 + 6 * image_width] + Q_mat_ptr[7] * y[1 + 7 * image_width]) * Q_mat_ptr[1] + (Q_mat_ptr[0] * y[2] + Q_mat_ptr[1] * y[2 + image_width] + Q_mat_ptr[2] * y[2 + 2 * image_width] + Q_mat_ptr[3] * y[2 + 3 * image_width] + Q_mat_ptr[4] * y[2 + 4 * image_width] + Q_mat_ptr[5] * y[2 + 5 * image_width] + Q_mat_ptr[6] * y[2 + 6 * image_width] + Q_mat_ptr[7] * y[2 + 7 * image_width]) * Q_mat_ptr[2] + (Q_mat_ptr[0] * y[3] + Q_mat_ptr[1] * y[3 + image_width] + Q_mat_ptr[2] * y[3 + 2 * image_width] + Q_mat_ptr[3] * y[3 + 3 * image_width] + Q_mat_ptr[4] * y[3 + 4 * image_width] + Q_mat_ptr[5] * y[3 + 5 * image_width] + Q_mat_ptr[6] * y[3 + 6 * image_width] + Q_mat_ptr[7] * y[3 + 7 * image_width]) * Q_mat_ptr[3] + (Q_mat_ptr[0] * y[4] + Q_mat_ptr[1] * y[4 + image_width] + Q_mat_ptr[2] * y[4 + 2 * image_width] + Q_mat_ptr[3] * y[4 + 3 * image_width] + Q_mat_ptr[4] * y[4 + 4 * image_width] + Q_mat_ptr[5] * y[4 + 5 * image_width] + Q_mat_ptr[6] * y[4 + 6 * image_width] + Q_mat_ptr[7] * y[4 + 7 * image_width]) * Q_mat_ptr[4] + (Q_mat_ptr[0] * y[5] + Q_mat_ptr[1] * y[5 + image_width] + Q_mat_ptr[2] * y[5 + 2 * image_width] + Q_mat_ptr[3] * y[5 + 3 * image_width] + Q_mat_ptr[4] * y[5 + 4 * image_width] + Q_mat_ptr[5] * y[5 + 5 * image_width] + Q_mat_ptr[6] * y[5 + 6 * image_width] + Q_mat_ptr[7] * y[5 + 7 * image_width]) * Q_mat_ptr[5] + (Q_mat_ptr[0] * y[6] + Q_mat_ptr[1] * y[6 + image_width] + Q_mat_ptr[2] * y[6 + 2 * image_width] + Q_mat_ptr[3] * y[6 + 3 * image_width] + Q_mat_ptr[4] * y[6 + 4 * image_width] + Q_mat_ptr[5] * y[6 + 5 * image_width] + Q_mat_ptr[6] * y[6 + 6 * image_width] + Q_mat_ptr[7] * y[6 + 7 * image_width]) * Q_mat_ptr[6] + (Q_mat_ptr[0] * y[7] + Q_mat_ptr[1] * y[7 + image_width] + Q_mat_ptr[2] * y[7 + 2 * image_width] + Q_mat_ptr[3] * y[7 + 3 * image_width] + Q_mat_ptr[4] * y[7 + 4 * image_width] + Q_mat_ptr[5] * y[7 + 5 * image_width] + Q_mat_ptr[6] * y[7 + 6 * image_width] + Q_mat_ptr[7] * y[7 + 7 * image_width]) * Q_mat_ptr[7];
						S_mat_ptr[1] = (Q_mat_ptr[0] * y[0] + Q_mat_ptr[1] * y[image_width] + Q_mat_ptr[2] * y[2 * image_width] + Q_mat_ptr[3] * y[3 * image_width] + Q_mat_ptr[4] * y[4 * image_width] + Q_mat_ptr[5] * y[5 * image_width] + Q_mat_ptr[6] * y[6 * image_width] + Q_mat_ptr[7] * y[7 * image_width]) * Q_mat_ptr[8] + (Q_mat_ptr[0] * y[1] + Q_mat_ptr[1] * y[1 + image_width] + Q_mat_ptr[2] * y[1 + 2 * image_width] + Q_mat_ptr[3] * y[1 + 3 * image_width] + Q_mat_ptr[4] * y[1 + 4 * image_width] + Q_mat_ptr[5] * y[1 + 5 * image_width] + Q_mat_ptr[6] * y[1 + 6 * image_width] + Q_mat_ptr[7] * y[1 + 7 * image_width]) * Q_mat_ptr[9] + (Q_mat_ptr[0] * y[2] + Q_mat_ptr[1] * y[2 + image_width] + Q_mat_ptr[2] * y[2 + 2 * image_width] + Q_mat_ptr[3] * y[2 + 3 * image_width] + Q_mat_ptr[4] * y[2 + 4 * image_width] + Q_mat_ptr[5] * y[2 + 5 * image_width] + Q_mat_ptr[6] * y[2 + 6 * image_width] + Q_mat_ptr[7] * y[2 + 7 * image_width]) * Q_mat_ptr[10] + (Q_mat_ptr[0] * y[3] + Q_mat_ptr[1] * y[3 + image_width] + Q_mat_ptr[2] * y[3 + 2 * image_width] + Q_mat_ptr[3] * y[3 + 3 * image_width] + Q_mat_ptr[4] * y[3 + 4 * image_width] + Q_mat_ptr[5] * y[3 + 5 * image_width] + Q_mat_ptr[6] * y[3 + 6 * image_width] + Q_mat_ptr[7] * y[3 + 7 * image_width]) * Q_mat_ptr[11] + (Q_mat_ptr[0] * y[4] + Q_mat_ptr[1] * y[4 + image_width] + Q_mat_ptr[2] * y[4 + 2 * image_width] + Q_mat_ptr[3] * y[4 + 3 * image_width] + Q_mat_ptr[4] * y[4 + 4 * image_width] + Q_mat_ptr[5] * y[4 + 5 * image_width] + Q_mat_ptr[6] * y[4 + 6 * image_width] + Q_mat_ptr[7] * y[4 + 7 * image_width]) * Q_mat_ptr[12] + (Q_mat_ptr[0] * y[5] + Q_mat_ptr[1] * y[5 + image_width] + Q_mat_ptr[2] * y[5 + 2 * image_width] + Q_mat_ptr[3] * y[5 + 3 * image_width] + Q_mat_ptr[4] * y[5 + 4 * image_width] + Q_mat_ptr[5] * y[5 + 5 * image_width] + Q_mat_ptr[6] * y[5 + 6 * image_width] + Q_mat_ptr[7] * y[5 + 7 * image_width]) * Q_mat_ptr[13] + (Q_mat_ptr[0] * y[6] + Q_mat_ptr[1] * y[6 + image_width] + Q_mat_ptr[2] * y[6 + 2 * image_width] + Q_mat_ptr[3] * y[6 + 3 * image_width] + Q_mat_ptr[4] * y[6 + 4 * image_width] + Q_mat_ptr[5] * y[6 + 5 * image_width] + Q_mat_ptr[6] * y[6 + 6 * image_width] + Q_mat_ptr[7] * y[6 + 7 * image_width]) * Q_mat_ptr[14] + (Q_mat_ptr[0] * y[7] + Q_mat_ptr[1] * y[7 + image_width] + Q_mat_ptr[2] * y[7 + 2 * image_width] + Q_mat_ptr[3] * y[7 + 3 * image_width] + Q_mat_ptr[4] * y[7 + 4 * image_width] + Q_mat_ptr[5] * y[7 + 5 * image_width] + Q_mat_ptr[6] * y[7 + 6 * image_width] + Q_mat_ptr[7] * y[7 + 7 * image_width]) * Q_mat_ptr[15];
						S_mat_ptr[2] = (Q_mat_ptr[0] * y[0] + Q_mat_ptr[1] * y[image_width] + Q_mat_ptr[2] * y[2 * image_width] + Q_mat_ptr[3] * y[3 * image_width] + Q_mat_ptr[4] * y[4 * image_width] + Q_mat_ptr[5] * y[5 * image_width] + Q_mat_ptr[6] * y[6 * image_width] + Q_mat_ptr[7] * y[7 * image_width]) * Q_mat_ptr[16] + (Q_mat_ptr[0] * y[1] + Q_mat_ptr[1] * y[1 + image_width] + Q_mat_ptr[2] * y[1 + 2 * image_width] + Q_mat_ptr[3] * y[1 + 3 * image_width] + Q_mat_ptr[4] * y[1 + 4 * image_width] + Q_mat_ptr[5] * y[1 + 5 * image_width] + Q_mat_ptr[6] * y[1 + 6 * image_width] + Q_mat_ptr[7] * y[1 + 7 * image_width]) * Q_mat_ptr[17] + (Q_mat_ptr[0] * y[2] + Q_mat_ptr[1] * y[2 + image_width] + Q_mat_ptr[2] * y[2 + 2 * image_width] + Q_mat_ptr[3] * y[2 + 3 * image_width] + Q_mat_ptr[4] * y[2 + 4 * image_width] + Q_mat_ptr[5] * y[2 + 5 * image_width] + Q_mat_ptr[6] * y[2 + 6 * image_width] + Q_mat_ptr[7] * y[2 + 7 * image_width]) * Q_mat_ptr[18] + (Q_mat_ptr[0] * y[3] + Q_mat_ptr[1] * y[3 + image_width] + Q_mat_ptr[2] * y[3 + 2 * image_width] + Q_mat_ptr[3] * y[3 + 3 * image_width] + Q_mat_ptr[4] * y[3 + 4 * image_width] + Q_mat_ptr[5] * y[3 + 5 * image_width] + Q_mat_ptr[6] * y[3 + 6 * image_width] + Q_mat_ptr[7] * y[3 + 7 * image_width]) * Q_mat_ptr[19] + (Q_mat_ptr[0] * y[4] + Q_mat_ptr[1] * y[4 + image_width] + Q_mat_ptr[2] * y[4 + 2 * image_width] + Q_mat_ptr[3] * y[4 + 3 * image_width] + Q_mat_ptr[4] * y[4 + 4 * image_width] + Q_mat_ptr[5] * y[4 + 5 * image_width] + Q_mat_ptr[6] * y[4 + 6 * image_width] + Q_mat_ptr[7] * y[4 + 7 * image_width]) * Q_mat_ptr[20] + (Q_mat_ptr[0] * y[5] + Q_mat_ptr[1] * y[5 + image_width] + Q_mat_ptr[2] * y[5 + 2 * image_width] + Q_mat_ptr[3] * y[5 + 3 * image_width] + Q_mat_ptr[4] * y[5 + 4 * image_width] + Q_mat_ptr[5] * y[5 + 5 * image_width] + Q_mat_ptr[6] * y[5 + 6 * image_width] + Q_mat_ptr[7] * y[5 + 7 * image_width]) * Q_mat_ptr[21] + (Q_mat_ptr[0] * y[6] + Q_mat_ptr[1] * y[6 + image_width] + Q_mat_ptr[2] * y[6 + 2 * image_width] + Q_mat_ptr[3] * y[6 + 3 * image_width] + Q_mat_ptr[4] * y[6 + 4 * image_width] + Q_mat_ptr[5] * y[6 + 5 * image_width] + Q_mat_ptr[6] * y[6 + 6 * image_width] + Q_mat_ptr[7] * y[6 + 7 * image_width]) * Q_mat_ptr[22] + (Q_mat_ptr[0] * y[7] + Q_mat_ptr[1] * y[7 + image_width] + Q_mat_ptr[2] * y[7 + 2 * image_width] + Q_mat_ptr[3] * y[7 + 3 * image_width] + Q_mat_ptr[4] * y[7 + 4 * image_width] + Q_mat_ptr[5] * y[7 + 5 * image_width] + Q_mat_ptr[6] * y[7 + 6 * image_width] + Q_mat_ptr[7] * y[7 + 7 * image_width]) * Q_mat_ptr[23];
						S_mat_ptr[3] = Q_mat_ptr[0] * y[3] + Q_mat_ptr[1] * y[3 + image_width] + Q_mat_ptr[2] * y[3 + 2 * image_width] + Q_mat_ptr[3] * y[3 + 3 * image_width] + Q_mat_ptr[4] * y[3 + 4 * image_width] + Q_mat_ptr[5] * y[3 + 5 * image_width] + Q_mat_ptr[6] * y[3 + 6 * image_width] + Q_mat_ptr[7] * y[3 + 7 * image_width];
						S_mat_ptr[4] = (Q_mat_ptr[8] * y[0] + Q_mat_ptr[9] * y[image_width] + Q_mat_ptr[10] * y[2 * image_width] + Q_mat_ptr[11] * y[3 * image_width] + Q_mat_ptr[12] * y[4 * image_width] + Q_mat_ptr[13] * y[5 * image_width] + Q_mat_ptr[14] * y[6 * image_width] + Q_mat_ptr[15] * y[7 * image_width]) * Q_mat_ptr[0] + (Q_mat_ptr[8] * y[1] + Q_mat_ptr[9] * y[1 + image_width] + Q_mat_ptr[10] * y[1 + 2 * image_width] + Q_mat_ptr[11] * y[1 + 3 * image_width] + Q_mat_ptr[12] * y[1 + 4 * image_width] + Q_mat_ptr[13] * y[1 + 5 * image_width] + Q_mat_ptr[14] * y[1 + 6 * image_width] + Q_mat_ptr[15] * y[1 + 7 * image_width]) * Q_mat_ptr[1] + (Q_mat_ptr[8] * y[2] + Q_mat_ptr[9] * y[2 + image_width] + Q_mat_ptr[10] * y[2 + 2 * image_width] + Q_mat_ptr[11] * y[2 + 3 * image_width] + Q_mat_ptr[12] * y[2 + 4 * image_width] + Q_mat_ptr[13] * y[2 + 5 * image_width] + Q_mat_ptr[14] * y[2 + 6 * image_width] + Q_mat_ptr[15] * y[2 + 7 * image_width]) * Q_mat_ptr[2] + (Q_mat_ptr[8] * y[3] + Q_mat_ptr[9] * y[3 + image_width] + Q_mat_ptr[10] * y[3 + 2 * image_width] + Q_mat_ptr[11] * y[3 + 3 * image_width] + Q_mat_ptr[12] * y[3 + 4 * image_width] + Q_mat_ptr[13] * y[3 + 5 * image_width] + Q_mat_ptr[14] * y[3 + 6 * image_width] + Q_mat_ptr[15] * y[3 + 7 * image_width]) * Q_mat_ptr[3] + (Q_mat_ptr[8] * y[4] + Q_mat_ptr[9] * y[4 + image_width] + Q_mat_ptr[10] * y[4 + 2 * image_width] + Q_mat_ptr[11] * y[4 + 3 * image_width] + Q_mat_ptr[12] * y[4 + 4 * image_width] + Q_mat_ptr[13] * y[4 + 5 * image_width] + Q_mat_ptr[14] * y[4 + 6 * image_width] + Q_mat_ptr[15] * y[4 + 7 * image_width]) * Q_mat_ptr[4] + (Q_mat_ptr[8] * y[5] + Q_mat_ptr[9] * y[5 + image_width] + Q_mat_ptr[10] * y[5 + 2 * image_width] + Q_mat_ptr[11] * y[5 + 3 * image_width] + Q_mat_ptr[12] * y[5 + 4 * image_width] + Q_mat_ptr[13] * y[5 + 5 * image_width] + Q_mat_ptr[14] * y[5 + 6 * image_width] + Q_mat_ptr[15] * y[5 + 7 * image_width]) * Q_mat_ptr[5] + (Q_mat_ptr[8] * y[6] + Q_mat_ptr[9] * y[6 + image_width] + Q_mat_ptr[10] * y[6 + 2 * image_width] + Q_mat_ptr[11] * y[6 + 3 * image_width] + Q_mat_ptr[12] * y[6 + 4 * image_width] + Q_mat_ptr[13] * y[6 + 5 * image_width] + Q_mat_ptr[14] * y[6 + 6 * image_width] + Q_mat_ptr[15] * y[6 + 7 * image_width]) * Q_mat_ptr[6] + (Q_mat_ptr[8] * y[7] + Q_mat_ptr[9] * y[7 + image_width] + Q_mat_ptr[10] * y[7 + 2 * image_width] + Q_mat_ptr[11] * y[7 + 3 * image_width] + Q_mat_ptr[12] * y[7 + 4 * image_width] + Q_mat_ptr[13] * y[7 + 5 * image_width] + Q_mat_ptr[14] * y[7 + 6 * image_width] + Q_mat_ptr[15] * y[7 + 7 * image_width]) * Q_mat_ptr[7];
						S_mat_ptr[5] = (Q_mat_ptr[8] * y[0] + Q_mat_ptr[9] * y[image_width] + Q_mat_ptr[10] * y[2 * image_width] + Q_mat_ptr[11] * y[3 * image_width] + Q_mat_ptr[12] * y[4 * image_width] + Q_mat_ptr[13] * y[5 * image_width] + Q_mat_ptr[14] * y[6 * image_width] + Q_mat_ptr[15] * y[7 * image_width]) * Q_mat_ptr[8] + (Q_mat_ptr[8] * y[1] + Q_mat_ptr[9] * y[1 + image_width] + Q_mat_ptr[10] * y[1 + 2 * image_width] + Q_mat_ptr[11] * y[1 + 3 * image_width] + Q_mat_ptr[12] * y[1 + 4 * image_width] + Q_mat_ptr[13] * y[1 + 5 * image_width] + Q_mat_ptr[14] * y[1 + 6 * image_width] + Q_mat_ptr[15] * y[1 + 7 * image_width]) * Q_mat_ptr[9] + (Q_mat_ptr[8] * y[2] + Q_mat_ptr[9] * y[2 + image_width] + Q_mat_ptr[10] * y[2 + 2 * image_width] + Q_mat_ptr[11] * y[2 + 3 * image_width] + Q_mat_ptr[12] * y[2 + 4 * image_width] + Q_mat_ptr[13] * y[2 + 5 * image_width] + Q_mat_ptr[14] * y[2 + 6 * image_width] + Q_mat_ptr[15] * y[2 + 7 * image_width]) * Q_mat_ptr[10] + (Q_mat_ptr[8] * y[3] + Q_mat_ptr[9] * y[3 + image_width] + Q_mat_ptr[10] * y[3 + 2 * image_width] + Q_mat_ptr[11] * y[3 + 3 * image_width] + Q_mat_ptr[12] * y[3 + 4 * image_width] + Q_mat_ptr[13] * y[3 + 5 * image_width] + Q_mat_ptr[14] * y[3 + 6 * image_width] + Q_mat_ptr[15] * y[3 + 7 * image_width]) * Q_mat_ptr[11] + (Q_mat_ptr[8] * y[4] + Q_mat_ptr[9] * y[4 + image_width] + Q_mat_ptr[10] * y[4 + 2 * image_width] + Q_mat_ptr[11] * y[4 + 3 * image_width] + Q_mat_ptr[12] * y[4 + 4 * image_width] + Q_mat_ptr[13] * y[4 + 5 * image_width] + Q_mat_ptr[14] * y[4 + 6 * image_width] + Q_mat_ptr[15] * y[4 + 7 * image_width]) * Q_mat_ptr[12] + (Q_mat_ptr[8] * y[5] + Q_mat_ptr[9] * y[5 + image_width] + Q_mat_ptr[10] * y[5 + 2 * image_width] + Q_mat_ptr[11] * y[5 + 3 * image_width] + Q_mat_ptr[12] * y[5 + 4 * image_width] + Q_mat_ptr[13] * y[5 + 5 * image_width] + Q_mat_ptr[14] * y[5 + 6 * image_width] + Q_mat_ptr[15] * y[5 + 7 * image_width]) * Q_mat_ptr[13] + (Q_mat_ptr[8] * y[6] + Q_mat_ptr[9] * y[6 + image_width] + Q_mat_ptr[10] * y[6 + 2 * image_width] + Q_mat_ptr[11] * y[6 + 3 * image_width] + Q_mat_ptr[12] * y[6 + 4 * image_width] + Q_mat_ptr[13] * y[6 + 5 * image_width] + Q_mat_ptr[14] * y[6 + 6 * image_width] + Q_mat_ptr[15] * y[6 + 7 * image_width]) * Q_mat_ptr[14] + (Q_mat_ptr[8] * y[7] + Q_mat_ptr[9] * y[7 + image_width] + Q_mat_ptr[10] * y[7 + 2 * image_width] + Q_mat_ptr[11] * y[7 + 3 * image_width] + Q_mat_ptr[12] * y[7 + 4 * image_width] + Q_mat_ptr[13] * y[7 + 5 * image_width] + Q_mat_ptr[14] * y[7 + 6 * image_width] + Q_mat_ptr[15] * y[7 + 7 * image_width]) * Q_mat_ptr[15];
						S_mat_ptr[6] = (Q_mat_ptr[8] * y[0] + Q_mat_ptr[9] * y[image_width] + Q_mat_ptr[10] * y[2 * image_width] + Q_mat_ptr[11] * y[3 * image_width] + Q_mat_ptr[12] * y[4 * image_width] + Q_mat_ptr[13] * y[5 * image_width] + Q_mat_ptr[14] * y[6 * image_width] + Q_mat_ptr[15] * y[7 * image_width]) * Q_mat_ptr[16] + (Q_mat_ptr[8] * y[1] + Q_mat_ptr[9] * y[1 + image_width] + Q_mat_ptr[10] * y[1 + 2 * image_width] + Q_mat_ptr[11] * y[1 + 3 * image_width] + Q_mat_ptr[12] * y[1 + 4 * image_width] + Q_mat_ptr[13] * y[1 + 5 * image_width] + Q_mat_ptr[14] * y[1 + 6 * image_width] + Q_mat_ptr[15] * y[1 + 7 * image_width]) * Q_mat_ptr[17] + (Q_mat_ptr[8] * y[2] + Q_mat_ptr[9] * y[2 + image_width] + Q_mat_ptr[10] * y[2 + 2 * image_width] + Q_mat_ptr[11] * y[2 + 3 * image_width] + Q_mat_ptr[12] * y[2 + 4 * image_width] + Q_mat_ptr[13] * y[2 + 5 * image_width] + Q_mat_ptr[14] * y[2 + 6 * image_width] + Q_mat_ptr[15] * y[2 + 7 * image_width]) * Q_mat_ptr[18] + (Q_mat_ptr[8] * y[3] + Q_mat_ptr[9] * y[3 + image_width] + Q_mat_ptr[10] * y[3 + 2 * image_width] + Q_mat_ptr[11] * y[3 + 3 * image_width] + Q_mat_ptr[12] * y[3 + 4 * image_width] + Q_mat_ptr[13] * y[3 + 5 * image_width] + Q_mat_ptr[14] * y[3 + 6 * image_width] + Q_mat_ptr[15] * y[3 + 7 * image_width]) * Q_mat_ptr[19] + (Q_mat_ptr[8] * y[4] + Q_mat_ptr[9] * y[4 + image_width] + Q_mat_ptr[10] * y[4 + 2 * image_width] + Q_mat_ptr[11] * y[4 + 3 * image_width] + Q_mat_ptr[12] * y[4 + 4 * image_width] + Q_mat_ptr[13] * y[4 + 5 * image_width] + Q_mat_ptr[14] * y[4 + 6 * image_width] + Q_mat_ptr[15] * y[4 + 7 * image_width]) * Q_mat_ptr[20] + (Q_mat_ptr[8] * y[5] + Q_mat_ptr[9] * y[5 + image_width] + Q_mat_ptr[10] * y[5 + 2 * image_width] + Q_mat_ptr[11] * y[5 + 3 * image_width] + Q_mat_ptr[12] * y[5 + 4 * image_width] + Q_mat_ptr[13] * y[5 + 5 * image_width] + Q_mat_ptr[14] * y[5 + 6 * image_width] + Q_mat_ptr[15] * y[5 + 7 * image_width]) * Q_mat_ptr[21] + (Q_mat_ptr[8] * y[6] + Q_mat_ptr[9] * y[6 + image_width] + Q_mat_ptr[10] * y[6 + 2 * image_width] + Q_mat_ptr[11] * y[6 + 3 * image_width] + Q_mat_ptr[12] * y[6 + 4 * image_width] + Q_mat_ptr[13] * y[6 + 5 * image_width] + Q_mat_ptr[14] * y[6 + 6 * image_width] + Q_mat_ptr[15] * y[6 + 7 * image_width]) * Q_mat_ptr[22] + (Q_mat_ptr[8] * y[7] + Q_mat_ptr[9] * y[7 + image_width] + Q_mat_ptr[10] * y[7 + 2 * image_width] + Q_mat_ptr[11] * y[7 + 3 * image_width] + Q_mat_ptr[12] * y[7 + 4 * image_width] + Q_mat_ptr[13] * y[7 + 5 * image_width] + Q_mat_ptr[14] * y[7 + 6 * image_width] + Q_mat_ptr[15] * y[7 + 7 * image_width]) * Q_mat_ptr[23];
						S_mat_ptr[7] = Q_mat_ptr[8] * y[3] + Q_mat_ptr[9] * y[3 + image_width] + Q_mat_ptr[10] * y[3 + 2 * image_width] + Q_mat_ptr[11] * y[3 + 3 * image_width] + Q_mat_ptr[12] * y[3 + 4 * image_width] + Q_mat_ptr[13] * y[3 + 5 * image_width] + Q_mat_ptr[14] * y[3 + 6 * image_width] + Q_mat_ptr[15] * y[3 + 7 * image_width];
						S_mat_ptr[8] = (Q_mat_ptr[16] * y[0] + Q_mat_ptr[17] * y[image_width] + Q_mat_ptr[18] * y[2 * image_width] + Q_mat_ptr[19] * y[3 * image_width] + Q_mat_ptr[20] * y[4 * image_width] + Q_mat_ptr[21] * y[5 * image_width] + Q_mat_ptr[22] * y[6 * image_width] + Q_mat_ptr[23] * y[7 * image_width]) * Q_mat_ptr[0] + (Q_mat_ptr[16] * y[1] + Q_mat_ptr[17] * y[1 + image_width] + Q_mat_ptr[18] * y[1 + 2 * image_width] + Q_mat_ptr[19] * y[1 + 3 * image_width] + Q_mat_ptr[20] * y[1 + 4 * image_width] + Q_mat_ptr[21] * y[1 + 5 * image_width] + Q_mat_ptr[22] * y[1 + 6 * image_width] + Q_mat_ptr[23] * y[1 + 7 * image_width]) * Q_mat_ptr[1] + (Q_mat_ptr[16] * y[2] + Q_mat_ptr[17] * y[2 + image_width] + Q_mat_ptr[18] * y[2 + 2 * image_width] + Q_mat_ptr[19] * y[2 + 3 * image_width] + Q_mat_ptr[20] * y[2 + 4 * image_width] + Q_mat_ptr[21] * y[2 + 5 * image_width] + Q_mat_ptr[22] * y[2 + 6 * image_width] + Q_mat_ptr[23] * y[2 + 7 * image_width]) * Q_mat_ptr[2] + (Q_mat_ptr[16] * y[3] + Q_mat_ptr[17] * y[3 + image_width] + Q_mat_ptr[18] * y[3 + 2 * image_width] + Q_mat_ptr[19] * y[3 + 3 * image_width] + Q_mat_ptr[20] * y[3 + 4 * image_width] + Q_mat_ptr[21] * y[3 + 5 * image_width] + Q_mat_ptr[22] * y[3 + 6 * image_width] + Q_mat_ptr[23] * y[3 + 7 * image_width]) * Q_mat_ptr[3] + (Q_mat_ptr[16] * y[4] + Q_mat_ptr[17] * y[4 + image_width] + Q_mat_ptr[18] * y[4 + 2 * image_width] + Q_mat_ptr[19] * y[4 + 3 * image_width] + Q_mat_ptr[20] * y[4 + 4 * image_width] + Q_mat_ptr[21] * y[4 + 5 * image_width] + Q_mat_ptr[22] * y[4 + 6 * image_width] + Q_mat_ptr[23] * y[4 + 7 * image_width]) * Q_mat_ptr[4] + (Q_mat_ptr[16] * y[5] + Q_mat_ptr[17] * y[5 + image_width] + Q_mat_ptr[18] * y[5 + 2 * image_width] + Q_mat_ptr[19] * y[5 + 3 * image_width] + Q_mat_ptr[20] * y[5 + 4 * image_width] + Q_mat_ptr[21] * y[5 + 5 * image_width] + Q_mat_ptr[22] * y[5 + 6 * image_width] + Q_mat_ptr[23] * y[5 + 7 * image_width]) * Q_mat_ptr[5] + (Q_mat_ptr[16] * y[6] + Q_mat_ptr[17] * y[6 + image_width] + Q_mat_ptr[18] * y[6 + 2 * image_width] + Q_mat_ptr[19] * y[6 + 3 * image_width] + Q_mat_ptr[20] * y[6 + 4 * image_width] + Q_mat_ptr[21] * y[6 + 5 * image_width] + Q_mat_ptr[22] * y[6 + 6 * image_width] + Q_mat_ptr[23] * y[6 + 7 * image_width]) * Q_mat_ptr[6] + (Q_mat_ptr[16] * y[7] + Q_mat_ptr[17] * y[7 + image_width] + Q_mat_ptr[18] * y[7 + 2 * image_width] + Q_mat_ptr[19] * y[7 + 3 * image_width] + Q_mat_ptr[20] * y[7 + 4 * image_width] + Q_mat_ptr[21] * y[7 + 5 * image_width] + Q_mat_ptr[22] * y[7 + 6 * image_width] + Q_mat_ptr[23] * y[7 + 7 * image_width]) * Q_mat_ptr[7];
						S_mat_ptr[9] = (Q_mat_ptr[16] * y[0] + Q_mat_ptr[17] * y[image_width] + Q_mat_ptr[18] * y[2 * image_width] + Q_mat_ptr[19] * y[3 * image_width] + Q_mat_ptr[20] * y[4 * image_width] + Q_mat_ptr[21] * y[5 * image_width] + Q_mat_ptr[22] * y[6 * image_width] + Q_mat_ptr[23] * y[7 * image_width]) * Q_mat_ptr[8] + (Q_mat_ptr[16] * y[1] + Q_mat_ptr[17] * y[1 + image_width] + Q_mat_ptr[18] * y[1 + 2 * image_width] + Q_mat_ptr[19] * y[1 + 3 * image_width] + Q_mat_ptr[20] * y[1 + 4 * image_width] + Q_mat_ptr[21] * y[1 + 5 * image_width] + Q_mat_ptr[22] * y[1 + 6 * image_width] + Q_mat_ptr[23] * y[1 + 7 * image_width]) * Q_mat_ptr[9] + (Q_mat_ptr[16] * y[2] + Q_mat_ptr[17] * y[2 + image_width] + Q_mat_ptr[18] * y[2 + 2 * image_width] + Q_mat_ptr[19] * y[2 + 3 * image_width] + Q_mat_ptr[20] * y[2 + 4 * image_width] + Q_mat_ptr[21] * y[2 + 5 * image_width] + Q_mat_ptr[22] * y[2 + 6 * image_width] + Q_mat_ptr[23] * y[2 + 7 * image_width]) * Q_mat_ptr[10] + (Q_mat_ptr[16] * y[3] + Q_mat_ptr[17] * y[3 + image_width] + Q_mat_ptr[18] * y[3 + 2 * image_width] + Q_mat_ptr[19] * y[3 + 3 * image_width] + Q_mat_ptr[20] * y[3 + 4 * image_width] + Q_mat_ptr[21] * y[3 + 5 * image_width] + Q_mat_ptr[22] * y[3 + 6 * image_width] + Q_mat_ptr[23] * y[3 + 7 * image_width]) * Q_mat_ptr[11] + (Q_mat_ptr[16] * y[4] + Q_mat_ptr[17] * y[4 + image_width] + Q_mat_ptr[18] * y[4 + 2 * image_width] + Q_mat_ptr[19] * y[4 + 3 * image_width] + Q_mat_ptr[20] * y[4 + 4 * image_width] + Q_mat_ptr[21] * y[4 + 5 * image_width] + Q_mat_ptr[22] * y[4 + 6 * image_width] + Q_mat_ptr[23] * y[4 + 7 * image_width]) * Q_mat_ptr[12] + (Q_mat_ptr[16] * y[5] + Q_mat_ptr[17] * y[5 + image_width] + Q_mat_ptr[18] * y[5 + 2 * image_width] + Q_mat_ptr[19] * y[5 + 3 * image_width] + Q_mat_ptr[20] * y[5 + 4 * image_width] + Q_mat_ptr[21] * y[5 + 5 * image_width] + Q_mat_ptr[22] * y[5 + 6 * image_width] + Q_mat_ptr[23] * y[5 + 7 * image_width]) * Q_mat_ptr[13] + (Q_mat_ptr[16] * y[6] + Q_mat_ptr[17] * y[6 + image_width] + Q_mat_ptr[18] * y[6 + 2 * image_width] + Q_mat_ptr[19] * y[6 + 3 * image_width] + Q_mat_ptr[20] * y[6 + 4 * image_width] + Q_mat_ptr[21] * y[6 + 5 * image_width] + Q_mat_ptr[22] * y[6 + 6 * image_width] + Q_mat_ptr[23] * y[6 + 7 * image_width]) * Q_mat_ptr[14] + (Q_mat_ptr[16] * y[7] + Q_mat_ptr[17] * y[7 + image_width] + Q_mat_ptr[18] * y[7 + 2 * image_width] + Q_mat_ptr[19] * y[7 + 3 * image_width] + Q_mat_ptr[20] * y[7 + 4 * image_width] + Q_mat_ptr[21] * y[7 + 5 * image_width] + Q_mat_ptr[22] * y[7 + 6 * image_width] + Q_mat_ptr[23] * y[7 + 7 * image_width]) * Q_mat_ptr[15];
						S_mat_ptr[10] = (Q_mat_ptr[16] * y[0] + Q_mat_ptr[17] * y[image_width] + Q_mat_ptr[18] * y[2 * image_width] + Q_mat_ptr[19] * y[3 * image_width] + Q_mat_ptr[20] * y[4 * image_width] + Q_mat_ptr[21] * y[5 * image_width] + Q_mat_ptr[22] * y[6 * image_width] + Q_mat_ptr[23] * y[7 * image_width]) * Q_mat_ptr[16] + (Q_mat_ptr[16] * y[1] + Q_mat_ptr[17] * y[1 + image_width] + Q_mat_ptr[18] * y[1 + 2 * image_width] + Q_mat_ptr[19] * y[1 + 3 * image_width] + Q_mat_ptr[20] * y[1 + 4 * image_width] + Q_mat_ptr[21] * y[1 + 5 * image_width] + Q_mat_ptr[22] * y[1 + 6 * image_width] + Q_mat_ptr[23] * y[1 + 7 * image_width]) * Q_mat_ptr[17] + (Q_mat_ptr[16] * y[2] + Q_mat_ptr[17] * y[2 + image_width] + Q_mat_ptr[18] * y[2 + 2 * image_width] + Q_mat_ptr[19] * y[2 + 3 * image_width] + Q_mat_ptr[20] * y[2 + 4 * image_width] + Q_mat_ptr[21] * y[2 + 5 * image_width] + Q_mat_ptr[22] * y[2 + 6 * image_width] + Q_mat_ptr[23] * y[2 + 7 * image_width]) * Q_mat_ptr[18] + (Q_mat_ptr[16] * y[3] + Q_mat_ptr[17] * y[3 + image_width] + Q_mat_ptr[18] * y[3 + 2 * image_width] + Q_mat_ptr[19] * y[3 + 3 * image_width] + Q_mat_ptr[20] * y[3 + 4 * image_width] + Q_mat_ptr[21] * y[3 + 5 * image_width] + Q_mat_ptr[22] * y[3 + 6 * image_width] + Q_mat_ptr[23] * y[3 + 7 * image_width]) * Q_mat_ptr[19] + (Q_mat_ptr[16] * y[4] + Q_mat_ptr[17] * y[4 + image_width] + Q_mat_ptr[18] * y[4 + 2 * image_width] + Q_mat_ptr[19] * y[4 + 3 * image_width] + Q_mat_ptr[20] * y[4 + 4 * image_width] + Q_mat_ptr[21] * y[4 + 5 * image_width] + Q_mat_ptr[22] * y[4 + 6 * image_width] + Q_mat_ptr[23] * y[4 + 7 * image_width]) * Q_mat_ptr[20] + (Q_mat_ptr[16] * y[5] + Q_mat_ptr[17] * y[5 + image_width] + Q_mat_ptr[18] * y[5 + 2 * image_width] + Q_mat_ptr[19] * y[5 + 3 * image_width] + Q_mat_ptr[20] * y[5 + 4 * image_width] + Q_mat_ptr[21] * y[5 + 5 * image_width] + Q_mat_ptr[22] * y[5 + 6 * image_width] + Q_mat_ptr[23] * y[5 + 7 * image_width]) * Q_mat_ptr[21] + (Q_mat_ptr[16] * y[6] + Q_mat_ptr[17] * y[6 + image_width] + Q_mat_ptr[18] * y[6 + 2 * image_width] + Q_mat_ptr[19] * y[6 + 3 * image_width] + Q_mat_ptr[20] * y[6 + 4 * image_width] + Q_mat_ptr[21] * y[6 + 5 * image_width] + Q_mat_ptr[22] * y[6 + 6 * image_width] + Q_mat_ptr[23] * y[6 + 7 * image_width]) * Q_mat_ptr[22] + (Q_mat_ptr[16] * y[7] + Q_mat_ptr[17] * y[7 + image_width] + Q_mat_ptr[18] * y[7 + 2 * image_width] + Q_mat_ptr[19] * y[7 + 3 * image_width] + Q_mat_ptr[20] * y[7 + 4 * image_width] + Q_mat_ptr[21] * y[7 + 5 * image_width] + Q_mat_ptr[22] * y[7 + 6 * image_width] + Q_mat_ptr[23] * y[7 + 7 * image_width]) * Q_mat_ptr[23];
						S_mat_ptr[11] = Q_mat_ptr[16] * y[3] + Q_mat_ptr[17] * y[3 + image_width] + Q_mat_ptr[18] * y[3 + 2 * image_width] + Q_mat_ptr[19] * y[3 + 3 * image_width] + Q_mat_ptr[20] * y[3 + 4 * image_width] + Q_mat_ptr[21] * y[3 + 5 * image_width] + Q_mat_ptr[22] * y[3 + 6 * image_width] + Q_mat_ptr[23] * y[3 + 7 * image_width];
						S_mat_ptr[12] = y[3 * image_width] * Q_mat_ptr[0] + y[1 + 3 * image_width] * Q_mat_ptr[1] + y[2 + 3 * image_width] * Q_mat_ptr[2] + Q_mat_ptr[3] * y[3 + 3 * image_width] + y[4 + 3 * image_width] * Q_mat_ptr[4] + y[5 + 3 * image_width] * Q_mat_ptr[5] + y[6 + 3 * image_width] * Q_mat_ptr[6] + y[7 + 3 * image_width] * Q_mat_ptr[7];
						S_mat_ptr[13] = y[3 * image_width] * Q_mat_ptr[8] + y[1 + 3 * image_width] * Q_mat_ptr[9] + y[2 + 3 * image_width] * Q_mat_ptr[10] + Q_mat_ptr[11] * y[3 + 3 * image_width] + y[4 + 3 * image_width] * Q_mat_ptr[12] + y[5 + 3 * image_width] * Q_mat_ptr[13] + y[6 + 3 * image_width] * Q_mat_ptr[14] + y[7 + 3 * image_width] * Q_mat_ptr[15];
						S_mat_ptr[14] = y[3 * image_width] * Q_mat_ptr[16] + y[1 + 3 * image_width] * Q_mat_ptr[17] + y[2 + 3 * image_width] * Q_mat_ptr[18] + Q_mat_ptr[19] * y[3 + 3 * image_width] + y[4 + 3 * image_width] * Q_mat_ptr[20] + y[5 + 3 * image_width] * Q_mat_ptr[21] + y[6 + 3 * image_width] * Q_mat_ptr[22] + y[7 + 3 * image_width] * Q_mat_ptr[23];
						S_mat_ptr[15] = y[3 + 3 * image_width];
					}
					CV_MAT_ELEM(*lut_valid_sign_mat,uchar,index_y - aoi_y_min,index_x - aoi_x_min) =1;		
				}
				CV_MAT_ELEM(*g_mat,double,i,j) = 
					delta_x*delta_x*delta_x * (delta_y*delta_y*delta_y * S_mat_ptr[0] + delta_y*delta_y * S_mat_ptr[4] + delta_y * S_mat_ptr[8] + S_mat_ptr[12] ) +
					delta_x*delta_x * (delta_y*delta_y*delta_y * S_mat_ptr[1] + delta_y*delta_y * S_mat_ptr[5] + delta_y * S_mat_ptr[9] + S_mat_ptr[13] ) +
					delta_x  * (delta_y*delta_y*delta_y * S_mat_ptr[2] + delta_y*delta_y * S_mat_ptr[6] + delta_y * S_mat_ptr[10] + S_mat_ptr[14] ) +
					(delta_y*delta_y*delta_y * S_mat_ptr[3] + delta_y*delta_y * S_mat_ptr[7] + delta_y * S_mat_ptr[11] + S_mat_ptr[15] );
			}
		}
	}
	else
	{
		for (int i =0;i<template_height;i++)
		{
			for (int j =0;j<template_width;j++)
			{
				//第一种
				double pos_x =reference_point_x + j - template_half_width + u + ux * (j - template_half_width) + uy * (i - template_half_height);
				double pos_y =reference_point_y + i - template_half_height + v + vx * (j - template_half_width) + vy * (i - template_half_height);

				if (pos_x<aoi_x_min || pos_x>=aoi_x_max||
					pos_y<aoi_y_min || pos_y>=aoi_y_max)//右侧为开区间
				{
					return false;
				}
				int index_x = cvFloor(pos_x);
				int index_y = cvFloor(pos_y);

				double delta_x = pos_x - index_x;
				double delta_y = pos_y - index_y;
				uchar sign = CV_MAT_ELEM(*lut_valid_sign_mat,uchar,index_y - aoi_y_min,index_x - aoi_x_min);
				double* S_mat_ptr = lut_mat->data.db+ ((index_y - aoi_y_min) * aoi_width+index_x - aoi_x_min) * lut_element_size_;
				if (sign!=1)
				{
					uchar* y;
					if (Q_mat_type==1)
					{
						y = image->data.ptr + (index_y-2)*image_width+index_x-2;
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

					} 
					else
					{
						y = image->data.ptr + (index_y-3)*image_width+index_x-3;
						S_mat_ptr[0] = (Q_mat_ptr[0] * y[0] + Q_mat_ptr[1] * y[image_width] + Q_mat_ptr[2] * y[2 * image_width] + Q_mat_ptr[3] * y[3 * image_width] + Q_mat_ptr[4] * y[4 * image_width] + Q_mat_ptr[5] * y[5 * image_width] + Q_mat_ptr[6] * y[6 * image_width] + Q_mat_ptr[7] * y[7 * image_width]) * Q_mat_ptr[0] + (Q_mat_ptr[0] * y[1] + Q_mat_ptr[1] * y[1 + image_width] + Q_mat_ptr[2] * y[1 + 2 * image_width] + Q_mat_ptr[3] * y[1 + 3 * image_width] + Q_mat_ptr[4] * y[1 + 4 * image_width] + Q_mat_ptr[5] * y[1 + 5 * image_width] + Q_mat_ptr[6] * y[1 + 6 * image_width] + Q_mat_ptr[7] * y[1 + 7 * image_width]) * Q_mat_ptr[1] + (Q_mat_ptr[0] * y[2] + Q_mat_ptr[1] * y[2 + image_width] + Q_mat_ptr[2] * y[2 + 2 * image_width] + Q_mat_ptr[3] * y[2 + 3 * image_width] + Q_mat_ptr[4] * y[2 + 4 * image_width] + Q_mat_ptr[5] * y[2 + 5 * image_width] + Q_mat_ptr[6] * y[2 + 6 * image_width] + Q_mat_ptr[7] * y[2 + 7 * image_width]) * Q_mat_ptr[2] + (Q_mat_ptr[0] * y[3] + Q_mat_ptr[1] * y[3 + image_width] + Q_mat_ptr[2] * y[3 + 2 * image_width] + Q_mat_ptr[3] * y[3 + 3 * image_width] + Q_mat_ptr[4] * y[3 + 4 * image_width] + Q_mat_ptr[5] * y[3 + 5 * image_width] + Q_mat_ptr[6] * y[3 + 6 * image_width] + Q_mat_ptr[7] * y[3 + 7 * image_width]) * Q_mat_ptr[3] + (Q_mat_ptr[0] * y[4] + Q_mat_ptr[1] * y[4 + image_width] + Q_mat_ptr[2] * y[4 + 2 * image_width] + Q_mat_ptr[3] * y[4 + 3 * image_width] + Q_mat_ptr[4] * y[4 + 4 * image_width] + Q_mat_ptr[5] * y[4 + 5 * image_width] + Q_mat_ptr[6] * y[4 + 6 * image_width] + Q_mat_ptr[7] * y[4 + 7 * image_width]) * Q_mat_ptr[4] + (Q_mat_ptr[0] * y[5] + Q_mat_ptr[1] * y[5 + image_width] + Q_mat_ptr[2] * y[5 + 2 * image_width] + Q_mat_ptr[3] * y[5 + 3 * image_width] + Q_mat_ptr[4] * y[5 + 4 * image_width] + Q_mat_ptr[5] * y[5 + 5 * image_width] + Q_mat_ptr[6] * y[5 + 6 * image_width] + Q_mat_ptr[7] * y[5 + 7 * image_width]) * Q_mat_ptr[5] + (Q_mat_ptr[0] * y[6] + Q_mat_ptr[1] * y[6 + image_width] + Q_mat_ptr[2] * y[6 + 2 * image_width] + Q_mat_ptr[3] * y[6 + 3 * image_width] + Q_mat_ptr[4] * y[6 + 4 * image_width] + Q_mat_ptr[5] * y[6 + 5 * image_width] + Q_mat_ptr[6] * y[6 + 6 * image_width] + Q_mat_ptr[7] * y[6 + 7 * image_width]) * Q_mat_ptr[6] + (Q_mat_ptr[0] * y[7] + Q_mat_ptr[1] * y[7 + image_width] + Q_mat_ptr[2] * y[7 + 2 * image_width] + Q_mat_ptr[3] * y[7 + 3 * image_width] + Q_mat_ptr[4] * y[7 + 4 * image_width] + Q_mat_ptr[5] * y[7 + 5 * image_width] + Q_mat_ptr[6] * y[7 + 6 * image_width] + Q_mat_ptr[7] * y[7 + 7 * image_width]) * Q_mat_ptr[7];
						S_mat_ptr[1] = (Q_mat_ptr[0] * y[0] + Q_mat_ptr[1] * y[image_width] + Q_mat_ptr[2] * y[2 * image_width] + Q_mat_ptr[3] * y[3 * image_width] + Q_mat_ptr[4] * y[4 * image_width] + Q_mat_ptr[5] * y[5 * image_width] + Q_mat_ptr[6] * y[6 * image_width] + Q_mat_ptr[7] * y[7 * image_width]) * Q_mat_ptr[8] + (Q_mat_ptr[0] * y[1] + Q_mat_ptr[1] * y[1 + image_width] + Q_mat_ptr[2] * y[1 + 2 * image_width] + Q_mat_ptr[3] * y[1 + 3 * image_width] + Q_mat_ptr[4] * y[1 + 4 * image_width] + Q_mat_ptr[5] * y[1 + 5 * image_width] + Q_mat_ptr[6] * y[1 + 6 * image_width] + Q_mat_ptr[7] * y[1 + 7 * image_width]) * Q_mat_ptr[9] + (Q_mat_ptr[0] * y[2] + Q_mat_ptr[1] * y[2 + image_width] + Q_mat_ptr[2] * y[2 + 2 * image_width] + Q_mat_ptr[3] * y[2 + 3 * image_width] + Q_mat_ptr[4] * y[2 + 4 * image_width] + Q_mat_ptr[5] * y[2 + 5 * image_width] + Q_mat_ptr[6] * y[2 + 6 * image_width] + Q_mat_ptr[7] * y[2 + 7 * image_width]) * Q_mat_ptr[10] + (Q_mat_ptr[0] * y[3] + Q_mat_ptr[1] * y[3 + image_width] + Q_mat_ptr[2] * y[3 + 2 * image_width] + Q_mat_ptr[3] * y[3 + 3 * image_width] + Q_mat_ptr[4] * y[3 + 4 * image_width] + Q_mat_ptr[5] * y[3 + 5 * image_width] + Q_mat_ptr[6] * y[3 + 6 * image_width] + Q_mat_ptr[7] * y[3 + 7 * image_width]) * Q_mat_ptr[11] + (Q_mat_ptr[0] * y[4] + Q_mat_ptr[1] * y[4 + image_width] + Q_mat_ptr[2] * y[4 + 2 * image_width] + Q_mat_ptr[3] * y[4 + 3 * image_width] + Q_mat_ptr[4] * y[4 + 4 * image_width] + Q_mat_ptr[5] * y[4 + 5 * image_width] + Q_mat_ptr[6] * y[4 + 6 * image_width] + Q_mat_ptr[7] * y[4 + 7 * image_width]) * Q_mat_ptr[12] + (Q_mat_ptr[0] * y[5] + Q_mat_ptr[1] * y[5 + image_width] + Q_mat_ptr[2] * y[5 + 2 * image_width] + Q_mat_ptr[3] * y[5 + 3 * image_width] + Q_mat_ptr[4] * y[5 + 4 * image_width] + Q_mat_ptr[5] * y[5 + 5 * image_width] + Q_mat_ptr[6] * y[5 + 6 * image_width] + Q_mat_ptr[7] * y[5 + 7 * image_width]) * Q_mat_ptr[13] + (Q_mat_ptr[0] * y[6] + Q_mat_ptr[1] * y[6 + image_width] + Q_mat_ptr[2] * y[6 + 2 * image_width] + Q_mat_ptr[3] * y[6 + 3 * image_width] + Q_mat_ptr[4] * y[6 + 4 * image_width] + Q_mat_ptr[5] * y[6 + 5 * image_width] + Q_mat_ptr[6] * y[6 + 6 * image_width] + Q_mat_ptr[7] * y[6 + 7 * image_width]) * Q_mat_ptr[14] + (Q_mat_ptr[0] * y[7] + Q_mat_ptr[1] * y[7 + image_width] + Q_mat_ptr[2] * y[7 + 2 * image_width] + Q_mat_ptr[3] * y[7 + 3 * image_width] + Q_mat_ptr[4] * y[7 + 4 * image_width] + Q_mat_ptr[5] * y[7 + 5 * image_width] + Q_mat_ptr[6] * y[7 + 6 * image_width] + Q_mat_ptr[7] * y[7 + 7 * image_width]) * Q_mat_ptr[15];
						S_mat_ptr[2] = (Q_mat_ptr[0] * y[0] + Q_mat_ptr[1] * y[image_width] + Q_mat_ptr[2] * y[2 * image_width] + Q_mat_ptr[3] * y[3 * image_width] + Q_mat_ptr[4] * y[4 * image_width] + Q_mat_ptr[5] * y[5 * image_width] + Q_mat_ptr[6] * y[6 * image_width] + Q_mat_ptr[7] * y[7 * image_width]) * Q_mat_ptr[16] + (Q_mat_ptr[0] * y[1] + Q_mat_ptr[1] * y[1 + image_width] + Q_mat_ptr[2] * y[1 + 2 * image_width] + Q_mat_ptr[3] * y[1 + 3 * image_width] + Q_mat_ptr[4] * y[1 + 4 * image_width] + Q_mat_ptr[5] * y[1 + 5 * image_width] + Q_mat_ptr[6] * y[1 + 6 * image_width] + Q_mat_ptr[7] * y[1 + 7 * image_width]) * Q_mat_ptr[17] + (Q_mat_ptr[0] * y[2] + Q_mat_ptr[1] * y[2 + image_width] + Q_mat_ptr[2] * y[2 + 2 * image_width] + Q_mat_ptr[3] * y[2 + 3 * image_width] + Q_mat_ptr[4] * y[2 + 4 * image_width] + Q_mat_ptr[5] * y[2 + 5 * image_width] + Q_mat_ptr[6] * y[2 + 6 * image_width] + Q_mat_ptr[7] * y[2 + 7 * image_width]) * Q_mat_ptr[18] + (Q_mat_ptr[0] * y[3] + Q_mat_ptr[1] * y[3 + image_width] + Q_mat_ptr[2] * y[3 + 2 * image_width] + Q_mat_ptr[3] * y[3 + 3 * image_width] + Q_mat_ptr[4] * y[3 + 4 * image_width] + Q_mat_ptr[5] * y[3 + 5 * image_width] + Q_mat_ptr[6] * y[3 + 6 * image_width] + Q_mat_ptr[7] * y[3 + 7 * image_width]) * Q_mat_ptr[19] + (Q_mat_ptr[0] * y[4] + Q_mat_ptr[1] * y[4 + image_width] + Q_mat_ptr[2] * y[4 + 2 * image_width] + Q_mat_ptr[3] * y[4 + 3 * image_width] + Q_mat_ptr[4] * y[4 + 4 * image_width] + Q_mat_ptr[5] * y[4 + 5 * image_width] + Q_mat_ptr[6] * y[4 + 6 * image_width] + Q_mat_ptr[7] * y[4 + 7 * image_width]) * Q_mat_ptr[20] + (Q_mat_ptr[0] * y[5] + Q_mat_ptr[1] * y[5 + image_width] + Q_mat_ptr[2] * y[5 + 2 * image_width] + Q_mat_ptr[3] * y[5 + 3 * image_width] + Q_mat_ptr[4] * y[5 + 4 * image_width] + Q_mat_ptr[5] * y[5 + 5 * image_width] + Q_mat_ptr[6] * y[5 + 6 * image_width] + Q_mat_ptr[7] * y[5 + 7 * image_width]) * Q_mat_ptr[21] + (Q_mat_ptr[0] * y[6] + Q_mat_ptr[1] * y[6 + image_width] + Q_mat_ptr[2] * y[6 + 2 * image_width] + Q_mat_ptr[3] * y[6 + 3 * image_width] + Q_mat_ptr[4] * y[6 + 4 * image_width] + Q_mat_ptr[5] * y[6 + 5 * image_width] + Q_mat_ptr[6] * y[6 + 6 * image_width] + Q_mat_ptr[7] * y[6 + 7 * image_width]) * Q_mat_ptr[22] + (Q_mat_ptr[0] * y[7] + Q_mat_ptr[1] * y[7 + image_width] + Q_mat_ptr[2] * y[7 + 2 * image_width] + Q_mat_ptr[3] * y[7 + 3 * image_width] + Q_mat_ptr[4] * y[7 + 4 * image_width] + Q_mat_ptr[5] * y[7 + 5 * image_width] + Q_mat_ptr[6] * y[7 + 6 * image_width] + Q_mat_ptr[7] * y[7 + 7 * image_width]) * Q_mat_ptr[23];
						S_mat_ptr[3] = Q_mat_ptr[0] * y[3] + Q_mat_ptr[1] * y[3 + image_width] + Q_mat_ptr[2] * y[3 + 2 * image_width] + Q_mat_ptr[3] * y[3 + 3 * image_width] + Q_mat_ptr[4] * y[3 + 4 * image_width] + Q_mat_ptr[5] * y[3 + 5 * image_width] + Q_mat_ptr[6] * y[3 + 6 * image_width] + Q_mat_ptr[7] * y[3 + 7 * image_width];
						S_mat_ptr[4] = (Q_mat_ptr[8] * y[0] + Q_mat_ptr[9] * y[image_width] + Q_mat_ptr[10] * y[2 * image_width] + Q_mat_ptr[11] * y[3 * image_width] + Q_mat_ptr[12] * y[4 * image_width] + Q_mat_ptr[13] * y[5 * image_width] + Q_mat_ptr[14] * y[6 * image_width] + Q_mat_ptr[15] * y[7 * image_width]) * Q_mat_ptr[0] + (Q_mat_ptr[8] * y[1] + Q_mat_ptr[9] * y[1 + image_width] + Q_mat_ptr[10] * y[1 + 2 * image_width] + Q_mat_ptr[11] * y[1 + 3 * image_width] + Q_mat_ptr[12] * y[1 + 4 * image_width] + Q_mat_ptr[13] * y[1 + 5 * image_width] + Q_mat_ptr[14] * y[1 + 6 * image_width] + Q_mat_ptr[15] * y[1 + 7 * image_width]) * Q_mat_ptr[1] + (Q_mat_ptr[8] * y[2] + Q_mat_ptr[9] * y[2 + image_width] + Q_mat_ptr[10] * y[2 + 2 * image_width] + Q_mat_ptr[11] * y[2 + 3 * image_width] + Q_mat_ptr[12] * y[2 + 4 * image_width] + Q_mat_ptr[13] * y[2 + 5 * image_width] + Q_mat_ptr[14] * y[2 + 6 * image_width] + Q_mat_ptr[15] * y[2 + 7 * image_width]) * Q_mat_ptr[2] + (Q_mat_ptr[8] * y[3] + Q_mat_ptr[9] * y[3 + image_width] + Q_mat_ptr[10] * y[3 + 2 * image_width] + Q_mat_ptr[11] * y[3 + 3 * image_width] + Q_mat_ptr[12] * y[3 + 4 * image_width] + Q_mat_ptr[13] * y[3 + 5 * image_width] + Q_mat_ptr[14] * y[3 + 6 * image_width] + Q_mat_ptr[15] * y[3 + 7 * image_width]) * Q_mat_ptr[3] + (Q_mat_ptr[8] * y[4] + Q_mat_ptr[9] * y[4 + image_width] + Q_mat_ptr[10] * y[4 + 2 * image_width] + Q_mat_ptr[11] * y[4 + 3 * image_width] + Q_mat_ptr[12] * y[4 + 4 * image_width] + Q_mat_ptr[13] * y[4 + 5 * image_width] + Q_mat_ptr[14] * y[4 + 6 * image_width] + Q_mat_ptr[15] * y[4 + 7 * image_width]) * Q_mat_ptr[4] + (Q_mat_ptr[8] * y[5] + Q_mat_ptr[9] * y[5 + image_width] + Q_mat_ptr[10] * y[5 + 2 * image_width] + Q_mat_ptr[11] * y[5 + 3 * image_width] + Q_mat_ptr[12] * y[5 + 4 * image_width] + Q_mat_ptr[13] * y[5 + 5 * image_width] + Q_mat_ptr[14] * y[5 + 6 * image_width] + Q_mat_ptr[15] * y[5 + 7 * image_width]) * Q_mat_ptr[5] + (Q_mat_ptr[8] * y[6] + Q_mat_ptr[9] * y[6 + image_width] + Q_mat_ptr[10] * y[6 + 2 * image_width] + Q_mat_ptr[11] * y[6 + 3 * image_width] + Q_mat_ptr[12] * y[6 + 4 * image_width] + Q_mat_ptr[13] * y[6 + 5 * image_width] + Q_mat_ptr[14] * y[6 + 6 * image_width] + Q_mat_ptr[15] * y[6 + 7 * image_width]) * Q_mat_ptr[6] + (Q_mat_ptr[8] * y[7] + Q_mat_ptr[9] * y[7 + image_width] + Q_mat_ptr[10] * y[7 + 2 * image_width] + Q_mat_ptr[11] * y[7 + 3 * image_width] + Q_mat_ptr[12] * y[7 + 4 * image_width] + Q_mat_ptr[13] * y[7 + 5 * image_width] + Q_mat_ptr[14] * y[7 + 6 * image_width] + Q_mat_ptr[15] * y[7 + 7 * image_width]) * Q_mat_ptr[7];
						S_mat_ptr[5] = (Q_mat_ptr[8] * y[0] + Q_mat_ptr[9] * y[image_width] + Q_mat_ptr[10] * y[2 * image_width] + Q_mat_ptr[11] * y[3 * image_width] + Q_mat_ptr[12] * y[4 * image_width] + Q_mat_ptr[13] * y[5 * image_width] + Q_mat_ptr[14] * y[6 * image_width] + Q_mat_ptr[15] * y[7 * image_width]) * Q_mat_ptr[8] + (Q_mat_ptr[8] * y[1] + Q_mat_ptr[9] * y[1 + image_width] + Q_mat_ptr[10] * y[1 + 2 * image_width] + Q_mat_ptr[11] * y[1 + 3 * image_width] + Q_mat_ptr[12] * y[1 + 4 * image_width] + Q_mat_ptr[13] * y[1 + 5 * image_width] + Q_mat_ptr[14] * y[1 + 6 * image_width] + Q_mat_ptr[15] * y[1 + 7 * image_width]) * Q_mat_ptr[9] + (Q_mat_ptr[8] * y[2] + Q_mat_ptr[9] * y[2 + image_width] + Q_mat_ptr[10] * y[2 + 2 * image_width] + Q_mat_ptr[11] * y[2 + 3 * image_width] + Q_mat_ptr[12] * y[2 + 4 * image_width] + Q_mat_ptr[13] * y[2 + 5 * image_width] + Q_mat_ptr[14] * y[2 + 6 * image_width] + Q_mat_ptr[15] * y[2 + 7 * image_width]) * Q_mat_ptr[10] + (Q_mat_ptr[8] * y[3] + Q_mat_ptr[9] * y[3 + image_width] + Q_mat_ptr[10] * y[3 + 2 * image_width] + Q_mat_ptr[11] * y[3 + 3 * image_width] + Q_mat_ptr[12] * y[3 + 4 * image_width] + Q_mat_ptr[13] * y[3 + 5 * image_width] + Q_mat_ptr[14] * y[3 + 6 * image_width] + Q_mat_ptr[15] * y[3 + 7 * image_width]) * Q_mat_ptr[11] + (Q_mat_ptr[8] * y[4] + Q_mat_ptr[9] * y[4 + image_width] + Q_mat_ptr[10] * y[4 + 2 * image_width] + Q_mat_ptr[11] * y[4 + 3 * image_width] + Q_mat_ptr[12] * y[4 + 4 * image_width] + Q_mat_ptr[13] * y[4 + 5 * image_width] + Q_mat_ptr[14] * y[4 + 6 * image_width] + Q_mat_ptr[15] * y[4 + 7 * image_width]) * Q_mat_ptr[12] + (Q_mat_ptr[8] * y[5] + Q_mat_ptr[9] * y[5 + image_width] + Q_mat_ptr[10] * y[5 + 2 * image_width] + Q_mat_ptr[11] * y[5 + 3 * image_width] + Q_mat_ptr[12] * y[5 + 4 * image_width] + Q_mat_ptr[13] * y[5 + 5 * image_width] + Q_mat_ptr[14] * y[5 + 6 * image_width] + Q_mat_ptr[15] * y[5 + 7 * image_width]) * Q_mat_ptr[13] + (Q_mat_ptr[8] * y[6] + Q_mat_ptr[9] * y[6 + image_width] + Q_mat_ptr[10] * y[6 + 2 * image_width] + Q_mat_ptr[11] * y[6 + 3 * image_width] + Q_mat_ptr[12] * y[6 + 4 * image_width] + Q_mat_ptr[13] * y[6 + 5 * image_width] + Q_mat_ptr[14] * y[6 + 6 * image_width] + Q_mat_ptr[15] * y[6 + 7 * image_width]) * Q_mat_ptr[14] + (Q_mat_ptr[8] * y[7] + Q_mat_ptr[9] * y[7 + image_width] + Q_mat_ptr[10] * y[7 + 2 * image_width] + Q_mat_ptr[11] * y[7 + 3 * image_width] + Q_mat_ptr[12] * y[7 + 4 * image_width] + Q_mat_ptr[13] * y[7 + 5 * image_width] + Q_mat_ptr[14] * y[7 + 6 * image_width] + Q_mat_ptr[15] * y[7 + 7 * image_width]) * Q_mat_ptr[15];
						S_mat_ptr[6] = (Q_mat_ptr[8] * y[0] + Q_mat_ptr[9] * y[image_width] + Q_mat_ptr[10] * y[2 * image_width] + Q_mat_ptr[11] * y[3 * image_width] + Q_mat_ptr[12] * y[4 * image_width] + Q_mat_ptr[13] * y[5 * image_width] + Q_mat_ptr[14] * y[6 * image_width] + Q_mat_ptr[15] * y[7 * image_width]) * Q_mat_ptr[16] + (Q_mat_ptr[8] * y[1] + Q_mat_ptr[9] * y[1 + image_width] + Q_mat_ptr[10] * y[1 + 2 * image_width] + Q_mat_ptr[11] * y[1 + 3 * image_width] + Q_mat_ptr[12] * y[1 + 4 * image_width] + Q_mat_ptr[13] * y[1 + 5 * image_width] + Q_mat_ptr[14] * y[1 + 6 * image_width] + Q_mat_ptr[15] * y[1 + 7 * image_width]) * Q_mat_ptr[17] + (Q_mat_ptr[8] * y[2] + Q_mat_ptr[9] * y[2 + image_width] + Q_mat_ptr[10] * y[2 + 2 * image_width] + Q_mat_ptr[11] * y[2 + 3 * image_width] + Q_mat_ptr[12] * y[2 + 4 * image_width] + Q_mat_ptr[13] * y[2 + 5 * image_width] + Q_mat_ptr[14] * y[2 + 6 * image_width] + Q_mat_ptr[15] * y[2 + 7 * image_width]) * Q_mat_ptr[18] + (Q_mat_ptr[8] * y[3] + Q_mat_ptr[9] * y[3 + image_width] + Q_mat_ptr[10] * y[3 + 2 * image_width] + Q_mat_ptr[11] * y[3 + 3 * image_width] + Q_mat_ptr[12] * y[3 + 4 * image_width] + Q_mat_ptr[13] * y[3 + 5 * image_width] + Q_mat_ptr[14] * y[3 + 6 * image_width] + Q_mat_ptr[15] * y[3 + 7 * image_width]) * Q_mat_ptr[19] + (Q_mat_ptr[8] * y[4] + Q_mat_ptr[9] * y[4 + image_width] + Q_mat_ptr[10] * y[4 + 2 * image_width] + Q_mat_ptr[11] * y[4 + 3 * image_width] + Q_mat_ptr[12] * y[4 + 4 * image_width] + Q_mat_ptr[13] * y[4 + 5 * image_width] + Q_mat_ptr[14] * y[4 + 6 * image_width] + Q_mat_ptr[15] * y[4 + 7 * image_width]) * Q_mat_ptr[20] + (Q_mat_ptr[8] * y[5] + Q_mat_ptr[9] * y[5 + image_width] + Q_mat_ptr[10] * y[5 + 2 * image_width] + Q_mat_ptr[11] * y[5 + 3 * image_width] + Q_mat_ptr[12] * y[5 + 4 * image_width] + Q_mat_ptr[13] * y[5 + 5 * image_width] + Q_mat_ptr[14] * y[5 + 6 * image_width] + Q_mat_ptr[15] * y[5 + 7 * image_width]) * Q_mat_ptr[21] + (Q_mat_ptr[8] * y[6] + Q_mat_ptr[9] * y[6 + image_width] + Q_mat_ptr[10] * y[6 + 2 * image_width] + Q_mat_ptr[11] * y[6 + 3 * image_width] + Q_mat_ptr[12] * y[6 + 4 * image_width] + Q_mat_ptr[13] * y[6 + 5 * image_width] + Q_mat_ptr[14] * y[6 + 6 * image_width] + Q_mat_ptr[15] * y[6 + 7 * image_width]) * Q_mat_ptr[22] + (Q_mat_ptr[8] * y[7] + Q_mat_ptr[9] * y[7 + image_width] + Q_mat_ptr[10] * y[7 + 2 * image_width] + Q_mat_ptr[11] * y[7 + 3 * image_width] + Q_mat_ptr[12] * y[7 + 4 * image_width] + Q_mat_ptr[13] * y[7 + 5 * image_width] + Q_mat_ptr[14] * y[7 + 6 * image_width] + Q_mat_ptr[15] * y[7 + 7 * image_width]) * Q_mat_ptr[23];
						S_mat_ptr[7] = Q_mat_ptr[8] * y[3] + Q_mat_ptr[9] * y[3 + image_width] + Q_mat_ptr[10] * y[3 + 2 * image_width] + Q_mat_ptr[11] * y[3 + 3 * image_width] + Q_mat_ptr[12] * y[3 + 4 * image_width] + Q_mat_ptr[13] * y[3 + 5 * image_width] + Q_mat_ptr[14] * y[3 + 6 * image_width] + Q_mat_ptr[15] * y[3 + 7 * image_width];
						S_mat_ptr[8] = (Q_mat_ptr[16] * y[0] + Q_mat_ptr[17] * y[image_width] + Q_mat_ptr[18] * y[2 * image_width] + Q_mat_ptr[19] * y[3 * image_width] + Q_mat_ptr[20] * y[4 * image_width] + Q_mat_ptr[21] * y[5 * image_width] + Q_mat_ptr[22] * y[6 * image_width] + Q_mat_ptr[23] * y[7 * image_width]) * Q_mat_ptr[0] + (Q_mat_ptr[16] * y[1] + Q_mat_ptr[17] * y[1 + image_width] + Q_mat_ptr[18] * y[1 + 2 * image_width] + Q_mat_ptr[19] * y[1 + 3 * image_width] + Q_mat_ptr[20] * y[1 + 4 * image_width] + Q_mat_ptr[21] * y[1 + 5 * image_width] + Q_mat_ptr[22] * y[1 + 6 * image_width] + Q_mat_ptr[23] * y[1 + 7 * image_width]) * Q_mat_ptr[1] + (Q_mat_ptr[16] * y[2] + Q_mat_ptr[17] * y[2 + image_width] + Q_mat_ptr[18] * y[2 + 2 * image_width] + Q_mat_ptr[19] * y[2 + 3 * image_width] + Q_mat_ptr[20] * y[2 + 4 * image_width] + Q_mat_ptr[21] * y[2 + 5 * image_width] + Q_mat_ptr[22] * y[2 + 6 * image_width] + Q_mat_ptr[23] * y[2 + 7 * image_width]) * Q_mat_ptr[2] + (Q_mat_ptr[16] * y[3] + Q_mat_ptr[17] * y[3 + image_width] + Q_mat_ptr[18] * y[3 + 2 * image_width] + Q_mat_ptr[19] * y[3 + 3 * image_width] + Q_mat_ptr[20] * y[3 + 4 * image_width] + Q_mat_ptr[21] * y[3 + 5 * image_width] + Q_mat_ptr[22] * y[3 + 6 * image_width] + Q_mat_ptr[23] * y[3 + 7 * image_width]) * Q_mat_ptr[3] + (Q_mat_ptr[16] * y[4] + Q_mat_ptr[17] * y[4 + image_width] + Q_mat_ptr[18] * y[4 + 2 * image_width] + Q_mat_ptr[19] * y[4 + 3 * image_width] + Q_mat_ptr[20] * y[4 + 4 * image_width] + Q_mat_ptr[21] * y[4 + 5 * image_width] + Q_mat_ptr[22] * y[4 + 6 * image_width] + Q_mat_ptr[23] * y[4 + 7 * image_width]) * Q_mat_ptr[4] + (Q_mat_ptr[16] * y[5] + Q_mat_ptr[17] * y[5 + image_width] + Q_mat_ptr[18] * y[5 + 2 * image_width] + Q_mat_ptr[19] * y[5 + 3 * image_width] + Q_mat_ptr[20] * y[5 + 4 * image_width] + Q_mat_ptr[21] * y[5 + 5 * image_width] + Q_mat_ptr[22] * y[5 + 6 * image_width] + Q_mat_ptr[23] * y[5 + 7 * image_width]) * Q_mat_ptr[5] + (Q_mat_ptr[16] * y[6] + Q_mat_ptr[17] * y[6 + image_width] + Q_mat_ptr[18] * y[6 + 2 * image_width] + Q_mat_ptr[19] * y[6 + 3 * image_width] + Q_mat_ptr[20] * y[6 + 4 * image_width] + Q_mat_ptr[21] * y[6 + 5 * image_width] + Q_mat_ptr[22] * y[6 + 6 * image_width] + Q_mat_ptr[23] * y[6 + 7 * image_width]) * Q_mat_ptr[6] + (Q_mat_ptr[16] * y[7] + Q_mat_ptr[17] * y[7 + image_width] + Q_mat_ptr[18] * y[7 + 2 * image_width] + Q_mat_ptr[19] * y[7 + 3 * image_width] + Q_mat_ptr[20] * y[7 + 4 * image_width] + Q_mat_ptr[21] * y[7 + 5 * image_width] + Q_mat_ptr[22] * y[7 + 6 * image_width] + Q_mat_ptr[23] * y[7 + 7 * image_width]) * Q_mat_ptr[7];
						S_mat_ptr[9] = (Q_mat_ptr[16] * y[0] + Q_mat_ptr[17] * y[image_width] + Q_mat_ptr[18] * y[2 * image_width] + Q_mat_ptr[19] * y[3 * image_width] + Q_mat_ptr[20] * y[4 * image_width] + Q_mat_ptr[21] * y[5 * image_width] + Q_mat_ptr[22] * y[6 * image_width] + Q_mat_ptr[23] * y[7 * image_width]) * Q_mat_ptr[8] + (Q_mat_ptr[16] * y[1] + Q_mat_ptr[17] * y[1 + image_width] + Q_mat_ptr[18] * y[1 + 2 * image_width] + Q_mat_ptr[19] * y[1 + 3 * image_width] + Q_mat_ptr[20] * y[1 + 4 * image_width] + Q_mat_ptr[21] * y[1 + 5 * image_width] + Q_mat_ptr[22] * y[1 + 6 * image_width] + Q_mat_ptr[23] * y[1 + 7 * image_width]) * Q_mat_ptr[9] + (Q_mat_ptr[16] * y[2] + Q_mat_ptr[17] * y[2 + image_width] + Q_mat_ptr[18] * y[2 + 2 * image_width] + Q_mat_ptr[19] * y[2 + 3 * image_width] + Q_mat_ptr[20] * y[2 + 4 * image_width] + Q_mat_ptr[21] * y[2 + 5 * image_width] + Q_mat_ptr[22] * y[2 + 6 * image_width] + Q_mat_ptr[23] * y[2 + 7 * image_width]) * Q_mat_ptr[10] + (Q_mat_ptr[16] * y[3] + Q_mat_ptr[17] * y[3 + image_width] + Q_mat_ptr[18] * y[3 + 2 * image_width] + Q_mat_ptr[19] * y[3 + 3 * image_width] + Q_mat_ptr[20] * y[3 + 4 * image_width] + Q_mat_ptr[21] * y[3 + 5 * image_width] + Q_mat_ptr[22] * y[3 + 6 * image_width] + Q_mat_ptr[23] * y[3 + 7 * image_width]) * Q_mat_ptr[11] + (Q_mat_ptr[16] * y[4] + Q_mat_ptr[17] * y[4 + image_width] + Q_mat_ptr[18] * y[4 + 2 * image_width] + Q_mat_ptr[19] * y[4 + 3 * image_width] + Q_mat_ptr[20] * y[4 + 4 * image_width] + Q_mat_ptr[21] * y[4 + 5 * image_width] + Q_mat_ptr[22] * y[4 + 6 * image_width] + Q_mat_ptr[23] * y[4 + 7 * image_width]) * Q_mat_ptr[12] + (Q_mat_ptr[16] * y[5] + Q_mat_ptr[17] * y[5 + image_width] + Q_mat_ptr[18] * y[5 + 2 * image_width] + Q_mat_ptr[19] * y[5 + 3 * image_width] + Q_mat_ptr[20] * y[5 + 4 * image_width] + Q_mat_ptr[21] * y[5 + 5 * image_width] + Q_mat_ptr[22] * y[5 + 6 * image_width] + Q_mat_ptr[23] * y[5 + 7 * image_width]) * Q_mat_ptr[13] + (Q_mat_ptr[16] * y[6] + Q_mat_ptr[17] * y[6 + image_width] + Q_mat_ptr[18] * y[6 + 2 * image_width] + Q_mat_ptr[19] * y[6 + 3 * image_width] + Q_mat_ptr[20] * y[6 + 4 * image_width] + Q_mat_ptr[21] * y[6 + 5 * image_width] + Q_mat_ptr[22] * y[6 + 6 * image_width] + Q_mat_ptr[23] * y[6 + 7 * image_width]) * Q_mat_ptr[14] + (Q_mat_ptr[16] * y[7] + Q_mat_ptr[17] * y[7 + image_width] + Q_mat_ptr[18] * y[7 + 2 * image_width] + Q_mat_ptr[19] * y[7 + 3 * image_width] + Q_mat_ptr[20] * y[7 + 4 * image_width] + Q_mat_ptr[21] * y[7 + 5 * image_width] + Q_mat_ptr[22] * y[7 + 6 * image_width] + Q_mat_ptr[23] * y[7 + 7 * image_width]) * Q_mat_ptr[15];
						S_mat_ptr[10] = (Q_mat_ptr[16] * y[0] + Q_mat_ptr[17] * y[image_width] + Q_mat_ptr[18] * y[2 * image_width] + Q_mat_ptr[19] * y[3 * image_width] + Q_mat_ptr[20] * y[4 * image_width] + Q_mat_ptr[21] * y[5 * image_width] + Q_mat_ptr[22] * y[6 * image_width] + Q_mat_ptr[23] * y[7 * image_width]) * Q_mat_ptr[16] + (Q_mat_ptr[16] * y[1] + Q_mat_ptr[17] * y[1 + image_width] + Q_mat_ptr[18] * y[1 + 2 * image_width] + Q_mat_ptr[19] * y[1 + 3 * image_width] + Q_mat_ptr[20] * y[1 + 4 * image_width] + Q_mat_ptr[21] * y[1 + 5 * image_width] + Q_mat_ptr[22] * y[1 + 6 * image_width] + Q_mat_ptr[23] * y[1 + 7 * image_width]) * Q_mat_ptr[17] + (Q_mat_ptr[16] * y[2] + Q_mat_ptr[17] * y[2 + image_width] + Q_mat_ptr[18] * y[2 + 2 * image_width] + Q_mat_ptr[19] * y[2 + 3 * image_width] + Q_mat_ptr[20] * y[2 + 4 * image_width] + Q_mat_ptr[21] * y[2 + 5 * image_width] + Q_mat_ptr[22] * y[2 + 6 * image_width] + Q_mat_ptr[23] * y[2 + 7 * image_width]) * Q_mat_ptr[18] + (Q_mat_ptr[16] * y[3] + Q_mat_ptr[17] * y[3 + image_width] + Q_mat_ptr[18] * y[3 + 2 * image_width] + Q_mat_ptr[19] * y[3 + 3 * image_width] + Q_mat_ptr[20] * y[3 + 4 * image_width] + Q_mat_ptr[21] * y[3 + 5 * image_width] + Q_mat_ptr[22] * y[3 + 6 * image_width] + Q_mat_ptr[23] * y[3 + 7 * image_width]) * Q_mat_ptr[19] + (Q_mat_ptr[16] * y[4] + Q_mat_ptr[17] * y[4 + image_width] + Q_mat_ptr[18] * y[4 + 2 * image_width] + Q_mat_ptr[19] * y[4 + 3 * image_width] + Q_mat_ptr[20] * y[4 + 4 * image_width] + Q_mat_ptr[21] * y[4 + 5 * image_width] + Q_mat_ptr[22] * y[4 + 6 * image_width] + Q_mat_ptr[23] * y[4 + 7 * image_width]) * Q_mat_ptr[20] + (Q_mat_ptr[16] * y[5] + Q_mat_ptr[17] * y[5 + image_width] + Q_mat_ptr[18] * y[5 + 2 * image_width] + Q_mat_ptr[19] * y[5 + 3 * image_width] + Q_mat_ptr[20] * y[5 + 4 * image_width] + Q_mat_ptr[21] * y[5 + 5 * image_width] + Q_mat_ptr[22] * y[5 + 6 * image_width] + Q_mat_ptr[23] * y[5 + 7 * image_width]) * Q_mat_ptr[21] + (Q_mat_ptr[16] * y[6] + Q_mat_ptr[17] * y[6 + image_width] + Q_mat_ptr[18] * y[6 + 2 * image_width] + Q_mat_ptr[19] * y[6 + 3 * image_width] + Q_mat_ptr[20] * y[6 + 4 * image_width] + Q_mat_ptr[21] * y[6 + 5 * image_width] + Q_mat_ptr[22] * y[6 + 6 * image_width] + Q_mat_ptr[23] * y[6 + 7 * image_width]) * Q_mat_ptr[22] + (Q_mat_ptr[16] * y[7] + Q_mat_ptr[17] * y[7 + image_width] + Q_mat_ptr[18] * y[7 + 2 * image_width] + Q_mat_ptr[19] * y[7 + 3 * image_width] + Q_mat_ptr[20] * y[7 + 4 * image_width] + Q_mat_ptr[21] * y[7 + 5 * image_width] + Q_mat_ptr[22] * y[7 + 6 * image_width] + Q_mat_ptr[23] * y[7 + 7 * image_width]) * Q_mat_ptr[23];
						S_mat_ptr[11] = Q_mat_ptr[16] * y[3] + Q_mat_ptr[17] * y[3 + image_width] + Q_mat_ptr[18] * y[3 + 2 * image_width] + Q_mat_ptr[19] * y[3 + 3 * image_width] + Q_mat_ptr[20] * y[3 + 4 * image_width] + Q_mat_ptr[21] * y[3 + 5 * image_width] + Q_mat_ptr[22] * y[3 + 6 * image_width] + Q_mat_ptr[23] * y[3 + 7 * image_width];
						S_mat_ptr[12] = y[3 * image_width] * Q_mat_ptr[0] + y[1 + 3 * image_width] * Q_mat_ptr[1] + y[2 + 3 * image_width] * Q_mat_ptr[2] + Q_mat_ptr[3] * y[3 + 3 * image_width] + y[4 + 3 * image_width] * Q_mat_ptr[4] + y[5 + 3 * image_width] * Q_mat_ptr[5] + y[6 + 3 * image_width] * Q_mat_ptr[6] + y[7 + 3 * image_width] * Q_mat_ptr[7];
						S_mat_ptr[13] = y[3 * image_width] * Q_mat_ptr[8] + y[1 + 3 * image_width] * Q_mat_ptr[9] + y[2 + 3 * image_width] * Q_mat_ptr[10] + Q_mat_ptr[11] * y[3 + 3 * image_width] + y[4 + 3 * image_width] * Q_mat_ptr[12] + y[5 + 3 * image_width] * Q_mat_ptr[13] + y[6 + 3 * image_width] * Q_mat_ptr[14] + y[7 + 3 * image_width] * Q_mat_ptr[15];
						S_mat_ptr[14] = y[3 * image_width] * Q_mat_ptr[16] + y[1 + 3 * image_width] * Q_mat_ptr[17] + y[2 + 3 * image_width] * Q_mat_ptr[18] + Q_mat_ptr[19] * y[3 + 3 * image_width] + y[4 + 3 * image_width] * Q_mat_ptr[20] + y[5 + 3 * image_width] * Q_mat_ptr[21] + y[6 + 3 * image_width] * Q_mat_ptr[22] + y[7 + 3 * image_width] * Q_mat_ptr[23];
						S_mat_ptr[15] = y[3 + 3 * image_width];
					}
					CV_MAT_ELEM(*lut_valid_sign_mat,uchar,index_y - aoi_y_min,index_x - aoi_x_min) =1;						
				}		
				CV_MAT_ELEM(*g_mat,double,i,j) = 
					delta_x*delta_x*delta_x * (delta_y*delta_y*delta_y * S_mat_ptr[0] + delta_y*delta_y * S_mat_ptr[4] + delta_y * S_mat_ptr[8] + S_mat_ptr[12] ) +
					delta_x*delta_x * (delta_y*delta_y*delta_y * S_mat_ptr[1] + delta_y*delta_y * S_mat_ptr[5] + delta_y * S_mat_ptr[9] + S_mat_ptr[13] ) +
					delta_x  * (delta_y*delta_y*delta_y * S_mat_ptr[2] + delta_y*delta_y * S_mat_ptr[6] + delta_y * S_mat_ptr[10] + S_mat_ptr[14] ) +
					(delta_y*delta_y*delta_y * S_mat_ptr[3] + delta_y*delta_y * S_mat_ptr[7] + delta_y * S_mat_ptr[11] + S_mat_ptr[15] );

			}
		}
	}

	return true;
}


bool UniformCubicNatureSplineInterpolationMethod::Two_Dimension_Interpolation_Many_Points_For_Image_64F(
	CvMat* image,//64F
	int reference_point_x,
	int reference_point_y,
	CvMat* displacement_input,//映射后应全部处于图像宽度和高度内，64F,6行1列
	int aoi_x_min,
	int aoi_x_max,//上闭区间，下开区间
	int aoi_y_min,
	int aoi_y_max,//左闭区间，右边开区间
	CvMat* lut_mat,
	CvMat* lut_valid_sign_mat,
	CvMat* Q_mat,
	CvMat* g_mat
	)
{
	//判断
	//大小
	int image_width= image->width;
	int image_height = image->height;
	int template_width = g_mat->cols;
	int template_height = g_mat->cols;
	int lut_width = lut_mat->cols;
	int lut_height = lut_mat->rows;

	if (displacement_input->rows!=6 && displacement_input->cols!=1)
	{
		return false;
	} 

	if (aoi_x_min<0 || aoi_x_max>=image_width||
		aoi_y_min<0 || aoi_y_max>=image_height)
	{
		return false;
	}
	int aoi_width = aoi_x_max-aoi_x_min;
	int aoi_height = aoi_y_max-aoi_y_min;

	if (lut_mat->cols != aoi_width*lut_element_size_||
		lut_mat->rows !=aoi_height||
		lut_valid_sign_mat->cols!=aoi_width||
		lut_valid_sign_mat->rows!= aoi_height)
	{
		return false;
	}

	int Q_mat_type =0;//1--4x6,2--4x8
	if (Q_mat->rows==4 && Q_mat->cols==6)
	{
		Q_mat_type =1;
	}
	else if (Q_mat->rows==4 && Q_mat->cols==8)
	{
		Q_mat_type=2;
	} 
	else
	{
		return false;
	}



	//类型
	int image_type = 0;//1--CV_8U,2--CV_64F
	if (CV_MAT_TYPE(image->type) ==CV_64F)
	{
		image_type = 2;
	}
	//else if (CV_MAT_TYPE(image->type) ==CV_64F)
	//{
	//	image_type = 2;
	//} 
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
	if ( CV_MAT_TYPE(displacement_input->type) != CV_64F)
	{
		return false;
	}
	if (CV_MAT_TYPE(g_mat->type) != CV_64F || CV_MAT_TYPE(Q_mat->type) != CV_64F || CV_MAT_TYPE(lut_valid_sign_mat->type)!=CV_8U)
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
	double* Q_mat_ptr = Q_mat->data.db;


	if (lut_type==1)
	{
		for (int i =0;i<template_height;i++)
		{
			for (int j =0;j<template_width;j++)
			{
				//第一种
				double pos_x =reference_point_x + j - template_half_width + u + ux * (j - template_half_width) + uy * (i - template_half_height);
				double pos_y =reference_point_y + i - template_half_height + v + vx * (j - template_half_width) + vy * (i - template_half_height);

				if (pos_x<aoi_x_min || pos_x>=aoi_x_max||
					pos_y<aoi_y_min || pos_y>=aoi_y_max)//右侧为开区间
				{
					return false;
				}
				int index_x = cvFloor(pos_x);
				int index_y = cvFloor(pos_y);

				double delta_x = pos_x - index_x;
				double delta_y = pos_y - index_y;
				uchar sign = CV_MAT_ELEM(*lut_valid_sign_mat,uchar,index_y - aoi_y_min,index_x - aoi_x_min);
				float* S_mat_ptr = lut_mat->data.fl+ ((index_y - aoi_y_min) * aoi_width+index_x - aoi_x_min) * lut_element_size_;
				if (sign!=1)
				{
					double* y;
					if (Q_mat_type==1)
					{
						y = image->data.db + (index_y-2)*image_width+index_x-2;
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

					} 
					else
					{
						y = image->data.db + (index_y-3)*image_width+index_x-3;
						S_mat_ptr[0] = (Q_mat_ptr[0] * y[0] + Q_mat_ptr[1] * y[image_width] + Q_mat_ptr[2] * y[2 * image_width] + Q_mat_ptr[3] * y[3 * image_width] + Q_mat_ptr[4] * y[4 * image_width] + Q_mat_ptr[5] * y[5 * image_width] + Q_mat_ptr[6] * y[6 * image_width] + Q_mat_ptr[7] * y[7 * image_width]) * Q_mat_ptr[0] + (Q_mat_ptr[0] * y[1] + Q_mat_ptr[1] * y[1 + image_width] + Q_mat_ptr[2] * y[1 + 2 * image_width] + Q_mat_ptr[3] * y[1 + 3 * image_width] + Q_mat_ptr[4] * y[1 + 4 * image_width] + Q_mat_ptr[5] * y[1 + 5 * image_width] + Q_mat_ptr[6] * y[1 + 6 * image_width] + Q_mat_ptr[7] * y[1 + 7 * image_width]) * Q_mat_ptr[1] + (Q_mat_ptr[0] * y[2] + Q_mat_ptr[1] * y[2 + image_width] + Q_mat_ptr[2] * y[2 + 2 * image_width] + Q_mat_ptr[3] * y[2 + 3 * image_width] + Q_mat_ptr[4] * y[2 + 4 * image_width] + Q_mat_ptr[5] * y[2 + 5 * image_width] + Q_mat_ptr[6] * y[2 + 6 * image_width] + Q_mat_ptr[7] * y[2 + 7 * image_width]) * Q_mat_ptr[2] + (Q_mat_ptr[0] * y[3] + Q_mat_ptr[1] * y[3 + image_width] + Q_mat_ptr[2] * y[3 + 2 * image_width] + Q_mat_ptr[3] * y[3 + 3 * image_width] + Q_mat_ptr[4] * y[3 + 4 * image_width] + Q_mat_ptr[5] * y[3 + 5 * image_width] + Q_mat_ptr[6] * y[3 + 6 * image_width] + Q_mat_ptr[7] * y[3 + 7 * image_width]) * Q_mat_ptr[3] + (Q_mat_ptr[0] * y[4] + Q_mat_ptr[1] * y[4 + image_width] + Q_mat_ptr[2] * y[4 + 2 * image_width] + Q_mat_ptr[3] * y[4 + 3 * image_width] + Q_mat_ptr[4] * y[4 + 4 * image_width] + Q_mat_ptr[5] * y[4 + 5 * image_width] + Q_mat_ptr[6] * y[4 + 6 * image_width] + Q_mat_ptr[7] * y[4 + 7 * image_width]) * Q_mat_ptr[4] + (Q_mat_ptr[0] * y[5] + Q_mat_ptr[1] * y[5 + image_width] + Q_mat_ptr[2] * y[5 + 2 * image_width] + Q_mat_ptr[3] * y[5 + 3 * image_width] + Q_mat_ptr[4] * y[5 + 4 * image_width] + Q_mat_ptr[5] * y[5 + 5 * image_width] + Q_mat_ptr[6] * y[5 + 6 * image_width] + Q_mat_ptr[7] * y[5 + 7 * image_width]) * Q_mat_ptr[5] + (Q_mat_ptr[0] * y[6] + Q_mat_ptr[1] * y[6 + image_width] + Q_mat_ptr[2] * y[6 + 2 * image_width] + Q_mat_ptr[3] * y[6 + 3 * image_width] + Q_mat_ptr[4] * y[6 + 4 * image_width] + Q_mat_ptr[5] * y[6 + 5 * image_width] + Q_mat_ptr[6] * y[6 + 6 * image_width] + Q_mat_ptr[7] * y[6 + 7 * image_width]) * Q_mat_ptr[6] + (Q_mat_ptr[0] * y[7] + Q_mat_ptr[1] * y[7 + image_width] + Q_mat_ptr[2] * y[7 + 2 * image_width] + Q_mat_ptr[3] * y[7 + 3 * image_width] + Q_mat_ptr[4] * y[7 + 4 * image_width] + Q_mat_ptr[5] * y[7 + 5 * image_width] + Q_mat_ptr[6] * y[7 + 6 * image_width] + Q_mat_ptr[7] * y[7 + 7 * image_width]) * Q_mat_ptr[7];
						S_mat_ptr[1] = (Q_mat_ptr[0] * y[0] + Q_mat_ptr[1] * y[image_width] + Q_mat_ptr[2] * y[2 * image_width] + Q_mat_ptr[3] * y[3 * image_width] + Q_mat_ptr[4] * y[4 * image_width] + Q_mat_ptr[5] * y[5 * image_width] + Q_mat_ptr[6] * y[6 * image_width] + Q_mat_ptr[7] * y[7 * image_width]) * Q_mat_ptr[8] + (Q_mat_ptr[0] * y[1] + Q_mat_ptr[1] * y[1 + image_width] + Q_mat_ptr[2] * y[1 + 2 * image_width] + Q_mat_ptr[3] * y[1 + 3 * image_width] + Q_mat_ptr[4] * y[1 + 4 * image_width] + Q_mat_ptr[5] * y[1 + 5 * image_width] + Q_mat_ptr[6] * y[1 + 6 * image_width] + Q_mat_ptr[7] * y[1 + 7 * image_width]) * Q_mat_ptr[9] + (Q_mat_ptr[0] * y[2] + Q_mat_ptr[1] * y[2 + image_width] + Q_mat_ptr[2] * y[2 + 2 * image_width] + Q_mat_ptr[3] * y[2 + 3 * image_width] + Q_mat_ptr[4] * y[2 + 4 * image_width] + Q_mat_ptr[5] * y[2 + 5 * image_width] + Q_mat_ptr[6] * y[2 + 6 * image_width] + Q_mat_ptr[7] * y[2 + 7 * image_width]) * Q_mat_ptr[10] + (Q_mat_ptr[0] * y[3] + Q_mat_ptr[1] * y[3 + image_width] + Q_mat_ptr[2] * y[3 + 2 * image_width] + Q_mat_ptr[3] * y[3 + 3 * image_width] + Q_mat_ptr[4] * y[3 + 4 * image_width] + Q_mat_ptr[5] * y[3 + 5 * image_width] + Q_mat_ptr[6] * y[3 + 6 * image_width] + Q_mat_ptr[7] * y[3 + 7 * image_width]) * Q_mat_ptr[11] + (Q_mat_ptr[0] * y[4] + Q_mat_ptr[1] * y[4 + image_width] + Q_mat_ptr[2] * y[4 + 2 * image_width] + Q_mat_ptr[3] * y[4 + 3 * image_width] + Q_mat_ptr[4] * y[4 + 4 * image_width] + Q_mat_ptr[5] * y[4 + 5 * image_width] + Q_mat_ptr[6] * y[4 + 6 * image_width] + Q_mat_ptr[7] * y[4 + 7 * image_width]) * Q_mat_ptr[12] + (Q_mat_ptr[0] * y[5] + Q_mat_ptr[1] * y[5 + image_width] + Q_mat_ptr[2] * y[5 + 2 * image_width] + Q_mat_ptr[3] * y[5 + 3 * image_width] + Q_mat_ptr[4] * y[5 + 4 * image_width] + Q_mat_ptr[5] * y[5 + 5 * image_width] + Q_mat_ptr[6] * y[5 + 6 * image_width] + Q_mat_ptr[7] * y[5 + 7 * image_width]) * Q_mat_ptr[13] + (Q_mat_ptr[0] * y[6] + Q_mat_ptr[1] * y[6 + image_width] + Q_mat_ptr[2] * y[6 + 2 * image_width] + Q_mat_ptr[3] * y[6 + 3 * image_width] + Q_mat_ptr[4] * y[6 + 4 * image_width] + Q_mat_ptr[5] * y[6 + 5 * image_width] + Q_mat_ptr[6] * y[6 + 6 * image_width] + Q_mat_ptr[7] * y[6 + 7 * image_width]) * Q_mat_ptr[14] + (Q_mat_ptr[0] * y[7] + Q_mat_ptr[1] * y[7 + image_width] + Q_mat_ptr[2] * y[7 + 2 * image_width] + Q_mat_ptr[3] * y[7 + 3 * image_width] + Q_mat_ptr[4] * y[7 + 4 * image_width] + Q_mat_ptr[5] * y[7 + 5 * image_width] + Q_mat_ptr[6] * y[7 + 6 * image_width] + Q_mat_ptr[7] * y[7 + 7 * image_width]) * Q_mat_ptr[15];
						S_mat_ptr[2] = (Q_mat_ptr[0] * y[0] + Q_mat_ptr[1] * y[image_width] + Q_mat_ptr[2] * y[2 * image_width] + Q_mat_ptr[3] * y[3 * image_width] + Q_mat_ptr[4] * y[4 * image_width] + Q_mat_ptr[5] * y[5 * image_width] + Q_mat_ptr[6] * y[6 * image_width] + Q_mat_ptr[7] * y[7 * image_width]) * Q_mat_ptr[16] + (Q_mat_ptr[0] * y[1] + Q_mat_ptr[1] * y[1 + image_width] + Q_mat_ptr[2] * y[1 + 2 * image_width] + Q_mat_ptr[3] * y[1 + 3 * image_width] + Q_mat_ptr[4] * y[1 + 4 * image_width] + Q_mat_ptr[5] * y[1 + 5 * image_width] + Q_mat_ptr[6] * y[1 + 6 * image_width] + Q_mat_ptr[7] * y[1 + 7 * image_width]) * Q_mat_ptr[17] + (Q_mat_ptr[0] * y[2] + Q_mat_ptr[1] * y[2 + image_width] + Q_mat_ptr[2] * y[2 + 2 * image_width] + Q_mat_ptr[3] * y[2 + 3 * image_width] + Q_mat_ptr[4] * y[2 + 4 * image_width] + Q_mat_ptr[5] * y[2 + 5 * image_width] + Q_mat_ptr[6] * y[2 + 6 * image_width] + Q_mat_ptr[7] * y[2 + 7 * image_width]) * Q_mat_ptr[18] + (Q_mat_ptr[0] * y[3] + Q_mat_ptr[1] * y[3 + image_width] + Q_mat_ptr[2] * y[3 + 2 * image_width] + Q_mat_ptr[3] * y[3 + 3 * image_width] + Q_mat_ptr[4] * y[3 + 4 * image_width] + Q_mat_ptr[5] * y[3 + 5 * image_width] + Q_mat_ptr[6] * y[3 + 6 * image_width] + Q_mat_ptr[7] * y[3 + 7 * image_width]) * Q_mat_ptr[19] + (Q_mat_ptr[0] * y[4] + Q_mat_ptr[1] * y[4 + image_width] + Q_mat_ptr[2] * y[4 + 2 * image_width] + Q_mat_ptr[3] * y[4 + 3 * image_width] + Q_mat_ptr[4] * y[4 + 4 * image_width] + Q_mat_ptr[5] * y[4 + 5 * image_width] + Q_mat_ptr[6] * y[4 + 6 * image_width] + Q_mat_ptr[7] * y[4 + 7 * image_width]) * Q_mat_ptr[20] + (Q_mat_ptr[0] * y[5] + Q_mat_ptr[1] * y[5 + image_width] + Q_mat_ptr[2] * y[5 + 2 * image_width] + Q_mat_ptr[3] * y[5 + 3 * image_width] + Q_mat_ptr[4] * y[5 + 4 * image_width] + Q_mat_ptr[5] * y[5 + 5 * image_width] + Q_mat_ptr[6] * y[5 + 6 * image_width] + Q_mat_ptr[7] * y[5 + 7 * image_width]) * Q_mat_ptr[21] + (Q_mat_ptr[0] * y[6] + Q_mat_ptr[1] * y[6 + image_width] + Q_mat_ptr[2] * y[6 + 2 * image_width] + Q_mat_ptr[3] * y[6 + 3 * image_width] + Q_mat_ptr[4] * y[6 + 4 * image_width] + Q_mat_ptr[5] * y[6 + 5 * image_width] + Q_mat_ptr[6] * y[6 + 6 * image_width] + Q_mat_ptr[7] * y[6 + 7 * image_width]) * Q_mat_ptr[22] + (Q_mat_ptr[0] * y[7] + Q_mat_ptr[1] * y[7 + image_width] + Q_mat_ptr[2] * y[7 + 2 * image_width] + Q_mat_ptr[3] * y[7 + 3 * image_width] + Q_mat_ptr[4] * y[7 + 4 * image_width] + Q_mat_ptr[5] * y[7 + 5 * image_width] + Q_mat_ptr[6] * y[7 + 6 * image_width] + Q_mat_ptr[7] * y[7 + 7 * image_width]) * Q_mat_ptr[23];
						S_mat_ptr[3] = Q_mat_ptr[0] * y[3] + Q_mat_ptr[1] * y[3 + image_width] + Q_mat_ptr[2] * y[3 + 2 * image_width] + Q_mat_ptr[3] * y[3 + 3 * image_width] + Q_mat_ptr[4] * y[3 + 4 * image_width] + Q_mat_ptr[5] * y[3 + 5 * image_width] + Q_mat_ptr[6] * y[3 + 6 * image_width] + Q_mat_ptr[7] * y[3 + 7 * image_width];
						S_mat_ptr[4] = (Q_mat_ptr[8] * y[0] + Q_mat_ptr[9] * y[image_width] + Q_mat_ptr[10] * y[2 * image_width] + Q_mat_ptr[11] * y[3 * image_width] + Q_mat_ptr[12] * y[4 * image_width] + Q_mat_ptr[13] * y[5 * image_width] + Q_mat_ptr[14] * y[6 * image_width] + Q_mat_ptr[15] * y[7 * image_width]) * Q_mat_ptr[0] + (Q_mat_ptr[8] * y[1] + Q_mat_ptr[9] * y[1 + image_width] + Q_mat_ptr[10] * y[1 + 2 * image_width] + Q_mat_ptr[11] * y[1 + 3 * image_width] + Q_mat_ptr[12] * y[1 + 4 * image_width] + Q_mat_ptr[13] * y[1 + 5 * image_width] + Q_mat_ptr[14] * y[1 + 6 * image_width] + Q_mat_ptr[15] * y[1 + 7 * image_width]) * Q_mat_ptr[1] + (Q_mat_ptr[8] * y[2] + Q_mat_ptr[9] * y[2 + image_width] + Q_mat_ptr[10] * y[2 + 2 * image_width] + Q_mat_ptr[11] * y[2 + 3 * image_width] + Q_mat_ptr[12] * y[2 + 4 * image_width] + Q_mat_ptr[13] * y[2 + 5 * image_width] + Q_mat_ptr[14] * y[2 + 6 * image_width] + Q_mat_ptr[15] * y[2 + 7 * image_width]) * Q_mat_ptr[2] + (Q_mat_ptr[8] * y[3] + Q_mat_ptr[9] * y[3 + image_width] + Q_mat_ptr[10] * y[3 + 2 * image_width] + Q_mat_ptr[11] * y[3 + 3 * image_width] + Q_mat_ptr[12] * y[3 + 4 * image_width] + Q_mat_ptr[13] * y[3 + 5 * image_width] + Q_mat_ptr[14] * y[3 + 6 * image_width] + Q_mat_ptr[15] * y[3 + 7 * image_width]) * Q_mat_ptr[3] + (Q_mat_ptr[8] * y[4] + Q_mat_ptr[9] * y[4 + image_width] + Q_mat_ptr[10] * y[4 + 2 * image_width] + Q_mat_ptr[11] * y[4 + 3 * image_width] + Q_mat_ptr[12] * y[4 + 4 * image_width] + Q_mat_ptr[13] * y[4 + 5 * image_width] + Q_mat_ptr[14] * y[4 + 6 * image_width] + Q_mat_ptr[15] * y[4 + 7 * image_width]) * Q_mat_ptr[4] + (Q_mat_ptr[8] * y[5] + Q_mat_ptr[9] * y[5 + image_width] + Q_mat_ptr[10] * y[5 + 2 * image_width] + Q_mat_ptr[11] * y[5 + 3 * image_width] + Q_mat_ptr[12] * y[5 + 4 * image_width] + Q_mat_ptr[13] * y[5 + 5 * image_width] + Q_mat_ptr[14] * y[5 + 6 * image_width] + Q_mat_ptr[15] * y[5 + 7 * image_width]) * Q_mat_ptr[5] + (Q_mat_ptr[8] * y[6] + Q_mat_ptr[9] * y[6 + image_width] + Q_mat_ptr[10] * y[6 + 2 * image_width] + Q_mat_ptr[11] * y[6 + 3 * image_width] + Q_mat_ptr[12] * y[6 + 4 * image_width] + Q_mat_ptr[13] * y[6 + 5 * image_width] + Q_mat_ptr[14] * y[6 + 6 * image_width] + Q_mat_ptr[15] * y[6 + 7 * image_width]) * Q_mat_ptr[6] + (Q_mat_ptr[8] * y[7] + Q_mat_ptr[9] * y[7 + image_width] + Q_mat_ptr[10] * y[7 + 2 * image_width] + Q_mat_ptr[11] * y[7 + 3 * image_width] + Q_mat_ptr[12] * y[7 + 4 * image_width] + Q_mat_ptr[13] * y[7 + 5 * image_width] + Q_mat_ptr[14] * y[7 + 6 * image_width] + Q_mat_ptr[15] * y[7 + 7 * image_width]) * Q_mat_ptr[7];
						S_mat_ptr[5] = (Q_mat_ptr[8] * y[0] + Q_mat_ptr[9] * y[image_width] + Q_mat_ptr[10] * y[2 * image_width] + Q_mat_ptr[11] * y[3 * image_width] + Q_mat_ptr[12] * y[4 * image_width] + Q_mat_ptr[13] * y[5 * image_width] + Q_mat_ptr[14] * y[6 * image_width] + Q_mat_ptr[15] * y[7 * image_width]) * Q_mat_ptr[8] + (Q_mat_ptr[8] * y[1] + Q_mat_ptr[9] * y[1 + image_width] + Q_mat_ptr[10] * y[1 + 2 * image_width] + Q_mat_ptr[11] * y[1 + 3 * image_width] + Q_mat_ptr[12] * y[1 + 4 * image_width] + Q_mat_ptr[13] * y[1 + 5 * image_width] + Q_mat_ptr[14] * y[1 + 6 * image_width] + Q_mat_ptr[15] * y[1 + 7 * image_width]) * Q_mat_ptr[9] + (Q_mat_ptr[8] * y[2] + Q_mat_ptr[9] * y[2 + image_width] + Q_mat_ptr[10] * y[2 + 2 * image_width] + Q_mat_ptr[11] * y[2 + 3 * image_width] + Q_mat_ptr[12] * y[2 + 4 * image_width] + Q_mat_ptr[13] * y[2 + 5 * image_width] + Q_mat_ptr[14] * y[2 + 6 * image_width] + Q_mat_ptr[15] * y[2 + 7 * image_width]) * Q_mat_ptr[10] + (Q_mat_ptr[8] * y[3] + Q_mat_ptr[9] * y[3 + image_width] + Q_mat_ptr[10] * y[3 + 2 * image_width] + Q_mat_ptr[11] * y[3 + 3 * image_width] + Q_mat_ptr[12] * y[3 + 4 * image_width] + Q_mat_ptr[13] * y[3 + 5 * image_width] + Q_mat_ptr[14] * y[3 + 6 * image_width] + Q_mat_ptr[15] * y[3 + 7 * image_width]) * Q_mat_ptr[11] + (Q_mat_ptr[8] * y[4] + Q_mat_ptr[9] * y[4 + image_width] + Q_mat_ptr[10] * y[4 + 2 * image_width] + Q_mat_ptr[11] * y[4 + 3 * image_width] + Q_mat_ptr[12] * y[4 + 4 * image_width] + Q_mat_ptr[13] * y[4 + 5 * image_width] + Q_mat_ptr[14] * y[4 + 6 * image_width] + Q_mat_ptr[15] * y[4 + 7 * image_width]) * Q_mat_ptr[12] + (Q_mat_ptr[8] * y[5] + Q_mat_ptr[9] * y[5 + image_width] + Q_mat_ptr[10] * y[5 + 2 * image_width] + Q_mat_ptr[11] * y[5 + 3 * image_width] + Q_mat_ptr[12] * y[5 + 4 * image_width] + Q_mat_ptr[13] * y[5 + 5 * image_width] + Q_mat_ptr[14] * y[5 + 6 * image_width] + Q_mat_ptr[15] * y[5 + 7 * image_width]) * Q_mat_ptr[13] + (Q_mat_ptr[8] * y[6] + Q_mat_ptr[9] * y[6 + image_width] + Q_mat_ptr[10] * y[6 + 2 * image_width] + Q_mat_ptr[11] * y[6 + 3 * image_width] + Q_mat_ptr[12] * y[6 + 4 * image_width] + Q_mat_ptr[13] * y[6 + 5 * image_width] + Q_mat_ptr[14] * y[6 + 6 * image_width] + Q_mat_ptr[15] * y[6 + 7 * image_width]) * Q_mat_ptr[14] + (Q_mat_ptr[8] * y[7] + Q_mat_ptr[9] * y[7 + image_width] + Q_mat_ptr[10] * y[7 + 2 * image_width] + Q_mat_ptr[11] * y[7 + 3 * image_width] + Q_mat_ptr[12] * y[7 + 4 * image_width] + Q_mat_ptr[13] * y[7 + 5 * image_width] + Q_mat_ptr[14] * y[7 + 6 * image_width] + Q_mat_ptr[15] * y[7 + 7 * image_width]) * Q_mat_ptr[15];
						S_mat_ptr[6] = (Q_mat_ptr[8] * y[0] + Q_mat_ptr[9] * y[image_width] + Q_mat_ptr[10] * y[2 * image_width] + Q_mat_ptr[11] * y[3 * image_width] + Q_mat_ptr[12] * y[4 * image_width] + Q_mat_ptr[13] * y[5 * image_width] + Q_mat_ptr[14] * y[6 * image_width] + Q_mat_ptr[15] * y[7 * image_width]) * Q_mat_ptr[16] + (Q_mat_ptr[8] * y[1] + Q_mat_ptr[9] * y[1 + image_width] + Q_mat_ptr[10] * y[1 + 2 * image_width] + Q_mat_ptr[11] * y[1 + 3 * image_width] + Q_mat_ptr[12] * y[1 + 4 * image_width] + Q_mat_ptr[13] * y[1 + 5 * image_width] + Q_mat_ptr[14] * y[1 + 6 * image_width] + Q_mat_ptr[15] * y[1 + 7 * image_width]) * Q_mat_ptr[17] + (Q_mat_ptr[8] * y[2] + Q_mat_ptr[9] * y[2 + image_width] + Q_mat_ptr[10] * y[2 + 2 * image_width] + Q_mat_ptr[11] * y[2 + 3 * image_width] + Q_mat_ptr[12] * y[2 + 4 * image_width] + Q_mat_ptr[13] * y[2 + 5 * image_width] + Q_mat_ptr[14] * y[2 + 6 * image_width] + Q_mat_ptr[15] * y[2 + 7 * image_width]) * Q_mat_ptr[18] + (Q_mat_ptr[8] * y[3] + Q_mat_ptr[9] * y[3 + image_width] + Q_mat_ptr[10] * y[3 + 2 * image_width] + Q_mat_ptr[11] * y[3 + 3 * image_width] + Q_mat_ptr[12] * y[3 + 4 * image_width] + Q_mat_ptr[13] * y[3 + 5 * image_width] + Q_mat_ptr[14] * y[3 + 6 * image_width] + Q_mat_ptr[15] * y[3 + 7 * image_width]) * Q_mat_ptr[19] + (Q_mat_ptr[8] * y[4] + Q_mat_ptr[9] * y[4 + image_width] + Q_mat_ptr[10] * y[4 + 2 * image_width] + Q_mat_ptr[11] * y[4 + 3 * image_width] + Q_mat_ptr[12] * y[4 + 4 * image_width] + Q_mat_ptr[13] * y[4 + 5 * image_width] + Q_mat_ptr[14] * y[4 + 6 * image_width] + Q_mat_ptr[15] * y[4 + 7 * image_width]) * Q_mat_ptr[20] + (Q_mat_ptr[8] * y[5] + Q_mat_ptr[9] * y[5 + image_width] + Q_mat_ptr[10] * y[5 + 2 * image_width] + Q_mat_ptr[11] * y[5 + 3 * image_width] + Q_mat_ptr[12] * y[5 + 4 * image_width] + Q_mat_ptr[13] * y[5 + 5 * image_width] + Q_mat_ptr[14] * y[5 + 6 * image_width] + Q_mat_ptr[15] * y[5 + 7 * image_width]) * Q_mat_ptr[21] + (Q_mat_ptr[8] * y[6] + Q_mat_ptr[9] * y[6 + image_width] + Q_mat_ptr[10] * y[6 + 2 * image_width] + Q_mat_ptr[11] * y[6 + 3 * image_width] + Q_mat_ptr[12] * y[6 + 4 * image_width] + Q_mat_ptr[13] * y[6 + 5 * image_width] + Q_mat_ptr[14] * y[6 + 6 * image_width] + Q_mat_ptr[15] * y[6 + 7 * image_width]) * Q_mat_ptr[22] + (Q_mat_ptr[8] * y[7] + Q_mat_ptr[9] * y[7 + image_width] + Q_mat_ptr[10] * y[7 + 2 * image_width] + Q_mat_ptr[11] * y[7 + 3 * image_width] + Q_mat_ptr[12] * y[7 + 4 * image_width] + Q_mat_ptr[13] * y[7 + 5 * image_width] + Q_mat_ptr[14] * y[7 + 6 * image_width] + Q_mat_ptr[15] * y[7 + 7 * image_width]) * Q_mat_ptr[23];
						S_mat_ptr[7] = Q_mat_ptr[8] * y[3] + Q_mat_ptr[9] * y[3 + image_width] + Q_mat_ptr[10] * y[3 + 2 * image_width] + Q_mat_ptr[11] * y[3 + 3 * image_width] + Q_mat_ptr[12] * y[3 + 4 * image_width] + Q_mat_ptr[13] * y[3 + 5 * image_width] + Q_mat_ptr[14] * y[3 + 6 * image_width] + Q_mat_ptr[15] * y[3 + 7 * image_width];
						S_mat_ptr[8] = (Q_mat_ptr[16] * y[0] + Q_mat_ptr[17] * y[image_width] + Q_mat_ptr[18] * y[2 * image_width] + Q_mat_ptr[19] * y[3 * image_width] + Q_mat_ptr[20] * y[4 * image_width] + Q_mat_ptr[21] * y[5 * image_width] + Q_mat_ptr[22] * y[6 * image_width] + Q_mat_ptr[23] * y[7 * image_width]) * Q_mat_ptr[0] + (Q_mat_ptr[16] * y[1] + Q_mat_ptr[17] * y[1 + image_width] + Q_mat_ptr[18] * y[1 + 2 * image_width] + Q_mat_ptr[19] * y[1 + 3 * image_width] + Q_mat_ptr[20] * y[1 + 4 * image_width] + Q_mat_ptr[21] * y[1 + 5 * image_width] + Q_mat_ptr[22] * y[1 + 6 * image_width] + Q_mat_ptr[23] * y[1 + 7 * image_width]) * Q_mat_ptr[1] + (Q_mat_ptr[16] * y[2] + Q_mat_ptr[17] * y[2 + image_width] + Q_mat_ptr[18] * y[2 + 2 * image_width] + Q_mat_ptr[19] * y[2 + 3 * image_width] + Q_mat_ptr[20] * y[2 + 4 * image_width] + Q_mat_ptr[21] * y[2 + 5 * image_width] + Q_mat_ptr[22] * y[2 + 6 * image_width] + Q_mat_ptr[23] * y[2 + 7 * image_width]) * Q_mat_ptr[2] + (Q_mat_ptr[16] * y[3] + Q_mat_ptr[17] * y[3 + image_width] + Q_mat_ptr[18] * y[3 + 2 * image_width] + Q_mat_ptr[19] * y[3 + 3 * image_width] + Q_mat_ptr[20] * y[3 + 4 * image_width] + Q_mat_ptr[21] * y[3 + 5 * image_width] + Q_mat_ptr[22] * y[3 + 6 * image_width] + Q_mat_ptr[23] * y[3 + 7 * image_width]) * Q_mat_ptr[3] + (Q_mat_ptr[16] * y[4] + Q_mat_ptr[17] * y[4 + image_width] + Q_mat_ptr[18] * y[4 + 2 * image_width] + Q_mat_ptr[19] * y[4 + 3 * image_width] + Q_mat_ptr[20] * y[4 + 4 * image_width] + Q_mat_ptr[21] * y[4 + 5 * image_width] + Q_mat_ptr[22] * y[4 + 6 * image_width] + Q_mat_ptr[23] * y[4 + 7 * image_width]) * Q_mat_ptr[4] + (Q_mat_ptr[16] * y[5] + Q_mat_ptr[17] * y[5 + image_width] + Q_mat_ptr[18] * y[5 + 2 * image_width] + Q_mat_ptr[19] * y[5 + 3 * image_width] + Q_mat_ptr[20] * y[5 + 4 * image_width] + Q_mat_ptr[21] * y[5 + 5 * image_width] + Q_mat_ptr[22] * y[5 + 6 * image_width] + Q_mat_ptr[23] * y[5 + 7 * image_width]) * Q_mat_ptr[5] + (Q_mat_ptr[16] * y[6] + Q_mat_ptr[17] * y[6 + image_width] + Q_mat_ptr[18] * y[6 + 2 * image_width] + Q_mat_ptr[19] * y[6 + 3 * image_width] + Q_mat_ptr[20] * y[6 + 4 * image_width] + Q_mat_ptr[21] * y[6 + 5 * image_width] + Q_mat_ptr[22] * y[6 + 6 * image_width] + Q_mat_ptr[23] * y[6 + 7 * image_width]) * Q_mat_ptr[6] + (Q_mat_ptr[16] * y[7] + Q_mat_ptr[17] * y[7 + image_width] + Q_mat_ptr[18] * y[7 + 2 * image_width] + Q_mat_ptr[19] * y[7 + 3 * image_width] + Q_mat_ptr[20] * y[7 + 4 * image_width] + Q_mat_ptr[21] * y[7 + 5 * image_width] + Q_mat_ptr[22] * y[7 + 6 * image_width] + Q_mat_ptr[23] * y[7 + 7 * image_width]) * Q_mat_ptr[7];
						S_mat_ptr[9] = (Q_mat_ptr[16] * y[0] + Q_mat_ptr[17] * y[image_width] + Q_mat_ptr[18] * y[2 * image_width] + Q_mat_ptr[19] * y[3 * image_width] + Q_mat_ptr[20] * y[4 * image_width] + Q_mat_ptr[21] * y[5 * image_width] + Q_mat_ptr[22] * y[6 * image_width] + Q_mat_ptr[23] * y[7 * image_width]) * Q_mat_ptr[8] + (Q_mat_ptr[16] * y[1] + Q_mat_ptr[17] * y[1 + image_width] + Q_mat_ptr[18] * y[1 + 2 * image_width] + Q_mat_ptr[19] * y[1 + 3 * image_width] + Q_mat_ptr[20] * y[1 + 4 * image_width] + Q_mat_ptr[21] * y[1 + 5 * image_width] + Q_mat_ptr[22] * y[1 + 6 * image_width] + Q_mat_ptr[23] * y[1 + 7 * image_width]) * Q_mat_ptr[9] + (Q_mat_ptr[16] * y[2] + Q_mat_ptr[17] * y[2 + image_width] + Q_mat_ptr[18] * y[2 + 2 * image_width] + Q_mat_ptr[19] * y[2 + 3 * image_width] + Q_mat_ptr[20] * y[2 + 4 * image_width] + Q_mat_ptr[21] * y[2 + 5 * image_width] + Q_mat_ptr[22] * y[2 + 6 * image_width] + Q_mat_ptr[23] * y[2 + 7 * image_width]) * Q_mat_ptr[10] + (Q_mat_ptr[16] * y[3] + Q_mat_ptr[17] * y[3 + image_width] + Q_mat_ptr[18] * y[3 + 2 * image_width] + Q_mat_ptr[19] * y[3 + 3 * image_width] + Q_mat_ptr[20] * y[3 + 4 * image_width] + Q_mat_ptr[21] * y[3 + 5 * image_width] + Q_mat_ptr[22] * y[3 + 6 * image_width] + Q_mat_ptr[23] * y[3 + 7 * image_width]) * Q_mat_ptr[11] + (Q_mat_ptr[16] * y[4] + Q_mat_ptr[17] * y[4 + image_width] + Q_mat_ptr[18] * y[4 + 2 * image_width] + Q_mat_ptr[19] * y[4 + 3 * image_width] + Q_mat_ptr[20] * y[4 + 4 * image_width] + Q_mat_ptr[21] * y[4 + 5 * image_width] + Q_mat_ptr[22] * y[4 + 6 * image_width] + Q_mat_ptr[23] * y[4 + 7 * image_width]) * Q_mat_ptr[12] + (Q_mat_ptr[16] * y[5] + Q_mat_ptr[17] * y[5 + image_width] + Q_mat_ptr[18] * y[5 + 2 * image_width] + Q_mat_ptr[19] * y[5 + 3 * image_width] + Q_mat_ptr[20] * y[5 + 4 * image_width] + Q_mat_ptr[21] * y[5 + 5 * image_width] + Q_mat_ptr[22] * y[5 + 6 * image_width] + Q_mat_ptr[23] * y[5 + 7 * image_width]) * Q_mat_ptr[13] + (Q_mat_ptr[16] * y[6] + Q_mat_ptr[17] * y[6 + image_width] + Q_mat_ptr[18] * y[6 + 2 * image_width] + Q_mat_ptr[19] * y[6 + 3 * image_width] + Q_mat_ptr[20] * y[6 + 4 * image_width] + Q_mat_ptr[21] * y[6 + 5 * image_width] + Q_mat_ptr[22] * y[6 + 6 * image_width] + Q_mat_ptr[23] * y[6 + 7 * image_width]) * Q_mat_ptr[14] + (Q_mat_ptr[16] * y[7] + Q_mat_ptr[17] * y[7 + image_width] + Q_mat_ptr[18] * y[7 + 2 * image_width] + Q_mat_ptr[19] * y[7 + 3 * image_width] + Q_mat_ptr[20] * y[7 + 4 * image_width] + Q_mat_ptr[21] * y[7 + 5 * image_width] + Q_mat_ptr[22] * y[7 + 6 * image_width] + Q_mat_ptr[23] * y[7 + 7 * image_width]) * Q_mat_ptr[15];
						S_mat_ptr[10] = (Q_mat_ptr[16] * y[0] + Q_mat_ptr[17] * y[image_width] + Q_mat_ptr[18] * y[2 * image_width] + Q_mat_ptr[19] * y[3 * image_width] + Q_mat_ptr[20] * y[4 * image_width] + Q_mat_ptr[21] * y[5 * image_width] + Q_mat_ptr[22] * y[6 * image_width] + Q_mat_ptr[23] * y[7 * image_width]) * Q_mat_ptr[16] + (Q_mat_ptr[16] * y[1] + Q_mat_ptr[17] * y[1 + image_width] + Q_mat_ptr[18] * y[1 + 2 * image_width] + Q_mat_ptr[19] * y[1 + 3 * image_width] + Q_mat_ptr[20] * y[1 + 4 * image_width] + Q_mat_ptr[21] * y[1 + 5 * image_width] + Q_mat_ptr[22] * y[1 + 6 * image_width] + Q_mat_ptr[23] * y[1 + 7 * image_width]) * Q_mat_ptr[17] + (Q_mat_ptr[16] * y[2] + Q_mat_ptr[17] * y[2 + image_width] + Q_mat_ptr[18] * y[2 + 2 * image_width] + Q_mat_ptr[19] * y[2 + 3 * image_width] + Q_mat_ptr[20] * y[2 + 4 * image_width] + Q_mat_ptr[21] * y[2 + 5 * image_width] + Q_mat_ptr[22] * y[2 + 6 * image_width] + Q_mat_ptr[23] * y[2 + 7 * image_width]) * Q_mat_ptr[18] + (Q_mat_ptr[16] * y[3] + Q_mat_ptr[17] * y[3 + image_width] + Q_mat_ptr[18] * y[3 + 2 * image_width] + Q_mat_ptr[19] * y[3 + 3 * image_width] + Q_mat_ptr[20] * y[3 + 4 * image_width] + Q_mat_ptr[21] * y[3 + 5 * image_width] + Q_mat_ptr[22] * y[3 + 6 * image_width] + Q_mat_ptr[23] * y[3 + 7 * image_width]) * Q_mat_ptr[19] + (Q_mat_ptr[16] * y[4] + Q_mat_ptr[17] * y[4 + image_width] + Q_mat_ptr[18] * y[4 + 2 * image_width] + Q_mat_ptr[19] * y[4 + 3 * image_width] + Q_mat_ptr[20] * y[4 + 4 * image_width] + Q_mat_ptr[21] * y[4 + 5 * image_width] + Q_mat_ptr[22] * y[4 + 6 * image_width] + Q_mat_ptr[23] * y[4 + 7 * image_width]) * Q_mat_ptr[20] + (Q_mat_ptr[16] * y[5] + Q_mat_ptr[17] * y[5 + image_width] + Q_mat_ptr[18] * y[5 + 2 * image_width] + Q_mat_ptr[19] * y[5 + 3 * image_width] + Q_mat_ptr[20] * y[5 + 4 * image_width] + Q_mat_ptr[21] * y[5 + 5 * image_width] + Q_mat_ptr[22] * y[5 + 6 * image_width] + Q_mat_ptr[23] * y[5 + 7 * image_width]) * Q_mat_ptr[21] + (Q_mat_ptr[16] * y[6] + Q_mat_ptr[17] * y[6 + image_width] + Q_mat_ptr[18] * y[6 + 2 * image_width] + Q_mat_ptr[19] * y[6 + 3 * image_width] + Q_mat_ptr[20] * y[6 + 4 * image_width] + Q_mat_ptr[21] * y[6 + 5 * image_width] + Q_mat_ptr[22] * y[6 + 6 * image_width] + Q_mat_ptr[23] * y[6 + 7 * image_width]) * Q_mat_ptr[22] + (Q_mat_ptr[16] * y[7] + Q_mat_ptr[17] * y[7 + image_width] + Q_mat_ptr[18] * y[7 + 2 * image_width] + Q_mat_ptr[19] * y[7 + 3 * image_width] + Q_mat_ptr[20] * y[7 + 4 * image_width] + Q_mat_ptr[21] * y[7 + 5 * image_width] + Q_mat_ptr[22] * y[7 + 6 * image_width] + Q_mat_ptr[23] * y[7 + 7 * image_width]) * Q_mat_ptr[23];
						S_mat_ptr[11] = Q_mat_ptr[16] * y[3] + Q_mat_ptr[17] * y[3 + image_width] + Q_mat_ptr[18] * y[3 + 2 * image_width] + Q_mat_ptr[19] * y[3 + 3 * image_width] + Q_mat_ptr[20] * y[3 + 4 * image_width] + Q_mat_ptr[21] * y[3 + 5 * image_width] + Q_mat_ptr[22] * y[3 + 6 * image_width] + Q_mat_ptr[23] * y[3 + 7 * image_width];
						S_mat_ptr[12] = y[3 * image_width] * Q_mat_ptr[0] + y[1 + 3 * image_width] * Q_mat_ptr[1] + y[2 + 3 * image_width] * Q_mat_ptr[2] + Q_mat_ptr[3] * y[3 + 3 * image_width] + y[4 + 3 * image_width] * Q_mat_ptr[4] + y[5 + 3 * image_width] * Q_mat_ptr[5] + y[6 + 3 * image_width] * Q_mat_ptr[6] + y[7 + 3 * image_width] * Q_mat_ptr[7];
						S_mat_ptr[13] = y[3 * image_width] * Q_mat_ptr[8] + y[1 + 3 * image_width] * Q_mat_ptr[9] + y[2 + 3 * image_width] * Q_mat_ptr[10] + Q_mat_ptr[11] * y[3 + 3 * image_width] + y[4 + 3 * image_width] * Q_mat_ptr[12] + y[5 + 3 * image_width] * Q_mat_ptr[13] + y[6 + 3 * image_width] * Q_mat_ptr[14] + y[7 + 3 * image_width] * Q_mat_ptr[15];
						S_mat_ptr[14] = y[3 * image_width] * Q_mat_ptr[16] + y[1 + 3 * image_width] * Q_mat_ptr[17] + y[2 + 3 * image_width] * Q_mat_ptr[18] + Q_mat_ptr[19] * y[3 + 3 * image_width] + y[4 + 3 * image_width] * Q_mat_ptr[20] + y[5 + 3 * image_width] * Q_mat_ptr[21] + y[6 + 3 * image_width] * Q_mat_ptr[22] + y[7 + 3 * image_width] * Q_mat_ptr[23];
						S_mat_ptr[15] = y[3 + 3 * image_width];
					}
					CV_MAT_ELEM(*lut_valid_sign_mat,uchar,index_y - aoi_y_min,index_x - aoi_x_min) =1;		
				}
				CV_MAT_ELEM(*g_mat,double,i,j) = 
					delta_x*delta_x*delta_x * (delta_y*delta_y*delta_y * S_mat_ptr[0] + delta_y*delta_y * S_mat_ptr[4] + delta_y * S_mat_ptr[8] + S_mat_ptr[12] ) +
					delta_x*delta_x * (delta_y*delta_y*delta_y * S_mat_ptr[1] + delta_y*delta_y * S_mat_ptr[5] + delta_y * S_mat_ptr[9] + S_mat_ptr[13] ) +
					delta_x  * (delta_y*delta_y*delta_y * S_mat_ptr[2] + delta_y*delta_y * S_mat_ptr[6] + delta_y * S_mat_ptr[10] + S_mat_ptr[14] ) +
					(delta_y*delta_y*delta_y * S_mat_ptr[3] + delta_y*delta_y * S_mat_ptr[7] + delta_y * S_mat_ptr[11] + S_mat_ptr[15] );
			}
		}
	}
	else
	{
		for (int i =0;i<template_height;i++)
		{
			for (int j =0;j<template_width;j++)
			{
				//第一种
				double pos_x =reference_point_x + j - template_half_width + u + ux * (j - template_half_width) + uy * (i - template_half_height);
				double pos_y =reference_point_y + i - template_half_height + v + vx * (j - template_half_width) + vy * (i - template_half_height);

				if (pos_x<aoi_x_min || pos_x>=aoi_x_max||
					pos_y<aoi_y_min || pos_y>=aoi_y_max)//右侧为开区间
				{
					return false;
				}
				int index_x = cvFloor(pos_x);
				int index_y = cvFloor(pos_y);

				double delta_x = pos_x - index_x;
				double delta_y = pos_y - index_y;
				uchar sign = CV_MAT_ELEM(*lut_valid_sign_mat,uchar,index_y - aoi_y_min,index_x - aoi_x_min);
				double* S_mat_ptr = lut_mat->data.db+ ((index_y - aoi_y_min) * aoi_width+index_x - aoi_x_min) * lut_element_size_;
				if (sign!=1)
				{
					double* y;
					if (Q_mat_type==1)
					{
						y = image->data.db + (index_y-2)*image_width+index_x-2;
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

					} 
					else
					{
						y = image->data.db + (index_y-3)*image_width+index_x-3;
						S_mat_ptr[0] = (Q_mat_ptr[0] * y[0] + Q_mat_ptr[1] * y[image_width] + Q_mat_ptr[2] * y[2 * image_width] + Q_mat_ptr[3] * y[3 * image_width] + Q_mat_ptr[4] * y[4 * image_width] + Q_mat_ptr[5] * y[5 * image_width] + Q_mat_ptr[6] * y[6 * image_width] + Q_mat_ptr[7] * y[7 * image_width]) * Q_mat_ptr[0] + (Q_mat_ptr[0] * y[1] + Q_mat_ptr[1] * y[1 + image_width] + Q_mat_ptr[2] * y[1 + 2 * image_width] + Q_mat_ptr[3] * y[1 + 3 * image_width] + Q_mat_ptr[4] * y[1 + 4 * image_width] + Q_mat_ptr[5] * y[1 + 5 * image_width] + Q_mat_ptr[6] * y[1 + 6 * image_width] + Q_mat_ptr[7] * y[1 + 7 * image_width]) * Q_mat_ptr[1] + (Q_mat_ptr[0] * y[2] + Q_mat_ptr[1] * y[2 + image_width] + Q_mat_ptr[2] * y[2 + 2 * image_width] + Q_mat_ptr[3] * y[2 + 3 * image_width] + Q_mat_ptr[4] * y[2 + 4 * image_width] + Q_mat_ptr[5] * y[2 + 5 * image_width] + Q_mat_ptr[6] * y[2 + 6 * image_width] + Q_mat_ptr[7] * y[2 + 7 * image_width]) * Q_mat_ptr[2] + (Q_mat_ptr[0] * y[3] + Q_mat_ptr[1] * y[3 + image_width] + Q_mat_ptr[2] * y[3 + 2 * image_width] + Q_mat_ptr[3] * y[3 + 3 * image_width] + Q_mat_ptr[4] * y[3 + 4 * image_width] + Q_mat_ptr[5] * y[3 + 5 * image_width] + Q_mat_ptr[6] * y[3 + 6 * image_width] + Q_mat_ptr[7] * y[3 + 7 * image_width]) * Q_mat_ptr[3] + (Q_mat_ptr[0] * y[4] + Q_mat_ptr[1] * y[4 + image_width] + Q_mat_ptr[2] * y[4 + 2 * image_width] + Q_mat_ptr[3] * y[4 + 3 * image_width] + Q_mat_ptr[4] * y[4 + 4 * image_width] + Q_mat_ptr[5] * y[4 + 5 * image_width] + Q_mat_ptr[6] * y[4 + 6 * image_width] + Q_mat_ptr[7] * y[4 + 7 * image_width]) * Q_mat_ptr[4] + (Q_mat_ptr[0] * y[5] + Q_mat_ptr[1] * y[5 + image_width] + Q_mat_ptr[2] * y[5 + 2 * image_width] + Q_mat_ptr[3] * y[5 + 3 * image_width] + Q_mat_ptr[4] * y[5 + 4 * image_width] + Q_mat_ptr[5] * y[5 + 5 * image_width] + Q_mat_ptr[6] * y[5 + 6 * image_width] + Q_mat_ptr[7] * y[5 + 7 * image_width]) * Q_mat_ptr[5] + (Q_mat_ptr[0] * y[6] + Q_mat_ptr[1] * y[6 + image_width] + Q_mat_ptr[2] * y[6 + 2 * image_width] + Q_mat_ptr[3] * y[6 + 3 * image_width] + Q_mat_ptr[4] * y[6 + 4 * image_width] + Q_mat_ptr[5] * y[6 + 5 * image_width] + Q_mat_ptr[6] * y[6 + 6 * image_width] + Q_mat_ptr[7] * y[6 + 7 * image_width]) * Q_mat_ptr[6] + (Q_mat_ptr[0] * y[7] + Q_mat_ptr[1] * y[7 + image_width] + Q_mat_ptr[2] * y[7 + 2 * image_width] + Q_mat_ptr[3] * y[7 + 3 * image_width] + Q_mat_ptr[4] * y[7 + 4 * image_width] + Q_mat_ptr[5] * y[7 + 5 * image_width] + Q_mat_ptr[6] * y[7 + 6 * image_width] + Q_mat_ptr[7] * y[7 + 7 * image_width]) * Q_mat_ptr[7];
						S_mat_ptr[1] = (Q_mat_ptr[0] * y[0] + Q_mat_ptr[1] * y[image_width] + Q_mat_ptr[2] * y[2 * image_width] + Q_mat_ptr[3] * y[3 * image_width] + Q_mat_ptr[4] * y[4 * image_width] + Q_mat_ptr[5] * y[5 * image_width] + Q_mat_ptr[6] * y[6 * image_width] + Q_mat_ptr[7] * y[7 * image_width]) * Q_mat_ptr[8] + (Q_mat_ptr[0] * y[1] + Q_mat_ptr[1] * y[1 + image_width] + Q_mat_ptr[2] * y[1 + 2 * image_width] + Q_mat_ptr[3] * y[1 + 3 * image_width] + Q_mat_ptr[4] * y[1 + 4 * image_width] + Q_mat_ptr[5] * y[1 + 5 * image_width] + Q_mat_ptr[6] * y[1 + 6 * image_width] + Q_mat_ptr[7] * y[1 + 7 * image_width]) * Q_mat_ptr[9] + (Q_mat_ptr[0] * y[2] + Q_mat_ptr[1] * y[2 + image_width] + Q_mat_ptr[2] * y[2 + 2 * image_width] + Q_mat_ptr[3] * y[2 + 3 * image_width] + Q_mat_ptr[4] * y[2 + 4 * image_width] + Q_mat_ptr[5] * y[2 + 5 * image_width] + Q_mat_ptr[6] * y[2 + 6 * image_width] + Q_mat_ptr[7] * y[2 + 7 * image_width]) * Q_mat_ptr[10] + (Q_mat_ptr[0] * y[3] + Q_mat_ptr[1] * y[3 + image_width] + Q_mat_ptr[2] * y[3 + 2 * image_width] + Q_mat_ptr[3] * y[3 + 3 * image_width] + Q_mat_ptr[4] * y[3 + 4 * image_width] + Q_mat_ptr[5] * y[3 + 5 * image_width] + Q_mat_ptr[6] * y[3 + 6 * image_width] + Q_mat_ptr[7] * y[3 + 7 * image_width]) * Q_mat_ptr[11] + (Q_mat_ptr[0] * y[4] + Q_mat_ptr[1] * y[4 + image_width] + Q_mat_ptr[2] * y[4 + 2 * image_width] + Q_mat_ptr[3] * y[4 + 3 * image_width] + Q_mat_ptr[4] * y[4 + 4 * image_width] + Q_mat_ptr[5] * y[4 + 5 * image_width] + Q_mat_ptr[6] * y[4 + 6 * image_width] + Q_mat_ptr[7] * y[4 + 7 * image_width]) * Q_mat_ptr[12] + (Q_mat_ptr[0] * y[5] + Q_mat_ptr[1] * y[5 + image_width] + Q_mat_ptr[2] * y[5 + 2 * image_width] + Q_mat_ptr[3] * y[5 + 3 * image_width] + Q_mat_ptr[4] * y[5 + 4 * image_width] + Q_mat_ptr[5] * y[5 + 5 * image_width] + Q_mat_ptr[6] * y[5 + 6 * image_width] + Q_mat_ptr[7] * y[5 + 7 * image_width]) * Q_mat_ptr[13] + (Q_mat_ptr[0] * y[6] + Q_mat_ptr[1] * y[6 + image_width] + Q_mat_ptr[2] * y[6 + 2 * image_width] + Q_mat_ptr[3] * y[6 + 3 * image_width] + Q_mat_ptr[4] * y[6 + 4 * image_width] + Q_mat_ptr[5] * y[6 + 5 * image_width] + Q_mat_ptr[6] * y[6 + 6 * image_width] + Q_mat_ptr[7] * y[6 + 7 * image_width]) * Q_mat_ptr[14] + (Q_mat_ptr[0] * y[7] + Q_mat_ptr[1] * y[7 + image_width] + Q_mat_ptr[2] * y[7 + 2 * image_width] + Q_mat_ptr[3] * y[7 + 3 * image_width] + Q_mat_ptr[4] * y[7 + 4 * image_width] + Q_mat_ptr[5] * y[7 + 5 * image_width] + Q_mat_ptr[6] * y[7 + 6 * image_width] + Q_mat_ptr[7] * y[7 + 7 * image_width]) * Q_mat_ptr[15];
						S_mat_ptr[2] = (Q_mat_ptr[0] * y[0] + Q_mat_ptr[1] * y[image_width] + Q_mat_ptr[2] * y[2 * image_width] + Q_mat_ptr[3] * y[3 * image_width] + Q_mat_ptr[4] * y[4 * image_width] + Q_mat_ptr[5] * y[5 * image_width] + Q_mat_ptr[6] * y[6 * image_width] + Q_mat_ptr[7] * y[7 * image_width]) * Q_mat_ptr[16] + (Q_mat_ptr[0] * y[1] + Q_mat_ptr[1] * y[1 + image_width] + Q_mat_ptr[2] * y[1 + 2 * image_width] + Q_mat_ptr[3] * y[1 + 3 * image_width] + Q_mat_ptr[4] * y[1 + 4 * image_width] + Q_mat_ptr[5] * y[1 + 5 * image_width] + Q_mat_ptr[6] * y[1 + 6 * image_width] + Q_mat_ptr[7] * y[1 + 7 * image_width]) * Q_mat_ptr[17] + (Q_mat_ptr[0] * y[2] + Q_mat_ptr[1] * y[2 + image_width] + Q_mat_ptr[2] * y[2 + 2 * image_width] + Q_mat_ptr[3] * y[2 + 3 * image_width] + Q_mat_ptr[4] * y[2 + 4 * image_width] + Q_mat_ptr[5] * y[2 + 5 * image_width] + Q_mat_ptr[6] * y[2 + 6 * image_width] + Q_mat_ptr[7] * y[2 + 7 * image_width]) * Q_mat_ptr[18] + (Q_mat_ptr[0] * y[3] + Q_mat_ptr[1] * y[3 + image_width] + Q_mat_ptr[2] * y[3 + 2 * image_width] + Q_mat_ptr[3] * y[3 + 3 * image_width] + Q_mat_ptr[4] * y[3 + 4 * image_width] + Q_mat_ptr[5] * y[3 + 5 * image_width] + Q_mat_ptr[6] * y[3 + 6 * image_width] + Q_mat_ptr[7] * y[3 + 7 * image_width]) * Q_mat_ptr[19] + (Q_mat_ptr[0] * y[4] + Q_mat_ptr[1] * y[4 + image_width] + Q_mat_ptr[2] * y[4 + 2 * image_width] + Q_mat_ptr[3] * y[4 + 3 * image_width] + Q_mat_ptr[4] * y[4 + 4 * image_width] + Q_mat_ptr[5] * y[4 + 5 * image_width] + Q_mat_ptr[6] * y[4 + 6 * image_width] + Q_mat_ptr[7] * y[4 + 7 * image_width]) * Q_mat_ptr[20] + (Q_mat_ptr[0] * y[5] + Q_mat_ptr[1] * y[5 + image_width] + Q_mat_ptr[2] * y[5 + 2 * image_width] + Q_mat_ptr[3] * y[5 + 3 * image_width] + Q_mat_ptr[4] * y[5 + 4 * image_width] + Q_mat_ptr[5] * y[5 + 5 * image_width] + Q_mat_ptr[6] * y[5 + 6 * image_width] + Q_mat_ptr[7] * y[5 + 7 * image_width]) * Q_mat_ptr[21] + (Q_mat_ptr[0] * y[6] + Q_mat_ptr[1] * y[6 + image_width] + Q_mat_ptr[2] * y[6 + 2 * image_width] + Q_mat_ptr[3] * y[6 + 3 * image_width] + Q_mat_ptr[4] * y[6 + 4 * image_width] + Q_mat_ptr[5] * y[6 + 5 * image_width] + Q_mat_ptr[6] * y[6 + 6 * image_width] + Q_mat_ptr[7] * y[6 + 7 * image_width]) * Q_mat_ptr[22] + (Q_mat_ptr[0] * y[7] + Q_mat_ptr[1] * y[7 + image_width] + Q_mat_ptr[2] * y[7 + 2 * image_width] + Q_mat_ptr[3] * y[7 + 3 * image_width] + Q_mat_ptr[4] * y[7 + 4 * image_width] + Q_mat_ptr[5] * y[7 + 5 * image_width] + Q_mat_ptr[6] * y[7 + 6 * image_width] + Q_mat_ptr[7] * y[7 + 7 * image_width]) * Q_mat_ptr[23];
						S_mat_ptr[3] = Q_mat_ptr[0] * y[3] + Q_mat_ptr[1] * y[3 + image_width] + Q_mat_ptr[2] * y[3 + 2 * image_width] + Q_mat_ptr[3] * y[3 + 3 * image_width] + Q_mat_ptr[4] * y[3 + 4 * image_width] + Q_mat_ptr[5] * y[3 + 5 * image_width] + Q_mat_ptr[6] * y[3 + 6 * image_width] + Q_mat_ptr[7] * y[3 + 7 * image_width];
						S_mat_ptr[4] = (Q_mat_ptr[8] * y[0] + Q_mat_ptr[9] * y[image_width] + Q_mat_ptr[10] * y[2 * image_width] + Q_mat_ptr[11] * y[3 * image_width] + Q_mat_ptr[12] * y[4 * image_width] + Q_mat_ptr[13] * y[5 * image_width] + Q_mat_ptr[14] * y[6 * image_width] + Q_mat_ptr[15] * y[7 * image_width]) * Q_mat_ptr[0] + (Q_mat_ptr[8] * y[1] + Q_mat_ptr[9] * y[1 + image_width] + Q_mat_ptr[10] * y[1 + 2 * image_width] + Q_mat_ptr[11] * y[1 + 3 * image_width] + Q_mat_ptr[12] * y[1 + 4 * image_width] + Q_mat_ptr[13] * y[1 + 5 * image_width] + Q_mat_ptr[14] * y[1 + 6 * image_width] + Q_mat_ptr[15] * y[1 + 7 * image_width]) * Q_mat_ptr[1] + (Q_mat_ptr[8] * y[2] + Q_mat_ptr[9] * y[2 + image_width] + Q_mat_ptr[10] * y[2 + 2 * image_width] + Q_mat_ptr[11] * y[2 + 3 * image_width] + Q_mat_ptr[12] * y[2 + 4 * image_width] + Q_mat_ptr[13] * y[2 + 5 * image_width] + Q_mat_ptr[14] * y[2 + 6 * image_width] + Q_mat_ptr[15] * y[2 + 7 * image_width]) * Q_mat_ptr[2] + (Q_mat_ptr[8] * y[3] + Q_mat_ptr[9] * y[3 + image_width] + Q_mat_ptr[10] * y[3 + 2 * image_width] + Q_mat_ptr[11] * y[3 + 3 * image_width] + Q_mat_ptr[12] * y[3 + 4 * image_width] + Q_mat_ptr[13] * y[3 + 5 * image_width] + Q_mat_ptr[14] * y[3 + 6 * image_width] + Q_mat_ptr[15] * y[3 + 7 * image_width]) * Q_mat_ptr[3] + (Q_mat_ptr[8] * y[4] + Q_mat_ptr[9] * y[4 + image_width] + Q_mat_ptr[10] * y[4 + 2 * image_width] + Q_mat_ptr[11] * y[4 + 3 * image_width] + Q_mat_ptr[12] * y[4 + 4 * image_width] + Q_mat_ptr[13] * y[4 + 5 * image_width] + Q_mat_ptr[14] * y[4 + 6 * image_width] + Q_mat_ptr[15] * y[4 + 7 * image_width]) * Q_mat_ptr[4] + (Q_mat_ptr[8] * y[5] + Q_mat_ptr[9] * y[5 + image_width] + Q_mat_ptr[10] * y[5 + 2 * image_width] + Q_mat_ptr[11] * y[5 + 3 * image_width] + Q_mat_ptr[12] * y[5 + 4 * image_width] + Q_mat_ptr[13] * y[5 + 5 * image_width] + Q_mat_ptr[14] * y[5 + 6 * image_width] + Q_mat_ptr[15] * y[5 + 7 * image_width]) * Q_mat_ptr[5] + (Q_mat_ptr[8] * y[6] + Q_mat_ptr[9] * y[6 + image_width] + Q_mat_ptr[10] * y[6 + 2 * image_width] + Q_mat_ptr[11] * y[6 + 3 * image_width] + Q_mat_ptr[12] * y[6 + 4 * image_width] + Q_mat_ptr[13] * y[6 + 5 * image_width] + Q_mat_ptr[14] * y[6 + 6 * image_width] + Q_mat_ptr[15] * y[6 + 7 * image_width]) * Q_mat_ptr[6] + (Q_mat_ptr[8] * y[7] + Q_mat_ptr[9] * y[7 + image_width] + Q_mat_ptr[10] * y[7 + 2 * image_width] + Q_mat_ptr[11] * y[7 + 3 * image_width] + Q_mat_ptr[12] * y[7 + 4 * image_width] + Q_mat_ptr[13] * y[7 + 5 * image_width] + Q_mat_ptr[14] * y[7 + 6 * image_width] + Q_mat_ptr[15] * y[7 + 7 * image_width]) * Q_mat_ptr[7];
						S_mat_ptr[5] = (Q_mat_ptr[8] * y[0] + Q_mat_ptr[9] * y[image_width] + Q_mat_ptr[10] * y[2 * image_width] + Q_mat_ptr[11] * y[3 * image_width] + Q_mat_ptr[12] * y[4 * image_width] + Q_mat_ptr[13] * y[5 * image_width] + Q_mat_ptr[14] * y[6 * image_width] + Q_mat_ptr[15] * y[7 * image_width]) * Q_mat_ptr[8] + (Q_mat_ptr[8] * y[1] + Q_mat_ptr[9] * y[1 + image_width] + Q_mat_ptr[10] * y[1 + 2 * image_width] + Q_mat_ptr[11] * y[1 + 3 * image_width] + Q_mat_ptr[12] * y[1 + 4 * image_width] + Q_mat_ptr[13] * y[1 + 5 * image_width] + Q_mat_ptr[14] * y[1 + 6 * image_width] + Q_mat_ptr[15] * y[1 + 7 * image_width]) * Q_mat_ptr[9] + (Q_mat_ptr[8] * y[2] + Q_mat_ptr[9] * y[2 + image_width] + Q_mat_ptr[10] * y[2 + 2 * image_width] + Q_mat_ptr[11] * y[2 + 3 * image_width] + Q_mat_ptr[12] * y[2 + 4 * image_width] + Q_mat_ptr[13] * y[2 + 5 * image_width] + Q_mat_ptr[14] * y[2 + 6 * image_width] + Q_mat_ptr[15] * y[2 + 7 * image_width]) * Q_mat_ptr[10] + (Q_mat_ptr[8] * y[3] + Q_mat_ptr[9] * y[3 + image_width] + Q_mat_ptr[10] * y[3 + 2 * image_width] + Q_mat_ptr[11] * y[3 + 3 * image_width] + Q_mat_ptr[12] * y[3 + 4 * image_width] + Q_mat_ptr[13] * y[3 + 5 * image_width] + Q_mat_ptr[14] * y[3 + 6 * image_width] + Q_mat_ptr[15] * y[3 + 7 * image_width]) * Q_mat_ptr[11] + (Q_mat_ptr[8] * y[4] + Q_mat_ptr[9] * y[4 + image_width] + Q_mat_ptr[10] * y[4 + 2 * image_width] + Q_mat_ptr[11] * y[4 + 3 * image_width] + Q_mat_ptr[12] * y[4 + 4 * image_width] + Q_mat_ptr[13] * y[4 + 5 * image_width] + Q_mat_ptr[14] * y[4 + 6 * image_width] + Q_mat_ptr[15] * y[4 + 7 * image_width]) * Q_mat_ptr[12] + (Q_mat_ptr[8] * y[5] + Q_mat_ptr[9] * y[5 + image_width] + Q_mat_ptr[10] * y[5 + 2 * image_width] + Q_mat_ptr[11] * y[5 + 3 * image_width] + Q_mat_ptr[12] * y[5 + 4 * image_width] + Q_mat_ptr[13] * y[5 + 5 * image_width] + Q_mat_ptr[14] * y[5 + 6 * image_width] + Q_mat_ptr[15] * y[5 + 7 * image_width]) * Q_mat_ptr[13] + (Q_mat_ptr[8] * y[6] + Q_mat_ptr[9] * y[6 + image_width] + Q_mat_ptr[10] * y[6 + 2 * image_width] + Q_mat_ptr[11] * y[6 + 3 * image_width] + Q_mat_ptr[12] * y[6 + 4 * image_width] + Q_mat_ptr[13] * y[6 + 5 * image_width] + Q_mat_ptr[14] * y[6 + 6 * image_width] + Q_mat_ptr[15] * y[6 + 7 * image_width]) * Q_mat_ptr[14] + (Q_mat_ptr[8] * y[7] + Q_mat_ptr[9] * y[7 + image_width] + Q_mat_ptr[10] * y[7 + 2 * image_width] + Q_mat_ptr[11] * y[7 + 3 * image_width] + Q_mat_ptr[12] * y[7 + 4 * image_width] + Q_mat_ptr[13] * y[7 + 5 * image_width] + Q_mat_ptr[14] * y[7 + 6 * image_width] + Q_mat_ptr[15] * y[7 + 7 * image_width]) * Q_mat_ptr[15];
						S_mat_ptr[6] = (Q_mat_ptr[8] * y[0] + Q_mat_ptr[9] * y[image_width] + Q_mat_ptr[10] * y[2 * image_width] + Q_mat_ptr[11] * y[3 * image_width] + Q_mat_ptr[12] * y[4 * image_width] + Q_mat_ptr[13] * y[5 * image_width] + Q_mat_ptr[14] * y[6 * image_width] + Q_mat_ptr[15] * y[7 * image_width]) * Q_mat_ptr[16] + (Q_mat_ptr[8] * y[1] + Q_mat_ptr[9] * y[1 + image_width] + Q_mat_ptr[10] * y[1 + 2 * image_width] + Q_mat_ptr[11] * y[1 + 3 * image_width] + Q_mat_ptr[12] * y[1 + 4 * image_width] + Q_mat_ptr[13] * y[1 + 5 * image_width] + Q_mat_ptr[14] * y[1 + 6 * image_width] + Q_mat_ptr[15] * y[1 + 7 * image_width]) * Q_mat_ptr[17] + (Q_mat_ptr[8] * y[2] + Q_mat_ptr[9] * y[2 + image_width] + Q_mat_ptr[10] * y[2 + 2 * image_width] + Q_mat_ptr[11] * y[2 + 3 * image_width] + Q_mat_ptr[12] * y[2 + 4 * image_width] + Q_mat_ptr[13] * y[2 + 5 * image_width] + Q_mat_ptr[14] * y[2 + 6 * image_width] + Q_mat_ptr[15] * y[2 + 7 * image_width]) * Q_mat_ptr[18] + (Q_mat_ptr[8] * y[3] + Q_mat_ptr[9] * y[3 + image_width] + Q_mat_ptr[10] * y[3 + 2 * image_width] + Q_mat_ptr[11] * y[3 + 3 * image_width] + Q_mat_ptr[12] * y[3 + 4 * image_width] + Q_mat_ptr[13] * y[3 + 5 * image_width] + Q_mat_ptr[14] * y[3 + 6 * image_width] + Q_mat_ptr[15] * y[3 + 7 * image_width]) * Q_mat_ptr[19] + (Q_mat_ptr[8] * y[4] + Q_mat_ptr[9] * y[4 + image_width] + Q_mat_ptr[10] * y[4 + 2 * image_width] + Q_mat_ptr[11] * y[4 + 3 * image_width] + Q_mat_ptr[12] * y[4 + 4 * image_width] + Q_mat_ptr[13] * y[4 + 5 * image_width] + Q_mat_ptr[14] * y[4 + 6 * image_width] + Q_mat_ptr[15] * y[4 + 7 * image_width]) * Q_mat_ptr[20] + (Q_mat_ptr[8] * y[5] + Q_mat_ptr[9] * y[5 + image_width] + Q_mat_ptr[10] * y[5 + 2 * image_width] + Q_mat_ptr[11] * y[5 + 3 * image_width] + Q_mat_ptr[12] * y[5 + 4 * image_width] + Q_mat_ptr[13] * y[5 + 5 * image_width] + Q_mat_ptr[14] * y[5 + 6 * image_width] + Q_mat_ptr[15] * y[5 + 7 * image_width]) * Q_mat_ptr[21] + (Q_mat_ptr[8] * y[6] + Q_mat_ptr[9] * y[6 + image_width] + Q_mat_ptr[10] * y[6 + 2 * image_width] + Q_mat_ptr[11] * y[6 + 3 * image_width] + Q_mat_ptr[12] * y[6 + 4 * image_width] + Q_mat_ptr[13] * y[6 + 5 * image_width] + Q_mat_ptr[14] * y[6 + 6 * image_width] + Q_mat_ptr[15] * y[6 + 7 * image_width]) * Q_mat_ptr[22] + (Q_mat_ptr[8] * y[7] + Q_mat_ptr[9] * y[7 + image_width] + Q_mat_ptr[10] * y[7 + 2 * image_width] + Q_mat_ptr[11] * y[7 + 3 * image_width] + Q_mat_ptr[12] * y[7 + 4 * image_width] + Q_mat_ptr[13] * y[7 + 5 * image_width] + Q_mat_ptr[14] * y[7 + 6 * image_width] + Q_mat_ptr[15] * y[7 + 7 * image_width]) * Q_mat_ptr[23];
						S_mat_ptr[7] = Q_mat_ptr[8] * y[3] + Q_mat_ptr[9] * y[3 + image_width] + Q_mat_ptr[10] * y[3 + 2 * image_width] + Q_mat_ptr[11] * y[3 + 3 * image_width] + Q_mat_ptr[12] * y[3 + 4 * image_width] + Q_mat_ptr[13] * y[3 + 5 * image_width] + Q_mat_ptr[14] * y[3 + 6 * image_width] + Q_mat_ptr[15] * y[3 + 7 * image_width];
						S_mat_ptr[8] = (Q_mat_ptr[16] * y[0] + Q_mat_ptr[17] * y[image_width] + Q_mat_ptr[18] * y[2 * image_width] + Q_mat_ptr[19] * y[3 * image_width] + Q_mat_ptr[20] * y[4 * image_width] + Q_mat_ptr[21] * y[5 * image_width] + Q_mat_ptr[22] * y[6 * image_width] + Q_mat_ptr[23] * y[7 * image_width]) * Q_mat_ptr[0] + (Q_mat_ptr[16] * y[1] + Q_mat_ptr[17] * y[1 + image_width] + Q_mat_ptr[18] * y[1 + 2 * image_width] + Q_mat_ptr[19] * y[1 + 3 * image_width] + Q_mat_ptr[20] * y[1 + 4 * image_width] + Q_mat_ptr[21] * y[1 + 5 * image_width] + Q_mat_ptr[22] * y[1 + 6 * image_width] + Q_mat_ptr[23] * y[1 + 7 * image_width]) * Q_mat_ptr[1] + (Q_mat_ptr[16] * y[2] + Q_mat_ptr[17] * y[2 + image_width] + Q_mat_ptr[18] * y[2 + 2 * image_width] + Q_mat_ptr[19] * y[2 + 3 * image_width] + Q_mat_ptr[20] * y[2 + 4 * image_width] + Q_mat_ptr[21] * y[2 + 5 * image_width] + Q_mat_ptr[22] * y[2 + 6 * image_width] + Q_mat_ptr[23] * y[2 + 7 * image_width]) * Q_mat_ptr[2] + (Q_mat_ptr[16] * y[3] + Q_mat_ptr[17] * y[3 + image_width] + Q_mat_ptr[18] * y[3 + 2 * image_width] + Q_mat_ptr[19] * y[3 + 3 * image_width] + Q_mat_ptr[20] * y[3 + 4 * image_width] + Q_mat_ptr[21] * y[3 + 5 * image_width] + Q_mat_ptr[22] * y[3 + 6 * image_width] + Q_mat_ptr[23] * y[3 + 7 * image_width]) * Q_mat_ptr[3] + (Q_mat_ptr[16] * y[4] + Q_mat_ptr[17] * y[4 + image_width] + Q_mat_ptr[18] * y[4 + 2 * image_width] + Q_mat_ptr[19] * y[4 + 3 * image_width] + Q_mat_ptr[20] * y[4 + 4 * image_width] + Q_mat_ptr[21] * y[4 + 5 * image_width] + Q_mat_ptr[22] * y[4 + 6 * image_width] + Q_mat_ptr[23] * y[4 + 7 * image_width]) * Q_mat_ptr[4] + (Q_mat_ptr[16] * y[5] + Q_mat_ptr[17] * y[5 + image_width] + Q_mat_ptr[18] * y[5 + 2 * image_width] + Q_mat_ptr[19] * y[5 + 3 * image_width] + Q_mat_ptr[20] * y[5 + 4 * image_width] + Q_mat_ptr[21] * y[5 + 5 * image_width] + Q_mat_ptr[22] * y[5 + 6 * image_width] + Q_mat_ptr[23] * y[5 + 7 * image_width]) * Q_mat_ptr[5] + (Q_mat_ptr[16] * y[6] + Q_mat_ptr[17] * y[6 + image_width] + Q_mat_ptr[18] * y[6 + 2 * image_width] + Q_mat_ptr[19] * y[6 + 3 * image_width] + Q_mat_ptr[20] * y[6 + 4 * image_width] + Q_mat_ptr[21] * y[6 + 5 * image_width] + Q_mat_ptr[22] * y[6 + 6 * image_width] + Q_mat_ptr[23] * y[6 + 7 * image_width]) * Q_mat_ptr[6] + (Q_mat_ptr[16] * y[7] + Q_mat_ptr[17] * y[7 + image_width] + Q_mat_ptr[18] * y[7 + 2 * image_width] + Q_mat_ptr[19] * y[7 + 3 * image_width] + Q_mat_ptr[20] * y[7 + 4 * image_width] + Q_mat_ptr[21] * y[7 + 5 * image_width] + Q_mat_ptr[22] * y[7 + 6 * image_width] + Q_mat_ptr[23] * y[7 + 7 * image_width]) * Q_mat_ptr[7];
						S_mat_ptr[9] = (Q_mat_ptr[16] * y[0] + Q_mat_ptr[17] * y[image_width] + Q_mat_ptr[18] * y[2 * image_width] + Q_mat_ptr[19] * y[3 * image_width] + Q_mat_ptr[20] * y[4 * image_width] + Q_mat_ptr[21] * y[5 * image_width] + Q_mat_ptr[22] * y[6 * image_width] + Q_mat_ptr[23] * y[7 * image_width]) * Q_mat_ptr[8] + (Q_mat_ptr[16] * y[1] + Q_mat_ptr[17] * y[1 + image_width] + Q_mat_ptr[18] * y[1 + 2 * image_width] + Q_mat_ptr[19] * y[1 + 3 * image_width] + Q_mat_ptr[20] * y[1 + 4 * image_width] + Q_mat_ptr[21] * y[1 + 5 * image_width] + Q_mat_ptr[22] * y[1 + 6 * image_width] + Q_mat_ptr[23] * y[1 + 7 * image_width]) * Q_mat_ptr[9] + (Q_mat_ptr[16] * y[2] + Q_mat_ptr[17] * y[2 + image_width] + Q_mat_ptr[18] * y[2 + 2 * image_width] + Q_mat_ptr[19] * y[2 + 3 * image_width] + Q_mat_ptr[20] * y[2 + 4 * image_width] + Q_mat_ptr[21] * y[2 + 5 * image_width] + Q_mat_ptr[22] * y[2 + 6 * image_width] + Q_mat_ptr[23] * y[2 + 7 * image_width]) * Q_mat_ptr[10] + (Q_mat_ptr[16] * y[3] + Q_mat_ptr[17] * y[3 + image_width] + Q_mat_ptr[18] * y[3 + 2 * image_width] + Q_mat_ptr[19] * y[3 + 3 * image_width] + Q_mat_ptr[20] * y[3 + 4 * image_width] + Q_mat_ptr[21] * y[3 + 5 * image_width] + Q_mat_ptr[22] * y[3 + 6 * image_width] + Q_mat_ptr[23] * y[3 + 7 * image_width]) * Q_mat_ptr[11] + (Q_mat_ptr[16] * y[4] + Q_mat_ptr[17] * y[4 + image_width] + Q_mat_ptr[18] * y[4 + 2 * image_width] + Q_mat_ptr[19] * y[4 + 3 * image_width] + Q_mat_ptr[20] * y[4 + 4 * image_width] + Q_mat_ptr[21] * y[4 + 5 * image_width] + Q_mat_ptr[22] * y[4 + 6 * image_width] + Q_mat_ptr[23] * y[4 + 7 * image_width]) * Q_mat_ptr[12] + (Q_mat_ptr[16] * y[5] + Q_mat_ptr[17] * y[5 + image_width] + Q_mat_ptr[18] * y[5 + 2 * image_width] + Q_mat_ptr[19] * y[5 + 3 * image_width] + Q_mat_ptr[20] * y[5 + 4 * image_width] + Q_mat_ptr[21] * y[5 + 5 * image_width] + Q_mat_ptr[22] * y[5 + 6 * image_width] + Q_mat_ptr[23] * y[5 + 7 * image_width]) * Q_mat_ptr[13] + (Q_mat_ptr[16] * y[6] + Q_mat_ptr[17] * y[6 + image_width] + Q_mat_ptr[18] * y[6 + 2 * image_width] + Q_mat_ptr[19] * y[6 + 3 * image_width] + Q_mat_ptr[20] * y[6 + 4 * image_width] + Q_mat_ptr[21] * y[6 + 5 * image_width] + Q_mat_ptr[22] * y[6 + 6 * image_width] + Q_mat_ptr[23] * y[6 + 7 * image_width]) * Q_mat_ptr[14] + (Q_mat_ptr[16] * y[7] + Q_mat_ptr[17] * y[7 + image_width] + Q_mat_ptr[18] * y[7 + 2 * image_width] + Q_mat_ptr[19] * y[7 + 3 * image_width] + Q_mat_ptr[20] * y[7 + 4 * image_width] + Q_mat_ptr[21] * y[7 + 5 * image_width] + Q_mat_ptr[22] * y[7 + 6 * image_width] + Q_mat_ptr[23] * y[7 + 7 * image_width]) * Q_mat_ptr[15];
						S_mat_ptr[10] = (Q_mat_ptr[16] * y[0] + Q_mat_ptr[17] * y[image_width] + Q_mat_ptr[18] * y[2 * image_width] + Q_mat_ptr[19] * y[3 * image_width] + Q_mat_ptr[20] * y[4 * image_width] + Q_mat_ptr[21] * y[5 * image_width] + Q_mat_ptr[22] * y[6 * image_width] + Q_mat_ptr[23] * y[7 * image_width]) * Q_mat_ptr[16] + (Q_mat_ptr[16] * y[1] + Q_mat_ptr[17] * y[1 + image_width] + Q_mat_ptr[18] * y[1 + 2 * image_width] + Q_mat_ptr[19] * y[1 + 3 * image_width] + Q_mat_ptr[20] * y[1 + 4 * image_width] + Q_mat_ptr[21] * y[1 + 5 * image_width] + Q_mat_ptr[22] * y[1 + 6 * image_width] + Q_mat_ptr[23] * y[1 + 7 * image_width]) * Q_mat_ptr[17] + (Q_mat_ptr[16] * y[2] + Q_mat_ptr[17] * y[2 + image_width] + Q_mat_ptr[18] * y[2 + 2 * image_width] + Q_mat_ptr[19] * y[2 + 3 * image_width] + Q_mat_ptr[20] * y[2 + 4 * image_width] + Q_mat_ptr[21] * y[2 + 5 * image_width] + Q_mat_ptr[22] * y[2 + 6 * image_width] + Q_mat_ptr[23] * y[2 + 7 * image_width]) * Q_mat_ptr[18] + (Q_mat_ptr[16] * y[3] + Q_mat_ptr[17] * y[3 + image_width] + Q_mat_ptr[18] * y[3 + 2 * image_width] + Q_mat_ptr[19] * y[3 + 3 * image_width] + Q_mat_ptr[20] * y[3 + 4 * image_width] + Q_mat_ptr[21] * y[3 + 5 * image_width] + Q_mat_ptr[22] * y[3 + 6 * image_width] + Q_mat_ptr[23] * y[3 + 7 * image_width]) * Q_mat_ptr[19] + (Q_mat_ptr[16] * y[4] + Q_mat_ptr[17] * y[4 + image_width] + Q_mat_ptr[18] * y[4 + 2 * image_width] + Q_mat_ptr[19] * y[4 + 3 * image_width] + Q_mat_ptr[20] * y[4 + 4 * image_width] + Q_mat_ptr[21] * y[4 + 5 * image_width] + Q_mat_ptr[22] * y[4 + 6 * image_width] + Q_mat_ptr[23] * y[4 + 7 * image_width]) * Q_mat_ptr[20] + (Q_mat_ptr[16] * y[5] + Q_mat_ptr[17] * y[5 + image_width] + Q_mat_ptr[18] * y[5 + 2 * image_width] + Q_mat_ptr[19] * y[5 + 3 * image_width] + Q_mat_ptr[20] * y[5 + 4 * image_width] + Q_mat_ptr[21] * y[5 + 5 * image_width] + Q_mat_ptr[22] * y[5 + 6 * image_width] + Q_mat_ptr[23] * y[5 + 7 * image_width]) * Q_mat_ptr[21] + (Q_mat_ptr[16] * y[6] + Q_mat_ptr[17] * y[6 + image_width] + Q_mat_ptr[18] * y[6 + 2 * image_width] + Q_mat_ptr[19] * y[6 + 3 * image_width] + Q_mat_ptr[20] * y[6 + 4 * image_width] + Q_mat_ptr[21] * y[6 + 5 * image_width] + Q_mat_ptr[22] * y[6 + 6 * image_width] + Q_mat_ptr[23] * y[6 + 7 * image_width]) * Q_mat_ptr[22] + (Q_mat_ptr[16] * y[7] + Q_mat_ptr[17] * y[7 + image_width] + Q_mat_ptr[18] * y[7 + 2 * image_width] + Q_mat_ptr[19] * y[7 + 3 * image_width] + Q_mat_ptr[20] * y[7 + 4 * image_width] + Q_mat_ptr[21] * y[7 + 5 * image_width] + Q_mat_ptr[22] * y[7 + 6 * image_width] + Q_mat_ptr[23] * y[7 + 7 * image_width]) * Q_mat_ptr[23];
						S_mat_ptr[11] = Q_mat_ptr[16] * y[3] + Q_mat_ptr[17] * y[3 + image_width] + Q_mat_ptr[18] * y[3 + 2 * image_width] + Q_mat_ptr[19] * y[3 + 3 * image_width] + Q_mat_ptr[20] * y[3 + 4 * image_width] + Q_mat_ptr[21] * y[3 + 5 * image_width] + Q_mat_ptr[22] * y[3 + 6 * image_width] + Q_mat_ptr[23] * y[3 + 7 * image_width];
						S_mat_ptr[12] = y[3 * image_width] * Q_mat_ptr[0] + y[1 + 3 * image_width] * Q_mat_ptr[1] + y[2 + 3 * image_width] * Q_mat_ptr[2] + Q_mat_ptr[3] * y[3 + 3 * image_width] + y[4 + 3 * image_width] * Q_mat_ptr[4] + y[5 + 3 * image_width] * Q_mat_ptr[5] + y[6 + 3 * image_width] * Q_mat_ptr[6] + y[7 + 3 * image_width] * Q_mat_ptr[7];
						S_mat_ptr[13] = y[3 * image_width] * Q_mat_ptr[8] + y[1 + 3 * image_width] * Q_mat_ptr[9] + y[2 + 3 * image_width] * Q_mat_ptr[10] + Q_mat_ptr[11] * y[3 + 3 * image_width] + y[4 + 3 * image_width] * Q_mat_ptr[12] + y[5 + 3 * image_width] * Q_mat_ptr[13] + y[6 + 3 * image_width] * Q_mat_ptr[14] + y[7 + 3 * image_width] * Q_mat_ptr[15];
						S_mat_ptr[14] = y[3 * image_width] * Q_mat_ptr[16] + y[1 + 3 * image_width] * Q_mat_ptr[17] + y[2 + 3 * image_width] * Q_mat_ptr[18] + Q_mat_ptr[19] * y[3 + 3 * image_width] + y[4 + 3 * image_width] * Q_mat_ptr[20] + y[5 + 3 * image_width] * Q_mat_ptr[21] + y[6 + 3 * image_width] * Q_mat_ptr[22] + y[7 + 3 * image_width] * Q_mat_ptr[23];
						S_mat_ptr[15] = y[3 + 3 * image_width];
					}
					CV_MAT_ELEM(*lut_valid_sign_mat,uchar,index_y - aoi_y_min,index_x - aoi_x_min) =1;						
				}		
				CV_MAT_ELEM(*g_mat,double,i,j) = 
					delta_x*delta_x*delta_x * (delta_y*delta_y*delta_y * S_mat_ptr[0] + delta_y*delta_y * S_mat_ptr[4] + delta_y * S_mat_ptr[8] + S_mat_ptr[12] ) +
					delta_x*delta_x * (delta_y*delta_y*delta_y * S_mat_ptr[1] + delta_y*delta_y * S_mat_ptr[5] + delta_y * S_mat_ptr[9] + S_mat_ptr[13] ) +
					delta_x  * (delta_y*delta_y*delta_y * S_mat_ptr[2] + delta_y*delta_y * S_mat_ptr[6] + delta_y * S_mat_ptr[10] + S_mat_ptr[14] ) +
					(delta_y*delta_y*delta_y * S_mat_ptr[3] + delta_y*delta_y * S_mat_ptr[7] + delta_y * S_mat_ptr[11] + S_mat_ptr[15] );

			}
		}
	}

	return true;
}
bool UniformCubicNatureSplineInterpolationMethod::Two_Dimension_Interpolation_Many_Points_Value_No_Check(
	CvMat* image,
	int reference_point_x,
	int reference_point_y,
	CvMat* displacement_input,//映射后应全部处于图像宽度和高度内，64F,6行1列
	int aoi_x_min,
	int aoi_x_max,//上闭区间，下开区间
	int aoi_y_min,
	int aoi_y_max,//左闭区间，右边开区间
	CvMat* lut_mat,
	CvMat* g_mat
	)
{
	//判断
	//大小
	int image_width= image->width;
	int image_height = image->height;
	int model_width = g_mat->cols;
	int model_height = g_mat->cols;
	int lut_width = lut_mat->cols;
	int lut_height = lut_mat->rows;

	if (displacement_input->rows!=6 && displacement_input->cols!=1)
	{
		return false;
	} 

	if (aoi_x_min<0 || aoi_x_max>=image_width||
		aoi_y_min<0 || aoi_y_max>=image_height)
	{
		return false;
	}
	int aoi_width = aoi_x_max-aoi_x_min;
	int aoi_height = aoi_y_max-aoi_y_min;

	if (lut_mat->cols != aoi_width*lut_element_size_||
		lut_mat->rows !=aoi_height)
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
	if ( CV_MAT_TYPE(displacement_input->type) != CV_64F)
	{
		return false;
	}
	if (
		CV_MAT_TYPE(g_mat->type) != CV_64F
		)
	{
		return false;
	}


	double u = CV_MAT_ELEM(*displacement_input,double,0,0);
	double ux = CV_MAT_ELEM(*displacement_input,double,1,0);
	double uy = CV_MAT_ELEM(*displacement_input,double,2,0) ;
	double v = CV_MAT_ELEM(*displacement_input,double,3,0) ;
	double vx = CV_MAT_ELEM(*displacement_input,double,4,0) ;
	double vy = CV_MAT_ELEM(*displacement_input,double,5,0);



	int model_half_width = (model_width-1)/2;
	int model_half_height = (model_height-1)/2;


	if (lut_type==1)
	{
		for (int i =0;i<model_height;i++)
		{
			for (int j =0;j<model_width;j++)
			{
				//第一种
				double pos_x =reference_point_x + j - model_half_width + u + ux * (j - model_half_width) + uy * (i - model_half_height);
				double pos_y =reference_point_y + i - model_half_height + v + vx * (j - model_half_width) + vy * (i - model_half_height);
				double delta_x = pos_x - cvFloor(pos_x);
				double delta_y = pos_y - cvFloor(pos_y);
				float* S_mat_ptr = lut_mat->data.fl+ ((cvFloor(pos_y) - aoi_y_min) * aoi_width+cvFloor(pos_x) - aoi_x_min) * lut_element_size_;
				CV_MAT_ELEM(*g_mat,double,i,j) = 
					delta_x*delta_x*delta_x * (delta_y*delta_y*delta_y * S_mat_ptr[0] + delta_y*delta_y * S_mat_ptr[4] + delta_y * S_mat_ptr[8] + S_mat_ptr[12] ) +
					delta_x*delta_x * (delta_y*delta_y*delta_y * S_mat_ptr[1] + delta_y*delta_y * S_mat_ptr[5] + delta_y * S_mat_ptr[9] + S_mat_ptr[13] ) +
					delta_x  * (delta_y*delta_y*delta_y * S_mat_ptr[2] + delta_y*delta_y * S_mat_ptr[6] + delta_y * S_mat_ptr[10] + S_mat_ptr[14] ) +
					(delta_y*delta_y*delta_y * S_mat_ptr[3] + delta_y*delta_y * S_mat_ptr[7] + delta_y * S_mat_ptr[11] + S_mat_ptr[15] );



				//第二种--没第一种快
				//double delta_x = reference_point_x + j - model_half_width + u + ux * (j - model_half_width) + uy * (i - model_half_height) - cvFloor(reference_point_x + j - model_half_width + u + ux * (j - model_half_width) + uy * (i - model_half_height));
				//double delta_y = reference_point_y + i - model_half_height + v + vx * (j - model_half_width) + vy * (i - model_half_height) - cvFloor(reference_point_y + i - model_half_height + v + vx * (j - model_half_width) + vy * (i - model_half_height));
				//int lut_index =((reference_point_y + i - model_half_height + v + vx * (j - model_half_width) + vy * (i - model_half_height) -aoi_y_min)*aoi_width +reference_point_x + j - model_half_width + u + ux * (j - model_half_width) + uy * (i - model_half_height) -aoi_x_min )*lut_element_size_;
				//double* S_mat_ptr = lut_mat->data.db+ lut_index;
				//CV_MAT_ELEM(*g_mat,double,i,j) = 
				//	delta_x*delta_x*delta_x * (delta_y*delta_y*delta_y * S_mat_ptr[0] + delta_y*delta_y * S_mat_ptr[4] + delta_y * S_mat_ptr[8] + S_mat_ptr[12] ) +
				//	delta_x*delta_x * (delta_y*delta_y*delta_y * S_mat_ptr[1] + delta_y*delta_y * S_mat_ptr[5] + delta_y * S_mat_ptr[9] + S_mat_ptr[13] ) +
				//	delta_x  * (delta_y*delta_y*delta_y * S_mat_ptr[2] + delta_y*delta_y * S_mat_ptr[6] + delta_y * S_mat_ptr[10] + S_mat_ptr[14] ) +
				//	(delta_y*delta_y*delta_y * S_mat_ptr[3] + delta_y*delta_y * S_mat_ptr[7] + delta_y * S_mat_ptr[11] + S_mat_ptr[15] );


			}
		}
	}
	else
	{
		for (int i =0;i<model_height;i++)
		{
			for (int j =0;j<model_width;j++)
			{
				//第一种
				double pos_x =reference_point_x + j - model_half_width + u + ux * (j - model_half_width) + uy * (i - model_half_height);
				double pos_y =reference_point_y + i - model_half_height + v + vx * (j - model_half_width) + vy * (i - model_half_height);
				double delta_x = pos_x - cvFloor(pos_x);
				double delta_y = pos_y - cvFloor(pos_y);
				double* S_mat_ptr = lut_mat->data.db+ ((cvFloor(pos_y) - aoi_y_min) * aoi_width+cvFloor(pos_x) - aoi_x_min) * lut_element_size_;
				CV_MAT_ELEM(*g_mat,double,i,j) = 
					delta_x*delta_x*delta_x * (delta_y*delta_y*delta_y * S_mat_ptr[0] + delta_y*delta_y * S_mat_ptr[4] + delta_y * S_mat_ptr[8] + S_mat_ptr[12] ) +
					delta_x*delta_x * (delta_y*delta_y*delta_y * S_mat_ptr[1] + delta_y*delta_y * S_mat_ptr[5] + delta_y * S_mat_ptr[9] + S_mat_ptr[13] ) +
					delta_x  * (delta_y*delta_y*delta_y * S_mat_ptr[2] + delta_y*delta_y * S_mat_ptr[6] + delta_y * S_mat_ptr[10] + S_mat_ptr[14] ) +
					(delta_y*delta_y*delta_y * S_mat_ptr[3] + delta_y*delta_y * S_mat_ptr[7] + delta_y * S_mat_ptr[11] + S_mat_ptr[15] );



				//第二种--没第一种快
				//double delta_x = reference_point_x + j - model_half_width + u + ux * (j - model_half_width) + uy * (i - model_half_height) - cvFloor(reference_point_x + j - model_half_width + u + ux * (j - model_half_width) + uy * (i - model_half_height));
				//double delta_y = reference_point_y + i - model_half_height + v + vx * (j - model_half_width) + vy * (i - model_half_height) - cvFloor(reference_point_y + i - model_half_height + v + vx * (j - model_half_width) + vy * (i - model_half_height));
				//int lut_index =((reference_point_y + i - model_half_height + v + vx * (j - model_half_width) + vy * (i - model_half_height) -aoi_y_min)*aoi_width +reference_point_x + j - model_half_width + u + ux * (j - model_half_width) + uy * (i - model_half_height) -aoi_x_min )*lut_element_size_;
				//double* S_mat_ptr = lut_mat->data.db+ lut_index;
				//CV_MAT_ELEM(*g_mat,double,i,j) = 
				//	delta_x*delta_x*delta_x * (delta_y*delta_y*delta_y * S_mat_ptr[0] + delta_y*delta_y * S_mat_ptr[4] + delta_y * S_mat_ptr[8] + S_mat_ptr[12] ) +
				//	delta_x*delta_x * (delta_y*delta_y*delta_y * S_mat_ptr[1] + delta_y*delta_y * S_mat_ptr[5] + delta_y * S_mat_ptr[9] + S_mat_ptr[13] ) +
				//	delta_x  * (delta_y*delta_y*delta_y * S_mat_ptr[2] + delta_y*delta_y * S_mat_ptr[6] + delta_y * S_mat_ptr[10] + S_mat_ptr[14] ) +
				//	(delta_y*delta_y*delta_y * S_mat_ptr[3] + delta_y*delta_y * S_mat_ptr[7] + delta_y * S_mat_ptr[11] + S_mat_ptr[15] );


			}
		}
	}


	return true;
}


bool UniformCubicNatureSplineInterpolationMethod::Build_8X8_D_Mat(
	CvMat* reference_image,
	CvMat* Q_mat,
	int aoi_left_top_point_x_in_image,
	int aoi_left_top_point_y_in_image,
	CvMat* aoi_sign_mat,
	int horizontal_step,
	int vertical_step,
	int template_half_width,
	int template_half_height,
	CvMat* D_mat
	)
{
	//省略判断
	//判断
	//大小
	int image_width= reference_image->width;
	int image_height = reference_image->height;

	int template_width = 2*template_half_width+1;
	int template_height = 2*template_half_height+1;
	int template_point_count = template_height*template_width;


	int mat_col_count = aoi_sign_mat->cols;
	int mat_row_count = aoi_sign_mat->rows;
	if (D_mat->cols!=mat_col_count*6*template_point_count||
		D_mat->rows!=mat_row_count)
	{
		return false;
	}


	if (Q_mat->rows!=4||Q_mat->cols!=8)
	{
		return false;
	}


	//类型
	int image_type = 0;//1--CV_8U,2--CV_64F
	if (CV_MAT_TYPE(reference_image->type) ==CV_8U )
	{
		image_type = 1;
	}
	//else if (CV_MAT_TYPE(reference_image->type) ==CV_64F)
	//{
	//	image_type = 2;
	//} 
	else
	{
		return false;
	}
	if (CV_MAT_TYPE(Q_mat->type) !=CV_64F ||
		CV_MAT_TYPE(D_mat->type) !=CV_64F ||
		CV_MAT_TYPE(aoi_sign_mat->type) !=CV_8U )
	{
		return false;
	}




	double* Q_mat_ptr = Q_mat->data.db;
	double* D_mat_ptr = D_mat->data.db;

	CvMat* _6xS_mat = cvCreateMat(6,template_point_count,CV_64F);
	CvMat* Sx6_mat = cvCreateMat(template_point_count,6,CV_64F);
	CvMat* hessian_mat = cvCreateMat(6,6,CV_64F);
	CvMat* inverse_hessian_mat = cvCreateMat(6,6,CV_64F);
	double* inverse_hessian_mat_ptr = inverse_hessian_mat->data.db;


	for (int i =0;i<mat_row_count;i++)
	{
		for (int j =0;j<mat_col_count;j++)
		{
			uchar sign = CV_MAT_ELEM(*aoi_sign_mat,uchar,i,j);
			if (sign==1)
			{
				

				for (int m=-template_half_height;m<=template_half_height;m++)
				{
					for (int n =-template_half_width;n<=template_half_width;n++)
					{
						//int center_y =  i*vertical_step +aoi_left_top_point_y_in_image ;
						//int center_x = j*horizontal_step+aoi_left_top_point_x_in_image;
						//int index_i_min = =center_y+m -3;
						//int index_j_min = center_x+n -3;
						int index_i_min = i*vertical_step +aoi_left_top_point_y_in_image+m -3;
						int index_j_min = j*horizontal_step+aoi_left_top_point_x_in_image +n -3;

						uchar* y=reference_image->data.ptr +index_i_min*image_width+index_j_min;
						double fx = (double)(
							y[3 * image_width] * Q_mat_ptr[16] + y[1 + 3 * image_width] * Q_mat_ptr[17] + y[2 + 3 * image_width] * Q_mat_ptr[18] + Q_mat_ptr[19] * y[3 + 3 * image_width] + y[4 + 3 * image_width] * Q_mat_ptr[20] + y[5 + 3 * image_width] * Q_mat_ptr[21] + y[6 + 3 * image_width] * Q_mat_ptr[22] + y[7 + 3 * image_width] * Q_mat_ptr[23]
						);
						double fy = (double)(
							Q_mat_ptr[16] * y[3] + Q_mat_ptr[17] * y[3 + image_width] + Q_mat_ptr[18] * y[3 + 2 * image_width] + Q_mat_ptr[19] * y[3 + 3 * image_width] + Q_mat_ptr[20] * y[3 + 4 * image_width] + Q_mat_ptr[21] * y[3 + 5 * image_width] + Q_mat_ptr[22] * y[3 + 6 * image_width] + Q_mat_ptr[23] * y[3 + 7 * image_width]
						);
						CV_MAT_ELEM(*_6xS_mat,double,0,(m+template_half_height)*template_width+n+template_half_width) = fx;
						CV_MAT_ELEM(*_6xS_mat,double,1,(m+template_half_height)*template_width+n+template_half_width) = n*fx;
						CV_MAT_ELEM(*_6xS_mat,double,2,(m+template_half_height)*template_width+n+template_half_width) = m*fx;
						CV_MAT_ELEM(*_6xS_mat,double,3,(m+template_half_height)*template_width+n+template_half_width) = fy;
						CV_MAT_ELEM(*_6xS_mat,double,4,(m+template_half_height)*template_width+n+template_half_width) = n*fy;
						CV_MAT_ELEM(*_6xS_mat,double,5,(m+template_half_height)*template_width+n+template_half_width) = m*fy;


						CV_MAT_ELEM(*Sx6_mat,double,(m+template_half_height)*template_width+n+template_half_width , 0) = fx;
						CV_MAT_ELEM(*Sx6_mat,double,(m+template_half_height)*template_width+n+template_half_width , 1) = n*fx;
						CV_MAT_ELEM(*Sx6_mat,double,(m+template_half_height)*template_width+n+template_half_width , 2) = m*fx;
						CV_MAT_ELEM(*Sx6_mat,double,(m+template_half_height)*template_width+n+template_half_width , 3) = fy;
						CV_MAT_ELEM(*Sx6_mat,double,(m+template_half_height)*template_width+n+template_half_width , 4) = n*fy;
						CV_MAT_ELEM(*Sx6_mat,double,(m+template_half_height)*template_width+n+template_half_width , 5) = m*fy;

					}
				}
				cvMatMul(_6xS_mat,Sx6_mat,hessian_mat);
				cvInvert(hessian_mat,inverse_hessian_mat);

				for (int l = 0;l<6;l++)
				{
					for (int k =0 ;k<template_point_count;k++)
					{
						D_mat_ptr[l*template_point_count+k] = 
							CV_MAT_ELEM(*inverse_hessian_mat,double,l,0) * CV_MAT_ELEM(*_6xS_mat,double,0,k)+
							CV_MAT_ELEM(*inverse_hessian_mat,double,l,1) * CV_MAT_ELEM(*_6xS_mat,double,1,k)+
							CV_MAT_ELEM(*inverse_hessian_mat,double,l,2) * CV_MAT_ELEM(*_6xS_mat,double,2,k)+
							CV_MAT_ELEM(*inverse_hessian_mat,double,l,3) * CV_MAT_ELEM(*_6xS_mat,double,3,k)+
							CV_MAT_ELEM(*inverse_hessian_mat,double,l,4) * CV_MAT_ELEM(*_6xS_mat,double,4,k)+
							CV_MAT_ELEM(*inverse_hessian_mat,double,l,5) * CV_MAT_ELEM(*_6xS_mat,double,5,k);
					}
				}

			}
		}
	}
	cvReleaseMat(&_6xS_mat);
	cvReleaseMat(&Sx6_mat);
	cvReleaseMat(&hessian_mat);
	cvReleaseMat(&inverse_hessian_mat);
	return true;




}

bool UniformCubicNatureSplineInterpolationMethod::Build_6X6_D_Mat(
	CvMat* reference_image,
	CvMat* Q_mat,
	int aoi_left_top_point_x_in_image,
	int aoi_left_top_point_y_in_image,
	CvMat* aoi_sign_mat,
	int horizontal_step,
	int vertical_step,
	int template_half_width,
	int template_half_height,
	CvMat* D_mat
	)
{
	//省略判断
	//判断
	//大小
	int image_width= reference_image->width;
	int image_height = reference_image->height;

	int template_width = 2*template_half_width+1;
	int template_height = 2*template_half_height+1;
	int template_point_count = template_height*template_width;


	int mat_col_count = aoi_sign_mat->cols;
	int mat_row_count = aoi_sign_mat->rows;
	if (D_mat->cols!=mat_col_count*6*template_point_count||
		D_mat->rows!=mat_row_count)
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
	//else if (CV_MAT_TYPE(reference_image->type) ==CV_64F)
	//{
	//	image_type = 2;
	//} 
	else
	{
		return false;
	}
	if (CV_MAT_TYPE(Q_mat->type) !=CV_64F ||
		CV_MAT_TYPE(D_mat->type) !=CV_64F ||
		CV_MAT_TYPE(aoi_sign_mat->type) !=CV_8U )
	{
		return false;
	}




	double* Q_mat_ptr = Q_mat->data.db;
	double* D_mat_ptr = D_mat->data.db;

	CvMat* _6xS_mat = cvCreateMat(6,template_point_count,CV_64F);
	CvMat* Sx6_mat = cvCreateMat(template_point_count,6,CV_64F);
	CvMat* hessian_mat = cvCreateMat(6,6,CV_64F);
	CvMat* inverse_hessian_mat = cvCreateMat(6,6,CV_64F);
	double* inverse_hessian_mat_ptr = inverse_hessian_mat->data.db;


	for (int i =0;i<mat_row_count;i++)
	{
		for (int j =0;j<mat_col_count;j++)
		{
			uchar sign = CV_MAT_ELEM(*aoi_sign_mat,uchar,i,j);
			if (sign==1)
			{
				for (int m=-template_half_height;m<=template_half_height;m++)
				{
					for (int n =-template_half_width;n<=template_half_width;n++)
					{
						//int center_y =  i*vertical_step +aoi_left_top_point_y_in_image ;
						//int center_x = j*horizontal_step+aoi_left_top_point_x_in_image;
						//int index_i_min = =center_y+m -3;
						//int index_j_min = center_x+n -3;
						int index_i_min = i*vertical_step +aoi_left_top_point_y_in_image+m -2;
						int index_j_min = j*horizontal_step+aoi_left_top_point_x_in_image +n -2;

						uchar* y=reference_image->data.ptr +index_i_min*image_width+index_j_min;

						double fx = (double)(
							y[2 * image_width] * Q_mat_ptr[12] + y[1 + 2 * image_width] * Q_mat_ptr[13] + Q_mat_ptr[14] * y[2 + 2 * image_width] + y[3 + 2 * image_width] * Q_mat_ptr[15] + y[4 + 2 * image_width] * Q_mat_ptr[16] + y[5 + 2 * image_width] * Q_mat_ptr[17]
						);
						double fy = (double)(
							Q_mat_ptr[12] * y[2] + Q_mat_ptr[13] * y[2 + image_width] + Q_mat_ptr[14] * y[2 + 2 * image_width] + Q_mat_ptr[15] * y[2 + 3 * image_width] + Q_mat_ptr[16] * y[2 + 4 * image_width] + Q_mat_ptr[17] * y[2 + 5 * image_width]
						);
						CV_MAT_ELEM(*_6xS_mat,double,0,(m+template_half_height)*template_width+n+template_half_width) = fx;
						CV_MAT_ELEM(*_6xS_mat,double,1,(m+template_half_height)*template_width+n+template_half_width) = n*fx;
						CV_MAT_ELEM(*_6xS_mat,double,2,(m+template_half_height)*template_width+n+template_half_width) = m*fx;
						CV_MAT_ELEM(*_6xS_mat,double,3,(m+template_half_height)*template_width+n+template_half_width) = fy;
						CV_MAT_ELEM(*_6xS_mat,double,4,(m+template_half_height)*template_width+n+template_half_width) = n*fy;
						CV_MAT_ELEM(*_6xS_mat,double,5,(m+template_half_height)*template_width+n+template_half_width) = m*fy;


						CV_MAT_ELEM(*Sx6_mat,double,(m+template_half_height)*template_width+n+template_half_width , 0) = fx;
						CV_MAT_ELEM(*Sx6_mat,double,(m+template_half_height)*template_width+n+template_half_width , 1) = n*fx;
						CV_MAT_ELEM(*Sx6_mat,double,(m+template_half_height)*template_width+n+template_half_width , 2) = m*fx;
						CV_MAT_ELEM(*Sx6_mat,double,(m+template_half_height)*template_width+n+template_half_width , 3) = fy;
						CV_MAT_ELEM(*Sx6_mat,double,(m+template_half_height)*template_width+n+template_half_width , 4) = n*fy;
						CV_MAT_ELEM(*Sx6_mat,double,(m+template_half_height)*template_width+n+template_half_width , 5) = m*fy;

					}
				}
				cvMatMul(_6xS_mat,Sx6_mat,hessian_mat);
				double det = cvInvert(hessian_mat,inverse_hessian_mat);


				for (int l = 0;l<6;l++)
				{
					for (int k =0 ;k<template_point_count;k++)
					{
						D_mat_ptr[(i*mat_col_count+j)*template_point_count * 6 +l*template_point_count+k] = -
							CV_MAT_ELEM(*inverse_hessian_mat,double,l,0) * CV_MAT_ELEM(*_6xS_mat,double,0,k)-
							CV_MAT_ELEM(*inverse_hessian_mat,double,l,1) * CV_MAT_ELEM(*_6xS_mat,double,1,k)-
							CV_MAT_ELEM(*inverse_hessian_mat,double,l,2) * CV_MAT_ELEM(*_6xS_mat,double,2,k)-
							CV_MAT_ELEM(*inverse_hessian_mat,double,l,3) * CV_MAT_ELEM(*_6xS_mat,double,3,k)-
							CV_MAT_ELEM(*inverse_hessian_mat,double,l,4) * CV_MAT_ELEM(*_6xS_mat,double,4,k)-
							CV_MAT_ELEM(*inverse_hessian_mat,double,l,5) * CV_MAT_ELEM(*_6xS_mat,double,5,k);					
					}
				}

			}
		}
	}
	cvReleaseMat(&_6xS_mat);
	cvReleaseMat(&Sx6_mat);
	cvReleaseMat(&hessian_mat);
	cvReleaseMat(&inverse_hessian_mat);
	return true;
}

bool UniformCubicNatureSplineInterpolationMethod::Build_Two_Dimension_Interpolation_8X8_Right_Reference_Points_Gradient_Value_LUT( 
	CvMat* right_reference_image,
	CvMat* Q_mat,
	int left_reference_aoi_left_top_point_x_in_image,
	int left_reference_aoi_left_top_point_y_in_image,
	CvMat* reference_displacement_valid_sign_mat,
	CvMat* reference_displacement_u_mat,
	CvMat* reference_displacement_ux_mat,
	CvMat* reference_displacement_uy_mat,
	CvMat* reference_displacement_v_mat,
	CvMat* reference_displacement_vx_mat,
	CvMat* reference_displacement_vy_mat,
	int template_half_width,
	int template_half_height,
	CvMat* right_reference_image_gradient_x_mat,
	CvMat* right_reference_image_gradient_y_mat
	)
{
	return true;
}

bool UniformCubicNatureSplineInterpolationMethod::Build_Two_Dimension_Interpolation_6X6_Right_Reference_Points_Gradient_Value_LUT( 
	CvMat* right_reference_image,
	CvMat* Q_mat,
	int left_reference_aoi_left_top_point_x_in_image,
	int left_reference_aoi_left_top_point_y_in_image,
	CvMat* reference_displacement_valid_sign_mat,
	CvMat* reference_displacement_u_mat,
	CvMat* reference_displacement_ux_mat,
	CvMat* reference_displacement_uy_mat,
	CvMat* reference_displacement_v_mat,
	CvMat* reference_displacement_vx_mat,
	CvMat* reference_displacement_vy_mat,
	int template_half_width,
	int template_half_height,
	CvMat* right_reference_image_gradient_x_mat,
	CvMat* right_reference_image_gradient_y_mat
	)
{
	return true;
}




//bool UniformCubicNatureSplineInterpolationMethod::Two_Dimension_Interpolation_Many_Points( 
//	CvMat* image,
//	int reference_point_x,
//	int reference_point_y,
//	CvMat* displacement_input,//映射后应全部处于图像宽度和高度内，64F,6行1列
//	QHash<HashKey2d,CvMat*>* lut,
//	CvMat* Q_mat,
//	CvMat* g_mat
//	)
//{
//	//判断
//	//大小
//	int image_width= image->width;
//	int image_height = image->height;
//	int template_width = g_mat->cols;
//	int template_height = g_mat->cols;
//
//	if (displacement_input->rows!=6 && displacement_input->cols!=1)
//	{
//		return false;
//	} 
//	int Q_mat_type = 0;//1--6X6,2--8X8
//	if (Q_mat->cols==6&&Q_mat->rows==4)
//	{
//		Q_mat_type=1;
//	} 
//	else if (Q_mat->cols==8&&Q_mat->rows==4)
//	{
//		Q_mat_type=2;
//	}
//	else
//	{
//		return false;
//	}
//
//
//
//
//
//	//类型
//	int image_type = 0;//1--CV_8U,2--CV_64F
//	if (CV_MAT_TYPE(image->type) ==CV_8U )
//	{
//		image_type = 1;
//	}
//	else if (CV_MAT_TYPE(image->type) ==CV_64F)
//	{
//		image_type = 2;
//	} 
//	else
//	{
//		return false;
//	}
//
//	if ( CV_MAT_TYPE(displacement_input->type) != CV_64F)
//	{
//		return false;
//	}
//	if (CV_MAT_TYPE(g_mat->type) != CV_64F||CV_MAT_TYPE(Q_mat->type) != CV_64F)
//	{
//		return false;
//	}
//	//LUT为64F
//
//
//	//
//
//
//
//	double u = CV_MAT_ELEM(*displacement_input,double,0,0);
//	double ux = CV_MAT_ELEM(*displacement_input,double,1,0);
//	double uy = CV_MAT_ELEM(*displacement_input,double,2,0);
//	double v = CV_MAT_ELEM(*displacement_input,double,3,0) ;
//	double vx = CV_MAT_ELEM(*displacement_input,double,4,0);
//	double vy = CV_MAT_ELEM(*displacement_input,double,5,0);
//
//	int template_half_width = (template_width-1)/2;
//	int template_half_height = (template_height-1)/2;
//
//
//
//	for (int i =0;i<template_height;i++)
//	{
//		for (int j =0;j<template_width;j++)
//		{
//			//第一种
//			double pos_x =reference_point_x + j - template_half_width + u + ux * (j - template_half_width) + uy * (i - template_half_height);
//			double pos_y =reference_point_y + i - template_half_height + v + vx * (j - template_half_width) + vy * (i - template_half_height);
//
//			if (pos_x<0 || pos_x>=image_width-1||
//				pos_y<0 || pos_y>=image_height-1)//右侧为开区间
//			{
//				return false;
//			}
//			HashKey2d key(cvFloor(pos_x),cvFloor(pos_y));
//			double delta_x = pos_x - key.index_col_;
//			double delta_y = pos_y - key.index_row_;
//			double* S_mat_ptr;
//
//			
//			if (!lut->contains(key))
//			{
//				CvMat* element = cvCreateMat(4,4,CV_64F);
//				double* lut_element_ptr = element->data.db;
//				double* Q_mat_ptr = Q_mat->data.db;
//
//				int index_i_min;
//				int index_j_min;				
//				if (Q_mat_type==1)
//				{
//					index_i_min = key.index_row_-2;
//					index_j_min = key.index_col_-2;			
//					uchar* y = image->data.ptr + index_i_min*image_width+index_j_min;
//
//					lut_element_ptr[0] = (Q_mat_ptr[0] * y[0] + Q_mat_ptr[1] * y[image_width] + Q_mat_ptr[2] * y[2 * image_width] + Q_mat_ptr[3] * y[3 * image_width] + Q_mat_ptr[4] * y[4 * image_width] + Q_mat_ptr[5] * y[5 * image_width]) * Q_mat_ptr[0] + (Q_mat_ptr[0] * y[1] + Q_mat_ptr[1] * y[1 + image_width] + Q_mat_ptr[2] * y[1 + 2 * image_width] + Q_mat_ptr[3] * y[1 + 3 * image_width] + Q_mat_ptr[4] * y[1 + 4 * image_width] + Q_mat_ptr[5] * y[1 + 5 * image_width]) * Q_mat_ptr[1] + (Q_mat_ptr[0] * y[2] + Q_mat_ptr[1] * y[2 + image_width] + Q_mat_ptr[2] * y[2 + 2 * image_width] + Q_mat_ptr[3] * y[2 + 3 * image_width] + Q_mat_ptr[4] * y[2 + 4 * image_width] + Q_mat_ptr[5] * y[2 + 5 * image_width]) * Q_mat_ptr[2] + (Q_mat_ptr[0] * y[3] + Q_mat_ptr[1] * y[3 + image_width] + Q_mat_ptr[2] * y[3 + 2 * image_width] + Q_mat_ptr[3] * y[3 + 3 * image_width] + Q_mat_ptr[4] * y[3 + 4 * image_width] + Q_mat_ptr[5] * y[3 + 5 * image_width]) * Q_mat_ptr[3] + (Q_mat_ptr[0] * y[4] + Q_mat_ptr[1] * y[4 + image_width] + Q_mat_ptr[2] * y[4 + 2 * image_width] + Q_mat_ptr[3] * y[4 + 3 * image_width] + Q_mat_ptr[4] * y[4 + 4 * image_width] + Q_mat_ptr[5] * y[4 + 5 * image_width]) * Q_mat_ptr[4] + (Q_mat_ptr[0] * y[5] + Q_mat_ptr[1] * y[5 + image_width] + Q_mat_ptr[2] * y[5 + 2 * image_width] + Q_mat_ptr[3] * y[5 + 3 * image_width] + Q_mat_ptr[4] * y[5 + 4 * image_width] + Q_mat_ptr[5] * y[5 + 5 * image_width]) * Q_mat_ptr[5];
//					lut_element_ptr[1] = (Q_mat_ptr[0] * y[0] + Q_mat_ptr[1] * y[image_width] + Q_mat_ptr[2] * y[2 * image_width] + Q_mat_ptr[3] * y[3 * image_width] + Q_mat_ptr[4] * y[4 * image_width] + Q_mat_ptr[5] * y[5 * image_width]) * Q_mat_ptr[6] + (Q_mat_ptr[0] * y[1] + Q_mat_ptr[1] * y[1 + image_width] + Q_mat_ptr[2] * y[1 + 2 * image_width] + Q_mat_ptr[3] * y[1 + 3 * image_width] + Q_mat_ptr[4] * y[1 + 4 * image_width] + Q_mat_ptr[5] * y[1 + 5 * image_width]) * Q_mat_ptr[7] + (Q_mat_ptr[0] * y[2] + Q_mat_ptr[1] * y[2 + image_width] + Q_mat_ptr[2] * y[2 + 2 * image_width] + Q_mat_ptr[3] * y[2 + 3 * image_width] + Q_mat_ptr[4] * y[2 + 4 * image_width] + Q_mat_ptr[5] * y[2 + 5 * image_width]) * Q_mat_ptr[8] + (Q_mat_ptr[0] * y[3] + Q_mat_ptr[1] * y[3 + image_width] + Q_mat_ptr[2] * y[3 + 2 * image_width] + Q_mat_ptr[3] * y[3 + 3 * image_width] + Q_mat_ptr[4] * y[3 + 4 * image_width] + Q_mat_ptr[5] * y[3 + 5 * image_width]) * Q_mat_ptr[9] + (Q_mat_ptr[0] * y[4] + Q_mat_ptr[1] * y[4 + image_width] + Q_mat_ptr[2] * y[4 + 2 * image_width] + Q_mat_ptr[3] * y[4 + 3 * image_width] + Q_mat_ptr[4] * y[4 + 4 * image_width] + Q_mat_ptr[5] * y[4 + 5 * image_width]) * Q_mat_ptr[10] + (Q_mat_ptr[0] * y[5] + Q_mat_ptr[1] * y[5 + image_width] + Q_mat_ptr[2] * y[5 + 2 * image_width] + Q_mat_ptr[3] * y[5 + 3 * image_width] + Q_mat_ptr[4] * y[5 + 4 * image_width] + Q_mat_ptr[5] * y[5 + 5 * image_width]) * Q_mat_ptr[11];
//					lut_element_ptr[2] = (Q_mat_ptr[0] * y[0] + Q_mat_ptr[1] * y[image_width] + Q_mat_ptr[2] * y[2 * image_width] + Q_mat_ptr[3] * y[3 * image_width] + Q_mat_ptr[4] * y[4 * image_width] + Q_mat_ptr[5] * y[5 * image_width]) * Q_mat_ptr[12] + (Q_mat_ptr[0] * y[1] + Q_mat_ptr[1] * y[1 + image_width] + Q_mat_ptr[2] * y[1 + 2 * image_width] + Q_mat_ptr[3] * y[1 + 3 * image_width] + Q_mat_ptr[4] * y[1 + 4 * image_width] + Q_mat_ptr[5] * y[1 + 5 * image_width]) * Q_mat_ptr[13] + (Q_mat_ptr[0] * y[2] + Q_mat_ptr[1] * y[2 + image_width] + Q_mat_ptr[2] * y[2 + 2 * image_width] + Q_mat_ptr[3] * y[2 + 3 * image_width] + Q_mat_ptr[4] * y[2 + 4 * image_width] + Q_mat_ptr[5] * y[2 + 5 * image_width]) * Q_mat_ptr[14] + (Q_mat_ptr[0] * y[3] + Q_mat_ptr[1] * y[3 + image_width] + Q_mat_ptr[2] * y[3 + 2 * image_width] + Q_mat_ptr[3] * y[3 + 3 * image_width] + Q_mat_ptr[4] * y[3 + 4 * image_width] + Q_mat_ptr[5] * y[3 + 5 * image_width]) * Q_mat_ptr[15] + (Q_mat_ptr[0] * y[4] + Q_mat_ptr[1] * y[4 + image_width] + Q_mat_ptr[2] * y[4 + 2 * image_width] + Q_mat_ptr[3] * y[4 + 3 * image_width] + Q_mat_ptr[4] * y[4 + 4 * image_width] + Q_mat_ptr[5] * y[4 + 5 * image_width]) * Q_mat_ptr[16] + (Q_mat_ptr[0] * y[5] + Q_mat_ptr[1] * y[5 + image_width] + Q_mat_ptr[2] * y[5 + 2 * image_width] + Q_mat_ptr[3] * y[5 + 3 * image_width] + Q_mat_ptr[4] * y[5 + 4 * image_width] + Q_mat_ptr[5] * y[5 + 5 * image_width]) * Q_mat_ptr[17];
//					lut_element_ptr[3] = Q_mat_ptr[0] * y[2] + Q_mat_ptr[1] * y[2 + image_width] + Q_mat_ptr[2] * y[2 + 2 * image_width] + Q_mat_ptr[3] * y[2 + 3 * image_width] + Q_mat_ptr[4] * y[2 + 4 * image_width] + Q_mat_ptr[5] * y[2 + 5 * image_width];
//					lut_element_ptr[4] = (Q_mat_ptr[6] * y[0] + Q_mat_ptr[7] * y[image_width] + Q_mat_ptr[8] * y[2 * image_width] + Q_mat_ptr[9] * y[3 * image_width] + Q_mat_ptr[10] * y[4 * image_width] + Q_mat_ptr[11] * y[5 * image_width]) * Q_mat_ptr[0] + (Q_mat_ptr[6] * y[1] + Q_mat_ptr[7] * y[1 + image_width] + Q_mat_ptr[8] * y[1 + 2 * image_width] + Q_mat_ptr[9] * y[1 + 3 * image_width] + Q_mat_ptr[10] * y[1 + 4 * image_width] + Q_mat_ptr[11] * y[1 + 5 * image_width]) * Q_mat_ptr[1] + (Q_mat_ptr[6] * y[2] + Q_mat_ptr[7] * y[2 + image_width] + Q_mat_ptr[8] * y[2 + 2 * image_width] + Q_mat_ptr[9] * y[2 + 3 * image_width] + Q_mat_ptr[10] * y[2 + 4 * image_width] + Q_mat_ptr[11] * y[2 + 5 * image_width]) * Q_mat_ptr[2] + (Q_mat_ptr[6] * y[3] + Q_mat_ptr[7] * y[3 + image_width] + Q_mat_ptr[8] * y[3 + 2 * image_width] + Q_mat_ptr[9] * y[3 + 3 * image_width] + Q_mat_ptr[10] * y[3 + 4 * image_width] + Q_mat_ptr[11] * y[3 + 5 * image_width]) * Q_mat_ptr[3] + (Q_mat_ptr[6] * y[4] + Q_mat_ptr[7] * y[4 + image_width] + Q_mat_ptr[8] * y[4 + 2 * image_width] + Q_mat_ptr[9] * y[4 + 3 * image_width] + Q_mat_ptr[10] * y[4 + 4 * image_width] + Q_mat_ptr[11] * y[4 + 5 * image_width]) * Q_mat_ptr[4] + (Q_mat_ptr[6] * y[5] + Q_mat_ptr[7] * y[5 + image_width] + Q_mat_ptr[8] * y[5 + 2 * image_width] + Q_mat_ptr[9] * y[5 + 3 * image_width] + Q_mat_ptr[10] * y[5 + 4 * image_width] + Q_mat_ptr[11] * y[5 + 5 * image_width]) * Q_mat_ptr[5];
//					lut_element_ptr[5] = (Q_mat_ptr[6] * y[0] + Q_mat_ptr[7] * y[image_width] + Q_mat_ptr[8] * y[2 * image_width] + Q_mat_ptr[9] * y[3 * image_width] + Q_mat_ptr[10] * y[4 * image_width] + Q_mat_ptr[11] * y[5 * image_width]) * Q_mat_ptr[6] + (Q_mat_ptr[6] * y[1] + Q_mat_ptr[7] * y[1 + image_width] + Q_mat_ptr[8] * y[1 + 2 * image_width] + Q_mat_ptr[9] * y[1 + 3 * image_width] + Q_mat_ptr[10] * y[1 + 4 * image_width] + Q_mat_ptr[11] * y[1 + 5 * image_width]) * Q_mat_ptr[7] + (Q_mat_ptr[6] * y[2] + Q_mat_ptr[7] * y[2 + image_width] + Q_mat_ptr[8] * y[2 + 2 * image_width] + Q_mat_ptr[9] * y[2 + 3 * image_width] + Q_mat_ptr[10] * y[2 + 4 * image_width] + Q_mat_ptr[11] * y[2 + 5 * image_width]) * Q_mat_ptr[8] + (Q_mat_ptr[6] * y[3] + Q_mat_ptr[7] * y[3 + image_width] + Q_mat_ptr[8] * y[3 + 2 * image_width] + Q_mat_ptr[9] * y[3 + 3 * image_width] + Q_mat_ptr[10] * y[3 + 4 * image_width] + Q_mat_ptr[11] * y[3 + 5 * image_width]) * Q_mat_ptr[9] + (Q_mat_ptr[6] * y[4] + Q_mat_ptr[7] * y[4 + image_width] + Q_mat_ptr[8] * y[4 + 2 * image_width] + Q_mat_ptr[9] * y[4 + 3 * image_width] + Q_mat_ptr[10] * y[4 + 4 * image_width] + Q_mat_ptr[11] * y[4 + 5 * image_width]) * Q_mat_ptr[10] + (Q_mat_ptr[6] * y[5] + Q_mat_ptr[7] * y[5 + image_width] + Q_mat_ptr[8] * y[5 + 2 * image_width] + Q_mat_ptr[9] * y[5 + 3 * image_width] + Q_mat_ptr[10] * y[5 + 4 * image_width] + Q_mat_ptr[11] * y[5 + 5 * image_width]) * Q_mat_ptr[11];
//					lut_element_ptr[6] = (Q_mat_ptr[6] * y[0] + Q_mat_ptr[7] * y[image_width] + Q_mat_ptr[8] * y[2 * image_width] + Q_mat_ptr[9] * y[3 * image_width] + Q_mat_ptr[10] * y[4 * image_width] + Q_mat_ptr[11] * y[5 * image_width]) * Q_mat_ptr[12] + (Q_mat_ptr[6] * y[1] + Q_mat_ptr[7] * y[1 + image_width] + Q_mat_ptr[8] * y[1 + 2 * image_width] + Q_mat_ptr[9] * y[1 + 3 * image_width] + Q_mat_ptr[10] * y[1 + 4 * image_width] + Q_mat_ptr[11] * y[1 + 5 * image_width]) * Q_mat_ptr[13] + (Q_mat_ptr[6] * y[2] + Q_mat_ptr[7] * y[2 + image_width] + Q_mat_ptr[8] * y[2 + 2 * image_width] + Q_mat_ptr[9] * y[2 + 3 * image_width] + Q_mat_ptr[10] * y[2 + 4 * image_width] + Q_mat_ptr[11] * y[2 + 5 * image_width]) * Q_mat_ptr[14] + (Q_mat_ptr[6] * y[3] + Q_mat_ptr[7] * y[3 + image_width] + Q_mat_ptr[8] * y[3 + 2 * image_width] + Q_mat_ptr[9] * y[3 + 3 * image_width] + Q_mat_ptr[10] * y[3 + 4 * image_width] + Q_mat_ptr[11] * y[3 + 5 * image_width]) * Q_mat_ptr[15] + (Q_mat_ptr[6] * y[4] + Q_mat_ptr[7] * y[4 + image_width] + Q_mat_ptr[8] * y[4 + 2 * image_width] + Q_mat_ptr[9] * y[4 + 3 * image_width] + Q_mat_ptr[10] * y[4 + 4 * image_width] + Q_mat_ptr[11] * y[4 + 5 * image_width]) * Q_mat_ptr[16] + (Q_mat_ptr[6] * y[5] + Q_mat_ptr[7] * y[5 + image_width] + Q_mat_ptr[8] * y[5 + 2 * image_width] + Q_mat_ptr[9] * y[5 + 3 * image_width] + Q_mat_ptr[10] * y[5 + 4 * image_width] + Q_mat_ptr[11] * y[5 + 5 * image_width]) * Q_mat_ptr[17];
//					lut_element_ptr[7] = Q_mat_ptr[6] * y[2] + Q_mat_ptr[7] * y[2 + image_width] + Q_mat_ptr[8] * y[2 + 2 * image_width] + Q_mat_ptr[9] * y[2 + 3 * image_width] + Q_mat_ptr[10] * y[2 + 4 * image_width] + Q_mat_ptr[11] * y[2 + 5 * image_width];
//					lut_element_ptr[8] = (Q_mat_ptr[12] * y[0] + Q_mat_ptr[13] * y[image_width] + Q_mat_ptr[14] * y[2 * image_width] + Q_mat_ptr[15] * y[3 * image_width] + Q_mat_ptr[16] * y[4 * image_width] + Q_mat_ptr[17] * y[5 * image_width]) * Q_mat_ptr[0] + (Q_mat_ptr[12] * y[1] + Q_mat_ptr[13] * y[1 + image_width] + Q_mat_ptr[14] * y[1 + 2 * image_width] + Q_mat_ptr[15] * y[1 + 3 * image_width] + Q_mat_ptr[16] * y[1 + 4 * image_width] + Q_mat_ptr[17] * y[1 + 5 * image_width]) * Q_mat_ptr[1] + (Q_mat_ptr[12] * y[2] + Q_mat_ptr[13] * y[2 + image_width] + Q_mat_ptr[14] * y[2 + 2 * image_width] + Q_mat_ptr[15] * y[2 + 3 * image_width] + Q_mat_ptr[16] * y[2 + 4 * image_width] + Q_mat_ptr[17] * y[2 + 5 * image_width]) * Q_mat_ptr[2] + (Q_mat_ptr[12] * y[3] + Q_mat_ptr[13] * y[3 + image_width] + Q_mat_ptr[14] * y[3 + 2 * image_width] + Q_mat_ptr[15] * y[3 + 3 * image_width] + Q_mat_ptr[16] * y[3 + 4 * image_width] + Q_mat_ptr[17] * y[3 + 5 * image_width]) * Q_mat_ptr[3] + (Q_mat_ptr[12] * y[4] + Q_mat_ptr[13] * y[4 + image_width] + Q_mat_ptr[14] * y[4 + 2 * image_width] + Q_mat_ptr[15] * y[4 + 3 * image_width] + Q_mat_ptr[16] * y[4 + 4 * image_width] + Q_mat_ptr[17] * y[4 + 5 * image_width]) * Q_mat_ptr[4] + (Q_mat_ptr[12] * y[5] + Q_mat_ptr[13] * y[5 + image_width] + Q_mat_ptr[14] * y[5 + 2 * image_width] + Q_mat_ptr[15] * y[5 + 3 * image_width] + Q_mat_ptr[16] * y[5 + 4 * image_width] + Q_mat_ptr[17] * y[5 + 5 * image_width]) * Q_mat_ptr[5];
//					lut_element_ptr[9] = (Q_mat_ptr[12] * y[0] + Q_mat_ptr[13] * y[image_width] + Q_mat_ptr[14] * y[2 * image_width] + Q_mat_ptr[15] * y[3 * image_width] + Q_mat_ptr[16] * y[4 * image_width] + Q_mat_ptr[17] * y[5 * image_width]) * Q_mat_ptr[6] + (Q_mat_ptr[12] * y[1] + Q_mat_ptr[13] * y[1 + image_width] + Q_mat_ptr[14] * y[1 + 2 * image_width] + Q_mat_ptr[15] * y[1 + 3 * image_width] + Q_mat_ptr[16] * y[1 + 4 * image_width] + Q_mat_ptr[17] * y[1 + 5 * image_width]) * Q_mat_ptr[7] + (Q_mat_ptr[12] * y[2] + Q_mat_ptr[13] * y[2 + image_width] + Q_mat_ptr[14] * y[2 + 2 * image_width] + Q_mat_ptr[15] * y[2 + 3 * image_width] + Q_mat_ptr[16] * y[2 + 4 * image_width] + Q_mat_ptr[17] * y[2 + 5 * image_width]) * Q_mat_ptr[8] + (Q_mat_ptr[12] * y[3] + Q_mat_ptr[13] * y[3 + image_width] + Q_mat_ptr[14] * y[3 + 2 * image_width] + Q_mat_ptr[15] * y[3 + 3 * image_width] + Q_mat_ptr[16] * y[3 + 4 * image_width] + Q_mat_ptr[17] * y[3 + 5 * image_width]) * Q_mat_ptr[9] + (Q_mat_ptr[12] * y[4] + Q_mat_ptr[13] * y[4 + image_width] + Q_mat_ptr[14] * y[4 + 2 * image_width] + Q_mat_ptr[15] * y[4 + 3 * image_width] + Q_mat_ptr[16] * y[4 + 4 * image_width] + Q_mat_ptr[17] * y[4 + 5 * image_width]) * Q_mat_ptr[10] + (Q_mat_ptr[12] * y[5] + Q_mat_ptr[13] * y[5 + image_width] + Q_mat_ptr[14] * y[5 + 2 * image_width] + Q_mat_ptr[15] * y[5 + 3 * image_width] + Q_mat_ptr[16] * y[5 + 4 * image_width] + Q_mat_ptr[17] * y[5 + 5 * image_width]) * Q_mat_ptr[11];
//					lut_element_ptr[10] = (Q_mat_ptr[12] * y[0] + Q_mat_ptr[13] * y[image_width] + Q_mat_ptr[14] * y[2 * image_width] + Q_mat_ptr[15] * y[3 * image_width] + Q_mat_ptr[16] * y[4 * image_width] + Q_mat_ptr[17] * y[5 * image_width]) * Q_mat_ptr[12] + (Q_mat_ptr[12] * y[1] + Q_mat_ptr[13] * y[1 + image_width] + Q_mat_ptr[14] * y[1 + 2 * image_width] + Q_mat_ptr[15] * y[1 + 3 * image_width] + Q_mat_ptr[16] * y[1 + 4 * image_width] + Q_mat_ptr[17] * y[1 + 5 * image_width]) * Q_mat_ptr[13] + (Q_mat_ptr[12] * y[2] + Q_mat_ptr[13] * y[2 + image_width] + Q_mat_ptr[14] * y[2 + 2 * image_width] + Q_mat_ptr[15] * y[2 + 3 * image_width] + Q_mat_ptr[16] * y[2 + 4 * image_width] + Q_mat_ptr[17] * y[2 + 5 * image_width]) * Q_mat_ptr[14] + (Q_mat_ptr[12] * y[3] + Q_mat_ptr[13] * y[3 + image_width] + Q_mat_ptr[14] * y[3 + 2 * image_width] + Q_mat_ptr[15] * y[3 + 3 * image_width] + Q_mat_ptr[16] * y[3 + 4 * image_width] + Q_mat_ptr[17] * y[3 + 5 * image_width]) * Q_mat_ptr[15] + (Q_mat_ptr[12] * y[4] + Q_mat_ptr[13] * y[4 + image_width] + Q_mat_ptr[14] * y[4 + 2 * image_width] + Q_mat_ptr[15] * y[4 + 3 * image_width] + Q_mat_ptr[16] * y[4 + 4 * image_width] + Q_mat_ptr[17] * y[4 + 5 * image_width]) * Q_mat_ptr[16] + (Q_mat_ptr[12] * y[5] + Q_mat_ptr[13] * y[5 + image_width] + Q_mat_ptr[14] * y[5 + 2 * image_width] + Q_mat_ptr[15] * y[5 + 3 * image_width] + Q_mat_ptr[16] * y[5 + 4 * image_width] + Q_mat_ptr[17] * y[5 + 5 * image_width]) * Q_mat_ptr[17];
//					lut_element_ptr[11] = Q_mat_ptr[12] * y[2] + Q_mat_ptr[13] * y[2 + image_width] + Q_mat_ptr[14] * y[2 + 2 * image_width] + Q_mat_ptr[15] * y[2 + 3 * image_width] + Q_mat_ptr[16] * y[2 + 4 * image_width] + Q_mat_ptr[17] * y[2 + 5 * image_width];
//					lut_element_ptr[12] = y[2 * image_width] * Q_mat_ptr[0] + y[1 + 2 * image_width] * Q_mat_ptr[1] + Q_mat_ptr[2] * y[2 + 2 * image_width] + y[3 + 2 * image_width] * Q_mat_ptr[3] + y[4 + 2 * image_width] * Q_mat_ptr[4] + y[5 + 2 * image_width] * Q_mat_ptr[5];
//					lut_element_ptr[13] = y[2 * image_width] * Q_mat_ptr[6] + y[1 + 2 * image_width] * Q_mat_ptr[7] + Q_mat_ptr[8] * y[2 + 2 * image_width] + y[3 + 2 * image_width] * Q_mat_ptr[9] + y[4 + 2 * image_width] * Q_mat_ptr[10] + y[5 + 2 * image_width] * Q_mat_ptr[11];
//					lut_element_ptr[14] = y[2 * image_width] * Q_mat_ptr[12] + y[1 + 2 * image_width] * Q_mat_ptr[13] + Q_mat_ptr[14] * y[2 + 2 * image_width] + y[3 + 2 * image_width] * Q_mat_ptr[15] + y[4 + 2 * image_width] * Q_mat_ptr[16] + y[5 + 2 * image_width] * Q_mat_ptr[17];
//					lut_element_ptr[15] = y[2 + 2 * image_width];
//				}
//				else
//				{
//					index_i_min = key.index_row_-3;
//					index_j_min = key.index_col_-3;
//					uchar* y = image->data.ptr + index_i_min*image_width+index_j_min;
//
//					lut_element_ptr[0] = (Q_mat_ptr[0] * y[0] + Q_mat_ptr[1] * y[image_width] + Q_mat_ptr[2] * y[2 * image_width] + Q_mat_ptr[3] * y[3 * image_width] + Q_mat_ptr[4] * y[4 * image_width] + Q_mat_ptr[5] * y[5 * image_width] + Q_mat_ptr[6] * y[6 * image_width] + Q_mat_ptr[7] * y[7 * image_width]) * Q_mat_ptr[0] + (Q_mat_ptr[0] * y[1] + Q_mat_ptr[1] * y[1 + image_width] + Q_mat_ptr[2] * y[1 + 2 * image_width] + Q_mat_ptr[3] * y[1 + 3 * image_width] + Q_mat_ptr[4] * y[1 + 4 * image_width] + Q_mat_ptr[5] * y[1 + 5 * image_width] + Q_mat_ptr[6] * y[1 + 6 * image_width] + Q_mat_ptr[7] * y[1 + 7 * image_width]) * Q_mat_ptr[1] + (Q_mat_ptr[0] * y[2] + Q_mat_ptr[1] * y[2 + image_width] + Q_mat_ptr[2] * y[2 + 2 * image_width] + Q_mat_ptr[3] * y[2 + 3 * image_width] + Q_mat_ptr[4] * y[2 + 4 * image_width] + Q_mat_ptr[5] * y[2 + 5 * image_width] + Q_mat_ptr[6] * y[2 + 6 * image_width] + Q_mat_ptr[7] * y[2 + 7 * image_width]) * Q_mat_ptr[2] + (Q_mat_ptr[0] * y[3] + Q_mat_ptr[1] * y[3 + image_width] + Q_mat_ptr[2] * y[3 + 2 * image_width] + Q_mat_ptr[3] * y[3 + 3 * image_width] + Q_mat_ptr[4] * y[3 + 4 * image_width] + Q_mat_ptr[5] * y[3 + 5 * image_width] + Q_mat_ptr[6] * y[3 + 6 * image_width] + Q_mat_ptr[7] * y[3 + 7 * image_width]) * Q_mat_ptr[3] + (Q_mat_ptr[0] * y[4] + Q_mat_ptr[1] * y[4 + image_width] + Q_mat_ptr[2] * y[4 + 2 * image_width] + Q_mat_ptr[3] * y[4 + 3 * image_width] + Q_mat_ptr[4] * y[4 + 4 * image_width] + Q_mat_ptr[5] * y[4 + 5 * image_width] + Q_mat_ptr[6] * y[4 + 6 * image_width] + Q_mat_ptr[7] * y[4 + 7 * image_width]) * Q_mat_ptr[4] + (Q_mat_ptr[0] * y[5] + Q_mat_ptr[1] * y[5 + image_width] + Q_mat_ptr[2] * y[5 + 2 * image_width] + Q_mat_ptr[3] * y[5 + 3 * image_width] + Q_mat_ptr[4] * y[5 + 4 * image_width] + Q_mat_ptr[5] * y[5 + 5 * image_width] + Q_mat_ptr[6] * y[5 + 6 * image_width] + Q_mat_ptr[7] * y[5 + 7 * image_width]) * Q_mat_ptr[5] + (Q_mat_ptr[0] * y[6] + Q_mat_ptr[1] * y[6 + image_width] + Q_mat_ptr[2] * y[6 + 2 * image_width] + Q_mat_ptr[3] * y[6 + 3 * image_width] + Q_mat_ptr[4] * y[6 + 4 * image_width] + Q_mat_ptr[5] * y[6 + 5 * image_width] + Q_mat_ptr[6] * y[6 + 6 * image_width] + Q_mat_ptr[7] * y[6 + 7 * image_width]) * Q_mat_ptr[6] + (Q_mat_ptr[0] * y[7] + Q_mat_ptr[1] * y[7 + image_width] + Q_mat_ptr[2] * y[7 + 2 * image_width] + Q_mat_ptr[3] * y[7 + 3 * image_width] + Q_mat_ptr[4] * y[7 + 4 * image_width] + Q_mat_ptr[5] * y[7 + 5 * image_width] + Q_mat_ptr[6] * y[7 + 6 * image_width] + Q_mat_ptr[7] * y[7 + 7 * image_width]) * Q_mat_ptr[7];
//					lut_element_ptr[1] = (Q_mat_ptr[0] * y[0] + Q_mat_ptr[1] * y[image_width] + Q_mat_ptr[2] * y[2 * image_width] + Q_mat_ptr[3] * y[3 * image_width] + Q_mat_ptr[4] * y[4 * image_width] + Q_mat_ptr[5] * y[5 * image_width] + Q_mat_ptr[6] * y[6 * image_width] + Q_mat_ptr[7] * y[7 * image_width]) * Q_mat_ptr[8] + (Q_mat_ptr[0] * y[1] + Q_mat_ptr[1] * y[1 + image_width] + Q_mat_ptr[2] * y[1 + 2 * image_width] + Q_mat_ptr[3] * y[1 + 3 * image_width] + Q_mat_ptr[4] * y[1 + 4 * image_width] + Q_mat_ptr[5] * y[1 + 5 * image_width] + Q_mat_ptr[6] * y[1 + 6 * image_width] + Q_mat_ptr[7] * y[1 + 7 * image_width]) * Q_mat_ptr[9] + (Q_mat_ptr[0] * y[2] + Q_mat_ptr[1] * y[2 + image_width] + Q_mat_ptr[2] * y[2 + 2 * image_width] + Q_mat_ptr[3] * y[2 + 3 * image_width] + Q_mat_ptr[4] * y[2 + 4 * image_width] + Q_mat_ptr[5] * y[2 + 5 * image_width] + Q_mat_ptr[6] * y[2 + 6 * image_width] + Q_mat_ptr[7] * y[2 + 7 * image_width]) * Q_mat_ptr[10] + (Q_mat_ptr[0] * y[3] + Q_mat_ptr[1] * y[3 + image_width] + Q_mat_ptr[2] * y[3 + 2 * image_width] + Q_mat_ptr[3] * y[3 + 3 * image_width] + Q_mat_ptr[4] * y[3 + 4 * image_width] + Q_mat_ptr[5] * y[3 + 5 * image_width] + Q_mat_ptr[6] * y[3 + 6 * image_width] + Q_mat_ptr[7] * y[3 + 7 * image_width]) * Q_mat_ptr[11] + (Q_mat_ptr[0] * y[4] + Q_mat_ptr[1] * y[4 + image_width] + Q_mat_ptr[2] * y[4 + 2 * image_width] + Q_mat_ptr[3] * y[4 + 3 * image_width] + Q_mat_ptr[4] * y[4 + 4 * image_width] + Q_mat_ptr[5] * y[4 + 5 * image_width] + Q_mat_ptr[6] * y[4 + 6 * image_width] + Q_mat_ptr[7] * y[4 + 7 * image_width]) * Q_mat_ptr[12] + (Q_mat_ptr[0] * y[5] + Q_mat_ptr[1] * y[5 + image_width] + Q_mat_ptr[2] * y[5 + 2 * image_width] + Q_mat_ptr[3] * y[5 + 3 * image_width] + Q_mat_ptr[4] * y[5 + 4 * image_width] + Q_mat_ptr[5] * y[5 + 5 * image_width] + Q_mat_ptr[6] * y[5 + 6 * image_width] + Q_mat_ptr[7] * y[5 + 7 * image_width]) * Q_mat_ptr[13] + (Q_mat_ptr[0] * y[6] + Q_mat_ptr[1] * y[6 + image_width] + Q_mat_ptr[2] * y[6 + 2 * image_width] + Q_mat_ptr[3] * y[6 + 3 * image_width] + Q_mat_ptr[4] * y[6 + 4 * image_width] + Q_mat_ptr[5] * y[6 + 5 * image_width] + Q_mat_ptr[6] * y[6 + 6 * image_width] + Q_mat_ptr[7] * y[6 + 7 * image_width]) * Q_mat_ptr[14] + (Q_mat_ptr[0] * y[7] + Q_mat_ptr[1] * y[7 + image_width] + Q_mat_ptr[2] * y[7 + 2 * image_width] + Q_mat_ptr[3] * y[7 + 3 * image_width] + Q_mat_ptr[4] * y[7 + 4 * image_width] + Q_mat_ptr[5] * y[7 + 5 * image_width] + Q_mat_ptr[6] * y[7 + 6 * image_width] + Q_mat_ptr[7] * y[7 + 7 * image_width]) * Q_mat_ptr[15];
//					lut_element_ptr[2] = (Q_mat_ptr[0] * y[0] + Q_mat_ptr[1] * y[image_width] + Q_mat_ptr[2] * y[2 * image_width] + Q_mat_ptr[3] * y[3 * image_width] + Q_mat_ptr[4] * y[4 * image_width] + Q_mat_ptr[5] * y[5 * image_width] + Q_mat_ptr[6] * y[6 * image_width] + Q_mat_ptr[7] * y[7 * image_width]) * Q_mat_ptr[16] + (Q_mat_ptr[0] * y[1] + Q_mat_ptr[1] * y[1 + image_width] + Q_mat_ptr[2] * y[1 + 2 * image_width] + Q_mat_ptr[3] * y[1 + 3 * image_width] + Q_mat_ptr[4] * y[1 + 4 * image_width] + Q_mat_ptr[5] * y[1 + 5 * image_width] + Q_mat_ptr[6] * y[1 + 6 * image_width] + Q_mat_ptr[7] * y[1 + 7 * image_width]) * Q_mat_ptr[17] + (Q_mat_ptr[0] * y[2] + Q_mat_ptr[1] * y[2 + image_width] + Q_mat_ptr[2] * y[2 + 2 * image_width] + Q_mat_ptr[3] * y[2 + 3 * image_width] + Q_mat_ptr[4] * y[2 + 4 * image_width] + Q_mat_ptr[5] * y[2 + 5 * image_width] + Q_mat_ptr[6] * y[2 + 6 * image_width] + Q_mat_ptr[7] * y[2 + 7 * image_width]) * Q_mat_ptr[18] + (Q_mat_ptr[0] * y[3] + Q_mat_ptr[1] * y[3 + image_width] + Q_mat_ptr[2] * y[3 + 2 * image_width] + Q_mat_ptr[3] * y[3 + 3 * image_width] + Q_mat_ptr[4] * y[3 + 4 * image_width] + Q_mat_ptr[5] * y[3 + 5 * image_width] + Q_mat_ptr[6] * y[3 + 6 * image_width] + Q_mat_ptr[7] * y[3 + 7 * image_width]) * Q_mat_ptr[19] + (Q_mat_ptr[0] * y[4] + Q_mat_ptr[1] * y[4 + image_width] + Q_mat_ptr[2] * y[4 + 2 * image_width] + Q_mat_ptr[3] * y[4 + 3 * image_width] + Q_mat_ptr[4] * y[4 + 4 * image_width] + Q_mat_ptr[5] * y[4 + 5 * image_width] + Q_mat_ptr[6] * y[4 + 6 * image_width] + Q_mat_ptr[7] * y[4 + 7 * image_width]) * Q_mat_ptr[20] + (Q_mat_ptr[0] * y[5] + Q_mat_ptr[1] * y[5 + image_width] + Q_mat_ptr[2] * y[5 + 2 * image_width] + Q_mat_ptr[3] * y[5 + 3 * image_width] + Q_mat_ptr[4] * y[5 + 4 * image_width] + Q_mat_ptr[5] * y[5 + 5 * image_width] + Q_mat_ptr[6] * y[5 + 6 * image_width] + Q_mat_ptr[7] * y[5 + 7 * image_width]) * Q_mat_ptr[21] + (Q_mat_ptr[0] * y[6] + Q_mat_ptr[1] * y[6 + image_width] + Q_mat_ptr[2] * y[6 + 2 * image_width] + Q_mat_ptr[3] * y[6 + 3 * image_width] + Q_mat_ptr[4] * y[6 + 4 * image_width] + Q_mat_ptr[5] * y[6 + 5 * image_width] + Q_mat_ptr[6] * y[6 + 6 * image_width] + Q_mat_ptr[7] * y[6 + 7 * image_width]) * Q_mat_ptr[22] + (Q_mat_ptr[0] * y[7] + Q_mat_ptr[1] * y[7 + image_width] + Q_mat_ptr[2] * y[7 + 2 * image_width] + Q_mat_ptr[3] * y[7 + 3 * image_width] + Q_mat_ptr[4] * y[7 + 4 * image_width] + Q_mat_ptr[5] * y[7 + 5 * image_width] + Q_mat_ptr[6] * y[7 + 6 * image_width] + Q_mat_ptr[7] * y[7 + 7 * image_width]) * Q_mat_ptr[23];
//					lut_element_ptr[3] = Q_mat_ptr[0] * y[3] + Q_mat_ptr[1] * y[3 + image_width] + Q_mat_ptr[2] * y[3 + 2 * image_width] + Q_mat_ptr[3] * y[3 + 3 * image_width] + Q_mat_ptr[4] * y[3 + 4 * image_width] + Q_mat_ptr[5] * y[3 + 5 * image_width] + Q_mat_ptr[6] * y[3 + 6 * image_width] + Q_mat_ptr[7] * y[3 + 7 * image_width];
//					lut_element_ptr[4] = (Q_mat_ptr[8] * y[0] + Q_mat_ptr[9] * y[image_width] + Q_mat_ptr[10] * y[2 * image_width] + Q_mat_ptr[11] * y[3 * image_width] + Q_mat_ptr[12] * y[4 * image_width] + Q_mat_ptr[13] * y[5 * image_width] + Q_mat_ptr[14] * y[6 * image_width] + Q_mat_ptr[15] * y[7 * image_width]) * Q_mat_ptr[0] + (Q_mat_ptr[8] * y[1] + Q_mat_ptr[9] * y[1 + image_width] + Q_mat_ptr[10] * y[1 + 2 * image_width] + Q_mat_ptr[11] * y[1 + 3 * image_width] + Q_mat_ptr[12] * y[1 + 4 * image_width] + Q_mat_ptr[13] * y[1 + 5 * image_width] + Q_mat_ptr[14] * y[1 + 6 * image_width] + Q_mat_ptr[15] * y[1 + 7 * image_width]) * Q_mat_ptr[1] + (Q_mat_ptr[8] * y[2] + Q_mat_ptr[9] * y[2 + image_width] + Q_mat_ptr[10] * y[2 + 2 * image_width] + Q_mat_ptr[11] * y[2 + 3 * image_width] + Q_mat_ptr[12] * y[2 + 4 * image_width] + Q_mat_ptr[13] * y[2 + 5 * image_width] + Q_mat_ptr[14] * y[2 + 6 * image_width] + Q_mat_ptr[15] * y[2 + 7 * image_width]) * Q_mat_ptr[2] + (Q_mat_ptr[8] * y[3] + Q_mat_ptr[9] * y[3 + image_width] + Q_mat_ptr[10] * y[3 + 2 * image_width] + Q_mat_ptr[11] * y[3 + 3 * image_width] + Q_mat_ptr[12] * y[3 + 4 * image_width] + Q_mat_ptr[13] * y[3 + 5 * image_width] + Q_mat_ptr[14] * y[3 + 6 * image_width] + Q_mat_ptr[15] * y[3 + 7 * image_width]) * Q_mat_ptr[3] + (Q_mat_ptr[8] * y[4] + Q_mat_ptr[9] * y[4 + image_width] + Q_mat_ptr[10] * y[4 + 2 * image_width] + Q_mat_ptr[11] * y[4 + 3 * image_width] + Q_mat_ptr[12] * y[4 + 4 * image_width] + Q_mat_ptr[13] * y[4 + 5 * image_width] + Q_mat_ptr[14] * y[4 + 6 * image_width] + Q_mat_ptr[15] * y[4 + 7 * image_width]) * Q_mat_ptr[4] + (Q_mat_ptr[8] * y[5] + Q_mat_ptr[9] * y[5 + image_width] + Q_mat_ptr[10] * y[5 + 2 * image_width] + Q_mat_ptr[11] * y[5 + 3 * image_width] + Q_mat_ptr[12] * y[5 + 4 * image_width] + Q_mat_ptr[13] * y[5 + 5 * image_width] + Q_mat_ptr[14] * y[5 + 6 * image_width] + Q_mat_ptr[15] * y[5 + 7 * image_width]) * Q_mat_ptr[5] + (Q_mat_ptr[8] * y[6] + Q_mat_ptr[9] * y[6 + image_width] + Q_mat_ptr[10] * y[6 + 2 * image_width] + Q_mat_ptr[11] * y[6 + 3 * image_width] + Q_mat_ptr[12] * y[6 + 4 * image_width] + Q_mat_ptr[13] * y[6 + 5 * image_width] + Q_mat_ptr[14] * y[6 + 6 * image_width] + Q_mat_ptr[15] * y[6 + 7 * image_width]) * Q_mat_ptr[6] + (Q_mat_ptr[8] * y[7] + Q_mat_ptr[9] * y[7 + image_width] + Q_mat_ptr[10] * y[7 + 2 * image_width] + Q_mat_ptr[11] * y[7 + 3 * image_width] + Q_mat_ptr[12] * y[7 + 4 * image_width] + Q_mat_ptr[13] * y[7 + 5 * image_width] + Q_mat_ptr[14] * y[7 + 6 * image_width] + Q_mat_ptr[15] * y[7 + 7 * image_width]) * Q_mat_ptr[7];
//					lut_element_ptr[5] = (Q_mat_ptr[8] * y[0] + Q_mat_ptr[9] * y[image_width] + Q_mat_ptr[10] * y[2 * image_width] + Q_mat_ptr[11] * y[3 * image_width] + Q_mat_ptr[12] * y[4 * image_width] + Q_mat_ptr[13] * y[5 * image_width] + Q_mat_ptr[14] * y[6 * image_width] + Q_mat_ptr[15] * y[7 * image_width]) * Q_mat_ptr[8] + (Q_mat_ptr[8] * y[1] + Q_mat_ptr[9] * y[1 + image_width] + Q_mat_ptr[10] * y[1 + 2 * image_width] + Q_mat_ptr[11] * y[1 + 3 * image_width] + Q_mat_ptr[12] * y[1 + 4 * image_width] + Q_mat_ptr[13] * y[1 + 5 * image_width] + Q_mat_ptr[14] * y[1 + 6 * image_width] + Q_mat_ptr[15] * y[1 + 7 * image_width]) * Q_mat_ptr[9] + (Q_mat_ptr[8] * y[2] + Q_mat_ptr[9] * y[2 + image_width] + Q_mat_ptr[10] * y[2 + 2 * image_width] + Q_mat_ptr[11] * y[2 + 3 * image_width] + Q_mat_ptr[12] * y[2 + 4 * image_width] + Q_mat_ptr[13] * y[2 + 5 * image_width] + Q_mat_ptr[14] * y[2 + 6 * image_width] + Q_mat_ptr[15] * y[2 + 7 * image_width]) * Q_mat_ptr[10] + (Q_mat_ptr[8] * y[3] + Q_mat_ptr[9] * y[3 + image_width] + Q_mat_ptr[10] * y[3 + 2 * image_width] + Q_mat_ptr[11] * y[3 + 3 * image_width] + Q_mat_ptr[12] * y[3 + 4 * image_width] + Q_mat_ptr[13] * y[3 + 5 * image_width] + Q_mat_ptr[14] * y[3 + 6 * image_width] + Q_mat_ptr[15] * y[3 + 7 * image_width]) * Q_mat_ptr[11] + (Q_mat_ptr[8] * y[4] + Q_mat_ptr[9] * y[4 + image_width] + Q_mat_ptr[10] * y[4 + 2 * image_width] + Q_mat_ptr[11] * y[4 + 3 * image_width] + Q_mat_ptr[12] * y[4 + 4 * image_width] + Q_mat_ptr[13] * y[4 + 5 * image_width] + Q_mat_ptr[14] * y[4 + 6 * image_width] + Q_mat_ptr[15] * y[4 + 7 * image_width]) * Q_mat_ptr[12] + (Q_mat_ptr[8] * y[5] + Q_mat_ptr[9] * y[5 + image_width] + Q_mat_ptr[10] * y[5 + 2 * image_width] + Q_mat_ptr[11] * y[5 + 3 * image_width] + Q_mat_ptr[12] * y[5 + 4 * image_width] + Q_mat_ptr[13] * y[5 + 5 * image_width] + Q_mat_ptr[14] * y[5 + 6 * image_width] + Q_mat_ptr[15] * y[5 + 7 * image_width]) * Q_mat_ptr[13] + (Q_mat_ptr[8] * y[6] + Q_mat_ptr[9] * y[6 + image_width] + Q_mat_ptr[10] * y[6 + 2 * image_width] + Q_mat_ptr[11] * y[6 + 3 * image_width] + Q_mat_ptr[12] * y[6 + 4 * image_width] + Q_mat_ptr[13] * y[6 + 5 * image_width] + Q_mat_ptr[14] * y[6 + 6 * image_width] + Q_mat_ptr[15] * y[6 + 7 * image_width]) * Q_mat_ptr[14] + (Q_mat_ptr[8] * y[7] + Q_mat_ptr[9] * y[7 + image_width] + Q_mat_ptr[10] * y[7 + 2 * image_width] + Q_mat_ptr[11] * y[7 + 3 * image_width] + Q_mat_ptr[12] * y[7 + 4 * image_width] + Q_mat_ptr[13] * y[7 + 5 * image_width] + Q_mat_ptr[14] * y[7 + 6 * image_width] + Q_mat_ptr[15] * y[7 + 7 * image_width]) * Q_mat_ptr[15];
//					lut_element_ptr[6] = (Q_mat_ptr[8] * y[0] + Q_mat_ptr[9] * y[image_width] + Q_mat_ptr[10] * y[2 * image_width] + Q_mat_ptr[11] * y[3 * image_width] + Q_mat_ptr[12] * y[4 * image_width] + Q_mat_ptr[13] * y[5 * image_width] + Q_mat_ptr[14] * y[6 * image_width] + Q_mat_ptr[15] * y[7 * image_width]) * Q_mat_ptr[16] + (Q_mat_ptr[8] * y[1] + Q_mat_ptr[9] * y[1 + image_width] + Q_mat_ptr[10] * y[1 + 2 * image_width] + Q_mat_ptr[11] * y[1 + 3 * image_width] + Q_mat_ptr[12] * y[1 + 4 * image_width] + Q_mat_ptr[13] * y[1 + 5 * image_width] + Q_mat_ptr[14] * y[1 + 6 * image_width] + Q_mat_ptr[15] * y[1 + 7 * image_width]) * Q_mat_ptr[17] + (Q_mat_ptr[8] * y[2] + Q_mat_ptr[9] * y[2 + image_width] + Q_mat_ptr[10] * y[2 + 2 * image_width] + Q_mat_ptr[11] * y[2 + 3 * image_width] + Q_mat_ptr[12] * y[2 + 4 * image_width] + Q_mat_ptr[13] * y[2 + 5 * image_width] + Q_mat_ptr[14] * y[2 + 6 * image_width] + Q_mat_ptr[15] * y[2 + 7 * image_width]) * Q_mat_ptr[18] + (Q_mat_ptr[8] * y[3] + Q_mat_ptr[9] * y[3 + image_width] + Q_mat_ptr[10] * y[3 + 2 * image_width] + Q_mat_ptr[11] * y[3 + 3 * image_width] + Q_mat_ptr[12] * y[3 + 4 * image_width] + Q_mat_ptr[13] * y[3 + 5 * image_width] + Q_mat_ptr[14] * y[3 + 6 * image_width] + Q_mat_ptr[15] * y[3 + 7 * image_width]) * Q_mat_ptr[19] + (Q_mat_ptr[8] * y[4] + Q_mat_ptr[9] * y[4 + image_width] + Q_mat_ptr[10] * y[4 + 2 * image_width] + Q_mat_ptr[11] * y[4 + 3 * image_width] + Q_mat_ptr[12] * y[4 + 4 * image_width] + Q_mat_ptr[13] * y[4 + 5 * image_width] + Q_mat_ptr[14] * y[4 + 6 * image_width] + Q_mat_ptr[15] * y[4 + 7 * image_width]) * Q_mat_ptr[20] + (Q_mat_ptr[8] * y[5] + Q_mat_ptr[9] * y[5 + image_width] + Q_mat_ptr[10] * y[5 + 2 * image_width] + Q_mat_ptr[11] * y[5 + 3 * image_width] + Q_mat_ptr[12] * y[5 + 4 * image_width] + Q_mat_ptr[13] * y[5 + 5 * image_width] + Q_mat_ptr[14] * y[5 + 6 * image_width] + Q_mat_ptr[15] * y[5 + 7 * image_width]) * Q_mat_ptr[21] + (Q_mat_ptr[8] * y[6] + Q_mat_ptr[9] * y[6 + image_width] + Q_mat_ptr[10] * y[6 + 2 * image_width] + Q_mat_ptr[11] * y[6 + 3 * image_width] + Q_mat_ptr[12] * y[6 + 4 * image_width] + Q_mat_ptr[13] * y[6 + 5 * image_width] + Q_mat_ptr[14] * y[6 + 6 * image_width] + Q_mat_ptr[15] * y[6 + 7 * image_width]) * Q_mat_ptr[22] + (Q_mat_ptr[8] * y[7] + Q_mat_ptr[9] * y[7 + image_width] + Q_mat_ptr[10] * y[7 + 2 * image_width] + Q_mat_ptr[11] * y[7 + 3 * image_width] + Q_mat_ptr[12] * y[7 + 4 * image_width] + Q_mat_ptr[13] * y[7 + 5 * image_width] + Q_mat_ptr[14] * y[7 + 6 * image_width] + Q_mat_ptr[15] * y[7 + 7 * image_width]) * Q_mat_ptr[23];
//					lut_element_ptr[7] = Q_mat_ptr[8] * y[3] + Q_mat_ptr[9] * y[3 + image_width] + Q_mat_ptr[10] * y[3 + 2 * image_width] + Q_mat_ptr[11] * y[3 + 3 * image_width] + Q_mat_ptr[12] * y[3 + 4 * image_width] + Q_mat_ptr[13] * y[3 + 5 * image_width] + Q_mat_ptr[14] * y[3 + 6 * image_width] + Q_mat_ptr[15] * y[3 + 7 * image_width];
//					lut_element_ptr[8] = (Q_mat_ptr[16] * y[0] + Q_mat_ptr[17] * y[image_width] + Q_mat_ptr[18] * y[2 * image_width] + Q_mat_ptr[19] * y[3 * image_width] + Q_mat_ptr[20] * y[4 * image_width] + Q_mat_ptr[21] * y[5 * image_width] + Q_mat_ptr[22] * y[6 * image_width] + Q_mat_ptr[23] * y[7 * image_width]) * Q_mat_ptr[0] + (Q_mat_ptr[16] * y[1] + Q_mat_ptr[17] * y[1 + image_width] + Q_mat_ptr[18] * y[1 + 2 * image_width] + Q_mat_ptr[19] * y[1 + 3 * image_width] + Q_mat_ptr[20] * y[1 + 4 * image_width] + Q_mat_ptr[21] * y[1 + 5 * image_width] + Q_mat_ptr[22] * y[1 + 6 * image_width] + Q_mat_ptr[23] * y[1 + 7 * image_width]) * Q_mat_ptr[1] + (Q_mat_ptr[16] * y[2] + Q_mat_ptr[17] * y[2 + image_width] + Q_mat_ptr[18] * y[2 + 2 * image_width] + Q_mat_ptr[19] * y[2 + 3 * image_width] + Q_mat_ptr[20] * y[2 + 4 * image_width] + Q_mat_ptr[21] * y[2 + 5 * image_width] + Q_mat_ptr[22] * y[2 + 6 * image_width] + Q_mat_ptr[23] * y[2 + 7 * image_width]) * Q_mat_ptr[2] + (Q_mat_ptr[16] * y[3] + Q_mat_ptr[17] * y[3 + image_width] + Q_mat_ptr[18] * y[3 + 2 * image_width] + Q_mat_ptr[19] * y[3 + 3 * image_width] + Q_mat_ptr[20] * y[3 + 4 * image_width] + Q_mat_ptr[21] * y[3 + 5 * image_width] + Q_mat_ptr[22] * y[3 + 6 * image_width] + Q_mat_ptr[23] * y[3 + 7 * image_width]) * Q_mat_ptr[3] + (Q_mat_ptr[16] * y[4] + Q_mat_ptr[17] * y[4 + image_width] + Q_mat_ptr[18] * y[4 + 2 * image_width] + Q_mat_ptr[19] * y[4 + 3 * image_width] + Q_mat_ptr[20] * y[4 + 4 * image_width] + Q_mat_ptr[21] * y[4 + 5 * image_width] + Q_mat_ptr[22] * y[4 + 6 * image_width] + Q_mat_ptr[23] * y[4 + 7 * image_width]) * Q_mat_ptr[4] + (Q_mat_ptr[16] * y[5] + Q_mat_ptr[17] * y[5 + image_width] + Q_mat_ptr[18] * y[5 + 2 * image_width] + Q_mat_ptr[19] * y[5 + 3 * image_width] + Q_mat_ptr[20] * y[5 + 4 * image_width] + Q_mat_ptr[21] * y[5 + 5 * image_width] + Q_mat_ptr[22] * y[5 + 6 * image_width] + Q_mat_ptr[23] * y[5 + 7 * image_width]) * Q_mat_ptr[5] + (Q_mat_ptr[16] * y[6] + Q_mat_ptr[17] * y[6 + image_width] + Q_mat_ptr[18] * y[6 + 2 * image_width] + Q_mat_ptr[19] * y[6 + 3 * image_width] + Q_mat_ptr[20] * y[6 + 4 * image_width] + Q_mat_ptr[21] * y[6 + 5 * image_width] + Q_mat_ptr[22] * y[6 + 6 * image_width] + Q_mat_ptr[23] * y[6 + 7 * image_width]) * Q_mat_ptr[6] + (Q_mat_ptr[16] * y[7] + Q_mat_ptr[17] * y[7 + image_width] + Q_mat_ptr[18] * y[7 + 2 * image_width] + Q_mat_ptr[19] * y[7 + 3 * image_width] + Q_mat_ptr[20] * y[7 + 4 * image_width] + Q_mat_ptr[21] * y[7 + 5 * image_width] + Q_mat_ptr[22] * y[7 + 6 * image_width] + Q_mat_ptr[23] * y[7 + 7 * image_width]) * Q_mat_ptr[7];
//					lut_element_ptr[9] = (Q_mat_ptr[16] * y[0] + Q_mat_ptr[17] * y[image_width] + Q_mat_ptr[18] * y[2 * image_width] + Q_mat_ptr[19] * y[3 * image_width] + Q_mat_ptr[20] * y[4 * image_width] + Q_mat_ptr[21] * y[5 * image_width] + Q_mat_ptr[22] * y[6 * image_width] + Q_mat_ptr[23] * y[7 * image_width]) * Q_mat_ptr[8] + (Q_mat_ptr[16] * y[1] + Q_mat_ptr[17] * y[1 + image_width] + Q_mat_ptr[18] * y[1 + 2 * image_width] + Q_mat_ptr[19] * y[1 + 3 * image_width] + Q_mat_ptr[20] * y[1 + 4 * image_width] + Q_mat_ptr[21] * y[1 + 5 * image_width] + Q_mat_ptr[22] * y[1 + 6 * image_width] + Q_mat_ptr[23] * y[1 + 7 * image_width]) * Q_mat_ptr[9] + (Q_mat_ptr[16] * y[2] + Q_mat_ptr[17] * y[2 + image_width] + Q_mat_ptr[18] * y[2 + 2 * image_width] + Q_mat_ptr[19] * y[2 + 3 * image_width] + Q_mat_ptr[20] * y[2 + 4 * image_width] + Q_mat_ptr[21] * y[2 + 5 * image_width] + Q_mat_ptr[22] * y[2 + 6 * image_width] + Q_mat_ptr[23] * y[2 + 7 * image_width]) * Q_mat_ptr[10] + (Q_mat_ptr[16] * y[3] + Q_mat_ptr[17] * y[3 + image_width] + Q_mat_ptr[18] * y[3 + 2 * image_width] + Q_mat_ptr[19] * y[3 + 3 * image_width] + Q_mat_ptr[20] * y[3 + 4 * image_width] + Q_mat_ptr[21] * y[3 + 5 * image_width] + Q_mat_ptr[22] * y[3 + 6 * image_width] + Q_mat_ptr[23] * y[3 + 7 * image_width]) * Q_mat_ptr[11] + (Q_mat_ptr[16] * y[4] + Q_mat_ptr[17] * y[4 + image_width] + Q_mat_ptr[18] * y[4 + 2 * image_width] + Q_mat_ptr[19] * y[4 + 3 * image_width] + Q_mat_ptr[20] * y[4 + 4 * image_width] + Q_mat_ptr[21] * y[4 + 5 * image_width] + Q_mat_ptr[22] * y[4 + 6 * image_width] + Q_mat_ptr[23] * y[4 + 7 * image_width]) * Q_mat_ptr[12] + (Q_mat_ptr[16] * y[5] + Q_mat_ptr[17] * y[5 + image_width] + Q_mat_ptr[18] * y[5 + 2 * image_width] + Q_mat_ptr[19] * y[5 + 3 * image_width] + Q_mat_ptr[20] * y[5 + 4 * image_width] + Q_mat_ptr[21] * y[5 + 5 * image_width] + Q_mat_ptr[22] * y[5 + 6 * image_width] + Q_mat_ptr[23] * y[5 + 7 * image_width]) * Q_mat_ptr[13] + (Q_mat_ptr[16] * y[6] + Q_mat_ptr[17] * y[6 + image_width] + Q_mat_ptr[18] * y[6 + 2 * image_width] + Q_mat_ptr[19] * y[6 + 3 * image_width] + Q_mat_ptr[20] * y[6 + 4 * image_width] + Q_mat_ptr[21] * y[6 + 5 * image_width] + Q_mat_ptr[22] * y[6 + 6 * image_width] + Q_mat_ptr[23] * y[6 + 7 * image_width]) * Q_mat_ptr[14] + (Q_mat_ptr[16] * y[7] + Q_mat_ptr[17] * y[7 + image_width] + Q_mat_ptr[18] * y[7 + 2 * image_width] + Q_mat_ptr[19] * y[7 + 3 * image_width] + Q_mat_ptr[20] * y[7 + 4 * image_width] + Q_mat_ptr[21] * y[7 + 5 * image_width] + Q_mat_ptr[22] * y[7 + 6 * image_width] + Q_mat_ptr[23] * y[7 + 7 * image_width]) * Q_mat_ptr[15];
//					lut_element_ptr[10] = (Q_mat_ptr[16] * y[0] + Q_mat_ptr[17] * y[image_width] + Q_mat_ptr[18] * y[2 * image_width] + Q_mat_ptr[19] * y[3 * image_width] + Q_mat_ptr[20] * y[4 * image_width] + Q_mat_ptr[21] * y[5 * image_width] + Q_mat_ptr[22] * y[6 * image_width] + Q_mat_ptr[23] * y[7 * image_width]) * Q_mat_ptr[16] + (Q_mat_ptr[16] * y[1] + Q_mat_ptr[17] * y[1 + image_width] + Q_mat_ptr[18] * y[1 + 2 * image_width] + Q_mat_ptr[19] * y[1 + 3 * image_width] + Q_mat_ptr[20] * y[1 + 4 * image_width] + Q_mat_ptr[21] * y[1 + 5 * image_width] + Q_mat_ptr[22] * y[1 + 6 * image_width] + Q_mat_ptr[23] * y[1 + 7 * image_width]) * Q_mat_ptr[17] + (Q_mat_ptr[16] * y[2] + Q_mat_ptr[17] * y[2 + image_width] + Q_mat_ptr[18] * y[2 + 2 * image_width] + Q_mat_ptr[19] * y[2 + 3 * image_width] + Q_mat_ptr[20] * y[2 + 4 * image_width] + Q_mat_ptr[21] * y[2 + 5 * image_width] + Q_mat_ptr[22] * y[2 + 6 * image_width] + Q_mat_ptr[23] * y[2 + 7 * image_width]) * Q_mat_ptr[18] + (Q_mat_ptr[16] * y[3] + Q_mat_ptr[17] * y[3 + image_width] + Q_mat_ptr[18] * y[3 + 2 * image_width] + Q_mat_ptr[19] * y[3 + 3 * image_width] + Q_mat_ptr[20] * y[3 + 4 * image_width] + Q_mat_ptr[21] * y[3 + 5 * image_width] + Q_mat_ptr[22] * y[3 + 6 * image_width] + Q_mat_ptr[23] * y[3 + 7 * image_width]) * Q_mat_ptr[19] + (Q_mat_ptr[16] * y[4] + Q_mat_ptr[17] * y[4 + image_width] + Q_mat_ptr[18] * y[4 + 2 * image_width] + Q_mat_ptr[19] * y[4 + 3 * image_width] + Q_mat_ptr[20] * y[4 + 4 * image_width] + Q_mat_ptr[21] * y[4 + 5 * image_width] + Q_mat_ptr[22] * y[4 + 6 * image_width] + Q_mat_ptr[23] * y[4 + 7 * image_width]) * Q_mat_ptr[20] + (Q_mat_ptr[16] * y[5] + Q_mat_ptr[17] * y[5 + image_width] + Q_mat_ptr[18] * y[5 + 2 * image_width] + Q_mat_ptr[19] * y[5 + 3 * image_width] + Q_mat_ptr[20] * y[5 + 4 * image_width] + Q_mat_ptr[21] * y[5 + 5 * image_width] + Q_mat_ptr[22] * y[5 + 6 * image_width] + Q_mat_ptr[23] * y[5 + 7 * image_width]) * Q_mat_ptr[21] + (Q_mat_ptr[16] * y[6] + Q_mat_ptr[17] * y[6 + image_width] + Q_mat_ptr[18] * y[6 + 2 * image_width] + Q_mat_ptr[19] * y[6 + 3 * image_width] + Q_mat_ptr[20] * y[6 + 4 * image_width] + Q_mat_ptr[21] * y[6 + 5 * image_width] + Q_mat_ptr[22] * y[6 + 6 * image_width] + Q_mat_ptr[23] * y[6 + 7 * image_width]) * Q_mat_ptr[22] + (Q_mat_ptr[16] * y[7] + Q_mat_ptr[17] * y[7 + image_width] + Q_mat_ptr[18] * y[7 + 2 * image_width] + Q_mat_ptr[19] * y[7 + 3 * image_width] + Q_mat_ptr[20] * y[7 + 4 * image_width] + Q_mat_ptr[21] * y[7 + 5 * image_width] + Q_mat_ptr[22] * y[7 + 6 * image_width] + Q_mat_ptr[23] * y[7 + 7 * image_width]) * Q_mat_ptr[23];
//					lut_element_ptr[11] = Q_mat_ptr[16] * y[3] + Q_mat_ptr[17] * y[3 + image_width] + Q_mat_ptr[18] * y[3 + 2 * image_width] + Q_mat_ptr[19] * y[3 + 3 * image_width] + Q_mat_ptr[20] * y[3 + 4 * image_width] + Q_mat_ptr[21] * y[3 + 5 * image_width] + Q_mat_ptr[22] * y[3 + 6 * image_width] + Q_mat_ptr[23] * y[3 + 7 * image_width];
//					lut_element_ptr[12] = y[3 * image_width] * Q_mat_ptr[0] + y[1 + 3 * image_width] * Q_mat_ptr[1] + y[2 + 3 * image_width] * Q_mat_ptr[2] + Q_mat_ptr[3] * y[3 + 3 * image_width] + y[4 + 3 * image_width] * Q_mat_ptr[4] + y[5 + 3 * image_width] * Q_mat_ptr[5] + y[6 + 3 * image_width] * Q_mat_ptr[6] + y[7 + 3 * image_width] * Q_mat_ptr[7];
//					lut_element_ptr[13] = y[3 * image_width] * Q_mat_ptr[8] + y[1 + 3 * image_width] * Q_mat_ptr[9] + y[2 + 3 * image_width] * Q_mat_ptr[10] + Q_mat_ptr[11] * y[3 + 3 * image_width] + y[4 + 3 * image_width] * Q_mat_ptr[12] + y[5 + 3 * image_width] * Q_mat_ptr[13] + y[6 + 3 * image_width] * Q_mat_ptr[14] + y[7 + 3 * image_width] * Q_mat_ptr[15];
//					lut_element_ptr[14] = y[3 * image_width] * Q_mat_ptr[16] + y[1 + 3 * image_width] * Q_mat_ptr[17] + y[2 + 3 * image_width] * Q_mat_ptr[18] + Q_mat_ptr[19] * y[3 + 3 * image_width] + y[4 + 3 * image_width] * Q_mat_ptr[20] + y[5 + 3 * image_width] * Q_mat_ptr[21] + y[6 + 3 * image_width] * Q_mat_ptr[22] + y[7 + 3 * image_width] * Q_mat_ptr[23];
//					lut_element_ptr[15] = y[3 + 3 * image_width];
//				}
//
//				lut->insert(key,element);
//				S_mat_ptr = element->data.db;
//			} 
//			else
//			{
//				S_mat_ptr = lut->value(key)->data.db;
//			}
//			
//			CV_MAT_ELEM(*g_mat,double,i,j) = 
//				delta_x*delta_x*delta_x * (delta_y*delta_y*delta_y * S_mat_ptr[0] + delta_y*delta_y * S_mat_ptr[4] + delta_y * S_mat_ptr[8] + S_mat_ptr[12] ) +
//				delta_x*delta_x * (delta_y*delta_y*delta_y * S_mat_ptr[1] + delta_y*delta_y * S_mat_ptr[5] + delta_y * S_mat_ptr[9] + S_mat_ptr[13] ) +
//				delta_x  * (delta_y*delta_y*delta_y * S_mat_ptr[2] + delta_y*delta_y * S_mat_ptr[6] + delta_y * S_mat_ptr[10] + S_mat_ptr[14] ) +
//				(delta_y*delta_y*delta_y * S_mat_ptr[3] + delta_y*delta_y * S_mat_ptr[7] + delta_y * S_mat_ptr[11] + S_mat_ptr[15] );
//		}
//	}
//	return true;
//}
