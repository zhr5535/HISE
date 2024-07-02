#include <QObject>
#include <QtGui>

#include "helpmethod.h"
#include "internal/bicubic_spline_interpolation.h"

#ifndef DEFLOW_SPARSE_TRACING
#include "planarstrainmethod.h"
#endif // !DEFLOW_SPARSE_TRACING

bool HelpMethod::poly2mat(const QList<QList<QPointF>>& poly,
	CvMat* mast_sign_mat,
	int* bound_rect_left_top_x, int* bound_rect_left_top_y,
	int* bound_rect_width, int* bound_rect_height)
{
	//
	if (CV_MAT_TYPE(mast_sign_mat->type) != CV_8U ){
		return false;
	}
	int image_height = mast_sign_mat->rows;
	int image_width = mast_sign_mat->cols;
	if (poly.isEmpty()){
		return false;
	}

	//int list_count = poly.length();//传过来的点列表有问题， 暂时如此修改
	QList<QPolygonF> poly_normal_list;
	QList<QPolygonF> poly_substract_list;
	for (int i =0;i< poly.length();i++)
	{
		decltype(auto) list_one = poly[i];
		int point_count = list_one.length();
		//提取正常的多边形
		int first_poly_substract_head_index =-1;
		QPolygonF poly_normal;
		for (int j =0;j<point_count;j++)
		{
			double pos_x = list_one[j].x();
			double pos_y = list_one[j].y();
			if (pos_x <0 || pos_y <0)
			{
				first_poly_substract_head_index = j;
				break;
			}
			else
			{
				if (pos_x >= image_width )
				{
					pos_x = image_width -1;
				}
				if (pos_y >= image_height )
				{
					pos_y = image_height -1;
				}
				poly_normal<<list_one[j];
			}
		}
		if (poly_normal.isEmpty()) {
			continue;
		}
		poly_normal_list<<poly_normal;

		//提取反向的多边形
		if (first_poly_substract_head_index>0)
		{
			int current_poly_substract_head_index = first_poly_substract_head_index;// -1.0 -1.0
			while(current_poly_substract_head_index < point_count)
			{
				QPolygonF poly_substract;
				int next_poly_head_index = -1;
				for (int j = current_poly_substract_head_index+1;j<point_count;j++)
				{
					double pos_x = list_one[j].x();
					double pos_y = list_one[j].y();
					if (pos_x <0 || pos_y<0  )
					{
						next_poly_head_index = j;
						break;
					}
					else
					{
						if (pos_x >= image_width )
						{
							pos_x = image_width -1;
						}
						if (pos_y >= image_height )
						{
							pos_y = image_height -1;
						}
						poly_substract<<list_one[j];
					}
				}

				if (next_poly_head_index >0)//后面还有
				{
					if (!poly_substract.isEmpty())
					{
						poly_substract_list<<poly_substract;
					}					
					current_poly_substract_head_index = next_poly_head_index;
				}
				else//最后一个
				{
					if (!poly_substract.isEmpty())
					{
						poly_substract_list<<poly_substract;
					}
					current_poly_substract_head_index = point_count;
				}				
			}
		}		
	}

	QImage qimage(image_width,image_height,QImage::Format_ARGB32);
	qimage.fill(QColor(0,0,0,255));
	QPainter painter;
	painter.begin(&qimage);
	
	//对于正多边形
	for (int i =0;i<poly_normal_list.length();i++)
	{
		QPainterPath normal_path;
		normal_path.addPolygon(poly_normal_list[i]);
		painter.fillPath(normal_path,QBrush(QColor(255,0,0,255)));
	}
	//对于反多边形
	for (int i =0;i<poly_substract_list.length();i++)
	{
		QPainterPath substract_path;
		substract_path.addPolygon(poly_substract_list[i]);
		painter.fillPath(substract_path,QBrush(QColor(0,0,0,255)));
	}
	painter.end();

	int left_top_x=image_width+1;
	int left_top_y=image_height+1;
	int right_botton_x = -1;
	int right_bottom_y = -1;
	for (int i =0;i<image_height;i++)
	{
		uchar* line =qimage.scanLine(i);
		for (int j =0;j<image_width;j++)
		{
			uchar a = line[4*j+2];
			if (a != 0)
			{
				CV_MAT_ELEM(*mast_sign_mat,uchar,i,j) = 1;
				if (j<left_top_x)
				{
					left_top_x =j;
				}
				if (i<left_top_y)
				{
					left_top_y =i;
				}
				if (j>right_botton_x)
				{
					right_botton_x =j;
				}
				if (i>right_bottom_y)
				{
					right_bottom_y = i;
				}
			}
			else
			{
				CV_MAT_ELEM(*mast_sign_mat,uchar,i,j) = 0;
			}			
		}
	}
	*bound_rect_left_top_x = left_top_x;
	*bound_rect_left_top_y = left_top_y;
	*bound_rect_width = right_botton_x - left_top_x +1;
	*bound_rect_height = right_bottom_y - left_top_y +1;

	return true;
}


CvMat* HelpMethod::BuildCalculateMaskSignMat( int grid_step, CvMat* mask_sign_mat_in_image, int bound_rect_left_top_x, int bound_rect_left_top_y, int bound_rect_width, int bound_rect_height )
{
	int mat_col_count_= cvCeil((double)(bound_rect_width)/(double)(grid_step));
	int mat_row_count_ = cvCeil((double)(bound_rect_height)/(double)(grid_step));
	if (mat_col_count_<=0 || mat_row_count_<=0)
	{
		return NULL;
	}
	CvMat* mask_sign_mat = cvCreateMat(mat_row_count_,mat_col_count_,CV_8U);
	cvZero(mask_sign_mat);

	for (int i =0;i<mat_row_count_;i++)
	{
		for (int j =0;j<mat_col_count_;j++)
		{
			uchar sign_in_image = CV_MAT_ELEM(*mask_sign_mat_in_image,uchar,i*grid_step + bound_rect_left_top_y,j*grid_step +bound_rect_left_top_x);
			if (sign_in_image==1)
			{
				CV_MAT_ELEM(*mask_sign_mat,uchar,i,j) = 1;
			}
		}
	}

	return mask_sign_mat;

}


bool HelpMethod::DeleteBoundGridPoint( CvMat* mask_sign_mat, CvMat* mask_sign_in_image, int patch_half_size, int grid_step, int mask_left_top_point_x, int mask_left_top_point_y)
{
	int mat_col_count = mask_sign_mat->cols;
	int mat_row_count = mask_sign_mat->rows;
	int image_height = mask_sign_in_image->rows;
	int image_width = mask_sign_in_image->cols;

	//修正边缘
	for (int i =0;i<mat_row_count;i++)
	{
		for (int j =0;j<mat_col_count;j++)
		{
			uchar sign_calc = CV_MAT_ELEM(*mask_sign_mat,uchar,i,j);
			if (sign_calc ==1)
			{
				int index_i_in_image = i*grid_step+mask_left_top_point_y;
				int index_j_in_image = j*grid_step+mask_left_top_point_x;

				bool is_valid = true;
				for (int m = -patch_half_size;m<=patch_half_size;m++)
				{
					for (int n = -patch_half_size;n<=patch_half_size;n++)
					{
						if (index_i_in_image < (- m + UBSplineInterp6x6_t::start_indent)||
							index_i_in_image >= (image_height -1 - m - UBSplineInterp6x6_t::end_indent)||
							index_j_in_image < (- n + UBSplineInterp6x6_t::start_indent) ||
							index_j_in_image >= (image_width - 1 - n - UBSplineInterp6x6_t::end_indent))
						{
							is_valid =false;
							break;
						}
						uchar sign_valid_region = CV_MAT_ELEM(*mask_sign_in_image,uchar,index_i_in_image+m,index_j_in_image+n);
						if (sign_valid_region !=1)
						{
							is_valid =false;
							break;
						}
					}
				}
				if (!is_valid)
				{
					CV_MAT_ELEM(*mask_sign_mat,uchar,i,j) =0;
				}
			}
		}
	}
	return true;
}

bool HelpMethod::GetValidCalculateMaskSize(int grid_step,int bound_rect_width, int bound_rect_height,int* mat_col_count,int* mat_row_count )
{
	int mat_cols= cvCeil((double)(bound_rect_width)/(double)(grid_step));
	int mat_rows = cvCeil((double)(bound_rect_height)/(double)(grid_step));
	if (mat_cols<=0 || mat_rows<=0)
	{
		return false;
	}
	*mat_col_count = mat_cols;
	*mat_row_count = mat_rows;
	return true;
}

bool HelpMethod::HandleEdge(
							CvMat* mask_sign_in_image, 
							CvMat* mask_sign_mat, 
							CvMat* edge_weight_mat,							
							int patch_half_size, 
							int grid_step, 
							int mask_left_top_point_x, 
							int mask_left_top_point_y,
							double validity_limit_in_persent
							)
{
	int mat_col_count = mask_sign_mat->cols;
	int mat_row_count = mask_sign_mat->rows;
	int image_height = mask_sign_in_image->rows;
	int image_width = mask_sign_in_image->cols;
	int patch_size = patch_half_size*2+1;
	int template_point_count = patch_size*patch_size;

	//修正边缘
	for (int i =0;i<mat_row_count;i++)
	{
		for (int j =0;j<mat_col_count;j++)
		{
			int index_i_in_image = i*grid_step+mask_left_top_point_y;
			int index_j_in_image = j*grid_step+mask_left_top_point_x;
			uchar sign_in_image = CV_MAT_ELEM(*mask_sign_in_image,uchar,index_i_in_image,index_j_in_image);

			int one_demesion_index = i*mat_col_count+j;
			CvMat weight_mat = cvMat(patch_size,patch_size,CV_8U,edge_weight_mat->data.ptr +one_demesion_index*template_point_count);
			if (sign_in_image == 1)
			{
				//尚未判断出是否为边缘点
				CvMat* sub_valid = cvCreateMat(patch_size,patch_size,CV_8U);
				cvZero(sub_valid);
				for (int m = -patch_half_size;m<=patch_half_size;m++)
				{
					for (int n = -patch_half_size;n<=patch_half_size;n++)
					{
						int nei_i_in_image = index_i_in_image+m;
						int nei_j_in_image = index_j_in_image+n;

						//判断是否在图像及插值边界
						//bool is_image_and_inter_edge = false;
						if (nei_i_in_image < (- m + UBSplineInterp6x6_t::start_indent)||
							nei_i_in_image >= (image_height -1 - m - UBSplineInterp6x6_t::end_indent)||
							nei_j_in_image < (- n + UBSplineInterp6x6_t::start_indent) ||
							nei_j_in_image >= (image_width - 1 - n - UBSplineInterp6x6_t::end_indent))//图像及插值边界
						{
							continue;				
						}

						//判断模板内点是否在aoi内
						uchar nei_sign = CV_MAT_ELEM(*mask_sign_in_image,uchar,nei_i_in_image,nei_j_in_image);
						if (nei_sign == 1)
						{
							CV_MAT_ELEM(*sub_valid,uchar,m+patch_half_size,n+patch_half_size) = 1;
						}
					}
				}

				int count_no_zero = cvCountNonZero(sub_valid); //记录该模板内，在AOI之内的点的个数
				bool is_template_full = true;//指示该模板内的所有点是不是都在aoi之内
				if (count_no_zero != template_point_count)
				{
					is_template_full = false;
				}

				//检查
				if (is_template_full)//内部计算点
				{
					//非边界点
					CV_MAT_ELEM(*mask_sign_mat,uchar,i,j) = 1;
					cvZero(sub_valid);//此句可有可无，
				}
				else//边缘计算点
				{
#ifndef DEFLOW_SPARSE_TRACING
					/* 考虑连通性 */
					CvMat* connection = cvCreateMat(patch_size,patch_size,CV_8U);
					cvZero(connection);
					PlanarStrainMethod::ConnectionFilter(patch_half_size,sub_valid,connection);

					for (int r = 0; r < patch_size; r++)
					{
						for (int c = 0; c < patch_size; c++)
						{
							uchar sign_sub = CV_MAT_ELEM(*sub_valid,uchar,r,c) ;
							uchar sign_connection = CV_MAT_ELEM(*connection,uchar,r,c) ;
							if (sign_sub !=1  || sign_connection!=1)
							{
								CV_MAT_ELEM(*sub_valid,uchar,r,c)= 0;
							}
						}
					}				
					cvReleaseMat(&connection);


					/* 和中点连线要全部在区域中 */
					CvMat* convex = cvCloneMat(sub_valid);
					for (int r =0;r<patch_size;r++)
					{
						for (int c =0;c<patch_size;c++)
						{
							uchar sign_convex = CV_MAT_ELEM(*convex,uchar,r,c) ;
							if (sign_convex ==1)
							{
								if (!PlanarStrainMethod::ConnectLineConvex(convex,r,c))
								{
									CV_MAT_ELEM(*sub_valid,uchar,r,c) = 0;
								}
							}
						}
					}
					cvReleaseMat(&convex);

					count_no_zero = cvCountNonZero(sub_valid);
					double validity = (double)count_no_zero/(double)template_point_count;
					if (validity>=validity_limit_in_persent)
					{
						CV_MAT_ELEM(*mask_sign_mat,uchar,i,j) = 2;
					} 
					else
					{
						CV_MAT_ELEM(*mask_sign_mat,uchar,i,j) = 0;
					}
#endif
				}
				cvCopy(sub_valid,&weight_mat);
				cvReleaseMat(&sub_valid);
			}
		}
	}
	return true;
}

cv::Mat deflow::helper::_Get_mat(const CvMat* src) noexcept
{
	return cv::Mat(src->rows, src->cols, src->type, src->data.ptr);
}
