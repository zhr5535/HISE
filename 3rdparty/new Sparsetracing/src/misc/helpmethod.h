#ifndef HELPMETHOD_H
#define HELPMETHOD_H

#include <opencv2/core.hpp>

struct CvMat;

class HelpMethod
{
public:
	// Convert polygon to CvMat
	static bool poly2mat(
		const QList<QList<QPointF>>& poly,
		CvMat* mast_sign_mat,
		int* bound_rect_left_top_x,
		int* bound_rect_left_top_y,
		int* bound_rect_width,
		int* bound_rect_height
		);

	static CvMat* BuildCalculateMaskSignMat(
		int grid_step,
		CvMat* mask_sign_mat_in_image,		
		int bound_rect_left_top_x,
		int bound_rect_left_top_y,
		int bound_rect_width,
		int bound_rect_height
		);

	static bool DeleteBoundGridPoint(
		CvMat* mask_sign_mat,
		CvMat* mask_sign_in_image,
		int patch_half_size,
		int grid_step,
		int mask_left_top_point_x,
		int mask_left_top_point_y
		);

	static bool GetValidCalculateMaskSize(
		int grid_step,
		int bound_rect_width, 
		int bound_rect_height,
		int* mat_col_count,
		int* mat_row_count 
		);

	static bool HandleEdge( 
		CvMat* mask_sign_in_image, 
		CvMat* mask_sign_mat, 
		CvMat* edge_weight_mat,							
		int patch_half_size, 
		int grid_step, 
		int mask_left_top_point_x, 
		int mask_left_top_point_y,
		double validity_limit_in_persent
		);//边缘点为2，权重矩阵0-1 ,,//非边缘点为1，权重矩阵为1
	
private:
	
};

namespace deflow {
namespace helper {
template<typename ..._Args>
auto _Poly_to_mat(_Args... args) {
	return HelpMethod::poly2mat(args...);
}
template<typename ..._Args>
auto _Proc_roi_edge(_Args... args) {
	return HelpMethod::HandleEdge(args...);
}

cv::Mat _Get_mat(const CvMat* src) noexcept;

} // namespace helper
} // namespace deflow

#endif // HELPMETHOD_H