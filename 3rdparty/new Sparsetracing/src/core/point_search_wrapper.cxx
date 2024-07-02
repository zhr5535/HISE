#include <algorithm> // for std::find_if
#include <execution> // for std::execution::par

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "core/search_func.hpp"
#include "utils/functions.h"

namespace deflow {
/***************** Impl. of Base Class correlation_match ***********************/
correlation_match::_Template& correlation_match::_Template::create(size_t size) {
	if(data.rows!=size || data.cols!=size)
		data = detail::make_cvmat<value_t>(size, size);
	if (gx.rows != size || gx.cols != size)
		gx = detail::make_cvmat<value_t>(size, size);
	if (gy.rows != size || gy.cols != size)
		gy = detail::make_cvmat<value_t>(size, size);
	inverse_hessian = detail::make_cvmat<value_t>(6, 6);

	return (*this);
}
void correlation_match::_LUT::create(size_t rows, size_t cols) {
	x_min = y_min = internal::interpolation_t::start_indent;
	x_max = cols - internal::interpolation_t::end_indent;
	y_max = rows - internal::interpolation_t::end_indent;

	if (image.rows != rows || image.cols != cols)
		image = detail::make_cvmat<value_t>(rows, cols);

	cell_size = internal::interpolation_t::cell_size;
	const auto srows = y_max - y_min, scols = x_max - x_min;
	if(data.rows!=srows || data.cols!=scols*cell_size)
		data = detail::make_cvmat<float_t>(srows, scols* cell_size);
	if (mask.rows != srows || mask.cols != scols)
		mask = detail::make_cvmat<uint8_t>(srows, scols);
}

void correlation_match::_LUT::free()
{
	image.release();
	data.release();
	mask.release();
	x_max = y_max = 0;
	x_min = y_min = 2;
}

correlation_match::correlation_match(const options_t& pars) noexcept
	:_Myoptions(pars) 
{
	_MyQmat = detail::make_cvmat<value_t>(4, 6);
	{ //internal
		auto _Qmat = cvMat(_MyQmat);
		internal::interpolation_t::Build_Two_Dimension_Interpolation_6X6_Q_Matrix_4X6(&_Qmat);
	}
}

correlation_match::correlation_match(const _Myt& other) noexcept
{
	_Myrefimg = other._Myrefimg;
	_Myoptions = other._Myoptions;
	_Myrefwnd = other._Myrefwnd;
	_Mytarlut = other._Mytarlut;
	_MyQmat = other._MyQmat;
	_Myzncc = other._Myzncc;
	_Myparams = other._Myparams;
}

correlation_match::correlation_match(_Myt&& other) noexcept
{
	_Myrefimg = std::move(other._Myrefimg);
	_Myoptions = std::move(other._Myoptions);
	_Myrefwnd = std::move(other._Myrefwnd);
	_Mytarlut = std::move(other._Mytarlut);
	_MyQmat = std::move(other._MyQmat);
	//_Myzncc = std::move(other._Myzncc);
	//_Myparams = std::move(other._Myparams);
}

correlation_match::~correlation_match()
{
	_Mytarlut.free();
}

correlation_match& correlation_match::init(const matrix_t& ref, uint32_t num_tasks) noexcept
{
	_Myrefimg = ref;
	if (!_Myoptions->enable_brute_threadify) {
		num_tasks = detail::get_num_tasks(num_tasks);
	}
	if (num_tasks != _Myrefwnd.size()) {
		_Myrefwnd.resize(num_tasks);
	}
	for (auto& wnd : _Myrefwnd) {
		wnd.create(_Myoptions->patch_size);
	}
	_Mytarlut.create(_Myoptions->imrows, _Myoptions->imcols);
	return (*this);
}
correlation_match& correlation_match::init(matrix_t&& ref, uint32_t num_tasks) noexcept
{
	return init(ref, num_tasks);
}

void correlation_match::update_ref_transforms(const matrix_t& pars) noexcept
{
	if (pars.empty() || _Myrefwnd.empty() || pars.rows != _Myrefwnd.size()) {
		return;
	}
	for (auto i = 0; i < _Myrefwnd.size(); ++i) {
		_Myrefwnd[i].center.x = pars.ptr<value_t>(i)[0];
		_Myrefwnd[i].center.y = pars.ptr<value_t>(i)[3];
		_Myrefwnd[i].transform = pars.row(i) * 1;
		_Myrefwnd[i].transform.ptr<value_t>()[0] = 0;
		_Myrefwnd[i].transform.ptr<value_t>()[3] = 0;
	}
}

const correlation_match::_Myt& correlation_match::_Get() const noexcept
{
	return (*this);
}

correlation_match::_Myt& correlation_match::_Get() noexcept
{
	return (*this);
}

correlation_match& correlation_match::_Precomp(size_t idx, point_t point, matrix_t tmat)
{
	auto& _Ref = _Myrefwnd[idx];
	_Ref.create(_Myoptions->patch_size);
	_Ref.transform = tmat.clone();
	_Ref.center = point;

	auto _Refmat = cvMat(_Myrefimg);
	auto _Qmat = cvMat(_MyQmat);
	auto _Gxmat = cvMat(_Ref.gx), _Gymat = cvMat(_Ref.gy);
	auto& _Data = _Ref.data;
	if (tmat.empty()) { // Use the first of the image chunk as reference
		const auto _Org = point.to<int32_t>() - _Myoptions->patch_radius();
		_Data = _Myrefimg({ _Org.x, _Org.y, _Myoptions->patch_size, _Myoptions->patch_size });
		internal::interpolation_t::Integer_Points_Gradient_One_Template(
			&_Refmat, &_Qmat, int32_t(point.x), int32_t(point.y),
			&_Gxmat, &_Gymat
		);
	}
	else { // Use updated reference image, and the silding window is deformed
		auto _T = tmat.ptr<value_t>();
		auto _Tmp_lut = cvCreateMat(_Mytarlut.rows(), _Mytarlut.width(), CV_32F);
		auto _Tmp_mask = cvCreateMat(_Mytarlut.rows(), _Mytarlut.cols(), CV_8U);
		auto _Wndmat = cvMat(_Data);
		internal::interpolation_t::Value_And_Gradient_One_Affine_Template_64F(
			&_Refmat, _T[0]+point.x, _T[3]+point.y, _T[1], _T[2], _T[4], _T[5],
			_Mytarlut.x_min, _Mytarlut.x_max, _Mytarlut.y_min, _Mytarlut.y_max,
			_Tmp_lut, _Tmp_mask, &_Qmat, &_Wndmat, &_Gxmat, &_Gymat
		);
		//_Ref.center.x += _T[0], _Ref.center.y += _T[1];
	}
	_Ref.mean = (_Ref.mask.empty() ? cv::mean(_Data) : cv::mean(_Data, _Ref.mask))[0];
	{ //internal
		auto _Wndmat = cvMat(_Data);
		internal::correlation_t::CalculateSecondOrderCenterMoment(
			&_Wndmat, &_Ref.mean, &_Ref.ssdv
		);
		auto _Hessmat = cvMat(_Ref.inverse_hessian);
		internal::correlation_t::CalculateNewtonRaphsonInverseHessian(
			&_Gxmat, &_Gymat, &_Hessmat
		);
	}
	return (*this);
}

/*********************** Impl. of Derived Class stereo_match_wrapper ******************************/
stereo_match_wrapper& stereo_match_wrapper::set_condition(const matrix_t& f, value_t radius)
{
	_MyF = f;
	return (*this);
}

stereo_match_wrapper& stereo_match_wrapper::constraint(value_t radius)
{
	_MyF = get_fundamental_matrix(options());
	return (*this);
}

std::array<float, 4> stereo_match_wrapper::epline(float x, float y) const noexcept
{
	const auto l = _Get_epline({ value_t(x), value_t(y) });
	const auto x0 = float(0);
	const auto y0 = float(-l.z / l.y);
	const auto x1 = float(_Myoptions->imcols);
	const auto y1 = float(-(l.x*x1+l.z) / l.y);
	return { x0,y0,x1,y1 };
}

stereo_match_wrapper::matrix_t&
stereo_match_wrapper::run(const matrix_t& tar, const point_array_t& points, matrix_t tmats)
{
	if (detail::have_same_value_type(tar, _Mytarlut.image)) {
		_Mytarlut.image = tar;
	}
	else {
		tar.convertTo(_Mytarlut.image, _Mytarlut.image.type());
	}
	if (_Myoptions->corr_keep_coefs) {
		_Myzncc = detail::make_cvmat<float_t>(points.size(), 1);
	}

	auto _Pars = detail::make_cvmat<value_t>(points.size(), 6);
	const auto _Size = tmats.empty() ? points.size() : std::min<size_t>(points.size(), tmats.rows);
#ifdef USE_TBB
	const auto _Num_groups = _Size / _Myrefwnd.size();
	// Process first _Num_groups*_Myrefwnd.size() points...
	for (auto _Gidx = 0; _Gidx < _Num_groups; ++_Gidx) try {
		tbb::parallel_for(size_t{ 0 }, _Myrefwnd.size(), [&](auto _Widx) {
			const auto _Idx = _Gidx * _Myrefwnd.size() + _Widx;
			auto _Tmat = tmats.empty() ? matrix_t() : tmats.row(_Idx);
			_Precomp(_Widx, points[_Idx], _Tmat);
			});
		tbb::parallel_for(size_t{ 0 }, _Myrefwnd.size(), [&](auto _Widx) {
			const auto _Idx = _Gidx * _Myrefwnd.size() + _Widx;
			_Search({ _Widx, _Idx }, points[_Idx], _Pars);
			});
	}
	catch (const std::exception& e) {
		const auto _Msg = e.what();
	}

	// Process the remaind points...
	if (const auto _Res_num = _Size - _Num_groups * _Myrefwnd.size(); _Res_num) try {
		tbb::parallel_for(size_t{ 0 }, _Res_num, [&](auto _Widx) {
			const auto _Idx = _Num_groups * _Myrefwnd.size() + _Widx;
			auto _Tmat = tmats.empty() ? matrix_t() : tmats.row(_Idx);
			_Precomp(_Widx, points[_Idx], _Tmat);
			});
		tbb::parallel_for(size_t{ 0 }, _Res_num, [&](auto _Widx) {
			const auto _Idx = _Num_groups * _Myrefwnd.size() + _Widx;
			_Search({ _Widx, _Idx }, points[_Idx], _Pars);
			});
	}
	catch (const DEFLOW_USE_STD(exception)& e) {
		const auto _Msg = e.what();
	}
#elif
	for (auto _Idx = 0; _Idx < _Size; ++_Idx) {
		auto _Tmat = tmats.empty() ? matrix_t() : tmats.row(_Idx);
		_Precomp(_Idx, points[_Idx], _Tmat);
		_Search(_Idx, points[_Idx], _Pars);
	}
#endif
	_Myparams = DEFLOW_USE_STD(move)(_Pars);
	return _Myparams;
}

stereo_match_wrapper::matrix_t
stereo_match_wrapper::run(const std::string& path, matrix_t transform)
{
	auto right_img = imread(path);

	for (auto idx = 0; idx < _Myrefwnd.size(); ++idx) {
		auto& ref_poi = _Myrefwnd[idx];
		auto ref_wnd_mat = cvMat(ref_poi.data);
		auto right_wnd_transform = transform.row(idx).clone();

		/*internal::correlation_1374_t::Neighbour_Integer_Search(
			&ref_wnd_mat, &ref_poi.mean, &ref_poi.ssdv,

		);*/
	}
	return matrix_t();
}

stereo_match_wrapper::unique_neighbor_matcher_tuple stereo_match_wrapper::detach(bool share_reference)
{
	// Initialize...
	auto _Left = std::make_unique<sparse_neighbor_match_wrapper>(_Myoptions);
	auto _Right = std::make_unique<sparse_neighbor_match_wrapper>(_Myoptions);
	_Left->init(_Myrefimg.clone(), _Myrefwnd.size());
	if (share_reference) {
		_Right->init(_Myrefimg.clone(), _Myrefwnd.size());
		_Right->share_ref(true);
	}
	else {
		_Right->init(_Mytarlut.image.clone(), _Myrefwnd.size());
	}

	// Declare left branch
	auto _Points = point_array_t(_Myrefwnd.size());
	auto _Left_tmat = matrix_t();
	for (auto _It = _Points.begin(); _It < _Points.end(); ++_It) {
		const auto _Off = std::distance(_Points.begin(), _It);
		*_It = _Myrefwnd[_Off].center;
	}
	_Left->declare(_Points, _Left_tmat);

	// Declare right branch
	if (share_reference) {
		_Right->declare(_Points, _Left_tmat);
	}
	else {
		auto _Right_tmat = matrix_t();
		for (auto _It = _Points.begin(); _It < _Points.end(); ++_It) {
			const auto _Off = std::distance(_Points.begin(), _It);
			_It->x += _Myparams.at<value_t>(_Off, 0);
			_It->y += _Myparams.at<value_t>(_Off, 3);
			matrix_t transform = /*keep_transform?_Myparams.row(_Off):*/make_cvmat(1, _Myparams.cols, 0.);
			_Right_tmat.push_back(transform);
		}
		_Right->declare(_Points, _Right_tmat);
	}

	return std::make_tuple(std::move(_Left), std::move(_Right));
}

stereo_match_wrapper::matrix_t stereo_match_wrapper::_Search(index_n_t<2> idx, point_t point, matrix_t& pars)
{
	using point2i_t = internal::Vec_<uint32_t, 2>;

	const auto _Radius = _Myoptions->patch_radius();
	const auto _Start_y = _Radius;
	const auto _End_y = _Myrefimg.rows - _Radius;
	const auto _Start_x = _Radius;
	const auto _End_x = _Myrefimg.cols - _Radius;
	const auto _L = _Get_epline(point);
	auto _Cands = DEFLOW_USE_STD(vector<point2i_t>)();
	for (auto y = _Start_y; y < _End_y; ++y) {
		for (auto x = _Start_x; x < _End_x; ++x) {
			const auto _Distance = _L * point3_t(x, y, 1);
			if (std::abs(_Distance) < _Myoptions->epline_width) {
				_Cands.push_back({ uint32_t(x), uint32_t(y) });
			}
		} // loop over columns
	} // loop over rows

	auto& _Ref = _Myrefwnd[idx[0]];
	auto[_Idx, _C0] = internal::_Pixel_search<point2i_t>(_Ref, _Mytarlut.image, _Cands);
	if (_Idx < _Cands.size()) {
		auto& _T = _Ref.transform;
		if (_T.empty()) {
			_T = detail::make_cvmat<value_t>(6, 1);
		}
		_T.ptr<value_t>()[0] = _Cands[_Idx].x - point.x;
		_T.ptr<value_t>()[3] = _Cands[_Idx].y - point.y;
	}
	else {
		return {};
	}
	auto[_It, _C1, _Pars] = internal::_Subpixel_refine(_Ref, _Mytarlut, _MyQmat, _Myoptions->brief_corr());
	for (auto i = 0; i < _Pars.rows; ++i) { // Store the estimated parameters
		pars.ptr<value_t>(idx[1])[i] = _Pars.ptr<value_t>()[i];
	}
	if (_Myoptions->corr_keep_coefs) {
		_Myzncc.ptr<float_t>()[idx[1]] = _C1;
	}

	return _Pars;
}

stereo_match_wrapper::point3_t stereo_match_wrapper::_Get_epline(point_t p) const noexcept
{
	point3_t _L;
	{
		const auto prow0 = _MyF.ptr<value_t>(0);
		const auto prow1 = _MyF.ptr<value_t>(1);
		const auto prow2 = _MyF.ptr<value_t>(2);
		_L.x = prow0[0] * p.x + prow0[1] * p.y + prow0[2];
		_L.y = prow1[0] * p.x + prow1[1] * p.y + prow1[2];
		_L.z = prow2[0] * p.x + prow2[1] * p.y + prow2[2];
	}
	return _L / DEFLOW_USE_STD(sqrt)(_L.x * _L.x + _L.y * _L.y);
}

sparse_neighbor_match_wrapper& sparse_neighbor_match_wrapper::set_radius(const int32_t* value) noexcept
{
	_Myradius = value;
	return (*this);
}

bool sparse_neighbor_match_wrapper::declare(const point_array_t& points, matrix_t tmats)
{
	const auto _Size = tmats.empty() ? points.size() : std::min<size_t>(points.size(), tmats.rows);
	if (_Myrefwnd.size() < _Size) {
		_Mybase::init(_Myrefimg, _Size);
	}
	auto _Is_success = true;
#ifdef USE_TBB
	try {
		tbb::parallel_for(size_t{ 0 }, _Myrefwnd.size(), [&](auto _Idx) {
			auto _Tmat = tmats.empty() ? matrix_t() : tmats.row(_Idx).t();
			_Precomp(_Idx, points[_Idx], _Tmat);
			});
	}
	catch (const DEFLOW_USE_STD(exception)& e) {
		const auto _Msg = e.what();
#ifdef _DEBUG
		printf(_Msg);
#endif
		_Is_success = false;
	}
#elif
	for (auto _Idx = 0; _Idx < _Myrefwnd.size(); ++_Idx) {
		auto _Tmat = tmats.empty() ? matrix_t() : tmats.row(_Idx);
		_Precomp(_Idx, points[_Idx], _Tmat);
	}
#endif
	return _Is_success;
}

sparse_neighbor_match_wrapper::matrix_t&
sparse_neighbor_match_wrapper::run(const matrix_t& tar, const point_array_t&, matrix_t pred_pars)
{
	// Get target image resource and initialize the LUT...
	if (detail::have_same_value_type(tar, _Mytarlut.image)) {
		_Mytarlut.image = tar;
	}
	else {
		tar.convertTo(_Mytarlut.image, _Mytarlut.image.type());
	}
	_Mytarlut.data = 0;
	_Mytarlut.mask = 0;

	// If the number of reference points is changed, _Myzncc and _Myparams will be resized.
	if (_Myoptions->corr_keep_coefs && _Myzncc.rows != _Myrefwnd.size()) {
		_Myzncc = detail::make_cvmat<float_t>(_Myrefwnd.size(), 1);
	}
	_Myparams = pred_pars.clone();
	if (_Myparams.empty()||_Myparams.rows != _Myrefwnd.size()) {
		_Myparams = detail::make_cvmat<value_t>(_Myrefwnd.size(), 6);
	}

#ifdef USE_TBB
	try {
		tbb::parallel_for(size_t{ 0 }, _Myrefwnd.size(), [&](auto _Idx) {
			if (_Has_shared_ref) {
				auto point = point_t();
				point.x = _Myparams.ptr<value_t>(_Idx)[0];
				point.y = _Myparams.ptr<value_t>(_Idx)[3];
				_Search({ _Idx, _Idx }, point, _Myparams);
			}
			else {
				_Search({ _Idx, _Idx }, _Myrefwnd[_Idx].center, _Myparams);
			}
			});
	}
	catch (const DEFLOW_USE_STD(exception)& e) {
		const auto _Msg = e.what();
	}
#elif
	for (auto _Idx = 0; _Idx < _Myrefwnd.size(); ++_Idx) {
		_Search({ _Idx, _Idx }, _Myrefwnd[_Idx].center, _Myparams);
	}
#endif
	return _Myparams;
}

sparse_neighbor_match_wrapper::matrix_t&
sparse_neighbor_match_wrapper::run(const DEFLOW_USE_STD(string)& path, const matrix_t& pred_transfrom)
{
	return run(imread(path), {}, pred_transfrom);
}

sparse_neighbor_match_wrapper::matrix_t
sparse_neighbor_match_wrapper::_Search(index_n_t<2> idx, point_t point, matrix_t& pars)
{
	decltype(auto) _Ref = _Myrefwnd[idx[0]];
	decltype(auto) _Roi = _Myoptions->roi_mask;

	auto _Cands = DEFLOW_USE_STD(vector<point_t>)();
	//[dwd_offset, up_offset, left_offset, right_offset]
	const auto _Radius = _Myoptions->search_radius;
	for (auto r = -_Radius[0]; r <= _Radius[1]; ++r) {
		const auto y = point.y + r;
		for (auto c = -_Radius[2]; c <= _Radius[3]; ++c) {
			const auto x = point.x + c;
			_Cands.push_back({ x, y });
		}
	}

	auto _Pars = pars.ptr<value_t>(idx[1]);
	value_t _Disp_grads[] = {_Pars[1], _Pars[2], _Pars[4], _Pars[5]};
#ifdef USE_TBB
	DEFLOW_USE_STD(vector<value_t>) _Scores(_Cands.size());
	tbb::parallel_for(size_t{ 0 }, _Cands.size(), [&](auto&& _Off) {
		const auto& u = _Cands[_Off];
		auto _Gwnd = detail::make_cvmat<value_t>(_Ref.data.rows, _Ref.data.cols);
		internal::interpolation_t::value_one_affine_template_64f(
			_Mytarlut.image, u.x, u.y, _Disp_grads,
			_Mytarlut.x_min, _Mytarlut.x_max, _Mytarlut.y_min, _Mytarlut.y_max,
			_Mytarlut.data, _Mytarlut.mask, _MyQmat, _Gwnd);
		auto g_mean = DEFLOW_USE_CV(mean)(_Gwnd)[0];
		internal::correlation_t::eval_zncc(_Ref.data, _Gwnd, &_Ref.mean, &_Ref.ssdv, &g_mean, &_Scores[_Off]);
		});
	const auto _Off = std::distance(_Scores.begin(), std::max_element(_Scores.begin(), _Scores.end()));
	if (_Off < _Cands.size()) {
		auto& _T = _Ref.transform = pars.cols==1?pars.clone():pars.row(idx[1]).t();
		if (_T.empty()) {
			_T = detail::make_cvmat<value_t>(6, 1);
		}
		_T.ptr<value_t>()[0] = _Cands[_Off].x - _Ref.center.x;
		_T.ptr<value_t>()[3] = _Cands[_Off].y - _Ref.center.y;
	}
	else {
		return {};
	}
	auto [_It, _C1, _Pmat] = internal::_Subpixel_refine(_Ref, _Mytarlut, _MyQmat, _Myoptions->brief_corr());
	DEFLOW_USE_STD(copy)(_Pmat.begin<value_t>(), _Pmat.end<value_t>(), _Pars);
	if (_Myoptions->corr_keep_coefs) {
		_Myzncc.ptr<float_t>()[idx[1]] = _C1;
	}
	return _Pmat;
#else

#endif
	return matrix_t();
}
}