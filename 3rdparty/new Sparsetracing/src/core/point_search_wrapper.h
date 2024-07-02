#pragma once

#include <opencv2/core.hpp>
#include <tuple>

#include "utils/vectors.h"
#include "utils/functions.h"
#include "sparse/core/deflow_options.h"

namespace deflow {

class stereo_match_wrapper;
class sparse_neighbor_match_wrapper;
class dense_neighbor_match_wrapper;
// Alias of unique pointer to neighbor_match_wrapper
using unique_sparse_neighbor_matcher = std::unique_ptr<sparse_neighbor_match_wrapper>;

/// <summary>
/// \brief CLASS, base class for image correlation search
/// </summary>
class DEFLOW_ALG correlation_match {
	using _Myt = correlation_match;
public:
	using value_t = double;
	using point_t = internal::Vec_<value_t, 2>;
	using matrix_t = cv::Mat;
	using point_array_t = std::vector<point_t>;
	template<size_t _N>
	using index_n_t = std::array<size_t, _N>;
	
	/**
	 * \brief CTOR, set search parameters and allocate reference window and LUT memories. 
	 */
	explicit correlation_match(const options_t& pars) noexcept;
	explicit correlation_match(const _Myt& other) noexcept;
	explicit correlation_match(_Myt&& other) noexcept;
	~correlation_match();

	/**
	 * \brief Init with a reference image. The owership is reserved by source data.
	 * \param ref Reference image.
	 * \param num_tasks Number of tasks to execute the correlation matching, it should
	          be equal to the number of computation points given in method ::run().
	 */
	_Myt& init(const matrix_t& ref, uint32_t num_tasks=1) noexcept;
	/**
	 * \brief Init with a reference image. The owership is moved to this object.
	 */
	_Myt& init(matrix_t&& ref, uint32_t num_tasks=1) noexcept;

	/**
	 * \brief Batch matching.
	 * \param points N computation points in the reference image.
	 * \param tmats [Optional] match deformed images if a non-empty
					matrix (Nx6, [u,ux,uy,v,vx,vy]) is passed. The rows
					of 'tmats' must be the same as the size of 'points'.
	 */
	virtual matrix_t& run(const matrix_t& tar, const point_array_t& points, matrix_t tmats)=0;

	/**
	 * \brief Update the reference transform parameters.
	 */
	void update_ref_transforms(const matrix_t& pars) noexcept;

	/**
	 * \brief Get the estimated deformation parameters
	 */
	decltype(auto) get_params() const noexcept {
		return (_Myparams);
	}
	matrix_t&& get_params() noexcept {
		return std::forward<matrix_t>(_Myparams);
	}

	/**
	 * \brief Get correlation coeffs matrix cv::Mat<float>.
	 */
	inline decltype(auto)(scores)()const noexcept {
		return (_Myzncc);
	}
	inline decltype(auto)(scores)()noexcept {
		return (_Myzncc);
	}

	/**
	 * \brief Get options in type of options_t.
	 */
	inline decltype(auto)(options)()const noexcept {
		return (_Myoptions);
	}
	inline decltype(auto)(options)()noexcept {
		return (_Myoptions);
	}

	/**
	 * \brief Get reference subsets.
	 */
	inline decltype(auto)ref_subsets() const noexcept {
		return (_Myrefwnd);
	}

	inline void release_memory() {
		_Myrefwnd.clear();
		_Mytarlut.data.release();
		_Mytarlut.image.release();
		_Mytarlut.mask.release();
	}

protected:
	const _Myt& _Get() const noexcept;
	_Myt& _Get() noexcept;

	/**
	 * \brief Precomputation for a start point and task index.
	 * \param idx Index of thread task. If the num_tasks in method
			  ::set_ref_image(...,num_tasks) is 1, idx should be 0.
	 * \param point Computation point in the reference image.
	 * \param tmat [Optional] match deformed images if a non-empty
					matrix (6x1, [u,ux,uy,v,vx,vy]) is passed.
	 */
	_Myt& _Precomp(size_t idx, point_t point, matrix_t tmat = matrix_t());

	/**
	 * \brief Interface for correlation search.
	 * \param idx Indices of the task (idx[0], local) to be run the search and
			  the given reference point (idx[1], global).
	 * \param point Coords of reference point.
	 * \param pars Matrix of deformation parameters for all points.
	 * \return matrix_t with shape of 1x6 or 6x1, which holds the parameters associated to 'point'.
	 */
	virtual matrix_t _Search(index_n_t<2> idx, point_t point, matrix_t& pars) = 0;

	// Reference to the reference image, but donot get the ownership [see reference counting]
	matrix_t _Myrefimg{};
	// Search parameter settings
	options_t _Myoptions;

	struct _Template
	{
		point_t center;
		matrix_t data;
		matrix_t gx;
		matrix_t gy;
		matrix_t mask{}; // preserved for defining template shape
		value_t mean{ 0 };
		value_t ssdv{ 0 }; //second-order central moment, #sum{f-mean}^2

		matrix_t inverse_hessian; //6x6 matrix
		matrix_t transform; //6x1 matrix [u, ux, uy, v, vx, vy]

		_Template& create(size_t size);
	};
public:
	using template_t = _Template;
protected:
	// Reference templates, sliding according to the point
	std::vector<template_t> _Myrefwnd;

	struct _LUT {
		size_t x_min{ 2 }, x_max{ 0 };
		size_t y_min{ 2 }, y_max{ 0 };
		size_t cell_size{ 16 };
		matrix_t image; //left or right deform image
		matrix_t data;  //interpolation LUT, requires 48(16x3)-fold memory
		matrix_t mask;  //valid sign matrix for LUT

		void create(size_t rows, size_t cols);
		void free();

		// Eq. y_max - y_min
		inline auto rows() const noexcept {
			return y_max - y_min;
		}
		// Eq. x_max - x_min
		inline auto cols() const noexcept {
			return (x_max - x_min);
		}
		// Eq. cols()*cell_size
		inline auto width() const noexcept {
			return cols()*cell_size;
		}
	};
public:
	using lut_t = _LUT;
protected:
	// Target image LUT
	lut_t _Mytarlut;

	// Interpolation matrix Q: 6x6
	matrix_t _MyQmat;
	// Correlation coefficients
	matrix_t _Myzncc;
	// Matrix holds evaluated deformation parameters
	matrix_t _Myparams;
};

/// <summary>
/// \brief CLASS, stereo image search
/// </summary>
class DEFLOW_ALG stereo_match_wrapper : public correlation_match {
	
	using _Myt = stereo_match_wrapper;
	using _Mybase = correlation_match;
public:
	using point3_t = internal::Vec_<value_t,3>;
	using unique_neighbor_matcher_tuple = std::tuple<unique_sparse_neighbor_matcher, unique_sparse_neighbor_matcher>;

	/**
	 * \brief CTOR, #sa Base correlation_match
	 */
	explicit stereo_match_wrapper(const options_t& options) noexcept
		:_Mybase(options) {
	};

	/**
	 * \brief Set fundamental matrix F and search radius
	 */
	_Myt& set_condition(const matrix_t& F, value_t radius = 5);
	_Myt& constraint(value_t radius = 5);

	std::array<float, 4> epline(float x, float y) const noexcept;

	/**
	 * \brief Override of _Mybase::run;
	 */
	virtual matrix_t& run(const matrix_t& tar, const point_array_t& points, matrix_t tmats={}) override;

	/**
	 * \brief Stereo point match with the idential reference points;
	 */
	matrix_t run(const std::string& path, matrix_t transforms);

	/**
	 * \brief Make match branches with type of std::unique_ptr<neighbor_match_wrapper>.
	 * \param keep_transform Tag for using the reference transformation or not.
	 * \return A tuple where the first one is the left neighbor_matcher and the second is
	 *        the right neighbor_matcher.
	 */
	unique_neighbor_matcher_tuple detach(bool share_reference = false);

private:
	/**
	 * \brief Overrie of _Mybase::_Search.
	 * \param idx Indices of the task (idx[0], local) to be run the search and 
	          the given reference point (idx[1], global).
	 * \param point Coords of reference point.
	 * \param pars Matrix of deformation parameters for all points.
	 * \return matrix_t with shape of 1x6 or 6x1, which holds the parameters associated to 'point'.
	 */
	virtual matrix_t _Search(index_n_t<2> idx, point_t point, matrix_t& pars) override;

	/**
	 * \brief Get epipolar line corresponding to the given ref point 'p'.
	 * \param p Coords of reference point.
	 * \param point3_t holds coefs (a, b, c) of the estimated epipolar line.
	 */
	point3_t _Get_epline(point_t p) const noexcept;

	/*Fields*/
	using _Mybase::_Myoptions;
	matrix_t _MyF{};
};

/// <summary>
/// \brief CLASS, Sparse Neighborhood Image Search
/// </summary>
class DEFLOW_ALG sparse_neighbor_match_wrapper : public correlation_match {
	using _Myt = sparse_neighbor_match_wrapper;
	using _Mybase = correlation_match;
public:
	/**
	 * \brief CTOR, #sa Base correlation_match
	 */
	explicit sparse_neighbor_match_wrapper(const options_t& pars) noexcept
		:_Mybase(pars) {
	};
	/**
	 * \brief CTOR, from a base class instance
	 */
	explicit sparse_neighbor_match_wrapper(const _Mybase& base) noexcept
		:_Mybase(base) {
	}
	/**
	 * \brief CTOR, from a base class instance
	 */
	explicit sparse_neighbor_match_wrapper(_Mybase&& base) noexcept
		:_Mybase(std::move(base)) {
	}

	/**
	 * \brief Set radius of neighborhood search.
	 * \param value The radius value.
	 * \return Reference of this object instance.
	 */
	_Myt& set_radius(const int32_t* value) noexcept;

	/**
	 * \brief Set masks for search region
	 * \param roi The mask for ROI, 1 for inner points and 2 for edge points.
	 * \param edge The mask for ROI edge, tag with 1 for edge points.
	 * \return Reference of this object instance.
	 */
	_Myt& set_masks(matrix_t roi, matrix_t edge) noexcept;

	/**
	 * \brief Declare work space according to the given computation points.
	 * \param points N computation points in the reference image.
	 * \param tmats [Optional] match deformed images if a non-empty
					matrix (Nx6, [u,ux,uy,v,vx,vy]) is passed. The rows
					of 'tmats' must be the same as the size of 'points'.
					The parameter is used for updating reference image.
	 */
	bool declare(const point_array_t& points, matrix_t tmats);

	/**
	 * \brief Override of _Mybase::run. The last two parameters are placeholders.
	 * \param tar Target image to search the computational points.
	 */
	virtual matrix_t& run(const matrix_t& tar, const point_array_t&_={}, matrix_t={}) override;

	/**
	 * \brief Overload of method _Mybase::run(...).
	 */
	matrix_t& run(const std::string& path, const matrix_t& pred_transfrom);

	inline decltype(auto) origin_point(size_t idx) const noexcept {
		return (_Myrefwnd[idx].center);
	}
	
	inline void share_ref(bool value = true) noexcept { _Has_shared_ref = value; }
private:
	/**
	 * \brief Overrie of _Mybase::_Search.
	 * \param idx Indices of the task (idx[0], local) to be run the search and
			  the given reference point (idx[1], global).
	 * \param point Coords of reference point.
	 * \param pars Matrix of deformation parameters for all points.
	 * \return matrix_t with shape of 1x6 or 6x1, which holds the parameters associated to 'point'.
	 */
	virtual matrix_t _Search(index_n_t<2> idx, point_t point, matrix_t& pars) override;

	/**
	 * \brief Update position of a POI to a new one.
	 */
	point_t _Update(const point_t& point) const noexcept;

	const int32_t* _Myradius{ _Myoptions->search_radius };
	bool _Has_shared_ref{ false };
	using _Mybase::_Myoptions;
};
}