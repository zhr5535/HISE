#pragma once
#include <map>
#include <list>
#include <mutex>
#include <opencv2/core/mat.hpp>

#include "sparse/__defs_ports__.hpp"
#include "deflow_types.hpp"

namespace deflow {
/// <summary>
/// \brief STRUCT Summary, stores the results of each computation stage (index).
/// </summary>
struct Summary {
#pragma region FIELDS
	// Index of current proccesed stage
	uint32_t stage_index{ 0 };
	// Elapsed time (s) of processing current stage
	float_t elap_time{ 10 };
	// Mean number of iterations for all quadrature points
	float_t num_iters{ 5 };

	// Correlation score maps, $(RN x RM) x 2$ 
	DEFLOW_USE_CV(Mat) corr_map[2];
	// Matched image coordinates, $(RNRM x C2) x 2$
	DEFLOW_USE_CV(Mat) coord_uv_map[2];
	// Validity mask of quadruture points, $(RN x RM)$
	DEFLOW_USE_CV(Mat) qp_valmask;

	// Reconstructed 3D coordinates, $RNRM x 3$ or $3 x RNRM$
	DEFLOW_USE_CV(Mat) coord_xyz_map;
	// 3d displacement fields, $RNRM x 3$ or $3 x RNRM$
	DEFLOW_USE_CV(Mat) disp_map;

	// std::map holds strain maps according to the defined strain tag
	DF_USE_STD(map)<strain_tag, DF_USE_CV(Mat)> strain_map;
#pragma endregion

	DEFLOW_ALG Summary& set_index(uint32_t idx) noexcept;
	DEFLOW_ALG Summary& set_elapsed_time(float_t elt) noexcept;
	DEFLOW_ALG Summary& set_num_iters(float_t its) noexcept;

private:
	DEFLOW_USE_STD(mutex) _Mymutex;
};
using summary_t = DEFLOW_USE_STD(shared_ptr<Summary>);
using unique_summary_t = DEFLOW_USE_STD(unique_ptr<Summary>);
using summary_list_t = DEFLOW_USE_STD(list<Summary>);

inline auto make_shared_summary() noexcept {
	return DEFLOW_USE_STD(make_shared<Summary>)();
}
inline auto make_unique_summary() noexcept {
	return DEFLOW_USE_STD(make_unique<Summary>)();
}

struct BriefSummary {
	size_t step_idx{ 0 };
	float_t elapsed_time{ 1000 }; //s
	float_t num_iters;
};
using brief_summary_t = BriefSummary;

/// <summary>
/// \brief Store summary of current computation step.
/// </summary>
/// <typeparam name="Op">User-defined functor</typeparam>
/// <param name="summary">\sa struct Summary</param>
/// <param name="op">Function to peform save operation</param>
/// <returns>A BOOL flag indicating whether the save operation was successful.</returns>
template<typename Op>
bool save(const Summary& summary, Op&& op) {
	return op(summary);
}
}