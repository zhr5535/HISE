#include "sparse/core/deflow_summary.h"

namespace deflow {
DEFLOW_ALG Summary& Summary::set_index(uint32_t idx) noexcept
{
	_Mymutex.lock();
	stage_index = idx;
	_Mymutex.unlock();
	return (*this);
}
DEFLOW_ALG Summary& Summary::set_elapsed_time(float_t elt) noexcept
{
	_Mymutex.lock();
	elap_time = elt;
	_Mymutex.unlock();
	return (*this);
}
DEFLOW_ALG Summary& Summary::set_num_iters(float_t its) noexcept
{
	_Mymutex.lock();
	num_iters = its;
	_Mymutex.unlock();
	return (*this);
}

}