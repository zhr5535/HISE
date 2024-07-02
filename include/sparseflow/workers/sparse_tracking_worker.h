#pragma once
#include <QList>
#include <QPoint>
#include "sparseflow/workers/worker.hpp"
#include "sparseflow/core/deflow_options.h"

class QTimer;

namespace deflow {
class sparse_neighbor_match_wrapper;

class DEFLOW_ALG SparseTrackingWorker : public Worker {
	Q_OBJECT;
	using _Mybase = Worker;
	using _Myt = SparseTrackingWorker;
	using _Myimpl_t = DEFLOW_USE_STD(shared_ptr<sparse_neighbor_match_wrapper>);
	using _Mybuff_t = DEFLOW_USE_CV(Mat);
	using _Mytimer_t = DEFLOW_USE_STD(add_pointer_t<QTimer>);
public:
	enum image_input_tag :uint8_t {
		PLAIN_FILE = 0, //< offline plain image tracking
		PLAIN_BUFF = 1, //< online plain image tracking
		CODED_FILE = 2, //< offline coded marker tracking
		CODED_BUFF = 3, //< online coded marker tracking
		UNDE = 4,       //< ...undefined case
	};
	struct coded_point_type {
		int32_t id{ -99 };
		float_t x{ 0 }, y{ 0 };
		float_t r{ 0 };
	};
	using coded_point_t = coded_point_type;

	explicit SparseTrackingWorker(const options_t& options) noexcept;
	~SparseTrackingWorker();

	//< Set the source tag of the input image data
	_Myt& input_from(image_input_tag tag) noexcept;

	//< Initialize a online worker without threading.
	// Pipe: init()->snapshot()->operator(idx)
	// \param 'pois' holds the integer coords of points to be tracked.
	_Myt& init(QList<QPointF>&& pois, const QImage* buf) noexcept;

	//< Launch the worker by giving `pois` list and the `offset`
	//  relative to the first frame (default is no offset)
	_Myt& invoke(QList<QPointF>&& pois, index_t offset = 0) noexcept;
	//< Overload, for invoking online correlation tracing worker
	// \param `pois` The picked points to be tracked.
	// \param `buf` The image buffer of the camera stream.
	_Myt& invoke(QList<QPointF>&& pois, const QImage* buf) noexcept;
	//< Overload, for invoking online coded marker tracing worker
	// \param `buf` The image buffer of the camera stream.
	_Myt& invoke(const QImage* buf) noexcept;

	//< Get tracked coords of all POIs in all processed frames
	decltype(auto)(operator())() const noexcept {
		return (_Myoffsets);
	}
	decltype(auto)(operator())() noexcept {
		return (_Myoffsets);
	}
	//< Get tracked coords of all POIs in the frame `idx`
	decltype(auto) operator()(index_t idx) const noexcept {
		return (*this)()[idx];
	}

	//< Get the built-in buffer for online single image cache.
	//--The buffer is created by calling the `::invoke` method.
	decltype(auto)(buff)() const noexcept {
		return (_Mybuff);
	}
	decltype(auto)(buff)() noexcept {
		return (_Mybuff);
	}

	//< Query the tracker status
	inline bool is_offline() const noexcept {
		return _Myinput == PLAIN_FILE || _Myinput == CODED_FILE;
	}
	inline bool is_online() const noexcept {
		return _Myinput == PLAIN_BUFF || _Myinput == CODED_BUFF;
	}
	inline bool is_timed() const noexcept {
		return _Myinput == PLAIN_FILE || _Myinput == PLAIN_BUFF;
	}
	inline bool is_subscribed() const noexcept {
		return is_offline() || is_online();
	}

	decltype(auto) offset(const QList<coded_point_t>& cpts) noexcept {
		if (_Mycodepts.empty()) _Mycodepts.push_back(cpts);
		else {
			QList<coded_point_t> diff;
			for (decltype(auto) cp : _Mycodepts.front()) {
				auto it = std::find_if(cpts.cbegin(), cpts.cend(),
					[&cp](auto&& p) {
					return cp.id == p.id;
					});
				if (it != cpts.cend()) {
					diff.append({ cp.id, it->x-cp.x, it->y-cp.y, it->r });
				}
			}
			_Mycodepts.push_back(diff);
		}
		return _Mycodepts.back();
	}

	void free() { _Free(); }

public slots:
	void forward(index_t idx=0) noexcept;
	void snapshot(QImage* flow, bool enable_fwd=true) noexcept;

signals:
	void detected(index_t idx, const QList<coded_point_t>& pts);

private:
	virtual void _Exec() noexcept override;
	virtual void _Free() override;

	_Myimpl_t _Myimpl{ nullptr };
	_Mybuff_t _Mybuff; //Created when online invoking is called

	index_t _Mystartidx{ 0 };
	image_input_tag _Myinput{image_input_tag::UNDE};
	QList<QVector<QPointF>> _Myoffsets;
	QList<QList<coded_point_t>> _Mycodepts;
};

using sparsetracking_worker_t = SparseTrackingWorker;
using shared_sparsetrack_worker_t = DF_USE_STD(shared_ptr<SparseTrackingWorker>);

inline auto make_shared_worker(const options_t& op) noexcept {
	return DF_USE_STD(make_shared)<sparsetracking_worker_t>(op);
}
inline auto make_unique_worker(const options_t& op) noexcept {
	return DF_USE_STD(make_unique)<sparsetracking_worker_t>(op);
}
}
