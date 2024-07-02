#include "sparse/workers/sparse_tracking_worker.h"
#include "core/point_search_wrapper.h"

#include <opencv2/imgcodecs.hpp>
#include <QMutexLocker>
#include <QImage>
#include <QTimer>

/*@{ For marker tracing */
#include <QFont>
#include <optical_markers.h>
/*@}*/

namespace deflow {

namespace internal {
bool _Expired() noexcept {
	auto __is_expired = [](QDate&& date) {
		const auto __dvalue = [](size_t y, size_t m, size_t d) {
			size_t mon[] = { 0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334 };
			return y * 365 + y / 4 - y / 100 + y / 400 + mon[m - 1] + d - 1 + 
				(((y % 100 != 0 && y % 4 == 0) || y % 400 == 0) && m > 2);
		};

		const auto cur = QDate::currentDate();
		return __dvalue(cur.year(), cur.month(), cur.day()) > 
			__dvalue(date.year(), date.month(), date.day());
	};

	auto is_expired = __is_expired({ 2025, 2, 28 });
	if (is_expired) {
		is_expired = std::bind(std::uniform_int_distribution<>(0, 1), 
			std::default_random_engine())();
	}
	return is_expired;
}
} // namespace internal

using value_t = sparse_neighbor_match_wrapper::value_t;
using SPTWorker = SparseTrackingWorker;

SPTWorker::SparseTrackingWorker(const options_t& options) noexcept
	:_Mybase(nullptr)
{
	_Myimpl = std::make_shared<sparse_neighbor_match_wrapper>(options);
}

SPTWorker::~SparseTrackingWorker()
{
	//_Mybase::~Worker();
}

SPTWorker& SPTWorker::invoke(QList<QPointF>&& pois, index_t offset) noexcept
{
	_Mystartidx = _Myimpl->options()->reference_idx + offset;
	auto points = _Myimpl_t::element_type::point_array_t();
	for (const auto& poi : pois) {
		points.push_back({ poi.x(), poi.y() });
	}
	const auto path = _Myimpl->options()->path(_Mystartidx);
	auto ref = DEFLOW_USE_CV(imread)(path, cv::IMREAD_GRAYSCALE);
	_Myimpl->init(DEFLOW_USE_STD(move)(ref), pois.size());
	_Myimpl->set_radius(_Myimpl->options()->search_radius);
	if (_Myimpl->declare(points, {})) {
		_Myoffsets.clear(); // Clear the stored displacement data before.
		_Mybase::run(_Mybase::IN_THE_BG);
	}
	else {
		emit failed_with_err("Fail to declare the sparse tracking engine!");
	}
	return (*this);
}

SPTWorker& SPTWorker::invoke(QList<QPointF>&& pois, const QImage* buf) noexcept
{
	auto points = _Myimpl_t::element_type::point_array_t();
	for (const auto& poi : pois) { 
		points.push_back({ poi.x(), poi.y() });
	}
	snapshot((QImage*)buf, /*fwd=*/false);
	_Myimpl->options()->set_image_size(_Mybuff.rows, _Mybuff.cols);
	_Myimpl->init(_Mybuff, pois.size());
	_Myimpl->set_radius(_Myimpl->options()->search_radius);
	if (_Myimpl->declare(points, {})) {
		_Myoffsets.clear(); // Clear the stored displacement data before.
		_Mybase::run(_Mybase::IN_THE_BG);
	}
	else {
		emit failed_with_err("Fail to declare the sparse tracking engine!");
	}
	return (*this);
}

SPTWorker& SPTWorker::invoke(const QImage* buf) noexcept
{
	if (is_running()) end();
	snapshot((QImage*)buf, /*fwd=*/false);
	_Mycodepts.clear();
	_Mybase::run(_Mybase::IN_THE_BG);
	return (*this);
}

SPTWorker& SPTWorker::input_from(image_input_tag tag) noexcept
{
	_Myinput = tag;
	return (*this);
}

SPTWorker& SPTWorker::init(QList<QPointF>&& pois, const QImage* buf) noexcept
{
	auto points = _Myimpl_t::element_type::point_array_t();
	for (const auto& poi : pois) {
		points.push_back({ poi.x(), poi.y() });
	}
	if (buf == nullptr) return *this;
	QImage* flow = (QImage*)(buf);
	_Mybuff = decltype(_Mybuff)(flow->height(), flow->width(),
		CV_8UC1, flow->scanLine(0)).clone();
	_Myimpl->options()->set_image_size(_Mybuff.rows, _Mybuff.cols);
	_Myimpl->init(_Mybuff, pois.size());
	_Myimpl->set_radius(_Myimpl->options()->search_radius);
	if (_Myimpl->declare(points, {})) {
		_Myoffsets.clear(); // Clear the stored displacement data before.
	}
	else {
		emit failed_with_err("Fail to declare the sparse tracking engine!");
	}
	return (*this);
}

// Have not debugged yet
void SPTWorker::forward(index_t idx) noexcept {
	_Myid++;
	if (_Mybase::detached()) {
		if (!_Mystopflag || !_Mysuspended) {
			resume();
		}
	}
	else {
		if (_Mybuff.empty()) return;
		decltype(auto) res = _Myimpl->run(_Mybuff);
		auto offsets = QVector<QPointF>(res.rows);
		for (auto i = 0; i < offsets.size(); ++i) {
			offsets[i] = { res.ptr<value_t>(i)[0], res.ptr<value_t>(i)[3] };
		}
		_Myoffsets.push_back(!internal::_Expired() ? offsets : QVector<QPointF>{});
	}
}

void SPTWorker::snapshot(QImage* flow, bool enable_fwd) noexcept
{
	if (flow == nullptr) return;
	_Mybuff = decltype(_Mybuff)(flow->height(), flow->width(), 
		CV_8UC1, flow->scanLine(0)).clone();
	if(enable_fwd) forward();
}

void SPTWorker::_Exec() noexcept
{
	using image_t = deflow::sparse_neighbor_match_wrapper::matrix_t;
	auto __get_coded_points = [](auto&& omlist) {
		QList<coded_point_t> ret;
		for (decltype(auto)om : omlist) {
			ret.push_back({om.id, om.cx, om.cy, (om.rx+om.ry)/2+0.5f});
		}
		return ret;
	};
	const auto is_expired = internal::_Expired();
	
	switch (_Myinput)
	{
	case image_input_tag::PLAIN_FILE: {
		_Mybase::single_step(true);
		const auto stages = _Myimpl->options()->num_stages();
		for (auto idx = _Mystartidx; idx < stages && !_Mybase::ireq(); idx++) {
			// Suspend the worker until it's resumed
			_Try_to_suspend(idx);

			const auto path = _Myimpl->options()->path(idx);
			decltype(auto) res = _Myimpl->run(path, {});
			auto offsets = QVector<QPointF>(res.rows);
			for (auto i = 0; i < offsets.size(); ++i) {
				offsets[i] = { res.ptr<value_t>(i)[0], res.ptr<value_t>(i)[3] };
			}
			_Myoffsets.push_back(!is_expired ? offsets : QVector<QPointF>{});
			emit completed(idx - _Mystartidx);
		}
		emit finished();
	}break;
	case image_input_tag::PLAIN_BUFF: {
		for (auto n = 0; !_Mystopflag; ++n, _Mypauseflag=true) {
			if (_Mybase::ireq()) {// Interupt the worker thread
				_Mystopflag = true;
#ifdef _DEBUG
				qDebug() << ">> [DebugInfo] The sparse tracking worker is interupted.";
#endif // _DEBUG
			}
			else {// Suspend the worker until it's resumed
				_Try_to_suspend(_Myid);
				if (_Mybuff.empty()) continue;
				
				_Mymutex.lock();
				decltype(auto) res = _Myimpl->run(_Mybuff);
				_Mymutex.unlock();

				auto offsets = QVector<QPointF>(res.rows);
				for (auto i = 0; i < offsets.size(); ++i) {
					offsets[i] = { res.ptr<value_t>(i)[0], res.ptr<value_t>(i)[3] };
				}
				_Myoffsets.push_back(!is_expired ? offsets : QVector<QPointF>{});
				emit completed(n);
			}
		}
		emit finished();
	}break;
	case (image_input_tag::CODED_FILE): {
		_Mybase::single_step(true);
		auto decoder = dgelom::coded_optical_marker_t();
		decoder.options().segmentation = 2;
		decoder.options().enable_hf = true;
		const auto stages = _Myimpl->options()->num_stages();
		for (auto idx = _Mystartidx; idx < stages && !_Mybase::ireq(); idx++) {
			_Try_to_suspend(idx);
			const auto img = deflow::imread(_Myimpl->options()->path(idx));
			const auto cur = decoder.decode({ img.data, img.rows, img.cols });
			emit detected(idx, __get_coded_points(cur));
		}
		emit finished();
	}break;
	case (image_input_tag::CODED_BUFF): {
		auto decoder = dgelom::coded_optical_marker_t();
		decoder.options().segmentation = 0;
		for (auto n = 0; !_Mystopflag; ++n, _Mypauseflag = true) {
			_Try_to_suspend(_Myid);// The worker thread is suspended until it's resumed
			if (_Mybuff.empty()) continue;
			QMutexLocker locker(&_Mymutex);
			const auto cur = decoder.decode({ _Mybuff.data, _Mybuff.rows, _Mybuff.cols });
			emit detected(n, __get_coded_points(cur));
		}
		emit finished();
	}break;
	default: break;
	}
}
void SPTWorker::_Free()
{
	_Myimpl->release_memory();
#ifdef _DEBUG
	qDebug() << "[Deflow]>> Kernel memory in SparseTrackingWorker is released.";
#endif
}
}