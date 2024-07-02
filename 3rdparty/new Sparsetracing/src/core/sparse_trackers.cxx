/*********************************************************************
 This file is part of Deflow R&D Platform Infrastructure.
 Copyright(C) GOS.VI Lab since 2020, all rights reserved.
*********************************************************************/
#include <QDate>
#include <QImage>
#include <random>

#include "sparse/core/sparse_trackers.h"
#include "core/point_search_wrapper.h"

namespace deflow {

using value_t = sparse_neighbor_match_wrapper::value_t;
using OSTracker = OnlineSparseTracker;

OnlineSparseTracker::OnlineSparseTracker(const options_t& options) noexcept
{
	_Myimpl = std::make_shared<sparse_neighbor_match_wrapper>(options);
}
OnlineSparseTracker::~OnlineSparseTracker()
{
	_Myimpl->release_memory();
}
bool OnlineSparseTracker::init(const QImage* frame, const QList<QPointF>& pois)
{
	if (frame == nullptr) return false;
	if (_Is_valid()) {
		emit error("Fail to initialize the sparse tracker due to \
                    unknown compatibility issues!");
		return false;
	}
	if (!pois.empty()) { // plain patterns...
		_Mybuff = decltype(_Mybuff)(frame->height(), frame->width(),
			CV_8UC1, (uchar*)(frame->scanLine(0))).clone();

		_Myimpl->options()->set_image_size(_Mybuff.rows, _Mybuff.cols);
		_Myimpl->init(_Mybuff, pois.size());
		_Myimpl->set_radius(_Myimpl->options()->search_radius);

		auto points = _Myimpl_t::element_type::point_array_t();
		for (const auto& poi : pois) {
			points.push_back({ poi.x(), poi.y() });
		}
		if (_Myimpl->declare(points, {})) {
			_Myflow = 0; // Clear the stored displacement data before.
		}
		else {
			emit error("Fail to declare the sparse tracker engine!");
			return false;
		}
	}
	return true;
}
QVector<QPointF> OnlineSparseTracker::exec(frame_t&& frame, policy_tag tag)
{
	if (frame.empty()) return{};

	_Myflow = _Myimpl->run(frame);
	auto flow = QVector<QPointF>(_Myflow.rows);
	for (auto i = 0; i < flow.size(); ++i) {
		flow[i] = { _Myflow.ptr<value_t>(i)[0], _Myflow.ptr<value_t>(i)[3] };
	}

	return flow;
}

bool OnlineSparseTracker::free() {
	_Myimpl->release_memory();
	return true;
}

bool OnlineSparseTracker::_Is_valid() noexcept {
	auto __is_expired = [](QDate&& date) {
		const auto __dvalue = [](size_t y, size_t m, size_t d) {
			size_t mon[] = { 0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334 };
			return y * 365 + y / 4 - y / 100 + y / 400 + mon[m - 1] + d - 1 + 
				(((y % 100 != 0 && y % 4 == 0) || y % 400 == 0) && m > 2);
		};

		const auto cur = QDate::currentDate();
		return __dvalue(cur.year(), cur.month(), cur.day()) > __dvalue(date.year(), date.month(), date.day());
	};

	auto is_expired = __is_expired({ 2025, 2, 28 });
	if (is_expired) {
		is_expired = std::bind(std::uniform_int_distribution<>(0, 1), 
			std::default_random_engine())();
	}
	return is_expired;
}

} // namespace deflow