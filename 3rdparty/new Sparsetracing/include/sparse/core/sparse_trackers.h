/*********************************************************************
 This file is part of Deflow R&D Platform Infrastructure.
 Copyright(C) GOS.VI Lab since 2020, all rights reserved.
*********************************************************************/
#pragma once
#include <QList>
#include <QPoint>
#include <QObject>

#include "sparse/core/deflow_options.h"

namespace deflow {
class sparse_neighbor_match_wrapper;

class DEFLOW_ALG OnlineSparseTracker : public QObject {
	Q_OBJECT;
	using _Myt = OnlineSparseTracker;
	using _Mybuff_t = DF_USE_CV(Mat);
	using _Myimpl_t = DF_USE_STD(shared_ptr<sparse_neighbor_match_wrapper>);
public:
	enum class pattern_tag : uint8_t
	{
		PLAIN = 0,
		CODED = 1,
		UNDEF = 99,
	};
	enum class policy_tag : uint8_t
	{
		/**
		 * Track in a fixed domain on Lagrange frame 
		 */
		FIXED_LAGRANIAN_DOMAIN = 0,
		FLWUP_LAGRANIAN_DOMAIN = 1,
	};
	using frame_t = _Mybuff_t;
	using options_t = options_t;

	explicit OnlineSparseTracker(const options_t& options) noexcept;
	~OnlineSparseTracker();

	/**
	 * \brief Initialize the sparse tracker with a reference frame.
	 * \param ref - A given valid reference image frame
	 * \param pois  - List of POIs to be tracked. If the passed poi list is 
	    empty the tracker will be built to track coded optical markers.
	 */
	bool init(const QImage* ref, const QList<QPointF>& pois = {});

	QVector<QPointF> exec(frame_t&& frame, policy_tag tag={});

	bool free();

signals:
	void error(const QString& msg);

private:
	bool _Is_valid() noexcept;

	_Mybuff_t _Mybuff; //< Buffer for caching online image stream
	_Myimpl_t _Myimpl{ nullptr };
	_Mybuff_t _Myflow; //< Holds displacement data of POIs for current frame
};

struct tracker_tag {};
struct online : tracker_tag {};
struct offline : tracker_tag {};

template<class _Cat>
using sparse_tracker_t = std::conditional_t<std::is_same_v<online, _Cat>,
	OnlineSparseTracker, void>;
}