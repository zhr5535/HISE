#pragma once

#include <QImage>

#include <sparseflow/workers/worker.hpp>
#include <sparseflow/core/sparse_trackers.h> //for deflow::OnlineSparseTracker

#ifdef _DEBUG
#include <qdebug.h>
#endif

namespace deflow
{
    class OnlineSparseTrackingWorker : public WorkerThread
    {
        Q_OBJECT;
        using _Mybase = deflow::WorkerThread;
        using _Myt = OnlineSparseTrackingWorker;

    public:
        using tracker_t = deflow::sparse_tracker_t<deflow::online>;

        OnlineSparseTrackingWorker(const tracker_t::options_t &opts)
            : _Mybase(nullptr), _Myeng(opts)
        {
            connect(&_Myeng, &tracker_t::error, [&](auto msg)
                    { _Mybase::_Mymessage = msg; });
            //< Try to invoke next step automatically if no timer is used.
            connect(this, &_Myt::forward, [&]
                    { _Mybase::resume(); });
        }
        ~OnlineSparseTrackingWorker()
        {
            _Myeng.free();
            _Mybase::end();
        }

        /* void snapshot(QImage *flow, bool enable_fwd) noexcept
        {
            if (flow == nullptr)
                return;
            _Mybuf = flow;
            if (enable_fwd)
                resume();
        } */

        void invoke(QImage *buf, const QList<QPointF> &pois, bool timed = true)
        {
            _Mybuf = buf;
            _Mytimed = timed;
            if (_Myeng.init(buf, std::move(pois)))
            {
                _Myoffsets.clear();
                _Mycount = 0;
                _Mybase::run();
            }
            else
            {
                // For processing invoke error
            }
        }

        bool is_timed() const noexcept
        {
            return _Mytimed;
        }

        void is_timed(bool val) noexcept
        {
            _Mytimed = val;
        }

        //< Get tracked coords of all POIs in all processed frames
        decltype(auto)(operator())() const noexcept
        {
            return (_Myoffsets);
        }
        decltype(auto)(operator())() noexcept
        {
            return (_Myoffsets);
        }
        //< Get tracked coords of all POIs in the frame `idx`
        decltype(auto) operator()(index_t idx) const noexcept
        {
            return (*this)()[idx];
        }

    signals:
        void forward();

    private:
        void _Exec() noexcept override
        {
            try
            {
                QMutexLocker locker(&_Mymutex);
                auto frame = tracker_t::frame_t(_Mybuf->height(), _Mybuf->width(),
                                                CV_8UC1, (uchar *)(_Mybuf->scanLine(0)))
                                 .clone();
                if (frame.data)
                {
                    auto flow = _Myeng.exec(std::move(frame));
                    _Myoffsets.push_back(flow);
                    _Mypauseflag = true;
                    emit completed(_Mycount++);
                }
#ifdef _DEBUG
                else
                {
                    qDebug() << tr(">> Invalid frame: ID = %1").arg(_Mycount);
                }
#endif
                if (!is_timed())
                { // not controlled by timer, then resume next step
                    emit forward();
                }
            }
            catch (const std::exception &e)
            {
                this->message() = e.what();
                emit _Mybase::failed(_Mycount);
            }
        }

        const QImage *_Mybuf{nullptr};
        index_t _Mycount{0};
        tracker_t _Myeng;
        QList<QVector<QPointF>> _Myoffsets;
        std::atomic_bool _Mytimed{true};
    };
}