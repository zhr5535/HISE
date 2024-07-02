#ifndef CAL_WORKER_H
#define CAL_WORKER_H

#include <QDebug>
#include "sparseflow/workers/online_sparse_tracking_worker.hpp"

class calculation_worker
{
public:
    calculation_worker();
    //~calculation_worker();

public:
    QList<QPointF> m_poi_list;

    deflow::options_t m_options;

    std::unique_ptr<deflow::OnlineSparseTrackingWorker> m_sparseflow;
};

#endif // CAL_WORKER_H
