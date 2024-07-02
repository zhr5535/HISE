#include "calculation_worker.h"

calculation_worker::calculation_worker()
    : m_options(deflow::make_shared_options()),                               
      m_sparseflow(std::make_unique<deflow::OnlineSparseTrackingWorker>(m_options)) 
{                                                                             
}

/* calculation_worker::~calculation_worker()
{
    m_sparseflow.reset();
    m_options.reset();
} */