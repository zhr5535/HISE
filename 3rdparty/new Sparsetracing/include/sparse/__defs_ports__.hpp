/*********************************************************************
 This file is part of Deflow R&D Platform Infrastructure.
 Copyright(C) GOS.VI Lab since 2020, all rights reserved.
*********************************************************************/
#pragma once

#include <QtCore/qglobal.h>

#ifdef DEFLOW_SPARSE_ALGORITHM_LIB
# define DEFLOW_SPARSE_ALGORITHM_EXPORT //__declspec(dllexport)
#else
# define DEFLOW_SPARSE_ALGORITHM_EXPORT //__declspec(dllimport)
#endif

#ifndef DEFLOW_ALG
#define DEFLOW_ALG DEFLOW_SPARSE_ALGORITHM_EXPORT
#endif

#ifndef DEFLOW_THR_ENABLE_TBB
#define DEFLOW_THR_ENABLE_TBB 1
#endif

#ifndef USE_TBB
#define USE_TBB 1
#endif

#ifndef DEFLOW_USE_CV
#define DEFLOW_USE_CV(T) cv::T
#endif
#ifndef DEFLOW_USE_STD
#define DEFLOW_USE_STD(T) std::T
#endif

#ifndef DF_USE_CV
#define DF_USE_CV DEFLOW_USE_CV
#endif // !DF_USE_CV
#ifndef DF_USE_STD
#define DF_USE_STD DEFLOW_USE_STD
#endif // !DF_USE_STD

#ifndef DEFLOW_SPARSE_TRACING
#define DEFLOW_SPARSE_TRACING
#endif
