/*********************************************************************
 This file is part of Deflow R&D Platform Infrastructure.
 Copyright(C) GOS.VI Lab since 2020, all rights reserved.
*********************************************************************/
#pragma once

namespace deflow 
{
	enum class point_search_policy {
		DETACH_REF = 0,
		SHARED_DEF = 1,
		SINGLE_REF = 2,
	};
	enum class strain_tag
	{
		LAG=0, // Lagrange strain
		EUL=1, // Euler-Almansi strain
		ENG=2, // Engineering strain
		TRU=3, // True strain
		PRI=4, // Principle strain
	};

	enum class strain_calc_tag {
		BRIEF=0, // Eval strain_tag::LAG and ENG only
		VERBO=1, // Eval all strain fields
	};

	enum class strain_exec_policy {
		SEQ=0, // Exect in sequential
		PAR=1, // Exect in parallel
	};

}
