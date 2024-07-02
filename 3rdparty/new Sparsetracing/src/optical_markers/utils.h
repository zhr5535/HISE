/**********************************************************************
 * This file is a part of DGELOM(C) Deflow Application.
 * Copyright (C) 2020-2021, Zhilong (Dgelom) Su, All Rights Reserved.
 *
 * This software is distributed WITHOUT ANY WARRANTY; without even
 * the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE.
 *********************************************************************/
#pragma once
#include <array>
#include <vector>
#include <cstdint>

enum DetectContoursMethod
{
	OTSU_Method = 0,
	CANNY_Method = 1,
	ADAPTIVE_THRESH_Method = 2,
	CANNY_Self_FindContours_Method = 3,
	Self_CANNY_Method = 4,
};
enum SubPixelPosMethod
{
	NoSubPixel_Match = 0,					 // ���������ض�λ
	Interpolation_Ellipse_Match = 1,		 // ����ʽ��ֵ
	Interpolation_Rotaion_Ellipse_Match = 2, // ����ʽ��ֵ��������ת��
	Gauss_surface_fit_Ellipse_Match = 3,	 // ��˹�������
	Surface_fit_Ellipse_Match = 4,			 // ������ϣ����ݶȷ������ζ���ʽ���
	Gauss_Curve_Fit = 5,					 // ��˹��������ݶȷ���
	Gray_Moment = 6,						 // �ҶȾط�����һά��������ǰ3�׻ҶȾ�
	Binary_Centroid = 7,					 // ��ֵ�����ķ������ķ�
	Gray_Centroid = 8,						 // ���ķ�
	Squared_Gray_Centroid = 9,				 // �Ҷ�ƽ�����ķ�
};
using subpix_refine_tag = SubPixelPosMethod;
enum MarkPointColorType
{
	BlackDownWhiteUp = 0,
	WhiteDownBlackUp = 1,
	Uncertainty = 2,
};
enum CodePointBitesType
{
	CodeBites15 = 0,
	CodeBites12 = 1,
};
enum CMMMode
{
	CalculationMode = 0,
	EvaluationMode = 1,
};
enum CMMProcessStep
{
	Start = 0,
	ChooseSettingsFinished = 1,
	ImageDetectedFinished = 2,
	PreOridentFinished = 3,
	BalanceFinished = 4,
	UnCodeMatcheFinished = 5,
	UseScaleBarsFinished = 6,
	EvaluationStep = 7,
};

enum CodingBits
{
	CB15 = 0,
	CB12 = 1,
	CB10 = 2,
};

/// <summary>
/// \brief Get virtual (dummy) code IDs for 15-bit coded mark points.
/// </summary>
/// <returns>std::array with size of 429</returns>
std::array<int, 429> _Get_dummy_code_ids_b15() noexcept;

/// <summary>
/// \brief Get truth code IDs for 15-bit coded mark points.
/// </summary>
/// <returns>std::array with size of 429</returns>
std::array<int, 429> _Get_decode_ids_b15() noexcept;
std::vector<int32_t> _Get_decode_id_b15() noexcept;
/// <summary>
/// \brief Get reserved code IDs of 15-bit markers for scale- and orienting-bars.
/// </summary>
/// <returns>std::array with size of 9</returns>
std::array<int, 4> _Get_reserved_ids_b15() noexcept;

//
static int *GetAutoOridentBarID(int &array_size /*=0*/)
{
	array_size = _Get_reserved_ids_b15().size();
	return _Get_reserved_ids_b15().data();
}

#ifndef DGE_USE_STD
#define DGE_USE_STD(TYPE) std::TYPE
#endif // !DGE_USE_STD

#ifndef DGE_USING_STD
#define DGE_USING_STD(TYPE) using std::TYPE;
#endif // !DGE_USING_STD

#ifndef DGE_USE_CV
#define DGE_USE_CV(TYPE) cv::TYPE
#endif // !DGE_USE_CV
#ifndef DGE_USING_CV
#define DGE_USING_CV(TYPE) using cv::TYPE;
#endif // !DGE_USE_CV