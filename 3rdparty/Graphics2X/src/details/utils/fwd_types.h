#pragma once

enum class GraphicEditorStatus
{
	UNUSED = -1,
	READY = 0,
	DRAW_RECT = 1,
	DRAW_CIRC = 2,
	DRAW_POLY = 3,
	DRAW_POINT = 4,
	CLIP_RECT = 5,
	CLIP_CIRC = 6,
	CLIP_POLY = 7,
	AUTO_SEED = 8,
	DEL_SHAPE = 9,
	DEL_NODE = 10,
	SELECT = 11,
	DRAW_LINE = 12,
	DRAW_SOLOLINE = 13,
	MOVE_REGION = 14,
};
using GraphicEditCmd = GraphicEditorStatus;

enum class PointStyle : uint8_t {
	TARGET_CROSS = 0,
	FRAMED_CROSS = 1,
	FILLED_CIRCL = 2,
	HOLLOW_CIRCL = 3,
	PINNED_POINT = 4,
};

template<typename T> struct Pi {
	static constexpr auto value = T(3.14159265);
};
/**
 *\brief The value of Pi for a given type T. 
 */
template<typename T = float>
#ifdef _HAS_CXX17
inline constexpr auto pi_v = Pi<T>::value;
#else
constexpr auto pi_v = Pi<T>::value;
#endif