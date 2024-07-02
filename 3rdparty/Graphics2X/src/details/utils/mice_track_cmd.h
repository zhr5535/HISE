#pragma once
#include <QPointF>

class MouseTracking
{
public:
	MouseTracking(MaskEditorGraphicsScene* scene, QPointF point) 
		: mm_scene(scene), m_point(point) {}

	void exec() {};

private:
	QPointF m_point;
	MaskEditorGraphicsScene* mm_scene = nullptr;
};
