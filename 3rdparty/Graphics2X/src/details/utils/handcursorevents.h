#pragma once

#include "utils/editablegraphicsscene.h"

class HandCursorEvent
{
public:
	HandCursorEvent(EditableGraphicsScene* scene, QPointF point1)
		: m_scene(scene),handPos(point1){}

	void press() noexcept; //select a target
	void move() noexcept;
private:

	EditableGraphicsScene* m_scene;
	QPointF handPos;
};
