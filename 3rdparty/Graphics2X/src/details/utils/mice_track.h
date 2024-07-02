#pragma once
#include <QPointF>

class EditableGraphicsScene;

class MouseTrackCommand
{
public:
	MouseTrackCommand(EditableGraphicsScene* scene, QPointF point1) 
		: m_scene(scene), point(point1) {
	}
	void exec();

private:
	QPointF point;
	EditableGraphicsScene* m_scene;
};
