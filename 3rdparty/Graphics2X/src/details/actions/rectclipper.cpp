#include "actions/rectclipper.h"
#include "utils/regionitem.h"
#include "utils/nodeitem.h"

void RectClipper::exec()
/* initialize cutting the rectangular */
{
	/* initialize the need cut area */
	_needCutArea = new AreaItem();
	_needCutArea->setPolygon(QPolygonF(QRectF(QPointF(0,0),QPointF(0,0))));
	m_scene->addItem(_needCutArea);
	_needCutArea->setPos(_p1);
	_needCutArea->transparency_count = 0; // the cut part is transparent
	m_scene->m_shapes << _needCutArea;

	/* initialize the node of the area */
	for (int i = 0; i < 4; i++)
	{
		NodeItem* node = new NodeItem(_needCutArea);
		_needCutArea->node_list << node;
		node->setPos(_p1);
	}

	m_scene->add_cursor_label();     // show the coordinate label
}

void RectClipper::move()
/* when the mouse move, cutting the rectangle */
{
	if (!m_scene->m_shapes.isEmpty()) {

		QRectF rect(_p1,_p2);

		/* when the mouse move, update the polygon */
		_needCutArea = m_scene->m_shapes.last();
		_needCutArea->setPolygon(QPolygonF(rect.translated(-_p1)));
		
		/* when the mouse move, update the nodes */
		_needCutArea->node_list.value(0)->setPos(rect.topLeft());
		_needCutArea->node_list.value(1)->setPos(rect.topRight());
		_needCutArea->node_list.value(2)->setPos(rect.bottomRight());
		_needCutArea->node_list.value(3)->setPos(rect.bottomLeft());

		if (m_scene->m_is_merge_poly) /* if need merge */
		{
			m_scene->ClipInitSet();
			m_scene->ClipRegionMove(_needCutArea);
		}

		m_scene->move_cursor_label(_p2); // move the coordinate label
	} 

}

void RectClipper::clip()
/* when the left mouse pressed, end cutting */
{
	if (!m_scene->m_shapes.isEmpty()) {
		_needCutArea = m_scene->m_shapes.last();
		if (m_scene->m_is_merge_poly)
		{
			m_scene->ClipInitSet();
			m_scene->ClipRegionPress(_needCutArea);
		}

		m_scene->DelArea(_needCutArea);
		m_scene->m_shapes.removeOne(_needCutArea);
		m_scene->delete_cursor_label();
	}
}

void RectClipper::undo()
{

}