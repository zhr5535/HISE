#include "actions/polygonclipper.h"
#include "utils/regionitem.h"
#include "utils/nodeitem.h"

void PolygonClipper::exec()
/* initialize to cut polygon */
{
	/* initialize the cutting area */
	_polygon_area = new AreaItem();
	QPolygonF polygon;
	polygon << QPointF(0,0) << QPointF(0,0);
	_polygon_area->setPolygon(polygon);
	_polygon_area->transparency_count = 0;  // set the transparency
	_polygon_area->setPos(_p1);
	m_scene->addItem(_polygon_area);
	m_scene->m_shapes<<_polygon_area;

	/* initialize the nodes of the area */
	NodeItem* node = new NodeItem(_polygon_area);
	NodeItem* node_next = new NodeItem(_polygon_area);
	node->setPos(_p1);
	node_next->setPos(_p1);
	_polygon_area->node_list << node << node_next;

	m_scene->add_cursor_label();     // show the coordinate when mouse pressed
}

void PolygonClipper::undo()
{

}
void PolygonClipper::move()
/* when the move mouse, the cutting polygon change */
{
	if (!m_scene->m_shapes.isEmpty()) {

		/* update the polygon */
		_polygon_area = m_scene->m_shapes.last();
		QPointF pos_refer = _polygon_area->node_list.value(0)->scenePos();  // reference point
		QPolygonF polygon = _polygon_area->polygon();
		NodeItem* node_next = _polygon_area->node_list.last();
		polygon.last() = node_next->scenePos() - pos_refer;
		_polygon_area->setPolygon(polygon);

		/* update the nodes */
		node_next->setPos(_p1);

		if (m_scene->m_is_merge_poly) { // clip the polygon
			m_scene->ClipInitSet();
			m_scene->ClipRegionMove(_polygon_area);
		}

		m_scene->move_cursor_label(_p1); // move the coordinate label
	}
}


void PolygonClipper::next()
/* continue to cut polygon */
{
	/* update the area */
	_polygon_area = m_scene->m_shapes.last();
	NodeItem* node_next = new NodeItem(_polygon_area);
	node_next->setPos(_p1);
	_polygon_area->node_list << node_next;

	/* update the area polygon */
	QPolygonF polygon = _polygon_area->polygon();
	polygon << polygon.last();
	_polygon_area->setPolygon(polygon);
}

void PolygonClipper::clip()
{
	if(m_scene->m_shapes.isEmpty())
		return;
	_polygon_area = m_scene->m_shapes.last();
	if (_polygon_area->node_list.length()<3)
	{
		m_scene->DelArea(_polygon_area);
		m_scene->m_shapes.removeOne(_polygon_area);
		return;
	}
	//	m_scene->PolygonCancel();//小于三个点取消显示


// 	if (!m_scene->m_is_merge_poly)
// 		return;
	m_scene->ClipInitSet();
	m_scene->ClipRegionPress(_polygon_area);

	m_scene->DelArea(_polygon_area);
	m_scene->m_shapes.removeOne(_polygon_area);
}