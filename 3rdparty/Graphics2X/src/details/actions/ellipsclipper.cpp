#include "utils/regionitem.h"
#include "utils/nodeitem.h"
#include "ellipsclipper.h"

/* initialize to cut circle */
void EllipsClipper::exec() {
	/* if the inner triangle exist, then delete it */
	if (m_scene->m_triangle_shape != nullptr) {
		m_scene->removeItem(m_scene->m_triangle_shape);
		m_scene->m_triangle_shape = nullptr;
	}

	/* initialize the area and nodes */
	auto area_triangle = new AreaItem();  // inner triangle
	auto node_first = new NodeItem(area_triangle);
	auto node_second = new NodeItem(area_triangle);
	area_triangle->setPos(_p1);
	node_first->setPos(_p1);
	node_second->setPos(_p1);
	QPolygonF poly;
	poly << QPointF(0,0) << QPointF(0,0);
	area_triangle->setPolygon(poly);
	area_triangle->node_list << node_first << node_second;
	area_triangle->transparency_count = 0;

	m_scene->addItem(area_triangle);	
	m_scene->m_triangle_shape = area_triangle;
	m_scene->add_cursor_label();     // show the coordinate
}

void EllipsClipper::move()
/* when the move mouse, cutting the circle */
{
	auto area_triangle = m_scene->m_triangle_shape;
	auto node_last = area_triangle->node_list.last();
	node_last->setPos(_p1);  // update the node

	/* update the polygon */
	auto pos_refer = area_triangle->node_list.value(0)->scenePos();  // reference point
	auto polygon = area_triangle->polygon();
	polygon.last() = node_last->scenePos() - pos_refer;
	area_triangle->setPolygon(polygon);
	area_triangle->pen_style = true;

	if (area_triangle->node_list.count() >= 3) // drawing the cutting circle
	{
		m_scene->DrawEmptyEllipse(area_triangle->node_list.value(0)->scenePos(), area_triangle->node_list.value(1)->scenePos(), _p1 );
		if (m_scene->m_is_merge_poly)
		{
			m_scene->ClipInitSet();  // initial clipping
			m_scene->ClipRegionMove(m_scene->m_shapes.last());
			m_scene->m_shapes.last()->pen_style = true;
		}
	}

	m_scene->move_cursor_label(_p1);   // move the coordinate label
}

void EllipsClipper::clip()
/* continue to cut circle */
{
	if (m_scene->m_triangle_shape->node_list.length() < 3)  // the second point of the circle
	{
		auto area_circle = new AreaItem(); // outer circle
		m_scene->addItem(area_circle);
		m_scene->m_shapes << area_circle;
		auto area_triangle = m_scene->m_triangle_shape;
		auto node_next = new NodeItem(m_scene->m_triangle_shape);
		node_next->setPos(_p1);
		m_scene->m_triangle_shape->node_list << node_next;
		auto polygon = area_triangle->polygon();
		polygon << polygon.last();
		m_scene->m_triangle_shape->setPolygon(polygon);
	}
	else  // end to cut the circle, the third point of the circle
	{
		m_scene->removeItem(m_scene->m_triangle_shape);
		auto _area = m_scene->m_shapes.last();
		if (m_scene->m_is_merge_poly) {
			m_scene->ClipInitSet();
			m_scene->ClipRegionPress(_area);
		}

		m_scene->DelArea(_area);
		m_scene->m_shapes.removeOne(_area);
		m_scene->m_isdrawing =false;
		m_scene->delete_cursor_label();
	}
}

void EllipsClipper::undo() {
}

/* when cutting the circle, the right mouse is pressed */
void EllipsClipper::right_click() {
	// if we are drawing the second point or third point
	if (m_scene->m_triangle_shape != nullptr) {
		// the third point, end drawing the cutting circle and clip
		if (m_scene->m_triangle_shape->node_list.length() == 3) {
			// _area is the circle, the area need to cut
			auto _area = m_scene->m_shapes.last();
			if (m_scene->m_is_merge_poly) {
				m_scene->ClipInitSet();
				m_scene->ClipRegionPress(_area);
			}
			m_scene->DelArea(_area);
			m_scene->m_shapes.removeOne(_area);
		}

		// remove inner triangle
		m_scene->removeItem(m_scene->m_triangle_shape);
		m_scene->m_triangle_shape = nullptr;

		// delete the coordinate label
		m_scene->delete_cursor_label();
	}
}