#include "primitives/ellips.h"
#include "utils/regionitem.h"
#include "utils/nodeitem.h"
#include "utils/editablegraphicsscene.h"

void EllipsPainter::exec()
/* initialize drawing circle */
{
	/* if the area_triangle exist, delete it */
	if (m_scene->m_triangle_shape != nullptr) {
		m_scene->removeItem(m_scene->m_triangle_shape);
		m_scene->m_triangle_shape = nullptr;
	}

	/* initialize the inner triangle and nodes */
	auto area_triangle = new AreaItem(); // inner triangle
	auto node_first = new NodeItem(area_triangle);  // nodes of the triangle
	auto node_second = new NodeItem(area_triangle);
	area_triangle->setPos(_p1);
	node_first->setPos(_p1);
	node_second->setPos(_p1);

	QPolygonF polygon;
	polygon << QPointF(0,0) << QPointF(0,0);
	area_triangle->setPolygon(polygon);
	area_triangle->node_list << node_first << node_second;
	m_scene->m_triangle_shape = area_triangle;
	m_scene->addItem(area_triangle);
	m_scene->add_cursor_label();     // show the coordinate

}

void EllipsPainter::move()
/* drawing the circle, when the mouse move */
{
	auto area_triangle = m_scene->m_triangle_shape;
	if (!area_triangle || area_triangle->node_list.empty()) return;
	auto node_last = area_triangle->node_list.last();

	node_last->setPos(_p1);

	auto pos_refer = area_triangle->node_list.value(0)->scenePos();
	auto polygon = area_triangle->polygon();
	if (polygon.empty()) return;
	polygon.last() = node_last->scenePos() - pos_refer;
	area_triangle->pen_style = true;
	area_triangle->setPolygon(polygon);
	if (area_triangle->node_list.count() >= 3)   // drawing the circle
	{
		m_scene->DrawEllipse(area_triangle->node_list.value(0)->scenePos(), 
			area_triangle->node_list.value(1)->scenePos(), _p1 ); 
		if (m_scene->m_is_merge_poly)  // merge
		{
			m_scene->m_selectedregions = m_scene->m_shapes.length()-1;
			m_scene->AddInitSet();
			m_scene->AddRegionMove(m_scene->m_shapes.last());
		}
	}
	m_scene->move_cursor_label(_p1); // move the coordinate label
}

void EllipsPainter::merge()
/* left mouse pressed when drawing circle */
{
	if (!m_scene->m_triangle_shape || m_scene->m_triangle_shape->node_list.empty()) {
		return;
	}
	if (m_scene->m_triangle_shape->node_list.length() < 3) // drawing the second point of the circle	
	{  
		_ellips_area = new AreaItem(); 
		m_scene->addItem(_ellips_area);   // add the circle to the scene
		m_scene->m_shapes << _ellips_area;
		m_scene->AreaLastSel();

		auto area_triangle = m_scene->m_triangle_shape;
		auto node_next = new NodeItem(m_scene->m_triangle_shape);
		node_next->setPos(_p1);
		m_scene->m_triangle_shape->node_list << node_next;
		auto poly = area_triangle->polygon();
		poly << poly.last();
		m_scene->m_triangle_shape->setPolygon(poly);
	} else // drawing the third point of the circle
	{ 
		m_scene->removeItem(m_scene->m_triangle_shape);
		if (m_scene->m_is_merge_poly) {
			m_scene->AddInitSet();
			m_scene->AddRegionPress(m_scene->m_shapes.last());
		}
		m_scene->m_shapes.last()->SetNodesVisible(true);
		m_scene->m_isdrawing =false;
		m_scene->delete_cursor_label();  // delete the coordinate label

	}
}

void EllipsPainter::undo()
{
	//m_scene->removeItem(_ellips_area);
	//for(int i = 0; i<_ellips_area->node_list.length() ;i++)
	//{
	//	m_scene->removeItem(_ellips_area->node_list.value(i));
	//}
	//m_scene->m_shapes.removeOne(_ellips_area);
}

void EllipsPainter::redo()
{
	/*m_scene->addItem(_ellips_area);
	for(int i = 0; i<_ellips_area->node_list.length() ;i++)
	{
		m_scene->addItem(_ellips_area->node_list.value(i));
	}
	m_scene->m_shapes<<_ellips_area;*/
}

void EllipsPainter::right_click()
/* when drawing the circle, we press the right button */
{
	if (m_scene->m_triangle_shape != nullptr)  // if we are drawing the second point or third point
	{
		int num = m_scene->m_triangle_shape->node_list.length();
		if (num == 3) // the third point, end drawing
		{
			if (m_scene->m_is_merge_poly) // merge
			{
				m_scene->AddInitSet();
				m_scene->AddRegionPress(m_scene->m_shapes.last());
			}
		}

		/* delete the inner triangle and its nodes */
		m_scene->removeItem(m_scene->m_triangle_shape);
		m_scene->m_triangle_shape = nullptr;

		m_scene->delete_cursor_label();  // delete the coordinate label
	}
}
