#include "actions/shapedeleter.h"
#include "utils/regionitem.h"

void DeleteAreaCommand::exec()
/* delete the area if point in area */
{
	foreach(AreaItem* area, m_scene->m_shapes) 
	{
		if(area->PointInArea(area->mapFromScene(_p1)))
		{
			m_scene->DelArea(area);
			m_scene->m_shapes.removeOne(area);
		}
	}
}

void DeleteAreaCommand::undo()
{

}