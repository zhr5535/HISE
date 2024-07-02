#pragma once

#include <QPointF>
#include <QStack>
#include <QPainterPath>

#include <cmath>

#include "fwd_types.h"
#include "property.hpp"
#include "wheelscalablegraphicsscene.h"

using QGrSME = QGraphicsSceneMouseEvent;
template<class _Ty>
using QList2D = QList<QList<_Ty>>;
using Rgn2D = QList<QPointF>;
class AreaItem;
class NodeItem;
class PointItem;
class CursorLabel;

struct RegionLinePt {
	int flag;
	double x;
	double y;
	QString marker;

	struct RegionLinePt* next;
};
using ClipRegion = RegionLinePt;

class EditableGraphicsScene : public WheelScalableGraphicsScene
{
	Q_OBJECT;
	using _Mybase = WheelScalableGraphicsScene;
	using _Myt = EditableGraphicsScene;
	using _MyStatus = GraphicEditorStatus;
	struct _Equation_params { double a, b, c; };

	static constexpr auto _Mycursor_shape = Qt::CursorShape::PointingHandCursor;
public:
	using status_type = _MyStatus;
	using pointitem_list = QList<PointItem*>;

	EditableGraphicsScene(QObject* parent = nullptr);
	~EditableGraphicsScene();

	void add_cursor_label();

	void delete_cursor_label();

	void move_cursor_label(QPointF pos);

	/**
	 * \brief Checks if there are any items in the scene.
	 */
	bool empty() const noexcept;

	QRectF get_fit_view_rect();

	AreaItem* cursor_captured_shape(QPointF pos);

	PointItem* cursor_captured_point(QPointF pos);

	NodeItem* cursor_captured_node(QPointF pos);

	/**
	 * \brief Move the idx-th point with shifts dx and dy.
	 */
	void move_point(size_t idx, float_t dx, float_t dy) noexcept;

	/**
	 * \brief Move the idx-th point to [x, y]
	 */
	void move_point_to(size_t idx, float_t x, float_t y) noexcept;

	/**
	 * \brief Set draw drawing status for mice click response.
	 */
	inline _MyStatus draw(_MyStatus status) noexcept {
		return m_drawstatus = status;
	}

	/**
	 * \brief Set draw drawing status for mice clicked response.
	 */
	inline const _MyStatus& draw_status()const noexcept {
		return m_drawstatus;
	}
	inline _MyStatus& draw_status()noexcept {
		return m_drawstatus;
	}

	/**
	 * \brief Draw a free point with a shape which is given by QPainterPath.
	 */
	void draw_free_point(const QPainterPath& shape = {})noexcept;

	/**
	 * \brief Draw a single line with two given end-points.
	 */
	void draw_single_line(QPointF start, QPointF end) noexcept;

	/**
	 * \brief Check if the 'pos' located in the scene rect.
	 */
	bool in_scene_rect(QPointF pos) noexcept;

	/**
	 * \brief Clear all shapes in the scene.
	 */
	void clear_all_regions() noexcept;

	/**
	 * \brief Fill the scene pixmap with a pointer to QImage object.
	 * This allows for updating the scene image efficiently.
	 */
	QGraphicsPixmapItem* set_pixmap(QImage* ptr) noexcept;

	/**
	 * \brief Collect and return positions of scatter points.
	 */
	const pointitem_list& get_points() const noexcept;
	pointitem_list& get_points() noexcept;

public: //Following methods will be deprecated
	QList<QPointF> ClipAnyRegion(AreaItem* areaMain, AreaItem* areaClip,bool clipType);//clipType==true 裁减区域按顺时针排列

	QList<QPointF> AddAnyRegion(AreaItem* areaMain, AreaItem* areaClip,bool clipType);//clipType==true 添加区域按顺时针排列

	QList<QPointF> NodeCrossRegion(AreaItem* area, NodeItem* nodegrabbed);//拖动node,与选区其他边线产生交点

	double CalArea(AreaItem* area);

	void ConvertAreaNodeSort(AreaItem* area);

	bool IsIntersect(QPointF P11, QPointF P12, QPointF P21, QPointF P22, QPointF P);//两条线段是否相交

	bool IsInRegion(double x, double y, AreaItem* area);

	bool IsInLine(double x, double y, double x1,double y1,double x2,double y2);

	bool CalLineJoint(double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4, double *xx, double *yy);

	int  ClipResultNum(QList<QPointF> qList);

	void AddRegionPress(AreaItem* _area);

	void AddRegionMove(AreaItem* _area);

	void ClipRegionPress(AreaItem* _area);

	void ClipRegionMove(AreaItem* _area);

	void DelArea(AreaItem* _area);

	void AddInitSet();

	void ClipInitSet();

	void RefreshAreas();

	bool IsContainArea(AreaItem* _area1, AreaItem* _area2);//判断area1是否包含area2

	bool AreaContainArea(AreaItem* set, AreaItem* subset);

	bool AreaIntersectArea(AreaItem* set1, AreaItem* set2);

	void DrawEllipse(QPointF pt1, QPointF pt2, QPointF pt3);

	void DrawEmptyEllipse(QPointF pt1, QPointF pt2, QPointF pt3);

	void GetVertDeuceLine(QPointF pt1, QPointF pt2, _Equation_params& para);

	bool GetIntersectionPoint(_Equation_params para1, _Equation_params para2, QPointF &center);

	double GetDistance(long x1, long y1, long x2, long y2);

	QPointF GetStartPoint();

	QPolygonF GetAreaToPolygon(AreaItem* area);//获取区域的多边形

	AreaItem* CopyArea(AreaItem* _tempArea);

	/*return a selection copy from _other region*/
	AreaItem* cpyFrom(const AreaItem* _other);

	AreaItem* PListToArea(QList<QPointF> pList);

	AreaItem* PListToNoAddArea(QList<QPointF> pList);//由点绘制1个区域

	QVector<AreaItem*> PListToMulitArea(QList<QPointF> pList);//点的序列到多个区域

	QVector<AreaItem*> AreaToMulitArea(AreaItem* area);//单个区域到数组区域

	QList<QPolygonF*> PListToPolygons(QList<QPointF> pointList);

	QList<QPointF> PolygonsToPList(QList<QPolygonF*> polygons, bool isCross);

	void PListToNormalize(QList<QPointF>& pointList);

	void PolygonToCorrectOrder(QList<QPolygonF*>& polygons);

	void AreaLastSel();

	void DrawExternalArea(QList<QPointF> pList);

	void DrawExternalStartPoint(QPointF p);

	void clear_all_points();

	bool DeleteSeed(QPointF point);

	bool DeleteNode(QPointF point);

	QVector<AreaItem*> AreaDeleteNodeToAreas(AreaItem* area, NodeItem* node);

	QVector<AreaItem*> AreaToAreas(AreaItem* area);

	QPointF CorrectPointInSceneRect(QPointF point);

	QList2D<QPointF> get_areas() const noexcept;

	QList<QPointF> CollectStartPoints();

	/**
	 *\brief Change the area to a list of point.
	 *\note The area should not contain inner areas.
	 */
	QList<QPointF> AreaToPointList(AreaItem* area);

	void DrawExternalStartPoints(QList<QPointF> _pList);

	void ClearLastArea();

	void InOut(float rate);

	void DrawExternalAreas(QList<QList<QPointF>> _pList);

	int posAreaType(QPointF pos);

	int rightButtonAreaType(QPointF* pos);

	void ReentrantPolygonClipping(QPolygonF& oldPoly, QRectF rect, QPolygonF& newPoly);

	QSet<PointItem*> seedInArea(AreaItem* area);

	/*properties*/
public:
	PROPERTY(int, Transparency);
	__set__(Transparency) { m_transparency = Transparency; }
	__get__(Transparency) { return m_transparency; }

	READONLY_PROPERTY(QList<Rgn2D>, regions);
	__get__(regions) { return _Gather_selections(); }

	READONLY_PROPERTY(QList<QPointF>, points);
	__get__(points) { return _Gather_points(); }

private:
	void mousePressEvent(QGrSME *e);
	void mouseMoveEvent(QGrSME *e);
	void mouseReleaseEvent(QGrSME * e);
	void mouseDoubleClickEvent(QGrSME* e);
	void keyPressEvent(QKeyEvent *event);

	void setAreaToNode(QPointF point);
	void GetLeftX(QPointF p, double& lp);
	void GetLeftY(QPointF p, double& lp);
	void GetRightX(QPointF p, double& lp);
	void GetRightY(QPointF p, double& lp);

	auto _Get_roi_corners(QPointF point, std::initializer_list<double> cn)const noexcept;
	QList<Rgn2D> _Gather_selections();
	QList<QPointF> _Gather_points();

signals:
	void cursor_shape(Qt::CursorShape);
	void complete(status_type);
	void delete_item(uint8_t);
	void deleted_point_item(size_t); //Report which point item is removed
	void mouse_tracking();
	void right_button_drawing();
	void SignalSSSIGPointPicked(QPointF); // the user has picked the SSSIG Point

private slots:
	void SlotPickSSSIGPoint();

public:
	int m_selectedregions;
	int m_grayvalue;
	bool m_selshapekg;
	bool m_is_merge_poly;
	bool m_handcursor;
	bool m_isdrawing;
	bool m_is_cursor_info_visible;
	bool m_is_seed_selected;
	bool m_is_clipping;   /* if is cutting, then m_is_clipping = true */

	PointItem* m_translate_seed = nullptr;
	QPointF m_last_seed_pos;
	QPointF m_eventpos;
	CursorLabel* m_horcursor_info = nullptr;
	CursorLabel* m_vercursor_info = nullptr;
	QList<AreaItem*> m_shapes;
	QList<AreaItem*> m_shapes_redo;
	QList<PointItem*> m_points;
	AreaItem* m_temp_shape = nullptr;
	AreaItem* m_triangle_shape = nullptr;
	QList<NodeItem*> m_inner_nodes;
	QVector<AreaItem*> m_temp_shapes;

	_MyStatus m_drawstatus;
	_MyStatus m_drawstatus_before_rightclick;
	PointStyle m_ptstyle{ PointStyle::FRAMED_CROSS };
private:
	int m_transparency;
	bool m_is_addpoint;

	QPointF m_last_pressedpos;
	_MyStatus m_last_drawstatus;
};

using GraphicsSceneClientEditor = EditableGraphicsScene;
/*Graphics Scene Client Editor for the ROIs or selections*/
using GrSCE = GraphicsSceneClientEditor;