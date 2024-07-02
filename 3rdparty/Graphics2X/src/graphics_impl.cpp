#include <limits>
#include <QGraphicsPixmapItem>
#include "utils/editablegraphicsscene.h"
#include "graphics/graphics_fwds.h"

NAMESPACE_BEGIN(dgelom)
Graphics::Graphics(QObject* parent)
	:_Myimpl(new EditableGraphicsScene(parent))
{
	connect(_Myimpl, &EditableGraphicsScene::complete, this, &Graphics::completed);
	connect(_Myimpl, &EditableGraphicsScene::delete_item, this, &Graphics::_Update_buffers);
	connect(_Myimpl, &EditableGraphicsScene::deleted_point_item, this, &Graphics::removed_point);
}

void Graphics::set_point_style(point_style style) noexcept
{
	_Myimpl->m_ptstyle = decltype(_Myimpl->m_ptstyle)(style);
}

Graphics::command_t Graphics::exec(command_t cmd) noexcept
{
	_Myimpl->draw(GraphicEditorStatus(cmd));
	// Disable drawing status if the command is `add_point`
	_Mydrawing = cmd == command_t::add_point? false : true;
	return cmd;
}

void Graphics::remove(item_t item) noexcept
{
	switch (item)
	{
	case item_t::point:
		_Myimpl->draw_status() = GraphicEditCmd::DEL_NODE;
		break;
	case item_t::area:
		_Myimpl->draw_status() = GraphicEditCmd::DEL_SHAPE;
		break;
	case item_t::dylabel:
		_Myimpl->delete_cursor_label();
		break;
	case item_t::all_regions:
		_Myimpl->clear_all_regions();
		break;
	case item_t::all_items:
		_Myimpl->clear_all_regions();
		_Myimpl->clear_all_points();
		break;
	default:
		_Myimpl->draw_status() = GraphicEditCmd::DEL_SHAPE;
		break;
	}
}

void Graphics::clear_scene(bool only_stored_items)
{
	if (auto _Points = _Myimpl->points; !_Points.isEmpty()) {
		__points_buf__ = std::move(_Points);
	}
	if (auto _Regions = _Myimpl->regions; !_Regions.isEmpty()) {
		__regions_buf__ = std::move(_Regions);
	}

	remove(item_t::all_items);
	if (!only_stored_items) {
		scene()->clear();
	}
}

Graphics::point_list Graphics::points(item_t item) const noexcept
{
	return item==item_t::point?point_list():_Myimpl->points;
}

void Graphics::draw(const point_list& points)
{
	_Myimpl->DrawExternalStartPoints(points);
}

void Graphics::draw(const QPointF& point)
{
	_Myimpl->DrawExternalStartPoint(point);
}

Graphics::region_list Graphics::regions() const noexcept
{
	const auto ret = _Myimpl->regions;
	return ret.empty() ? __regions_buf__ : ret;
}

void Graphics::draw(const region_list& regions)
{
	return _Myimpl->DrawExternalAreas(regions);
}

void Graphics::draw(const point_list& points, const region_list& regions)
{
	if (points.empty()) {
		_Myimpl->DrawExternalStartPoints(__points_buf__);
	}
	else {
		_Myimpl->DrawExternalStartPoints(points);
	}
	if (regions.empty()) {
		_Myimpl->DrawExternalAreas(__regions_buf__);
	}
	else {
		_Myimpl->DrawExternalAreas(regions);
	}
}

size_t Graphics::add_temp_point(const QPointF& pos, int radius, QColor color)
{
	auto item = new QGraphicsEllipseItem(-radius, -radius, radius << 1, radius << 1);
	item->setPos(pos);
	color.setAlpha(180);
	item->setBrush(QBrush(color));
	item->setPen(Qt::NoPen);
	scene()->addItem(item);
	_Mytempdots.push_back(item);
	return _Mytempdots.size();
}

void Graphics::add_temp_line(const QPointF& start, const QPointF& end, QColor color)
{
	auto line = QLine(start.x(), start.y(), end.x(), end.y());
	_Mytemplines.push_back(new QGraphicsLineItem(line));
	_Mytemplines.back()->setPen({ color });
	scene()->addItem(_Mytemplines.back());
}

void Graphics::move_temp_point(int idx, const QPointF& pos)
{
	if (idx < _Mytempdots.size()) {
		_Mytempdots[idx]->setPos(pos);
		const auto items = scene()->items();
		auto max = *std::max_element(items.begin(), items.end(), [](auto pre, auto nxt) {
			return pre->zValue() > nxt->zValue();
			});
		_Mytempdots[idx]->setZValue(max->zValue() + 1);
	}
}

void Graphics::move_temp_line(int idx, const QPointF& start, const QPointF& end)
{
	if (idx < _Mytemplines.size()) {
		_Mytemplines[idx]->setLine(QLineF(start, end));
	}
}

void Graphics::add_temp_points(const std::vector<ColoredPointF>& pts, int radius)
{
	for (const auto& p : pts) {
		this->add_temp_point(p.point, radius, p.color);
	}
}

void Graphics::move_temp_points(const std::vector<ColoredPointF>& pts)
{
	for (auto _It = pts.begin(); _It != pts.end(); ++_It) {
		this->move_temp_point(_It - pts.begin(), _It->point);
	}
}

void Graphics::remove_temp_point(const QPointF& pos)
{
	decltype(auto) item = std::find_if(_Mytempdots.begin(), _Mytempdots.end(), 
		[=](auto p) {
			const auto _Diff = (p->pos() - pos);
			return std::abs(_Diff.x()) < 0.5 && std::abs(_Diff.y());
		});
	if (item != _Mytempdots.end()) {
		scene()->removeItem(*item);
		_Mytempdots.removeOne(*item);
	}
}

void Graphics::remove_temp_line(int idx)
{
	if (idx < _Mytemplines.size()) {
		const auto item = _Mytemplines[idx];
		scene()->removeItem(item);
		_Mytemplines.removeOne(item);
	}
}

void Graphics::clear_temp_items()
{
	while (!_Mytempdots.isEmpty()) {
		scene()->removeItem(_Mytempdots.last());
		_Mytempdots.removeLast();
	}
	while (!_Mytemplines.isEmpty()) {
		scene()->removeItem(_Mytemplines.last());
		_Mytemplines.removeLast();
	}
}

size_t Graphics::num_temp_points() const noexcept
{
	return _Mytempdots.size();
}

const QGraphicsEllipseItem* Graphics::temp_point(size_t idx) const noexcept
{
	return _Mytempdots[idx];
}

QGraphicsEllipseItem* Graphics::temp_point(size_t idx) noexcept
{
	return _Mytempdots[idx];
}

QRectF Graphics::scene_rect() const noexcept
{
	return _Myimpl->sceneRect();
}

void Graphics::reset_mouse_event() noexcept
{
	_Myimpl->draw_status() = GraphicEditCmd::READY;
}

bool& Graphics::drawing() const noexcept
{
	return _Mydrawing;
}

Graphics::command_t Graphics::status() const noexcept
{
	return command_t(_Myimpl->m_drawstatus);
}

void Graphics::_Update_buffers() noexcept
{
	__points_buf__ = _Myimpl->points;
	__regions_buf__ = _Myimpl->regions;
}

Graphics::operator scene_ptr() const noexcept
{
	return _Myimpl;
}

Graphics::operator scene_ptr() noexcept
{
	return _Myimpl;
}

EditableLiveImageScene::EditableLiveImageScene(QObject* parent)
	:Graphics(parent) 
{
}

EditableLiveImageScene::~EditableLiveImageScene()
{
	_Mypixmap = nullptr;
}

void EditableLiveImageScene::set_image(image_ptr image) noexcept
{
	_Myimprox = image;
	if (!_Mypixmap) {
		_Mypixmap = this->_Myimpl->set_pixmap(_Myimprox);
	}
	else {
		refresh_image();
	}
}

void EditableLiveImageScene::set_image(const QImage& image, bool added) noexcept
{
	if (!_Mypixmap || added == true) {
		_Mypixmap = this->_Myimpl->addPixmap(QPixmap::fromImage(image));
	}
	else {
		_Mypixmap->setPixmap(QPixmap::fromImage(image));
	}
}

void EditableLiveImageScene::refresh_image() noexcept
{
	if (_Mypixmap) {
		_Mypixmap->setPixmap(QPixmap::fromImage(*_Myimprox));
	}
}

void EditableSingleImageScene::show_tags(const QStringList& tags, int index, int offset) 
{
	if (tags.isEmpty() || !_Mytags.isEmpty()) {
		while (!_Mytags.isEmpty()) {
			scene()->removeItem(_Mytags.last());
			_Mytags.removeLast();
		}
	}
	if (!tags.isEmpty()) {
		if (index < 0) {
			for (auto i = 0; i < std::min(tags.size(), _Mytempdots.size()); ++i) {
				_Mytags.push_back(new QGraphicsTextItem(tags[i]));
				const auto item = _Mytempdots[i];
				const auto pos = item->pos();
				_Mytags.back()->setPos(pos.x()+offset, pos.y()+offset);
				auto color = item->brush().color();
				color.setAlpha(255);
				_Mytags.back()->setDefaultTextColor(color);
				scene()->addItem(_Mytags.back());
			}
		}
		else {
			const auto item = _Mytempdots[index];
			_Mytags.push_back(new QGraphicsTextItem(tags.first()));
			_Mytags.back()->setPos(item->pos().x()+offset, item->pos().y()+offset);
			auto color = item->brush().color();
			color.setAlpha(255);
			_Mytags.back()->setDefaultTextColor(color);
			scene()->addItem(_Mytags.back());
		}
	}
}

NAMESPACE_END(dgelom)