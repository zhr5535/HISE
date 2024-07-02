/**********************************************************************
 * Copyright (C) 2021-2023, Zhilong (Dgelom) Su, All Rights Reserved.
 *
 * This software is distributed WITHOUT ANY WARRANTY; without even
 * the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE.
 *********************************************************************/
#pragma once
#include <QList>
#include <QPoint>
#include <QColor>
#include <QObject>

#include "internal/configs.h"

/// <summary>
/// \brief Forward declarations
/// </summary>
class QGraphicsScene;
class QGraphicsLineItem;
class QGraphicsTextItem;
class QGraphicsPixmapItem;
class QGraphicsEllipseItem;
class EditableGraphicsScene;

NAMESPACE_BEGIN(dgelom)
class Graphics : public QObject
{
    Q_OBJECT;
public:
    using scene_ptr = std::add_pointer_t<EditableGraphicsScene>;
    using point_list = QList<QPointF>;
    using region_list = QList<point_list>;

    enum command_t : uint8_t
    {
        waiting = 0,
        draw_line = 12,
        draw_rect = 1,
        draw_circ = 2,
        draw_poly = 3,
        add_point = 4,
        clip_rect = 5,
        clip_circ = 6,
        clip_poly = 7,
        auto_seed = 8,

        delete_point = 10,
        delete_label = 11,

        move_region = 14,
    };
    enum class item_t : uint8_t
    {
        any,
        point,
        seed,
        area,
        dylabel,
        all_regions,
        all_items,
    };
    enum class point_style : uint8_t {
        target_cross = 0,
        framed_cross = 1,
        filled_circl = 2,
        hollow_circl = 3,
        pinned_point = 4,
    };
    struct ColoredPointF {
        QPointF point;
        QColor color;
    };

    Graphics(QObject* parent = nullptr);
    ~Graphics() noexcept = default;

    /**
     * \brief Get graphics scene with a given type. 
     */
    template<typename Ptr = QGraphicsScene*>
    Ptr scene() noexcept {
        return Ptr(operator scene_ptr());
    }

    /**
     * \brief Set the shape style for scatter or seed point(s)
     */
    void set_point_style(point_style style) noexcept;

    /**
     * \brief Execute a graphics command supported in "command_t" 
       by binding the command to the left-click event of the mice.
     * \param 'cmd' Graphics command from command_t.
     */
    command_t exec(command_t cmd) noexcept;

    /**
     * \brief Delete a given graphics item from the scene. Default is 
     * \param 'item' Any value defined in item_t.
     */
    void remove(item_t item = item_t::any) noexcept;

    /**
     * \brief Clear the scene by removing all stored items and temporal items.
     * Call this method, the removed point and region items will be stored in two buffers,
     * allowing to call method `draw(const point_list&, const region_list&)` by giving two
     * empty arguments to restored the removed items.
     */
    void clear_scene(bool only_stored_items = true);

    /**
     * \brief Get all individual points.
     */
    point_list points(item_t item=item_t::seed) const noexcept;

    /**
     * \brief Get all closed shapes.
     */
    region_list regions() const noexcept;

    /**
     * \brief Draw with user defined points
     */
    void draw(const point_list& points);
    void draw(const QPointF& point);

    /**
     * \brief Draw with user defined polygons
     */
    void draw(const region_list& regions);

    /**
     * \brief Draw points and regions togethor. Default is to draw the cached points and regions.
     */
    void draw(const point_list& points = {}, const region_list& regions = {});

    /**
     * \brief Get the rect range of the graphics scene.
     */
    QRectF scene_rect() const noexcept;

    /**
     * \brief Free mouse event to make it stays in the default.
     */
    void reset_mouse_event() noexcept;

    /**
     * \brief Query the drawing status of background event.
     */
    bool& drawing() const noexcept;

    /**
     * \brief Get the current drawing status before right click.
     */
    command_t status() const noexcept;

    /// </ Following methods for operating temporal items
    size_t add_temp_point(const QPointF& pos, int radius = 5, QColor color = { 255,0,0 });
    void add_temp_line(const QPointF& start, const QPointF& end, QColor color = { 255,0,0 });
    void move_temp_point(int idx, const QPointF& pos);
    void move_temp_line(int idx, const QPointF& start, const QPointF& end);
    void add_temp_points(const std::vector<ColoredPointF>& pts, int radius = 5);
    void move_temp_points(const std::vector<ColoredPointF>& pts);
    void remove_temp_point(const QPointF& pos);
    void remove_temp_line(int idx);
    void clear_temp_items();
    size_t num_temp_points() const noexcept;
    const QGraphicsEllipseItem* temp_point(size_t idx) const noexcept;
    QGraphicsEllipseItem* temp_point(size_t idx) noexcept;

signals:
    // Emitted once the current command is executed.
    void completed();
    void removed_point(size_t idx);

protected:
    operator scene_ptr() const noexcept;
    operator scene_ptr() noexcept;

    scene_ptr _Myimpl;
    mutable bool _Mydrawing{ false };

private:
    void _Update_buffers() noexcept;
    point_list __points_buf__;
    region_list __regions_buf__;

protected:
    QList<QGraphicsEllipseItem*> _Mytempdots;
    QList<QGraphicsLineItem*> _Mytemplines;
};

/// <summary>
/// CLASS, Grapihcs scene for displaying live image data and drawing graphic items on the image.
/// </summary>
class EditableLiveImageScene : public Graphics 
{
    Q_OBJECT;
public:
    using image_ptr = std::add_pointer_t<QImage>;
    using pixmap_ptr = std::add_pointer_t<QGraphicsPixmapItem>;

    EditableLiveImageScene(QObject* parent = nullptr);
    ~EditableLiveImageScene();
    
    /**
     * \brief Fill the scene with a pointer to QImage object.
     * This allows for updating the scene image efficiently.
     */
    void set_image(image_ptr image) noexcept;
    void set_image(const QImage& image, bool added=false) noexcept;

public slots:
    /**
     * \brief SLOT, update the pixmap filled by 'set_pixmap' in previous.
     * This allows for showing live video or online image sequence.
     */
    void refresh_image() noexcept;

signals:
    void image_received();

private:
    // Proxy, do not take the owership, #sa set_image().
    image_ptr _Myimprox{ nullptr };

    // Pointer to the global scene pixmap, #sa refresh_image().
    pixmap_ptr _Mypixmap{ nullptr };
};

/// <summary>
/// CLASS, derived from class `EditableLiveImageScene`
/// </summary>
class EditableSingleImageScene : public EditableLiveImageScene
{
    using _Mybase = EditableLiveImageScene;
public:
    EditableSingleImageScene(QObject* parent = nullptr) noexcept
        :EditableLiveImageScene(parent) {
    }

    void set_image(const QImage& image) noexcept {
        _Mybase::set_image(image);
    }
    void add_image(const QImage& image) noexcept {
        _Mybase::set_image(image, true);
    }

    /**
     * \brief Show tags for the temporal items. The existing tag item(s)
     * will be removed if the tag list `tags` is empty.
     * \param `tags` List of tag text to be shown.
     * \param `index` Position of the cached temporal items to be tagged. 
                      Default, -1, indicates to tag all temporal items.
     * \param `color` Color of the tag(s).
     */
    void show_tags(const QStringList& tags, int index = -1, int offset=8);

private:
    void set_image(image_ptr image) noexcept = delete;
    void refresh_image() noexcept = delete;
    void image_received() = delete;

    QList<QGraphicsTextItem*> _Mytags;
};

NAMESPACE_END(dgelom)