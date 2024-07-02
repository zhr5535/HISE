#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <memory>
#include "acquisition_worker.h"
#include "calculation_worker.h"
#include "graphics/graphics_fwds.h"

namespace Ui
{
    class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private:
    void show_error_msg(QString message, unsigned int error_num);

public slots:
    void on_bnEnum_clicked() { enum_devices(); };

    void on_bnOpenClose_clicked();

    void on_bnStartStop_clicked();

    void on_bnGetParam_clicked() { get_param(); };

    void on_bnSetParam_clicked() { set_param(); };

    void on_bnStartStopTracking_clicked();

    void on_bnPauseResumeTracking_clicked();

    void on_bnForwardTracking_clicked() { forward_tracking(); };

    void on_bnZoomIn_clicked();

    void on_bnZoomOut_clicked();

    void update_live_scene(const QImage &image);

protected:
    bool eventFilter(QObject *obj, QEvent *event) override;    

private:
    void enum_devices();
    void open_camera();
    void close_camera();
    void start_acquisition();
    void stop_acquisition();
    void get_param();
    void set_param();
    void start_tracking();
    void stop_tracking();
    void pause_tracking();
    void resume_tracking();
    void forward_tracking();

private:
    Ui::MainWindow *ui;

    MV_CC_DEVICE_INFO_LIST m_device_list; // 枚举到的相机列表，包含相机信息

    dgelom::EditableLiveImageScene *m_live_scene; // graphicView中的scene,显示相机实时画面

    HikCamera *m_camera; // 当前选择的相机

    acquisition_worker *m_acqworker; // 采集线程

    std::unique_ptr<calculation_worker> m_calworker; // 计算线程

    bool m_has_autofit; // 是否已经自适应窗口大小
};

#endif // MAINWINDOW_H
