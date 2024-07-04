#include "mainwindow.h"
#include "graphics/scalview2d.h"
#include "ui_mainwindow.h"
#include <QMessageBox>
#include <QStyleFactory>
#include <QTextCodec>
#include <QEvent>
#include <QMouseEvent>
#include <QWheelEvent>
#include <QCoreApplication>
#include <iostream>

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent),
                                          ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    m_live_scene = nullptr;
    memset(&m_device_list, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
    m_camera = nullptr;
    m_acqworker = nullptr;
    m_has_autofit = false;

    // ui->ComboDevices->show();
    // ui->bnEnum->show();
    ui->bnOpenClose->setEnabled(false);
    ui->bnStartStop->hide();
    ui->bnStartStopTracking->hide();
    ui->bnPauseResumeTracking->hide();
    ui->bnForwardTracking->hide();

    ui->lExposure->hide();
    ui->tbExposure->hide();
    ui->lGain->hide();
    ui->tbGain->hide();
    ui->lFrameRate->hide();
    ui->tbFrameRate->hide();
}

MainWindow::~MainWindow()
{
    if (m_live_scene)
        delete m_live_scene;
    if (m_acqworker)
        delete m_acqworker;
    if (m_camera)
        delete m_camera;
    delete ui;
}

// 显示错误信息
void MainWindow::show_error_msg(QString message, unsigned int error_num)
{
    QString error_msg = message;
    if (error_num != 0)
    {
        QString temp_msg;
        temp_msg.sprintf(": Error = %x: ", error_num);
        error_msg += temp_msg;
    }

    switch (error_num)
    {
    case MV_E_HANDLE:
        error_msg += "Error or invalid handle ";
        break;
    case MV_E_SUPPORT:
        error_msg += "Not supported function ";
        break;
    case MV_E_BUFOVER:
        error_msg += "Cache is full ";
        break;
    case MV_E_CALLORDER:
        error_msg += "Function calling order error ";
        break;
    case MV_E_PARAMETER:
        error_msg += "Incorrect parameter ";
        break;
    case MV_E_RESOURCE:
        error_msg += "Applying resource failed ";
        break;
    case MV_E_NODATA:
        error_msg += "No data ";
        break;
    case MV_E_PRECONDITION:
        error_msg += "Precondition error, or running environment changed ";
        break;
    case MV_E_VERSION:
        error_msg += "Version mismatches ";
        break;
    case MV_E_NOENOUGH_BUF:
        error_msg += "Insufficient memory ";
        break;
    case MV_E_ABNORMAL_IMAGE:
        error_msg += "Abnormal image, maybe incomplete image because of lost packet ";
        break;
    case MV_E_UNKNOW:
        error_msg += "Unknown error ";
        break;
    case MV_E_GC_GENERIC:
        error_msg += "General error ";
        break;
    case MV_E_GC_ACCESS:
        error_msg += "Node accessing condition error ";
        break;
    case MV_E_ACCESS_DENIED:
        error_msg += "No permission ";
        break;
    case MV_E_BUSY:
        error_msg += "Device is busy, or network disconnected ";
        break;
    case MV_E_NETER:
        error_msg += "Network error ";
        break;
    }

    QMessageBox::information(nullptr, "PROMPT", error_msg);
}

void MainWindow::on_bnOpenClose_clicked()
{
    if (ui->bnOpenClose->text() == "Open")
    {
        open_camera();
        ui->bnOpenClose->setText("Close");
        ui->bnStartStop->setText("Start");
        ui->bnStartStopTracking->setText("Track");
        ui->bnPauseResumeTracking->setText("Pause");
    }
    else if (ui->bnOpenClose->text() == "Close")
    {
        close_camera();
        ui->bnOpenClose->setText("Open");
    }
}

void MainWindow::on_bnStartStop_clicked()
{
    if (ui->bnStartStop->text() == "Start")
    {
        start_acquisition();
        ui->bnStartStop->setText("Stop");
    }
    else if (ui->bnStartStop->text() == "Stop")
    {
        stop_acquisition();
        ui->bnStartStop->setText("Start");
    }
}

void MainWindow::on_bnStartStopTracking_clicked()
{
    if (ui->bnStartStopTracking->text() == "Track")
    {
        start_tracking();
        ui->bnStartStopTracking->setText("End");
    }
    else if (ui->bnStartStopTracking->text() == "End")
    {
        stop_tracking();
        ui->bnStartStopTracking->setText("Track");
    }
}

void MainWindow::on_bnPauseResumeTracking_clicked()
{
    if (ui->bnPauseResumeTracking->text() == "Pause")
    {
        pause_tracking();
        ui->bnPauseResumeTracking->setText("Resume");
    }
    else if (ui->bnPauseResumeTracking->text() == "Resume")
    {
        resume_tracking();
        ui->bnPauseResumeTracking->setText("Pause");
    }
}

void MainWindow::on_bnZoomIn_clicked()
{
    ui->graphicsView->zoom_slider()->setValue(ui->graphicsView->zoom_slider()->value() + 1);
}

void MainWindow::on_bnZoomOut_clicked()
{
    ui->graphicsView->zoom_slider()->setValue(ui->graphicsView->zoom_slider()->value() - 1);
}

// ch：枚举设备 en:Enumerate devices
void MainWindow::enum_devices()
{
    ui->ComboDevices->clear();
    QTextCodec::setCodecForLocale(QTextCodec::codecForName("GBK"));
    ui->ComboDevices->setStyle(QStyleFactory::create("Windows"));
    // en:Enumerate all devices within subnet
    // ch：枚举子网内所有设备
    memset(&m_device_list, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
    int nRet = HikCamera::enum_devices(&m_device_list);
    if (MV_OK != nRet)
    {
        return;
    }

    // en:Add value to the information list box and display
    // ch：将信息添加到信息列表框中并显示
    for (unsigned int i = 0; i < m_device_list.nDeviceNum; i++)
    {

        MV_CC_DEVICE_INFO *device_info = m_device_list.pDeviceInfo[i];
        if (nullptr == device_info)
        {
            continue;
        }
        char user_name[256] = {0};
        if (device_info->nTLayerType == MV_GIGE_DEVICE)
        {
            int nIp1 = ((m_device_list.pDeviceInfo[i]->SpecialInfo.stGigEInfo.nCurrentIp & 0xff000000) >> 24);
            int nIp2 = ((m_device_list.pDeviceInfo[i]->SpecialInfo.stGigEInfo.nCurrentIp & 0x00ff0000) >> 16);
            int nIp3 = ((m_device_list.pDeviceInfo[i]->SpecialInfo.stGigEInfo.nCurrentIp & 0x0000ff00) >> 8);
            int nIp4 = (m_device_list.pDeviceInfo[i]->SpecialInfo.stGigEInfo.nCurrentIp & 0x000000ff);

            if (strcmp("", (char *)device_info->SpecialInfo.stGigEInfo.chUserDefinedName) != 0)
            {
                snprintf(user_name, 256, "[%d]GigE:   %s (%s) (%d.%d.%d.%d)", i, device_info->SpecialInfo.stGigEInfo.chUserDefinedName,
                         device_info->SpecialInfo.stGigEInfo.chSerialNumber, nIp1, nIp2, nIp3, nIp4);
            }
            else
            {
                snprintf(user_name, 256, "[%d]GigE:   %s (%s) (%d.%d.%d.%d)", i, device_info->SpecialInfo.stGigEInfo.chModelName,
                         device_info->SpecialInfo.stGigEInfo.chSerialNumber, nIp1, nIp2, nIp3, nIp4);
            }
        }
        else if (device_info->nTLayerType == MV_USB_DEVICE)
        {
            if (strcmp("", (char *)device_info->SpecialInfo.stUsb3VInfo.chUserDefinedName) != 0)
            {
                snprintf(user_name, 256, "[%d]UsbV3:  %s (%s)", i, device_info->SpecialInfo.stUsb3VInfo.chUserDefinedName,
                         device_info->SpecialInfo.stUsb3VInfo.chSerialNumber);
            }
            else
            {
                snprintf(user_name, 256, "[%d]UsbV3:  %s (%s)", i, device_info->SpecialInfo.stUsb3VInfo.chModelName,
                         device_info->SpecialInfo.stUsb3VInfo.chSerialNumber);
            }
        }
        else
        {
            show_error_msg("Unknown device enumerated", 0);
        }
        ui->ComboDevices->addItem(QString::fromLocal8Bit(user_name));
    }

    if (0 == m_device_list.nDeviceNum)
    {
        show_error_msg("No device", 0);
        return;
    }
    ui->ComboDevices->setCurrentIndex(0);
    ui->bnOpenClose->setEnabled(true);
}

// ch：打开设备 en:Open device
void MainWindow::open_camera()
{
    int nIndex = ui->ComboDevices->currentIndex();
    if ((nIndex < 0) | (nIndex >= MV_MAX_DEVICE_NUM))
    {
        show_error_msg("Please select device", 0);
        return;
    }

    // en:Device instance created by device information
    //  ch：根据设备信息创建设备实例
    if (nullptr == m_device_list.pDeviceInfo[nIndex])
    {
        show_error_msg("Device does not exist", 0);
        return;
    }

    if (m_camera == nullptr)
    {
        m_camera = new HikCamera;
        if (nullptr == m_camera)
        {
            return;
        }
    }

    // ch:创建实时图像显示场景 en:Create real-time image display scene
    if (m_live_scene == nullptr)
    {
        m_live_scene = new dgelom::EditableLiveImageScene(nullptr);
        ui->graphicsView->setScene(m_live_scene->scene());
        m_live_scene->scene()->setSceneRect(0, 0, m_camera->outframe().stFrameInfo.nWidth, m_camera->outframe().stFrameInfo.nHeight);
        ui->graphicsView->installEventFilter(this); // 安装事件过滤器
    }
    m_has_autofit = false;

    int nRet = m_camera->open(m_device_list.pDeviceInfo[nIndex]);
    if (MV_OK != nRet)
    {
        delete m_camera;
        m_camera = nullptr;
        show_error_msg("Open Fail", nRet);
        return;
    }

    //  en:Detection network optimal package size(It only works for the GigE camera)m_device_list.pDeviceInfo[nIndex]->nTLayerType == MV_GIGE_DEVICE)
    //  ch：检测网络最佳包大小(仅适用于GigE相机)
    if (m_device_list.pDeviceInfo[nIndex]->nTLayerType == MV_GIGE_DEVICE)
    {
        unsigned int nPacketSize = 0;
        nRet = m_camera->get_optimal_packet_size(&nPacketSize);
        if (nRet == MV_OK)
        {
            nRet = m_camera->set_int_value("GevSCPSPacketSize", nPacketSize);
            if (nRet != MV_OK)
            {
                show_error_msg("Warning: Set Packet Size fail!", nRet);
            }
        }
        else
        {
            show_error_msg("Warning: Get Packet Size fail!", nRet);
        }
    }

    m_camera->set_enum_value("AcquisitionMode", MV_ACQ_MODE_CONTINUOUS);
    m_camera->set_enum_value("TriggerMode", MV_TRIGGER_MODE_OFF);

    get_param(); // en:Get Parameter ch：获取参数

    ui->ComboDevices->hide();
    ui->bnEnum->hide();
    // ui->bnOpenClose->hide();
    ui->bnStartStop->show();
    ui->bnStartStopTracking->show();
    ui->bnPauseResumeTracking->show();
    ui->bnForwardTracking->show();

    ui->lExposure->show();
    ui->tbExposure->show();
    ui->lGain->show();
    ui->tbGain->show();
    ui->lFrameRate->show();
    ui->tbFrameRate->show();

    // ui->bnOpenClose->setEnabled(false);
    ui->bnStartStop->setEnabled(true);
    ui->bnStartStopTracking->setEnabled(false);
    ui->bnPauseResumeTracking->setEnabled(false);
    ui->bnForwardTracking->setEnabled(false);

    ui->tbExposure->setEnabled(true);
    ui->tbGain->setEnabled(true);
    ui->tbFrameRate->setEnabled(true);
    ui->bnSetParam->setEnabled(true);
    ui->bnGetParam->setEnabled(true);
}

// ch：关闭设备 en:Close device
void MainWindow::close_camera()
{
    if (m_camera)
    {
        m_camera->close();
        if (m_live_scene)
        {
            delete m_live_scene;
            m_live_scene = nullptr;
        }
        if (m_acqworker)
        {
            delete m_acqworker;
            m_acqworker = nullptr;
        }
        delete m_camera;
        m_camera = nullptr;
    }

    ui->ComboDevices->show();
    ui->bnEnum->show();
    // ui->bnOpenClose->show();
    ui->bnStartStop->hide();
    ui->bnStartStopTracking->hide();
    ui->bnPauseResumeTracking->hide();
    ui->bnForwardTracking->hide();

    ui->lExposure->hide();
    ui->tbExposure->hide();
    ui->lGain->hide();
    ui->tbGain->hide();
    ui->lFrameRate->hide();
    ui->tbFrameRate->hide();

    ui->tbExposure->setEnabled(false);
    ui->tbGain->setEnabled(false);
    ui->tbFrameRate->setEnabled(false);
    ui->bnSetParam->setEnabled(false);
    ui->bnGetParam->setEnabled(false);
}

// ch：开始采集 en:Start acquisition
void MainWindow::start_acquisition()
{
    if (m_camera == nullptr)
    {
        show_error_msg("Please open the camera first", 0);
        return;
    }
    if (m_live_scene == nullptr)
    {
        m_live_scene = new dgelom::EditableLiveImageScene(nullptr);
        ui->graphicsView->setScene(m_live_scene->scene());
        m_live_scene->scene()->setSceneRect(0, 0, m_camera->outframe().stFrameInfo.nWidth, m_camera->outframe().stFrameInfo.nHeight);
        ui->graphicsView->installEventFilter(this); // 安装事件过滤器
    }
    // ch:创建采集线程 en:Create acquisition thread
    if (m_acqworker == nullptr)
    {
        m_acqworker = new acquisition_worker(m_camera);
    }
    m_acqworker->start();
    QObject::connect(m_acqworker, &acquisition_worker::image_captured, this, &MainWindow::update_live_scene);

    ui->bnOpenClose->setEnabled(false);
    ui->bnStartStopTracking->setEnabled(true);
    ui->bnPauseResumeTracking->setEnabled(false);
    ui->bnForwardTracking->setEnabled(false);
}

// ch：停止采集 en:Stop acquisition
void MainWindow::stop_acquisition()
{
    m_acqworker->stop_acquisition();
    do
    {
        QThread::msleep(30);
    } while (m_acqworker->is_grabbing());
    delete m_acqworker;
    m_acqworker = nullptr;

    if (m_calworker)
    {
        m_calworker->m_sparseflow->end();
    }

    ui->bnOpenClose->setEnabled(true);
    ui->bnStartStopTracking->setEnabled(false);
    ui->bnPauseResumeTracking->setEnabled(false);
    ui->bnForwardTracking->setEnabled(false);
}

// ch：获取参数 en:Get parameter
void MainWindow::get_param()
{
    MVCC_FLOATVALUE stFloatValue;
    memset(&stFloatValue, 0, sizeof(MVCC_FLOATVALUE));

    int nRet = m_camera->get_float_value("ExposureTime", &stFloatValue);
    if (MV_OK != nRet)
    {
        show_error_msg("Get Exposure Time Fail", nRet);
    }
    else
    {
        ui->tbExposure->setText(QString("%1").arg(stFloatValue.fCurValue));
    }

    nRet = m_camera->get_float_value("Gain", &stFloatValue);
    if (MV_OK != nRet)
    {
        show_error_msg("Get Gain Fail", nRet);
    }
    else
    {
        ui->tbGain->setText(QString("%1").arg(stFloatValue.fCurValue));
    }

    nRet = m_camera->get_float_value("ResultingFrameRate", &stFloatValue);
    if (MV_OK != nRet)
    {
        show_error_msg("Get Frame Rate Fail", nRet);
    }
    else
    {
        ui->tbFrameRate->setText(QString("%1").arg(stFloatValue.fCurValue));
    }
}

// ch：设置参数 en:Set parameter
void MainWindow::set_param()
{
    m_camera->set_enum_value("ExposureAuto", 0);
    int nRet = m_camera->set_float_value("ExposureTime", ui->tbExposure->text().toFloat());
    if (MV_OK != nRet)
    {
        show_error_msg("Set Exposure Time Fail", nRet);
    }

    m_camera->set_enum_value("GainAuto", 0);
    nRet = m_camera->set_float_value("Gain", ui->tbGain->text().toFloat());
    if (MV_OK != nRet)
    {
        show_error_msg("Set Gain Fail", nRet);
    }

    nRet = m_camera->set_float_value("AcquisitionFrameRate", ui->tbFrameRate->text().toFloat());
    if (MV_OK != nRet)
    {
        show_error_msg("Set Frame Rate Fail", nRet);
    }

    get_param(); // ch：获取参数(更新参数) en:Get parameter (update parameter)
}

// ch：开始跟踪 en:Start tracking
void MainWindow::start_tracking()
{
    m_calworker = std::make_unique<calculation_worker>();
    int x = ui->tbX->text().toInt();
    int y = ui->tbY->text().toInt();
    m_calworker.get()->m_poi_list << QPointF(x, y);

    m_calworker.get()->m_options->set_image_size(m_camera->outframe().stFrameInfo.nHeight, m_camera->outframe().stFrameInfo.nWidth);

    QObject::connect(m_calworker.get()->m_sparseflow.get(), &deflow::OnlineSparseTrackingWorker::completed, [&](int idx)
                     { 
    //decltype(auto) offset_0 = m_calworker.get()->m_sparseflow->operator()().front();
    decltype(auto) offset_list = m_calworker.get()->m_sparseflow->operator()(idx);  
    if(idx==0)
        printf("[0]:\t0.000000\t0.000000\n");
    for (auto i = 0; i < offset_list.size(); i++)
    {        
        const auto xp = offset_list[i].x();
        const auto yp = offset_list[i].y();
        //cout << "x=" << x << "\ty=" << y << endl;
        printf("[%d]:\t%f\t%f\n",++idx,xp,yp);
        fflush(stdout);
    } });

    std::cout << "start tracking" << std::endl;
    m_calworker.get()->m_sparseflow->invoke(m_acqworker->get_image_ptr(), m_calworker->m_poi_list, false); // ch:开始跟踪,设置single step为false(连续计算) en:Start tracking, set single step to false (continuous calculation)

    ui->bnPauseResumeTracking->setEnabled(true);
    ui->bnForwardTracking->setEnabled(false);
}

// ch：停止跟踪 en:Stop tracking
void MainWindow::stop_tracking()
{
    m_calworker.get()->m_sparseflow->end();

    ui->bnPauseResumeTracking->setEnabled(false);
    ui->bnForwardTracking->setEnabled(false);
}

// ch：暂停跟踪 en:Pause tracking
void MainWindow::pause_tracking()
{
    m_calworker.get()->m_sparseflow->is_timed(true);

    ui->bnForwardTracking->setEnabled(true);
}

// ch：恢复跟踪 en:Resume tracking
void MainWindow::resume_tracking()
{
    m_calworker.get()->m_sparseflow->is_timed(false);
    m_calworker.get()->m_sparseflow->forward();

    ui->bnForwardTracking->setEnabled(false);
}

// ch：单步跟踪 en:Forward tracking
void MainWindow::forward_tracking()
{
    m_calworker.get()->m_sparseflow->forward();
}

// ch：更新实时图像 en:Update real-time image
void MainWindow::update_live_scene(const QImage &image)
{
    m_live_scene->set_image(image);
    if (!m_has_autofit)
    {
        ui->graphicsView->autofit();
        m_has_autofit = true;
    }
}

bool MainWindow::eventFilter(QObject *obj, QEvent *event)
{
    if (obj == ui->graphicsView)
    { // 确保事件来自graphicsView
        if (event->type() == QEvent::MouseButtonPress)
        {
            QMouseEvent *mouseEvent = static_cast<QMouseEvent *>(event);
            if (mouseEvent->button() == Qt::RightButton)
            {
                m_live_scene->exec(dgelom::Graphics::add_point);
                
                /* QPointF clickPoint = ui->graphicsView->mapToScene(mouseEvent->pos()); // 获取点击点的坐标
                qDebug() << "Clicked at:" << clickPoint;
                m_live_scene->clear_temp_items();
                m_live_scene->add_temp_point(clickPoint);

                ui->tbX->setText(QString::number((int)clickPoint.x()));
                ui->tbY->setText(QString::number((int)clickPoint.y()));    */
            }
        }
    }
    return QObject::eventFilter(obj, event);
}
