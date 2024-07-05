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
#include <QtWidgets/QComboBox>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent),
                                          ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    m_live_scene = nullptr;
    memset(&m_device_list, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
    m_camera = nullptr;
    m_acqworker = nullptr;
    m_has_autofit = false;

    QComboBox *ComboDevices = new QComboBox(ui->tbEnum);
    ComboDevices->setObjectName(QString::fromUtf8("ComboDevices"));
    ComboDevices->setGeometry(QRect(0, 0, 400, 25));
    ui->tbEnum->addWidget(ComboDevices);

    QLabel *lExposure = new QLabel("Expo", ui->tbCamParam);
    lExposure->setObjectName(QString::fromUtf8("lExposure"));
    lExposure->setFixedSize(35, 25);
    lExposure->setAlignment(Qt::AlignCenter);
    QLabel *lGain = new QLabel("Gain", ui->tbCamParam);
    lGain->setObjectName(QString::fromUtf8("lGain"));
    lGain->setFixedSize(35, 25);
    lGain->setAlignment(Qt::AlignCenter);
    QLabel *lFrameRate = new QLabel("FPS", ui->tbCamParam);
    lFrameRate->setObjectName(QString::fromUtf8("lFrameRate"));
    lFrameRate->setFixedSize(35, 25);
    lFrameRate->setAlignment(Qt::AlignCenter);
    QLineEdit *leExposure = new QLineEdit(ui->tbCamParam);
    leExposure->setObjectName(QString::fromUtf8("leExposure"));
    leExposure->setFixedSize(65, 25);
    leExposure->setEnabled(false);
    QLineEdit *leGain = new QLineEdit(ui->tbCamParam);
    leGain->setObjectName(QString::fromUtf8("leGain"));
    leGain->setFixedSize(65, 25);
    leGain->setEnabled(false);
    QLineEdit *leFrameRate = new QLineEdit(ui->tbCamParam);
    leFrameRate->setObjectName(QString::fromUtf8("leFrameRate"));
    leFrameRate->setFixedSize(65, 25);
    leFrameRate->setEnabled(false);
    ui->tbCamParam->addWidget(lExposure);
    ui->tbCamParam->addWidget(leExposure);
    ui->tbCamParam->addSeparator();
    ui->tbCamParam->addWidget(lGain);
    ui->tbCamParam->addWidget(leGain);
    ui->tbCamParam->addSeparator();
    ui->tbCamParam->addWidget(lFrameRate);
    ui->tbCamParam->addWidget(leFrameRate);

    ui->tbEnum->setVisible(true);
    ui->tbControl->setVisible(true);
    ui->tbTracking->setVisible(false);
    ui->tbCamParam->setVisible(false);
    ui->actionOpenClose->setEnabled(false);

    ui->actionOpenClose->setEnabled(false);
    ui->actionStartStop->setEnabled(false);
    ui->actionStartStopTracking->setEnabled(false);
    ui->actionCameraParam->setEnabled(false);
    ui->actionPauseResumeTracking->setEnabled(false);
    ui->actionForwardTracking->setEnabled(false);

    connect(ui->actionEnum, &QAction::triggered, this, &MainWindow::actionEnum_triggered);
    connect(ui->actionOpenClose, &QAction::triggered, this, &MainWindow::actionOpenClose_triggered);
    connect(ui->actionStartStop, &QAction::triggered, this, &MainWindow::actionStartStop_triggered);
    connect(ui->actionStartStopTracking, &QAction::triggered, this, &MainWindow::actionStartStopTracking_triggered);
    connect(ui->actionPauseResumeTracking, &QAction::triggered, this, &MainWindow::actionPauseResumeTracking_triggered);
    connect(ui->actionForwardTracking, &QAction::triggered, this, &MainWindow::actionForwardTracking_triggered);
    connect(ui->actionCameraParam, &QAction::triggered, this, &MainWindow::actionCameraParam_triggered);

    connect(leExposure, &QLineEdit::returnPressed, this, &MainWindow::set_param);
    connect(leExposure, &QLineEdit::editingFinished, this, &MainWindow::get_param);
    connect(leGain, &QLineEdit::returnPressed, this, &MainWindow::set_param);
    connect(leGain, &QLineEdit::editingFinished, this, &MainWindow::get_param);
    connect(leFrameRate, &QLineEdit::returnPressed, this, &MainWindow::set_param);
    connect(leFrameRate, &QLineEdit::editingFinished, this, &MainWindow::get_param);

    // showMaximized(); // 自动最大化显示
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

void MainWindow::actionOpenClose_triggered()
{
    if (ui->actionOpenClose->text() == "Open")
    {
        open_camera();
        ui->actionOpenClose->setText("Close");
        ui->actionStartStop->setText("Start");
        ui->actionStartStopTracking->setText("Track");
        ui->actionPauseResumeTracking->setText("Pause");
    }
    else if (ui->actionOpenClose->text() == "Close")
    {
        close_camera();
        ui->actionOpenClose->setText("Open");
    }
}

void MainWindow::actionStartStop_triggered()
{
    if (ui->actionStartStop->text() == "Start")
    {
        start_acquisition();
        ui->actionStartStop->setText("Stop");
    }
    else if (ui->actionStartStop->text() == "Stop")
    {
        stop_acquisition();
        ui->actionStartStop->setText("Start");
    }
}

void MainWindow::actionStartStopTracking_triggered()
{
    if (ui->actionStartStopTracking->text() == "Track")
    {
        start_tracking();
        ui->actionStartStopTracking->setText("End");
    }
    else if (ui->actionStartStopTracking->text() == "End")
    {
        stop_tracking();
        ui->actionStartStopTracking->setText("Track");
    }
}

void MainWindow::actionPauseResumeTracking_triggered()
{
    if (ui->actionPauseResumeTracking->text() == "Pause")
    {
        pause_tracking();
        ui->actionPauseResumeTracking->setText("Resume");
    }
    else if (ui->actionPauseResumeTracking->text() == "Resume")
    {
        resume_tracking();
        ui->actionPauseResumeTracking->setText("Pause");
    }
}

void MainWindow::actionCameraParam_triggered()
{
    if (ui->tbCamParam->isVisible())
    {
        ui->tbCamParam->setVisible(false);
    }
    else
    {
        ui->tbCamParam->setVisible(true);
    }
}

/* void MainWindow::on_bnZoomIn_clicked()
{
    ui->graphicsView->zoom_slider()->setValue(ui->graphicsView->zoom_slider()->value() + 1);
}

void MainWindow::on_bnZoomOut_clicked()
{
    ui->graphicsView->zoom_slider()->setValue(ui->graphicsView->zoom_slider()->value() - 1);
} */

// ch：枚举设备 en:Enumerate devices
void MainWindow::enum_devices()
{
    QComboBox *ComboDevices = findChild<QComboBox *>("ComboDevices");
    if (ComboDevices)
    {
        ComboDevices->clear();
        QTextCodec::setCodecForLocale(QTextCodec::codecForName("GBK"));
        ComboDevices->setStyle(QStyleFactory::create("Windows"));
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
            ComboDevices->addItem(QString::fromLocal8Bit(user_name));
        }

        if (0 == m_device_list.nDeviceNum)
        {
            show_error_msg("No device", 0);
            return;
        }
        ComboDevices->setCurrentIndex(0);
        ui->actionOpenClose->setEnabled(true);
    }
    else
    {
        show_error_msg("ComboDevices not found", 0);
    }
}

// ch：打开设备 en:Open device
void MainWindow::open_camera()
{
    QComboBox *ComboDevices = findChild<QComboBox *>("ComboDevices");
    if (ComboDevices)
    {
        int nIndex = ComboDevices->currentIndex();
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

        ui->tbEnum->setVisible(false);

        ui->actionStartStop->setEnabled(true);

        QLineEdit *leExposure = findChild<QLineEdit *>("leExposure");
        QLineEdit *leGain = findChild<QLineEdit *>("leGain");
        QLineEdit *leFrameRate = findChild<QLineEdit *>("leFrameRate");
        leExposure->setEnabled(true);
        leGain->setEnabled(true);
        leFrameRate->setEnabled(true);
        ui->actionCameraParam->setEnabled(true);
    }
    else
    {
        show_error_msg("ComboDevices not found", 0);
    }
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

    ui->actionStartStop->setEnabled(false);
    ui->actionStartStopTracking->setEnabled(false);
    ui->actionCameraParam->setEnabled(false);
    ui->actionPauseResumeTracking->setEnabled(false);
    ui->actionForwardTracking->setEnabled(false);

    ui->tbEnum->setVisible(true);
    ui->tbTracking->setVisible(false);
    ui->tbCamParam->setVisible(false);
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

    ui->actionOpenClose->setEnabled(false);
    ui->actionStartStopTracking->setEnabled(true);
    ui->actionPauseResumeTracking->setEnabled(false);
    ui->actionForwardTracking->setEnabled(false);
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

    ui->actionOpenClose->setEnabled(true);
    ui->actionStartStopTracking->setEnabled(false);
    ui->actionPauseResumeTracking->setEnabled(false);
    ui->actionForwardTracking->setEnabled(false);
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
        QLineEdit *leExposure = findChild<QLineEdit *>("leExposure");
        if (leExposure)
        {
            leExposure->setText(QString("%1").arg(stFloatValue.fCurValue));
        }
        else
        {
            show_error_msg("leExposure not found", 0);
        }
    }

    nRet = m_camera->get_float_value("Gain", &stFloatValue);
    if (MV_OK != nRet)
    {
        show_error_msg("Get Gain Fail", nRet);
    }
    else
    {
        QLineEdit *leGain = findChild<QLineEdit *>("leGain");
        if (leGain)
        {
            leGain->setText(QString("%1").arg(stFloatValue.fCurValue));
        }
        else
        {
            show_error_msg("leGain not found", 0);
        }
    }

    nRet = m_camera->get_float_value("ResultingFrameRate", &stFloatValue);
    if (MV_OK != nRet)
    {
        show_error_msg("Get Frame Rate Fail", nRet);
    }
    else
    {
        QLineEdit *leFrameRate = findChild<QLineEdit *>("leFrameRate");
        if (leFrameRate)
        {
            leFrameRate->setText(QString("%1").arg(stFloatValue.fCurValue));
        }
        else
        {
            show_error_msg("leFrameRate not found", 0);
        }
    }
}

// ch：设置参数 en:Set parameter
void MainWindow::set_param()
{
    QLineEdit *leExposure = findChild<QLineEdit *>("leExposure");
    if (leExposure)
    {
        m_camera->set_enum_value("ExposureAuto", 0);
        int nRet = m_camera->set_float_value("ExposureTime", leExposure->text().toFloat());
        if (MV_OK != nRet)
        {
            show_error_msg("Set Exposure Time Fail", nRet);
        }
    }
    else
    {
        show_error_msg("leExposure not found", 0);
    }

    QLineEdit *leGain = findChild<QLineEdit *>("leGain");
    if (leGain)
    {
        m_camera->set_enum_value("GainAuto", 0);
        int nRet = m_camera->set_float_value("Gain", leGain->text().toFloat());
        if (MV_OK != nRet)
        {
            show_error_msg("Set Gain Fail", nRet);
        }
    }
    else
    {
        show_error_msg("leGain not found", 0);
    }

    QLineEdit *leFrameRate = findChild<QLineEdit *>("leFrameRate");
    if (leFrameRate)
    {
        int nRet = m_camera->set_float_value("AcquisitionFrameRate", leFrameRate->text().toFloat());
        if (MV_OK != nRet)
        {
            show_error_msg("Set Frame Rate Fail", nRet);
        }
    }
    else
    {
        show_error_msg("leFrameRate not found", 0);
    }

    get_param(); // ch：获取参数(更新参数) en:Get parameter (update parameter)
}

// ch：开始跟踪 en:Start tracking
void MainWindow::start_tracking()
{
    m_calworker = std::make_unique<calculation_worker>();

    // 暂时测试
    int x = 1224;
    int y = 1024;

    m_calworker.get()->m_poi_list << QPointF(x, y);

    m_calworker.get()->m_options->set_image_size(m_camera->outframe().stFrameInfo.nHeight, m_camera->outframe().stFrameInfo.nWidth);

    QObject::connect(m_calworker.get()->m_sparseflow.get(), &deflow::OnlineSparseTrackingWorker::completed, [&](int idx)
                     { 
    //decltype(auto) offset_0 = m_calworker.get()->m_sparseflow->operator()().front();
    decltype(auto) offset_list = m_calworker.get()->m_sparseflow->operator()(idx);  
    if(idx == 0)
        printf("[0]:\t0.000000\t0.000000\n");
    for (auto i = 0; i < offset_list.size(); i++)
    {        
        const auto xp =  offset_list[i].x();
        const auto yp = offset_list[i].y();
        //cout << "x=" << x << "\ty=" << y << endl;
        printf("[%d]:\t%f\t%f\n",++idx,xp,yp);
        fflush(stdout);                                
    } });

    std::cout << "start tracking" << std::endl;
    m_calworker.get()->m_sparseflow->invoke(m_acqworker->get_image_ptr(), m_calworker->m_poi_list, false); // ch:开始跟踪,设置single step为false(连续计算) en:Start tracking, set single step to false (continuous calculation)

    ui->tbTracking->setVisible(true);

    ui->actionPauseResumeTracking->setEnabled(true);
    ui->actionForwardTracking->setEnabled(false);
}

// ch：停止跟踪 en:Stop tracking
void MainWindow::stop_tracking()
{
    m_calworker.get()->m_sparseflow->end();

    ui->tbTracking->setVisible(false);
    ui->actionPauseResumeTracking->setEnabled(false);
    ui->actionForwardTracking->setEnabled(false);
}

// ch：暂停跟踪 en:Pause tracking
void MainWindow::pause_tracking()
{
    m_calworker.get()->m_sparseflow->is_timed(true);

    ui->actionForwardTracking->setEnabled(true);
}

// ch：恢复跟踪 en:Resume tracking
void MainWindow::resume_tracking()
{
    m_calworker.get()->m_sparseflow->is_timed(false);
    m_calworker.get()->m_sparseflow->forward();

    ui->actionForwardTracking->setEnabled(false);
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
