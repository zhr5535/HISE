#include "acquisition_worker.h"

acquisition_worker::acquisition_worker(HikCamera *cam, QObject *parent)
    : QThread(parent), m_device(cam), m_isgrabbing(false)
{
}

acquisition_worker::~acquisition_worker()
{
    stop_acquisition();
    wait();
}

void acquisition_worker::stop_acquisition()
{
    QMutexLocker locker(&m_mutex);
    m_device->stop_grabbing();
    m_isgrabbing = false;
}

const QImage acquisition_worker::get_image() const
{
    QMutexLocker locker(&m_mutex);
    return m_image;
}

QImage *acquisition_worker::get_image_ptr()
{
    QMutexLocker locker(&m_mutex);
    return &m_image;
}

void acquisition_worker::set_camera(HikCamera *camera)
{
    m_device = camera;
}

bool acquisition_worker::is_grabbing() const
{
    QMutexLocker locker(&m_mutex);
    return m_isgrabbing;
}

void __stdcall acquisition_worker::image_callback(unsigned char *data, MV_FRAME_OUT_INFO_EX *frame_info, void *user)
{

    if (user)
    {
        acquisition_worker *this_worker = static_cast<acquisition_worker *>(user);
        MV_FRAME_OUT frame = {0};
        frame.pBufAddr = data;
        frame.stFrameInfo = *frame_info;
        this_worker->m_device->set_outframe(frame);
        // 测试用：回调写入相机类内帧信息
        /* qDebug() << "image_callback: addr=[" << this_worker->m_device->outframe().pBufAddr << "] [" << this_worker->m_device->outframe().stFrameInfo.nWidth << "," << this_worker->m_device->outframe().stFrameInfo.nHeight << "]"; */

        QImage temp_image(data, frame_info->nWidth, frame_info->nHeight, QImage::Format_Grayscale8);
        {
            QMutexLocker locker(&this_worker->m_mutex);
            this_worker->m_image = temp_image;
        }
        emit this_worker->image_captured(temp_image);
    }
    else
    {
        qDebug() << "image_callback: user is null";
    }
}

void acquisition_worker::run()
{
    m_device->register_image_callback(image_callback, this);
    m_device->start_grabbing();
    m_isgrabbing = true;
}
