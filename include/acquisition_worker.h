#ifndef ACQ_WORKER_H
#define ACQ_WORKER_H

#include <QThread>
#include <QImage>
#include <QMutex>
#include <QDebug>
#include "hikcamera/hikcamera_control.h"

class acquisition_worker : public QThread
{
    Q_OBJECT;

public:
    acquisition_worker(HikCamera *cam = nullptr, QObject *parent = nullptr);
    ~acquisition_worker();

    void stop_acquisition();
    const QImage get_image() const;
    QImage *get_image_ptr();
    void set_camera(HikCamera *camera);
    bool is_grabbing() const;

signals:
    void image_captured(QImage image);
    void error_occurred(const QString &errorMessage);

protected:
    void static __stdcall image_callback(unsigned char *data, MV_FRAME_OUT_INFO_EX *frame_info, void *user);

    void run() override;

private:
    QImage m_image;

    HikCamera *m_device;

    mutable QMutex m_mutex;

    bool m_isgrabbing;

public:
    QMetaObject::Connection m_captured_connection;
};

#endif // ACQ_WORKER_H