#include <iostream>
#include <sstream>
#include <QApplication>
#include <QObject>

#include <opencv2/opencv.hpp>
#include "sparse_tracking_worker.h"
using namespace std;

string file_path(int i)
{
    stringstream path;
    path << "./test_images/speckle_" << i << "_0.bmp";
    return path.str();
}

int main(int argc, char *argv[])
{
    QCoreApplication app(argc, argv);
    auto _options_ = deflow::make_shared_options();
    auto options = deflow::options_t(_options_);
    auto worker = std::make_unique<deflow::SparseTrackingWorker>(options);

    QStringList image_list;
    for (int i = 0; i < 21; i++)
    {
        image_list << QString::fromStdString(file_path(i));
    }
    cout << "image_list is empty?=" << image_list.empty() << "   size=" << image_list.size() << endl;

    worker->input_from(deflow::SparseTrackingWorker::image_input_tag::PLAIN_FILE);
    if (worker->is_offline())
    {
        _options_->image_paths = image_list;
    }
    cout << "stereo_paths is empty?=" << _options_->image_paths.empty() << "   size=" << _options_->image_paths.size() << endl;
    /*else
    {
        _options_->set_image_size(1024, 1024);
    }*/

    /*auto image = cv::imread("/home/dynvis/work/tracing_test/test_images.speckle_0_0.bmp", cv::IMREAD_GRAYSCALE);

    cout << "image is empty?=" << image.empty() << endl;*/

    QList<QPointF> poi_list;
    poi_list << QPointF(200, 200);

    /*if (worker->is_online())
    {
        worker->invoke(std::move(poi_list), camera->data());
    }*/

    if (worker->is_offline())
    {
        QObject::connect(worker.get(), &deflow::SparseTrackingWorker::completed,
                         [&](int idx)
                         { 
    decltype(auto) offset_0 = worker->operator()().front();
    decltype(auto) offset_list = worker->operator()(idx);
    for (auto i = 0; i < offset_list.size(); i++)
    {
        const auto x = offset_list[i].x();
        const auto y = offset_list[i].y();
        cout << "x=" << x << "\ty=" << y << endl;
    } });
        worker->invoke(std::move(poi_list), int32_t(0));
    }
    //worker->end();
    return app.exec();
    //return 0;
}

/*void _report_(int idx)
{
    decltype(auto) offset_0 = worker->operator()().front();
    decltype(auto) offset_list = worker->operator()(idx);
    for (auto i = 0; i < offset_list.size(); i++)
    {
        const auto x = offset_list[i].x();
        const auto y = offset_list[i].y();
        cout << "x=" << x << endl;
        cout << "y=" << y << endl;
    }
}*/