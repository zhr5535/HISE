Demo: main.cpp
	g++ -o test main.cpp ../src/mainwindow.cpp ../src/acquisition_worker.cpp ../src/calculation_worker.cpp ../src/hikcamera_control.cpp moc_mainwindow.cpp moc_acquisition_worker.cpp -I. -I../include -I../include/graphics -L../lib -L/opt/MVS/lib/aarch64 -lgraphics2x -lsparseflow -lMvCameraControl -fPIC `pkg-config --cflags --libs Qt5Core Qt5Gui Qt5Widgets opencv4 tbb` -fpermissive


clean:
	rm test -rf
