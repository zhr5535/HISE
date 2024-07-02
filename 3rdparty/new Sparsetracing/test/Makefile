Demo: main.cpp
	g++ -o test main.cpp -I../include/sparse/workers -I../include -L../lib/aarch64_linux -lsparseflow -fpermissive -fPIC `pkg-config --cflags --libs opencv4 Qt5Core Qt5Gui Qt5PrintSupport Qt5Widgets tbb` -fpermissive


clean:
	rm test -rf