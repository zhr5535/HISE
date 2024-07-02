#include "sparse/workers/worker.hpp"

deflow::Worker::Worker(_Mybase* parent) 
	:_Mybase(parent) {
	/*@{*/
	connect(thread(), &QThread::started, this, &_Myt::_Exec);
	connect(thread(), &QThread::finished, this, &_Myt::_Free);
	connect(thread(), &QThread::finished, thread(), &QThread::deleteLater);
	/*@}*/
}
deflow::Worker::~Worker() {
	this->end();
}

void deflow::Worker::run(category_t thrcat) noexcept
{
	if (thrcat) {
		detach().start();
	}
	else {
		_Exec();
	}
}
deflow::Worker::_Myt& deflow::Worker::detach(bool v) noexcept
{
	if (v) {
		this->moveToThread(&_Mythread);
		connect(thread(), &QThread::finished, this, &Worker::deleteLater);
	}
	_Detached = v;
	return (*this);
}

void deflow::Worker::start() noexcept
{
	_Mystopflag = false;
	if (_Detached) {
		_Mythread.start();
	}
}

void deflow::Worker::wait() noexcept
{
	if (_Detached) {
		_Mythread.wait();
	}
}

void deflow::Worker::suspend(bool value) noexcept
{
	_Mysuspended = value;
}

void deflow::Worker::end(bool after_wake_all) noexcept
{
	if (is_running()) {
		if(after_wake_all) _Mywaitcond.wakeAll();
		_Mythread.requestInterruption();
		_Mystopflag = true;
		_Mythread.quit();
		_Mythread.wait();
		_Detached = false;
	}
}

void deflow::Worker::pause() noexcept
{
	if (is_running()) {
		_Mypauseflag = true;
		emit paused(_Myid);
	}
}

void deflow::Worker::resume() noexcept
{
	if (is_running()) {
		_Mypauseflag = false;
		_Mywaitcond.wakeAll();
	}
}

QString deflow::Worker::message() const noexcept {
	return (_Mymessage);
}

bool deflow::Worker::detached() const noexcept {
	return _Detached;
}

bool deflow::Worker::is_running() const noexcept {
	return _Mythread.isRunning();
}

bool deflow::Worker::is_paused() const noexcept {
	return _Mypauseflag;
}

bool(deflow::Worker::ireq)() const noexcept
{
	return _Mythread.isInterruptionRequested();
}

const deflow::Worker::index_t&(deflow::Worker::index)() const noexcept
{
	return _Myid;
}
deflow::Worker::index_t&(deflow::Worker::index)() noexcept {
	return _Myid;
}

deflow::Worker::thread_t deflow::Worker::thread() noexcept {
	return DF_USE_STD(addressof)(_Mythread);
}

void deflow::Worker::single_step(bool flag) noexcept
{
	_Mystopflag = flag;
}

void deflow::Worker::_Try_to_suspend(int i) noexcept
{
	if (_Mypauseflag) { // Suspend the worker if it is paused
		_Mymutex.lock();
		_Mywaitcond.wait(&(this->_Mymutex));
		_Mymutex.unlock();
		emit paused(i);
	}
}
