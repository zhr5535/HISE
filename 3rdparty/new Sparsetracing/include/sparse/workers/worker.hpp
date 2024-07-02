#pragma once
#include <QObject>
#include <QThread>
#include <QMutex>
#include <QWaitCondition>
#include <math.h>

#include "sparse/__defs_ports__.hpp"

namespace deflow
{
class DEFLOW_ALG Worker : public QObject
{
	Q_OBJECT;
	using _Mybase = QObject;
	using _Myt = Worker;
public:
	using fvalue_type = double_t;
	using ivalue_type = int64_t;
	using index_t = int32_t;
	using string_t = QString;
	using thread_t = DF_USE_STD(add_pointer_t<QThread>);
	using pointer = DF_USE_STD(add_pointer_t<fvalue_type>);

	enum _Category : bool {
		IN_THE_FG = DF_USE_STD(false_type)::value, //For foreground thread
		IN_THE_BG = DF_USE_STD(true_type)::value,  //For background thread
	};
	using category_t = _Category;
	enum class _Error_tag : uint8_t {
		FILE_READ_ERROR=0, // Caused by missing file or file corruption
		FILE_PATH_ERROR=1, // Caused by invalid path(s) or null path list
		IMAG_PROC_ERROR=2, // Caused by image matching or other processing errors

	};
	using error_t = _Error_tag;

	Worker(_Mybase* parent = nullptr);
	~Worker();

	/// <summary>
	/// \brief Run this worker with specified thread category.
	/// </summary>
	void run(category_t thrcat = IN_THE_BG) noexcept;

	/// <summary>
	/// \brief Detach a worker thread from the main thread.
	/// The detached worker will run in the background by calling 'start()'.
	/// \param v True for detaching and false for donot.
	/// </summary>
	_Myt& detach(bool v=true) noexcept;

	/// <summary>
	/// \brief Invoke a detached worker thread. 
	/// This method works followed by the method detach().
	/// </summary>
	void start() noexcept;

	void wait() noexcept;
	void suspend(bool value=true) noexcept;
	void pause() noexcept;
	void resume() noexcept;

	/// <summary>
	/// \brief End an invoked worker thread.
	/// </summary>
	void end(bool after_wake_all=true) noexcept;

	/// <summary>
	/// \brief Get the error message of the worker.
	/// </summary>
	string_t message() const noexcept;

	/// <summary>
	/// \brief Query if the worker thread is detached from the main thread.
	/// </summary>
	/// <returns>True if the worker thread is detached else false</returns>
	bool(detached)() const noexcept;

	/// <summary>
	/// \brief Query the status of the worker thread is running or not.
	/// </summary>
	/// <returns>True if the worker thread is running else false</returns>
	bool(is_running)()const noexcept;

	/// <summary>
	/// \brief Query the status of the worker thread is suspended or not.
	/// </summary>
	/// <returns>True if the worker thread is suspended else false</returns>
	bool(is_suspended)()const noexcept;

	/// <summary>
	/// \brief Query if the worker thread is paused or not.
	/// </summary>
	bool(is_paused)() const noexcept;

	/// <summary>
	/// \brief Status request method.
	/// </summary>
	bool(ireq)()const noexcept;

	/// <summary>
	/// \brief Getter/Setter of the user defined thread index "_Myid".
	/// </summary>
	const index_t&(index)() const noexcept;
	index_t&(index)() noexcept;

	/// <summary>
	/// \brief Getter of the pointer to buit-in thread instance.
	/// </summary>
	thread_t(thread)() noexcept;

	/// <summary>
	/// \brief Disable or enable the while loop.
	/// </summary>
	/// <param name="flag">true for disable, and false for enable.</param>
	void single_step(bool flag = true) noexcept;

signals:
	/// <summary>
	/// \brief Emitted after the task given in _Exec() is completed.
	/// </summary>
	void finished();
	void completed(int id = 0);

	/// <summary>
	/// \brief Emitted after the task is canceled.
	/// </summary>
	void canceled(int id = 0);

	/// <summary>
	/// \brief Emitted when the task is paused.
	/// </summary>
	void paused(int id = 0);

	/// <summary>
	/// \brief Emitted when the worker thread is suspended.
	/// </summary>
	void suspended();

	/// <summary>
	/// \brief Emitted when the task is abnormal interrupted. 
	/// For details, connect it to a slot to check the message by calling message().
	/// </summary>
	void failed(error_t);
	void failed_with_err(const string_t&);
protected:
	/// <summary>
	/// \brief Virtual interface to run a task given by its override in a specialized worker.
	/// </summary>
	virtual void _Exec() noexcept = 0;

	/// <summary>
	/// \brief Protected pure virtual method
	/// </summary>
	virtual void _Free() = 0;

	/// <summary>
	/// \brief Protected method
	/// </summary>
	/// <param name="i"></param>
	/// <returns></returns>
	void _Try_to_suspend(int i) noexcept;

	// Field of the worker thread
	QThread _Mythread;

	// Indicate if the worker is detached from the main thread
	bool _Detached{ false };

	// User specified index for the worker
	index_t _Myid{ 0 };

	// For recording the error message
	string_t _Mymessage;

	/*@{ The following fields for thread hanging */
	QMutex _Mymutex;
	QWaitCondition _Mywaitcond;
	std::atomic_bool _Mypauseflag{ false };
	std::atomic_bool _Mystopflag{ false };
	// 
	std::atomic_bool _Mysuspended{ false };
	/*@}*/
};

// Alias of nested type `Worker::_Category`
using worker_cat_t = Worker::category_t;

// Alias of nested type `Worker::_Error_tag`
using worker_err_t = Worker::error_t;

class DEFLOW_ALG WorkerThread : public QObject
{
	Q_OBJECT;
	using _Mybase = QObject;
	using _Myt = WorkerThread;
public:
	using fvalue_type = double_t;
	using pointer = std::add_pointer_t<fvalue_type>;
	using ivalue_type = int64_t;
	using index_t = int32_t;

	inline WorkerThread(_Mybase* parent = nullptr) noexcept
		:_Mybase(parent) {
		/*@{*/
		connect(this->thread(), &QThread::started, this, &_Myt::_Run);
		/*@}*/
	};
	inline ~WorkerThread() noexcept {
		this->end();
	};

	/// <summary>
	/// \brief Run this worker with a detached thread or not.
	/// The default is to run on detached worker, otherwise 
	/// the main thread will be blocked.
	/// </summary>
	inline void run(bool detached=true) noexcept {
		if (detached) {
			detach().start();
		}
		else {
			_Exec();
		}
	}

	/// <summary>
	/// \brief Detach a worker thread from the main thread.
	/// The detached worker will run in the background by calling 'start()'.
	/// </summary>
	inline _Myt& detach() noexcept {
		this->moveToThread(&_Mythread);
		_Detached = true;
		return (*this);
	}

	/// <summary>
	/// \brief Invoke a detached worker thread. This method works followed by the method detach().
	/// </summary>
	inline void start() noexcept {
		_Mystopflag = false;
		if (_Detached) {
			_Mythread.start();
		}
	}

	inline void wait() noexcept {
		if (_Detached) {
			_Mythread.wait();
		}
	}

	inline void end() noexcept {
		if (is_running()) {
			_Mythread.requestInterruption();
			_Mywaitcond.wakeAll();
			_Mythread.quit();
			_Mythread.wait(100);
			_Detached = false;
			_Mystopflag = true;
		}
	}

	inline decltype(auto) message() const noexcept {
		return (_Mymessage);
	}
	inline decltype(auto) message() noexcept {
		return (_Mymessage);
	}

	inline bool detached() const noexcept {
		return _Detached;
	}

	inline bool is_running() const noexcept {
		return _Mythread.isRunning();
	}

	inline void pause() noexcept {
		if (is_running()) {
			_Mypauseflag = true;
			emit paused(_Myid);
		}
	}

	inline void resume() noexcept {
		if (is_running()) {
			_Mypauseflag = false;
			_Mywaitcond.wakeAll();
		}
	}

	/// <summary>
	/// \brief Getter/Setter of the user defined thread index "_Myid".
	/// </summary>
	inline decltype(auto) index() const noexcept {
		return (_Myid);
	}
	inline decltype(auto) index() noexcept {
		return (_Myid);
	}

	/// <summary>
	/// \brief Getter of the pointer to buit-in thread instance.
	/// </summary>
	inline const QThread* thread() const noexcept {
		return std::addressof(_Mythread);
	}
	inline QThread* thread() noexcept {
		return std::addressof(_Mythread);
	}

	/// <summary>
	/// \brief Disable or enable the while loop.
	/// </summary>
	/// <param name="flag">true for disable, and false for enable.</param>
	inline auto single_step(bool flag = true) noexcept {
		_Mystopflag = flag;
	}

signals:
	/// <summary>
	/// \brief Emitted after the task given in _Exec() is completed.
	/// </summary>
	void finished();
	void completed(int id = 0);

	/// <summary>
	/// \brief Emitted after the task is canceled.
	/// </summary>
	void canceled(int id = 0);

	/// <summary>
	/// \brief Emitted when the task is paused.
	/// </summary>
	void paused(int id = 0);

	/// <summary>
	/// \brief Emitted when the task is abnormal interrupted. 
	/// For details, connect it to a slot to check the message by calling message().
	/// </summary>
	void failed(int idx = 0);

protected:
	virtual void _Run() final {
		while (!_Mystopflag) {
			_Exec();
			if (_Mypauseflag) { // Suspend the worker if _Mypauseflag is true
				_Mymutex.lock();
				_Mywaitcond.wait(&(this->_Mymutex));
				_Mymutex.unlock();
			}
			_Mypauseflag = false;
		}
	}

	/// <summary>
	/// \brief Pure virtual interface to execute specified task.
	/// The task should be defined by its overriding in a derived worker, and
	/// if the task is performed only once, `_Mystopflag` should be set to true
	/// at the end of `_Exec()` implementation. Otherwise, _Exec() will be called
	/// in the while-loop in `_Run()` until the worker thread is ended or interrupted.
	/// </summary>
	virtual void _Exec() noexcept = 0;

	// Field of the worker thread
	QThread _Mythread;

	// Indicate if the worker is detached from the main thread
	bool _Detached{ false };

	// User specified index for the worker
	uint32_t _Myid{ 0 };

	// For recording the error message
	QString _Mymessage;

	/*@{ The following fields for thread hanging */
	QMutex _Mymutex;
	QWaitCondition _Mywaitcond;
	std::atomic_bool _Mypauseflag{ false };
	std::atomic_bool _Mystopflag{ false };
	/*@}*/
};

} // namespace deflow