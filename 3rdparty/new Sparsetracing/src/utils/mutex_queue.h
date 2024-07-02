#ifndef BLOCKINGQUEUE_H
#define BLOCKINGQUEUE_H
#include <QMutexLocker>
#include <QMutex>
#include <QWaitCondition>
#include <QQueue>

template<class T>
class MutexQueue
{
public:
	MutexQueue(int max_size)
	{		
		max_size_ = max_size;
	};
	~MutexQueue()
	{
		
	};
	void Reset()
	{
		QMutexLocker locker(&mutex_);
		quit_ = false;
		wait_condition_.wakeAll();		
	}
	void Quit()
	{
		QMutexLocker locker(&mutex_);
		queue_.clear();
		quit_ = true;
		wait_condition_.wakeAll();
	};
	bool TryEnqueue(const T& t)
	{
		QMutexLocker locker(&mutex_);
		if(queue_.size()>=max_size_)
		{
			return false;
		}
		queue_.enqueue(t);
		wait_condition_.wakeAll();
		return true;
	}
	bool Enqueue(const T& t)
	{
		QMutexLocker locker(&mutex_);
		while(queue_.size() >= max_size_ && !quit_)
		{
			wait_condition_.wait(&mutex_);
		}
		if (quit_)
		{
			return false;
		}
		queue_.enqueue(t);
		wait_condition_.wakeAll();
		return true;
	}
	bool Dequeue(T* t)
	{
		QMutexLocker locker(&mutex_);
		while(queue_.isEmpty() && !quit_)
		{
			wait_condition_.wait(&mutex_);
		}
		if (quit_)
		{
			return false;
		}
		*t = queue_.dequeue();
		wait_condition_.wakeAll();
		return true;
	};

private:
	QQueue<T> queue_;
	QMutex mutex_;
	QWaitCondition wait_condition_;
	bool quit_;
	int max_size_;
};

#endif // BLOCKINGQUEUE_H
