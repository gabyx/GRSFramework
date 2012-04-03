#include <assert.h>
#include <stdio.h>
#include <boost/bind.hpp>
#include <srutil/event/event.hpp>

struct ListenerInterface
{
	virtual void event1() = 0;
	virtual void event2(int) = 0;
	virtual void eventDestroy() = 0;
	virtual void eventImpossible() = 0;
};

typedef srutil::event_source<ListenerInterface*> TestEventSource;

class TestEventListener : private ListenerInterface
{
	TestEventSource::binder_type binder;
	TestEventSource& source;

public:
	static int instanceCount;

	TestEventListener(TestEventSource& source)	: source(source)
	{
		instanceCount++;
		binder.bind(source, this);
	}

	~TestEventListener()
	{
		instanceCount--;
	}

private:
	virtual void event1()
	{
		printf("event1 received (%p)\n", this);
	}

	virtual void event2(int value)
	{
		printf("event2 received (%d; %p)\n", value, this);
		source.emit(boost::bind(&ListenerInterface::event1, _1));
		source.emit(boost::bind(&ListenerInterface::eventDestroy, _1));
	}

	virtual void eventDestroy()
	{
		printf("eventDestroy received (%p)\n", this);
		delete this;
	}

	virtual void eventImpossible()
	{
		printf("eventImpossible received (%p)\n", this);
		assert(false);
	}
};

int TestEventListener::instanceCount = 0;

TestEventSource g_source;

int main()
{
	new TestEventListener(g_source);
	new TestEventListener(g_source);
	new TestEventListener(g_source);

	g_source.emit(boost::bind(&ListenerInterface::event1, _1));
	g_source.emit(boost::bind(&ListenerInterface::event2, _1, 10));
	g_source.emit(boost::bind(&ListenerInterface::eventImpossible, _1));

	assert(TestEventListener::instanceCount == 0);
	return 0;
}
