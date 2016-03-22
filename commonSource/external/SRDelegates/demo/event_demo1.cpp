/*
  An example of using SRUtil.Event library with callback interfaces.


*/
#include <iostream>
#include <srutil/event/event.hpp>
#include <boost/bind.hpp>
#include <boost/utility.hpp>

class Widget : boost::noncopyable
{
public:
	class MouseEventsListener;
	typedef srutil::event_source<MouseEventsListener*> MouseEventsSource;

	MouseEventsSource const & getMouseEventsSource() const {return mouseEventSource;}
	void Test();

private:
	MouseEventsSource mouseEventSource;
};

class Widget::MouseEventsListener : boost::noncopyable
{
public:
	virtual void onMouseMove(int x, int y) = 0;
	virtual void onMouseDown(int x, int y, int button) = 0;
	virtual void onMouseUp(int x, int y, int button) = 0;
};

class WidgetObserver : Widget::MouseEventsListener
{
public:
	WidgetObserver(Widget* widget)
	{
		binder.bind(widget->getMouseEventsSource(), this);
	}

private:
	Widget::MouseEventsSource::binder_type binder;

	void onMouseMove(int x, int y) {std::cout << "Mouse pointer moved to (" << x << ", " << y << ").\n";}
	void onMouseDown(int x, int y, int button) {std::cout << "Mouse button " << button << " pressed at (" << x << ", " << y << ").\n";}
	void onMouseUp(int x, int y, int button) {std::cout << "Mouse button " << button << " released at (" << x << ", " << y << ").\n";}
};

void Widget::Test()
{
	mouseEventSource.emit(boost::bind(&MouseEventsListener::onMouseDown, _1, 10, 10, 1));
	mouseEventSource.emit(boost::bind(&MouseEventsListener::onMouseMove, _1, 15, 15));
	mouseEventSource.emit(boost::bind(&MouseEventsListener::onMouseMove, _1, 20, 15));
	mouseEventSource.emit(boost::bind(&MouseEventsListener::onMouseMove, _1, 25, 15));
	mouseEventSource.emit(boost::bind(&MouseEventsListener::onMouseMove, _1, 25, 20));
	mouseEventSource.emit(boost::bind(&MouseEventsListener::onMouseUp, _1, 25, 20, 1));
}

int main()
{
	Widget widget;
	widget.Test(); // no listeners, nothing happened

	WidgetObserver observer(&widget);
	widget.Test(); // observer revieves events

	return 0;
}
