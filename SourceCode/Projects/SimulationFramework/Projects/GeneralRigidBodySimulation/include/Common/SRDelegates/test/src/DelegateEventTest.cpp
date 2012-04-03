#include <stdio.h>
#include <srutil/event/event.hpp>
#include <srutil/delegate/delegate.hpp>

typedef srutil::delegate0<void> TheDelegate;
typedef TheDelegate::invoker_type TheInvoker;
typedef srutil::event_source<TheDelegate> TestEventSource;

TestEventSource g_source;

void test_function1() {printf("Test function 1 called\n");}
void test_function2() {printf("Test function 2 called\n");}
void test_function3() {printf("Test function 3 called\n");}


int main()
{
	TestEventSource::binder_type binder1;
	TestEventSource::binder_type binder2;
	TestEventSource::binder_type binder3;

	binder1.bind(g_source, TheDelegate::from_function<&test_function1>());
	binder2.bind(g_source, TheDelegate::from_function<&test_function2>());
	binder3.bind(g_source, TheDelegate::from_function<&test_function3>());

	g_source.emit(TheInvoker());

	return 0;
}
