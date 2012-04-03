BOOST_NO_EXCEPTIONS
namespace boost
{
	void throw_exception(std::exception const &) {}
}

#include <stdio.h>
#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <srutil/event/event.hpp>
#include <srutil/delegate/delegate.hpp>

typedef boost::function0<void> TheDelegate;
typedef srutil::delegate_invoker0<void> TheInvoker;
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

	binder1.bind(g_source, &test_function1);
	binder2.bind(g_source, &test_function2);
	binder3.bind(g_source, &test_function3);

	g_source.emit(TheInvoker());

	return 0;
}
