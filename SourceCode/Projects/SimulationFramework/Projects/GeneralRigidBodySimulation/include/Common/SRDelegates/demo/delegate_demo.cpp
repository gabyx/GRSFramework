/*
A simple example of using delegates with prefered syntax.

Copyright 
*/

#include <iostream>
//#define SRUTIL_DELEGATE_PREFERRED_SYNTAX
#include <srutil/delegate/delegate.hpp>

#ifdef SRUTIL_DELEGATE_PREFERRED_SYNTAX
typedef srutil::delegate<void (int, int)> TestDelegate;
#else
typedef srutil::delegate2<void, int, int> TestDelegate;
#endif

void f(int, int) {std::cout << "f invoked.\n";}
class TestClass
{
public:
	void m1(int, int) {std::cout << "m1 invoked for object " << this << ".\n";}
	void m2(int, int) const {std::cout << "m2 invoked for object " << this << ".\n";}
	static void m3(int, int) {std::cout << "m3 invoked.\n";}
};

void test(TestDelegate d)
{
	if (d)
	{
		std::cout << "Direct invoking... ";
		d(5, 10);

		std::cout << "Invoking through invoker... ";
		TestDelegate::invoker_type inv(5, 10);
		inv(d);
	}

	if (!d)
		std::cout << "The delegate is empty.\n";
}

int main()
{
	TestClass obj;

	test(TestDelegate());
	test(TestDelegate::from_function<&f>());
	test(TestDelegate::from_method<TestClass, &TestClass::m1>(&obj));
	test(TestDelegate::from_const_method<TestClass, &TestClass::m2>(&obj));
	test(TestDelegate::from_function<&TestClass::m3>());
	return 0;
}
