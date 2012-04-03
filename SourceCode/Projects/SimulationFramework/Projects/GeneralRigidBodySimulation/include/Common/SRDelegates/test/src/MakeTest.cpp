#include <srutil/delegate/delegate.hpp>
#include <assert.h>
using namespace srutil;

int call_count = 0;

class TestClass
{
public:
	static void s0() {++call_count;}
	void f0() {++call_count;}
	void c0() const {++call_count;}

	static int s1() {++call_count;return 0;}
	int f1() {++call_count;return 0;}
	int c1() const {++call_count;return 0;}

	static void s2(int) {++call_count;}
	void f2(int) {++call_count;}
	void c2(int) const {++call_count;}

	static int s3(int) {++call_count;return 0;}
	int f3(int) {++call_count;return 0;}
	int c3(int) const {++call_count;return 0;}

	static void s4(int, long, float, double, void*) {++call_count;}
	void f4(int, long, float, double, void*) {++call_count;}
	void c4(int, long, float, double, void*) const {++call_count;}
};

void s0f() {++call_count;}
int s1f() {++call_count; return 0;}
void s2f(int) {++call_count;}
int s3f(int) {++call_count; return 0;}
void s4f(int, long, float, double, void*) {++call_count;}


int main()
{
	TestClass obj;

	typedef delegate0<void> D0;
	typedef delegate0<int> D1;
	typedef delegate1<void, int> D2;
	typedef delegate1<int, int> D3;
	typedef delegate5<void, int, long, float, double, void*> D4;

	D0::from_function<s0f>()();
	D0::from_function<TestClass::s0>()();
	D0::from_method<TestClass, &TestClass::f0>(&obj)();
	D0::from_const_method<TestClass, &TestClass::c0>(&obj)();

	assert(call_count == 4);

	int sum = 0;
	sum += D1::from_function<s1f>()();
	sum += D1::from_function<TestClass::s1>()();
	sum += D1::from_method<TestClass, &TestClass::f1>(&obj)();
	sum += D1::from_const_method<TestClass, &TestClass::c1>(&obj)();

	assert(call_count == 8);

	D2::from_function<s2f>()(sum);
	D2::from_function<TestClass::s2>()(sum);
	D2::from_method<TestClass, &TestClass::f2>(&obj)(sum);
	D2::from_const_method<TestClass, &TestClass::c2>(&obj)(sum);

	assert(call_count == 12);

	sum += D3::from_function<s3f>()(sum);
	sum += D3::from_function<TestClass::s3>()(sum);
	sum += D3::from_method<TestClass, &TestClass::f3>(&obj)(sum);
	sum += D3::from_const_method<TestClass, &TestClass::c3>(&obj)(sum);

	assert(call_count == 16);

	D4::from_function<s4f>()(sum, 1, 2.0, 3.0, (void*)0);
	D4::from_function<TestClass::s4>()(sum, 1, 2.0, 3.0, (void*)0);
	D4::from_method<TestClass, &TestClass::f4>(&obj)(sum, 1, 2.0, 3.0, (void*)0);
	D4::from_const_method<TestClass, &TestClass::c4>(&obj)(sum, 1, 2.0, 3.0, (void*)0);

	assert(call_count == 20);

	return 0;
}

