#include <srutil/delegate/delegate.hpp>
using namespace srutil;

int main()
{
	delegate0<void> d1; d1;
	delegate0<int> d2; d2;
	delegate1<void, int> d3; d3;
	delegate1<int, int> d4; d4;
	delegate5<void, int, long, float, double, void*> d5; d5;

	return 0;
}
