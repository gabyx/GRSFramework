#define SRUTIL_DELEGATE_PREFERRED_SYNTAX
#include <srutil/delegate/delegate.hpp>
using namespace srutil;

int main()
{
	delegate<void ()> d1; d1;
	delegate<int ()> d2; d2;
	delegate<void (int)> d3; d3;
	delegate<int (int)> d4; d4;
	delegate<void (int, long, float, double, void*)> d5; d5;
	return 0;
}
