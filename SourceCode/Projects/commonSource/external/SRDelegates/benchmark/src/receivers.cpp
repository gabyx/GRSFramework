#include "receivers.hpp"

int Receiver1::static_call_counter = 0;

void Receiver1::method1()
{
	on_static_method_invoked(1);
}

Argument Receiver1::method3()
{
	on_static_method_invoked(3);
	return Argument();
}

void Receiver1::method5()
{
	on_method_invoked(5);
}

Argument Receiver1::method7()
{
	on_method_invoked(7);
	return Argument();
}

void Receiver1::method9(Argument)
{
  on_method_invoked(9);
}

Argument Receiver1::method11(Argument)
{
	on_method_invoked(11);
	return Argument();
}

void Receiver1::method13(Argument, Argument)
{
	on_method_invoked(13);
}

Argument Receiver1::method15(Argument, Argument)
{
	on_method_invoked(15);
	return Argument();
}

void Receiver1::method17()
{
	on_method_invoked(17);
}

Argument Receiver1::method19()
{
	on_method_invoked(19);
	return Argument();
}

void Receiver1::method21(Argument)
{
	on_method_invoked(21);
}

Argument Receiver1::method23(Argument)
{
	on_method_invoked(23);
	return Argument();
}

void Receiver1::method25(Argument, Argument)
{
	on_method_invoked(25);
}

Argument Receiver1::method27(Argument, Argument)
{
	on_method_invoked(27);
	return Argument();
}

void Receiver1::method29(void*, int, long)
{
	on_method_invoked(29);
}

bool Receiver1::method31(void*, int, long)
{
	on_method_invoked(31);
	return true;
}

void Receiver1::method33(void*, int, long)
{
	on_method_invoked(23);
}

bool Receiver1::method35(void*, int, long)
{
	on_method_invoked(35);
	return true;
}
