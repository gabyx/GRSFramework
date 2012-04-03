#include <stdio.h>
#include <time.h>

#include "Receivers.hpp"
#include "Invoker.hpp"
#include "FastDelegate.h"
#include <srutil/delegate/delegate.hpp>

using namespace fastdelegate;
using namespace srutil;

extern int Zero;

template <typename T>
T prevent_optimization(T& d)
{
	return *(&d + Zero);
}

template <class TInvoker, class TDelegate>
int measurement(TInvoker invoker, TDelegate d)
{
	const int iteration_count = 100000000;
	const int delegates_count = 10;
	TDelegate delegates[delegates_count];

	clock_t before = clock();
	for (TDelegate *j = delegates; j != delegates + delegates_count; ++j)
		*j = prevent_optimization(d);

	for (int i = 0; i != iteration_count / delegates_count; ++i)
	{
		for (TDelegate *j = delegates; j != delegates + delegates_count; ++j)
			invoker(*j);
	}
	clock_t after = clock();
	return (int)(after - before);
}

template <class TInvoker, class TFastDelegate, class TSRDelegate>
void compare_delegates(int method_id, int receiver_id,
			TInvoker invoker, TFastDelegate fd, TSRDelegate sd)
{
	printf("Test %d.%d: ", method_id, receiver_id);

	int fd_time = measurement(invoker, fd);
	int sd_time = measurement(invoker, sd);

	double fdsd_rate = fd_time != 0 ? (double)sd_time / fd_time : -1.0;
	printf("FD: %d; SD: %d\t(%g)\n", fd_time, sd_time, fdsd_rate);	
}

#define DO_BANCHMARK_STATIC_0(METHOD_ID, RETURN_TYPE) compare_delegates(METHOD_ID, 1, \
		Invoker0<RETURN_TYPE>(), \
		FastDelegate0<RETURN_TYPE>(Receiver1::method##METHOD_ID), \
		delegate0<RETURN_TYPE>::from_function<Receiver1::method##METHOD_ID>() \
	);

#define DO_BANCHMARK_0_IMPL(METHOD_ID,RECEIVER_NUM,RETURN_TYPE) compare_delegates(METHOD_ID, RECEIVER_NUM, \
		Invoker0<RETURN_TYPE>(), \
		FastDelegate0<RETURN_TYPE>(&r##RECEIVER_NUM, &Receiver##RECEIVER_NUM::method##METHOD_ID), \
		delegate0<RETURN_TYPE>::from_method<Receiver1,&Receiver##RECEIVER_NUM::method##METHOD_ID>(&r##RECEIVER_NUM) \
	);

#define DO_BANCHMARK_0(METHOD_ID,RETURN_TYPE) \
		DO_BANCHMARK_0_IMPL(METHOD_ID,1,RETURN_TYPE) \
		DO_BANCHMARK_0_IMPL(METHOD_ID,2,RETURN_TYPE) \
		DO_BANCHMARK_0_IMPL(METHOD_ID,3,RETURN_TYPE)

#define DO_BANCHMARK_1_IMPL(METHOD_ID,RECEIVER_NUM,RETURN_TYPE,ARG1) compare_delegates(METHOD_ID, RECEIVER_NUM, \
		Invoker1<RETURN_TYPE,ARG1>(), \
		FastDelegate1<ARG1,RETURN_TYPE>(&r##RECEIVER_NUM, &Receiver##RECEIVER_NUM::method##METHOD_ID), \
		delegate1<RETURN_TYPE,ARG1>::from_method<Receiver1,&Receiver##RECEIVER_NUM::method##METHOD_ID>(&r##RECEIVER_NUM) \
	);


#define DO_BANCHMARK_1(METHOD_ID,RETURN_TYPE, ARG1) \
		DO_BANCHMARK_1_IMPL(METHOD_ID,1,RETURN_TYPE,ARG1) \
		DO_BANCHMARK_1_IMPL(METHOD_ID,2,RETURN_TYPE,ARG1) \
		DO_BANCHMARK_1_IMPL(METHOD_ID,3,RETURN_TYPE,ARG1)

#define DO_BANCHMARK_2_IMPL(METHOD_ID,RECEIVER_NUM,RETURN_TYPE,ARG1,ARG2) compare_delegates(METHOD_ID, RECEIVER_NUM, \
		Invoker2<RETURN_TYPE,ARG1,ARG2>(), \
		FastDelegate2<ARG1,ARG2,RETURN_TYPE>(&r##RECEIVER_NUM, &Receiver##RECEIVER_NUM::method##METHOD_ID), \
		delegate2<RETURN_TYPE,ARG1,ARG2>::from_method<Receiver1,&Receiver##RECEIVER_NUM::method##METHOD_ID>(&r##RECEIVER_NUM) \
	);

#define DO_BANCHMARK_2(METHOD_ID,RETURN_TYPE,ARG1,ARG2) \
		DO_BANCHMARK_2_IMPL(METHOD_ID,1,RETURN_TYPE,ARG1,ARG2) \
		DO_BANCHMARK_2_IMPL(METHOD_ID,2,RETURN_TYPE,ARG1,ARG2) \
		DO_BANCHMARK_2_IMPL(METHOD_ID,3,RETURN_TYPE,ARG1,ARG2)

#define DO_BANCHMARK_3_IMPL(METHOD_ID,RECEIVER_NUM,RETURN_TYPE,ARG1,ARG2,ARG3) compare_delegates(METHOD_ID, RECEIVER_NUM, \
		Invoker3<RETURN_TYPE,ARG1,ARG2,ARG3>(), \
		FastDelegate3<ARG1,ARG2,ARG3,RETURN_TYPE>(&r##RECEIVER_NUM, &Receiver##RECEIVER_NUM::method##METHOD_ID), \
		delegate3<RETURN_TYPE,ARG1,ARG2,ARG3>::from_method<Receiver1,&Receiver##RECEIVER_NUM::method##METHOD_ID>(&r##RECEIVER_NUM) \
	);

#define DO_BANCHMARK_3(METHOD_ID,RETURN_TYPE,ARG1,ARG2,ARG3) \
		DO_BANCHMARK_3_IMPL(METHOD_ID,1,RETURN_TYPE,ARG1,ARG2,ARG3) \
		DO_BANCHMARK_3_IMPL(METHOD_ID,2,RETURN_TYPE,ARG1,ARG2,ARG3) \
		DO_BANCHMARK_3_IMPL(METHOD_ID,3,RETURN_TYPE,ARG1,ARG2,ARG3)

int main()
{
	Receiver1 r1;
	Receiver1 r2;
	Receiver1 r3;

	//DO_BANCHMARK_STATIC_0(1, void)
	DO_BANCHMARK_STATIC_0(2, void)
	DO_BANCHMARK_STATIC_0(3, Argument)
	DO_BANCHMARK_STATIC_0(4, Argument)

	DO_BANCHMARK_0(5, void)
	DO_BANCHMARK_0(6, void)
	DO_BANCHMARK_0(7, Argument)
	DO_BANCHMARK_0(8, Argument)

	DO_BANCHMARK_1(9, void, Argument)
	DO_BANCHMARK_1(10, void, Argument)
	DO_BANCHMARK_1(11, Argument, Argument)
	DO_BANCHMARK_1(12, Argument, Argument)

	DO_BANCHMARK_2(13, void, Argument, Argument)
	DO_BANCHMARK_2(14, void, Argument, Argument)
	DO_BANCHMARK_2(15, Argument, Argument, Argument)
	DO_BANCHMARK_2(16, Argument, Argument, Argument)

	DO_BANCHMARK_0(17, void)
	DO_BANCHMARK_0(18, void)
	DO_BANCHMARK_0(19, Argument)
	DO_BANCHMARK_0(20, Argument)

	DO_BANCHMARK_1(21, void, Argument)
	DO_BANCHMARK_1(22, void, Argument)
	DO_BANCHMARK_1(23, Argument, Argument)
	DO_BANCHMARK_1(24, Argument, Argument)

	DO_BANCHMARK_2(25, void, Argument, Argument)
	DO_BANCHMARK_2(26, void, Argument, Argument)
	DO_BANCHMARK_2(27, Argument, Argument, Argument)
	DO_BANCHMARK_2(28, Argument, Argument, Argument)

	DO_BANCHMARK_3(29, void, void*, int, long)
	DO_BANCHMARK_3(30, void, void*, int, long)
	DO_BANCHMARK_3(31, bool, void*, int, long)
	DO_BANCHMARK_3(32, bool, void*, int, long)

	DO_BANCHMARK_3(33, void, void*, int, long)
	DO_BANCHMARK_3(34, void, void*, int, long)
	DO_BANCHMARK_3(35, bool, void*, int, long)
	DO_BANCHMARK_3(36, bool, void*, int, long)

	return 0;
}

int Zero = 0;
