#ifndef INVOKER_HPP_INCLUDED
#define INVOKER_HPP_INCLUDED

template <typename R>
struct Invoker0
{
	template <class TDelegate>
	R operator()(TDelegate d) {return d();}
};

template <typename R, typename A1>
struct Invoker1
{
	A1 a1;

	template <class TDelegate>
	R operator()(TDelegate d) {return d(a1);}
};

template <typename R, typename A1, typename A2>
struct Invoker2
{
	A1 a1;
	A2 a2;

	template <class TDelegate>
	R operator()(TDelegate d) {return d(a1, a2);}
};

template <typename R, typename A1, typename A2, typename A3>
struct Invoker3
{
	A1 a1;
	A2 a2;
	A3 a3;

	template <class TDelegate>
	R operator()(TDelegate d) {return d(a1, a2, a3);}
};

#endif//INVOKER_HPP_INCLUDED
