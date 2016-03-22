#ifndef StripModifiers_hpp
#define StripModifiers_hpp


/** Recursively strips away all cv-qualifiers */

template<typename Type>
struct StripModifiers
{
	typedef Type type;
};

template<typename Type>
struct StripModifiers<const Type>
{
	typedef typename StripModifiers<Type>::type type;
};

template<typename Type>
struct StripModifiers<volatile Type>
{
	typedef typename StripModifiers<Type>::type type;
};

template<typename Type>
struct StripModifiers<const volatile Type>
{
	typedef typename StripModifiers<Type>::type type;
};

template<typename Type>
struct StripModifiers<Type*>
{
	typedef typename StripModifiers<Type>::type type;
};

template<typename Type>
struct StripModifiers<Type&>
{
	typedef typename StripModifiers<Type>::type type;
};

template<typename Type>
struct StripModifiers<Type&&>
{
	typedef typename StripModifiers<Type>::type type;
};


#endif
