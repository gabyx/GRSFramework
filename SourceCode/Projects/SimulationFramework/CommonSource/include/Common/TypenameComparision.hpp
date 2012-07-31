#ifndef TypenameComparision_hpp
#define TypenameComparision_hpp


struct FalseType { static const bool  value = false ; };
struct TrueType {  static const bool  value = true ; };


template <typename T1, typename T2>
struct IsSame
{
  static const bool result = false;
};


template <typename T>
struct IsSame<T,T>
{
static const bool result = true;
};



#endif