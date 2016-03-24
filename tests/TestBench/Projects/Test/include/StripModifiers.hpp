// ========================================================================================
//  GRSFramework 
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com> 
// 
//  This Source Code Form is subject to the terms of the GNU General Public License as 
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

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
