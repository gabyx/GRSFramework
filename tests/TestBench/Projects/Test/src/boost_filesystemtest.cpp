// ========================================================================================
//  GRSFramework 
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com> 
// 
//  This Source Code Form is subject to the terms of the GNU General Public License as 
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#include <iostream>
    #include <boost/filesystem/operations.hpp>
    #include <boost/filesystem/fstream.hpp>
    using namespace boost::filesystem;
    using namespace std;
     
    void show_files( const path & directory, bool recurse_into_subdirs = true )
    {
    if( exists( directory ) )
    {
    directory_iterator end ;
    for( directory_iterator iter(directory) ; iter != end ; ++iter )
    if ( is_directory( *iter ) )
    {
    cout << iter->path().string() << " (directory)\n" ;
    if( recurse_into_subdirs ) show_files(*iter) ;
    }
    else
    cout << iter->path().string() << " (file)\n" ;
    }
    }
     
    int main()
    {
      show_files( "C:\\cygwin\\home\\ZfMGPU" ) ;
      system("pause");
    }