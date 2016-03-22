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