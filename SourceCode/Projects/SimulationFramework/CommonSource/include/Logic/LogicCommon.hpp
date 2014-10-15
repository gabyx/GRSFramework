
#ifndef LogicCommon_hpp
#define LogicCommon_hpp

#include <vector>

class LogicSocketBase;

namespace LogicSocketCommon{

    using SocketListType = std::vector<LogicSocketBase*>;
    using SocketIterator = typename SocketListType::iterator;

};


#endif//LogicCommon_hpp
