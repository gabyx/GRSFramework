
#ifndef GMSF_Logic_LogicCommon_hpp
#define GMSF_Logic_LogicCommon_hpp

#include <vector>

#include "TypeDefs.hpp"

#include LogicTypes_INCLUDE_FILE

class LogicSocketBase;

namespace LogicSocketCommon{

    using SocketListType = std::vector<LogicSocketBase*>;
    using SocketIterator = typename SocketListType::iterator;

};


#endif//LogicCommon_hpp
