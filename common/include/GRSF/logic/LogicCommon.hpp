
#ifndef GRSF_logic_LogicCommon_hpp
#define GRSF_logic_LogicCommon_hpp

#include <vector>

#include "GRSF/common/TypeDefs.hpp"

#include LogicTypes_INCLUDE_FILE

class LogicSocketBase;

namespace LogicSocketCommon{

    using SocketListType = std::vector<LogicSocketBase*>;
    using SocketIterator = typename SocketListType::iterator;

};


#endif//LogicCommon_hpp
