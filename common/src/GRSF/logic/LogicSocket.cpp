
#include "GRSF/logic/LogicSocket.hpp"
#include "GRSF/logic/LogicNode.hpp"
#include <typeinfo>


void LogicSocketBase::link(LogicSocketBase *fsock)
{

	if(!fsock){
        ERRORMSG("Socket to link to is null!");
	}

	if(!(this->m_type == fsock->m_type)){
        ERRORMSG( " Types of sockets do not coincide: "
                 << LogicTypes::getTypeName(this->m_type) << " and " << LogicTypes::getTypeName(fsock->m_type)
                 << " for tool id: " << this->m_parent->m_id  << " and " << fsock->m_parent->m_id);
	}

	if(m_isInput)
	{
	    if(m_from){
            ERRORMSG("Only one link for input socket");
	    }
	    if(fsock->m_isInput){
            ERRORMSG("Cannot link input to input");
	    }

		m_from = fsock;
	}
	else
	{
	    if(!fsock->m_isInput){
            ERRORMSG("Cannot link output to output");
	    }
		if(std::find(m_to.begin(),m_to.end(),fsock) == m_to.end() )
		{
			m_to.push_back(fsock);
		}
	}

	fsock->m_connected = m_connected = true;

	if (m_parent)
	{
		m_parent->setLinked();
		LogicNode *nd = fsock->getParent();
		if (nd){
                nd->setLinked();
		}
		else{
		    ERRORMSG("This socket has no parent");
		}
	}else{
        ERRORMSG("This socket has no parent");
	}
}



