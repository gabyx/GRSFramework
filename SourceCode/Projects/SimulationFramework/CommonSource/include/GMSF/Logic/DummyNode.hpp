#ifndef GMSF_Logic_DummyNode_hpp
#define GMSF_Logic_DummyNode_hpp

#include "GMSF/Logic/LogicNode.hpp"

template<unsigned int NIN, unsigned int NOUT>
class DummyLogicNode : public LogicNode {
public:

    DummyLogicNode(unsigned int id) : LogicNode(id) {
        for(unsigned int i=0; i<NIN; i++) {
            addISock(0.0);
        }
        for(unsigned int i=0; i<NOUT; i++) {
            addOSock(0.0);
        }
    }

    virtual ~DummyLogicNode() {}

    void compute() {
        double t = 0;
        for(unsigned int i=0; i<NIN; i++) {
            t += getISocketValue<double>(i);
        }
        std::cout << m_id << ": " << t << std::endl;
        for(unsigned int i=0; i<NOUT; i++) {
            setOSocketValue(i,t+1);
        };
    }

};
#endif
