#ifndef StringFormatNode_hpp
#define StringFormatNode_hpp

#include "CommonFunctions.hpp"

#include "LogicNode.hpp"

namespace LogicNodes{

    template<typename T>
    class StringFormatNode: public LogicNode {
    public:

        struct Inputs {
            enum {
                In,
                Format,
                INPUTS_LAST
            };
        };

        struct Outputs {
            enum {
                String,
                OUTPUTS_LAST
            };
        };

        enum {
            N_INPUTS  = Inputs::INPUTS_LAST,
            N_OUTPUTS = Outputs::OUTPUTS_LAST - Inputs::INPUTS_LAST,
            N_SOCKETS = N_INPUTS + N_OUTPUTS
        };

        DECLARE_ISOCKET_TYPE(In, T );
        DECLARE_ISOCKET_TYPE(Format, std::string );
        DECLARE_OSOCKET_TYPE(String, std::string );


        StringFormatNode(unsigned int id, std::string format) : LogicNode(id) {
            ADD_ISOCK(In,T());
            ADD_ISOCK(Format,format);
            ADD_OSOCK(String,"");
        }

        virtual ~StringFormatNode() {
        }

        virtual void compute(){
            SET_OSOCKET_VALUE(String, Utilities::stringFormat( GET_ISOCKET_REF_VALUE(Format), GET_ISOCKET_REF_VALUE(In) ) );
        }
        virtual void initialize(){}
    };
};

#endif // StringFormatNode_hpp



