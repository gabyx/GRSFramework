#ifndef StringFormatNode_hpp
#define StringFormatNode_hpp

#include "CommonFunctions.hpp"

#include "LogicNode.hpp"

namespace LogicNodes{

    class StringFormatNode: public LogicNode {

    private:
        // Visitor
        struct VisitorConv{
            std::stringstream m_s;
            std::string * m_f = nullptr;


            void reset(){
                m_s.str("");
            }

            void setFormat(std::string * s){
                m_f = s;
            }

            template<typename T>
            void operator()(LogicSocket<T> * n){
                m_s << Utilities::stringFormat(*m_f,n->getValue()) ;
            }

//            template<typename T>
//            void operator()(LogicSocket<T> * n){
//                ERRORMSG("Cannot convert input of type: " << LogicTypes::getTypeName<T>() + " to string, not implemented!" );
//            }

        };

    public:


        struct Outputs {
            enum {
                String,
                OUTPUTS_LAST
            };
        };

        enum {
            N_OUTPUTS = Outputs::OUTPUTS_LAST
        };

        DECLARE_OSOCKET_TYPE(String, std::string );

        StringFormatNode(unsigned int id) : LogicNode(id) {
            ADD_OSOCK(String,"");
        }

        virtual ~StringFormatNode() {
        }

        template<typename T>
        void addInputAndFormatSocket(std::string format, T def = T()){
            addISock<T>(def);
            addISock<std::string>(format);
        }

        virtual void compute(){
            static VisitorConv conv;

            conv.reset();

            //Iterate over all inputs and apply converter visitor
            auto & inList =  getInputs();
            for(unsigned int i=0; i <inList.size(); i=i+2){
                conv.setFormat(  &getISocketRefValue<std::string>(i+1) );
                inList[i]->applyVisitor(conv);
            }
            std::cout << conv.m_s.str() << std::endl;
            SET_OSOCKET_VALUE(String, conv.m_s.str() );
        }
        virtual void initialize(){}
    };
};

#endif // StringFormatNode_hpp



