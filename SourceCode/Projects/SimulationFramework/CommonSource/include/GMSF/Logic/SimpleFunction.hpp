#ifndef GMSF_Logic_SimpleFunction_hpp
#define GMSF_Logic_SimpleFunction_hpp

#include <exprtk.hpp>

#include "GMSF/Logic/LogicNode.hpp"

namespace LogicNodes {


    template<typename InType, typename OutType = InType>
    class SimpleFunction : public LogicNode {
    private:
        using ParserT = double;
        using  expression_t = exprtk::expression<ParserT> ;
        exprtk::symbol_table<ParserT> m_symbol_table;
        expression_t m_expression;
        exprtk::parser<ParserT> m_parser;

        unsigned int m_inputs;
    public:

        DEFINE_DYNAMICSSYTEM_CONFIG_TYPES

        struct Outputs {
            enum {
                Out,
                OUTPUTS_LAST
            };
        };

        DECLARE_OSOCKET_TYPE(Out, OutType );

        SimpleFunction(unsigned int id, unsigned int inputs, std::string exprString) : LogicNode(id) {
            m_inputs = inputs;
            for(unsigned int i = 0; i < m_inputs; i++) {
                addISock<InType>(InType());
                //std::cout <<"Variables: " << std::string("in")+std::to_string(i) << std::endl;
                m_symbol_table.create_variable(std::string("in")+std::to_string(i));
            }

            ADD_OSOCK(Out,OutType());

            m_expression.register_symbol_table(m_symbol_table);

            m_parser.compile(exprString,m_expression);

        }

        ~SimpleFunction() {}

        // No initialization

        void compute() {
            for(unsigned int i = 0; i < m_inputs; i++) {
                m_symbol_table.get_variable(std::string("in")+std::to_string(i))->ref() =
                    static_cast<ParserT>(getISocketRefValue<InType>(i));
            }
            //std::cout << "Value: " << m_expression.value() << std::endl;
            SET_OSOCKET_VALUE(Out,static_cast<OutType>(m_expression.value()));
        }

    };

};

#endif
