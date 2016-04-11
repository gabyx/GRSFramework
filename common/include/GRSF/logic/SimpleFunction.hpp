// ========================================================================================
//  GRSFramework 
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com> 
// 
//  This Source Code Form is subject to the terms of the GNU General Public License as 
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef GRSF_logic_SimpleFunction_hpp
#define GRSF_logic_SimpleFunction_hpp

#include <exprtk.hpp>

#include <type_traits>
#include <boost/mpl/contains.hpp>

#include "GRSF/logic/LogicNode.hpp"
#include "GRSF/common/SfinaeMacros.hpp"

namespace LogicNodes {

    /** EvalType is the type which is used in the internal expression evaluator, only one type can be used so far!
    *   Input sockets are converted to EvalType and EvalType needs to be assignable to OutType;
    */

    template<typename EvalType, typename OutType = EvalType>
    class SimpleFunction : public LogicNode {
    private:
        using TEval = EvalType;
        using  expression_t = exprtk::expression<TEval> ;
        exprtk::symbol_table<TEval> m_symbol_table;
        expression_t m_expression;
        exprtk::parser<TEval> m_parser;

        unsigned int m_nInputs = 0;

        using vector_holder_ptr = decltype(m_symbol_table.get_vector("in"));
        vector_holder_ptr m_vectorHolder; ///< Is not really used after add_vector, but keep it here, for safety
        std::vector<TEval> m_v;

        struct CastVisitor{
            CastVisitor(TEval * t): m_t(t){}

            // Only overload the types which can be cast to TParser
            // TODO meta magic, to generate list of types which can be casted to TParser!

            template<typename T,
                 SFINAE_ENABLE_IF( (!boost::mpl::contains< typename LogicTypes::TypeSeqArithmetic, T>::type::value) )
            >
            void operator()(LogicSocket<T> * n){
                 GRSF_ERRORMSG("Cannot cast type: " << LogicTypes::getTypeName<T>() << " to type: " << demangle::type<TEval>() );
            }

            template<typename T,
                SFINAE_ENABLE_IF(  (boost::mpl::contains< typename LogicTypes::TypeSeqArithmetic, T>::type::value) )
            >
            void operator()(LogicSocket<T> * n){
               *m_t = n->getValueRef(); // write to m_t;
            }

            TEval *m_t;
        };

    public:

        DEFINE_DYNAMICSSYTEM_CONFIG_TYPES

        struct Outputs {
            enum {
                Out,
                OUTPUTS_LAST
            };
        };

        DECLARE_OSOCKET_TYPE(Out, OutType );

        SimpleFunction(unsigned int id) : LogicNode(id)
        {
            ADD_OSOCK(Out,OutType());
        }

        /** Input sockets can be different and they will be converted to TEval for evaluation*/
        template<typename T>
        void addInput( T value = T() ){
             addISock<T>(value);
        }

        void compileExpression(std::string exprString){

            m_nInputs = getInputs().size();

            m_v.resize(m_nInputs);
            m_symbol_table.add_vector("i",m_v);

            m_expression.register_symbol_table(m_symbol_table);

            if(!m_parser.compile(exprString,m_expression)){
                std::stringstream ss;

                ss << "Error: %s" << m_parser.error() << ", Expression: " << exprString << std::endl;

                for (std::size_t i = 0; i < m_parser.error_count(); ++i)
                {
                     auto error = m_parser.get_error(i);

                     ss << Utilities::stringFormat("Error: %02d Position: %02d Type: [%14s] Msg: %s\tExpression: %s\n",
                            i, error.token.position, exprtk::parser_error::to_str(error.mode), error.diagnostic, exprString);
                }
                GRSF_ERRORMSG("Error in expression compilation: " << std::endl << ss.str())
            }

            // get internal reference to the vector
            m_vectorHolder = m_symbol_table.get_vector("i");
        }

        ~SimpleFunction(){}

        // No reset

        void compute() {

            // Every input socket is convertet to TEval
            for(unsigned int i = 0; i < m_nInputs; i++) {
                 getISocket(i)->applyVisitor(  CastVisitor{(*m_vectorHolder)[i]} );
            }

            SET_OSOCKET_VALUE(Out,static_cast<OutType>(m_expression.value()));
        }
    };

};

#endif
