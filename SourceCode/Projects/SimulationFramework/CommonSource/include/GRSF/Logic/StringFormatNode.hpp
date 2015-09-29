#ifndef GRSF_Logic_StringFormatNode_hpp
#define GRSF_Logic_StringFormatNode_hpp

#include <type_traits>
#include <boost/mpl/contains.hpp>

#include "TinyFormatInclude.hpp"
#include "GRSF/Common/DemangleTypes.hpp"
#include "GRSF/Common/CommonFunctions.hpp"

#include "GRSF/Logic/LogicNode.hpp"

namespace LogicNodes{

    class StringFormatNode: public LogicNode {

    private:

    /** Light-weight argument storage which can be passed to tfm::vformat function to format with the format string */
    class VFormatList
    {
        public:
            /**
                Two versions of add(), to avoid copying and storing the value where possible.
            */
            template<typename T,
                    typename std::enable_if<std::is_lvalue_reference<T&>::value>::type* = nullptr
                    >
            void add(const T& value)
            {
                //std::cout << "added reference" << std::endl;
                m_argList.emplace_back(value);
            }

            /** save interal as we have received an universal reference, but only rvalue reference binds to this overload
            *   as we have an overload for lvalue references
            */
            template<typename T,
                    typename std::enable_if<std::is_rvalue_reference<T&&>::value>::type* = nullptr
                    >
            void add(T && value)
            {
                //std::cout << "copied" << std::endl;
                auto * p = new AnyT<T>(value);
                m_argStore.emplace_back( p );
                m_argList.emplace_back(p->value);
            }

            /** Cast to FormatList */
            operator tfm::FormatList()
            {
                return tfm::FormatList(m_argList.data(), m_argList.size());
            }

            /** Clear all arguments  (deletes all allocated memory) */
            void clear(){
                m_argList.clear();
                m_argStore.clear();
            }

        private:
            struct Any {
                virtual ~Any(){};
            };

            template<typename T>
            struct AnyT : Any
            {
                T value;
                AnyT(const T& value) : value(value) {}
                ~AnyT(){}
            };

            std::vector<tfm::detail::FormatArg> m_argList;
            std::vector<std::unique_ptr<Any>> m_argStore;
    };


    // Visitor for adding to Format List
    struct FormatListAdder{
        FormatListAdder(VFormatList & formatList): m_fList(formatList) {}

        // Only overload the types which can be used with the StringFormater
        // Types which are not in TypeSeqBasic
        template<typename T,
                 typename std::enable_if< ! boost::mpl::contains< typename LogicTypes::TypeSeqBasic, T>::type::value
                                        >::type * = nullptr
                >
        void operator()(LogicSocket<T> * n){
            ERRORMSG("Input type: " << LogicTypes::getTypeName<T>() << " cannot be used in StringFormatNode!");
        }

        // Types which are in TypeSeqBasic
        template<typename T,
                 typename std::enable_if<  boost::mpl::contains< typename LogicTypes::TypeSeqBasic, T >::type::value
                                        >::type * = nullptr
                >
        void operator()(LogicSocket<T> * n){
            //std::cout << "value: " <<n->getValue()<< std::endl;
            m_fList.add(n->getRefValue());
        }

        // Types which are in TypeSeqBasic
        void operator()(LogicSocket<boost::filesystem::path> * n){
            //std::cout << "value path: " <<n->getValue()<< std::endl;
            m_fList.add(n->getRefValue().string());
        }


        VFormatList & m_fList;
    };

    struct ValueSetter{
        ValueSetter(std::stringstream * p): m_s(p){};
        // Only overload the types to which a std::string can be assigned
        template<typename T,
                 typename std::enable_if< ! boost::mpl::contains< typename LogicTypes::TypeSeqStringAssignable, T>::type::value
                                        >::type * = nullptr
        >
        void operator()(LogicSocket<T> * n){
             ERRORMSG("Output type: " << LogicTypes::getTypeName<T>() << " cannot be used in StringFormatNode!");
        }

        template<typename T,
                 typename std::enable_if< boost::mpl::contains< typename LogicTypes::TypeSeqStringAssignable, T>::type::value
                                        >::type * = nullptr
        >
        void operator()(LogicSocket<T> * n){
           n->setValue(m_s->str());
        }

        std::stringstream * m_s;
    };

    public:

        struct Inputs {
            enum {
                Format,
                INPUTS_LAST
            };
        };

        struct Outputs {
            enum {
                OUTPUTS_LAST
            };
        };

         enum {
            N_INPUTS  = Inputs::INPUTS_LAST,
            N_OUTPUTS = Outputs::OUTPUTS_LAST,
        };

        DECLARE_ISOCKET_TYPE(Format, std::string );

        StringFormatNode(unsigned int id, std::string format) : LogicNode(id), m_adder(m_formatList), m_setter(&m_s) {
            ADD_ISOCK(Format,format);
        }

        virtual ~StringFormatNode() {
        }

        template<typename T>
        void addInput( T value = T() ){
             addISock<T>(value);
        }

        template<typename T>
        void addOutput( T value = T() ){
             addOSock<T>(value);
        }

        virtual void compute(){

            m_s.str("");
            m_formatList.clear();

            //Iterate over all inputs and add to format_list with visitor
            auto & ins = getInputs();
            auto s = ins.size();
            for(auto i = 1; i < s;++i){
                ins[i]->applyVisitor(m_adder);
            }

            // Convert the format string with the format list
            //std::cout << "Format: " << GET_ISOCKET_REF_VALUE(Format) << std::endl;
            try{
                tfm::vformat(m_s, GET_ISOCKET_REF_VALUE(Format).c_str(), m_formatList);
            }catch(...){
                ERRORMSG("Conversion of string in tool " << this->m_id << " failed!")
            }

            // Write to all output
            for(auto * out : getOutputs()){
                out->applyVisitor(m_setter);
            }
        }

        protected:
        std::stringstream m_s;
        private:
        VFormatList m_formatList;
        FormatListAdder m_adder;
        ValueSetter m_setter;

    };
};

#endif // StringFormatNode_hpp



