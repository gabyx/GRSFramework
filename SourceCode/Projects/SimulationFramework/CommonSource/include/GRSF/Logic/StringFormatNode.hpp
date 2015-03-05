#ifndef GRSF_Logic_StringFormatNode_hpp
#define GRSF_Logic_StringFormatNode_hpp

#include <type_traits>
#include <boost/mpl/contains.hpp>

#include "TinyFormatInclude.hpp"
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
            template<typename T>
            void add(const T& value)
            {
                m_argList.emplace_back(value);
            }

            /** save interal as we have received an universal reference, but only rvalue reference binds to this overload
            *   as we have an overload for lvalue references
            */
            template<typename T>
            void add(const T&& value)
            {
                m_argStore.emplace_back(new AnyT<T>(std::move(value)) );
                const T& storedValue = static_cast<AnyT<T>&>(*m_argStore.back()).value;
                m_argList.emplace_back(storedValue);
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
            struct Any { };

            template<typename T>
            struct AnyT : Any
            {
                T value;
                AnyT(const T& value) : value(value) { }
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
            ERRORMSG("Type: " << LogicTypes::getTypeName<T>() << " cannot be used in StringFormatNode!");
        }

        // Types which are in TypeSeqBasic
        template<typename T,
                 typename std::enable_if<  boost::mpl::contains< typename LogicTypes::TypeSeqBasic, T >::type::value
                                        >::type * = nullptr
                >
        void operator()(LogicSocket<T> * n){
            m_fList.add(n->getValue());
        }

        // Types which are in TypeSeqBasic
        void operator()(LogicSocket<boost::filesystem::path> * n){
            m_fList.add(n->getValue().string());
        }


        VFormatList & m_fList;
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
                String,
                OUTPUTS_LAST
            };
        };

         enum {
            N_INPUTS  = Inputs::INPUTS_LAST,
            N_OUTPUTS = Outputs::OUTPUTS_LAST,
        };

        DECLARE_ISOCKET_TYPE(Format, std::string );
        DECLARE_OSOCKET_TYPE(String, std::string );

        StringFormatNode(unsigned int id, std::string format) : LogicNode(id), m_adder(m_formatList) {
            ADD_ISOCK(Format,format);
            ADD_OSOCK(String,"");
        }

        virtual ~StringFormatNode() {
        }

        template<typename T>
        void addInput( T value = T() ){
             addISock<T>(value);
        }

        virtual void compute(){

            m_s.str("");
            m_formatList.clear();

            //Iterate over all inputs and add to format_list with visitor
            auto & inList =  getInputs();
            for(unsigned int i=1; i <inList.size(); i++){
                inList[i]->applyVisitor(m_adder);
            }


            // Convert the format string with the format list
            try{
                tfm::vformat(m_s, GET_ISOCKET_REF_VALUE(Format).c_str(), m_formatList);
            }catch(...){
                ERRORMSG("Conversion of string in tool " << this->m_id << " failed!")
            }
            SET_OSOCKET_VALUE(String, m_s.str());
        }
        virtual void initialize(){}

        private:
        VFormatList m_formatList;
        FormatListAdder m_adder;

        protected:
        std::stringstream m_s;
    };
};

#endif // StringFormatNode_hpp



