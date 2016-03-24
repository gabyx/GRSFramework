
#ifndef StringFormatterTest_hpp
#define StringFormatterTest_hpp

#include "TinyFormatInclude.hpp"

#include "GRSF/common/DemangleTypes.hpp"

#include <boost/filesystem/path.hpp>


 /** Light-weight argument storage which can be passed to tfm::vformat function to format with the format string */
    class VFormatList
    {
        public:

        VFormatList(){
            m_argStore.reserve(100);
        }

        template<typename T>
        void add(const T & value)
        {
            auto * p = new AnyT<T>(value);
            m_argStore.emplace_back(p);
        }

        void clear(){
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
                AnyT(const T& value) : value(value) {
                    std::cout << demangle::type<T>() << std::endl;
                }

                ~AnyT(){}
            };

            std::vector<std::unique_ptr<Any> > m_argStore;
    };

template<typename T>
struct StupidType{
    StupidType(const T & v):value(v){}
    T value;
    T getValue(){return value;}
};


void stringFormatterTest()
{
    VFormatList args;
//
//
//    StupidType<std::string> str("Frame");
//    StupidType<boost::filesystem::path> p("./output");
//
//    std::string s = p.getValue().string();
//    args.add(s);
//    args.add(s);
//
//
//    tfm::vformat(std::cout, "Fromatted: %s-%s\n", args);

     std::string str = "asdf";

    args.add(str);
    args.add(42.42);

    //tfm::vformat(std::cout, "%s : %.1f\n", args);

}

#endif // StringFormatterTest_hpp
