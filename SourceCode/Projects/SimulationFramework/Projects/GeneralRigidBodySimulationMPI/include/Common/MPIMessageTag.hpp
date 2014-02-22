#ifndef MPIMessageTags_hpp
#define MPIMessageTags_hpp

namespace MPILayer{
    enum class MPIMessageTag: unsigned int {
        GENERICMESSAGE = 1 << 0,
        STDSTRING =      1 << 1,
        BODY_MESSAGE =   1 << 2,
        EXTERNALCONTACTS_MESSAGE = 1 << 3,
        SPLITBODYFACTOR_MESSAGE  = 1 << 4,
        SPLITBODYUPDATE_MESSAGE  = 1 << 5,
        SPLITBODYSOLUTION_MESSAGE = 1<< 6
    };
};

#endif
