#ifndef GRSF_general_LogicParserBaseTraitsMacro_hpp
#define GRSF_general_LogicParserBaseTraitsMacro_hpp


#define DEFINE_LOGICPARSER_BASE_TYPE_TRAITS( TParserTraits ) \
    using ParserType = typename TParserTraits::ParserType; \
    using DataStorageType = typename TParserTraits::DataStorageType; \
    using LogType = typename TParserTraits::LogType; \
    using XMLNodeType = typename TParserTraits::XMLNodeType;\
    using XMLNodeItType = typename TParserTraits::XMLNodeItType;\
    using XMLAttributeType = typename TParserTraits::XMLAttributeType;


#endif
