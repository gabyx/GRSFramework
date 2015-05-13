#ifndef GRSF_Dynamics_General_MultiBodySimFile_hpp
#define GRSF_Dynamics_General_MultiBodySimFile_hpp

#include <type_traits>
#include <set>
#include <fstream>

#include <boost/filesystem.hpp>

#include "GRSF/Common/ContainerTag.hpp"

#include "GRSF/Common/StaticAssert.hpp"
#include "GRSF/Dynamics/General/RigidBodyId.hpp"
#include "GRSF/Dynamics/Buffers/DynamicsState.hpp"

#include "GRSF/Dynamics/General/MultiBodySimFileIOHelpers.hpp"

#include "GRSF/Dynamics/General/AdditionalBodyData.hpp"


/**
* @ingroup Common
* @defgroup MultiBodySimFile
* @{
*/

/**
* @brief Defines for the signature length of the .sim file.
*/
#define SIM_FILE_SIGNATURE_LENGTH 4
/**
* @brief Defines for the signature of the .sim file.
*/
#define SIM_FILE_SIGNATURE {'M','B','S','F'}

#define SIM_FILE_VERSION 2

/**
* @brief Defines the extentsion of the file.
*/
#define SIM_FILE_EXTENSION ".sim"

/**
* @brief Defines the extentsion of the file for an initial sim file.
*/
#define SIM_INIT_FILE_EXTENSION ".sim"

class SimFileJoiner; ///< Friend declaration inside of MultiBodySimFile
class SimFileResampler;


/**
* @brief This is the input output class for writting a binary multi body system file to the disk.
* It is used to save all simulated data.
* The layout of the document is as follows:
* - The first #SIM_FILE_SIGNATURE_LENGTH bytes are for the signature of the file.
* - 3 doubles: nSimBodies, nDOFqBody, nDofuBody.
* - DynamicsState:
*     - double --> Time in [seconds]
*       - all states for all nSimBodies
*         - ...
*         - nDOFqBody doubles --> generalized coordinates of all bodies.
*         - nDOFuBody doubles --> generalized velocities of all bodies.
*         - ...
*/
class MultiBodySimFile {
    friend class SimFileJoiner;
    friend class SimFileResampler;
    MAKE_ADDITIONALBODYDATA_FRIEND
public:


    MultiBodySimFile(unsigned int bufferSize = 1<<14);
    ~MultiBodySimFile();

    /**
    * @brief Opens a .sim file for read only.
    * @param file_path The path to the file to open.
    * @param readVelocities If the full
    * @param nSimBodies The number of bodies which should be included in the file.
    * @return true if the file is successfully opened and readable and false if not.
    * @detail if nSimBodies = 0, the sim file does not need to match nSimBodies
    */
    bool openRead(const boost::filesystem::path & file_path,
                  bool readVelocities = true,
                  unsigned int nDOFqBody = 0,
                  unsigned int nDOFuBody = 0,
                  unsigned int nSimBodies = 0);
    /**
    * @brief Opens a .sim file for write only.
    * @param file_path The path to the file to open.
    * @param nSimBodies The number of bodies which should be included in the file.
    * @return true if the file is successfully opened and writable and false if not.
    */
    bool openWrite(const boost::filesystem::path & file_path,
                   unsigned int nDOFqBody,
                   unsigned int nDOFuBody,
                   unsigned int nSimBodies,
                   bool truncate = true);

    /**
    * @brief Closes the .sim file which was opened by an openWrite or openRead command.
    */
    void close();

    /**
    * @brief Checks if there are still dynamics states to read or not.
    * @return true if there are still dynamics states to read, false if the file end has been reached.
    */
    inline bool isGood();

    bool writeTimeListToFile(const boost::filesystem::path & f);

    /**
    * @brief Write all states of the bodies to the file, writes position and velocity!
    */
    template<typename TRigidBodyContainer>
    inline void write(double time, const TRigidBodyContainer & bodyList);

    /**
    * @brief Writes all bodies from begin to end (Iterator  points to a RigidBody pointer) to the file!
    */
    template<typename TBodyIterator>
    void write(double time, TBodyIterator begin, TBodyIterator end);

    /**
    * @brief Reads in the state at the given time and if the id in states is found, the state with id in states is overwritten.
    * @param which is 0 for begin state, 1 for current state given at time, 2 for end state
    * @param time is the input time if variable which is 1, and the output time if variable which is 0 or 2
    * @tparam TBodyStateMap, is a std::map or any hashed map with mapped_type: RigidBodyState
    * @detail If time is in between two states in the sim file, then the next bigger is taken!
    *         If the time is greater then the maximum time, then this time is taken.
    */
    template<typename BodyStateContainer>
    inline bool readSpecific(BodyStateContainer & states,
                     double & stateTime,
                     bool readPos = true,
                     bool readVel= true,
                     short which = 2,
                     bool onlyUpdate = true) {
        return readSpecific_impl(states,stateTime,readPos,readVel,which,onlyUpdate);
    }


    template< typename BodyStateContainer>
    void read(  BodyStateContainer & states, double & time) {
        read_impl(states,time);
    }

    /**
    * @brief Operator to write a state to the file, writes position and velocity!
    */

    inline MultiBodySimFile & operator << (const DynamicsState* state);
    /**
    * @brief Overlaod!
    */

    inline MultiBodySimFile & operator << (const DynamicsState& state) {
        return this->operator<<(&state);
    }

    /**
    * @brief Operator to read a state from a file.
    */
    inline MultiBodySimFile & operator >> (DynamicsState* state);
    /**
    * @brief Overlaod!
    */

    inline MultiBodySimFile &  operator>>( DynamicsState & state ) {
        return this->operator>>(&state);
    }

    /**
    * @brief Dumps all data from file to this sim file.
    */
    MultiBodySimFile & operator << (MultiBodySimFile& file);

    /** Moves the read pointer \p off states in the direction specified by \p way  (analog to std::fstream::seekg)*/
    inline void seekgStates(std::streamoff off, std::ios_base::seekdir way = std::ios_base::cur);

    //  /**
    //  * @brief Gets the state at the time t.
    //  */
    //bool getStateAt(DynamicsState& state, PREC t);
    /**
    * @brief Gets the end state.
    */
    inline void getEndState(DynamicsState& state);

    inline std::string getErrorString(){
        m_errorString << " strerror: " << std::strerror(errno) << std::endl;
        return m_errorString.str();
    }

    unsigned int getNDOFq() const {
        return m_nDOFqBody;
    }
    unsigned int getNDOFu() const {
        return m_nDOFuBody;
    }
    unsigned int getNSimBodies() const {
        return m_nSimBodies;
    }
    std::streamsize getNStates() const{
        return m_nStates;
    }

    std::streamsize getBytesPerState() const{
        return m_nBytesPerState;
    }

    using TimeListType = std::vector<double>;
    TimeListType getTimeList();

    /** Simple structure to return from getDetails() */
    struct Details{
        std::streamsize m_nBytes;
        std::streamsize m_nStates;
        boost::filesystem::path m_filePath;
        std::streamsize m_headerLength;
        std::streamsize m_nBytesPerState;
        unsigned int m_nSimBodies;
        unsigned int m_nDOFuBody, m_nDOFqBody;
        std::streamsize m_nBytesPerQBody;
        std::streamsize m_nBytesPerUBody;
        std::streamsize m_nBytesPerBody;
        typename AdditionalBodyData::TypeEnum m_additionalBytesPerBodyType;
        std::streamsize m_nAdditionalBytesPerBody;
        bool m_readVelocities;
        TimeListType m_times;

        inline std::string getString(std::string linePrefix="\t"){
            std::stringstream s;
            s <<linePrefix << "Simfile: "<< m_filePath << std::endl
              << linePrefix << "\t nBytes: " << m_nBytes << std::endl
              << linePrefix << "\t nSimBodies: " << m_nSimBodies << std::endl
              << linePrefix << "\t nDOFqBody: "  << m_nDOFqBody << std::endl
              << linePrefix << "\t nDOFuBody: "  << m_nDOFuBody << std::endl
              << linePrefix << "\t nStates: " << m_nStates << std::endl
              << linePrefix << "\t nBytesPerState: " << m_nBytesPerState << std::endl
              << linePrefix << "\t nBytesPerBody: " << m_nBytesPerBody << std::endl
              << linePrefix << "\t nBytesPerQBody: " << m_nBytesPerQBody << std::endl
              << linePrefix << "\t nBytesPerUBody: " << m_nBytesPerUBody << std::endl
              << linePrefix << "\t addBytesBodyType: " << EnumConversion::toIntegral(m_additionalBytesPerBodyType) << std::endl
              << linePrefix << "\t nAdditionalBytesPerBody: " << m_nAdditionalBytesPerBody << std::endl
              << linePrefix << "\t readVelocities: " << m_readVelocities << std::endl
              << linePrefix<< "\t TimeList: [ ";

            if(m_times.size()>2){
                s << m_times[0] << "," << m_times[1] << ", ... , " << m_times.back();
            }else if(m_times.size()==2){
                s << m_times[0] << "," << m_times[1] ;
            }else if(m_times.size()==1){
                s << m_times[0];
            }
            s  << " ]" << std::endl;
            return s.str();
        }

         /** Appends the details to the XML node for the sim files */
        template<typename XMLNodeType>
        XMLNodeType addXML(XMLNodeType & node, bool timeList = false){
             auto s = node.append_child("SimFile");
             s.append_attribute("path").set_value(m_filePath.string().c_str());
             s.append_attribute("nBytes").set_value((long long int)m_nBytes);
             s.append_attribute("nSimBodies").set_value((long long int)m_nSimBodies);
             s.append_attribute("nDOFqBody").set_value((long long int)m_nDOFqBody);
             s.append_attribute("nDOFuBody").set_value((long long int)m_nDOFuBody);
             s.append_attribute("nStates").set_value((long long int)m_nStates);
             s.append_attribute("nBytesPerState").set_value((long long int)m_nBytesPerState);
             s.append_attribute("nBytesPerBody").set_value((long long int)m_nBytesPerBody);
             s.append_attribute("nBytesPerQBody").set_value((long long int)m_nBytesPerQBody);
             s.append_attribute("nBytesPerUBody").set_value((long long int)m_nBytesPerUBody);
             s.append_attribute("addBytesBodyType").set_value(EnumConversion::toIntegral(m_additionalBytesPerBodyType));
             s.append_attribute("nAdditionalBytesPerBody").set_value((long long int)m_nAdditionalBytesPerBody);
             s.append_attribute("readVelocities").set_value(m_readVelocities);

            auto t = s.append_attribute("timeList");
            std::stringstream ss;
            if(!m_times.empty()){
                auto l = m_times.size()-1;
                for(std::size_t i = 0; i<l;++i){
                    ss << m_times[i] << " ";
                }
                ss << m_times[l];
            }
            t.set_value(ss.str().c_str());

            return s;
        }
    };

    inline Details getDetails(bool timeList = false){
        Details d{m_nBytes,m_nStates,m_filePath,m_headerLength,m_nBytesPerState,m_nSimBodies,m_nDOFuBody,
                  m_nDOFqBody,m_nBytesPerQBody,m_nBytesPerUBody,
                  m_nBytesPerBody,m_additionalBytesPerBodyType,m_nAdditionalBytesPerBody,m_readVelocities};
        if(timeList){
            d.m_times = getTimeList(); // moves assign list
        }
        return d;
    }

private:

    bool openWrite_impl(const boost::filesystem::path & file_path,
                        unsigned int nDOFqBody,
                        unsigned int nDOFuBody,
                        unsigned int nSimBodies,
                        bool truncate = true,
                        AdditionalBodyData::TypeEnum additionalBytesType = AdditionalBodyData::TypeEnum::NOTHING,
                        std::streamsize additionalBytesPerBody = AdditionalBodyData::getAdditionalBytesPerBody(AdditionalBodyData::TypeEnum::NOTHING)
                        );


    template<typename C,
             typename std::enable_if< ContainerTags::is_associative(*static_cast<C*>(0)), int >::type  = 0,
             typename std::enable_if<std::is_same<typename C::mapped_type, RigidBodyState>::value, int >::type = 0
             >
    bool readSpecific_impl(C & states,
                   double & stateTime,
                   bool readPos,
                   bool readVel,
                   short which,
                   bool onlyUpdate);

    template< typename C,
              typename std::enable_if< ContainerTags::is_container(*static_cast<C*>(0)), int >::type = 0,
              typename std::enable_if< std::is_same<typename C::value_type, RigidBodyStateAdd>::value, int >::type = 0
              >
    void read_impl(  C & states, double & time);

    template<bool readVelocity = true, bool skipAddBytes = true>
    inline void readBodyState( RigidBodyState * state);


    inline void readBodyStateAdd( RigidBodyStateAdd * state);

    /**
    * @brief Operator to write a generic value to the file as binary data.
    */
    template<typename T>
    inline MultiBodySimFile & operator << (const T &value);
    /**
    * @brief Operator to read a generic value from a .sim file as binary data.
    */
    template<typename T>
    inline MultiBodySimFile & operator >> (T &value);


    std::fstream m_file_stream;                      ///< The file stream which represents the binary data.
    unsigned int m_buf_size;                         ///< The internal buffer size.
    char * m_Buffer;                                 ///< The buffer.

    static const char m_simFileSignature[SIM_FILE_SIGNATURE_LENGTH]; ///< The .sim file header.

    /**
    * @brief Writes the header to the file which has been opened.
    */
    void writeHeader();
    /**
    * @brief Reads the header from .sim file.
    * @return true if header has been read successfully.
    */
    bool readHeader();
    /**
    * @brief Reads in all the length of the .sim file.
    * @{
    */
    bool readLength();
    std::streamsize m_nBytes;
    std::streamsize m_nStates;
    /** @}*/

    /** @brief File path */
    boost::filesystem::path m_filePath;

    /** @brief Header length */
    static const  std::streamsize m_headerLength = SIM_FILE_SIGNATURE_LENGTH*sizeof(char)
            + sizeof(unsigned int)
            +3*sizeof(unsigned int) +2*sizeof(unsigned int) ; ///< 'MBSF' + nBodies, NDOFq, NDOFu, additionalBytesType (0=nothing, 1 = + process rank, etc.), additionalBytesPerBody



    /** @brief Determined from number of bodies! @{*/
    void setByteLengths();
    std::streamsize m_nBytesPerState; ///< m_nSimBodies*(q,u) + time
    unsigned int m_nSimBodies;
    /** @}*/

    std::streampos m_beginOfStates;

    unsigned int m_nDOFuBody, m_nDOFqBody;
    std::streamsize m_nBytesPerQBody;
    std::streamsize m_nBytesPerUBody;
    std::streamsize m_nBytesPerBody;

    // Write addditional bytes per body, not yet implemented, but the type is written in the header
    typename AdditionalBodyData::TypeEnum m_additionalBytesPerBodyType;

    std::streamsize getAdditionalBytesPerBody();
    std::streamsize m_nAdditionalBytesPerBody;

    bool m_readVelocities;

    std::stringstream m_errorString;

    // Copy constructor is private, we should never copy the file, because fstream is not copiable
    MultiBodySimFile & operator =(const MultiBodySimFile & file);


};
/** @} */





template<typename T>
MultiBodySimFile & MultiBodySimFile::operator<<( const T &value ) {
    m_file_stream.write(
        reinterpret_cast<char const *>(&value),
        sizeof(value)
    );
    return *this;
};


template<typename T>
MultiBodySimFile & MultiBodySimFile::operator>>( T &value ) {
    m_file_stream.read(
        reinterpret_cast<char *>(&value),
        sizeof(value)
    );
    return *this;
};

template<typename TRigidBodyContainer>
void MultiBodySimFile::write(double time, const TRigidBodyContainer & bodyList) {
    write( time, bodyList.beginOrdered(),bodyList.endOrdered());
}

template<typename TBodyIterator>
void MultiBodySimFile::write(double time, TBodyIterator begin, TBodyIterator end) {
    *this << time;
    ASSERTMSG(m_nSimBodies == std::distance(begin,end),"You try to write "<< std::distance(begin,end)
              <<"bodies into a file which was instanced to hold "<<m_nSimBodies);

    using BodyType = typename std::remove_reference<decltype(*(*begin))>::type;

    STATIC_ASSERTM((std::is_same<double, typename BodyType::PREC>::value),"OOPS! TAKE CARE if you compile here, SIM files can only be read with the PREC precision!")
    auto itEnd = end;
    for(auto it = begin; it != itEnd; ++it) {
        *this << (*it)->m_id;
        IOHelpers::writeBinary(m_file_stream, (*it)->get_q());
        IOHelpers::writeBinary(m_file_stream, (*it)->get_u());

//        AddBytes::write<m_additionalBytesPerBodyType>(m_file_stream);
    }
}


MultiBodySimFile &  MultiBodySimFile::operator<<( const DynamicsState* state ) {


    ASSERTMSG(m_nSimBodies == state->getNSimBodies(),
              "You try to write "<<state->getNSimBodies()<<"bodies into a file which was instanced to hold "<<m_nSimBodies);

    // write time
    *this << (double)state->m_t;
    // write states
    for(auto & b : state->m_SimBodyStates) {
        STATIC_ASSERTM((std::is_same<double, typename DynamicsState::PREC>::value),
                       "OOPS! TAKE CARE if you compile here, SIM files can only be read with the PREC precision!")
        *this << b.m_id;
        IOHelpers::writeBinary(m_file_stream, b.m_q );
        IOHelpers::writeBinary(m_file_stream, b.m_u );

//        AddBytes::write<m_additionalBytesPerBodyType>(m_file_stream);
    }

    m_nStates++;

    return *this;
}


MultiBodySimFile &  MultiBodySimFile::operator>>( DynamicsState* state ) {
    auto nSimBodies = state->getNSimBodies();
    ASSERTMSG(m_nSimBodies == nSimBodies,
              "You try to read "<<m_nSimBodies<<"bodies into a state which was instanced to hold "<<nSimBodies);

    // write time
    *this >> (double &)state->m_t;
    //std::cout << "m_t:"<< state->m_t <<std::endl;

    state->m_StateType = DynamicsState::NONE;
    // write states
    RigidBodyIdType id;
    for(unsigned int i=0 ; i< nSimBodies; i++) {

        *this >> id;
        //std::cout<< "MSIMFILE id: " << RigidBodyId::getBodyIdString(id)  << std::endl;
        auto * pState = state->getSimState(id);
        if(pState) {
            readBodyState<>(pState);
        } else {
            ERRORMSG("State for body id: " << id << "not found in DynamicsState")
            //m_file_stream.seekg(m_nBytesPerQBody+m_nBytesPerUBody+m_nAdditionalBytesPerBody ,std::ios_base::cur);
        }

    }
    return *this;
}


/** C is a RigidBodyState map */
template<
    typename C,
    typename std::enable_if< ContainerTags::is_associative(*static_cast<C*>(0) ), int >::type,
    typename std::enable_if<std::is_same<typename C::mapped_type, RigidBodyState>::value, int >::type
    >
bool MultiBodySimFile::readSpecific_impl(C & states,
                                 double & stateTime,
                                 bool readPos,
                                 bool readVel,
                                 short which,
                                 bool onlyUpdate) {
    m_errorString.str("");

    std::set<RigidBodyIdType> updatedStates;
    double currentTime = 0;
    double lastTime =-1;

    RigidBodyState * pState = nullptr;
    RigidBodyIdType id;
    bool timeFound = false;

    m_file_stream.seekg(m_beginOfStates);

    if(stateTime < 0.0) {
        stateTime = 0.0;
    }

    if(which == 0) {
        *this >> stateTime;

        timeFound = true;
    } else if(which == 1) {
        //Scan all states
        for( std::streamoff stateIdx = 0; stateIdx < m_nStates; stateIdx++) {
            *this >> currentTime;
            //std::cout << "time: " << currentTime << "lastTime: " << lastTime << "stateTime: " << stateTime<< std::endl;
            if( ( lastTime < stateTime && stateTime <= currentTime )|| stateTime <= 0.0) {
                timeFound = true;
                break;
            } else {
                lastTime = currentTime;
                if(stateIdx < m_nStates-1) {
                    m_file_stream.seekg( m_nBytesPerState - sizeof(double) ,std::ios_base::cur);
                }
            }
        }
        stateTime = currentTime;
    } else if(which == 2) {
        m_file_stream.seekg( (m_nStates-1)*m_nBytesPerState ,std::ios_base::cur);
        *this >> stateTime;
        timeFound = true;
    }

    if(timeFound) {

        ASSERTMSG(stateTime>=0.0, " Found state time is " << stateTime);
        // Current time found!
        for(unsigned int body = 0; body < m_nSimBodies; body++) {
            *this >> id;

            //std::cout << RigidBodyId::getBodyIdString(id) << std::endl;

            if(onlyUpdate) {
                auto res = states.find(id);
                if(res != states.end()) {
                    pState = &res->second;
                } else {
                    pState = nullptr;
                }
            } else {
                auto res = states[id];
                pState = &res;
            }

            // State found
            if(pState != nullptr ) {
                // Read in state:

                if(readVel){
                    readBodyState<true>(pState);
                }else{
                    readBodyState<false>(pState);
                }

                updatedStates.insert(id);

            } else {
                // State not found
                // Jump body (u and q and additional Data)
                m_file_stream.seekg( m_nBytesPerQBody + m_nBytesPerUBody + m_nAdditionalBytesPerBody ,std::ios_base::cur);
            }


        }

        m_file_stream.seekg(m_beginOfStates);

        if(onlyUpdate) {
            if(updatedStates.size() != states.size()) {
                m_errorString << "Some states have not been updated!; updated states: " << updatedStates.size() <<  std::endl;

                return false;
            }
        }
        return true;
    }

    m_file_stream.seekg(m_beginOfStates);

    m_errorString << "The time type: " << which << " (time: " <<stateTime <<") was not found" << std::endl;

    return false;
}


/** C is a RigidBodyState map */
template<
    typename C,
    typename std::enable_if< ContainerTags::is_container(*static_cast<C*>(0)), int >::type ,
    typename std::enable_if< std::is_same<typename C::value_type, RigidBodyStateAdd>::value, int >::type
    >
void MultiBodySimFile::read_impl(  C & states, double &time) {


    //ASSERTMSG(states.size() == m_nSimBodies,"Number of SimBodies: " << m_nSimBodies << " does not coincide with the number of states")

    states.resize(m_nSimBodies);

    // write time
    *this >> (double &)time;

    for(auto & s : states) {
        *this >> s.m_id;
        readBodyStateAdd(&s);
    }


}

void MultiBodySimFile::seekgStates(std::streamoff off, std::ios_base::seekdir way){
    switch(way){
        case std::ios_base::cur:
            // streamsize and streamoff are both signed!
            m_file_stream.seekg(m_nBytesPerState*off,way);
            break;
        case std::ios_base::beg:
            off *= m_nBytesPerState;
            if(off>=0){
               m_file_stream.seekg(m_beginOfStates);
               m_file_stream.seekg(off,std::ios_base::cur);
            }else{
               m_file_stream.setstate(std::ios_base::failbit);
            }
            break;

        case std::ios_base::end :
            // check if off is negative !
            if(off<=0){
                off = (m_nStates-1 + off)*m_nBytesPerState;
                m_file_stream.seekg(m_beginOfStates);
                m_file_stream.seekg(off ,std::ios_base::cur);
            }else{
               m_file_stream.setstate(std::ios_base::failbit);
            }
            break;
        default:
            break;
    }
    return;
}

bool MultiBodySimFile::isGood(){
    if(m_file_stream.good()) {
        // check if a state can still be read, and if we are not inside header!
        auto g = m_file_stream.tellg();
        if( (m_nBytes - g ) >=  m_nBytesPerState && g >= m_headerLength ) {
            return true;
        }
    }
    return false;
}


template<bool readVelocity, bool skipAddBytes>
void MultiBodySimFile::readBodyState( RigidBodyState * s) {

    IOHelpers::readBinary(m_file_stream,  s->m_q );
    // std::cout<< "MSIMFILE q: " << s->m_q.transpose()  << std::endl;
    if(readVelocity) {
        IOHelpers::readBinary(m_file_stream,  s->m_u );
        //  std::cout<< "MSIMFILE u: " << s->m_u.transpose()  << std::endl;
    } else {
        //Dont read in velocities, its not needed!
        m_file_stream.seekg(m_nBytesPerUBody,std::ios_base::cur);
    }
    if(skipAddBytes) {
        // Dont read in additional bytes
        m_file_stream.seekg(m_nAdditionalBytesPerBody,std::ios_base::cur);
    }
}

void MultiBodySimFile::readBodyStateAdd( RigidBodyStateAdd * s) {

    readBodyState<true,false>(s);

    s->m_data = AdditionalBodyData::createNew(s->m_data,m_additionalBytesPerBodyType);

    s->m_data->read(*this);
}


void MultiBodySimFile::getEndState(DynamicsState& state) {
    m_file_stream.seekg(m_beginOfStates);
    m_file_stream.seekg( (m_nStates-1)*m_nBytesPerState ,std::ios_base::cur);
    this->operator>>(&state);
    m_file_stream.seekg(m_beginOfStates);
}





#endif

