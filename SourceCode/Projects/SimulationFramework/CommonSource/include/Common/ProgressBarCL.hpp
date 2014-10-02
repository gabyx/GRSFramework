#ifndef ProgressBarCL_hpp
#define ProgressBarCL_hpp


/*
*  A stupid commando interface progress bar!
*/
template <typename T>
class ProgressBarCL{
public:

    ProgressBarCL(std::ostream & s,
                  std::string title,
                  T end, unsigned int length = 20
                  ):m_oStream(s)
    {
        m_end=end;
        m_length = length;
        m_title = title;
    }

    void start(){
        m_current = 0;
        m_oStream << m_title <<" [";
    }
    void update(unsigned int counter){

        T idx = std::min( (T)(((double)counter / (double)m_end) * (double)m_length) , m_length-1  );

        if(idx>m_current && m_current < m_length){
            for(T i = 0; i < idx-m_current; i++ ){
                m_oStream << "=";
            }
            m_current = idx;
        }
        if(m_current == m_length -1){
            m_oStream << "=]" <<std::endl;
            m_current = m_length;
        }
    }


private:
    std::string m_title;

    T m_current;
    T m_end;
    T m_length;


    std::ostream & m_oStream;
};



#endif // ProgressBarCL_hpp


