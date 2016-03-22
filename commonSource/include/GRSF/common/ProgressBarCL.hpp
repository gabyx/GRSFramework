#ifndef GRSF_Common_ProgressBarCL_hpp
#define GRSF_Common_ProgressBarCL_hpp


/*
*  A stupid commando interface progress bar!
*/
template <typename T>
class ProgressBarCL{
public:

    ProgressBarCL(std::ostream & s,
                  std::string title,
                  T start,
                  T end, unsigned int length = 20
                  ):m_oStream(s)
    {
        m_start = start;
        m_end   = end;

        m_length = length;
        m_title = title;
    }

    void start(){
        m_current = 0;
        m_oStream << m_title <<" [";



    }
    //   start                end
    //    340        341      342  ] // can be continous!
    //     | ---------|--------|     * this is the rest
    //     | = | = | = | = | = | (l=1)
    //       1   2   3   4   5
    //   c |-------->*
    // c = 340.8 - 340 = 0.8
    // i = (end -start) / l
    // idx = floor(c / i)  = 2 there should be the progress bar

    void update(unsigned int counter){

        double c = counter - m_start;
        double i = (double)(m_end-m_start) / (double)(m_length);
        unsigned int idx;
        if(i <= 0.0){
            idx = m_length;
        }else{
            idx = std::floor(c / i) + 1;
            idx = std::min(idx, m_length);
        }

        // pop out character till idx
        while(m_current < idx){
            m_oStream << "=";
            m_current++;
        }

        if(m_current == m_length){
            m_oStream << "]" << std::endl;
        }
    }


private:
    std::string m_title;

    T m_start, m_end;
    unsigned int m_length, m_current;

    std::ostream & m_oStream;
};



#endif // ProgressBarCL_hpp


