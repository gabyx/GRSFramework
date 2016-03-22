#ifndef MoveSemantics_hpp
#define MoveSemantics_hpp



struct Data{
    Data(int a):i(a){
        std::cout << " CTOR of " << i<< std::endl;
    };

    Data(const Data & data){
        std::cout << " COPY CTOR of " << data.i << "to" << i << std::endl;
        i = data.i;
    }

    Data(Data && data){
        std::cout << " MOVE CTOR" << data.i << " to " << i << std::endl;
        i = data.i;
    }

    Data & operator=(Data & data){
        //data = lvalue
        std::cout << " ASSIGN of " << data.i << "to" << i << std::endl;
        i = data.i;
        return *this;
    }

    Data & operator=(Data && data){
        //data = lvalue
        std::cout << " MOVE ASSIGN of " << data.i << "to" << i << std::endl;
        i = data.i;
        return *this;
    }


    Data & operator+=(int a){
        std::cout << " += of value " << a << " to " << i << std::endl;
        i=i+a;
        return *this;
    }

    int i;
};

struct Holder{
    Holder(): a(10) {};
    Data a;
    Data get(){return a;}
};

Data && returnModifiedCopy(const Data & data){
    // Make a copy, thats what this function does!
    Data a = data;
    a+=1;
    return std::move(a);
    //dangling reference!
}

Data returnModifiedCopy2(const Data & data){
    // Make a copy, thats what this function does!
    Data a = data;
    a+=1;
    return a;
    //return by value, explicit say
}

#endif // MoveSemantics_hpp
