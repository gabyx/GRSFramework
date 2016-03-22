
PRO = rtnorm

CXX = clang
CXXFLAG = -W -Wall -std=c++11
LIB = -lm -lstdc++


SRC2 = main.cpp
OBJ2 = main.o

HDRS = src/rtnorm.hpp

all : $(PRO)

$(PRO): $(OBJ1) $(OBJ2)
	$(CXX) -o $(PRO) $(OBJ1) $(OBJ2)  $(LIB)

$(OBJ2) : $(SRC2) $(HDRS)
	$(CXX) -o $(OBJ2) -c $(SRC2) $(CXXFLAG)

clean : 
	rm -rf $(PRO) ./*.o
