#ifndef RECEIVERS_HPP_INCLUDED
#define RECEIVERS_HPP_INCLUDED

class Argument
{
	char data[64];

public:
	Argument()
	{
		data[0] = 0;
	}
};

class Receiver1
{
	int call_counter;
	static int static_call_counter;

	void on_method_invoked(int) {call_counter++;}
	static void on_static_method_invoked(int) {static_call_counter++;}

public:
	Receiver1() : call_counter(0) {}
	virtual ~Receiver1() {}

	int getCallCounter() const {return call_counter;}
	static int getStaticCallCounter() {return static_call_counter;}

	static void method1();
	static void method2() {on_static_method_invoked(2);}
	static Argument method3();
	static Argument method4() {on_static_method_invoked(4); return Argument();}

	void method5();
	void method6() {on_method_invoked(6);}
	Argument method7();
	Argument method8() {on_method_invoked(8); return Argument();}

	void method9(Argument);
	void method10(Argument) {on_method_invoked(10);}
	Argument method11(Argument);
	Argument method12(Argument) {on_method_invoked(12); return Argument();}

	void method13(Argument, Argument);
	void method14(Argument, Argument) {on_method_invoked(14);}
	Argument method15(Argument, Argument);
	Argument method16(Argument, Argument) {on_method_invoked(16); return Argument();}

	virtual void method17();
	virtual void method18() {on_method_invoked(18);}
	virtual Argument method19();
	virtual Argument method20() {on_method_invoked(20); return Argument();}

	virtual void method21(Argument);
	virtual void method22(Argument) {on_method_invoked(22);}
	virtual Argument method23(Argument);
	virtual Argument method24(Argument) {on_method_invoked(24); return Argument();}

	virtual void method25(Argument, Argument);
	virtual void method26(Argument, Argument) {on_method_invoked(26);}
	virtual Argument method27(Argument, Argument);
	virtual Argument method28(Argument, Argument) {on_method_invoked(28); return Argument();}

	void method29(void*, int, long);
	void method30(void*, int, long) {on_method_invoked(30);}
	bool method31(void*, int, long);
	bool method32(void*, int, long) {on_method_invoked(32); return true;}

	virtual void method33(void*, int, long);
	virtual void method34(void*, int, long) {on_method_invoked(34);}
	virtual bool method35(void*, int, long);
	virtual bool method36(void*, int, long) {on_method_invoked(36); return true;}
};

class FirstBase
{
public:
	virtual ~FirstBase() {}
	virtual void some_virtual_method() {}
};

class Receiver2 : public FirstBase, public Receiver1
{   
};

class Receiver3 : virtual public FirstBase, virtual public Receiver1
{
};

#endif//RECEIVERS_HPP_INCLUDED
