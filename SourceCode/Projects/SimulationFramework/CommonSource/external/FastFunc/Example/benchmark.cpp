


int tempGlobalVar{0};
bool tempGlobalState{false};
int rawFunc(int& xx, int i) { xx+=i; tempGlobalState = !tempGlobalState; return i + 1; }
struct TestStruct { void rawMemFunc(int& xx) const { tempGlobalState = !tempGlobalState; xx += 10; } };
constexpr int numberOfTests{1};
constexpr int repeats{500000000};

int main()
{
    string tempStr{"hello from testequal (capture)"};
    FastFunc<void()> testEqual = nullptr;
    FastFunc<void()> testEqual2 = nullptr;
    if(testEqual == nullptr) testEqual = []{ lo << "hello from testequal (trivial)" << endl; };
    testEqual();
    if(testEqual != nullptr) testEqual = [&tempStr]{ lo << tempStr << endl; };
    testEqual(); testEqual = nullptr;
    if(testEqual == nullptr) testEqual = []{ lo << "hello from testequal (trivial)" << endl; };
    testEqual();
    if(testEqual != nullptr) testEqual = [&tempStr]{ lo << tempStr << endl; };
    testEqual();
    testEqual = testEqual2;
    testEqual2 = [&tempStr]{ lo << tempStr << endl; };
    testEqual = testEqual2;
    testEqual();
    testEqual2();
    testEqual2 = [&tempStr]{ lo << "new" + tempStr << endl; };
    testEqual = testEqual2;
    testEqual();
    testEqual2();


    int tempVar{0};
    TestStruct testStr;
    for(int nTest = 0; nTest < numberOfTests; ++nTest)
    {
        {
            startBenchmark(); { for(int i = 0; i < repeats; ++i) rawFunc(tempVar, i); } lo << lt("raw func") << endBenchmark() << endl;
        }
        {
            function<int(int&, int)> t1 = &rawFunc;
            startBenchmark(); { for(int i = 0; i < repeats; ++i) t1(tempVar, i); } lo << lt("raw func std::func") << endBenchmark() << endl;
        }
        {
            FastFunc<int(int&, int)> t1 = &rawFunc;
            startBenchmark(); { for(int i = 0; i < repeats; ++i) t1(tempVar, i); } lo << lt("raw func don_delegate") << endBenchmark() << endl;
        }
        {
            delegate<int(int&, int)> t1 = &rawFunc;
            startBenchmark(); { for(int i = 0; i < repeats; ++i) t1(tempVar, i); } lo << lt("raw func staticdelegate") << endBenchmark() << endl;
        } lo << endl;

        {
            startBenchmark(); { for(int i = 0; i < repeats; ++i) testStr.rawMemFunc(tempVar); } lo << lt("raw memfunc") << endBenchmark() << endl;
        }
        {
            function<void()> t1 = [&]{ testStr.rawMemFunc(tempVar); };
            startBenchmark(); { for(int i = 0; i < repeats; ++i) t1(); } lo << lt("raw memfunc  std::func") << endBenchmark() << endl;
        }
        {
            FastFunc<void(int&)> t1(&testStr, &TestStruct::rawMemFunc);
            startBenchmark(); { for(int i = 0; i < repeats; ++i) t1(tempVar); } lo << lt("raw memfunc  don_delegate") << endBenchmark() << endl;
        }
        {
            delegate<void()> t1 = [&]{ testStr.rawMemFunc(tempVar); };
            startBenchmark(); { for(int i = 0; i < repeats; ++i) t1(); } lo << lt("raw memfunc  staticdelegate") << endBenchmark() << endl;
        } lo << endl;

        {
            auto t1 = [](int i){ tempGlobalState = !tempGlobalState; return tempGlobalVar + i + 1; };
            startBenchmark(); { for(int i = 0; i < repeats; ++i) t1(i); } lo << lt("trivial auto") << endBenchmark() << endl;
        }
        {
            function<int(int)> t1 = [](int i){ tempGlobalState = !tempGlobalState; return tempGlobalVar + i + 1; };
            startBenchmark(); { for(int i = 0; i < repeats; ++i) t1(i); } lo << lt("trivial std::func") << endBenchmark() << endl;
        }
        {
            FastFunc<int(int)> t1 = [](int i){ tempGlobalState = !tempGlobalState; return tempGlobalVar + i + 1; };
            startBenchmark(); { for(int i = 0; i < repeats; ++i) t1(i); } lo << lt("trivial don_delegate") << endBenchmark() << endl;
        }
        {
            delegate<int(int)> t1 = [](int i){ tempGlobalState = !tempGlobalState; return tempGlobalVar + i + 1; };
            startBenchmark(); { for(int i = 0; i < repeats; ++i) t1(i); } lo << lt("trivial staticdelegate") << endBenchmark() << endl;
        } lo << endl;

        {
            function<int(int&, int)> t2 = rawFunc;
            function<void(int)> t1 = [&tempVar, &t2](int i){ tempVar = t2(tempVar, i); };
            startBenchmark(); { for(int i = 0; i < repeats; ++i) t1(i); } lo << lt("capture std::func") << endBenchmark() << endl;
        }
        {
            FastFunc<int(int&, int)> t2 = rawFunc;
            FastFunc<void(int)> t1 = [&tempVar, &t2](int i){ tempVar = t2(tempVar, i); };
            startBenchmark(); { for(int i = 0; i < repeats; ++i) t1(i); } lo << lt("capture don_delegate") << endBenchmark() << endl;
        }
        {
            delegate<int(int&, int)> t2 = rawFunc;
            delegate<void(int)> t1 = [&tempVar, &t2](int i){ tempVar = t2(tempVar, i); };
            startBenchmark(); { for(int i = 0; i < repeats; ++i) t1(i); } lo << lt("capture staticdelegate") << endBenchmark() << endl;
        } lo << endl;
    } lo << endl << tempVar << tempGlobalVar;

    return 0;
}
