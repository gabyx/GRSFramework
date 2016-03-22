#ifndef KNearstNeighbours_hpp
#define KNearstNeighbours_hpp

#include <list>
#include <queue>
#include <iterator>
#include <algorithm>
#include "GRSF/Common/CPUTimer.hpp"
#include <random>

void kNearstNeighboursTest(){

    double lower_bound = 0;
    double upper_bound = 10000;
    std::uniform_real_distribution<double> unif(lower_bound,upper_bound);
    std::default_random_engine re;

    int maxI = 10000;
    int k = 30; // k-smallest
    int n = 1000; // add to prio
    int loops = 1000;
    {
        std::list<double> prio;
        for(int i=0; i<k; i++){
            prio.push_back(unif(re));
        }
        //std::sort(prio.begin(),prio.end());
        prio.sort();

        double totalT = 0;
        for(int l=0; l<loops;l++){

            std::vector<double> others;
            for(int i =0;i<n;++i){
                others.push_back(unif(re));
            }
            START_TIMER(start);
            // sort others (all)
            std::partial_sort(others.begin(),(k>=others.size())? others.end() : std::next(others.begin(),k),others.end());
            // merge (inplace, first insert at end, then merge inplace)
            auto s = prio.size();

            prio.insert( std::next(prio.begin(),s),others.begin(), (k>=others.size())? others.end() : std::next(others.begin(),k ) );
            std::inplace_merge(prio.begin(),std::next(prio.begin(),s),prio.end());
            prio.erase(std::next(prio.begin(),k),prio.end()); // keeping size
            STOP_TIMER_SEC(count,start);
            totalT += count;
        }
        std::cout << "Merge " << k << " smallest of "
                  << n << " elements  into list: " << totalT/loops << "sec." << std::endl;
        std::cout << "result: " << std::endl;
        for(auto i : prio){
                std::cout << i <<", ";
        }
        std::cout << std::endl;
    }
     {
        std::vector<double> prio;
        for(int i=0; i<k; i++){
            prio.push_back(unif(re));
        }
        std::sort(prio.begin(),prio.end());
        //prio.sort();

        double totalT = 0;
        for(int l=0; l<loops;l++){

            std::vector<double> others;
            for(int i =0;i<n;++i){
                others.push_back(unif(re));
            }
            START_TIMER(start);
            // sort others
            std::partial_sort(others.begin(),(k>=others.size())? others.end() : others.begin()+k,others.end());
            // merge (inplace, first insert at end, then merge inplace)
            auto s = prio.size();
            prio.insert( prio.begin()+s,others.begin(), (k>=others.size())? others.end() : others.begin()+k  );
            std::inplace_merge(prio.begin(),prio.begin()+s,prio.end());
            prio.resize(k); // keeping size
            STOP_TIMER_SEC(count,start);
            totalT += count;
        }
        std::cout << "Merge " << k << " smallest of "
                  << n << " elements  into vector: " << totalT/loops << "sec." << std::endl;
        std::cout << "result: " << std::endl;
        for(auto i : prio){
                std::cout << i <<", ";
        }
        std::cout << std::endl;
    }
    {
        std::priority_queue<double> prio;
        for(int i=0; i<k; i++){
            prio.push(unif(re));
        }

        double totalT = 0;
        for(int l=0; l<loops;l++){

            std::vector<double> others;
            for(int i =0;i<n;++i){
                others.push_back(unif(re));
            }
            START_TIMER(start);
            // sort others
            auto s = (k>=others.size())? others.size() : k;
            std::nth_element(others.begin(),others.begin()+k,others.end());

            for(int i =0;i<s;++i){
                prio.push(others[i]);
                prio.pop(); // keeping the size
            }


            STOP_TIMER_SEC(count,start);
            totalT += count;
        }
        std::cout << "Merge " << k << " smallest of "
                  << n << " elements  into priority_queue: " << totalT/loops << "sec." << std::endl;
         std::cout << "result: " << std::endl;

        std::vector<double> l(prio.size());
        std::copy(&prio.top(), &prio.top() + prio.size(),  &l[0]);
        for(auto i : l){
                std::cout << i <<", ";
        }
        std::cout << std::endl;
    }
}
#endif // KNearstNeighbours_hpp
