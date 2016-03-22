#ifndef ExpectedMass_hpp
#define ExpectedMass_hpp
#include <iomanip>
#include <cmath>
#include <array>
#include <random>
#include <fstream>

    unsigned int n = 1000000;
    using PREC = double;
    PREC density = 2400;

void calcualteExpectedMassBin(){

    std::ifstream f("numbers.bin", std::ios::binary);
    if(!f.good()){
        std::cerr << "Could not open file!" << std::endl;
    }
    std::vector<PREC> masses;
    PREC d;
    std::cout << sizeof(PREC) << std::endl;
    while(1){
        f.read((char *)&d,sizeof(d));
        if(!f.eof()){
            d*=2*1e-3;
            PREC mass = d*d*d/3.0*M_PI_2*density;
            masses.push_back(mass);
        }else{
            break;
        }
    }
    f.close();
    std::cout << "Read " << masses.size() << " values " << std::endl;


     // Do reduction (to sum up more accurately)
    unsigned int stride = 1;
    auto s = masses.size();
    while(stride < s){
           for(unsigned int i = 0; i<n ; i+=2*stride){
                if(i+stride < s){
                    masses[i] += masses[i+stride];
                }
           }
           stride*=2;
    }
    // result is in masses[0];
    std::cout <<std::setprecision(30) << "Mass computed by reduction: \t\t\t\t" << masses[0] << std::endl;
}

void calculateExpectedMass(){

    using RandomGenType = std::mt19937_64;
    RandomGenType gen(51651651651);
    std::array<PREC,5> intervals {0.59, 0.7, 0.85, 1, 1.18};
    std::array<PREC,4> weights {1.36814, 1.99139, 0.29116, 0.039562};


     // integral over the pdf to normalize:
    PREC normalization =0;
    for(unsigned int i=0;i<4;i++){
        normalization += weights[i]*(intervals[i+1]-intervals[i]);
    }
    std::cout << std::setprecision(30) << "Normalization: " << normalization << std::endl;
    // normalize all weights (such that the integral gives 1)!
    for(auto & w : weights){
        w /= normalization;
    }

    std::piecewise_constant_distribution<PREC>
    distribution (intervals.begin(),intervals.end(),weights.begin());

    PREC mass = 0;
    std::vector<PREC> masses(n);
     std::ofstream f("numbersCpp.bin", std::ios::binary);
    for(unsigned int i=0;i<n;i++){
        auto r = distribution(gen);
        f.write((char*)&r,sizeof(r));
        auto d = 2*r*1e-3;
        masses[i] = d*d*d/3.0*M_PI_2*density;
        mass += masses[i];
    }
    f.close();

    // Do reduction (to sum up more accurately)
    unsigned int stride = 1;
    while(stride < n){
           for(unsigned int i = 0; i<n ; i+=2*stride){
                if(i+stride < n){
                    masses[i] += masses[i+stride];
                }
           }
           stride*=2;
    }
    // result is in masses[0];


    std::cout <<std::setprecision(30) << "Mass for n: "<< n << " spheres  with density: " << density << " = \t" << mass << " kg"<< std::endl;
    std::cout                         << "Mass computed by reduction: \t\t\t\t" << masses[0] << std::endl;
}


void calculateExpectedMassSimple(){

    using RandomGenType = std::mt19937_64;
    RandomGenType gen(51651651651);

    using PREC = double;
    std::array<PREC,5> intervals {0.5, 0.7, 0.8, 1, 1.2};
    std::array<PREC,4> weights   {1.5, 3, 0.5, 0.1};

     // integral over the pdf to normalize:
    PREC normalization =0;
    for(unsigned int i=0;i<4;i++){
        normalization += weights[i]*(intervals[i+1]-intervals[i]);
    }
    std::cout << std::setprecision(30) << "Normalization: " << normalization << std::endl;
    // normalize all weights (such that the integral gives 1)!
    for(auto & w : weights){
        w /= normalization;
        std::cout << "Probability : " << w <<std::endl;
    }

    std::piecewise_constant_distribution<PREC>
    distribution (intervals.begin(),intervals.end(),weights.begin());

    std::cout << "Densities: ";
    auto d = distribution.densities();
    std::copy(d.begin(), d.end(), std::ostream_iterator<PREC>(std::cout, " "));
    std::cout << std::endl;

    std::ofstream f("numbersCppSimple.bin", std::ios::binary);
    for(unsigned int i=0;i<n;i++){
        auto r = distribution(gen);
        f.write((char*)&r,sizeof(r));
    }
    f.close();
}




#endif // ExpectedMass_hpp


