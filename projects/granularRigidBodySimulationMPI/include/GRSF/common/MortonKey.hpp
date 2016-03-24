// ========================================================================================
//  GRSFramework 
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com> 
// 
//  This Source Code Form is subject to the terms of the GNU General Public License as 
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef GRSF_common_MortonKey_hpp
#define GRSF_common_MortonKey_hpp


#include <cstdint>


template<typename KeyType> class MortonKey;

template<>
class MortonKey<uint32_t> {

public:
    using uint32_t = Type;

    static Type encodeMorton2(Type x, Type y) {
        return (part1By1(y) << 1) + part1By1(x);
    }

    static Type encodeMorton3(Type x, Type y, Type z) {
        return (part1By2(z) << 2) + (part1By2(y) << 1) + part1By2(x);
    }

    static Type decodeMorton2X(Type code) {
        return compact1By1(code >> 0);
    }

    static Type decodeMorton2Y(Type code) {
        return compact1By1(code >> 1);
    }

    static Type decodeMorton3X(Type code) {
        return compact1By2(code >> 0);
    }

    static Type decodeMorton3Y(Type code) {
        return compact1By2(code >> 1);
    }

    static Type decodeMorton3Z(Type code) {
        return compact1By2(code >> 2);
    }


private:

    // "Insert" a 0 bit after each of the 16 low bits of x
    Type part1By1(Type x) {
        x &= 0x0000ffff;                  // x = ---- ---- ---- ---- fedc ba98 7654 3210
        x = (x ^ (x <<  8)) & 0x00ff00ff; // x = ---- ---- fedc ba98 ---- ---- 7654 3210
        x = (x ^ (x <<  4)) & 0x0f0f0f0f; // x = ---- fedc ---- ba98 ---- 7654 ---- 3210
        x = (x ^ (x <<  2)) & 0x33333333; // x = --fe --dc --ba --98 --76 --54 --32 --10
        x = (x ^ (x <<  1)) & 0x55555555; // x = -f-e -d-c -b-a -9-8 -7-6 -5-4 -3-2 -1-0
        return x;
    }

    // "Insert" two 0 bits after each of the 10 low bits of x
    Type part1By2(Type x) {

        x &= 0x000003ff;                  // x = ---- ---- ---- ---- ---- --98 7654 3210
        x = (x ^ (x << 16)) & 0xff0000ff; // x = ---- --98 ---- ---- ---- ---- 7654 3210
        x = (x ^ (x <<  8)) & 0x0300f00f; // x = ---- --98 ---- ---- 7654 ---- ---- 3210
        x = (x ^ (x <<  4)) & 0x030c30c3; // x = ---- --98 ---- 76-- --54 ---- 32-- --10
        x = (x ^ (x <<  2)) & 0x09249249; // x = ---- 9--8 --7- -6-- 5--4 --3- -2-- 1--0
        return x;
    }

    // Inverse of Part1By1 - "delete" all odd-indexed bits
    Type compact1By1(Type x) {
        x &= 0x55555555;                  // x = -f-e -d-c -b-a -9-8 -7-6 -5-4 -3-2 -1-0
        x = (x ^ (x >>  1)) & 0x33333333; // x = --fe --dc --ba --98 --76 --54 --32 --10
        x = (x ^ (x >>  2)) & 0x0f0f0f0f; // x = ---- fedc ---- ba98 ---- 7654 ---- 3210
        x = (x ^ (x >>  4)) & 0x00ff00ff; // x = ---- ---- fedc ba98 ---- ---- 7654 3210
        x = (x ^ (x >>  8)) & 0x0000ffff; // x = ---- ---- ---- ---- fedc ba98 7654 3210
        return x;
    }

    // Inverse of Part1By2 - "delete" all bits not at positions divisible by 3
    Type compact1By2(Type x) {
        x &= 0x09249249;                  // x = ---- 9--8 --7- -6-- 5--4 --3- -2-- 1--0
        x = (x ^ (x >>  2)) & 0x030c30c3; // x = ---- --98 ---- 76-- --54 ---- 32-- --10
        x = (x ^ (x >>  4)) & 0x0300f00f; // x = ---- --98 ---- ---- 7654 ---- ---- 3210
        x = (x ^ (x >>  8)) & 0xff0000ff; // x = ---- --98 ---- ---- ---- ---- 7654 3210
        x = (x ^ (x >> 16)) & 0x000003ff; // x = ---- ---- ---- ---- ---- --98 7654 3210
        return x;
    }

};


template<>
class MortonKey<uint64_t> {

public:
    using uint64_t = Type;

    static Type encodeMorton2(Type x, Type y) {
        return (part1By1(y) << 1) + part1By1(x);
    }

    static Type encodeMorton3(Type x, Type y, Type z) {
        return (part1By2(z) << 2) + (part1By2(y) << 1) + part1By2(x);
    }

    static Type decodeMorton2X(Type code) {
        return compact1By1(code >> 0);
    }

    static Type decodeMorton2Y(Type code) {
        return compact1By1(code >> 1);
    }

    static Type decodeMorton3X(Type code) {
        return compact1By2(code >> 0);
    }

    static Type decodeMorton3Y(Type code) {
        return compact1By2(code >> 1);
    }

    static Type decodeMorton3Z(Type code) {
        return compact1By2(code >> 2);
    }


private:

    // "Insert" a 0 bit after each of the 32 low bits of x
    Type part1By1(Type x) {

        return x;
    }

    // "Insert" two 0 bits after each of the 21 low bits of x
    Type part1By2(Type x) {
        x&=0x1FFFFF
        x=(x|(x<<20))&0x000001FFC00003FF;

            x = ---- ---- ---- ---- ---- ---- ---- ----#---- ---- ---k jihg fedc ba98 7654 3210
            x = ---- ---- ---- ---- ---- ---- ---k jihg#fe-- ---- ---- ---- ---- --98 7654 3210
            x = ---k --j- -i-- h--g --f- -e-- d--c --b-#-a-- 9--8 --7- -6-- 5--4 --3- -2-- 1--0

        x=(x|(x<<10))&0x0007E007C00F801F;


        x=(x|(x<<4))&0x00786070C0E181C3;
        x=(x|(x<<2))&0x0199219243248649;
        x=(x|(x<<2))&0x0649249249249249;
        x=(x|(x<<2))&0x1249249249249249;






            return x;
    }

    // Inverse of Part1By1 - "delete" all odd-indexed bits
    Type compact1By1(Type x) {

    }

    // Inverse of Part1By2 - "delete" all bits not at positions divisible by 3
    Type compact1By2(Type x) {

    }

};
#endif
