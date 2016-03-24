#ifndef GRSF_common_UnorderedContainerHelpers_hpp
#define GRSF_common_UnorderedContainerHelpers_hpp

/** Intersection and union function for unordered containers which support a fast lookup function find()
 *  Return values are moved by move-semantics, for c++11/c++14 this is efficient, otherwise it results in a copy
 */

namespace unorderedHelpers {

    template<typename UnorderedIn1, typename UnorderedIn2,
             typename UnorderedOut = UnorderedIn1>
    UnorderedOut makeIntersection(const  UnorderedIn1 &in1, const  UnorderedIn2 &in2)
    {
        if (in2.size() < in1.size()) {
        	return makeIntersection<UnorderedIn2,UnorderedIn1,UnorderedOut>(in2, in1);
        }

        UnorderedOut out;
        auto e = in2.end();
        for(auto & v : in1)
        {
        	if (in2.find(v) != e){
        		out.insert(v);
        	}
        }
        return out;
    }

    template<typename UnorderedIn1, typename UnorderedIn2,
             typename UnorderedOut = UnorderedIn1>
    UnorderedOut makeUnion(const UnorderedIn1 &in1, const UnorderedIn1 &in2)
    {
        UnorderedOut out;
        out.insert(in1.begin(), in1.end());
        out.insert(in2.begin(), in2.end());
        return out;
    }
}

#endif
