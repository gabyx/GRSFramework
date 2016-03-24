#ifndef GRSF_dynamics_general_ParserFunctions_hpp
#define GRSF_dynamics_general_ParserFunctions_hpp

#include "GRSF/common/TypeDefs.hpp"
#include "GRSF/common/LogDefines.hpp"

#include "GRSF/common/AssertionDebug.hpp"

#include "GRSF/common/CommonFunctions.hpp"

namespace ParserFunctions{

    DEFINE_LAYOUT_CONFIG_TYPES

    template<typename XMLNodeType>
    void parseTransformSequence(XMLNodeType & node,
                                Quaternion & q_KI,
                                Vector3 & I_r_IK ){

            q_KI.setIdentity();//QuaternionHelpers::setQuaternionZero(q_KI);
            I_r_IK.setZero();

            // Iterate over all transforms an successfully applying the total trasnformation!
            Vector3 trans;
            Vector3 axis;
            PREC angle;
            for ( XMLNodeType & transf : node.children("Trafo")) {


                if(!Utilities::stringToType(trans, transf.attribute("trans").value())) {
                    ERRORMSG("---> String conversion in InitialPositionTransforms: translation failed");
                }

                if(!Utilities::stringToType(axis, transf.attribute("axis").value())) {
                    ERRORMSG("---> String conversion in InitialPositionTransforms: rotationAxis failed");
                }

                if( axis.norm() == 0) {
                    ERRORMSG("---> Specified wrong axis in InitialPositionTransforms");
                }

                auto att = transf.attribute("deg");
                if(att) {
                    if(!Utilities::stringToType(angle, att.value())) {
                        ERRORMSG("---> String conversion in InitialPositionPosAxisAngle: rad failed");
                    }
                    angle = angle / 180.0 * M_PI;
                } else {
                    att = transf.attribute("rad");
                    if(att) {
                        if(!Utilities::stringToType(angle, att.value())) {
                            ERRORMSG("---> String conversion in InitialPositionPosAxisAngle: deg failed");
                        }
                    } else {
                        ERRORMSG("---> No angle found in InitialPositionPosAxisAngle");
                    }
                }
                axis.normalize();
                Quaternion q_BK(AngleAxis(angle,axis));
                trans=q_KI*trans;//QuaternionHelpers::rotateVector(q_KI, trans ); //K_r_KB = trans;
                I_r_IK +=  trans;  // + Rot_KI * K_r_KB; // Transforms like A_IK * K_r_KB;

                q_KI = q_KI*q_BK;//QuaternionHelpers::quatMult(q_KI,q_BK);
                // Sequential (aktiv) rotation ( A_AB * B_R_2 * A_BA * A_R_1 ) *A_x
                // is the same like: A_R_1 * B_R_2 (see documentation page)

            }


    }
};

#endif
