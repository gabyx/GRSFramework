// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef GRSF_dynamics_general_RigidBodySolverData_hpp
#define GRSF_dynamics_general_RigidBodySolverData_hpp

#include "GRSF/common/TypeDefs.hpp"

class RigidBodySolverData
{
public:
    DEFINE_LAYOUT_CONFIG_TYPES

    RigidBodySolverData() : m_t(0), m_overlapTotal(0){};

    /** The actual time, which belongs to FrontBuffer and m_r_S and I_q_IK */
    PREC m_t;

    /** Total amount of overlap  ( sum( g_i ) , g_i : gap functions)
        * See ContactGraphVisitors.hpp for choosing another computation ( max(g_i) for example)
        */
    PREC m_overlapTotal;
};

/** Class with  Data Structure for the Solver! */
class RigidBodySolverDataCONoG : public RigidBodySolverData
{
public:
    DEFINE_LAYOUT_CONFIG_TYPES
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    RigidBodySolverDataCONoG() : m_bInContactGraph(false)
    {
        m_K_omega_IK_beg.setZero();
        m_q_KI_beg.setIdentity();
        m_uBuffer.m_front.setZero();
        m_uBuffer.m_back.setZero();
    };

    /** no communication, is determined after body communcation */
    bool m_bInContactGraph;  ///< Flag which determines if this body is in the contact graph!

    /** Timestepper additional temporaries ======================
    * These values are valid in-between a time-step deltaT and are needed by the process which timesteps this body ->
    * owner
    * communication needed: process change during body comm (notify, update)
    */
    Vector3 m_K_omega_IK_beg;  ///< Angular velocity (omega) at beginning of timestep, important to save this for
                               /// timestep update in timestepper!
    Quaternion m_q_KI_beg;     ///< Quaternion at beginning of timestep, important to save this for timestep update in
                               /// timestepper!
    /** =========================================================*/

    /** Velocity buffer which is iterateted in the InclusionSolverCONoG
    * The back buffer is the velocity before the prox iteration (over all nodes)
    * The front buffer is the velocity which is used to during ONE prox iteration
    */
    class VelocityBuffer
    {
    public:
        VelocityBuffer() /*: m_front(m_front_internal.data()), m_back(m_back_internal.data()) */
        {
        }
        //        MatrixMap<VectorUBody> m_front;
        //        MatrixMap<VectorUBody> m_back;
        VectorUBody m_front;
        VectorUBody m_back;

    private:
        // Uncomment for faster switch!
        //        VectorUBody m_front_internal;
        //        VectorUBody m_back_internal;
    };

    using VelocityBufferType = VelocityBuffer;
    VelocityBufferType m_uBuffer;

    void swapBuffer()
    {
        m_uBuffer.m_front.swap(m_uBuffer.m_back);  // swap buffer
    }

    void reset()
    {
        m_uBuffer.m_front.setZero();
        m_bInContactGraph = false;
    };
};
#endif
