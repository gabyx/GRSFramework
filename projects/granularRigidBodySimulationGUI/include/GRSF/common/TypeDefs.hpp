// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef GRSF_common_TypeDefs_hpp
#define GRSF_common_TypeDefs_hpp

#include <random>

#include "GRSF/configFiles/ConfigureFile.hpp"
#include "GRSF/dynamics/general/LayoutConfigTypeDefs.hpp"

/// Forward declarations , includes and the GlobalConfig typed need to match!

#define StatePool_INCLUDE_FILE "GRSF/dynamics/buffers/StatePoolVisBackFront.hpp"
class StatePoolVisBackFront;

#define CollisionSolver_INCLUDE_FILE "GRSF/dynamics/collision/CollisionSolver.hpp"
class CollisionSolver;

#define DynamicsSystem_INCLUDE_FILE "GRSF/dynamics/general/DynamicsSystemGUI.hpp"
class DynamicsSystemGUI;
#define InclusionSolverSettings_INCLUDE_FILE "GRSF/dynamics/inclusion/InclusionSolverSettings.hpp"
class InclusionSolverSettings;

//#define InclusionSolver_INCLUDE_FILE "GRSF/dynamics/Inclusion/InclusionSolverCO.hpp"
// class InclusionSolverCO;
#define InclusionSolver_INCLUDE_FILE "GRSF/dynamics/inclusion/InclusionSolverCONoG.hpp"
class InclusionSolverCONoG;

#define RigidBody_INCLUDE_FILE "GRSF/dynamics/general/RigidBody.hpp"
class RigidBodyBase;

#define RigidBodySolverData_INCLUDE_FILE "GRSF/dynamics/general/RigidBodySolverData.hpp"
class RigidBodySolverDataCONoG;

#define TimeStepper_INCLUDE_FILE "GRSF/dynamics/general/MoreauTimeStepper.hpp"
class MoreauTimeStepper;

// Try to make framework settings simpler:
struct GlobalConfigs
{
    struct MyConfigs
    {
        using RigidBodyType = RigidBodyBase;

        using DynamicsSystemType = DynamicsSystemGUI;

        using TimeStepperType = MoreauTimeStepper;

        using CollisionSolverType = CollisionSolver;

        using InclusionSolverType         = InclusionSolverCONoG;
        using InclusionSolverSettingsType = InclusionSolverSettings;
    };

    struct GeneralConfigs
    {
        using RandomGeneratorType = std::mt19937;
    };

    struct SolverConfigs
    {
        using TimeStepperType = MyConfigs::TimeStepperType;
    };

    struct TimeStepperConfigs
    {
        using DynamicsSystemType  = typename MyConfigs::DynamicsSystemType;
        using CollisionSolverType = typename MyConfigs::CollisionSolverType;
        using InclusionSolverType = typename MyConfigs::InclusionSolverType;
        using StatePoolType       = StatePoolVisBackFront;
    };

    struct DynamicSystemConfigs
    {
        using RigidBodyType               = typename MyConfigs::RigidBodyType;
        using InclusionSolverSettingsType = typename MyConfigs::InclusionSolverSettingsType;
    };

    struct RigidBodyConfigs
    {
        using LayoutConfigType   = LayoutConfig<double, GeneralLayout<7, 6>>;
        using BodySolverDataType = RigidBodySolverDataCONoG;
    };

    struct InclusionSolverConfigs
    {
        using DynamicsSystemType          = typename MyConfigs::DynamicsSystemType;
        using InclusionSolverSettingsType = typename MyConfigs::InclusionSolverSettingsType;
    };
    struct CollisionSolverConfigs
    {
        using DynamicsSystemType = typename MyConfigs::DynamicsSystemType;
    };

    struct MPIInformationConfigs
    {
        using RankIdType = std::size_t;
    };
};

#define DEFINE_CONFIG_TYPES DEFINE_SOLVER_CONFIG_TYPES

#define DEFINE_SOLVER_CONFIG_TYPES DEFINE_TIMESTEPPER_CONFIG_TYPES

#define DEFINE_TIMESTEPPER_CONFIG_TYPES                                                          \
    using TimeStepperType     = typename GlobalConfigs::SolverConfigs::TimeStepperType;          \
    using StatePoolType       = typename GlobalConfigs::TimeStepperConfigs::StatePoolType;       \
    using InclusionSolverType = typename GlobalConfigs::TimeStepperConfigs::InclusionSolverType; \
    using CollisionSolverType = typename GlobalConfigs::TimeStepperConfigs::CollisionSolverType; \
    DEFINE_DYNAMICSSYTEM_CONFIG_TYPES

#define DEFINE_INCLUSIONS_SOLVER_CONFIG_TYPES                                                    \
    using CollisionSolverType = typename GlobalConfigs::TimeStepperConfigs::CollisionSolverType; \
    DEFINE_DYNAMICSSYTEM_CONFIG_TYPES

#define DEFINE_COLLISION_SOLVER_CONFIG_TYPES DEFINE_DYNAMICSSYTEM_CONFIG_TYPES

#define DEFINE_DYNAMICSSYTEM_CONFIG_TYPES                                                                          \
    using InclusionSolverSettingsType = typename GlobalConfigs::DynamicSystemConfigs::InclusionSolverSettingsType; \
    using DynamicsSystemType          = typename GlobalConfigs::TimeStepperConfigs::DynamicsSystemType;            \
    DEFINE_RIGIDBODY_CONFIG_TYPES

#define DEFINE_RIGIDBODY_CONFIG_TYPES                                                        \
    using RigidBodyType      = typename GlobalConfigs::DynamicSystemConfigs::RigidBodyType;  \
    using BodySolverDataType = typename GlobalConfigs::RigidBodyConfigs::BodySolverDataType; \
    DEFINE_LAYOUT_CONFIG_TYPES                                                               \
    DEFINE_GENERAL_CONFIG_TYPES

#define DEFINE_LAYOUT_CONFIG_TYPES                                                       \
    using LayoutConfigType = typename GlobalConfigs::RigidBodyConfigs::LayoutConfigType; \
    DEFINE_LAYOUT_CONFIG_TYPES_OF(GlobalConfigs::RigidBodyConfigs::LayoutConfigType)

#define DEFINE_MATRIX_TYPES                                                        \
    using PREC = typename GlobalConfigs::RigidBodyConfigs::LayoutConfigType::PREC; \
    DEFINE_MATRIX_TYPES_OF(GlobalConfigs::RigidBodyConfigs::LayoutConfigType::PREC)

#define DEFINE_GENERAL_CONFIG_TYPES using RandomGenType = typename GlobalConfigs::GeneralConfigs::RandomGeneratorType;

#define DEFINE_MPI_INFORMATION_CONFIG_TYPES using RankIdType = GlobalConfigs::MPIInformationConfigs::RankIdType;

#endif
