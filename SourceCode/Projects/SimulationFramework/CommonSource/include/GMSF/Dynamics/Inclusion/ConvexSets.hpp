/*
*  GMSF/Dynamics/Inclusion/ConvexSets.hpp
*
*  Created by Gabriel Nützi on 21.03.10.
*  Copyright 2010 ETH. All rights reserved.
*
*/

#ifndef GMSF_Dynamics_Inclusion_ConvexSets_hpp
#define GMSF_Dynamics_Inclusion_ConvexSets_hpp


// CONVEX SETS

   /** @addtogroup ProxFunctions
   * @{
   */

   /**
   * @brief Definitions of all convex sets to use with a prox function.
   */
   namespace ConvexSets{
      /**
      * @brief Convex set for \f$ C = \mathcal{R}_{+} \f$ .
      */
      struct RPlus{
         static const int Dimension=1;
      };

      /**
      * @brief Convex set for a unit disk \f$ C = \{ x | |x| < 1 \} \f$ .
      */
      struct Disk{
         static const int Dimension=2;
      };

      /**
      * @brief Convex set for a Contensou ellipsoid.
      */
      struct ContensouEllipsoid{
         static const int Dimension=3;
      };

      /**
      * @brief Convex set for  \f$ C_1 = \mathcal{R}_{+} \f$  and a unit disk \f$ C_2 = \{ x | |x| < 1 \} \f$ .
      * This function applies for triplets, the first value is proxed onto \f$ C_1 \f$
      * and the second to values in sequence are proxed on to \f$ C_2 \f$ (scaled with an aribtary radius) .
      */
      struct RPlusAndDisk{
         static const int Dimension=3;
      };

      /**
      * @brief Convex set for a Cone in \f$ \mathcal{R}^3 \f$ with center axis as the x-axis given as
      * \f$ K = {(x_1,x_2,x_3) \in \mathcal{R}^3 \ | \ \sqrt{(x_2^2 + x_3^2)} \leq \mu x_1 \} \f$ .
      */
      struct Cone3D{
         static const int Dimension=3;
      };

      /**
      * @brief Convex set for a AABB in \f$ \mathcal{R}^3 \f$ with min and max point
      */
      struct AABB{
         static const int Dimension=3;
      };


      /**
      * @brief Convex set for  \f$ C_1 = \mathcal{R}_{+} \f$  and a Contensou ellipsoid.
      * This function applies for triplets, the first value is proxed onto \f$ C_1 \f$  and the second to values in sequence are proxed on to the Contensou ellipsoid.
      */
      struct RPlusAndContensouEllipsoid{
         static const int Dimension=4;
      };

   };




   /** @}*/

#endif
