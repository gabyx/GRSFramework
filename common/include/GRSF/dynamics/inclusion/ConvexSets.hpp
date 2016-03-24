// ========================================================================================
//  GRSFramework 
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com> 
// 
//  This Source Code Form is subject to the terms of the GNU General Public License as 
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef GRSF_dynamics_inclusion_ConvexSets_hpp
#define GRSF_dynamics_inclusion_ConvexSets_hpp


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
