// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef GRSF_common_ColorGradient_hpp
#define GRSF_common_ColorGradient_hpp

#include <vector>
#include "GRSF/common/TypeDefs.hpp"

class ColorGradient
{
private:
    DEFINE_LAYOUT_CONFIG_TYPES

    struct ColorPoint
    {                   // Internal class used to store m_colors at different points in the gradient.
        Vector3 m_rgb;  // Red, green and blue values of our color.
        PREC m_val;     // Position of our m_color along the gradient (between 0 and 1).
        ColorPoint(Vector3 rgb, PREC value) : m_rgb(rgb), m_val(value)
        {
        }
    };
    std::vector<ColorPoint> m_color;  // An array of m_color points in ascending value.

public:
    //-- Default constructor:
    ColorGradient()
    {
    }

    void clearColors()
    {
        m_color.clear();
    }

    //-- Inserts a new color point into its correct position:
    void addColorPoint(Vector3 rgb, PREC value)
    {
        for (int i = 0; i < m_color.size(); i++)
        {
            if (value < m_color[i].m_val)
            {
                m_color.insert(m_color.begin() + i, ColorPoint(rgb, value));
                return;
            }
        }
        m_color.push_back(ColorPoint(rgb, value));
    }

    //-- Inserts a new color point into its correct position:
    void clearGradient()
    {
        m_color.clear();
    }

    //-- Places a 5 color heapmap gradient into the "m_color" vector:
    void createDefaultHeatMapGradient()
    {
        m_color.clear();
        m_color.push_back(ColorPoint(Vector3(0, 0, 1), 0.0f));   // Blue.
        m_color.push_back(ColorPoint(Vector3(0, 1, 1), 0.25f));  // Cyan.
        m_color.push_back(ColorPoint(Vector3(0, 1, 0), 0.5f));   // Green.
        m_color.push_back(ColorPoint(Vector3(1, 1, 0), 0.75f));  // Yellow.
        m_color.push_back(ColorPoint(Vector3(1, 0, 0), 1.0f));   // Red.
    }

    //-- Inputs a (value) between 0 and 1 and outputs the (red), (green) and (blue)
    //-- values representing that position in the gradient.
    void getColorAtValue(const PREC value, Vector3& rgb)
    {
        // std::cout << " value " << value << std::endl;
        if (m_color.size() == 0)
            return;

        for (int i = 0; i < m_color.size(); i++)
        {
            ColorPoint& currC = m_color[i];
            if (value < currC.m_val)
            {
                ColorPoint& prevC = m_color[std::max(0, i - 1)];
                PREC valueDiff    = (currC.m_val - prevC.m_val);
                PREC fractBetween = (valueDiff == 0.0) ? 0.0 : (value - prevC.m_val) / valueDiff;
                rgb               = (currC.m_rgb - prevC.m_rgb) * fractBetween + prevC.m_rgb;
                // std::cout << rgb.transpose() << std::endl;
                return;
            }
        }
        rgb = m_color.back().m_rgb;
        return;
    }
};

#endif  // ColorGradient_hpp
