/*
 * File name: canvasview_coords.h
 * Date:      2013/10/13 09:26
 * Author:    Jan Faigl
 */

#ifndef __CANVASVIEW_COORDS_H__
#define __CANVASVIEW_COORDS_H__

#include <boost/foreach.hpp>

#include <crl/gui/gui.h>
#include <crl/gui/colors.h>
#include <crl/gui/renderer.h>
#include <crl/gui/canvas.h>

#include "coords.h"

/// ----------------------------------------------------------------------------
inline crl::gui::CCanvasBase& operator<<(crl::gui::CCanvasBase &canvas, const Coords &coords) 
{
   canvas << coords.x << coords.y;
   return canvas;
}

/// ----------------------------------------------------------------------------
inline crl::gui::CCanvasBase& operator<<(crl::gui::CCanvasBase &canvas, const std::vector<Coords> &points) 
{
   BOOST_FOREACH(const Coords &pt, points) {
      canvas << pt;
   }
   return canvas;
}

#endif

/* end of canvasview_coords.h */
