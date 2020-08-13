/*
 * File name: canvasview_mkgreed.h
 * Date:      2016/12/09 08:01
 * Author:    Jan Faigl
 */

#ifndef __CANVASVIEW_GSOA_H__
#define __CANVASVIEW_GSOA_H__

#include <vector>

#include <boost/foreach.hpp>

#include <crl/gui/gui.h>
#include <crl/gui/colors.h>
#include <crl/gui/renderer.h>
#include <crl/gui/canvas.h>
#include <memory>
#include "target.h"

namespace graspc {

} // end namespace graspc

/// ----------------------------------------------------------------------------
inline crl::gui::CCanvasBase& operator<<(crl::gui::CCanvasBase &canvas, const TargetSet &target)
{
   canvas << target.coords[0].x << target.coords[0].y;
   canvas << target.coords[1].x << target.coords[1].y;
   return canvas;
}

/// ----------------------------------------------------------------------------
inline crl::gui::CCanvasBase& operator<<(crl::gui::CCanvasBase &canvas, const TargetSetPtrVector &targets)
{
   BOOST_FOREACH(std::shared_ptr<TargetSet> target, targets) {
        canvas << target->coords[0].x << target->coords[0].y;
        canvas << target->coords[1].x << target->coords[1].y;
   }
   return canvas;
}


#endif

/* end of canvasview_mkgreed.h */
