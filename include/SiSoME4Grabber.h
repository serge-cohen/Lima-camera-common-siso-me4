#ifndef SISOME4GRABBER_H
#define SISOME4GRABBER_H

/* siso-me4 plugin frame-grabber class
 * Copyright (C) 2013 IPANEMA USR3461, CNRS/MCC.
 * Written by Serge Cohen <serge.cohen@synchrotron-soleil.fr>
 *
 * This file is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 3 of
 * the License, or (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this file. If not, see <http://www.gnu.org/licenses/>.
 */

#if defined (__GNUC__) && (__GNUC__ == 3) && defined (__ELF__)
#   define GENAPI_DECL __attribute__((visibility("default")))
#   define GENAPI_DECL_ABSTRACT __attribute__((visibility("default")))
#endif

// System headers :
#include <stdlib.h>
#include <limits>
#include <ostream>

// Framegrabber SDK headers :
#include <fgrab_struct.h>
#include <fgrab_prototyp.h>
#include <fgrab_define.h>


// LImA headers :
#include "Debug.h"
//#include "HwMaxImageSizeCallback.h"
//#include "HwBufferMgr.h"


// VieworksVP plugin headers :

namespace lima
{
  namespace siso_me4
  {

    /*******************************************************************
     * \class Grabber
     * \brief object controlling the Silicon Software MicroEnable IV frame grabber
     *
     * This class is responsible of handling the frame grabber specific part of the
     * acquisition system : frame perparation and transfer, serial line communication
     * to/from the camera, setting of the frame-grabber and potentially programming
     * the frame-grabber
     *******************************************************************/
    class Grabber {
      DEB_CLASS_NAMESPC(DebModCamera, "Grabber", "siso-me4");

      //! The status of the grabber
      enum Status { Ready, Exposure, Readout, Latency, Fault };

      Grabber(int board_index, int cam_port, const std::string& applet_name);
      virtual ~Grabber();
      
      // Preparing the camera's SDK to acquire frames
      void prepareAcq();
      // Launches the SDK's acquisition and the m_acq_thread to retrieve frame buffers as they are ready
      void startAcq();
      // Stops the acquisition, as soon as the m_acq_thread is retrieving frame buffers.
      void stopAcq();
      

      
      
    };
    
  } // namespace siso_me4
} // namespace lima

#endif SISOME4GRABBER_H