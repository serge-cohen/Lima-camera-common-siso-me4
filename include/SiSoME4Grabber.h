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
#include <string>
#include <vector>

// Framegrabber SDK headers :
#include <fgrab_struct.h>
#include <fgrab_prototyp.h>
#include <fgrab_define.h>


// LImA headers :
#include "Debug.h"
//#include "HwMaxImageSizeCallback.h"
#include "HwBufferMgr.h"


// siso_me4 plugin headers :
#include "SiSoME4SerialLine.h"

/* To get extra information about parameters,
 the best would be to getParameterWithType wiath a type of 
 FG_PARAM_TYPE_STRUCT_FIELDPARAMACCESS
 and then go exploring the FieldParameterAccess which contains pretty much
 all useful information !!!
 In particular the native type in FgParamTypes.
 */

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
    public:
      //! The status of the grabber
      enum Status { Ready, Running, Fault };
      // Using directly the FgParamTypes instead !
//      enum siso_type {
//        siso_type_invalid = FG_PARAM_TYPE_INVALID,
//        siso_type_int32 = FG_PARAM_TYPE_INT32_T,
//        siso_type_uint32 = FG_PARAM_TYPE_UINT32_T,
//        siso_type_int64 = FG_PARAM_TYPE_INT64_T,
//        siso_type_uint64 = FG_PARAM_TYPE_UINT64_T,
//        siso_type_double = FG_PARAM_TYPE_DOUBLE,
//        siso_type_cstr = FG_PARAM_TYPE_CHAR_PTR,
//        siso_type_sizet = FG_PARAM_TYPE_SIZE_T,
//        siso_type_param_access = FG_PARAM_TYPE_STRUCT_FIELDPARAMACCESS,
//        siso_type_param_int = FG_PARAM_TYPE_STRUCT_FIELDPARAMINT,
//        siso_type_param_double = FG_PARAM_TYPE_STRUCT_FIELDPARAMDOUBLE
//      };
      enum siso_px_format {
        siso_px_8b = FG_GRAY,
        siso_px_16b = FG_GRAY16,
        siso_px_rgb_24b = FG_COL24,
        siso_px_rgb_32b = FG_COL32
      };
      enum siso_trig_mode {
        siso_trig_free = FREE_RUN,
        siso_trig_grabber = GRABBER_CONTROLLED,
        siso_trig_grabber_sync = GRABBER_CONTROLLED_SYNCHRON,
        siso_trig_async_soft = ASYNC_SOFTWARE_TRIGGER,
        siso_trig_async = ASYNC_TRIGGER
      };
      enum siso_access_mode {
        siso_access_auto = 0,
        siso_access_read = 1,
        siso_access_read_write = 3,
        siso_access_read_write_change = 7
      };

      Grabber(const std::string& i_siso_dir_5, int board_index, int cam_port, const std::string& applet_name, unsigned int dma_index);
      virtual ~Grabber();
      virtual int sisoError(int code) const;

    public:
      // Handling the number of frames to collect:
      virtual size_t getNumberFrame() const;
      virtual void setNumberFrame(size_t i_nb_frames);

      // Preparing the camera's SDK to acquire frames
      void prepareAcq();
      // Launches the SDK's acquisition and the m_acq_thread to retrieve frame buffers as they are ready
      void startAcq();
      // Stops the acquisition, as soon as the m_acq_thread is retrieving frame buffers.
      void stopAcq();
      
      // Getting the buffer control object :
      HwBufferCtrlObj* getBufferCtrlObj();
      
    public:  // Accessing the parameters known from the frame-grabber :
      FgParamTypes getParamterType(const int i_param_id) const;
      template <class T>
      int setParameter(const int i_param_id, T i_value);
      template <class T>
      int getParameter(const int i_param_id, T *o_value) const;
      template <class T>
      int setParameterNamed(const std::string& i_param_name, T i_value, int * o_param_id=NULL);
      template <class T>
      int getParameterNamed(const std::string& i_param_name, T *o_value, int * o_param_id=NULL) const;
      
      void setWidth(uint32_t i_val);
      void getWitdh(uint32_t &o_val) const;
      void setHeight(uint32_t i_val);
      void getHeight(uint32_t &o_val) const;
      void setDeviceTimeout(uint32_t i_val);
      void getDeviceTimeout(uint32_t &o_val) const;
      void setPixelFormat(siso_px_format i_val);
      void getPixelFormat(siso_px_format &o_val) const;
      void getBytePerPixel(size_t &o_val) const;
      
      SerialLine& getSerialLine();
      
    protected:
      //! Performing all the initialisation code… Ideally an init on the camera (in tango term)
      // could be limited to call this method.
      void init();
      //! Stopping an acquisition, iForce : without waiting the end of frame buffer retrieval by m_acq_thread (thread-safe)
      void doStopAcq(bool iImmediate);
      //! Setting the status in a thread safe manner (thread-safe)
      void setStatus(Grabber::Status iStatus, bool iForce);

      // ??? Those ones might be at init and un-mutable then ?
//      void setCameraPort(unsigned int i_port);
//      void getCameraPort(unsigned int &o_port) const;
      
    private:
      class AcqThread;
      friend class AcqThread;

      struct ParamInfo {
        int						m_id;
        std::string		m_name;
        
        ParamInfo(int i_id, std::string i_name) :
        m_id(i_id), m_name(i_name)
        {}
        ParamInfo() :
        m_id(-1), m_name("NO PARAM")
        {}
      };

    protected:
      std::string			m_applet_filename;
      unsigned int		m_board_index;
      unsigned int    m_cam_port;
      unsigned int		m_dma_index;
      Fg_Struct				*m_fg;
      SerialLine			*m_serial_line;
      std::vector<ParamInfo*>		m_param_table;

      // -- Members
      // LIMA / Acquisition (thread) related :
      SoftBufferCtrlObj						m_buffer_ctrl_obj;
      dma_mem											*m_next_dma_head;
      // Pure thread and signals :
      AcqThread*                	m_acq_thread;						// The thread retieving frame buffers from the SDK
      Cond                        m_cond;									// Waiting condition for inter thread signaling
      volatile bool								m_acq_thread_waiting;   // The m_acq_thread is waiting (main uses it to tell it to stop waiting)
      volatile bool								m_acq_thread_running;		// The m_acq_thread is running (main uses it to accept stopAcq)
      volatile bool								m_acq_thread_should_quit; // The main thread signals to m_acq_thread that it should quit.
      
      // A bit more general :
      size_t											m_nb_frames_to_collect; // The number of frames to collect in current sequence
      size_t											m_image_index;					// The index in the current sequence of the next image to retrieve
      bool												m_buffer_ringing;				// Should the buffer be considered as a ring buffer rather than a single use buffer.
      Status											m_status;								// The current status of the camera
    };

    template <class T>
    int
    Grabber::setParameter(const int i_param_id, T i_value)
    {
      DEB_MEMBER_FUNCT();
      if ( m_fg ) {
        return sisoError(Fg_setParameterWithType(m_fg, i_param_id, i_value, m_dma_index));
      }
      DEB_WARNING() << "Tried to set a parameter to a frame-grabber that is not properly initialised";
      return 0;
    }
    
    template <class T>
    int
    Grabber::getParameter(const int i_param_id, T *o_value) const
    {
      DEB_MEMBER_FUNCT();
      if ( m_fg ) {
        return sisoError(Fg_getParameterWithType(m_fg, i_param_id, o_value, m_dma_index));
      }
    }

    template <class T>
    int
    Grabber::setParameterNamed(const std::string& i_param_name, T i_value, int * o_param_id)
    {
      int the_id = Fg_getParameterIdByName(m_fg, i_param_name.c_str());
      if ( o_param_id )
        *o_param_id = the_id;
      return setParameter(the_id, i_value);
    }
    template <class T>
    int
    Grabber::getParameterNamed(const std::string& i_param_name, T *o_value, int * o_param_id) const
    {
      int the_id = Fg_getParameterIdByName(m_fg, i_param_name.c_str());
      if ( o_param_id )
        *o_param_id = the_id;
      return getParameter(the_id, o_value);
    }

    
  } // namespace siso_me4
} // namespace lima

#endif // SISOME4GRABBER_H