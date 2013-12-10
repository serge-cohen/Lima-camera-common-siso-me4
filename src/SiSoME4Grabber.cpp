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

// System headers :

// Camera SDK headers :

// LImA headers :

// SiSo-ME4 plugin headers :
#include "SiSoME4Grabber.h"


//---------------------------
//- utility thread
//---------------------------
namespace lima {
  namespace siso_me4 {
    class Grabber::AcqThread : public Thread
    {
      DEB_CLASS_NAMESPC(DebModCamera, "Grabber", "AcqThread");
    public:
      AcqThread(Grabber &aGrabber);
      virtual ~AcqThread();
      
    protected:
      virtual void threadFunction();
      
    private:
      Grabber&    m_grabber;
    };
  }
}




// lima::siso_me4::Grabber::
lima::siso_me4::Grabber::Grabber(int board_index, int cam_port, const std::string& applet_name, unsigned int dma_index) :
m_applet_filename(applet_name),
m_board_index(board_index),
m_cam_port(cam_port),
m_dma_index(dma_index),
m_fg(NULL),
m_serial_line(NULL),
m_param_table(),
m_buffer_ctrl_obj(),
m_acq_thread(NULL),
m_cond(),
m_acq_thread_waiting(true),
m_acq_thread_running(false),
m_acq_thread_should_quit(false),
m_nb_frames_to_collect(1),
m_image_index(0),
m_buffer_ringing(false),
m_status(Ready)
{
  DEB_CONSTRUCTOR();
  init();
}

lima::siso_me4::Grabber::~Grabber()
{
  // Forcing the stop of the acquisition :
  if ( m_acq_thread ) {
    doStopAcq(true);
    delete m_acq_thread;
    m_acq_thread = NULL;
  }
  
  if ( m_fg ) {
    if ( ! sisoError(Fg_FreeGrabber(m_fg)) ) {
      m_fg = NULL;
    }
  }
}


int
lima::siso_me4::Grabber::sisoError(int code) const
{
  DEB_MEMBER_FUNCT();
  if ( (m_fg) && (FG_OK != code) ) {
    DEB_WARNING() << "Got an error in lima::siso_me4::Grabber : " << code << ", meaning\n\t"
    << Fg_getErrorDescription(m_fg, code);
  }
  return code;
}

// Handling the number of frames to collect:

size_t
lima::siso_me4::Grabber::getNumberFrame() const
{
  return m_nb_frames_to_collect;
}

void
lima::siso_me4::Grabber::setNumberFrame(size_t i_nb_frames)
{
  m_nb_frames_to_collect = i_nb_frames;
}

// Preparing the camera's SDK to acquire frames
void
lima::siso_me4::Grabber::prepareAcq()
{
#warning SHOULD implement that later
}
// Launches the SDK's acquisition and the m_acq_thread to retrieve frame buffers as they are ready
void
lima::siso_me4::Grabber::startAcq()
{
#warning SHOULD implement that later
}
// Stops the acquisition, as soon as the m_acq_thread is retrieving frame buffers.
void
lima::siso_me4::Grabber::stopAcq()
{
#warning SHOULD implement that later
}


void
lima::siso_me4::Grabber::setWidth(unsigned int i_val)
{
  uint32_t		the_value = i_val;
  setParameter(FG_WIDTH, the_value);
}

void
lima::siso_me4::Grabber::getWitdh(unsigned int &o_val) const
{
  uint32_t		the_value;
  getParameter(FG_WIDTH, &the_value);
  o_val = the_value;
}

void
lima::siso_me4::Grabber::setHeight(unsigned int i_val)
{
  uint32_t		the_value = i_val;
  setParameter(FG_HEIGHT, the_value);
}

void
lima::siso_me4::Grabber::getHeight(unsigned int &o_val) const
{
  uint32_t		the_value;
  getParameter(FG_HEIGHT, &the_value);
  o_val = the_value;
}

void
lima::siso_me4::Grabber::setDeviceTimeout(unsigned int i_val)
{
  uint32_t		the_value = i_val;
  setParameter(FG_TIMEOUT, the_value);
}

void
lima::siso_me4::Grabber::getDeviceTimeout(unsigned int &o_val) const
{
  uint32_t		the_value;
  getParameter(FG_TIMEOUT, &the_value);
  o_val = the_value;
}

void
lima::siso_me4::Grabber::setPixelFormat(siso_px_format i_val)
{
  uint32_t		the_value = static_cast<int>(i_val);
  setParameter(FG_FORMAT, the_value);
}

void
lima::siso_me4::Grabber::getPixelFormat(siso_px_format &o_val) const
{
  uint32_t		the_value;
  getParameter(FG_FORMAT, &the_value);
  o_val = static_cast<siso_px_format>(the_value);
}


void
lima::siso_me4::Grabber::init()
{
  DEB_MEMBER_FUNCT();
  
  // If ever the frame-grabber structure was already initted, release it :
  if ( m_acq_thread ) {
    doStopAcq(true);
    delete m_acq_thread;
    m_acq_thread = NULL;
  }
  if ( m_fg ) {
    if ( ! sisoError(Fg_FreeGrabber(m_fg)) ) {
      m_fg = NULL;
      // Purging the table of parameters :
      while ( ! m_param_table.empty() ) {
        delete m_param_table.back();
        m_param_table.pop_back();
      }
    }
  }
  if ( m_serial_line ) {
    delete m_serial_line;
    m_serial_line = NULL;
  }

  // And now we are allocating the free-store resources :
  m_fg = Fg_Init(m_applet_filename.c_str(), m_board_index);
  if ( NULL== m_fg ) {
    DEB_WARNING() << "In lima::siso_me4::Grabber::init : UNABLE TO INIT the frame-grabber structure !!!";
    return;
  }
  // Constructing the table of parameters :
  DEB_TRACE() << "Populating the parameter table for the SiSo me4 from the VA : " << m_applet_filename;
  int			the_num_param = Fg_getNrOfParameter(m_fg);
  DEB_TRACE() << "This visual applet has " << the_num_param << " parameters";
  
  m_param_table.reserve(the_num_param);
  for (int i=0; the_num_param != i; ++i) {
    int							the_id = Fg_getParameterId(m_fg, i);
    std::string			the_name = std::string(Fg_getParameterName(m_fg, i));
    m_param_table.push_back(new ParamInfo(the_id, the_name));
    DEB_TRACE() << "Getting the parameter indexed " << i << ", ID is " << the_id << ", name is '" << the_name << "'";
  }
  
  // Getting the serial line :
  m_serial_line = new lima::siso_me4::SerialLine(2*m_board_index + m_cam_port, SerialLine::BR19200, SerialLine::Off, "\r\n", 10.0);
  
  // And the acquisition thread :
  m_acq_thread = new AcqThread(*this);
  // And start it :
  m_acq_thread->start();
}

void
lima::siso_me4::Grabber::doStopAcq(bool iImmediate)
{
  
}

void
lima::siso_me4::Grabber::setStatus(Grabber::Status iStatus, bool iForce)
{
  
}


//-----------------------------------------------------
// Taking care of the imptrlementation of the acquisition thread (_AcqThread / m_acq_thread)
//-----------------------------------------------------
lima::siso_me4::Grabber::AcqThread::AcqThread(Grabber &aGrabber) :
m_grabber(aGrabber)
{
  DEB_CONSTRUCTOR();
  // This seems to be useless since :
  // Linux supports PTHREAD_SCOPE_SYSTEM, but not PTHREAD_SCOPE_PROCESS.
  // (http://man7.org/linux/man-pages/man3/pthread_attr_setscope.3.html 2013-04-19) ?
  //  pthread_attr_setscope(&m_thread_attr, PTHREAD_SCOPE_PROCESS);
}


// Signaling to the m_acq_thread that it should quit, then waiting for it to do it.
lima::siso_me4::Grabber::AcqThread::~AcqThread()
{
  DEB_DESTRUCTOR();
  DEB_TRACE() << "Asking the acquisition thread to stop (setting m_cam.m_acq_thread_should_quit to true).";
  AutoMutex the_lock(m_grabber.m_cond.mutex());
  m_grabber.m_acq_thread_should_quit = true;
  // Signaling the acquisition thread to check if something should be done :
  m_grabber.m_cond.broadcast();
  the_lock.unlock();
  
  DEB_TRACE() << "Waiting for the acquisition thread to be done (joining the main thread).";
  join();
}


// Either waiting to be signaled, or looping with the SDK to retrieve frame buffers
void
lima::siso_me4::Grabber::AcqThread::threadFunction()
{
}

