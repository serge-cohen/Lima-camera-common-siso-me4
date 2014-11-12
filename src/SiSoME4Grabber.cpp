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
#include <limits.h>
#include <execinfo.h>

// Temporary : for ns timing in prepare
#include <time.h>


// Camera SDK headers :

// LImA headers :

// SiSo-ME4 plugin headers :
#include "SiSoME4Grabber.h"

// #define siso_me4_assert(ERR) if (sisoError(ERR)) { DEB_WARNING() << "Error code from " << __func__ << "(" << __FILE__ << "." << __LINE__ << ").\n"; }

static double mus_diff(struct timespec *end, struct timespec *orig);

double
mus_diff(struct timespec *end, struct timespec *orig)
{
  return (double)(end->tv_sec - orig->tv_sec) * 1000000.0 + (double)(end->tv_nsec - orig->tv_nsec)/1000.0;
}

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
lima::siso_me4::Grabber::Grabber(const std::string& i_siso_dir_5, int board_index, int cam_port, const std::string& applet_name, unsigned int dma_index) :
m_applet_filename(applet_name),
m_board_index(board_index),
m_cam_port(cam_port),
m_dma_index(dma_index),
m_fg(NULL),
m_serial_line(NULL),
m_param_table(),
m_buffer_ctrl_obj(),
m_next_dma_head(NULL),
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
  if ( ! i_siso_dir_5.empty() ) {
    setenv("SISODIR5", i_siso_dir_5.c_str(), true);
  }
  else {
    setenv("SISODIR5", "/opt/siso", false);
  }

  init();
}

lima::siso_me4::Grabber::~Grabber()
{
  DEB_DESTRUCTOR();
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
    else {
      DEB_WARNING() << "Unable to free the frame-grabber pointer : " << m_fg;
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
    
    void *stack_addr[50];
    std::size_t stack_size;
    char **stack_symb;
    stack_size = backtrace(stack_addr, 50);
    stack_symb = backtrace_symbols(stack_addr, stack_size);

    DEB_WARNING() << "\n\tPrinting stack-trace (gcc) (" << stack_size << " stack frames):";
    for (std::size_t i=0; stack_size!=i; ++i) {
      DEB_WARNING() << "\t(" << i << ") : " << stack_symb[i];
    }
    free(stack_symb);
    return code;
  }
  return (FG_OK != code) ? code : 0; // returning explicit 0 if no error occured.
}

// Handling the number of frames to collect:

frameindex_t
lima::siso_me4::Grabber::getNumberFrame() const
{
  DEB_MEMBER_FUNCT();
  return m_nb_frames_to_collect;
}

void
lima::siso_me4::Grabber::setNumberFrame(frameindex_t i_nb_frames)
{
  DEB_MEMBER_FUNCT();
  DEB_PARAM() << DEB_VAR1(i_nb_frames);
  m_nb_frames_to_collect = i_nb_frames;
  DEB_TRACE() << "Setting the number of frames to collect to " << m_nb_frames_to_collect;
}

frameindex_t
lima::siso_me4::Grabber::getNbHwAcquiredFrames() const
{
  DEB_MEMBER_FUNCT();
  return m_image_index;
}


void
lima::siso_me4::Grabber::getStatus(Status &o_status) const
{
  DEB_MEMBER_FUNCT();
  AutoMutex aLock(m_cond.mutex());
  o_status = m_status;
  DEB_RETURN() << DEB_VAR1(DEB_HEX(o_status));
}


/*!
 @brief Preparing the grabber and memory for an acquisition sequence
 
 This method is called just before the acquisition is started (that is startAcq).
 It is responsible for setting both the internals of the grabber and the
 internals of the graber's SDK for the planned acquisition sequence.
 Most of the work consists in the setting of the frame buffers, and passing them to
 the frame-grabber SDK runtime.
 */
/*!
 Maybe later : write a specific SoftBufferAllocMgr or SoftBufferCtrlObj
 subclass that better handles :
 * the possibility to have memory directly allocated by the SDK, and later better
 handled by the SDK
 */
void
lima::siso_me4::Grabber::prepareAcq()
{
  DEB_MEMBER_FUNCT();
  // Setting the next image index to 0 (since we are starting at 0):
  m_image_index = 0;

  // Taking some dates, at ns "resolution" :
  clockid_t       my_clock = CLOCK_MONOTONIC_RAW; // but could be CLOCK_PROCESS_CPUTIME_ID...
  struct timespec d_start, d_alloc_buf_s, d_alloc_buf_d, d_alloc_head_s, d_alloc_head_d, d_buf_to_fg_s, d_buf_to_fg_d;

  clock_gettime(my_clock, &d_start);

  // Retrieving the information required to allocate the memory:
  uint32_t				the_width, the_height;
  siso_px_format	the_px_format;
  size_t					the_bytes_per_px;
  
  //  setStatus(Running);
  getWitdh(the_width);
  getHeight(the_height);
  //  getPixelFormat(the_px_format);
  getBytePerPixel(the_bytes_per_px);
  
  // Retrieving useful information from the camera :
  size_t			the_image_size = the_width * the_height * the_bytes_per_px;
  
  DEB_TRACE() << "Getting the basic information from the camera : image size (in bytes) and pixel encoding.";
  DEB_ALWAYS() << "The image size in bytes is " << the_image_size
  << " (" << the_width << "x" << the_height
  << "), and the pixel encoding index is " << static_cast<uint32_t>(the_px_format)
  << ". Corresponding to " << the_bytes_per_px << "Bytes per pixel";
  
  DEB_TRACE() << "Image size parameters are : width " << the_width
  << ", height " << the_height
  << " and finaly byte per pixel " << the_bytes_per_px;
  
  // Converting the size into a lima frame size
  lima::Size	the_frame_size(static_cast<int>(the_width), static_cast<int>(the_height));
  FrameDim		the_frame_dim;  // Adding the information from the image/pixel format/depth
  the_frame_dim.setSize(the_frame_size);
  
//   // Setting the other information of the frame :
//   switch (the_px_format) {
//     case siso_px_8b:
//       the_frame_dim.setImageType(Bpp8);
//       break;
//     case siso_px_16b:
//       the_frame_dim.setImageType(Bpp16);
//       break;
//     case siso_px_rgb_24b:
//       the_frame_dim.setImageType(Bpp8);
//       DEB_TRACE() << "You have hit a bug in term of SiSo RGB-24bpp to lima type conversion";
// #warning SHOULD implement that later
//       break;
//     case siso_px_rgb_32b:
//       the_frame_dim.setImageType(Bpp8);
//       DEB_TRACE() << "You have hit a bug in term of SiSo RGB-32bpp to lima type conversion";
// #warning SHOULD implement that later
//       break;
//     default:
//       //! TODO : again trouble (signal to user), we don't know how to do that.
//       break;
//   }
  the_frame_dim.setImageType(Bpp16);
  
  int 				the_max_frames;
  int					the_alloc_frames;
  
  the_alloc_frames = (0 == m_nb_frames_to_collect) ? 128 : static_cast<int>(m_nb_frames_to_collect);
  DEB_ALWAYS() << "The number of frames to be collected is set to : " << the_alloc_frames << ", before testing the memory available";
  
  m_buffer_ctrl_obj.setFrameDim(the_frame_dim);
  //  m_buffer_ctrl_obj.getMaxNbBuffers(the_max_frames);
  //  the_max_frames = the_max_frames >> 2;
  the_max_frames = 64;
  DEB_ALWAYS() << "Given above parameters, maximum number of frames in memory is "
  << the_max_frames;
  if (( the_max_frames < the_alloc_frames ) || ( 0 == m_nb_frames_to_collect )) {
    // If not enough memory or continuous acquisition we go into ring buffer mode :
    the_alloc_frames = ( the_max_frames < the_alloc_frames ) ? the_max_frames : the_alloc_frames ;
    m_buffer_ringing = true;
    DEB_ALWAYS() << "Setting ring mode, since we are either in continuous acquisition or not enough memory";
  }
  else {
    // Otherwise we allocate exactly th number of buffers : one per requested frame
    the_alloc_frames = static_cast<int>(m_nb_frames_to_collect);
    m_buffer_ringing = false;
    DEB_ALWAYS() << "Setting the buffer single use mode";
  }
  
  DEB_ALWAYS() << "After testing available memory (and continuous acquisition), the mode is " << m_buffer_ringing << " and the number of frame to be allocated is : " << the_alloc_frames;
  
  clock_gettime(my_clock, &d_alloc_buf_s);
  StdBufferCbMgr& the_buffer = m_buffer_ctrl_obj.getBuffer();
  DEB_ALWAYS() << "Getting StdBufferCbMgr to allocate the buffers that we want to have";
  the_buffer.allocBuffers(the_alloc_frames, 1, the_frame_dim);
  int				the_frame_mem_size = the_frame_dim.getMemSize();
  clock_gettime(my_clock, &d_alloc_buf_d);

  if ( the_frame_mem_size != the_image_size ) {
    DEB_WARNING() << "You most likely hit an error condition where the a priori computed memory size of a frame ("
    << the_image_size << "B) is different from the memory allocated by lima StdBufferCbMgr ("
    << the_frame_mem_size << "B).";
  }
  
  // Handing the frame buffers to the SDK :
  clock_gettime(my_clock, &d_alloc_head_s);
  if ( m_next_dma_head ) {
    // As proposed by the SDK, we make sure that we start from an empty queue :
    DEB_ALWAYS() << "Flushing the queue of the framegrabber";
    Fg_FreeMemHead(m_fg, m_next_dma_head);
    m_next_dma_head = NULL;
  }

  // Preparing the next memory header :
  DEB_TRACE() << "Allocating the DMA memory header";
  m_next_dma_head = Fg_AllocMemHead(m_fg, the_image_size * static_cast<size_t>(the_alloc_frames), static_cast<frameindex_t>(the_alloc_frames));

  clock_gettime(my_clock, &d_alloc_head_d);
  
  // Then queue all the buffers allocated by StdBufferCbMgr
  DEB_TRACE() << "Pushing all the frame buffers to the frame grabber SDK";
  clock_gettime(my_clock, &d_buf_to_fg_s);
  for ( int i=0; the_alloc_frames != i; ++i) {
    void*		the_buffer_ptr = the_buffer.getFrameBufferPtr(i);
    //    std::cout << "About to add to the frame grabber the memory at pointer : " << the_buffer_ptr << " of size " << the_image_size << std::endl;
    int                 the_err_code;
    if ( 0 > (the_err_code = Fg_AddMem(m_fg, the_buffer_ptr, the_image_size, i, m_next_dma_head)))  {
      siso_me4_assert(the_err_code);
      DEB_WARNING() << "Unable to push the subbuffer " << i << " to the DMA memory for frame grabbing.\n"
		    << "Error code is " << the_err_code;
      //      THROW_HW_ERROR(Error) << "Unable to push the subbuffer " << i << " to the DMA memory for frame grabbing.";
    }
    else {
      if ( 0 == (i % 200) ) {
	DEB_ALWAYS() << "Just accepted the subbuffer index : " << the_err_code << " compared to i : " << i;
      }
    }
    DEB_TRACE() << "Queueing the frame buffer " << i << " done (should work).";
  }
  clock_gettime(my_clock, &d_buf_to_fg_d);
  DEB_ALWAYS() << "Finished queueing " << the_alloc_frames << " frame buffers to SiSo ME4 frame-grabber.";
  DEB_ALWAYS() << "Reporting some timing :\n" 
	       << "\tFull prepare timing " << mus_diff(&d_buf_to_fg_d, &d_start) << "mus, which is decomposed in :\n" 
	       << "\t\tSize computation " << mus_diff(&d_alloc_buf_s, &d_start) << "mus\n"
	       << "\t\tBuffer allocation (lima) " << mus_diff(&d_alloc_buf_d, &d_alloc_buf_s) << "mus\n"
	       << "\t\tBuffer's head allocation (siso) " << mus_diff(&d_alloc_head_d, &d_alloc_head_s) << "mus\n"
	       << "\t\tTransferring the buffers to the framgrabber " << mus_diff(&d_buf_to_fg_d, &d_buf_to_fg_s) << "mus\n" 
	       << "*******************************************************\n";
  // Seems to me that the «0 == m_nb_frames_to_collect» case corresponds to the continuous case
  // So next line is not making sense (and hence commented out) :
  // #warning Setting properly the continuous vs. fixed acquisition mode of the camera

  // No other specific settings should be done to tell the grabber if we are in
  // free-running or fixed number of frame mode (or so it seems).
}


/*!
 @brief Launching an acquisition sequence, including the frame retrieving thread.
 
 This method is called at the launch of the acquisition sequence, after the
 prepareAcq method but before any frame can be shot.
 
 This method is responsible to perform the following actions :
 * Tell the SDK to launch the acquisition sequence
 * Signal to the m_acq_thread that it should resume running
 because some frames will soon be available to retrieve
 */
void
lima::siso_me4::Grabber::startAcq()
{
  DEB_MEMBER_FUNCT();
  
  DEB_TRACE() << "Starting the acquisition by the camera (or triggering when in software trigger mode)";
  
  if ( 0 == m_image_index ) {
    // Setting the start timestamp of the buffer :
    m_buffer_ctrl_obj.getBuffer().setStartTimestamp(Timestamp::now());
    // Sending the start command to the SDK, depending of video mode (infinite) or not (known in advance number of frames).
    frameindex_t		the_nr_grab = (0 != m_nb_frames_to_collect) ?  m_nb_frames_to_collect : GRAB_INFINITE;
    //    std::cout << "fg : " << m_fg << "\ndma_index : " << m_dma_index << "\ndma_head : " << m_next_dma_head << std::endl;

    siso_me4_assert(Fg_AcquireEx(m_fg, m_dma_index, the_nr_grab, ACQ_BLOCK, m_next_dma_head));
  }
  
//  // Later on, should handle Software (Software_multi) triggering ...
//  // This is done through one of the functions Fg_sendSoftwareTrigger Fg_sendSoftwareTriggerEx
//  if ( Software == m_trig_mode ) {
//    // If we are in software trigger mode, the call to startAcq serves as the trigger :
//    sendCommand(siso_me4::SoftwareTrigger);
//  }
  
  DEB_TRACE() << "Resuming the action of the acquisition thread";
  AutoMutex    the_lock(m_cond.mutex());
  m_acq_thread_waiting = false;
  m_cond.broadcast();
  
  DEB_TRACE() << "Done, the acquisition is now started and the frame retrieving should take place in parallel in a second thread";
}

// Stops the acquisition, as soon as the m_acq_thread is retrieving frame buffers.
void
lima::siso_me4::Grabber::stopAcq()
{
  DEB_MEMBER_FUNCT();
  doStopAcq(false);
}

lima::HwBufferCtrlObj*
lima::siso_me4::Grabber::getBufferCtrlObj()
{
  DEB_MEMBER_FUNCT();
  return &m_buffer_ctrl_obj;
}

FgParamTypes
lima::siso_me4::Grabber::getParamterType(const int i_param_id) const
{
  DEB_MEMBER_FUNCT();
  FieldParameterAccess the_access;
  the_access.p_uint32_t = NULL;
  the_access.count = 0;
  
  if ( sisoError(Fg_getParameterWithType(m_fg, i_param_id, &the_access, m_dma_index, FG_PARAM_TYPE_STRUCT_FIELDPARAMACCESS)) ) {
    DEB_WARNING() << "Problem with getting a parameter within a FieldParameterAccess to get the parameter type";
  }
  return the_access.vtype;
}


void
lima::siso_me4::Grabber::setWidth(uint32_t i_val)
{
  DEB_MEMBER_FUNCT();

  uint32_t		the_value = i_val;
  setParameter(FG_WIDTH, the_value);
}

void
lima::siso_me4::Grabber::getWitdh(uint32_t &o_val) const
{
  DEB_MEMBER_FUNCT();

  uint32_t		the_value;
  getParameter(FG_WIDTH, &the_value);
  o_val = the_value;
}

void
lima::siso_me4::Grabber::setHeight(uint32_t i_val)
{
  DEB_MEMBER_FUNCT();

  uint32_t		the_value = i_val;
  setParameter(FG_HEIGHT, the_value);
}

void
lima::siso_me4::Grabber::getHeight(uint32_t &o_val) const
{
  DEB_MEMBER_FUNCT();

  uint32_t		the_value;
  getParameter(FG_HEIGHT, &the_value);
  o_val = the_value;
}

void
lima::siso_me4::Grabber::setDeviceTimeout(uint32_t i_val)
{
  DEB_MEMBER_FUNCT();

  uint32_t		the_value = i_val;
  setParameter(FG_TIMEOUT, the_value);
}

void
lima::siso_me4::Grabber::getDeviceTimeout(uint32_t &o_val) const
{
  DEB_MEMBER_FUNCT();

  uint32_t		the_value;
  getParameter(FG_TIMEOUT, &the_value);
  o_val = the_value;
}

void
lima::siso_me4::Grabber::setPixelFormat(siso_px_format i_val)
{
  DEB_MEMBER_FUNCT();

  uint32_t		the_value = static_cast<int>(i_val);
  setParameter(FG_FORMAT, the_value);
}

void
lima::siso_me4::Grabber::getPixelFormat(siso_px_format &o_val) const
{
  DEB_MEMBER_FUNCT();

  uint32_t		the_value;
  getParameter(FG_FORMAT, &the_value);
  o_val = static_cast<siso_px_format>(the_value);
}

void
lima::siso_me4::Grabber::getBytePerPixel(size_t &o_val) const
{
  DEB_MEMBER_FUNCT();

  siso_px_format		the_format;
  getPixelFormat(the_format);
  switch (the_format) {
    case siso_px_8b:
      o_val = 1;
      break;
    case siso_px_16b:
      o_val = 2;
      break;
    case siso_px_rgb_24b:
      o_val = 3;
      break;
    case siso_px_rgb_32b:
      o_val = 4;
      break;
      
    default:
      
      break;
  }
}

lima::siso_me4::SerialLine&
lima::siso_me4::Grabber::getSerialLine()
{
  return *m_serial_line;
}


void
lima::siso_me4::Grabber::init()
{
  DEB_MEMBER_FUNCT();
  
  // If the acquisition is taking place, make sure we stop it (including releasing the dma_mem head).
  if ( m_acq_thread ) {
    doStopAcq(true);
    delete m_acq_thread;
    m_acq_thread = NULL;
  }
  // If ever the frame-grabber structure was already initted, release it :
  if ( m_fg ) {
    if ( ! sisoError(Fg_FreeGrabber(m_fg)) ) {
      m_fg = NULL;
      // Purging the table of parameters :
      while ( ! m_param_table.empty() ) {
        delete m_param_table.back();
        m_param_table.pop_back();
      }
    }
    else {
      DEB_WARNING() << "Got an error while trying to free the frame grabber " << m_fg << ", within lima::siso_me4::Grabber::init() (" << __func__ << "(" << __FILE__ << "." << __LINE__ << ")."; 
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
  DEB_MEMBER_FUNCT();

  AutoMutex		the_lock(m_cond.mutex());
  bool				the_camera_acq;

  if ( iImmediate ) {
    DEB_TRACE() << "Sending a FORCEFULL acquisition stop, whatever is the current status because you asked for immediate stop";
    Fg_stopAcquireEx(m_fg, m_dma_index, m_next_dma_head, STOP_ASYNC); // Since immediate was requested perform an ASYNC stop.
    DEB_TRACE() << "Sent  Fg_stopAcquireEx(STOP_ASYNC) in doStopAcq immediate == TRUE";
    // Deblocking the acquisition thread (in case it is waiting)
    m_acq_thread_waiting = true; // Asking to stop as soon as not in charge
                                 // This is dangerous: causes the lock inside the lock ?
                                 //    _setStatus(Ready, false);
    m_status = Ready;
    // Finally broadcasting :
    m_cond.broadcast();
    DEB_TRACE() << "Returning from the FORCEFULL doStopAcq";
    return;
  }
  
  DEB_TRACE() << "Requested a stop, but waiting the opportunity in the acquisition thread";
  //  if ( the_camera_acq || (Ready != m_status) ) {
  if ( ! m_acq_thread_waiting ) {
#warning The current implementation looses the acquired frames that were not transfered yet, in the case of a stopAcq.
    while ( (! iImmediate) && (m_acq_thread_running) ) { // We are still actively retrieving frame buffers
                                                         //  the_lock.unlock();
                                                         //  the_lock.lock();
      m_acq_thread_waiting = true; // Asking to stop as soon as not in charge
      m_cond.broadcast();
      DEB_TRACE() << "Setting ourselves to wait a signal";
      m_cond.wait();
      DEB_TRACE() << "We just received a signal that it is usefull to check again if the acquisition thread is in a stoppable state";
    }
    DEB_TRACE() << "We finally got the signal that the currently exposed frame is now grabbed... We can go on for the acquisition stop";
    the_lock.unlock();
    
    // If we were not asked for immediate leaving, let the acquisition thread end it :
    //    if ( ! iImmediate ) {
    //      return;
    //    }
    
    DEB_TRACE() << "We should now STOP the acquisition";
    Fg_stopAcquireEx(m_fg, m_dma_index, m_next_dma_head, STOP_ASYNC); // We will not process those buffers anyway...
    DEB_TRACE() << "Sent a Fg_stopAcquireEx in doStopAcq immediate == FALSE." << "\n\t\tAnd now flushing the buffer queue since no more acquisition is in flight";
    if ( m_next_dma_head ) {
      Fg_FreeMemHead(m_fg, m_next_dma_head);
      m_next_dma_head = NULL;
    }
    DEB_TRACE() << "Finally setting the status to Ready";
    setStatus(Ready, false);
  }
  else {
    DEB_TRACE() << "One more doStopAcq, while the acquisition is either already stopped or about to be stopped... Skip the current one";
  }
  return;  
}

void
lima::siso_me4::Grabber::setStatus(Grabber::Status iStatus, bool iForce)
{
  DEB_MEMBER_FUNCT();
  DEB_TRACE() << "in _setStatus, about to ask for the lock to mutex";
  AutoMutex aLock(m_cond.mutex());
  DEB_TRACE() << "grabbed the lock";
  if( iForce || (Grabber::Fault != m_status) )
    m_status = iStatus;
  m_cond.broadcast();
  DEB_TRACE() << "_setStatus broadcasting and releasing soon the mutex lock.";
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
  DEB_MEMBER_FUNCT();
  AutoMutex					the_lock(m_grabber.m_cond.mutex());
  StdBufferCbMgr		&the_buffer = m_grabber.m_buffer_ctrl_obj.getBuffer();
  
  while (! m_grabber.m_acq_thread_should_quit ) {
    DEB_TRACE() << "[siso_me4 acquisition thread] Top of the loop";
    // We are looping until being «signaled» that we should quit.
    while ( m_grabber.m_acq_thread_waiting && ! m_grabber.m_acq_thread_should_quit ) {
      // Lets wait a signal telling that maybe something has to be done …
      DEB_TRACE() << "[siso_me4 acquisition thread] Setting the m_acq_thread_running to false (since we are waiting)";
      m_grabber.m_acq_thread_running = false; // Making sure the main class/thread knows nothing goes on
      m_grabber.m_cond.broadcast();
      DEB_TRACE() << "[siso_me4 acquisition thread] Waiting acquisition start";
      m_grabber.m_cond.wait();
    }
    // The main thread asked to get out of wait mode (setting m_cam.m_acq_thread_waiting to false)
    DEB_ALWAYS() << "[siso_me4 acquisition thread] Set running by main thread setting m_cam.m_acq_thread_waiting to false (or m_cam.m_acq_thread_should_quit to true)";
    m_grabber.m_acq_thread_running = true;
    
    if ( m_grabber.m_acq_thread_should_quit ) {
      // Should return ASAP, so that the thread could be «joined».
      DEB_TRACE() << "[siso_me4 acquisition thread] Quitting under request from main thread (m_acq_thread_should_quit)";
      return;
    }
    
    m_grabber.m_status = Grabber::Running;
    m_grabber.m_cond.broadcast();
    the_lock.unlock();
    
    DEB_TRACE() << "[siso_me4 acquisition thread] About to start looping to get the images-frame retrieved";
    bool						the_acq_goon = true;
    unsigned int		the_wait_timeout = 10;

    while ( the_acq_goon && ((0 == m_grabber.m_nb_frames_to_collect) || (m_grabber.m_nb_frames_to_collect != m_grabber.m_image_index)) ) {
#warning This setting might be problematic for cameras accepting (and used with) exposure time larger than a bit less than 10s !
      frameindex_t			the_new_frame;
      
      
      DEB_TRACE() << "[siso_me4 acquisition thread] Waiting for buffer index " << m_grabber.m_image_index << " (Fg_getImageEx)";
      //      std::cout << "[siso_me4 AT] fg : " << m_grabber.m_fg << "\ndma_index : " << m_grabber.m_dma_index << "\ndma_head : " << m_grabber.m_next_dma_head << std::endl;
      the_new_frame = Fg_getImageEx(m_grabber.m_fg, SEL_NEXT_IMAGE, 0, m_grabber.m_dma_index, 20, m_grabber.m_next_dma_head);
      if ( 0 == (m_grabber.m_image_index % 50) ) {
	struct timespec d_new;
	double actual_fps=0.0;
	double from_start_fps=0.0;
	clock_gettime(CLOCK_MONOTONIC_RAW, &d_new);
	if ( 0 == m_grabber.m_image_index ) {
	  m_grabber.m_date_start.tv_sec = d_new.tv_sec;
	  m_grabber.m_date_start.tv_nsec = d_new.tv_nsec;
	}
	if ( 0 != m_grabber.m_image_index ) {
	  actual_fps = 50.0 / mus_diff(&d_new, &(m_grabber.m_date_last_50)) * 1.0e6;
	  from_start_fps = static_cast<double>(m_grabber.m_image_index) /
	    mus_diff(&d_new, &(m_grabber.m_date_start)) * 1.0e6;
	}
	DEB_ALWAYS() << "[siso_me4 acquisition thread] DONE waiting for buffer index " << m_grabber.m_image_index 
		     << ", measured frame rate (on last 50) is " << actual_fps << "f/s, and " 
		     << from_start_fps << "f/s from sequence's start.";
	m_grabber.m_date_last_50.tv_sec = d_new.tv_sec;
	m_grabber.m_date_last_50.tv_nsec = d_new.tv_nsec;
      }
      // Testing if we were asked to stop the acquisition thread :
      // It is best to do that as soon as returning from the SDK, since otherwise it might block some
      // thread interacting with the main thread.
      DEB_TRACE() << "[siso_me4 acquisition thread] Locking to test if we were asked to stop";
      the_lock.lock();
      the_acq_goon = !m_grabber.m_acq_thread_waiting && !m_grabber.m_acq_thread_should_quit;
      m_grabber.m_acq_thread_running = the_acq_goon;
      DEB_TRACE() << "[siso_me4 acquisition thread] Should we continue at the end of this iteration : " << m_grabber.m_acq_thread_running << " AKA " << the_acq_goon;
      m_grabber.m_cond.broadcast();
      DEB_TRACE() << "[siso_me4 acquisition thread] Just broadcasted for other threads to know that it might be interesting to change state";
      the_lock.unlock();
      DEB_TRACE() << "[siso_me4 acquisition thread] Just Unlocked after state potential modification";

      DEB_TRACE() << "[siso_me4 acquisition thread] the_new_frame : " << the_new_frame;
      if ( 0 < the_new_frame ) { // We indeed got a frame back
        //  m_grabber.setStatus(Grabber::Running, false);
        // We managed to get an image buffer returned :
        HwFrameInfoType		the_frame_info;
        bool              the_frame_read;
        
        the_frame_info.acq_frame_nb = static_cast<int>(m_grabber.m_image_index);
        the_frame_read = the_buffer.newFrameReady(the_frame_info);
	if ( 0 == (m_grabber.m_image_index % 50) ) {
	  DEB_ALWAYS() << "[siso_me4 acquisition thread] image " << m_grabber.m_image_index <<" published with newFrameReady(), with result " << the_frame_read ;
	}
        the_acq_goon = the_acq_goon && the_frame_read;
        
        ++m_grabber.m_image_index;
        
//        if ( m_cam.m_buffer_ringing ) {
//          DEB_TRACE() << "[siso_me4 acquisition thread] As we are using a ring-buffer : re-queueing the acquired image on the buffer queue.";
//          AT_QueueBuffer(m_cam.m_camera_handle, the_returned_image, the_returned_image_size);
//          DEB_TRACE() << "[siso_me4 acquisition thread] There is NO guarantee that LIMA will be done with this buffer BEFORE andor SDK3 is changing its content !!!";
//        }
// In all cases, the buffer will be ringed back in, unless we leave it blocked…
        if ( m_grabber.m_buffer_ringing ) {
          // Deblocking the buffer right away, so that it can be used again for another image :
          Fg_setStatusEx(m_grabber.m_fg, FG_UNBLOCK, the_new_frame, m_grabber.m_dma_index, m_grabber.m_next_dma_head);
          frameindex_t		the_num_buf_blocked;
          frameindex_t		the_num_lost_frames;
          the_num_buf_blocked = Fg_getStatusEx(m_grabber.m_fg, NUMBER_OF_BLOCKED_IMAGES, 0, m_grabber.m_dma_index, m_grabber.m_next_dma_head);
          the_num_lost_frames = Fg_getStatusEx(m_grabber.m_fg, NUMBER_OF_LOST_IMAGES, 0, m_grabber.m_dma_index, m_grabber.m_next_dma_head);
          DEB_TRACE() << "[siso_me4 acquisition thread] Ringing : Number of blocked frames is " << the_num_buf_blocked;
          if ( 0 != the_num_lost_frames ) {
            DEB_WARNING() << "[siso_me4 acquisition thread] Ringing : Already lost (completely) " << the_num_lost_frames << "images !!!";
          }
        }
      }
      else if ( FG_TIMEOUT_ERR == the_new_frame ) {
        DEB_WARNING() << "A timeout occured while waiting for a frame, in the current settings, usage and camera it should not happen. Most likely something is going wrong... But still we will try again to get the frame.";
      }
      else {
        DEB_ERROR() << "[siso_me4 acquisition thread] Problem in retrieving the frame indexed " << m_grabber.m_image_index <<"!\n"
        << "\tFg_getImageEx returned an error " << the_new_frame << "\n"
        << "\t" << Fg_getErrorDescription(m_grabber.m_fg, static_cast<int>(the_new_frame)) << "\n"
        << "\t!!! returning to WAIT mode !!!";
        m_grabber.m_acq_thread_running = the_acq_goon = false;
        m_grabber.setStatus(Fault, false);
        break;
      }
      DEB_TRACE() << "[siso_me4 acquisition thread] End of the iteration, next iteration will be for image index " << m_grabber.m_image_index;
      
      if ( ! the_acq_goon ) {
        DEB_TRACE() << "[siso_me4 acquisition thread] In the middle of acquisition, got the request to stop the frame retrieving activity : m_acq_thread_waiting is " << m_grabber.m_acq_thread_waiting << " and m_acq_thread_should_quit is " << m_grabber.m_acq_thread_should_quit;
      }
      
    }
    
    // Once we arrived here, we have to purge the memory... Hoping that the last image(s) will not be lost (if not correctly copied).
    DEB_TRACE() << "[siso_me4 acquisition thread] Finished looping !";
    Fg_stopAcquireEx(m_grabber.m_fg, m_grabber.m_dma_index, m_grabber.m_next_dma_head, STOP_SYNC);
    DEB_TRACE() << "[siso_me4 acquisition thread] Sent stop to the frame grabber";

    Fg_FreeMemHead(m_grabber.m_fg, m_grabber.m_next_dma_head);
    m_grabber.m_next_dma_head = NULL;
    
//    StdBufferCbMgr& the_buffer = m_grabber.m_buffer_ctrl_obj.getBuffer();
//    DEB_TRACE() << "Getting StdBufferCbMgr to allocate the buffers that we want to have";
    // Indeed it's better NOT to release as long as not needed (so that we don't have memeory troubles in the use of the frames).
    // So we are commenting this one out, and leave the "release" to take place "transparently" in the prepareAcq since the
    // frame allocation make sure that first the previously used memeory is released.
    //    the_buffer.releaseBuffers();

    m_grabber.setStatus(Ready, false);
    // Returning to the waiting mode :
    the_lock.lock();
    m_grabber.m_acq_thread_waiting = true;
  }
}

