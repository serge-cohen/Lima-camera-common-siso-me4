/* siso-me4 plugin serial-line to camera class
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

// Camera SDK headers :

// LImA headers :

// SiSo-ME4 plugin headers :
#include "SiSoME4SerialLine.h"

lima::siso_me4::SerialLine::SerialLine(
    unsigned int i_serial_index,
    ME4_baud_rate i_baud_rate,
    ME4_parity i_parity,
    const std::string& i_line_term,
    double i_timeout,
    int i_block_size,
    double i_block_delay) :
HwSerialLine(i_line_term, i_timeout, i_block_size, i_block_delay),
m_serial_ref(NULL),
m_serial_port_index(i_serial_index),
m_baud_rate(i_baud_rate),
m_parity(i_parity)
{
	DEB_CONSTRUCTOR();
	DEB_PARAM() << DEB_VAR1(i_serial_index);

  init();

//	ostringstream os;
//	os << "Serial#" << edev.getDevNb();
//	DEB_SET_OBJ_NAME(os.str());
}

lima::siso_me4::SerialLine::~SerialLine()
{
  DEB_DESTRUCTOR();
  if ( m_serial_ref ) {
    clSerialClose(m_serial_ref);
    m_serial_ref = NULL;
  }
  else {
    DEB_WARNING() << "About to destroy a serial line that was not even inited";
  }
}

//! Though already implemented in super-class, I prefer to overload-it here.
void
lima::siso_me4::SerialLine::flush()
{
  DEB_MEMBER_FUNCT();
  if ( m_serial_ref ) {
    clFlushPort(m_serial_ref);
  }
  else {
    DEB_WARNING() << "You tried to flush a not inited serial link... Doing nothing.";
  }
}

void
lima::siso_me4::SerialLine::read(std::string& o_buffer, int i_max_len, double i_timeout)
{
  DEB_MEMBER_FUNCT();
  DEB_PARAM() << DEB_VAR2(i_max_len, i_timeout);

  // Computing the timeout in ms :
  unsigned int		the_timeout; // Timeout in ms (integer)
  switch (static_cast<TimeoutConst>(i_timeout)) {
    case TimeoutDefault:
      getTimeout(i_timeout);
      the_timeout = static_cast<unsigned int>(1000.0 * i_timeout);
      break;
      
    case TimeoutBlockForever:
      the_timeout = UINT_MAX;
      break;

    case TimeoutNoBlock:
      the_timeout = 0;
      break;
      
    default:
      the_timeout = static_cast<unsigned int>(1000.0 * i_timeout);
      break;
  }

  // Making sure that we can really handle the data requested :
  char		*the_buf = new char[i_max_len];
  // Performing the read itself :
  unsigned int		the_read_size = i_max_len;
  clSerialRead(m_serial_ref, the_buf, &the_read_size, the_timeout);
  // Copying the data back to the o_buffer :
  if ( the_read_size ) {
    o_buffer = std::string(the_buf, the_read_size);
  }
  else {
    DEB_TRACE() << "No data read back... But maybe this is somehow expected ?";
    o_buffer.resize(0);
  }
  delete[] the_buf;
}

void
lima::siso_me4::SerialLine::write(const std::string& i_buffer, bool no_wait)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR2(i_buffer, no_wait);
  if ( ! m_serial_ref ) {
    DEB_WARNING() << "Tried to write to a non-inited serial line !!! Performing NOT`HING.";
    return;
  }
  if ( no_wait ) { // There is no easy way to implement the «no_wait=true» version, so just
                   // ignore it and log to user if ever we were requested to «no_wait»
    DEB_WARNING() << "You requested to write with no_wait==true, while this is not possible on this frame-grabber... Just ignoring the value of no_wait";
    
  }
  double        the_dtimeout;  // The timeout in s (double)
  getTimeout(the_dtimeout);
  
  unsigned int the_timeout = static_cast<unsigned int>(1000.0 * the_dtimeout); // Timeout in ms (integer)
  unsigned int the_length = static_cast<unsigned int>(i_buffer.length());
  unsigned int the_done=0, the_next=the_length;
  int						the_err_code;
  while ( 0 != the_next ) {
    if ( (the_err_code=clSerialWrite(m_serial_ref, const_cast<char *>(i_buffer.data())+the_done, &the_next, the_timeout)) && (CL_ERR_TIMEOUT != the_err_code) ) {
      DEB_WARNING() << "Got an error (" << the_err_code << ") while trying to write to the serial line :\n\t"
      << m_err_string << "\n\tStopping to write to the serial line";
      break;
    }
    the_done += the_next;
    the_next = the_length - the_done;
  }
  if ( the_next ) {
    DEB_WARNING() << "Be AWARE that not all the data was send through the serial line, most likely due to some error in the transmission";
  }
  return;
}


void
lima::siso_me4::SerialLine::getNbAvailBytes(int &avail)
{
  DEB_MEMBER_FUNCT();
  if ( m_serial_ref ) {
    unsigned int		the_num_bytes;
    clGetNumBytesAvail(m_serial_ref, &the_num_bytes);
    avail = static_cast<int>(the_num_bytes);
  }
  else {
    DEB_WARNING() << "You reqeusted the number of bytes available from a serial link that is not inited. Returning 0";
    avail = 0;
  }
}


void
lima::siso_me4::SerialLine::setBaudRate(ME4_baud_rate i_baud_rate)
{
  DEB_MEMBER_FUNCT();
  if ( ! m_serial_ref ) {
    DEB_WARNING() << "Requested to set the baud-rate of a non inited serial link ! Will only affet the next init";
    m_baud_rate = i_baud_rate;
    return;
  }
  // Setting the transfer rate/bauds
  if ( checkError(clSetBaudRate(m_serial_ref, i_baud_rate)) ) {
    DEB_WARNING() << "Unable to set the baud-rate of serial line to the camera. The error message from the frame-grabber is : " << m_err_code << "\n'"
    << m_err_string << "'";
  }
  else {
    // Recording (in cache) the setting
    m_baud_rate = i_baud_rate;
  }
}

void
lima::siso_me4::SerialLine::getBaudRate(ME4_baud_rate &o_baud_rate) const
{
  o_baud_rate = m_baud_rate;
  return;
}

void
lima::siso_me4::SerialLine::setParity(ME4_parity i_parity)
{
  DEB_MEMBER_FUNCT();
  if ( ! m_serial_ref ) {
    DEB_WARNING() << "Requested to set the parity of a non inited serial link ! Will only affet the next init";
    m_parity = i_parity;
    return;
  }
  // Setting the parity
  if ( checkError(clSetParity(m_serial_ref, i_parity)) ) {
    DEB_WARNING() << "Unable to set the parity of serial line to the camera. The error message from the frame-grabber is : " << m_err_code << "\n'"
    << m_err_string << "'";
  }
  else {
    // Recording (in cache) the setting
    m_parity = i_parity;
  }
}

void
lima::siso_me4::SerialLine::getParity(ME4_parity &o_parity) const
{
  o_parity = m_parity;
  return;
}


int
lima::siso_me4::SerialLine::checkError(int i_err_code) const
{
  DEB_MEMBER_FUNCT();
  char					the_error_text[1024];
  unsigned int	the_text_size=1024;
  
  m_err_code = i_err_code;
  clGetErrorText(i_err_code, the_error_text, &the_text_size);
  m_err_string = std::string(the_error_text);
  return i_err_code;
}


void
lima::siso_me4::SerialLine::init()
{
  DEB_MEMBER_FUNCT();
  
  if ( NULL != m_serial_ref ) {
    clSerialClose(m_serial_ref);
    m_serial_ref = NULL;
  }
  
  unsigned int   the_num_ports;
  clGetNumSerialPorts(&the_num_ports);
  if ( the_num_ports <= m_serial_port_index ) {
    DEB_WARNING() << "You asked for serial port index " << m_serial_port_index << ", while there is only " << the_num_ports << " serial ports available (index starting at 0)";
  }

  // Opening the link
  if ( checkError(clSerialInit(m_serial_port_index, &m_serial_ref)) ) {
    DEB_WARNING() << "Unable to instanciate the serial line to the camera. The error message from the frame-grabber is : " << m_err_code << "\n'"
    << m_err_string << "'";
    if ( m_serial_ref ) {
      clSerialClose(m_serial_ref);
      m_serial_ref = NULL;
    }
    return;
  }
  
  setBaudRate(m_baud_rate);
  setParity(m_parity);
  // Done, we can «happily» return.
  return;
}
