#ifndef SISOME4SERIALLINE_H
#define SISOME4SERIALLINE_H

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

#if defined (__GNUC__) && (__GNUC__ == 3) && defined (__ELF__)
#   define GENAPI_DECL __attribute__((visibility("default")))
#   define GENAPI_DECL_ABSTRACT __attribute__((visibility("default")))
#endif

// System headers :
#include <stdlib.h>
#include <limits>
#include <ostream>

// Some lima stuff :
#include "HwSerialLine.h"

// Camera SDK headers :
#include <clser.h> // The serial lines ot the camera

namespace lima
{
  namespace siso_me4
  {
    
    /*******************************************************************
     * \class SerialLine
     * \brief object to use the Silicon Software MicroEnable IV CameraLink serial line
     *
     * This class is responsible ot expose the CameraLink integrated serial line
     * and tapping on the frame-grabber SDK part that is normalised by camera link
     * standard.
     *******************************************************************/
    class SerialLine : lima::HwSerialLine {
      DEB_CLASS_NAMESPC(DebModCamera, "SerialLine", "siso-me4");

      enum ME4_baud_rate {
        BR9600   = CL_BAUDRATE_9600,
        BR19200  = CL_BAUDRATE_19200,
        BR38400  = CL_BAUDRATE_38400,
        BR57600  = CL_BAUDRATE_57600,
        BR115200 = CL_BAUDRATE_115200,
        BR230400 = CL_BAUDRATE_230400,
        BR460800 = CL_BAUDRATE_460800,
        BR921600 = CL_BAUDRATE_921600
      };
      
      enum ME4_parity {
        Off = 0,
        On  = 1
      };
      
      SerialLine(unsigned int i_serial_index, ME4_baud_rate i_baud_rate, ME4_parity i_parity,
                 const std::string& i_line_term="\r", double i_timeout=1.0,
                 int i_block_size=0, double i_block_delay=0);
      
      virtual ~SerialLine();
      
      //! Though already implemented in super-class, I prefer to overload-it here.
      virtual void flush();
      
      virtual void read(std::string& o_buffer, int max_len,
                        double timeout=TimeoutDefault);
      
      virtual void write(const std::string& i_buffer, bool no_wait=false);

      virtual void getNbAvailBytes(int &avail);

/* These one should be provided by the super-class, based on read and write :
      virtual void writeRead( const std::string& writebuffer,
                             std::string& readbuffer, int max_len,
                             bool wr_no_wait=false,
                             double rd_timeout=TimeoutDefault );
      
      virtual void writeReadStr( const std::string& writebuffer,
                                std::string& readbuffer,
                                int max_len, const std::string& term,
                                bool wr_no_wait=false,
                                double rd_timeout=TimeoutDefault );
      
      virtual void readAvailable( std::string& buffer, int max_len );
      
      
      virtual void setLineTerm( const std::string& line_term );
      virtual void getLineTerm( std::string& line_term ) const;
      
      virtual void setTimeout( double timeout );
      virtual void getTimeout( double& timeout ) const;
      
      virtual void setBlockSize( int block_size );
      virtual void getBlockSize( int& block_size ) const;
      
      virtual void setBlockDelay( double block_delay );
      virtual void getBlockDelay( double& block_delay ) const;
      
      double checkDefTimeout( double timeout );
 */
      
      virtual void setBaudRate(ME4_baud_rate i_baud_rate);
      virtual void getBaudRate(ME4_baud_rate &o_baud_rate) const;
      virtual void setParity(ME4_parity i_parity);
      virtual void getParity(ME4_parity &o_parity) const;
    protected:
      virtual int checkError(int i_err_code) const;
      virtual void init();
      
    private:
      void						*m_serial_ref;
      unsigned int		m_serial_port_index;
      ME4_baud_rate		m_baud_rate;
      ME4_parity			m_parity;
      mutable int           m_err_code;
      mutable std::string   m_err_string;
    };
  }
}

#endif /* defined(SISOME4SERIALLINE_H) */
