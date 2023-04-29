/*  CAN-FIX Interface library for the Arduino
 *  Copyright (c) 2013 Phil Birkelbach
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#ifndef __CANFIX_H
#define __CANFIX_H

#include <Arduino.h>

//Bitrate definitions for the can_init() function
#define BITRATE_125  0
#define BITRATE_250  1
#define BITRATE_500  2
#define BITRATE_1000 3

// Node Specific Message Control Codes
#define NSM_START    0x6E0
#define NSM_ID       0
#define NSM_BITRATE  1
#define NSM_NODE_SET 2
#define NSM_DISABLE  3 // Disable Parameter
#define NSM_ENABLE   4 // Enable Parameter
#define NSM_REPORT   5
#define NSM_STATUS   6
#define NSM_FIRMWARE 7
#define NSM_TWOWAY   8
#define NSM_CONFSET  9  // Configuration Set
#define NSM_CONFGET  10 // Configuration Query
#define NSM_DESC     11 // Node description
#define NSM_PSET     12 //12 - 19 are the parameter set codes

#define MODE_BLOCK    0
#define MODE_NONBLOCK 1

#define FCB_ANNUNC    0x01;
#define FCB_QUALITY   0x02;
#define FCB_FAIL      0x04;

typedef struct {
  unsigned int id;
  byte length;
  byte data[8];
} CanFixFrame;


class CFParameter {
    public:
        word type;
        byte node;
        byte index;
        byte fcb;
        byte data[5];
        byte length;
        void setMetaData(byte meta);
        byte getMetaData(void);
        void setFlags(byte flags);
        byte getFlags(void);
};

class CanFix {
    private:
        byte deviceid, nodeid, fw_version;  //Node identification information
        unsigned long model;  //Model number for node identification
        byte writeFrame(CanFixFrame frame);
        void parameterEnable(CanFixFrame frame);
        void handleNodeSpecific(CanFixFrame frame);
        void handleFrame(CanFixFrame frame);

        //Function Pointers for Callbacks
        void (*__write_callback)(CanFixFrame);
        void (*__report_callback)(void);
        byte (*__twoway_callback)(byte, word);
        byte (*__config_callback)(word, byte *);
        byte (*__query_callback)(word, byte *, byte *);
        void (*__param_callback)(CFParameter);
        void (*__alarm_callback)(byte, word, byte*, byte);
        void (*__stream_callback)(byte, byte *, byte);
        void (*__bitrate_set_callback)(byte);
        void (*__node_set_callback)(byte);
        byte (*__parameter_enable_callback)(unsigned int, byte);
        byte (*__parameter_query_callback)(unsigned int);

    public:
        CanFix(byte device);
        // Communication and Event dispatcher
        void exec(CanFixFrame frame);
        // Property Functions
        void setNodeNumber(byte id);
        void setDeviceId(byte id);
        void setModel(unsigned long m);
        void setFwVersion(byte v);

        // Data Transfer Functions
        void sendStatus(word type, byte *data, byte length);
        void sendParam(CFParameter p);
        void sendAlarm(word type, byte *data, byte length);
        void sendStream(byte channel, byte *data, byte length);

        // Callback function assignments
        void set_write_callback(void (*f)(CanFixFrame));
        void set_report_callback(void (*f)(void));
        void set_twoway_callback(byte (*f)(byte, word));
        void set_config_callback(byte (*f)(word, byte *));
        void set_query_callback(byte (*f)(word, byte *, byte *));
        void set_param_callback(void (*f)(CFParameter));
        void set_alarm_callback(void (*f)(byte, word, byte*, byte));
        void set_stream_callback(void (*f)(byte, byte *, byte));
        void set_bitrate_callback(void (*f)(byte));
        void set_node_callback(void (*f)(byte));
        void set_parameter_enable_callback(byte (*f)(word, byte));
        void set_parameter_query_callback(byte (*f)(word));
};

#endif  //__CANFIX_H
