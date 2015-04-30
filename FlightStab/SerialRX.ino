/* FlightStab **************************************************************************************************/

/***************************************************************************************************************
*
* SERIAL RX
* see http://paparazzi.github.io/docs/latest/stm32_2subsystems_2radio__control_2spektrum__arch_8c_source.html
*
***************************************************************************************************************/

#if defined(SERIALRX_SBUS) && defined(SBUS_OUT)
  struct sbusFrame_s {
    uint8_t syncByte;
    // 176 bits of data (11 bits per channel * 16 channels) = 22 bytes.
    unsigned int chan0 : 11;
    unsigned int chan1 : 11;
    unsigned int chan2 : 11;
    unsigned int chan3 : 11;
    unsigned int chan4 : 11;
    unsigned int chan5 : 11;
    unsigned int chan6 : 11;
    unsigned int chan7 : 11;
    unsigned int chan8 : 11;
    unsigned int chan9 : 11;
    unsigned int chan10 : 11;
    unsigned int chan11 : 11;
    unsigned int chan12 : 11;
    unsigned int chan13 : 11;
    unsigned int chan14 : 11;
    unsigned int chan15 : 11;
    uint8_t flags;
    uint8_t endByte;
  } __attribute__ ((__packed__));
#endif  

#if defined(SERIALRX_SPEKTRUM)
#define SERIAL_FRAME_SIZE	16
//jrb - added 20140831 Add define for Quiet Time
#define SERIAL_WAIT_TIME	7000
int8_t rshift;
#endif

#if defined(SERIALRX_SBUS)
// The following value is added to the received pulse count
// to make the center pulse width = 1500 when the TX output is 1500
// TODO(noobee): i guess we would need to make this configurable..
#define SBUS_DECODE_OFFSET 1005 // 1009 for Futaba R6208SB, 1005 for Taranis FRSKY X8R, 984 for Orange R800x
#define SBUS_ENCODE_OFFSET 1408 // Used to convert internal OFS pulse width to 625ns per bit S.BUS value

//jrb - added 20140829 Use defines for SBUS Constants instead of hard coded numbers
// SBUS unique defines
#define SBUS_SYNC_BYTE 		0x0F
#define SBUS_END_BYTE		0x00
#define SERIAL_FRAME_SIZE	25
#define SERIAL_WAIT_TIME	5000
#endif
      
#if defined(SERIALRX_SRXL)
#include <util/crc16.h>
#define SERIAL_FRAME_SIZE	37		// changed to 37 for 16ch HoTT SUMD		      
#define MPX_OFFSET 		988
#define SERIAL_WAIT_TIME	5000  
#define MPX_SRL_12_CHAN_ID	0xA1
#define MPX_SRL_12_CHAN_SIZE	27
#define MPX_SRL_16_CHAN_ID	0xA2
#define MPX_SRL_16_CHAN_SIZE	35
#define HOTT_SUMD_ID1		0xA8
#define HOTT_SUMD_ID2		0x01	  
#endif
	  
#if (defined(SERIALRX_SPEKTRUM) || defined(SERIALRX_SBUS) || defined(SERIALRX_SRXL))
  #if defined(NANOWII)
    HardwareSerial *pSerial = &Serial1; // TODO: hardcoded for NanoWii for now
  #else
    HardwareSerial *pSerial = &Serial; 
  #endif // NANOWI
  
  //jrb add for debug  
  #if defined(SERIALRX_DEBUG) && 1
    volatile int8_t RXcount;
    int8_t PrintIndex;
    uint16_t work;
  #endif  


  void serialrx_init()
  {
    #if defined (SERIALRX_SPEKTRUM)
      pSerial->begin(115200L);
      rshift = cfg.serialrx_spektrum_levels == SERIALRX_SPEKTRUM_LEVELS_1024 ? 10 : 11; // 1024->10, 2048->11
    #endif
    #if defined (SERIALRX_SBUS)
      pSerial->begin(100000L, SERIAL_8E2);
      #if defined (FLIP_SERIALRX_DEBUG)
        pinMode(4,OUTPUT);
        pinMode(5,OUTPUT);
        pinMode(6,OUTPUT);
        digitalWrite(4,HIGH);		// D4 provides negative pulse that windows the serial receive frame
        digitalWrite(5,HIGH);		// D5 provides negative pulse when last byte of SBUS frame != 0x00
        digitalWrite(5,HIGH);		// D6 provides negative pulse when serialrx_update function is active
      #endif
    #endif
	#if defined(SERIALRX_SRXL)
	  pSerial->begin(115200L);
	#endif
  }
  
  // Make these variables global
  //jrb 20140831 use for all Serial Protocols  
  static uint32_t last_rx_time;
  uint32_t t;
  static uint8_t buf[SERIAL_FRAME_SIZE];
  #if defined(SBUS_OUT)
    static union {
      uint8_t buf_out[SERIAL_FRAME_SIZE]; 
      sbusFrame_s sbus_out;
    } out_union;
  #endif // SBUS_OUT
     

  bool serialrx_update()
  {
    static int8_t index = 0;
    #if defined (SERIALRX_SBUS)
      bool sbus_return = false;
    #endif // SERIALRX_SBUS  
    
  #if defined(SERIALRX_SRXL)
    bool sbus_return = false;
  #endif

  //jrb add for debug
  #if defined(SERIALRX_DEBUG) && 0
    Serial.println("Serial update");
  #endif
  
  #if defined (FLIP_SERIALRX_DEBUG)
    digitalWrite(5,HIGH);
    digitalWrite(6,LOW);
  #endif 
  
  //jrb - added 20140829 Limit "while loop" to not exceed buffer size   
    while (pSerial->available() && index<SERIAL_FRAME_SIZE) {
    uint8_t ch = pSerial->read();

//jrb - 20140832 make frame time test common to all Serial Protocols
    t = micros1();
    // we assume loop() calls to serialrx_update() in << SERIAL_WAIT_TIME intervals
    if ((int32_t)(t - last_rx_time) > SERIAL_WAIT_TIME) {
      index = 0; // found pause before new frame, resync
    #if defined (FLIP_SERIALRX_DEBUG)
      digitalWrite(4,HIGH);
    #endif      
    }
    last_rx_time = t;
      
  #if defined (SERIALRX_SPEKTRUM)    
    #warning SERIALRX_SPEKTRUM defined // emit device name 
  //jrb add for debug    
      #if defined(SERIALRX_DEBUG) && 0
        Serial.println("Serial Spektrum");
      #endif

      buf[index++] = ch;
      if (index >= SERIAL_FRAME_SIZE) {

/*jrb        
//  Satellites alone never return frame size information in buf[1] the best I can tell
//  Data always seems to be 2048 bits, even with 9XR transmitter.  The following
//  should be removed and 10 or 11 bit format saved as configuration data
//          if ((buf[2] & 0x80) == 0x00) { 
//          // single frame type or 1st frame of two, contains "transmitter type"
//            rshift = (buf[1] & 0x10) ? 11 : 10; // 11 or 10 bit data
//          }
jrb*/ 
 
        if (rshift > 0) {
          // 10 bit == f  0 c3 c2 c1  c0 d9 d8 d7 d6 d5 d4 d3 d2 d1 d0
          // 11 bit == f c3 c2 c1 c0 d10 d9 d8 d7 d6 d5 d4 d3 d2 d1 d0        
          for (int8_t i=2; i<2+2*7; i+=2) {
            uint16_t w = ((uint16_t)buf[i] << 8) | (uint16_t)buf[i+1];
            int8_t chan = (w >> rshift) & 0xf;
            if (chan < rx_chan_size) {
              *rx_chan[chan] = ((w << (11 - rshift) & 0x7ff) - 1024 + 1500); // scale to 11 bits 1024 +/- 684;
            }          
          }
        }  
        index = 0;

  //jrb add for debug       
  #if defined(SERIALRX_DEBUG) && 0   
      if (RXcount > 100)
      {
        Serial.print("rshift = ");
        Serial.print(rshift);
        if ((buf[2] & 0x80) == 0x00) { 
          Serial.println(" Low Channels");
        }
        else  {
          Serial.println(" High Channels");  
        }
        
        for (PrintIndex = 0; PrintIndex < 8; PrintIndex++)
        {
          Serial.print(*rx_chan[cfg.serialrx_order-2][PrintIndex]); Serial.print(' ');
        }
        Serial.println(' '); 
              
        for (PrintIndex = 0; PrintIndex < 16; PrintIndex += 2)
        { 
          work = (buf[PrintIndex]<<8)+ buf[PrintIndex + 1];
          Serial.print(work,HEX); Serial.print(' ');
        }
        Serial.println(' '); 
       RXcount = 0; 
      }
      RXcount++; 
  #endif
      return (true);
    }
    else
      return (false);
  }  
  #endif // SERIALRX_SPEKTRUM

  #if defined(SERIALRX_SRXL) // ----------------------------------------------
    #warning SERIALRX_SRXL defined // emit device name 
	  
      if (index < SERIAL_FRAME_SIZE)	//JRB Should this be just < so we don't overflow buffer????? YES!!!
        buf[index++] = ch;
      // MPX SRXL 12/16 channels	  
      if ( ((buf[0] == MPX_SRL_12_CHAN_ID) && (index >= MPX_SRL_12_CHAN_SIZE)) 
      	 || ((buf[0] == MPX_SRL_16_CHAN_ID) && (index >= MPX_SRL_16_CHAN_SIZE)) )
      {
		uint16_t crc = 0;
		for (uint8_t i=0; i<index; i++){
			uint8_t data;
			data = buf[i];
			crc = _crc_xmodem_update(crc, data);
		}
		if (crc == 0){
			volatile int16_t **p = rx_chan;
			// Only process first 8 channels
			for (int8_t i=2; i<18; i+=2) 
			{
			  uint16_t w = (((uint16_t)buf[i-1] << 8) | (uint16_t)buf[i] ) & 0xfff;
			  *p[(i>>1)-1] = ((((w << 1) + w) - (w >> 2)) >> 3) + 796;
			}
			sbus_return = true;
		}
        index = 0;
        
        #if defined(SERIAL_DEBUG) && 2
	      //Serial.println("Serial SRXL");
	      #endif
      }
      // HoTT SUMD
      else if ( ((buf[0] == HOTT_SUMD_ID1) && ((buf[1] & 0x7f) == HOTT_SUMD_ID2) ) )
      {
        uint8_t n_channels = buf[2];
        if (index >= ((n_channels << 1) +5))
        {
			uint16_t crc = 0;
			for (uint8_t i=0; i<(index-2); i++){
				uint8_t data;
				data = buf[i];
				crc = _crc_xmodem_update(crc, data);
			}
			if (crc == ((buf[index-2] << 8) + buf[index-1])){
			   volatile int16_t **p = rx_chan;
			   // Only process first 8 channels
			   for (int8_t i=2; i<18; i+=2) 
			   {
				 uint16_t w = (((uint16_t)buf[i+1] << 8) | (uint16_t)buf[i+2] );
				 *p[(i>>1)-1] = (w >> 3);
			   }
				sbus_return = true;
			}
			index = 0;
        #if defined(SERIAL_DEBUG) && 2
	      //Serial.println("Serial SUMD");
	      #endif
        } 
      }
    //his add for debug  
    #if defined(SERIAL_DEBUG) && 2   
      if (RXcount > 10){
        for (index = 0; index < 8; index++){
          Serial.print(*rx_chan[index]); Serial.print(' ');
        }
        Serial.println(' '); 
        RXcount = 0; 
      }
      if (sbus_return)
        RXcount++; 
    #endif
  }  
  return (sbus_return); 
  #endif // SERIALRX_SRXL -------------------------------------------------------


  #if defined (SERIALRX_SBUS)
    #warning SERIALRX_SBUS defined // emit device name 
    //jrb add for debug    
      #if defined(SERIALRX_DEBUG) && 0
        Serial.println("Serial S.BUS");
      #endif
      
      if (index == 0 && ch != SBUS_SYNC_BYTE) { 
        break;
      }
      #if defined (FLIP_SERIALRX_DEBUG)
        if (index == 0)
        {	
          digitalWrite(4,LOW);
        }  
      #endif
      buf[index++] = ch;
      if (index >= SERIAL_FRAME_SIZE) {
        volatile int16_t **p = rx_chan;
        uint8_t adj_index;
        //jrb - added 20140829 to detect and discard invalid frames
        if (buf[24] != SBUS_END_BYTE) { // There is a sync problem - discard this frame and try again
          index = 0;
          sbus_return = false;
      #if defined (FLIP_SERIALRX_DEBUG)
        digitalWrite(5,LOW);
      #endif          
  	} 	
         	
        // Only process first 8 channels
        *p[0] = (((((uint16_t)buf[1]  >> 0) | ((uint16_t)buf[2]  << 8)) & 0x7ff) >> 1) + SBUS_DECODE_OFFSET;
        *p[1] = (((((uint16_t)buf[2]  >> 3) | ((uint16_t)buf[3]  << 5)) & 0x7ff) >> 1) + SBUS_DECODE_OFFSET; 
        *p[2] = (((((uint16_t)buf[3]  >> 6) | ((uint16_t)buf[4]  << 2) | ((uint16_t)buf[5] << 10)) & 0x7ff) >> 1) + SBUS_DECODE_OFFSET; 
        *p[3] = (((((uint16_t)buf[5]  >> 1) | ((uint16_t)buf[6]  << 7)) & 0x7ff) >> 1) + SBUS_DECODE_OFFSET; 
        *p[4] = (((((uint16_t)buf[6]  >> 4) | ((uint16_t)buf[7]  << 4)) & 0x7ff) >> 1) + SBUS_DECODE_OFFSET; 
        *p[5] = (((((uint16_t)buf[7]  >> 7) | ((uint16_t)buf[8]  << 1) | ((uint16_t)buf[9] << 9)) & 0x7ff) >> 1) + SBUS_DECODE_OFFSET;
        *p[6] = (((((uint16_t)buf[9]  >> 2) | ((uint16_t)buf[10] << 6)) & 0x7ff) >> 1) + SBUS_DECODE_OFFSET; 
        *p[7] = (((((uint16_t)buf[10] >> 5) | ((uint16_t)buf[11] << 3)) & 0x7ff) >> 1) + SBUS_DECODE_OFFSET;
      
              
       
       // For some reason the SBUS pulse width values represent 625ns per bit instead of the 500ns per bit
       // normally associated with 2048 bit resoloution.
       // Adjust the actual value by +/-25%.  Sign determined by pulse width above or below center of 1520us
       // There are many ways to achieve this, the following method was selected as it required no other
       // to the existing code. 
       for(adj_index=0; adj_index<rx_chan_size; adj_index++)
       {
        if (*p[adj_index] < RX_WIDTH_MID)
       	  *p[adj_index] -= (RX_WIDTH_MID - *p[adj_index]) >> 2;		
       	else	
       	  *p[adj_index] += (*p[adj_index] - RX_WIDTH_MID) >> 2;
       }
       	 
      #if defined(SBUS_OUT)
        uint8_t j;
        // Copy input buffer to output buffer before stabilization
        for (j=0; j<SERIAL_FRAME_SIZE; j++)
          out_union.buf_out[j] = buf[j];
      #endif

 	index = 0;
        sbus_return = true; 
        
      #if defined (FLIP_SERIALRX_DEBUG)
        digitalWrite(4,HIGH);
      #endif
      }

  //jrb add for debug  
    #if defined(SERIALRX_DEBUG) && 1   
      if (RXcount > 100)
      {
        for (index = 0; index < 8; index++)
        {
          Serial.print(*rx_chan[cfg.serialrx_order-2][index]); Serial.print(' ');
        }
        Serial.println(' '); 
       RXcount = 0; 
      }
      RXcount++; 
    #endif
  }
    #if defined (FLIP_SERIALRX_DEBUG)
      digitalWrite(6,HIGH);
    #endif 
    
  return (sbus_return); 
  #endif // SERIALRX_SBUS
}

  #if defined (SBUS_OUT)
void send_sbus()
{
  // Send OFS processed channels to S.BUS Output Frame
  // The internal S.BUS values (1us per bit) must be converted to S.BUS values of 625ns per bit
  // Output values must be constrained to 2047 (0x7FF) as S.BUS only uses 11 bits per channel
  if (tx_chan_map[0] != NULL) out_union.sbus_out.chan0 = constrain((*tx_chan_map[0] * 1.6) - SBUS_ENCODE_OFFSET,0,2047);		
  if (tx_chan_map[1] != NULL) out_union.sbus_out.chan1 = constrain((*tx_chan_map[1] * 1.6) - SBUS_ENCODE_OFFSET,0,2047);
  if (tx_chan_map[2] != NULL) out_union.sbus_out.chan2 = constrain((*tx_chan_map[2] * 1.6) - SBUS_ENCODE_OFFSET,0,2047);
  if (tx_chan_map[3] != NULL) out_union.sbus_out.chan3 = constrain((*tx_chan_map[3] * 1.6) - SBUS_ENCODE_OFFSET,0,2047);
  if (tx_chan_map[4] != NULL) out_union.sbus_out.chan4 = constrain((*tx_chan_map[4] * 1.6) - SBUS_ENCODE_OFFSET,0,2047);
  if (tx_chan_map[5] != NULL) out_union.sbus_out.chan5 = constrain((*tx_chan_map[5] * 1.6) - SBUS_ENCODE_OFFSET,0,2047);
  if (tx_chan_map[6] != NULL) out_union.sbus_out.chan6 = constrain((*tx_chan_map[6] * 1.6) - SBUS_ENCODE_OFFSET,0,2047);
  if (tx_chan_map[7] != NULL) out_union.sbus_out.chan7 = constrain((*tx_chan_map[7] * 1.6) - SBUS_ENCODE_OFFSET,0,2047);  	
  Serial.write(out_union.buf_out, SERIAL_FRAME_SIZE); 	
}	
  #endif // SBUS_OUT
#endif // SERIALRX_SPEKTRUM) || SERIALRX_SBUS 

