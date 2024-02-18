#ifndef _GROVE_GPS_H_
#define _GROVE_GPS_H_

#include "mbed.h"
#include <stdlib.h>
#include <string>

class GroveGPS {

public:

	GroveGPS(PinName tx=D1, PinName rx=D0) : gps_serial(tx, rx, 9600) {
	    memset(_isr_line_bufs, 0, sizeof(_isr_line_bufs));
	    _first_line_in_use = true;
	    _isr_line_buf_pos = 0;
	    memset(_last_line, 0, sizeof(_last_line));
	    _last_line_updated = false;
	    gps_serial.attach(callback(this, &GroveGPS::read_serial), SerialBase::RxIrq);
	}

	struct GGA {
		double  utc_time; 		// Format: hhmmss.sss
		double  latitude; 		// Format: ddmm.mmmm
		char    ns_indicator; 	// Format: N=north or S=south
		double  longitude;		// Format: dddmm.mmmm
		char    ew_indicator;	// Format: E=east or W=west
		int position_fix;	// Options: [0=not available, 1=GPS SPS mode, 2=Differential GPS, 6=dead reckoning]
		int sats_used;		// Range: 0-12
		double  hdop;			// Horizontal Dilution of Precision
		double  msl_altitude;
		char    msl_altitude_units;
		double  geoid_separation;
		char    geoid_separation_units;
		long    age_of_diff;
		long    diff_ref_station_id;
	} gps_gga;

	void getTimestamp(char* buffer) {
        m.lock();
        parseLine();

		sprintf(buffer, "%f", gps_gga.utc_time);
        m.unlock();
	}

	void getLatitude(char* buffer) {
        m.lock();
        parseLine();

		double coordinate = gps_gga.latitude;
		if (gps_gga.position_fix==0)
			sprintf(buffer, "N/A");
		else
			sprintf(buffer, "%c%f", (gps_gga.ns_indicator == 'N') ? '0' : '-', coordinate);
        m.unlock();
	}

	void getLongitude(char* buffer) {
        m.lock();
        parseLine();

		double coordinate = gps_gga.longitude;
		if (gps_gga.position_fix==0)
			sprintf(buffer, "N/A");
		else
		sprintf(buffer, "%c%f", (gps_gga.ew_indicator == 'E') ? '0' : '-', coordinate);
        m.unlock();
	}
	
	void getAltitude(char* buffer) {
        m.lock();
        parseLine();

		double coordinate = gps_gga.msl_altitude;
		if (gps_gga.position_fix==0)
			sprintf(buffer, "N/A");
		else
			sprintf(buffer, " %f",  coordinate);
        m.unlock();
	}
	
	void update() {
        m.lock();
        parseLine();
  	m.unlock();
  	}

private:
    static const size_t max_line_length = 256;
    char _isr_line_bufs[2][max_line_length];
    bool _first_line_in_use;
    size_t _isr_line_buf_pos;
    char _last_line[max_line_length];
    bool _last_line_updated;

    RawSerial gps_serial;
    Mutex m;

    void read_serial() {
        while (gps_serial.readable()) {

            // Check for overflow
            if (_isr_line_buf_pos > max_line_length -1 ) {
                error("GPS error - line too long");
                _isr_line_buf_pos = 0;
            }

            // Add a character to the active buffer
            char *buf = _isr_line_bufs[_first_line_in_use ? 0 : 1];
            char value = gps_serial.getc();
            buf[_isr_line_buf_pos] = value;
            _isr_line_buf_pos++;

            // Check for end of line
            if (value == '\n') {
                buf[_isr_line_buf_pos] = 0;
                _isr_line_buf_pos = 0;

                // Save off this line if it is valid
                if (memcmp("$GPGGA", buf, 6) == 0) {
                    _first_line_in_use = !_first_line_in_use;
                    _last_line_updated = true;
                }
            }
        }
    }

	double convertGPSToDecimal(double coordinate) {
		int degrees = coordinate/100.0;
		int minutes = ((int)coordinate) % 100;
		double seconds = coordinate - ((int)coordinate);
		return degrees + (minutes+seconds)/60;

	}

	void parseLine() {
	    bool parse_gga = false;

	    // Atomically copy the line buffer since the ISR can change it at any time
	    core_util_critical_section_enter();
	    if (_last_line_updated) {
	        char *buf_saved = _isr_line_bufs[_first_line_in_use ? 1 : 0];
	        strcpy(_last_line, buf_saved);
	        parse_gga = true;
            _last_line_updated = false;
	    }
	    core_util_critical_section_exit();

	    if (parse_gga) {
	        parseGGA();
	    }
	}

	void parseGGA() {
  		char *line_pos = _last_line;
		for (int i=0; i<14; i++) {
			if (i==0) { 		// NMEA Tag
			} else if (i==1) { 	// UTC time
				gps_gga.utc_time = strtod(line_pos, 0);
				
				
			} else if (i==2) { 	// Latitude
				gps_gga.latitude = strtod(line_pos, 0);
				gps_gga.latitude=convertGPSToDecimal(gps_gga.latitude);
				
			} else if (i==3) { 	// Latitude North/South indicator
				gps_gga.ns_indicator = line_pos[0];
			} else if (i==4) { 	// Longitude
				gps_gga.longitude = strtod(line_pos, 0);
				gps_gga.longitude=convertGPSToDecimal(gps_gga.longitude);
			} else if (i==5) { 	// Longitude indicator
				gps_gga.ew_indicator = line_pos[0];
			} else if (i==6) {
				gps_gga.position_fix= strtod(line_pos, 0);
			}
			else if (i==7) {
				gps_gga.sats_used=  strtod(line_pos, 0);//nb satellite used
			}
			else if (i==8) {
				gps_gga.hdop=  strtod(line_pos, 0);//horizontal precision
			}
			else if (i==9) {
				gps_gga.msl_altitude=  strtod(line_pos, 0);//altitute
			}
			
			
			line_pos = strchr(line_pos, ',');
			if (line_pos == NULL) {
			    break;
			}
			line_pos += 1;
		}
	}
};

#endif
