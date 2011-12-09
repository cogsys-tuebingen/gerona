// NmeaParser.cpp: Implementation of the NMEA-0183 2.0 parser class.
//
// Copyright (C) 1998, 1999 by Jason Bevins
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License (COPYING.txt) for more details.
//

#include <math.h>
#include <string.h>
#include <stdlib.h>
#include "nmeaparser.h"


// This constant define the length of the date string in a sentence.
const int DATE_LEN = 6;

int CountChars (const char* sentence, char charToCount,
	uint charCount);


/////////////////////////////////////////////////////////////////////////////
// CNmeaData construction/destruction

CNmeaData::CNmeaData ()
{
	// The constructor for the CNmeaData object clears all data and resets the
	// times of last aquisition to 0.
	ResetData ();
}


/////////////////////////////////////////////////////////////////////////////
// CNmeaData methods

void CNmeaData::ResetData ()
// Purpose:
//  This function resets all the data in this object to its defaults.
{
	// Reset the data.
	lat       = 0.0;
	lon       = 0.0;
	altitude  = 0.0;
	speed     = 0.0;
	UTCYear   = 94;
	UTCMonth  = 6;
	UTCDay    = 1;
	UTCHour   = 0;
	UTCMinute = 0;
	UTCSecond = 0;
	track     = 0.0;
	magVariation = 0.0;
	hdop      = 1.0;
	numSats   = 0;

	// All data stored by this object is currently invalid.
	isValidLat      = false;
	isValidLon      = false;
	isValidAltitude = false;
	isValidSpeed    = false;
	isValidDate     = false;
	isValidTime     = false;
	isValidTrack    = false;
	isValidMagVariation = false;
	isValidHdop     = false;
	isValidSatData  = false;

	// Last fix was invalid.
	fix = FIX_INVALID;

	// No coordinate has ever been valid since this object was created.
	hasCoordEverBeenValid = false;
    timeStamp.update();
}


/////////////////////////////////////////////////////////////////////////////
// CNmeaParser construction/destruction

CNmeaParser::CNmeaParser ()
{
	// Defaults for the GSV sentence parser.
	m_lastSentenceNumber = 1;
	m_numSentences       = 1;
	m_numSatsExpected    = 0;
	m_numSatsLeft        = 0;
	m_satArrayPos        = 0;
}


/////////////////////////////////////////////////////////////////////////////
// CNmeaParser members

void CNmeaParser::GetData (CNmeaData& data) const
// Purpose:
//  This function retrieves the parsed data from the sentences.
{
	data = m_data;
}




int CNmeaParser::logSatelliteData (FILE *fp)
{
    return EOK;
}




void CNmeaParser::ParseGGA (const char* sentence)
// Purpose:
//  This function parses a GGA sentence; all data will be stored in the
//  CNmeaData object within this class.  This function will correctly parse
//  a partial GGA sentence.
// Pre:
//  The string to parse must be a valid GGA sentence.
// GPGGA Global positioning system fixed data
{
	char field[255];
	char hemisphereField[32];
	char hemisphereChar;
	char unitField[32];
	char unitChar;
	uint currentPos = 0;
	bool isMore = true; // More strings to parse?

	// Skip past the '$xxGGA'
	if ((isMore = GetNextField (field, sentence, currentPos)) == false) {
		return;
	}
	
	// UTC time.
	isMore = GetNextField (field, sentence, currentPos);
	ParseAndValidateTime (field);
	if (isMore == false) return;

	// Latitude.
	isMore = GetNextField (field, sentence, currentPos);
	if (isMore == false) return; 
	isMore = GetNextField (hemisphereField, sentence, currentPos);
	if (strlen (hemisphereField) != 0) {
		hemisphereChar = hemisphereField[0];
	} else {
		hemisphereChar = ' ';
	}
	ParseAndValidateLat (field, hemisphereChar);
	if (isMore == false) return; 

	// Longitude.
	isMore = GetNextField (field, sentence, currentPos);
	if (isMore == false) return; 
	isMore = GetNextField (hemisphereField, sentence, currentPos);
	if (strlen (hemisphereField) != 0) {
		hemisphereChar = hemisphereField[0];
	} else {
		hemisphereChar = ' ';
	}
	ParseAndValidateLon (field, hemisphereChar);
	if (isMore == false) return; 

	// Quality of GPS fix.
	isMore = GetNextField (field, sentence, currentPos);
	ParseAndValidateFixQuality (field);
	if (isMore == false) return; 
    
    // set timestamp
    m_data.timeStamp.update();
	// Skip number of sats tracked for now.
	isMore = GetNextField (field, sentence, currentPos);
	if (isMore == false) return; 

	// Horizontal dilution of precision (HDOP).
	isMore = GetNextField (field, sentence, currentPos);
	ParseAndValidateHdop (field);
	if (isMore == false) return;
	
	// Altitude.
	isMore = GetNextField (field, sentence, currentPos);
	if (isMore == false) return; 
	isMore = GetNextField (unitField, sentence, currentPos);
	if (strlen (unitField) != 0) {
		unitChar = unitField[0];
	} else {
		unitChar = ' ';
	}
	ParseAndValidateAltitude (field, unitChar);
	if (isMore == false) return; 
    
	// Everything else (geoid height and DGPS info) is ignored for now.
	return;
}


void CNmeaParser::ParseGLL (const char* sentence)
// Purpose:
//  This function parses a GLL sentence; all data will be stored in the
//  CNmeaData object within this class.  This function will correctly parse
//  a partial GLL sentence.
// Pre:
//  The string to parse must be a valid GLL sentence.
{
	char field[255];
	char hemisphereField[32];
	char hemisphereChar;
	uint currentPos = 0;
	bool isMore = true; // More strings to parse?

	// Count commas in this sentence to see if it's valid.
	if (CountChars (sentence, ',', SENTENCE_GLL_COMMAS) < 0) return;

	// Skip past the '$xxGLL'
	if ((isMore = GetNextField (field, sentence, currentPos)) == false) {
		return;
	}
	
	// Latitude
	isMore = GetNextField (field, sentence, currentPos);
	if (isMore == false) return; 
	isMore = GetNextField (hemisphereField, sentence, currentPos);
	if (strlen (hemisphereField) != 0) {
		hemisphereChar = hemisphereField[0];
	} else {
		hemisphereChar = ' ';
	}
	ParseAndValidateLat (field, hemisphereChar);
	if (isMore == false) return; 

	// Longitude
	isMore = GetNextField (field, sentence, currentPos);
	if (isMore == false) return; 
	isMore = GetNextField (hemisphereField, sentence, currentPos);
	if (strlen (hemisphereField) != 0) {
		hemisphereChar = hemisphereField[0];
	} else {
		hemisphereChar = ' ';
	}
	ParseAndValidateLon (field, hemisphereChar);
	if (isMore == false) return; 
    // set timestamp
    m_data.timeStamp.update();

	// UTC time
	isMore = GetNextField (field, sentence, currentPos);
	ParseAndValidateTime (field);
	if (isMore == false) return;
}


void CNmeaParser::ParseGSV (const char* sentence)
// Purpose:
//  This function parses a GSV sentence; all data will be stored in the
//  CNmeaData object within this class.  This function will correctly parse
//  a partial GSV sentence.
// Pre:
//  The string to parse must be a valid GSV sentence.
// Notes:
//  All GSV sentences from a single packet (a collection of NMEA sentences
//  sent from the GPS) must be processed before satellite information in the
//  CNmeaData object is updated.  There is data for only four satellites
//  in each GSV sentence, so multiple sentences must be processed.  For
//  example, if your GPS was tracking eight satellites, two GSV sentences is
//  sent from your GPS in each packet; both sentences must be parsed before
//  the CNmeaData object is updated with the satellite information.
{
	char field[255];
	int numSats;
	int numSentences;
	int sentenceNumber;
	uint currentPos = 0;
	bool isMore = true;  // More strings to parse?

	// Count commas in this sentence to see if it's valid.
	if (CountChars (sentence, ',', SENTENCE_GSV_COMMAS) < 0) return;

	// Skip past the '$xxGSV'
	if ((isMore = GetNextField (field, sentence, currentPos)) == false) {
		return;
	}

	// Determine the number of sentences that will make up the satellite data.
	isMore = GetNextField (field, sentence, currentPos);
	numSentences = atoi (field);
	if (isMore == false) return; 

	// Which sentence is this?.
	isMore = GetNextField (field, sentence, currentPos);
	sentenceNumber = atoi (field);
	if (isMore == false) return; 

	// How many satellites are in total?
	isMore = GetNextField (field, sentence, currentPos);
	numSats = atoi (field);
	if (isMore == false) return;

	// Is this the first sentence?  If so, reset the satellite information.
	if (sentenceNumber == 1) {
		m_lastSentenceNumber = 1;
		m_numSentences = numSentences;
		m_numSatsExpected = numSats;
		m_numSatsLeft = numSats;
		m_satArrayPos = 0;
	} else {
		// Make sure the satellite strings are being sent in order.  If not,
		// then you're screwed.
		if (sentenceNumber != m_lastSentenceNumber + 1) return;

		// BUGFIX:
		// Added by Clarke Brunt 20001112
		m_lastSentenceNumber = sentenceNumber;
	}

	// Parse the satellite string.  There are four satellite info fields per
	// sentence.
    int i;
	for (i = 0; i < 4; i++) {
		GetNextField (field, sentence, currentPos);
		if (strlen (field) != 0) {
			m_tempSatData[m_satArrayPos].prn = atoi (field);
			GetNextField (field, sentence, currentPos);
			if (strlen (field) != 0) {
				m_tempSatData[m_satArrayPos].elevation = atoi (field);
			}
			GetNextField (field, sentence, currentPos);
			if (strlen (field) != 0) {
				m_tempSatData[m_satArrayPos].azimuth = atoi (field);
			}
			GetNextField (field, sentence, currentPos);
			if (strlen (field) != 0) {
				m_tempSatData[m_satArrayPos].strength = atoi (field);
			}
			--m_numSatsLeft;
			++m_satArrayPos;
		} else {
			// Jump past the next three fields.
			for (int j = 0; j < 3; j++)
				GetNextField (field, sentence, currentPos);
		}
	}

	// If all the satellite information has been received, then update the
	// CNmeaData object with the new satellite data.
	if (m_numSatsLeft == 0) {
		for (i = 0; i < m_numSatsExpected; i++) {
			m_data.satData[i] = m_tempSatData[i];
		}
		m_data.numSats = m_numSatsExpected;
		m_data.isValidSatData = true;
	}
}


void CNmeaParser::ParseRMC (const char* sentence)
// Purpose:
//  This function parses an RMC sentence; all data will be stored in the
//  CNmeaData object within this class.  This function will correctly parse
//  a partial RMC sentence.
// Pre:
//  The string to parse must be a valid RMC sentence.
{
	char field[255];
	char hemisphereField[32];
	char hemisphereChar;
	char directionField[32];
	char directionChar;
	uint currentPos = 0;
	bool isMore = true; // More strings to parse?

	// Count commas in this sentence to see if it's valid.
	if (CountChars (sentence, ',', SENTENCE_RMC_COMMAS) < 0) return;

	// Skip past the '$xxRMC'
	if ((isMore = GetNextField (field, sentence, currentPos)) == false) {
		return;
	}
	
	// UTC time
	isMore = GetNextField (field, sentence, currentPos);
	ParseAndValidateTime (field);
	if (isMore == false) return;

	// Skip past the navigation warning indicator for now.
	isMore = GetNextField (field, sentence, currentPos);
	if (isMore == false) return;

	// Latitude
	isMore = GetNextField (field, sentence, currentPos);
	if (isMore == false) return; 
	isMore = GetNextField (hemisphereField, sentence, currentPos);
	if (strlen (hemisphereField) != 0) {
		hemisphereChar = hemisphereField[0];
	} else {
		hemisphereChar = ' ';
	}
	ParseAndValidateLat (field, hemisphereChar);
	if (isMore == false) return; 

	// Longitude
	isMore = GetNextField (field, sentence, currentPos);
	if (isMore == false) return; 
	isMore = GetNextField (hemisphereField, sentence, currentPos);
	if (strlen (hemisphereField) != 0) {
		hemisphereChar = hemisphereField[0];
	} else {
		hemisphereChar = ' ';
	}
	ParseAndValidateLon (field, hemisphereChar);
    // set timestamp
    m_data.timeStamp.update();

	if (isMore == false) return; 

	// Current speed, in mps.
	isMore = GetNextField (field, sentence, currentPos);
	ParseAndValidateSpeed (field);
	if (isMore == false) return; 

	// Current track, in degrees.
	isMore = GetNextField (field, sentence, currentPos);
	ParseAndValidateTrack (field);
	if (isMore == false) return; 

	// Current date
	isMore = GetNextField (field, sentence, currentPos);
	ParseAndValidateDate (field);
	if (isMore == false) return;
	
	// Magnetic variation (degrees from true north)
	isMore = GetNextField (field, sentence, currentPos);
	if (isMore == false) return; 
	isMore = GetNextField (directionField, sentence, currentPos);
	if (strlen (directionField) != 0) {
		directionChar = directionField[0];
	} else {
		directionChar = ' ';
	}
	ParseAndValidateMagVariation (field, directionChar);
	if (isMore == false) return; 
}

void CNmeaParser::ParseVTG (const char* sentence)
// Purpose:
//  This function parses an VTG sentence; all data will be stored in the
//  CNmeaData object within this class.  
// Pre:
//  The string to parse must be a valid VTG sentence.
{
	char field[255];
	uint currentPos = 0;
	bool isMore = true; // More strings to parse?

	// Count commas in this sentence to see if it's valid.
	if (CountChars (sentence, ',', SENTENCE_VTG_COMMAS) < 0) return;

	// Skip past the '$xxVTG'
	if ((isMore = GetNextField (field, sentence, currentPos)) == false) {
		return;
	}
	// $GPVTG,067,T,067,M,03.5,N,06.4,K

	// Current track, in degrees.
	isMore = GetNextField (field, sentence, currentPos);
	ParseAndValidateTrack (field);
	if (isMore == false) return; 
	// skip T
	isMore = GetNextField (field, sentence, currentPos);
	if (isMore == false) return; 
	// skip magnetic oourse
	isMore = GetNextField (field, sentence, currentPos);
	if (isMore == false) return; 
	isMore = GetNextField (field, sentence, currentPos);
	if (isMore == false) return; 

	// Current speed, in mps.
	isMore = GetNextField (field, sentence, currentPos);
	ParseAndValidateSpeed (field);
    // set timestamp
    m_data.timeStamp.update();

	return;
}




SENTENCE_STATUS CNmeaParser::ParseSentence (const char* sentence)
// Purpose:
//  This function parses a given NMEA sentence.  All valid information will be
//  stored within the CNmeaData object in this class; call the GetData()
//  member function to retrieve the data.
// Parameters:
//  const char* sentence:
//      The sentence to parse.
// Returns:
//  SENTENCE_VALID if the sentence passed is a valid NMEA sentence.
//  SENTENCE_INVALID if the sentence passed is not a valid NMEA sentence, or
//  the sentence type is unsupported (see CNmeaParser.h for a list of
//  supported sentences.)
//  SENTENCE_BAD_CHECKSUM if the sentence has an invalid checksum.
{
    // log the sentence to file
 
	if (IsValidSentenceType (sentence) == true) {
//		if (IsCorrectChecksum (sentence) == true) {
		if (true) {

			// Start the parsing 3 spaces past start to get past the initial
			// '$xx', where xx is the device type sending the sentences (GP =
			// GPS, etc.)
			uint currentPos = 3;
			char sentenceType[10];
			
			if (GetNextField (sentenceType, sentence, currentPos) == false) {
				return SENTENCE_INVALID;
			}
			// Parse the sentence.  Make sure the sentence has the correct
			// number of commas in it, otherwise the sentence is invalid.
			if (strcmp (sentenceType, "GGA") == 0) {
				if (CountChars (sentence, ',', SENTENCE_GGA_COMMAS) < 0) {
					return SENTENCE_INVALID;
				}
				ParseGGA (sentence);
			} else if (strcmp (sentenceType, "GLL") == 0) {
				if (CountChars (sentence, ',', SENTENCE_GLL_COMMAS) < 0) {
					return SENTENCE_INVALID;
				}
				ParseGLL (sentence);
			} else if (strcmp (sentenceType, "RMC") == 0) {
				if (CountChars (sentence, ',', SENTENCE_RMC_COMMAS) < 0) {
					return SENTENCE_INVALID;
				}
				ParseRMC (sentence);
			} else if (strcmp (sentenceType, "GSV") == 0) {
				if (CountChars (sentence, ',', SENTENCE_GSV_COMMAS) < 0) {
					return SENTENCE_INVALID;
				}
				ParseGSV (sentence);
			} else if (strcmp (sentenceType, "VTG") == 0) {
				if (CountChars (sentence, ',', SENTENCE_VTG_COMMAS) < 0) {
					return SENTENCE_INVALID;
				}
				ParseVTG (sentence);
			} else {
				return SENTENCE_INVALID;
			}
			return SENTENCE_VALID;
		} else {
			return SENTENCE_BAD_CHECKSUM;
		}
	} else {
		return SENTENCE_INVALID;
	}
}


/////////////////////////////////////////////////////////////////////////////
// Member functions for individual element parsing of sentences.

bool CNmeaParser::ParseDate (int& year, int& month, int& day,
	const char* dateString) const
// Purpose:
//  This function parses a date string from an NMEA sentence in DDMMYY format
//  and returns the year, month, and day values.
// Parameters:
//  int& year, int& month, int& year:
//      Upon exit, these variables will contain the year, month, and day
//      values specified in dateString, respectively.
//  const char* dateString:
//      The NMEA date string to parse.
// Returns:
//  true if the date string is in a valid format, false if not.
// Notes:
//  - NMEA sentences are *not* "Year 2000-compliant"{tm}; the software must
//    correctly determine the year's century.
//  - If this function returns false, then the variables year, month, and day
//    are unaffected.
{
	// Date must be six characters.
	if (strlen (dateString) >= DATE_LEN) {
		long tempDate = atol (dateString);
		int tempYear, tempMonth, tempDay;
		tempYear  = tempDate - ((tempDate / 100) * 100);
		tempMonth = (tempDate - ((tempDate / 10000) * 10000)) / 100;
		tempDay   = tempDate / 10000;
		// Check to see if the date is valid.  (This function will accept
		// Feb 31 as a valid date; no check is made for how many days are in
		// each month of our whacked calendar.)
		if (   (tempYear  >= 0 && tempYear  <= 99)
			&& (tempMonth >= 1 && tempMonth <= 12)
			&& (tempDay   >= 1 && tempDay   <= 31)) {
			year = tempYear;
			month = tempMonth;
			day = tempDay;
			return true;
		} else {
			return false;
		}
	} else {
		return false;
	} 
}


bool CNmeaParser::ParseDegrees (double& degrees, const char* degString) const
// Purpose:
//  This function converts a lat/lon string returned from an NMEA string into
//  a numeric representation of that string, in degrees.  (A lat/lon string
//  must be in the format DDMM.M(...) where D = degrees and M = minutes.)
// Pre:
//  The string degString must contain a number in the format DDMM.M(...).
// Parameters:
//  double& degrees:
//      Upon exit, degrees will contain the numeric representation of the
//      string passed to this function, in decimal degrees.
//  const char* degString:
//      Contains the string to convert.
// Returns:
//  - true if the conversion was successful.  If false is returned, either
//    degString was not in one of those required formats, or the string data
//    itself is invalid.  (For example, the string 23809.666 would not be
//    valid, as the 238th degree does not exist.)
//  - If this function returns false, then the parameter 'degrees' is
//    unaffected.
{
	if (strlen (degString) != 0) {

		double tempMinutes;
		double tempDegrees;
		double tempPosition;

		tempPosition = atof (degString);
		tempDegrees  = (double)((int)(tempPosition / 100.0));
		tempMinutes  = (tempPosition - (tempDegrees * 100.0));
		tempPosition = tempDegrees + (tempMinutes / 60.0);

		if (tempPosition >= 0.0 || tempPosition <= 180.0) {
			degrees = tempPosition;
			return true;
		} else {
			return false;
		}
		
	} else {
		return false;
	}
}


bool CNmeaParser::ParseTime (int& hour, int& minute, int& second,
	const char* timeString) const
// Purpose:
//  This function parses a time string from an NMEA sentence in HHMMSS.S(...)
//  format and returns the hour, minute, and second values.
// Parameters:
//  int& hour, int& minute, int& second:
//      Upon exit, these variables will contain the hour, minute, and second
//      values specified in timeString, respectively.
//  const char* timeString:
//      The NMEA time string to parse.
// Returns:
//  true if the time string is in a valid format, false if not.
// Notes:
//  - Decimal second values are truncated.
//  - If this function returns false, then the variables hour, minute, and
//    second are unaffected.
{
	if (strlen (timeString) != 0) {
		long tempTime = atol (timeString);
		int tempHour, tempMinute, tempSecond;
		tempHour   = tempTime / 10000;
		tempMinute = (tempTime - ((tempTime / 10000) * 10000)) / 100;
		tempSecond = tempTime - ((tempTime / 100) * 100);
		// Check to see if the time is valid.
		if (   (tempHour   >= 0 && tempHour   <= 23)
			&& (tempMinute >= 0 && tempMinute <= 59)
			&& (tempSecond >= 0 && tempSecond <= 61)) { // leap seconds
			hour   = tempHour;
			minute = tempMinute;
			second = tempSecond;
			return true;
		} else {
			return false;
		}
	} else {
		return false;
	} 
}


/////////////////////////////////////////////////////////////////////////////
// Parse And Validate member functions for CNmeaParser.
//
// Each of these member functions parse a specified field from a sentence and
// updates the appropriate member variables in the CNmeaData object.  If the
// data parsed is valid, the validation member variable associated the parsed
// value is set to true, otherwise it is set to false.

void CNmeaParser::ParseAndValidateAltitude (const char* field, const char unit)
// Purpose:
//  This function parses the altitude field of a sentence.
// Parameters:
//  const char* field:
//      The altitude field of the sentence to parse.
//  const char unit:
//      The unit of altitude.  Valid values are 'f' (feet) and 'm' (meters).
// Notes:
//  The resulting altitude data is specified in feet.
{
	// Initially assume data is invalid.
	m_data.isValidAltitude = false;

	if (strlen (field) != 0) {
		if (unit == 'f' || unit == 'F') {
			// Altitude is in feet.
			m_data.altitude = atof (field)*FEET_TO_METERS;
			m_data.isValidAltitude = true;
		} else if (unit == 'm' || unit == 'M') {
			// Altitude is in meters.  
			m_data.altitude = atof (field);
			m_data.isValidAltitude = true;
		}
	}
}


void CNmeaParser::ParseAndValidateDate (const char* field)
// Purpose:
//  This function parses the date field of a sentence, in the format DDMMYY.
{
	// Initially assume data is invalid.
	m_data.isValidDate = false;

	if (strlen (field) != 0) {
		int year, month, day;
		if (ParseDate (year, month, day, field) == true) {
			m_data.UTCYear  = year;
			m_data.UTCMonth = month;
			m_data.UTCDay   = day;
			m_data.isValidDate = true;
		}
	}
}


void CNmeaParser::ParseAndValidateFixQuality (const char* field)
// Purpose:
//  This function parses the GPS fix quality field of a sentence.
{
	if (strlen (field) != 0) {
		int fixQuality = atoi (field);
		if (fixQuality == 0) m_data.fix = FIX_INVALID;
		else if (fixQuality == 1) m_data.fix = FIX_GPS_OK;
		else if (fixQuality == 2) m_data.fix = FIX_DGPS_OK;
	}
}


void CNmeaParser::ParseAndValidateHdop (const char* field)
// Purpose:
//  This function parses the HDOP (horizontal dilution of precision) field of
//  a sentence.
{
	if (strlen (field) != 0) {
		m_data.hdop = atof (field);
		m_data.isValidHdop = true;
	} else {
		m_data.isValidHdop = false;
	}
}


void CNmeaParser::ParseAndValidateLat (const char* field, const char hem)
// Purpose:
//  This function parses the latitude field of a sentence in the format
//  DDMM.M(...).
// Parameters:
//  const char* field:
//      The latitude field of the sentence to parse.
//  const char hem:
//      The hemisphere that contains the location.  Valid values are 'N' and
//      'S'.
// Notes:
//  - If the latitude is in the southern hemisphere, the latitude member
//    variable will be negative.  (e.g., 4000.000 S will be stored as -40.0.)
//  - If the latitude field does not exist within a sentence, the fix
//    quality variable, m_data.fix, is set to FIX_INVALID.
{
	// Initially assume data is invalid.
	m_data.isValidLat = false;

	if (strlen (field) != 0) {
		// GPS lat/lon data has been received.
		// Set the fix quality to "GPS navigation."  This is because some
		// GPS's may not send GGA sentences; therefore the last fix quality
		// would never get set.
		if (m_data.fix == FIX_INVALID) {
			m_data.fix = FIX_GPS_OK;
		}
		m_data.hasCoordEverBeenValid = true;
		double degree;
		if (ParseDegrees (degree, field) == true) {
			if (hem == 'N') {
				// Northern hemisphere.
				m_data.lat = degree*PI/180.0;
				m_data.isValidLat = true;
			} else if (hem == 'S') {
				// Southern hemisphere, so make latitude negative.
				m_data.lat = -degree*PI/180.0;
				m_data.isValidLat = true;
			}
		}
	} else {
		m_data.fix = FIX_INVALID;
	}
}


void CNmeaParser::ParseAndValidateLon (const char* field, const char hem)
// Purpose:
//  Same as ParseAndValidateLat(), but the longitude is in the format
//  DDDMM.M(...).
// Notes:
//  - The valid values for the hem parameter are 'E' and 'W'.
//  - If the longitude is in the western hemisphere, the longitude member
//    variable will be negative.  (e.g., 4000.000 W will be stored as -40.0.)
//  - If the latitude field does not exist within a sentence, the last fix
//    quality variable, m_data.fix, is set to FIX_INVALID.
{
	// Initially assume data is invalid.
	m_data.isValidLon = false;

	if (strlen (field) != 0) {
		// GPS lat/lon data has been received.
		// Set the fix quality to "GPS navigation."  This is because some
		// GPS's may not send GGA sentences; therefore the last fix quality
		// would never get set.
		if (m_data.fix == FIX_INVALID) {
			m_data.fix = FIX_GPS_OK;
			m_data.hasCoordEverBeenValid = true;
		}
		double degree;
		if (ParseDegrees (degree, field) == true) {
			if (hem == 'E') {
				// Eastern hemisphere.
				m_data.lon = degree*PI/180.0;
				m_data.isValidLon = true;
			} else if (hem == 'W') {
				// Western hemisphere, so make longitude negative.
				m_data.lon = -degree*PI/180;
				m_data.isValidLon = true;
			}
		}
	} else {
		m_data.fix = FIX_INVALID;
	}
}


void CNmeaParser::ParseAndValidateMagVariation (const char* field,
	const char direction)
// Purpose:
//  This function parses the magnetic variation field of a sentence, in
//  relation to true north.
// Parameters:
//  const char* field:
//      The magnetic variation field of the sentence to parse, in degrees.
//  const char direction:
//      The direction of the field in relation to true north.  Valid values
//      are 'E' and 'W'.
// Notes:
//  If the magnetic variation points west of true north, the magnetic
//  variation variable will be negative.  (e.g., 020.3 W will be stored as
//  -20.3.)
{
	// Initially assume data is invalid.
	m_data.isValidMagVariation = false;

	if (strlen (field) != 0) {
		double degree = atof (field);
		if (degree >= 0.0 && degree <= 360.0) {
			if (direction == 'E') {
				m_data.magVariation = degree;
				m_data.isValidMagVariation = true;
			} else if (direction == 'W') {
				m_data.magVariation = -degree;
				m_data.isValidMagVariation = true;
			}
		}
	}
}


void CNmeaParser::ParseAndValidateSpeed (const char* field)
// Purpose:
//  This function parses the speed field of a sentence.
{
	if (strlen (field) != 0) {
        // convert to m/sec
		m_data.speed = atof (field)*KNOTS_TO_MPS;
		m_data.isValidSpeed = true;
	} else {
		m_data.isValidSpeed = false;
	}
}


void CNmeaParser::ParseAndValidateTime (const char* field)
// Purpose:
//  This function parses the date field of a sentence, in the format
//  HHMMSS.S(...), except the decimal second values are truncated.
{
	// Initially assume data is invalid.
	m_data.isValidTime = false;

	if (strlen (field) != 0) {
		int hour, minute, second;
		if (ParseTime (hour, minute, second, field) == true) {
			m_data.UTCHour   = hour;
			m_data.UTCMinute = minute;
			m_data.UTCSecond = second;
			m_data.isValidTime = true;
		}
	}
}


void CNmeaParser::ParseAndValidateTrack (const char* field)
// Purpose:
//  This function parses the track field of a sentence.
{
	// Initially assume data is invalid.
	m_data.isValidTrack = false;

	if (strlen (field) != 0) {
		double track = atof (field);
		if (track >= 0.0 && track <= 360.0) {
			m_data.track = track*PI/180.0;
			m_data.isValidTrack = true;
		}
	}
}


/////////////////////////////////////////////////////////////////////////////
// Miscellaneous member functions

bool CNmeaParser::GetNextField (char* data, const char* sentence,
	uint& currentPos) const
// Purpose:
//  This function retrieves the next field in the NMEA sentence.  A field is
//  defined as the text between two delimiters (in this case of NMEA
//  sentences, a delimiter is a comma character.)
// Pre:
//  The specified sentence is valid.  (Before calling this function, call the
//  member functions IsCorrectChecksum() and ValidSentenceType(), passing the
//  sentence to those functions.)
// Parameters:
//  char* data:
//      Upon exit, this string will contain the contents of the next field
//      in the sentence.
//  const char* sentence:
//      The NMEA sentence to parse.
//  uint& currentPos:
//      Determines the initial position within the NMEA sentence in which to
//      parse.  This function will grab all of the characters from
//      currentPos all the way to the character before the comma delimiter.
//      Upon exit, currentPosition will point to the next field in the string.
//      Note that the comma is not included in the field data.
// Returns:
//      true if there are more fields to parse, false if not.
// Notes:
//      To grab all of the fields, you can iteratively call GetNextData()
//      using the same variable that is passed as currentPosition, until
//      GetNextData() returns false.
{
	int srcPos = currentPos;
	int dstPos = 0;
	char currentChar = sentence[srcPos];

	// The delimiter character is the comma.
	while ((currentChar != '\0'  ) && (currentChar != ',')
		&& (currentChar != '\x0d') && (currentChar != '*')) {

		data[dstPos++] = currentChar;
		currentChar = sentence[++srcPos];
	}
	data[dstPos] = '\0';

	if (currentChar == ',') {
		// Next data field to parse will be past the comma.
		currentPos = srcPos + 1;
		return true;
	} else {
		// No more characters in the string to parse; this function has
		// arrived at the end of the string.
		return false;
	}
}


bool CNmeaParser::IsCorrectChecksum (const char* sentence) const
// Purpose:
//  This function calculates the sentence's checksum and compares it with the
//  checksum in the sentence.
// Pre:
//  The NMEA sentence is valid (ValidSentenceStructure() must be called with
//  this sentence before calling this function; that function must return
//  true.)
// Returns:
//  true if the checksum is valid or there is no checksum in the sentence.
//  (It is not necessary to have a device append a checksum to a sentence.)
//  Otherwise this function returns false.
// Notes:
//  The checksum in the sentence occurs after the * character.
{
	// Check all characters between the initial '$' and the last "*" in the
	// sentence and XOR them together.

	int charPos = 1; // start past the initial '$'.
	char currentChar = sentence[charPos];
	uint8 checksum = 0;

	while (currentChar != '*' && currentChar != '\0') {
		checksum ^= (uint8)currentChar;
		currentChar = sentence[++charPos];
	}

	// If no checksum exists (this function has reached the end of the string
	// without finding one), the sentence is good.
	if (sentence[charPos + 1] == '\0') return true;

	// Convert last two hex characters (the checksum) in the sentence with
	// the checksum this function has generated.
	char firstDigit = sentence[charPos + 1];
	char lastDigit  = sentence[charPos + 2];
	if (  (firstDigit <= '9' ? firstDigit - '0': (firstDigit - 'A') + 10) * 16
		+ (lastDigit  <= '9' ? lastDigit  - '0': (lastDigit  - 'A') + 10)
		== checksum) {
		return true;
	} else {
		return false;
	}


}


bool CNmeaParser::IsValidSentenceType (const char* sentence) const
// Purpose:
//  This function determines whether this is a valid NMEA sentence that this
//  class can support.
// Notes:
//  See the header file for a list of sentences supported by this class.
{
	if (strlen (sentence) > 6) {
		if (sentence[0] == '$') {
			// Get the three letters after the '$xx'; this is the type of
			// sentence.  (Note the xx is the type of device which is sending
			// the string.  For example, GP = GPS, etc.)
			char sentenceType[4];
			memcpy (sentenceType, &(sentence[3]), 3);
			sentenceType[3] = '\0';

			if (strcmp (sentenceType, "GGA") == 0) {
				return true;
			} else if (strcmp (sentenceType, "GLL") == 0) {
				return true;
			} else if (strcmp (sentenceType, "RMC") == 0) {
				return true;
			} else if (strcmp (sentenceType, "GSV") == 0) {
				return true;
			} else if (strcmp (sentenceType, "VTG") == 0) {
				return true;
			} else {
				return false;
			}
		} else {
			return false;
		}
	} else {
		return false;
	}
}


int CountChars (const char* string, char charToCount, uint charCount)
// Purpose:
//  This function counts the number of specified occurrances(sp?) of the
//  specified characters and compares to the number of characters that is
//  expected.
// Parameters:
//  const char* string:
//      The string to check.
//  char charToCount:
//      The character to count.
//  uint charCount:
//      The number of the characters specified by charToCount that is expected
//      to be contained within that string.
// Returns:
//  0 if the number of specified characters in the sentence matches charCount.
//  1 if the number of specified characters in the sentence is less than
//  charCount.
//  -1 if the number of specified characters in the sentence is greater than
//  charCount.
{
	uint stringSize = strlen (string);
	uint currentCharCount = 0;
	const char* currentChar = string;
	for (uint i = 0; i < stringSize; i++) {
		if (*currentChar++ == charToCount) ++currentCharCount;
	}
	if (currentCharCount > charCount) {
		return 1;
	} else if (currentCharCount < charCount) {
		return -1;
	} else {
		return 0;
	}
}
