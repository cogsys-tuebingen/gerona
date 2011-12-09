// NmeaParser.h: Definition of the NMEA-0183 2.0 parser class.
//
// The CNmeaParser class generates data from NMEA-0183 2.0 sentences.  This
// class can parse the following types of NMEA sentences:
//   - GGA
//   - GLL
//   - RMC
//   - GSV
//	 - VTG
//
// The data that is extracted from sentences are as follows:
//   - Latitude
//   - Longitude
//   - Altitude (feet)
//   - Speed (knots)
//   - Current time (UTC)
//   - Current date (UTC)
//   - Track (degrees)
//   - Magnetic variation
//   - Horizontal dilution of precision (HDOP)
//   - Satellite data (PRN, elevation, azimuth, signal strength)
//
// All other data from these sentences are ignored.
//
// To allow the CNmeaParser class to parse a new sentence, do the following:
//  - Add a ParseXXX() function, where XXX is the name of the sentence.  Use
//    one of the existing parsing functions (for example, ParseGLL()) as a
//    guide.
//  - If the new sentence contains new information (for example, DGPS station
//    ID), add a function that parses and validates that information; call
//    it from your ParseXXX() function.
//  - Add a variable to the CNmeaData class that holds any new information
//    that your function parses; also add a boolean isXXXValid.  Reset those
//    variables within the CNmeaData::ResetData() function.
//  - Add the constant SENTENCE_XXX_COMMAS to this header file; this constant
//    contains the number of commas in your sentence.
//  - Add the following code to the ParseSentences() function anywhere within
//    the large if/else block:
//
//    // XXX = sentence type
//    } else if (strcmp (sentenceType, "XXX") == 0) {
//        if (CountChars (sentence, ',', SENTENCE_XXX_COMMAS) == false) {
//            return SENTENCE_INVALID;
//        }
//        ParseXXX (sentence);
//
//  - Add the following code to the IsValidSentenceType() function anywhere
//    within the large if/else block.
//
//    // XXX = sentence type
//    } else if (strcmp (sentenceType, "XXX") == 0) {
//        return true;
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

#ifndef __NMEAPARSER_H
#define __NMEAPARSER_H

#include "mytypes.h"
#include "satdata.h"
#include "convertunit.h"
#include "consts.h"
#include "types.h"
#include "gps.h"

// Sentence parsing status.
enum SENTENCE_STATUS
{
	SENTENCE_VALID,        // Sentence parsed is valid.
	SENTENCE_INVALID,      // Sentence parsed has invalid data.
	SENTENCE_BAD_CHECKSUM  // Sentence parsed has a bad checksum.
};

// Number of commas in each sentence.  In order for a sentence to be valid,
// it must have a specified number of commas.
const int SENTENCE_GGA_COMMAS = 14;
const int SENTENCE_GLL_COMMAS = 4;  // 6 is normal sentence
const int SENTENCE_RMC_COMMAS = 11;
const int SENTENCE_GSV_COMMAS = 19;
const int SENTENCE_VTG_COMMAS = 8;

// Maximum size of an NMEA sentence (plus the NULL character.)
const int MAX_SENTENCE_SIZE = 1024;

// No GPS I'm aware of can track more than 12 satellites.
const uint8 MAX_SATS = 12;

// Data class stored with the parser.  To extract the data parsed from the
// parser, pass an object of this class to the parser.
// NOTE! NMEA sentences are not "Year 2000-compliant"{tm}
struct CNmeaData
{
	CNmeaData ();
	void ResetData ();
    
    TimeStamp timeStamp;
	// Data retrieved from the NMEA sentences.
	double lat;          // Latitude, in rad (positive=N, negative=S)
	double lon;          // Longitude, in rad (positive=E, negative=W)
	double altitude;     // Altitude, in m
	double speed;        // Speed, in mps
	double track;        // Current track, in rad.
	double magVariation; // Magnetic variation, in degrees.
	double hdop;         // Horizontal dilution of precision.
	int numSats;         // Number of satellites in the sky.
	int UTCYear;         // GPS Date (UTC), year part
	int UTCMonth;        // GPS Date (UTC), month part  
	int UTCDay;          // GPS Date (UTC), day part
	int UTCHour;         // GPS Time (UTC), hour part.
	int UTCMinute;       // GPS Time (UTC), minute part  
	int UTCSecond;       // GPS Time (UTC), second part
	CSatData satData[MAX_SATS];

	// Quality of last fix:
	// 0 = invalid, 3 = GPS fix, 5 = DGPS fix.
	GPS_FIX_QUALITY fix;

	// Validity of data parsed.
	bool isValidLat;          // Latitude
	bool isValidLon;          // Longitude
	bool isValidAltitude;     // Altitude
	bool isValidSpeed;        // Speed
	bool isValidDate;         // Date
	bool isValidTime;         // Time
	bool isValidTrack;        // Track
	bool isValidMagVariation; // Magnetic variation
	bool isValidHdop;         // Horizontal dilution of precision
	bool isValidSatData;      // Satellite data

	// Has a valid coordinate ever been sent over the serial port?
	bool hasCoordEverBeenValid;
};


class CNmeaParser
{
	public:
		CNmeaParser ();
		SENTENCE_STATUS ParseSentence (const char* sentence);
		void GetData (CNmeaData& data) const;
		void ResetData () {m_data.ResetData ();}        
		

	private:
		bool ParseDegrees (double& degrees, const char* degString) const;
		bool ParseDate (int& year, int& month, int& day,
			const char* dateString) const;
		bool ParseTime (int& hour, int& minute, int& second,
			const char* timeString) const;
		void ParseAndValidateAltitude    (const char* field, const char unit);
		void ParseAndValidateDate        (const char* field);
		void ParseAndValidateFixQuality  (const char* field);
		void ParseAndValidateLat         (const char* field, const char hem);
		void ParseAndValidateLon         (const char* field, const char hem);
		void ParseAndValidateHdop        (const char* field);
		void ParseAndValidateSpeed       (const char* field);
		void ParseAndValidateMagVariation(const char* field,
			const char direction);
		void ParseAndValidateTime  (const char* field);
		void ParseAndValidateTrack (const char* field);

		void ParseGGA (const char* sentence);
		void ParseGLL (const char* sentence);
		void ParseRMC (const char* sentence);
		void ParseGSV (const char* sentence);
		void ParseVTG (const char* sentence);

		bool GetNextField (char* data, const char* sentence,
			uint& currentPosition) const;
		bool IsValidSentenceType (const char* sentence) const;
		bool IsCorrectChecksum (const char* sentence) const;

       
        int logSatelliteData (FILE *fp);

		CNmeaData m_data;

		// Needed for parsing the GSV sentence.
		int m_lastSentenceNumber;// Which sentence number was the last one?
		int m_numSentences;      // Number of sentences to process.
		int m_numSatsExpected;   // Number of satellites expected to parse.
		int m_numSatsLeft;       // Number of satellites left to parse.
		int m_satArrayPos;       // Array position of the next sat entry.		
		CSatData m_tempSatData[MAX_SATS];

};

#endif
