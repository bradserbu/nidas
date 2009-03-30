/*
    Copyright 2009 UCAR, NCAR, All Rights Reserved

    $LastChangedDate:  $

    $LastChangedRevision:  $

    $LastChangedBy: dongl $

    $HeadURL: http://svn.eol.ucar.edu/svn/nidas/trunk/src/nidas/dynld/isff/WisardMote.h $

 */

#include "WisardMote.h"
#include <nidas/util/Logger.h>
#include <cmath>
#include <iostream>
#include <memory> // auto_ptr<>

using namespace nidas::dynld;
using namespace nidas::dynld::isff;
using namespace nidas::core;
using namespace std;
namespace n_u = nidas::util;


std::map<unsigned char, WisardMote::setFunc> WisardMote::nnMap;
//public:	static std::map<int, setFunc> nnMap;
static bool mapped = false;

NIDAS_CREATOR_FUNCTION_NS(isff,WisardMote)


WisardMote::WisardMote() {
	//static bool mapped = false;
	fromLittle = n_u::EndianConverter::getConverter(n_u::EndianConverter::EC_LITTLE_ENDIAN);

	if (! mapped) {
		WisardMote::nnMap[0x01] = &WisardMote::setPicTm;
		WisardMote::nnMap[0x0f] = &WisardMote::setPicDT;

		WisardMote::nnMap[0x20] = &WisardMote::setTsoilData;
		WisardMote::nnMap[0x21] = &WisardMote::setTsoilData;
		WisardMote::nnMap[0x22] = &WisardMote::setTsoilData;
		WisardMote::nnMap[0x23] = &WisardMote::setTsoilData;

		WisardMote::nnMap[0x24] = &WisardMote::setGsoilData;
		WisardMote::nnMap[0x25] = &WisardMote::setGsoilData;
		WisardMote::nnMap[0x26] = &WisardMote::setGsoilData;
		WisardMote::nnMap[0x27] = &WisardMote::setGsoilData;

		WisardMote::nnMap[0x28] = &WisardMote::setQsoilData;
		WisardMote::nnMap[0x29] = &WisardMote::setQsoilData;
		WisardMote::nnMap[0x2A] = &WisardMote::setQsoilData;
		WisardMote::nnMap[0x2B] = &WisardMote::setQsoilData;

		WisardMote::nnMap[0x2C] = &WisardMote::setTP01Data;
		WisardMote::nnMap[0x2D] = &WisardMote::setTP01Data;
		WisardMote::nnMap[0x2E] = &WisardMote::setTP01Data;
		WisardMote::nnMap[0x2F] = &WisardMote::setTP01Data;

		WisardMote::nnMap[0x49] = &WisardMote::setPwrData;

		WisardMote::nnMap[0x50] = &WisardMote::setRnetData;
		WisardMote::nnMap[0x51] = &WisardMote::setRnetData;
		WisardMote::nnMap[0x52] = &WisardMote::setRnetData;
		WisardMote::nnMap[0x53] = &WisardMote::setRnetData;

		WisardMote::nnMap[0x54] = &WisardMote::setRswData;
		WisardMote::nnMap[0x55] = &WisardMote::setRswData;
		WisardMote::nnMap[0x56] = &WisardMote::setRswData;
		WisardMote::nnMap[0x57] = &WisardMote::setRswData;

		WisardMote::nnMap[0x58] = &WisardMote::setRswData;
		WisardMote::nnMap[0x59] = &WisardMote::setRswData;
		WisardMote::nnMap[0x5A] = &WisardMote::setRswData;
		WisardMote::nnMap[0x5B] = &WisardMote::setRswData;

		WisardMote::nnMap[0x5C] = &WisardMote::setRlwData;
		WisardMote::nnMap[0x5D] = &WisardMote::setRlwData;
		WisardMote::nnMap[0x5E] = &WisardMote::setRlwData;
		WisardMote::nnMap[0x5F] = &WisardMote::setRlwData;

		WisardMote::nnMap[0x60] = &WisardMote::setRlwData;
		WisardMote::nnMap[0x61] = &WisardMote::setRlwData;
		WisardMote::nnMap[0x62] = &WisardMote::setRlwData;
		WisardMote::nnMap[0x63] = &WisardMote::setRlwData;
		mapped = true;
	}
}

bool WisardMote::process(const Sample* samp,list<const Sample*>& results) throw()
{
	/*  sample input --- there are multiple data-  */
	const unsigned char* cp= (const unsigned char*) samp->getConstVoidDataPtr();
	const unsigned char* eos = cp + samp->getDataByteLength();
	int len = samp->getDataByteLength();
	printf(" \n process -raw data = ");
    for (int i= 0; i<len; i++) printf(" %x ", *(cp+i));

	/*  find EOM  */
	if (!findEOM(cp, samp->getDataByteLength())) return false;

	/*  verify crc for data  */
	if (!findCRC(cp, samp->getDataByteLength())) return false;

	/*  get header  -- return data header, ignore other headers */
	nname="";
	msgLen=0;
	if (!findHead(cp, eos, msgLen)) return false;

	/*  move cp point to process data   */
	cp +=msgLen;

	// crc+eom+0x0(1+3+1) + sensorTypeId+data (1+2 at least) = 8
	while (cp+8 < eos) {
		/*  get data one set data */
		msgLen=0;
		data.clear();

		/* get sTypeId    */
		unsigned char sTypeId = *cp++; msgLen++;
		printf("\n\n\n SensorTypeId = %x \n",sTypeId);

		/* push nodename+sStypeId to list  */
		pushNodeName(getId(), sTypeId);                     //getId()--get dsm and sensor ids

		/* getData  */
		//printf("\n process-- check      sTypeId= %x          nnMap.[sTypeId] = %x   \n ", sTypeId, nnMap[sTypeId]);
		if ( nnMap[sTypeId]==NULL  ) {
			printf("\n process--getData--cannot find the setFunc.  sTypeId = %x ...   No data...\n ",sTypeId);
			return false;
		}
	   // printf("Good,  retrieving data ...    sTypeId = %x", sTypeId);
		(this->*nnMap[sTypeId])(cp,eos);

		//readData(cp, eos);
		if (data.size() == 0) 	return false;

		/*  output    */
		SampleT<float>* osamp = getSample<float>(data.size());
		osamp->setTimeTag(samp->getTimeTag());
		osamp->setId((dsm_sample_id_t)nodeIds[lnname]);
		for (unsigned int i=0; i<data.size(); i++) {
			osamp->getDataPtr()[i] = data[i];
			printf("\ndata i %f %i", data[i], i);
		}

		/* push out   */
		results.push_back(osamp);

		/* move cp to right position */
		cp += msgLen;
	}
	return true;
}

void WisardMote::fromDOMElement(
		const xercesc::DOMElement* node)
throw(n_u::InvalidParameterException)
{

	DSMSensor::fromDOMElement(node);

	const std::list<const Parameter*>& params = getParameters();
	list<const Parameter*>::const_iterator pi;
	for (pi = params.begin(); pi != params.end(); ++pi) {
		const Parameter* param = *pi;
		const string& pname = param->getName();
		if (pname == "rate") {
			if (param->getLength() != 1)
				throw n_u::InvalidParameterException(getName(),"parameter",
						"bad rate parameter");
			//setScanRate((int)param->getNumericValue(0));
		}
	}
}

void WisardMote::pushNodeName(unsigned int id, int sTypeId) {
	lnname = nname;
	lnname.push_back(',');
	char buffer [5];
	sprintf (buffer, "%x",sTypeId);
	lnname += buffer;
	remove(lnname.begin(), lnname.end(), ' ');
	printf("lnname= %s getId()= %i \n", lnname.c_str(), id);

	unsigned int sampleId = nodeIds[lnname];
	if (sampleId == 0) {
		sampleId = id + sTypeId;
		nodeIds[lnname] = sampleId;
	}
}

//void WisardMote::readData(const unsigned char* cp, const unsigned char* eos, vector<float>& data, int& msgLen)  {
void WisardMote::readData(const unsigned char* cp, const unsigned char* eos)  {

	/* get sTypeId    */
	int sTypeId = *cp++; msgLen++;
	printf("\n SensorTypeId = %x \n",sTypeId);

	/* push nodename+sStypeId to list  */
	pushNodeName(getId(), sTypeId);                     //getId()--get dsm and sensor ids

	/* getData  */
	(this->*nnMap[sTypeId])(cp,eos);
}

/**
 * find NodeName, version, MsgType (0-log sensortype+SN 1-seq+time+data 2-err msg)
 */
bool WisardMote::findHead(const unsigned char* cp, const unsigned char* eos, int& msgLen) {
	n_u::Logger::getInstance()->log(LOG_INFO, "findHead...");
	/* look for nodeName */
	for ( ; cp < eos; cp++, msgLen++) {
		char c = *cp;
		if (c!= ':') nname.push_back(c);
		else break;
	}
	if (*cp != ':') return false;

	cp++; msgLen++; //skip ':'
	if (cp == eos) return false;

	// version number
	int ver = *cp; msgLen++;
	if (++cp == eos) return false;

	// message type
	int mtype = *cp++; msgLen++;
	if (cp == eos) return false;

	switch(mtype) {
	case 0:
		/* unpack 1bytesId + 16 bit s/n */
		if (cp + 1 + 2 > eos) return false;
		int sId;
		sId = *cp++; msgLen++;
		int sn;
		sn = fromLittle->uint16Value(cp);
		cp += sizeof(uint16_t); msgLen+= sizeof(uint16_t);
		n_u::Logger::getInstance()->log(LOG_INFO,"NodeName=%s Ver=%x MsgType=%x STypeId=%x SN=%x hmsgLen=%i",
				nname.c_str(), ver, mtype, sId, sn, msgLen);
		return false;
	case 1:
		/* unpack 1byte seq + 16-bit time */
		if (cp + 1+ sizeof(uint16_t) > eos) return false;
		unsigned char seq;
		seq = *cp++;  msgLen++;
		int	time;
		time = fromLittle->uint16Value(cp);
		cp += sizeof(uint16_t); msgLen+=sizeof(uint16_t);
		n_u::Logger::getInstance()->log(LOG_INFO,"NodeName=%s Ver=%x MsgType=%x seq=%x time=%i hmsgLen=%i",
				nname.c_str(), ver, mtype, seq, time, msgLen);
		printf("\n NodeName=%s Ver=%x MsgType=%x seq=%x time=%i hmsgLen=%i",
				nname.c_str(), ver, mtype, seq, time, msgLen);
		break;
	case 2:
		n_u::Logger::getInstance()->log(LOG_ERR,"NodeName=%s Ver=%x MsgType=%x hmsgLen=%i ErrMsg=%s",
				nname.c_str(), ver, mtype, msgLen, cp);
		return false;//skip for now
	default:
		n_u::Logger::getInstance()->log(LOG_ERR,"Unknown msgType --- NodeName=%s Ver=%x MsgType=%x hmsgLen=%i",
				nname.c_str(), ver, mtype, msgLen);
		return false;
	}

	return true;
}

/**
 * EOM (0x03 0x04 0xd) + 0x0
 */
bool WisardMote::findEOM(const unsigned char* cp, unsigned char len) {
	n_u::Logger::getInstance()->log(LOG_INFO, "findEOM len= %d ",len);

	if (len< 4 ) {
		n_u::Logger::getInstance()->log(LOG_ERR,"Message length is too short --- len= %d", len );
		return false;
	}

	unsigned char lidx =len-1;
	if (*(cp+lidx)!= 0x0 ||*(cp+lidx-1)!= 0xd ||*(cp+lidx-2)!= 0x4 ||*(cp+lidx-3)!= 0x3 ) {
		n_u::Logger::getInstance()->log(LOG_ERR,"Bad EOM --- last 4 chars= %x %x %x %x ", *(cp+lidx-3), *(cp+lidx-2), *(cp+lidx-1), *(cp+lidx) );
		return false;
	}
	return true;
}


bool WisardMote::findCRC (const unsigned char* cp, unsigned char len) {
	unsigned char lidx =len-1;

	// retrieve CRC-- 3byteEOM  + 1byte0x0
	unsigned char crc= *(cp+lidx-4);

	//calculate Cksum
	unsigned char cksum = len - 5;  //skip CRC+EOM+0x0
	for(int i=0; i< (len-5); i++) {
		unsigned char c =*cp++;
		//printf("crc-cal-char= %x \n", c);
		cksum ^= c ;
	}

	if (cksum != crc ) {
		n_u::Logger::getInstance()->log(LOG_ERR,"Bad CKSUM --- %x vs  %x ", crc, cksum );
		return false;
	}
	return true;
}


void WisardMote::setPicTm(const unsigned char* cp, const unsigned char* eos){
	/* unpack  16 bit pic-time */
	if (cp + sizeof(uint16_t) > eos) return;
	int	pict;
	pict=  (fromLittle->uint16Value(cp));
	cp += sizeof(uint16_t); msgLen+=sizeof(uint16_t);
	n_u::Logger::getInstance()->log(LOG_INFO,"\nPic_time = %x ",  pict);
}
void WisardMote::setPicDT(const unsigned char* cp, const unsigned char* eos){
	/*  16 bit jday */
	if (cp + sizeof(uint16_t) > eos) return;
	int jday;
	jday= (fromLittle->uint16Value(cp));
	cp += sizeof(uint16_t); msgLen+=sizeof(uint16_t);
	/*  8 bit hour+ 8 bit min+ 8 bit sec  */
	if ((cp + 3* sizeof(uint8_t)) > eos) return;
	int hh,mm,ss;
	hh= *cp;
	cp += sizeof(uint8_t); msgLen+=sizeof(uint8_t);
	mm= *cp;
	cp += sizeof(uint8_t); msgLen+=sizeof(uint8_t);
	ss= *cp;
	cp += sizeof(uint8_t); msgLen+=sizeof(uint8_t);
	n_u::Logger::getInstance()->log(LOG_INFO,"\n jday= %x hh=%x mm=%x ss=%x",  jday, hh, mm, ss);
}

void WisardMote::setTsoilData(const unsigned char* cp, const unsigned char* eos){
	/* unpack 16 bit  */
	for (int i=0; i<4; i++) {
		if (cp + sizeof(int16_t) > eos) return;
		data.push_back((fromLittle->int16Value(cp))/10.0);
		cp += sizeof(int16_t); msgLen+=sizeof(int16_t);
	}
}
void WisardMote::setGsoilData(const unsigned char* cp, const unsigned char* eos){
	if (cp + sizeof(int16_t) > eos) return;
	data.push_back((fromLittle->int16Value(cp))/1.0);
	cp += sizeof(int16_t); msgLen+=sizeof(int16_t);
}
void WisardMote::setQsoilData(const unsigned char* cp, const unsigned char* eos){
	if (cp + sizeof(uint16_t) > eos) return;
	data.push_back((fromLittle->uint16Value(cp))/1.0);
	cp += sizeof(uint16_t); msgLen+=sizeof(uint16_t);
}
void WisardMote::setTP01Data(const unsigned char* cp, const unsigned char* eos){
	// 3 are singed
	for (int i=0; i<3; i++) {
		if (cp + sizeof(int16_t) > eos) return;
		data.push_back((fromLittle->int16Value(cp))/1.0);
		cp += sizeof(int16_t); msgLen+=sizeof(uint16_t);
	}
}

void WisardMote::setRnetData(const unsigned char* cp, const unsigned char* eos){
	if (cp + sizeof(int16_t) > eos) return;
	data.push_back((fromLittle->int16Value(cp))/10.0);
	cp += sizeof(int16_t); msgLen+=sizeof(uint16_t);
}
void WisardMote::setRswData(const unsigned char* cp, const unsigned char* eos){
	if (cp + sizeof(uint16_t) > eos) return;
	data.push_back((fromLittle->uint16Value(cp))/10.0);
	cp += sizeof(uint16_t); msgLen+=sizeof(uint16_t);
}
void WisardMote::setRlwData(const unsigned char* cp, const unsigned char* eos){
	for (int i=0; i<5; i++) {
		if (cp + sizeof(int16_t) > eos) return;
		data.push_back((fromLittle->int16Value(cp))/10.0);
		cp += sizeof(int16_t); msgLen+=sizeof(uint16_t);
	}
}

void WisardMote::setPwrData(const unsigned char* cp, const unsigned char* eos){
	for (int i=0; i<6; i++){
		if (cp + sizeof(uint16_t) > eos) return;
		data.push_back((fromLittle->uint16Value(cp))/10.0);
		cp += sizeof(uint16_t); msgLen+=sizeof(uint16_t);
	}
}

//const WisardMote::sMap= new WisardMote::sensorToFunc[265];
/*const WisardMote::sensorToFunc WisardMote::sMap[] = {
		{ },
		{ 0x01, &WisardMote::setPicTm},
		{ },{ },{ }, { },{ },{ },{ },{ }, { },{ },{ },{ },{ },
		{ 0x0F, &WisardMote::setPicDT},

		{ 0x20, &WisardMote::setTsoilData },
		{ 0x21, &WisardMote::setTsoilData },
		{ 0x22, &WisardMote::setTsoilData },
		{ 0x23, &WisardMote::setTsoilData },

		{ 0x24, &WisardMote::setGsoilData },
		{ 0x25, &WisardMote::setGsoilData },
		{ 0x26, &WisardMote::setGsoilData },
		{ 0x27, &WisardMote::setGsoilData },

		{ 0x28, &WisardMote::setQsoilData },
		{ 0x29, &WisardMote::setQsoilData },
		{ 0x2A, &WisardMote::setQsoilData },
		{ 0x2B, &WisardMote::setQsoilData },

		{ 0x2C, &WisardMote::setTP01Data },
		{ 0x2D, &WisardMote::setTP01Data },
		{ 0x2E, &WisardMote::setTP01Data },
		{ 0x2F, &WisardMote::setTP01Data },

		{ },{ },{ },{ },{ }, { },{ },{ },{ },{ }, { },{ },{ },{ },{ }, 	{ },
		{ },{ },{ },{ },{ }, { },{ },{ },{ }, { 0x49, &WisardMote::setPwrData }, { },{ },{ },{ },{ }, 	{ },

		{ 0x50, &WisardMote::setRnetData },
		{ 0x51, &WisardMote::setRnetData },
		{ 0x52, &WisardMote::setRnetData },
		{ 0x53, &WisardMote::setRnetData },

		{ 0x54, &WisardMote::setRswData },
		{ 0x55, &WisardMote::setRswData },
		{ 0x56, &WisardMote::setRswData },
		{ 0x57, &WisardMote::setRswData },

		{ 0x58, &WisardMote::setRswData },
		{ 0x59, &WisardMote::setRswData },
		{ 0x5A, &WisardMote::setRswData },
		{ 0x5B, &WisardMote::setRswData },

		{ 0x5C, &WisardMote::setRlwData },
		{ 0x5D, &WisardMote::setRlwData },
		{ 0x5E, &WisardMote::setRlwData },
		{ 0x5F, &WisardMote::setRlwData },

		{ 0x60, &WisardMote::setRlwData },
		{ 0x61, &WisardMote::setRlwData },
		{ 0x62, &WisardMote::setRlwData },
		{ 0x63, &WisardMote::setRlwData },
};
*/

