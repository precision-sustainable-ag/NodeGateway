// ACReliableMessage.cpp
//
//	Implement class to send long reliable string messages - inheriting from RHReliableDatagram
//
// Author: David Anderson
// Copyright (C) 2019 David Anderson
// $Id: ACReliableMessage.c, v 1.0 2019/05/30 $

#include <ACReliableMessage.h>

////////////////////////////////////////////////////////////////////
// Constructors
ACReliableMessage::ACReliableMessage(RHGenericDriver& driver, uint8_t thisAddress) : RHReliableDatagram(driver, thisAddress)
{

}

////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////
// reliably send a message
bool ACReliableMessage::sendMessage(String* msg, uint8_t to_address)
{
// We will steal one byte right after the header to show # of packets remaining.  When this reaches zero, we have the whole message.
	uint16_t Len = msg->length();
	uint8_t PacketCounter = (Len + AC_MAX_MSG_PACKET_SIZE - 1) / AC_MAX_MSG_PACKET_SIZE;	// number of packets required
	if (PacketCounter == 0) return true;											// Success!  We have transmitted nothing!
	PacketCounter--;																// convert to index/counter (zero = last item)

	// Serial.print("PacketCounter: ");
	// Serial.println(PacketCounter);
	// Serial.flush();

	uint16_t subdatastart = 0;
	uint16_t subdataend;
	uint16_t subdatalen;
	uint16_t x, i;
	if (Len > AC_MAX_MSG_PACKET_SIZE) subdatalen = RH_RF95_MAX_MESSAGE_LEN;
	else subdatalen = Len + 1;
	while (Len > 0) {
		if (Len > AC_MAX_MSG_PACKET_SIZE) {
			subdataend = subdatastart + AC_MAX_MSG_PACKET_SIZE;						// transmit up to, but not including subdataend
			Len = Len - AC_MAX_MSG_PACKET_SIZE;
		}
		else {
			subdataend = subdatastart + Len;
			Len = 0;
		}
		subdatalen = subdataend - subdatastart + 1;
		_PacketBuf[0] = PacketCounter;
		for (x = subdatastart, i = 1; x < subdataend; x++, i++) _PacketBuf[i] = (*msg)[x];// convert to byte array, only the portion we can send in this packet
		if (sendtoWait(_PacketBuf, subdatalen, to_address) == false) return false;	// send data.  Cancel send if one packet is unsuccessful.
//		Serial.print("<<Sending Packet ");
//		Serial.print(_PacketBuf[0]);
//		Serial.print(", size=");
//		Serial.print(subdatalen);
//		Serial.println(">>");
//		for (i = 1; i < subdatalen; i++) Serial.print(char(_PacketBuf[i]));// convert to byte array, only the portion we can send in this packet
//		Serial.println();
		PacketCounter--;
		subdatastart = subdataend;													// point to the next chunk
	}
	return true;			// success!  The message was transmitted and acknowledged
}

bool ACReliableMessage::sendMessage(uint8_t* msg, uint16_t msgLen, uint8_t to_address) {
	// We will steal one byte right after the header to show # of packets remaining.  When this reaches zero, we have the whole message.	
	
	uint8_t PacketCounter = (msgLen + AC_MAX_MSG_PACKET_SIZE - 1) / AC_MAX_MSG_PACKET_SIZE;	// number of packets required
	if (PacketCounter == 0) return true;											// Success!  We have transmitted nothing!
	PacketCounter--;																// convert to index/counter (zero = last item)

	// Serial.print("PacketCounter: ");
	// Serial.println(PacketCounter);
	// Serial.flush();

	uint16_t subdatastart = 0;
	uint16_t subdataend;
	uint16_t subdatalen;
	uint16_t x, i;
	if (msgLen > AC_MAX_MSG_PACKET_SIZE) subdatalen = RH_RF95_MAX_MESSAGE_LEN;
	else subdatalen = msgLen + 1;
	while (msgLen > 0) {
		if (msgLen > AC_MAX_MSG_PACKET_SIZE) {
			subdataend = subdatastart + AC_MAX_MSG_PACKET_SIZE;						// transmit up to, but not including subdataend
			msgLen = msgLen - AC_MAX_MSG_PACKET_SIZE;
		}
		else {
			subdataend = subdatastart + msgLen;
			msgLen = 0;
		}
		subdatalen = subdataend - subdatastart + 1;
		_PacketBuf[0] = PacketCounter;
		for (x = subdatastart, i = 1; x < subdataend; x++, i++) _PacketBuf[i] = *(msg + x);// convert to byte array, only the portion we can send in this packet
		if (sendtoWait(_PacketBuf, subdatalen, to_address) == false) return false;	// send data.  Cancel send if one packet is unsuccessful.
//		Serial.print("<<Sending Packet ");
//		Serial.print(_PacketBuf[0]);
//		Serial.print(", size=");
//		Serial.print(subdatalen);
//		Serial.println(">>");
//		for (i = 1; i < subdatalen; i++) Serial.print(char(_PacketBuf[i]));// convert to byte array, only the portion we can send in this packet
//		Serial.println();
		PacketCounter--;
		subdatastart = subdataend;													// point to the next chunk
	}
	return true;			// success!  The message was transmitted and acknowledged
}

////////////////////////////////////////////////////////////////////
//	recvMessage
//	Reliably receive a message that was sent using sendMessage
//	msg = string object to store the message in
//	StartTimeout = Time we are willing to wait for a message to begin
//	PacketTimeout = One the message is streaming, time allowed for one large packet to complete -- including retries
//  from_address = Radio address we recieved the message from
bool ACReliableMessage::recvMessage(String *msg, uint32_t StartTimeout, uint16_t PacketTimeout, uint8_t* from_address)
{  
	uint32_t waitStartTime = millis();
	uint8_t PacketCounter = 255;
	uint8_t PcktLen;
	*msg = "";
	while (PacketCounter > 0) {
		PcktLen = 255;
		if (recvfromAckTimeout(_PacketBuf, &PcktLen, PacketTimeout, from_address)) {		// wait for and receive a packet
			PacketCounter = _PacketBuf[0];
			for (uint8_t i = 1; i < PcktLen; i++) *msg += (char)_PacketBuf[i];						// append data to msg
			StartTimeout = 0;																// we got something.  PacketTimeout wait only from now on.
//			Serial.print("<<Got Packet ");
//			Serial.print(PacketCounter);
//			Serial.print(", size=");
//			Serial.print(PcktLen);
//			Serial.println(">>");
//			for (uint8_t i = 1; i < PcktLen; i++) Serial.print(char(_PacketBuf[i]));// convert to byte array, only the portion we can send in this packet
//			Serial.println();
		} else if (StartTimeout == 0) {
			Serial.println("recvMessage timeout 1");
			return false;											// we are mid-message, or not waiting for msg start.  End.
		}	else if ((millis() - waitStartTime) > StartTimeout) {
			Serial.println("recvMessage timeout 2");
			return false;					// we did not get anything back, and we are done waiting.
		}
	}
	return true;
}


void printBufAsChar(uint8_t* buf, uint16_t idx, uint16_t len) {
	for (uint16_t i = idx; i < (idx + len); i++) {
		Serial.print((char)*(buf + i));
	}
}


//New approach: We know with the first reception of data what the total size needs to be (actually we might go over by as much as 249 bytes) so just malloc and memcpy directly instead of double buffering
bool ACReliableMessage::recvMessage(uint8_t **msg, uint16_t *msgLen, uint32_t StartTimeout, uint16_t PacketTimeout, uint8_t* from_address) {  
	uint32_t waitStartTime = millis();
	uint8_t PacketCounter = 255;
	uint8_t PcktLen;

	bool result = true;
	
	*msgLen = 0;
	uint8_t *buf;
	bool needMalloc = true;
	
	while (PacketCounter > 0) {		
		PcktLen = 255;
		if (recvfromAckTimeout(_PacketBuf, &PcktLen, PacketTimeout, from_address)) {		// wait for and receive a packet
			PacketCounter = _PacketBuf[0];
			if (needMalloc) {
				buf = malloc((PacketCounter + 1) * (PcktLen - 1));
				needMalloc = false;

				// Serial.println("ACReliableMessage: ");
				// uint16_t bufAllocSize = *((uint16_t*)(buf - 2));
				// Serial.print("bufAllocSize: ");
				// Serial.println(bufAllocSize);
				// Serial.print("PcktLen: ");
				// Serial.println(PcktLen);
				// Serial.print("PacketCounter: ");
				// Serial.println(PacketCounter);
				// Serial.flush();
			}

			// Serial.print("PacketCounter: ");
			// Serial.println(PacketCounter);
			
			memcpy(buf + *msgLen, _PacketBuf + 1, PcktLen - 1);
			*msgLen += PcktLen - 1;
			
			StartTimeout = 0;																// we got something.  PacketTimeout wait only from now on.
		} else {
			if (StartTimeout == 0) {
				Serial.println("recvMessage timeout 1");
				result = false;
				break;
			}	else if ((millis() - waitStartTime) > StartTimeout) {
				Serial.println("recvMessage timeout 2");
				result = false;	
				break;
			}
		}
	}

	*msg = buf;

	// Serial.print("dataLen & data: ");
	// Serial.print(*msgLen);
	// Serial.print("; ");
	// printBufAsChar(buf, 0, *msgLen);
	// Serial.println();
	// Serial.flush();

	return result;
}


//Old approach: This works but is inefficient
// bool ACReliableMessage::recvMessage(uint8_t **msg, uint16_t *msgLen, uint32_t StartTimeout, uint16_t PacketTimeout, uint8_t* from_address) {  
// 	uint32_t waitStartTime = millis();
// 	uint8_t PacketCounter = 255;
// 	uint8_t PcktLen;

// 	bool result = true;
	
// 	*msgLen = 0;
// 	uint8_t *buf;
// 	uint8_t **bufArray = malloc(sizeof(uint8_t*) * 255);
// 	uint8_t bufArrayIdx = 0;


// 	while (PacketCounter > 0) {		
// 		PcktLen = 255;
// 		if (recvfromAckTimeout(_PacketBuf, &PcktLen, PacketTimeout, from_address)) {		// wait for and receive a packet
// 			buf = malloc(PcktLen);
// 			PacketCounter = _PacketBuf[0];
// 			Serial.print("PcktLen: ");
// 			Serial.print(PcktLen);
// 			Serial.print("; packetIdx & data: ");
// 			Serial.print(_PacketBuf[0]);
// 			Serial.print("; ");
// 			printBufAsChar(_PacketBuf, 1, PcktLen - 1);
// 			Serial.println();
// 			Serial.flush();
			
// 			*msgLen += PcktLen - 1;
// 			*(buf + 0) = PcktLen - 1;														//zero index will hold length
// 			memcpy(buf + 1, _PacketBuf + 1, PcktLen - 1);
// 			// for (uint8_t i = 1; i < PcktLen; i++)
// 			// 	*(buf + i) = _PacketBuf[i];
// 			StartTimeout = 0;																// we got something.  PacketTimeout wait only from now on.

// 			Serial.print("dataLen & data: ");
// 			Serial.print(*buf);
// 			Serial.print("; ");
// 			printBufAsChar(buf, 1, *(buf + 0));
// 			Serial.println();
// 			Serial.flush();

// 			//now load buf pointer to bufArray			
// 			*(bufArray + bufArrayIdx) = buf;			
// 			bufArrayIdx++;

// 		} else {
// 			if (StartTimeout == 0) {
// 				Serial.println("recvMessage timeout 1");
// 				result = false;
// 				break;
// 			}	else if ((millis() - waitStartTime) > StartTimeout) {
// 				Serial.println("recvMessage timeout 2");
// 				result = false;	
// 				break;
// 			}	
// 		}
		
// 	}

// 	Serial.print("msgLen: ");
// 	Serial.println(*msgLen);
// 	Serial.flush();

// 	//now consolidate packets into one
// 	*msg = malloc(*msgLen);
// 	uint16_t msgIdx = 0;
// 	for (uint8_t i = 0; i < bufArrayIdx; i++)	{
// 		uint8_t bufLen = *(*(bufArray + i) + 0);		//get the length of each at the zero index

// 		Serial.print("\nmsgIdx: ");
// 		Serial.print(msgIdx);
// 		Serial.print("; adding: ");
// 		printBufAsChar(*(bufArray + i), 1, *(*(bufArray + i) + 0));
// 		Serial.println();
// 		Serial.println(*(*(bufArray + i) + **(bufArray + i)));		//this should print the decimal value of the last character 

// 		memcpy((*msg) + msgIdx, (*(bufArray + i) + 1), bufLen);
// 		msgIdx += bufLen;

// 		Serial.print("Total message thus far: ");
// 		printBufAsChar(*msg, 0, msgIdx);
// 		Serial.println();
// 		Serial.flush();


// 		// for (uint8_t j = 1; j < bufLen; j++) {
// 		// 	*(msg + msgIdx) = *(*(bufArray + i) + j);
// 		// 	msgIdx++;
// 		// }

// 		//finished with memory now release it
// 		free(*(bufArray + i));
// 	}
// 	free(bufArray);

// 	return result;
// }

bool ACReliableMessage::init(float Frequency, uint16_t AckTimeout, int8_t TxPower, uint8_t Retries, uint8_t ThisAddress) {
	if (RHDatagram::init()) {
		RH_RF95 *myDriver;
		setTimeout(AckTimeout);
		setRetries(Retries);
		setThisAddress(ThisAddress);
		myDriver = (RH_RF95 *)&_driver;
		myDriver->setFrequency(Frequency);
		myDriver->setTxPower(TxPower);
		return true;
	}
	return false;
}
