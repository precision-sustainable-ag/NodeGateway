// ACReliableMessage.h
//
// Author: David Anderson
// Copyright (C) 2019 David Anderson
// $Id: ACReliableMessage.h, v 1.0 2019/05/30 $

#ifndef __ACReliableMessage_h
#define __ACReliableMessage_h

#include <RHReliableDatagram.h>
#include <RH_RF95.h>

#define AC_MAX_MSG_PACKET_SIZE		(RH_RF95_MAX_MESSAGE_LEN - 1)           //250
#define AC_MAX_MSG_LENGTH			(AC_MAX_MSG_PACKET_SIZE * 255)							


/////////////////////////////////////////////////////////////////////
/// \class ACReliableMessage ACReliableMessage.h <ACReliableMessage.h>
/// \brief RHReliableDatagram subclass for sending addressed, acknowledged, retransmitted strings of any length up to 64000 bytes.
///
///
class ACReliableMessage : public RHReliableDatagram
{
public:
    /// Constructor. 
    /// \param[in] driver The RadioHead driver to use to transport messages.
    /// \param[in] thisAddress The address to assign to this node. Defaults to 0
    ACReliableMessage(RHGenericDriver& driver, uint8_t thisAddress = 0);


    /// Send the message (in multple packets if necessary) (with retries) and waits for an ack with each packet. 
	/// Returns true if acknowledgements were received for each packet.
	/// Cancels transmission if any packet is not acknowledged and returns false.
    /// Synchronous: any message other than the desired ACK received while waiting is discarded.
    /// Blocks until an ACK is received or all retries are exhausted (ie up to retries*timeout milliseconds).
    /// If the destination address is the broadcast address RH_BROADCAST_ADDRESS (255), the message will 
    /// be sent as a broadcast, but receiving nodes do not acknowledge, and sendtoWait() returns true immediately
    /// without waiting for any acknowledgements.
    /// \param[in] address The address to send the message to.
    /// \param[in] msg message to send (string) -- each character is assumed to be 1 byte in size (ASCII encoding)
    /// \return true if the message was fully delivered.
    bool sendMessage(String* msg, uint8_t to_address);
    bool sendMessage(uint8_t* msg, uint16_t msgLen, uint8_t to_address);

    /// Similar to recvfromAck(), this will block until either a valid message available for this node
    /// or the timeout expires. Starts the receiver automatically.
    /// You should be sure to call this function frequently enough to not miss any messages
    /// It is recommended that you call it in your main loop.
    /// \param[in] msg Location to copy the received message
    /// \param[in] StartTimeout Maximum time to wait in milliseconds for the message to begin
    /// \param[in] PacketTimeout Maximum time to wait in milliseconds for the next packet of message data, after we start streaming data
    /// \param[in] from If present and not NULL, the referenced uint8_t will be set to the SRC address
    /// (not just those addressed to this node).
    /// \return true if a valid message was copied to buf
    bool recvMessage(String *msg, uint32_t StartTimeout, uint16_t PacketTimeout, uint8_t* from_address = NULL);
    bool recvMessage(uint8_t **msg, uint16_t *msgLen, uint32_t StartTimeout, uint16_t PacketTimeout, uint8_t* from_address = NULL);

	bool init(float Frequency, uint16_t AckTimeout, int8_t TxPower, uint8_t Retries, uint8_t ThisAddress);
	
private:
	uint8_t _PacketBuf[256];	// bufer for TX/RX of data.  Only one operation at a time.
};


void printBufAsChar(uint8_t* buf, uint16_t idx, uint16_t len);


#define CMDFLAG_QUERY           1
#define CMDFLAG_WRITE           2
// #define CMDFLAG_             4
// #define CMDFLAG_             8
// #define CMDFLAG_             16
// #define CMDFLAG_             32
// #define CMDFLAG_             64
#define CMDFLAG_HAVEREQUEST     128



#endif

