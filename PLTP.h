#pragma once
#include "BluetoothSerial.h"
#include "FS.h"
#include "mbedtls/md.h"
#include <sys/time.h>
#include <vector>
#include <esp_task_wdt.h>
// El famoso PLTP PL Transmission protocol

#define SECTION_BREAK 255
#define MES_TIME 1
#define MES_FILE_INFO 2
#define MES_FILE_CONTENT 3
#define START_MES 248
#define END_MES 249
#define START_CHECKSUM 250
#define END_CHECKSUM 251
#define MES_OK 252
#define MES_NOT_OK 247
#define START_TIME 253
#define START_LEN 254
#define PACKET_SIZE 256
#define DEBUG_PLTP // To activate debuging infos or not with Serial

typedef enum PLTP_State{
	Say_Time, Wait_Time, How_long, mlength, s_data, data_response, End, Receiving_mes, Receiving_checksum, Pending_
}PLTP_State;

class PLTP{
		// Formatting functions
		std::vector<uint8_t> getTimeAsVect(char* currentTime);
		std::vector<uint8_t> getLenAsVect(int size);

		uint8_t *getChecksum(uint8_t *data, unsigned int dataSize); // returns the message current checksum
		void processMessage(std::vector<uint8_t> message);

	public:
		enum MessageType{Time, FileInfo, FileContent};
		enum State{WaitingBegin, Connecting, ConnectedPendingData};
		typedef struct{MessageType type; uint8_t *content; int size;} Message;

		PLTP(bool base); /* true for base */
		bool begin();
		void end();
		bool sendTime();
		bool sendFile(File *file);
		bool processMessage(int timeout);

		Message getLastMessage();

		bool sendFullMessage(std::vector<uint8_t> fullMessage, char* currentTime);
		bool receiveMes();
		bool sendData(uint8_t *data, unsigned int dataSize, unsigned int tries = 5);
		bool waitData(int t);
		std::vector<uint8_t> getCurrentTime(); // Returns the current time then reset it so we don't use it multiple times
		std::vector<uint8_t>* divMes(std::vector<uint8_t> data); // Divide the message with 330bytes mes max
		double getProgress(); // Returns a number between 0 and 1 depending on how advanced we are on the download
		std::vector<uint8_t> getFullMessage(); // Returns the full message don't forget to free afterwards
		bool fullMessageReceived(); // Is the message fully received
	private:
		State currentState;
		BluetoothSerial *serialBT;
		bool base; // Is the program running on base or sensor (receive or send data)

		Message lastMessage;
};


// Just random functions I rewrote (instead of std ones) to make things easier
unsigned int intPow10(uint8_t i);
unsigned int getLenAsUInt(std::vector<uint8_t> len); 

