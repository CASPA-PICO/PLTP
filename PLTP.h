#pragma once
#include "BluetoothSerial.h"
#include "FS.h"
#include "mbedtls/md.h"
#include <sys/time.h>
#include <vector>
#include <esp_task_wdt.h>

#define SECTION_BREAK 255
#define MES_TIME 1
#define MES_FILE_INFO 2
#define MES_FILE_CONTENT 3
#define MES_OK 252
#define MES_NOT_OK 247
#define PACKET_SIZE 256

//Timeout de la connexion
#define CONNECTION_TIMEOUT 10*1000

//Affichage des informations de débogage dans la console
#define DEBUG_PLTP

typedef enum PLTP_State{
	Say_Time, Wait_Time, How_long, mlength, s_data, data_response, End, Receiving_mes, Receiving_checksum, Pending_
}PLTP_State;

class PLTP{
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

		bool sendData(uint8_t *data, unsigned int dataSize, unsigned int tries = 5);
	private:
		uint8_t *getChecksum(uint8_t *data, unsigned int dataSize);

		State currentState;
		BluetoothSerial *serialBT;
		bool base;

		Message lastMessage;
};