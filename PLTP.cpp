#include "PLTP.h"

/**
 * Constructeur du PLTP
 * @param base : True si l'appareil est la base ou False si l'appareil est le capteur
 */
PLTP::PLTP(bool base){
	this->base = base;
	serialBT = new BluetoothSerial();
	serialBT->setTimeout(CONNECTION_TIMEOUT);
	currentState = WaitingBegin;
	lastMessage.content = nullptr;
	lastMessage.size = 0;
}

/**
 * Démarre la connexion Bluetooth
 * @return True si la connexion est effectuée, False en cas d'échec
 */
bool PLTP::begin() {
	if(base){
#ifdef DEBUG_PLTP
		Serial.println("PLTP : begin as base");
#endif
		if(!serialBT->begin("CASPA-PICO-Base", true)){
#ifdef DEBUG_PLTP
			Serial.println("Failed to begin serialBT !");
#endif
			return false;
		}
#ifdef DEBUG_PLTP
		Serial.println("PLTP : base begin success, connecting to sensor...");
#endif
		currentState = Connecting;
		serialBT->connect("CASPA-PICO-Sensor");
		if(!serialBT->connected(CONNECTION_TIMEOUT)){
#ifdef DEBUG_PLTP
			Serial.println("Failed to connected to CASPA-PICO-Sensor");
#endif
			currentState = WaitingBegin;
			serialBT->end();
			return false;
		}
		currentState = ConnectedPendingData;
		return true;
	}
	else{
#ifdef DEBUG_PLTP
		Serial.println("PLTP : begin as sensor");
#endif
		if(!serialBT->begin("CASPA-PICO-Sensor", false)){
#ifdef DEBUG_PLTP
			Serial.println("Failed to begin PLTP as sensor !");
#endif
		}
		currentState = Connecting;
		unsigned long expire = millis() + CONNECTION_TIMEOUT;
		while(!serialBT->connected(CONNECTION_TIMEOUT) && expire > millis()){
			vTaskDelay(pdMS_TO_TICKS(200));
		}
		if(!serialBT->connected(CONNECTION_TIMEOUT)){
#ifdef DEBUG_PLTP
			Serial.println("Failed to connect to CASPA-PICO-Base");
#endif
			currentState = WaitingBegin;
			serialBT->end();
			return false;
		}
		currentState = ConnectedPendingData;
		return true;
	}
}

void PLTP::end() {
	serialBT->end();
	if(lastMessage.content != nullptr){
		delete[] lastMessage.content;
	}
	lastMessage.content = nullptr;
}

/**
 * Reçoit un message
 * @param timeout : délai maximum pour attendre la réception du message (en millisecondes)
 * @return True si un message a bien été reçu, False si aucun message n'a été reçu
 */
bool PLTP::processMessage(int timeout) {
	unsigned long expire = millis()+timeout;
	uint8_t messageHeader[39];
	uint8_t *messageBody = nullptr;
	int receivedCount = 0;
	int bodyLength;
	uint8_t messageChecksum[32];
	uint8_t *messageChecksumCalculated;
	int tries = 0;
	while(serialBT->connected() && (millis() < expire || tries < 3)){
		esp_task_wdt_reset();
		if(serialBT->available()){
			expire = millis() + timeout;
			//Si on a reçu moins de 39 octets -> on est toujours en train de recevoir le header du message
			if(receivedCount < 39){
				receivedCount += serialBT->readBytes(messageHeader+receivedCount, 39-receivedCount);
				if(receivedCount == 39){
					//Si le header est mal formé, on l'ignore
					if(messageHeader[0] != SECTION_BREAK || messageHeader[5] != SECTION_BREAK || messageHeader[38] != SECTION_BREAK){
						receivedCount = 0;
						memset(messageHeader, 0, 39);
						bodyLength = 0;
					}
					else{
						memcpy(&bodyLength, messageHeader+1, 4);
						memcpy(messageChecksum, messageHeader+6, 32);
#ifdef DEBUG_PLTP
						Serial.println("Got message header ! Message length : "+String(bodyLength));
#endif
						if(messageBody != nullptr){delete[] messageBody;}
						if(bodyLength > PACKET_SIZE){
#ifdef DEBUG_PLTP
							Serial.println("Message too long !! Abording...");
#endif
							return false;
						}
						messageBody = new uint8_t[bodyLength];
					}
				}
			}
			//Sinon on lit le corps du message
			else if(receivedCount-39 < bodyLength){
				receivedCount += serialBT->readBytes(messageBody+receivedCount-39, bodyLength-receivedCount+39);
			}

			//Si on a reçu le message au complet
			if(receivedCount > 39 && receivedCount-39 >= bodyLength){
				//On calcule le checksum du corps du message et on le compare avec le checksum passé en entête
				messageChecksumCalculated = getChecksum(messageBody, bodyLength);
				bool checksumOK = true;
				for(int i=0;i<32;i++){
					if(messageChecksum[i] != messageChecksumCalculated[i]){
						checksumOK = false;
						break;
					}
				}
				delete[] messageChecksumCalculated;

				if(checksumOK){
#ifdef DEBUG_PLTP
					Serial.println("Checksum OK !");
#endif
					serialBT->write(MES_OK);
					serialBT->flush();
					if(lastMessage.content != nullptr){delete[] lastMessage.content;}
					lastMessage.content = messageBody;
					messageBody = nullptr;
					lastMessage.size = bodyLength;
					switch(lastMessage.content[0]){
						case MES_TIME:
#ifdef DEBUG_PLTP
							Serial.println("Received MES_TIME !");
#endif
							lastMessage.type = Time;
							break;
						case MES_FILE_INFO:
#ifdef DEBUG_PLTP
							Serial.println("Received MES_FILE_INFO !");
#endif
							lastMessage.type = FileInfo;
							break;
						case MES_FILE_CONTENT:
#ifdef DEBUG_PLTP
							Serial.println("Received MES_FILE_CONTENT !");
#endif
							lastMessage.type = FileContent;
							break;
					}
					return true;
				}
				else{
					delete[] messageBody;
					messageBody = nullptr;
					if(tries < 3){
#ifdef DEBUG_PLTP
						Serial.println("Checksum NOT OK trying again !");
#endif
						serialBT->write(MES_NOT_OK);
						serialBT->flush();
						bodyLength = 0;
						receivedCount = 0;
						tries++;
						expire = millis() + timeout;
					}
					else{
#ifdef DEBUG_PLTP
						Serial.println("Checksum NOT OK exiting !");
#endif
						return false;
					}
				}
			}
		}
		else{
			vTaskDelay(pdMS_TO_TICKS(10));
			if(millis() > expire && receivedCount > 0 && tries < 3){
#ifdef DEBUG_PLTP
				Serial.println("PLTP : receive timeout but got some data, trying again");
#endif
				serialBT->write(MES_NOT_OK);
				serialBT->flush();
				delete[] messageBody;
				messageBody = nullptr;
				bodyLength = 0;
				receivedCount = 0;
				tries++;
				expire = millis() + timeout;
			}
		}
	}
#ifdef DEBUG_PLTP
	Serial.println("No message received ! (timeout or disconnected)");
#endif
	if(messageBody != nullptr){delete[] messageBody;}
	return false;
}

/**
 * Envoie de l'heure
 * @return True si l'heure a bien été transmise, False en cas d'échec
 */
bool PLTP::sendTime() {
	long currentTime = time(nullptr);
	uint8_t data[5];
	data[0] = MES_TIME;
	for(int i=0;i<4;i++){
		data[1+i] = (currentTime >> i * 8) & 0xFF;
	}
	return sendData(data, 5, 3);
}

/**
 * Envoie d'un fichier
 * @param file : fichier à envoyer
 * @return True si le fichier a bien été envoyé, False en cas d'échec
 */
bool PLTP::sendFile(File *file){
	if(file->isDirectory() || !file->available()){
		return false;
	}
	//On envoie l'entête du fichier
	uint8_t buffer[PACKET_SIZE];
	buffer[0] = MES_FILE_INFO;
	int fileSize = file->size();
	memcpy(buffer+1, &fileSize, 4);
	buffer[5] = SECTION_BREAK;
	strcpy((char*)buffer+6, file->name());
	if(!sendData(buffer, 6+strlen(file->name())+1)){
		return false;
	}

	//On envoie le fichier packet par packet
	int totalRead = 0;
	int lastRead = 0;
	buffer[0] = MES_FILE_CONTENT;
	while(totalRead < file->size()){
		lastRead = file->readBytes((char*)buffer+1, PACKET_SIZE-1);
		Serial.println("Read bytes from file : "+String(lastRead));
		if(!sendData(buffer, lastRead+1, 3)){
			return false;
		}
		totalRead += lastRead;
	}

	return true;
}

/**
 * Envoie de données
 * @param data : tableau des octets à envoyer
 * @param dataSize : taille du tableau des octets à envoyer
 * @param tries : nombre de tentatives avant abandon
 * @return True si les données ont bien été envoyés, False en cas d'échec
 */
bool PLTP::sendData(uint8_t *data, unsigned int dataSize, unsigned int tries) {
	if(!serialBT->connected()){
		currentState = WaitingBegin;
		return false;
	}

	/* On créé l'entête du packet
	 * On a dans l'odre :
	 * SECTION_BREAK + 4 octets de taille du packet + SECTION_BREAK + 32 octets de checksum des données
	 */
	uint8_t header[1+4+1+32+1];
	header[0] = SECTION_BREAK;
	for(int i=0;i<4;i++){
		header[1+i] = ((dataSize >> i*8) & 0xFF);
	}
	header[5] = SECTION_BREAK;
	uint8_t *checksum = getChecksum(data, dataSize);
	memcpy(&header[6], checksum, 32);
	delete[] checksum;
	header[38] = SECTION_BREAK;


	while(serialBT->available()){serialBT->read();} //Flush available data

	//On transmet le packet de données
	for(int attempts = 0;attempts < tries && serialBT->connected() ; attempts++){
#ifdef DEBUG_PLTP
		Serial.println("Sending header + "+String(dataSize)+" bytes of data (attempt "+String(attempts)+")");
#endif
		serialBT->write(header, 39);
		serialBT->write(data, dataSize);
		serialBT->flush();
		esp_task_wdt_reset();

		//Une fois les données envoyés on attend la réponse pour savoir si elles ont bien été transmise ou non
		unsigned long timeout = millis() + 30*1000;
		while(!serialBT->available() && timeout > millis()){
			vTaskDelay(pdMS_TO_TICKS(10));
		}
		if(serialBT->available() && serialBT->read() == MES_OK){
#ifdef DEBUG_PLTP
			Serial.println("Data received OK !");
#endif
			return true;
		}
		else{
#ifdef DEBUG_PLTP
			Serial.println("Data received NOT OK !");
#endif
		}
		while(serialBT->available()){serialBT->read();}
	}

	return false;
}

/**
 * Renvoie un checksum sur 32 octets en utilisant l'algorithm SHA-256
 * @param data : données pour le calcul du checkum
 * @param dataSize : taille des données pour le calcul du checksum
 * @return Retourne un tableau de 32 octets contenant le checksum
 */
uint8_t *PLTP::getChecksum(uint8_t *data, unsigned int dataSize) {
	uint8_t *shaResult = new uint8_t[32];
	mbedtls_md_context_t ctx;
	mbedtls_md_type_t md_type = MBEDTLS_MD_SHA256;
	mbedtls_md_init(&ctx);
	mbedtls_md_setup(&ctx, mbedtls_md_info_from_type(md_type), 0);
	mbedtls_md_starts(&ctx);
	mbedtls_md_update(&ctx, data, dataSize);
	mbedtls_md_finish(&ctx, shaResult);
	mbedtls_md_free(&ctx);
	return shaResult;
}

PLTP::Message PLTP::getLastMessage() {
	return lastMessage;
}