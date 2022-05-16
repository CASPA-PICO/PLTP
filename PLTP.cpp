#include "PLTP.h"

PLTP::PLTP(bool base){
	this->base = base;
	serialBT = new BluetoothSerial();
	currentState = WaitingBegin;
	lastMessage.content = nullptr;
	lastMessage.size = 0;
}

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
		serialBT->connect("CASPA-PICO-Sensor");
		currentState = Connecting;
		if(!serialBT->connected(30*1000)){
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
		serialBT->begin("CASPA-PICO-Sensor");
		currentState = Connecting;
		if(!serialBT->connected(30*1000)){
#ifdef DEBUG_PLTP
			Serial.println("Failed to connected to CASPA-PICO-Base");
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

bool PLTP::processMessage(int timeout) {
	unsigned long expire = millis()+timeout;
	uint8_t messageHeader[39];
	uint8_t *messageBody = nullptr;
	int receivedCount = 0;
	int bodyLength;
	uint8_t messageChecksum[32];
	uint8_t *messageChecksumCalculated;
	int tries = 0;
	//Timeout in ms
	while(serialBT->connected() && (millis() < expire || tries < 3)){
		esp_task_wdt_reset();
		if(serialBT->available()){
			expire = millis() + timeout;
			if(receivedCount < 39){
				//Receving header
				receivedCount += serialBT->readBytes(messageHeader+receivedCount, 39-receivedCount);
				if(receivedCount == 39){
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
			else if(receivedCount-39 < bodyLength){
				receivedCount += serialBT->readBytes(messageBody+receivedCount-39, bodyLength-receivedCount+39);
			}

			if(receivedCount > 39 &&  receivedCount-39 >= bodyLength){
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

bool PLTP::sendTime() {
	long currentTime = time(nullptr);
	uint8_t data[5];
	data[0] = MES_TIME;
	for(int i=0;i<4;i++){
		data[1+i] = (currentTime >> i * 8) & 0xFF;
	}
	return sendData(data, 5, 3);
}

bool PLTP::sendFile(File *file){
	if(file->isDirectory() || !file->available()){
		return false;
	}
	//Sending file header
	uint8_t buffer[PACKET_SIZE];
	buffer[0] = MES_FILE_INFO;
	int fileSize = file->size();
	memcpy(buffer+1, &fileSize, 4);
	buffer[5] = SECTION_BREAK;
	strcpy((char*)buffer+6, file->name());
	if(!sendData(buffer, 6+strlen(file->name())+1)){
		return false;
	}

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

bool PLTP::fullMessageReceived(){
	/*if(sizeMes == 0 && currentlyReceiving == 0){
		return false;
	}else{
		if(sizeMes == currentlyReceiving){
			Serial.printf("PLTP::fullMessageReceived -> OK!\n");
			return true;
		}
		return false;
	}*/
}

std::vector<uint8_t> PLTP::getCurrentTime(){/*
	std::vector<uint8_t> tmp = currentTime;
	currentTime.clear();
	return tmp;*/
}

std::vector<uint8_t> PLTP::getTimeAsVect(char* currentTime){
	#ifdef DEBUG_PLTP
    Serial.printf("PLTP::getTimeAsVect -> Current Time : %s\n", currentTime);
	#endif
	std::vector<uint8_t> vectTime;
	vectTime.push_back(START_TIME);
	for(int i = 0; i < (strlen(currentTime)-1); i++){
		vectTime.push_back(currentTime[i]);
	}
	return vectTime;
}

std::vector<uint8_t> PLTP::getLenAsVect(int size){
	char* strSize = new char[50];
	sprintf(strSize, "%d", size);
	#ifdef DEBUG_PLTP
	Serial.printf("PLTP::getLenAsVect -> Size = %d\n", size);
	#endif
	std::vector<uint8_t> vectSize;
	vectSize.push_back(START_LEN);
	for(int i = 0; i < strlen(strSize); i++){
		vectSize.push_back(strSize[i]);
	}
	return vectSize;
}


bool PLTP::sendFullMessage(std::vector<uint8_t> fullMessage, char* currentTime){/*
	// Sending first the current time
	std::vector<uint8_t> vectCurrentTime = getTimeAsVect(currentTime);
	#ifdef DEBUG_PLTP
	Serial.printf("PLTP::SendFullMessage -> On génère le currentTime : \n");
	for(int i = 0; i < vectCurrentTime.size(); i++){
		Serial.printf("%c ", vectCurrentTime[i]);
	}
	Serial.printf("\n");
	#endif
	sendData(vectCurrentTime);

	// Now sending the total length of the message to know where are we at and to allocate the right amount of memory on the other device
	int size = (fullMessage.size()/PACKET_SIZE)+1;
	std::vector<uint8_t> vectMessageLen = getLenAsVect(size);
	#ifdef DEBUG_PLTP
	Serial.printf("PLTP::SendFullMessage -> Message longueur généré =  ");
	for(int i = 0; i < vectMessageLen.size(); i++){
		Serial.printf("%c ", vectMessageLen[i]);
	}
	Serial.printf("\n");
	#endif
	sendData(vectMessageLen);

	// Now we can finally send the message divided by PACKET_SIZE bytes messages (max bluetooth length)
	std::vector<uint8_t>* dividedMes = new std::vector<uint8_t>[size];
	dividedMes = divMes(fullMessage);
	#ifdef DEBUG_PLTP
	Serial.printf("PLTP::SendFullMessage -> envoi des %d messages", size);
	#endif
	for(int i = 0; i < size; i++){
		sendData(dividedMes[i]);
	}
	delete [] dividedMes;*/
}

bool PLTP::sendData(uint8_t *data, unsigned int dataSize, unsigned int tries) {
	if(!serialBT->connected()){
		currentState = WaitingBegin;
		return false;
	}

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
	for(int attempts = 0;attempts < tries && serialBT->connected() ; attempts++){
#ifdef DEBUG_PLTP
		Serial.println("Sending header + "+String(dataSize)+" bytes of data (attempt "+String(attempts)+")");
#endif
		serialBT->write(header, 39);
		serialBT->write(data, dataSize);
		serialBT->flush();
		esp_task_wdt_reset();
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

/* 	Each time we send data we send 2 messages, the data itself and a Checksum right after
	Then we wait for the OK to arrive else we send it again till OK arrives
	Returns 1 if mess has been received, 0 if failed (more than 5 tries by default) */
/*bool PLTP::sendData(std::vector<uint8_t> _data, unsigned int maxAttempts){
	// Format message to add start_mes and end_mes
	std::vector<uint8_t> message;
	message.push_back(START_MES);
	for(int i = 0; i < _data.size(); i++){
		message.push_back(_data[i]);
	}
	message.push_back(END_MES);

	// Getting checksum from data
	std::vector<byte> checksum;
	checksum = getChecksum(_data, false);

	#ifdef DEBUG_PLTP
	Serial.printf("PLTP::sendData -> On est entré dans sendData avec mes = ");
	for(int i = 0; i < message.size(); i++){
		Serial.printf("%c ", message.at(i));
	}
	Serial.printf("\n");
	#endif

	//TODO: Peut être problème de validation du message précédent au lieu du nouveau -> Gros délai pour contrer ça pour le moment
	unsigned int nbAttempts = 0;
	do{
		SerialBT.write(&message[0], message.size());
		vTaskDelay(pdMS_TO_TICKS(2000)); // Waiting long enough for other device to process :3
		SerialBT.write(&checksum[0], 34); // Checksum is always 32 bytes + prefix and suffix
		vTaskDelay(pdMS_TO_TICKS(2000));
		if(MES_OK == SerialBT.read()){
			#ifdef DEBUG_PLTP
			Serial.printf("PLTP::sendData -> Message bien reçu!\n");
			#endif
			return 1;
		}
		#ifdef DEBUG_PLTP
		Serial.printf("PLTP::sendData -> Message toujours pas reçu...\n");
		#endif
	}while(nbAttempts < maxAttempts);
	return 0;
}*/

std::vector<uint8_t>* PLTP::divMes(std::vector<uint8_t> data){
	size_t mes_size = data.size();
	int size = mes_size/PACKET_SIZE;
	size++;
	#ifdef DEBUG_PLTP
	Serial.printf("PLTP::divMes -> Taille = %d  = %d\n", mes_size, size);
	#endif
	std::vector<uint8_t>* ret = new std::vector<uint8_t>[size];
	for(int i = 0; i < size ; i++){
		if((i+1) == size){
			ret[i].assign(data.begin()+(i*PACKET_SIZE*sizeof(uint8_t)), data.end());
		}else{
			ret[i].assign(data.begin()+(i*PACKET_SIZE*sizeof(uint8_t)), data.begin()+((i+1)*PACKET_SIZE*sizeof(uint8_t)));
		}
		#ifdef DEBUG_PLTP
		Serial.printf("PLTP::divMes -> Size = % i = %d\n", size, i);
		for(int j = 0; j < ret[i].size(); j++){
			Serial.printf("%c ", ret[i].at(j));
		}
		Serial.printf("\n");
		#endif
	}
	return ret;
}

unsigned int intPow10(uint8_t i){
	if(i == 0){
		return 1;
	}
	if(i == 1){
		return 10;
	}else{
		return intPow10(i-1)*10;
	}
}

unsigned int getLenAsUInt(std::vector<uint8_t> len){
	unsigned int ret = 0;
	for(int i = (len.size()-1); i > 0; i--){
		ret += intPow10(len.size()-i-1)*(len[i] - '0'); // to convert to int value instead of ascii
	}
	return ret;
}

double PLTP::getProgress(){
	/*if(sizeMes == 0)
		return 0;
	return (double)(currentlyReceiving+1) / (double)sizeMes;*/
}

void PLTP::processMessage(std::vector<uint8_t> message){
	/*#ifdef DEBUG_PLTP
	Serial.printf("PLTP::processMessage -> Message d'entrée (nb): ");
	for(int i = 0; i < message.size(); i++){
		Serial.printf("%d ", message[i]);
	}
	Serial.printf("\n");

	Serial.printf("PLTP::processMessage -> Message d'entrée (char): ");
	for(int i = 0; i < message.size(); i++){
		Serial.printf("%c ", message[i]);
	}
	Serial.printf("\n");
	#endif
	std::vector<uint8_t> tmp(message.begin() + sizeof(uint8_t), message.end()); // New array without the START_LEN
	switch(message[0]){
		case START_LEN:
			sizeMes = getLenAsUInt(message);
			#ifdef DEBUG_PLTP
			Serial.printf("PLTP::processMessage -> START_LEN! = %d\n", sizeMes);
			#endif
			currentlyReceiving = 0; // We are starting a new message so the counter is equal to zero
			//delete [] fullMessage; // reseting old message
			fullMessage = new std::vector<uint8_t>[sizeMes]; // Allocating a new one with the right size
			break;
		case START_TIME:
			currentTime.clear();
			currentTime = tmp;
			#ifdef DEBUG_PLTP
			Serial.printf("PLTP::processMessage -> START_TIME! currentTime = ");
			for(int i = 1; i < message.size(); i++){
				Serial.printf("%c", currentTime[i-1]);
			}
			Serial.printf("\n");
			#endif
			break;
		default: // default only when the actual data is received so we fill the array we created before.
			fullMessage[currentlyReceiving] = message;
			#ifdef DEBUG_PLTP
			Serial.printf("PLTP::processMessage -> %d Message!\n", currentlyReceiving);
			for(int i = 0; i < fullMessage[currentlyReceiving].size(); i++){
				Serial.printf("%c", fullMessage[currentlyReceiving][i]);
			}
			Serial.printf("\n");
			#endif
			currentlyReceiving++;
			break;
	}*/
}

std::vector<uint8_t> PLTP::getFullMessage(){
	/*std::vector<uint8_t> returnedFullMessage;
	for(int i = 0; i < sizeMes; i++){
		for(int j = 0; j < this->fullMessage[i].size(); j++){
			returnedFullMessage.push_back(this->fullMessage[i][j]);
		}
	}
	return returnedFullMessage;*/
}

bool PLTP::waitData(int t){/*
	static String* checksumReceived = new String;
	static std::vector<uint8_t> messageUINT;
	static std::vector<uint8_t> check;
	vTaskDelay(pdMS_TO_TICKS(20));
	if(CurrentState == Receiving_mes){
		#ifdef DEBUG_PLTP
		Serial.printf("PLTP::waitData -> Message reçu / mes = ");
		for(int i = 0; i< messageReceived->length(); i++){
			Serial.printf("%c ", messageReceived->charAt(i));
		}
		// We save the message before receiving the checksum (messageReceived will be message's checksum and not the message)
		Serial.printf("\nPLTP::waitData -> message received_t = ");
		#endif
		for(int i = 0; i < messageReceived->length(); i++){
			messageUINT.push_back((uint8_t) messageReceived->charAt(i));
			#ifdef DEBUG_PLTP
			Serial.printf("%d ", messageUINT[i]);
			#endif
		}
		#ifdef DEBUG_PLTP
		Serial.printf("\n");
		#endif
		CurrentState = Pending_;
	}else if(CurrentState == Receiving_checksum){
		checksumReceived = messageReceived;
		std::vector<uint8_t> checksumUINT;
		#ifdef DEBUG_PLTP
		Serial.printf("PLTP::waitData -> Checksum received = ");
		#endif
		for(int i = 0; i < checksumReceived->length(); i++){
			checksumUINT.push_back((uint8_t) checksumReceived->charAt(i));
			#ifdef DEBUG_PLTP
			Serial.printf("%d ", checksumUINT[i]);
			#endif
		}
		check = getChecksum(messageUINT, true);
		#ifdef DEBUG_PLTP
		Serial.printf("\nPLTP::waitData -> messageUINT = \n");
		for(int i = 0; i < messageUINT.size(); i++){
			Serial.printf("%c ", messageUINT[i]);
		}
		Serial.printf("\nPLTP::waitData -> checksum généré = \n");
		for(int i = 0; i < check.size(); i++){
			Serial.printf("%d ", check[i]);
		}
		Serial.printf("\n PLTP::waitData -> comparé à reçu:\n");
		for(int i = 0; i < checksumUINT.size(); i++){
			Serial.printf("%d ", checksumUINT[i]);
		}
		Serial.printf("\n");
		#endif
		if(checksumUINT == check){
			#ifdef DEBUG_PLTP
			Serial.printf("\nPLTP::waitData -> Message reçu avec bon checksum!!\n");
			#endif
			SerialBT.write(MES_OK);
			processMessage(messageUINT);
		}
		messageUINT.clear();
		CurrentState = Pending_;
	}*/
}

bool PLTP::receiveMes(){/*
	if(SerialBT.available()){
		unsigned char c = SerialBT.read();
		#ifdef DEBUG_PLTP
		Serial.printf("PLTP::receiveMes -> Received data! = %d  \t | \t %c\n", c,c);
		#endif
		if(c == START_MES){
			CurrentState = Receiving_mes;
			*messageReceived = SerialBT.readStringUntil(END_MES);
			messageReceived->remove(messageReceived->length());
			#ifdef DEBUG_PLTP
			Serial.printf("PLTP::receiveMes -> We'll receive message next... : \n");
			Serial.printf("PLTP::receiveMes -> %s\n", messageReceived);
			#endif
		}else if(c == START_CHECKSUM){
			delete messageReceived;
			messageReceived = new String;
			CurrentState = Receiving_checksum;
			*messageReceived = SerialBT.readStringUntil(END_CHECKSUM);
			#ifdef DEBUG_PLTP
			Serial.printf("PLTP::receiveMes -> Received checksum\n");
			#endif
			//#ifdef DEBUG_PLTP
			//Serial.printf("PLTP::receiveMes -> We'll receive checksum next...\n");
			//for(int i = 0; i < messageReceived->length(); i++){
			//	Serial.printf("%d ", messageReceived->charAt(i));
			//}
			//Serial.printf("\n");
			//#endif
			messageReceived->remove(messageReceived->length());
		}else{
			return false;
		}
		return true;
	}else{
		return false;
	}*/
}

/* Returns a 32 bytes checksum using sha-256 algorithm begins with  START_CHECKSUM and ends with END_CHECKSUM
 *	
 *	Input: 	- string_ = String to get checksum from 
 *			- root = false to add START_CHECKSUM or END_CHECKSUM
 * */
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
	/*Serial.printf("String = ");
	for(int i = 0; i < string_.size(); i++){
		Serial.printf("%c", string_.at(i));
	}
	Serial.printf("\nsha result = ");
	for(int i = 0; i < 32; i++){
		Serial.printf("%d ", shaResult[i]);
	}*/
	/*Serial.printf("\nChecksum dans func =");
	for(auto & elem : *result){
		Serial.printf("%d ", elem);
	}
	Serial.printf("\n");*/
	return shaResult;
}

PLTP::Message PLTP::getLastMessage() {
	return lastMessage;
}