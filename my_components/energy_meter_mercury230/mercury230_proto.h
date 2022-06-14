#ifndef MERCURY230_PROTO_H
#define MERCURY230_PROTO_H

#define PASS_1 "\1\1\1\1\1\1"
#define PASS_2 "\2\2\2\2\2\2"

#define PACKET_MIN_DELAY 500
#define ABORT_RECIVE_TIME 50
#define MIN_SCAN_PERIOD 2000

//коды ошибок
enum _replyReason:uint8_t { REP_OK=0, //Все нормально
                            ERROR_COMMAND=1, //Недопустимая команда или параметр
                            ERROR_HARDWARE=2,//Внутренняя ошибка счетчика
                            ERROR_ACCESS_LEVEL=3,//Недостаточен уровень для удовлетворения запроса
                            ERROR_CORE_TIME=4, //Внутренние часы счетчика уже корректировались в течение текущих суток
                            ERROR_CONNECTION=5, //Не открыт канал связи
                            ERROR_TIMEOUT=6, //Ошибка ответа, ошибка КС
                            BUFFER_OVERFLOW=7 // переполнениебуфера
};

//типы функций обратного вызова
typedef void (*callBack1_t)(float);
typedef void (*callBackStr_t)(char*);
typedef void (*callBack2_t)(float,float);
typedef void (*callBack3_t)(float,float,float);
typedef void (*callBack4_t)(float,float,float,float);
typedef void (*debug_t)(uint8_t, uint8_t*);

// инициализатор
// _addr - адрес прибора
// scanPeriod - период сканирования
// _admin - уровень доступа
void setupMerc(uint8_t _addr=0, uint32_t _scanPeriod=1000, bool _admin=false);
uint8_t availableMerc(); // обслуживание цикла/проверка наличия данных для отправки
uint8_t getByteForMerc(); // получение байта из буфера для отправки
uint8_t* getBuffForMerc(); // получение указателя на массив для отправки
_replyReason getFromMerc(uint8_t d);// входящие от счетчика данные пихаем сюда
void setUpdatePeriod(uint32_t period); // установка периода опросов

// для установки получения реальных данных, call back методом
void setCbPower(callBack4_t func);// будет вызвана при чтении из счетчика мощности
void setCbVolt(callBack3_t func);// будет вызвана при чтении из счетчика напряжения
void setCbCurrent(callBack3_t func);// будет вызвана при чтении из счетчика тока
void setCbKoef(callBack3_t func);// будет вызвана при чтении коэфициентов из счетчика мощности
void setCbAngles(callBack3_t func);// будет вызвана при чтении фазовых сдвигов
void setCbFreq(callBack1_t func);// будет вызвана при чтении частоты
void setCbValues(callBack2_t func); //будет вызвана при чтении напряжений

// kалбэки для отладки возвращают данные в момент начала отправки пакета или в момент окончания приема
void setCbDebug(debug_t in, debug_t out);

// для получения по запросу
char* readVersion();//версия прибора
char* readSerial();//серийник прибора
char* readFabData();//дата изготовления
uint16_t readCRC(); //CRC прибора
_replyReason getLastError();// возврат ошибок
char* getStrError(_replyReason lastError);// расшифровка ошибок 

//======================================================================================================
// ДАЛЕЕ СОДЕРЖИМОЕ CPP
//======================================================================================================

// типы пакетов по теме
enum _packetType:uint8_t { _OK=0,       // пакет проверки связи, используется только при тесте или пинге
                           CONNECT=1,    // установка конекта
                           CLOSE=2,      // закрытие конекта
                           WRITE=3,      // запись
                           READ=4,       // чтение параметров 
                           LIST=5,       // чтение журналов
                           READ_PARAMS=8 // чтение доп параметров 
};

// тип пакета в буфере отправки
enum _currentSend:uint8_t { NONE=0, //в буфере нет пакета
                            GET_TEST,
                            GET_ACCESS,
                            WRITE_TIME,
                            CORE_TIME,
                            GET_TIME,
                            GET_POWER,
                            GET_VOLTAGE,
                            GET_CURRENT,
                            GET_KOEF_POWER,
                            GET_FREQ,
                            GET_ANGLE_PH,
                            GET_DISTORTION,
                            GET_TEMP,
                            GET_LINEAR_VOLTAGE,
                            GET_VERS,
                            GET_SER_NUM,
                            GET_TIME_CODE,
                            GET_CRC,
                            GET_VALUE,
                            GET_ADDR
};

// тип запроса чтения параметров
enum _reqType:uint8_t { PARAM_SER_NUM  = 0 ,    // серийный номер и дату
                        PARAM_VERS     = 3,     // версия
                        PARAM_UNO      = 0x11,  // читаем один конкретный параметр, НЕ БУДУ ИСПОЛЬЗОВАТЬ
                        PARAM_ALL_FULL = 0x14,  // ответ по всем фазам, списком, без сокращения незначащих битов
                        PARAM_ALL      = 0x16,  // ответ по всем фазам, списком, в сокращенном формате, при запросе указывать  фазу 1(!!!)
                        PARAM_CRC      = 0x26   // читаем CRC прибора
};

//================== ИСХОДЯЩИЕ ПАКЕТЫ ========================

// общий буфер отправки
struct _sBuff{
    uint8_t addr; // адрес счетчика
    _packetType packType; // тип пакета
    uint8_t data[30]; // тело буфера
};

// дебильный формат меркурия, программист дятел.
uint32_t dm32_3(uint8_t* d){
   return (((((uint32_t)(d[0]&0x3f))<<8)+d[2])<<8)+d[1];  
}
uint32_t dm32_4(uint8_t* d){
   return (((((((uint32_t)d[1])<<8)+d[0])<<8)+d[3])<<8)+d[2];  
}


//============================ ПЕРЕМЕННЫЕ =======================
uint8_t readBuff[32] = {0}; // буфер входящих данных
uint8_t fromReadArrow = 0; // указатель точки заполнения буфера приема
uint8_t sendBuff[32] = {0}; // буфер отправки
_sBuff* sBuff = (_sBuff*)sendBuff; // фантом буфера отправки
uint8_t forSendSize = 0; // количество данных в буфере отправки
uint8_t forSendArrow = 0; // указатель на очередной байт в буфере для отправки 
_currentSend forSenfType = NONE; // тип пакета в буфере отправки, что ждем от пакета приема
uint8_t addr=0; // адрес счетчика
bool admin=false; // c каким уровнем доступа работаем
uint32_t scanPeriod = 0xFFFFFFFF; // период сканирования
uint32_t timeReadByte=0; //тут время последнего получения байта
uint32_t timeSendByte=0; //время последней отправки байта
char Version[15]={0}; //версия прибора
char SerNum[15]={0}; //серийник прибора
char FabricData[15]={0}; //датавыпуска 
uint16_t mainCRC=0; // CRC прибора
bool procError=false; // если во время цикла опроса возикнет ошибка
bool waiteReply = false; // флаг ожидания ответа, поднимается при отправке, снимается при получении
_replyReason lastError=REP_OK; // последний статус ответа
uint32_t scanTimer=millis(); // таймер периодов связи

// адреса обратных вызовов получения параметров
callBack4_t cbPower=nullptr;
callBack3_t cbVolt=nullptr;
callBack3_t cbCurrent=nullptr;
callBack3_t cbKoef=nullptr;
callBack3_t cbAngles=nullptr;
callBack1_t cbFreq=nullptr;
callBack2_t cbValues=nullptr;

// калбэки для отладки
debug_t debugIn=nullptr;
debug_t debugOut=nullptr;

//==================== ОБМЕН ДАННЫМИ ===================================
// таблица быстрого рассчета КС
uint16_t crcTable[] = {
        0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
        0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
        0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
        0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
        0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
        0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
        0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
        0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
        0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
        0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
        0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
        0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
        0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
        0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
        0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
        0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
        0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
        0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
        0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
        0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
        0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
        0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
        0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
        0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
        0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
        0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
        0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
        0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
        0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
        0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
        0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
        0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040
};

// для подсчета CS на лету
uint16_t stepCrc16mb(uint8_t in, bool start=false){
   static uint16_t crc=0xFFFF;
   if(start){ // первое обращение
      crc=0xFFFF;     
   }
   crc = ((crc >> 8) ^ crcTable[(crc ^ in) & 0xFF]);
   return crc;
}

// подсчет контрольной суммы сразу в буфере
uint8_t crc16mb(uint8_t *s, uint8_t count) {
    uint16_t* crc=(uint16_t*)(&(s[count-2])); 
    *crc=0xFFFF;
    for(int i = 0; i < count-2; i++) {
        *crc = ((*crc >> 8) ^ crcTable[(*crc ^ s[i]) & 0xFF]);
    }
    if(debugOut!=nullptr){ // показываем буфер для отладки
       debugOut(count, sendBuff);   
    }
    return count;
}

// открытие канала связи, безадресная
void sConnect(){
   sBuff->addr=addr;
   sBuff->packType=_OK;
   forSendSize = crc16mb(sendBuff, 4); // размер пакета
   forSendArrow = 0;
   forSenfType = GET_TEST;
   //esp_log_printf_(ESPHOME_LOG_LEVEL_ERROR, "HALLO", __LINE__, "sConnect");   
}

// запрос доступа, по умолчанию с паролем пользователя
void sAccess(){
   sBuff->addr = addr;
   sBuff->packType=CONNECT;
   if(admin){
     sBuff->data[0]=2; // уровень доступа
     uint8_t pass[]=PASS_2;
     memcpy(sBuff->data+1, pass, sizeof(pass));
   } else {
     sBuff->data[0]=1; // уровень доступа
     uint8_t pass[]=PASS_1;
     memcpy(sBuff->data+1,pass, sizeof(pass));
   }
   forSendSize = crc16mb(sendBuff, 11);
   forSendArrow = 0;
   forSenfType = GET_ACCESS;
   //esp_log_printf_(ESPHOME_LOG_LEVEL_ERROR, "HALLO", __LINE__, "sAccess");   
}

// предварительная подготовка к запросу параметров 6 байт
void _getParam(uint8_t param){
   sBuff->addr = addr;
   sBuff->packType = READ_PARAMS;
   sBuff->data[0]=(uint8_t)PARAM_ALL;
   sBuff->data[1]=param;
   forSendSize = crc16mb(sendBuff, 6); // возврат размера пакета
   forSendArrow = 0;
}

// запрос мощности
void sGetPower(){
   _getParam(0); //мощность P, все фазы
   forSenfType = GET_POWER;
};

// запрос напряжений
void sGetVoltage(){
   _getParam(0x11);// напряжение, для группового запроса указываем первую фазу
   forSenfType = GET_VOLTAGE;
};

// запрос тока
void sGetCurrent(){
   _getParam(0x21);// ток, для группового запроса указываем первую фазу
   forSenfType = GET_CURRENT;
};

// запрос коэфициентов мощности
void sGetKoefPower(){
   _getParam(0x31);// коэффициетны мошности, для группового запроса указываем первую фазу
   forSenfType = GET_KOEF_POWER;
};

// запрос частоты
void sGetFreq(){
   _getParam(0x40); 
   forSenfType = GET_FREQ;
};

//угол между фазными напряжениями
void sGetAnglePh(){
   _getParam(0x51);// углы, для группового запроса указываем первую фазу
   forSenfType = GET_ANGLE_PH;
};

//Чтение параметров устройства
void sGetVers(){
   sBuff->addr = addr;
   sBuff->packType = READ_PARAMS;
   sBuff->data[0]=1;
   forSendSize = crc16mb(sendBuff, 5);
   forSendArrow = 0;
   forSenfType = GET_VERS;
}

//Чтение сетевого адреса
void sGetAddr(){
   sBuff->addr = 0; 
   sBuff->packType = READ_PARAMS;
   sBuff->data[0] = 5; // параметр номер счетчика
   forSendSize = crc16mb(sendBuff, 5);
   forSendArrow = 0;
   forSenfType = GET_ADDR;
   //esp_log_printf_(ESPHOME_LOG_LEVEL_ERROR, "HALLO", __LINE__, "sGetAddr");   
}

// чтение показаний
void sGetValue(){
    sBuff->addr=addr;
    sBuff->packType = LIST;
    sBuff->data[0] =0; // энергия по сумме тарифов
    sBuff->data[1] = 0; // за весь период работы
    forSendSize = crc16mb(sendBuff,6); // возврат размера пакета
    forSendArrow = 0;
    forSenfType = GET_VALUE;
}

// возврат ошибок
_replyReason getLastError(){
  return lastError;
}
char* getStrError(_replyReason lastError){
  static char out[27]={0};
  char* rep=out;
  if((uint8_t)lastError & 0x80){ // это широковещалка
    lastError = (_replyReason)((uint8_t)lastError & 0x7F);
    strcpy(rep, "Broadcast ");
    rep+=10; //размер тега "Broadcast "    
  }
  if(lastError==REP_OK){
     strcpy(rep,"OK");
  } else if (lastError==ERROR_COMMAND){
     strcpy(rep,"Command error");
  } else if (lastError==ERROR_HARDWARE){
     strcpy(rep,"Hardware error");
  } else if (lastError==ERROR_ACCESS_LEVEL){
     strcpy(rep,"Access deny");
  } else if (lastError==ERROR_CORE_TIME){
     strcpy(rep,"Core time forbiden");
  } else if (lastError==ERROR_CONNECTION){
     strcpy(rep,"Connection close");
  } else if(lastError==ERROR_TIMEOUT){
     strcpy(rep,"Timeout error");
  } else if(lastError==BUFFER_OVERFLOW){
     strcpy(rep,"Buffer overflow");
  } else {
     strcpy(rep,"Unexpected");
  }
  return out;
}

// возврат версии в текстовом виде
char* readVersion(){//версия прибора
   return Version;
}
char* readSerial(){//серийник прибора
    return SerNum;
}
char* readFabData(){//дата изготовления
    return FabricData;
}

// возврат CRC прибора
uint16_t readCRC(){
    return mainCRC;
}

// разбор полученного пакета
void parceInbound(){
  if(debugIn!=nullptr){ // показываем буфер для отладки
     debugIn(fromReadArrow, readBuff);   
  }
  if(readBuff[0] == addr || readBuff[0] == 0){ // только если ответ он нашего счетчика
     if(fromReadArrow == 4){ // 4 байта скорее всего это пакет подтверждение
        lastError=(_replyReason)readBuff[1];
        if(lastError!=REP_OK && lastError!=ERROR_CORE_TIME){
            procError=true; // ошибка связи
        }
        if(readBuff[0] == 0){
           lastError=(_replyReason)(readBuff[1] | 0x80);// те же ошибки с флагом широковещалки
        }
        fromReadArrow=0;
     } else if(fromReadArrow == 5){ // 5 байтовый входящий
       if(forSenfType == GET_ADDR){
            addr=readBuff[2]; // ответ на запрос сетевого адреса
            fromReadArrow=0;
            lastError=REP_OK;
       }
     } else if(fromReadArrow == 15){ // пакет 4x3
     if(forSenfType == GET_POWER){ // ответ на запрос энергии
           fromReadArrow=0;
           lastError=REP_OK;
           if(cbPower!=nullptr){
              cbPower((float)dm32_3(readBuff+1)/100, 
                      (float)dm32_3(readBuff+4)/100,
                      (float)dm32_3(readBuff+7)/100,
                      (float)dm32_3(readBuff+10)/100);
           }
        }
     } else if(fromReadArrow == 12){ // пакет 3x3
        if(forSenfType == GET_VOLTAGE){ // ответ на запрос напряжения
           fromReadArrow=0;
           lastError=REP_OK;
           if(cbVolt!=nullptr){
              cbVolt((float)dm32_3(readBuff+1)/100, 
                     (float)dm32_3(readBuff+4)/100,
                     (float)dm32_3(readBuff+7)/100);
           }
        } else if (forSenfType == GET_KOEF_POWER){ // ответ на запрос коэфициентов
           fromReadArrow=0;
           lastError=REP_OK;
           if(cbKoef!=nullptr){
              cbKoef((float)dm32_3(readBuff+1)/100, 
                     (float)dm32_3(readBuff+4)/100,
                     (float)dm32_3(readBuff+7)/100);
           }
        } else if (forSenfType == GET_ANGLE_PH){ // ответ на запрос углов
           fromReadArrow=0;
           lastError=REP_OK;
           if(cbAngles!=nullptr){
              cbAngles((float)dm32_3(readBuff+1)/100, 
                       (float)dm32_3(readBuff+4)/100,
                       (float)dm32_3(readBuff+7)/100);
           }
        } else if (forSenfType == GET_CURRENT){ // ответ на запрос тока
           fromReadArrow=0;
           lastError=REP_OK;
           if(cbCurrent!=nullptr){
              cbCurrent((float)dm32_3(readBuff+1)/1000, 
                        (float)dm32_3(readBuff+4)/1000,
                        (float)dm32_3(readBuff+7)/1000);
           }
        }
     } else if (fromReadArrow == 6){ // 6 байт 
        if (forSenfType == GET_FREQ){ // ответ на запрос частоты
           fromReadArrow=0;
           lastError=REP_OK;
           if(cbFreq!=nullptr){
              cbFreq((float)dm32_3(readBuff+1)/100);
           }
        }
     } else if (fromReadArrow == 19){ // 19 байт
        if (forSenfType == GET_VERS){ // эти параметры калбэком не получаем, они нужены редко
            // получаем серийный номер
            snprintf(SerNum, sizeof(SerNum)-1, "%02d%02d%02d%02d",readBuff[1],readBuff[2],readBuff[3],readBuff[4]);
            // получаем версию
            snprintf(Version, sizeof(Version)-1, "%d.%02d.%02d",readBuff[8],readBuff[9],readBuff[10]);
            // дата выпуска
            snprintf(FabricData, sizeof(FabricData)-1, "%d/%02d/%02d",readBuff[5],readBuff[6],readBuff[7]);
            // остальные байты, вроде как расшифровка модификации счетчика, мне совсем не нужны
            fromReadArrow=0;
            lastError=REP_OK;
        } else if (forSenfType == GET_VALUE){ // показания счетчика Активные, Реактивные
           fromReadArrow=0;
            lastError=REP_OK;
            if(cbValues!=nullptr){
                cbValues((float)dm32_4(readBuff+1)/1000, (float)dm32_4(readBuff+9)/1000);
            }
        }            
     } else if (fromReadArrow == 17){ //17 байт
        //...
     }
  } else { // данные чужого счетчика
     fromReadArrow=0; // сбрасываем данные  
     lastError=REP_OK;
  }
}

void setCbPower(callBack4_t func){// будет вызвана при чтении из счетчика мощности
   cbPower=func;  
}
void setCbVolt(callBack3_t func){// будет вызвана при чтении из счетчика напряжения
   cbVolt=func;
}
void setCbCurrent(callBack3_t func){// будет вызвана при чтении из счетчика тока
   cbCurrent=func;   
}
void setCbKoef(callBack3_t func){// будет вызвана при чтении коэфициентов из счетчика мощности
   cbKoef=func;
}
void setCbAngles(callBack3_t func){// будет вызвана при чтении фазовых сдвигов
   cbAngles=func;  
}
void setCbFreq(callBack1_t func){// будет вызвана при чтении частоты
   cbFreq=func;
}
void setCbValues(callBack2_t func){ //будет вызвана при чтении напряжений
   cbValues=func;
}

void setupMerc(uint8_t _addr, uint32_t _scanPeriod, bool _admin){ // установка начальных параметров
   addr=_addr;
   admin=_admin;
   scanPeriod = _scanPeriod; // корректировка на время обработки
   scanTimer = millis() - _scanPeriod; // инициализация таймера опроса
}

void setUpdatePeriod(uint32_t period){ // изменение периода на лету
   if(period < MIN_SCAN_PERIOD){period = MIN_SCAN_PERIOD;}
   scanPeriod = period;
}

// коннектор исходящих данных
// проверка наличия данных для отправки, за одно цикл обработки
uint8_t availableMerc(){
    uint32_t _now=millis();
    static uint8_t counter=0;

    // КОНТРОЛЬ НЕ ОТВЕТА
    if(waiteReply && _now-timeReadByte>ABORT_RECIVE_TIME){ // отслеживаем таймаут приема байта
        waiteReply = false; 
        if(counter>5){ // поднимаем ошибки только на запросах данных
            lastError=ERROR_TIMEOUT;
        }
        procError=true;  // поднимаем ошибку для повтора цикла инициализации
    }
   
    // если в буфере есть данные - показать их количесто
    uint8_t ret=forSendSize-forSendArrow;
    if(ret){ // если есть данные для отправки
        return ret; //ничего не делаем
    } 
    
    // ЦИКЛ ОБРАБОТКИ
    if(_now-scanTimer>=scanPeriod){ // таймер шиклов опроса  
        if(!waiteReply && _now-timeSendByte>=PACKET_MIN_DELAY){  // ТАЙМЕР МЕЖПАкЕТНОГО ИНТРЕВАЛА, одновременно ждем ответ
            timeSendByte=_now;
            while(counter<=10){
                if      (counter==0){sConnect(); break;} // на нулевом шаге пожимаем руку
                else if (counter==1){sAccess(); break;}  // далее просим доступ
                else if (counter==2 && addr==0){sGetAddr(); counter=11; procError=true; break;} // поиск адреса // пока не найдем дальше не работаем
                else if (counter==3){sGetVers(); break;}     // далее версию, дату изготовления, серийник
                else if (counter==4){if(cbValues!=nullptr){sGetValue();  break;}}   // показания
                else if (counter==5){if(cbPower!=nullptr){sGetPower(); break;}}     // на этом шаге можем считать мощность
                else if (counter==6){if(cbVolt!=nullptr){sGetVoltage(); break;}}    // вольты
                else if (counter==7){if(cbCurrent!=nullptr){sGetCurrent(); break;}} // ток
                else if (counter==8){if(cbKoef!=nullptr){sGetKoefPower(); break;}}  // коэфициенты
                else if (counter==9){if(cbAngles!=nullptr){sGetAnglePh(); break;}}  // углы
                else if (counter==10){if(cbFreq!=nullptr){sGetFreq(); break;}}      // частота
                counter++;
            }
            if(counter++>10){ // зацикливание счетчика
                if(_now-scanTimer>2*scanPeriod){ // еСЛИ ВРЕМЯ ПЕРИОДА прешышает требуемое значительно
                    scanTimer=_now; // то сбрасываем таймер - пофиг регулярность
                } else {
                    scanTimer+=scanPeriod; // так мы учитываем уже отработанное время точно
                }
                if(procError){ // если в цикле была ошибка при связи
                    procError=false;
                    counter=0;  // пройдем весь цикл запросов, возможно снова нужна авторизация
                } else {
                    counter=3; // если без ошибок, то повторной инициализации не нужно, просто опросим параметры
                }
            }
        }
    } else {
        timeReadByte=_now; // для сброса ложной ошибки таймаута получения ответа
    }
    return forSendSize-forSendArrow; // возможно в буфере появились данные
}

//отдача байта счетчику
uint8_t getByteForMerc(){
    if(forSendSize-forSendArrow){ //если буфер не пустой 
        waiteReply = true; 
        uint8_t ret=sendBuff[forSendArrow++];
        if(forSendSize-forSendArrow==0){
           forSendSize=0;
           forSendArrow=0;
        }
        return ret;  
    }
    return 0;
}

//отдача буфера счетчику
uint8_t* getBuffForMerc(){
    uint8_t* ret= &(sendBuff[forSendArrow]);
    forSendArrow=0;
    forSendSize=0;
    return ret;
}

// входящий байт пихаем сюда
_replyReason getFromMerc(uint8_t d){
    uint32_t _now=millis();
    if(_now-timeReadByte>ABORT_RECIVE_TIME){ // таймаут получения данных
       goto reset_buff;  //ошибку не поднимае, нужно только для корректности данных
    } 
    if(fromReadArrow == sizeof(readBuff)){ // переполнение буфера
       procError=true;  // поднимаем ошибку для повтора цикла инициализации
       lastError = BUFFER_OVERFLOW;
    reset_buff:   
       fromReadArrow=0;  // сбрасываем уже не нужные данные
    }
    waiteReply = false; // снимаем флаг ожидания ответа, что то получили
    timeReadByte=_now; // засекаем время получения байта
    readBuff[fromReadArrow++]=d; // кладем байт в буфер
    if(fromReadArrow==1){ // если это начало пакета
       stepCrc16mb(d, true); // начнем счикать КС
    } else if( fromReadArrow <3){ // размер данных еще не достаточен для КС + данные
       stepCrc16mb(d);
    } else { // если получили пакет больше 2 байт+CS
       if(stepCrc16mb(d) == 0){ //вероятный конец данных контрольная сумма обнулилась !!!
          parceInbound();
          availableMerc(); // обслужить буфер для очистки
       }
    }
    return lastError;
}

// настройка отладки
void setCbDebug(debug_t in, debug_t out){
   debugIn=in;
   debugOut=out;
}

#endif //MERCURY230_PROTO_H
