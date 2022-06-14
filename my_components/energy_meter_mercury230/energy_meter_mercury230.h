
#pragma once

#include <Arduino.h>
#include "esphome.h"
#include <stdarg.h>
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"
#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "mercury230_proto.h"

#define BLUE_LED_PIN 16  //GPIO16 
#define RED_LED_PIN 4    //GPIO04
 
//namespace esphome {
//namespace energy_meter_mercury230 {

// Буфера передачи данных колбэком (хотел как лучше, а вышло как всегда)    
static float _Pa; 
static float _Pb; 
static float _Pc; 
static float _Psumm;
static bool powerReady=false;

static float _Va; 
static float _Vb; 
static float _Vc; 
static bool voltsReady=false;

static float _Ca; 
static float _Cb; 
static float _Cc; 
static bool currentReady=false;

static float _Aa;
static float _Ab;
static float _Ac;
static bool angleReady=false;

static float _Ra;
static float _Rb;
static float _Rc;
static bool ratioReady=false;

static float _Fr;
static bool freqReady=false;

static float ValuesA;
static float ValuesR;
static bool valuesReady=false;

uint8_t inPacket[32];
uint8_t sizeInPacket=0;
uint8_t outPacket[32];
uint8_t sizeOutPacket=0;


class Mercury : public PollingComponent {
    private:
        // указатель на UART, по которому общаемся с кондиционером
        UARTComponent *my_serial{nullptr};
        Sensor *VoltA {nullptr};
        Sensor *VoltB {nullptr};
        Sensor *VoltC {nullptr};
        Sensor *Amps {nullptr};
        Sensor *AmpA {nullptr};
        Sensor *AmpB {nullptr};
        Sensor *AmpC {nullptr};
        Sensor *Watts {nullptr};
        Sensor *WattA {nullptr};
        Sensor *WattB {nullptr};
        Sensor *WattC {nullptr};
        Sensor *RatioA {nullptr};
        Sensor *RatioB {nullptr};
        Sensor *RatioC {nullptr};
        Sensor *AngleA {nullptr};
        Sensor *AngleB {nullptr};
        Sensor *AngleC {nullptr};
        Sensor *Freq {nullptr};
        Sensor *ValueA {nullptr};
        Sensor *ValueR {nullptr};
        TextSensor *vers_string {nullptr};
        TextSensor *error_string {nullptr};
        TextSensor *sn_string {nullptr};
        TextSensor *fab_date_string {nullptr};
        const uint32_t minUpdatePeriod = 2000;
        const char *const TAG = "Mercury";


        // вывод отладочной информации в лог
        // 
        // dbgLevel - уровень сообщения, определен в ESPHome. За счет его использования можно из ESPHome управлять полнотой сведений в логе.
        // msg - сообщение, выводимое в лог
        // line - строка, на которой произошел вызов (удобно при отладке)
        //
        // Своровал, спасибо GrKoR :)
        void _debugMsg(const String &msg, uint8_t dbgLevel = ESPHOME_LOG_LEVEL_DEBUG, unsigned int line = 0, ... ){
            if (dbgLevel < ESPHOME_LOG_LEVEL_NONE) dbgLevel = ESPHOME_LOG_LEVEL_NONE;
            if (dbgLevel > ESPHOME_LOG_LEVEL_VERY_VERBOSE) dbgLevel = ESPHOME_LOG_LEVEL_VERY_VERBOSE;
            if (line == 0) line = __LINE__; // если строка не передана, берем текущую строку
            va_list vl;
            va_start(vl, line);
            esp_log_vprintf_(dbgLevel, TAG, line, msg.c_str(), vl);
            va_end(vl);
        }

        // выводим данные пакета в лог для отладки
        // 
        // dbgLevel - уровень сообщения, определен в ESPHome. За счет его использования можно из ESPHome управлять полнотой сведений в логе.
        // packet - указатель на пакет для вывода;
        //          если указатель на crc равен nullptr или первый байт в буфере не AC_PACKET_START_BYTE, то считаем, что передан битый пакет
        //          или не пакет вовсе. Для такого выводим только массив байт.
        //          Для нормального пакета данные выводятся с форматированием. 
        // line - строка, на которой произошел вызов (удобно при отладке)
        //
        void _debugPrintPacket(uint8_t* data, uint8_t size, bool in, uint8_t dbgLevel = ESPHOME_LOG_LEVEL_DEBUG, unsigned int line = 0){
            String st = "";
            char textBuf[11];
            // заполняем время получения пакета
            memset(textBuf, 0, 11);
            sprintf(textBuf, "%010u", millis());
            st = st + textBuf + ": ";
            // формируем преамбулы
            if (in) {
                st += "[<=] ";      // признак входящего пакета
            } else {
                st += "[=>] ";      // признак исходящего пакета
            } 
            for (uint8_t i=0; i<size; i++){
                memset(textBuf, 0, 11);
                sprintf(textBuf, "%02X", data[i]);
                st += textBuf;
                if(i<size-3){
                   st+=' ';
                } else if(i==size-3){
                   st+=F(" ("); 
                } else if(i==size-1){
                   st+=')'; 
                }                    
            }

            if (line == 0) line = __LINE__;
            _debugMsg(st, dbgLevel, line);
        }
        
        // функции обратного вызова, не удачно получилось :(
        static void cbPower(float Psumm, float Pa, float Pb, float Pc){// будет вызвана при чтении из счетчика мощности
            if(_Psumm!=Psumm){_Psumm=Psumm; powerReady=true;}
            if(_Pa!=Pa){_Pa=Pa; powerReady=true;}
            if(_Pb!=Pb){_Pb=Pb; powerReady=true;}
            if(_Pc!=Pc){_Pc=Pc; powerReady=true;}
            //_debugMsg(F("Get POWERS values"), ESPHOME_LOG_LEVEL_VERBOSE, __LINE__);
        }
        static void cbVolts(float Va, float Vb, float Vc){//  будет вызвана при чтении из счетчика напряжения
            if(_Va!=Va){_Va=Va; voltsReady=true;}
            if(_Vb!=Vb){_Vb=Vb; voltsReady=true;}
            if(_Vc!=Vc){_Vc=Vc; voltsReady=true;}
            //_debugMsg(F("Get VOLTS values"), ESPHOME_LOG_LEVEL_VERBOSE, __LINE__);
        }
        static void cbCurrent(float Ca, float Cb, float Cc){// будет вызвана при чтении из счетчика токов
            if(_Ca!=Ca){_Ca=Ca; currentReady=true;}
            if(_Cb!=Cb){_Cb=Cb; currentReady=true;}
            if(_Cc!=Cc){_Cc=Cc; currentReady=true;}
            //_debugMsg(F("Get CURRENTS values"), ESPHOME_LOG_LEVEL_VERBOSE, __LINE__);
        }
        static  void cbRatio(float Ra, float Rb, float Rc){// будет вызвана при чтении коэфициентов
            if(_Ra!=Ra){_Ra=Ra; ratioReady=true;}
            if(_Rb!=Rb){_Rb=Rb; ratioReady=true;}
            if(_Rc!=Rc){_Rc=Rc; ratioReady=true;}
            //_debugMsg(F("Get RATIOS values"), ESPHOME_LOG_LEVEL_VERBOSE, __LINE__);
        }
        static void cbAngles(float Aa, float Ab, float Ac){// будет вызвана при чтении фазовых сдвигов
            if(_Aa!=Aa){_Aa=Aa; angleReady=true;}
            if(_Ab!=Ab){_Ab=Ab; angleReady=true;}
            if(_Ac!=Ac){_Ac=Ac; angleReady=true;}
            //_debugMsg(F("Get PHASE ANGLES values"), ESPHOME_LOG_LEVEL_VERBOSE, __LINE__);
        }
        static void cbFreq(float Fr){// будет вызвана когда счетчик ответит на запрос о частоте
            if(_Fr!=Fr){_Fr=Fr; freqReady=true;} 
            //_debugMsg(F("Get FREQUENCY values"), ESPHOME_LOG_LEVEL_VERBOSE, __LINE__);
        }
        static void cbValues(float Aa, float Ar){// будет вызвана при получении показаний
            if(ValuesA!=Aa){ValuesA=Aa; valuesReady=true;}
            if(ValuesR!=Ar){ValuesR=Ar; valuesReady=true;}
            //_debugMsg(F("Get VALUES values"), ESPHOME_LOG_LEVEL_VERBOSE, __LINE__);
        }
        // для трансляции принятого пакета
        static void inDataReady(uint8_t size, uint8_t* buff){
           memcpy(inPacket,buff,size);
           sizeInPacket=size;
        }
        // для трансляции отправляемого пакета
        static void outDataReady(uint8_t size, uint8_t* buff){
           memcpy(outPacket,buff,size);
           sizeOutPacket=size;
        }

	public:
        Mercury( UARTComponent *parent, 
                        Sensor *sensor2, 
                        Sensor *sensor3, 
                        Sensor *sensor4, 
                        Sensor *sensor5, 
                        Sensor *sensor6, 
                        Sensor *sensor7, 
                        Sensor *sensor8, 
                        Sensor *sensor9, 
                        Sensor *sensor10, 
                        Sensor *sensor11, 
                        Sensor *sensor12, 
                        Sensor *sensor13, 
                        Sensor *sensor14, 
                        Sensor *sensor15, 
                        Sensor *sensor16, 
                        Sensor *sensor17, 
                        Sensor *sensor18, 
                        Sensor *sensor19, 
                        Sensor *sensor20, 
                        Sensor *sensor21, 
                        TextSensor *sensor22, 
                        TextSensor *sensor23, 
                        TextSensor *sensor24, 
                        TextSensor *sensor25)   :       my_serial(parent) , 
                                                        VoltA(sensor2) ,
                                                        VoltB(sensor3) ,
                                                        VoltC(sensor4) ,
                                                        Amps(sensor5) , 
                                                        AmpA(sensor6) , 
                                                        AmpB(sensor7) , 
                                                        AmpC(sensor8) , 
                                                        Watts(sensor9), 
                                                        WattA(sensor10), 
                                                        WattB(sensor11), 
                                                        WattC(sensor12), 
                                                        RatioA(sensor13), 
                                                        RatioB(sensor14), 
                                                        RatioC(sensor15),
                                                        AngleA(sensor16), 
                                                        AngleB(sensor17), 
                                                        AngleC(sensor18),
                                                        Freq(sensor19), 
                                                        ValueA(sensor20),
                                                        ValueR(sensor21),
                                                        vers_string(sensor22), 
                                                        error_string(sensor23),
                                                        sn_string(sensor24), 
                                                        fab_date_string(sensor25){}
      
        // вывод в дебаг текущей конфигурации компонента
        void dump_config() {
            ESP_LOGCONFIG(TAG, "Mercury:");
            LOG_SENSOR("", "Voltage phase A", this->VoltA);
            LOG_SENSOR("", "Voltage phase B", this->VoltB);
            LOG_SENSOR("", "Voltage phase C", this->VoltC);
            LOG_SENSOR("", "Amperage Summ", this->Amps);
            LOG_SENSOR("", "Amperage phase A", this->AmpA);
            LOG_SENSOR("", "Amperage phase B", this->AmpB);
            LOG_SENSOR("", "Amperage phase C", this->AmpC);
            LOG_SENSOR("", "Watts All", this->Watts);
            LOG_SENSOR("", "Watts phase A", this->WattA);
            LOG_SENSOR("", "Watts phase B", this->WattB);
            LOG_SENSOR("", "Watts phase C", this->WattC);
            LOG_SENSOR("", "Ratio phase A", this->RatioA);
            LOG_SENSOR("", "Ratio phase B", this->RatioB);
            LOG_SENSOR("", "Ratio phase C", this->RatioC);
            LOG_SENSOR("", "Phase shift AB", this->AngleA);
            LOG_SENSOR("", "Phase shift BC", this->AngleB);
            LOG_SENSOR("", "Phase shift CA", this->AngleC);
            LOG_SENSOR("", "Frequency", this->Freq);
            LOG_SENSOR("", "Values Active+", this->ValueA);
            LOG_SENSOR("", "Values Reactive+", this->ValueR);
            LOG_TEXT_SENSOR("", "Date of Мanufacture", this->fab_date_string);
            LOG_TEXT_SENSOR("", "Serial Number", this->sn_string);
            LOG_TEXT_SENSOR("", "Version", this->vers_string);
            LOG_TEXT_SENSOR("", "Last Error", this->error_string);
        }
        
        void setup() override {
             
            this->update_interval_=1000;
            
            // ЖОСТКИЙ КОЛХОЗ, потом переделаю
            // СВЕТОДИОДЫ, блин
            pinMode(BLUE_LED_PIN,OUTPUT);
            digitalWrite(BLUE_LED_PIN,LOW);
            pinMode(RED_LED_PIN,OUTPUT);
            digitalWrite(RED_LED_PIN,LOW);
            
            // установим функции обратного вызова, параметров которые хотим получать
            setCbPower(cbPower); // мощности
            setCbVolt(cbVolts);  // Вольты
            setCbCurrent(cbCurrent); // Токи
            setCbKoef(cbRatio); // Коэфициены
            setCbAngles(cbAngles); // Межфазные углы
            setCbFreq(cbFreq); // Частота
            setCbValues(cbValues); // показания
            
            // отладочный коннектор, позволяет получить указатель на буфера пакетов в момент отправки/получения
            setCbDebug(inDataReady, outDataReady); // будем печатать входящие и исходящие пакеты
            //setCbDebug(inDataReady, nullptr); // будем печатать ТОЛЬКО входящие
            
            // инициализация, адрес устройства 0 - поиск адреса
            setupMerc(0,minUpdatePeriod); // 
        }

        void loop() override {
            
            // если подключен uart
            if(my_serial!=nullptr){
                // если в буфере приема UART есть данные, значит счетчик что то прислал
                if(my_serial->available()){
                    uint8_t data;
                    my_serial->read_byte(&data); // получили байт от счетчика
                    getFromMerc(data); // передать байт в работу
                    digitalWrite(BLUE_LED_PIN,LOW); // погасить диод
                } 
            }
            
            // если подключен uart
            if(my_serial!=nullptr){
                uint8_t data=availableMerc(); // количество байт для отправки
                // если в буфере отправки есть данные - отправить счетчику
                // счетчик очень капризен к даймаутам, отправлять нужно непрерывным потоком !!!
                if(data){ 
                   uint8_t* buff=getBuffForMerc();
                   my_serial->write_array(buff,data);
                   digitalWrite(BLUE_LED_PIN,HIGH); // зажечь диод
                   //_debugPrintPacket(buff, data, false); 
                   //return; // тут нельзя долго сидеть :(
                }
            }
            
            // если нужно печатаем в лог исходящий пакет
            if(sizeOutPacket){
                _debugPrintPacket(outPacket, sizeOutPacket, false); 
                sizeOutPacket=0;
            }
            
            // если нужно печатаем в лог входящий пакет
            if(sizeInPacket){
                _debugPrintPacket(inPacket, sizeInPacket, true); 
                sizeInPacket=0;
            }
            
        }

        void update() override {

            // автокоррекция периода опроса
            static uint32_t upd_period = this->update_interval_-1;
            if(this->update_interval_ != upd_period){
                upd_period = this->update_interval_;
                if(upd_period < minUpdatePeriod){
                    upd_period = minUpdatePeriod;   
                }
                setUpdatePeriod(upd_period);
                _debugMsg(F("Core scan period %u"), ESPHOME_LOG_LEVEL_ERROR, __LINE__, upd_period);               
                upd_period = this->update_interval_;
            }
         
                    
            // контролируем ошибки связи и момент их возникновения
            static _replyReason oldError = ERROR_CORE_TIME; 
            if(oldError != getLastError()){ //если изменился статус ошибки
                oldError = getLastError();  // запомним новый статус
                if(error_string!=nullptr){ // публикация ошибок
                    error_string->publish_state(getStrError(oldError));//   
                }
                if(oldError==REP_OK){
                    _debugMsg(F("No errors !"), ESPHOME_LOG_LEVEL_WARN, __LINE__);
                    digitalWrite(RED_LED_PIN,LOW);
                } else {
                    _debugMsg(F("Error: %s"), ESPHOME_LOG_LEVEL_ERROR, __LINE__, getStrError(oldError));
                    digitalWrite(RED_LED_PIN,HIGH);
                }
            }
            
            static uint8_t counter=0;
            static uint8_t sendflg=0;
            // из-за долгой публикации пришлось разнести ее по времени
            while(counter<=8){

                if(counter==0){
                    if(readVersion()[0]!=0){ //версия прибора
                        if(vers_string!=nullptr) {
                            vers_string->publish_state(readVersion());
                            _debugMsg(F("PUB VERSION values: %s"), ESPHOME_LOG_LEVEL_VERBOSE, __LINE__, readVersion());
                            sendflg|=1;
                            break;
                        }
                    }
                } else if(counter==1){
                    if(readSerial()[0]!=0){ //серийный номер
                        if(sn_string!=nullptr) {
                            sn_string->publish_state(readSerial());
                            _debugMsg(F("PUB SERIAL NUM: %u"), ESPHOME_LOG_LEVEL_VERBOSE, __LINE__, readSerial());
                            sendflg|=2;
                            break;
                        }
                    }
                } else if(counter==2){
                    if(readFabData()[0]!=0){ //дата изготовления
                        if(fab_date_string!=nullptr) {
                            fab_date_string->publish_state(readFabData());
                            _debugMsg(F("PUB FABRIC DATA: %u"), ESPHOME_LOG_LEVEL_VERBOSE, __LINE__, readFabData());
                            sendflg|=4;
                            break;
                        }
                    }
                } else if(counter==3){
                    // ПУБЛИКУЕМ ДАННЫЕ
                    if(powerReady){
                        if(WattA!=nullptr) WattA->publish_state(_Pa); // публикуем мощности
                        if(WattB!=nullptr) WattB->publish_state(_Pb);
                        if(WattC!=nullptr) WattC->publish_state(_Pc);
                        if(Watts!=nullptr) Watts->publish_state(_Psumm);
                        powerReady=false;
                        break;
                    }
                } else if(counter==4){
                    if(voltsReady){
                        if(VoltA!=nullptr) VoltA->publish_state(_Va); // публикуем напряжения
                        if(VoltB!=nullptr) VoltB->publish_state(_Vb);
                        if(VoltC!=nullptr) VoltC->publish_state(_Vc);
                        voltsReady=false;
                        break;
                    }
                } else if(counter==5){
                    if(currentReady){
                        if(Amps!=nullptr) Amps->publish_state(_Ca+_Cb+_Cc); 
                        if(AmpA!=nullptr) AmpA->publish_state(_Ca);
                        if(AmpB!=nullptr) AmpB->publish_state(_Cb);
                        if(AmpC!=nullptr) AmpC->publish_state(_Cc);
                        currentReady=false;
                        break;
                    }
                } else if(counter==6){
                    if(ratioReady){
                        if(RatioA!=nullptr) RatioA->publish_state(_Ra);
                        if(RatioB!=nullptr) RatioB->publish_state(_Rb);
                        if(RatioC!=nullptr) RatioC->publish_state(_Rc);
                        ratioReady=false;
                        break;
                    }
                } else if(counter==7){
                    if(angleReady){
                        if(AngleA!=nullptr) AngleA->publish_state(_Aa);
                        if(AngleB!=nullptr) AngleB->publish_state(_Ab);
                        if(AngleC!=nullptr) AngleC->publish_state(_Ac);
                        angleReady=false;
                        break;
                    }
                } else if(counter==8){
                    if(freqReady){ 
                        if(Freq!=nullptr) Freq->publish_state(_Fr); // опубликуем частоту
                        freqReady=false;
                        break;
                    }
                    if(valuesReady){
                        if(ValueA!=nullptr) ValueA->publish_state(ValuesA);
                        if(ValueR!=nullptr) ValueR->publish_state(ValuesR);
                        valuesReady=false;
                        break;
                    }
                } 
                counter++;
            }
            if(counter++>8) {
                if(sendflg!=7){
                    counter=0;
                } else {
                    counter=3;
                }
            }
        }

};

//} // namespace energy_meter_mercury230
//} // namespace esphome  