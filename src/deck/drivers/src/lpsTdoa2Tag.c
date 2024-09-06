/*
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Copyright 2016, Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * lpsTdoa2Tag.c is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with lpsTdoa2Tag.c.  If not, see <http://www.gnu.org/licenses/>.
 */


#include <string.h>
#include <stdlib.h>

#include "FreeRTOS.h"
#include "task.h"

#include "autoconf.h"
#include "log.h"
#include "param.h"
#include "lpsTdoa2Tag.h"

#include "stabilizer_types.h"
#include "cfassert.h"

#include "estimator.h"

#include "physicalConstants.h"
#include "tdoaEngineInstance.h"

#if ANCHOR_STORAGE_COUNT < LOCODECK_NR_OF_TDOA2_ANCHORS
  #error "Tdoa engine storage is too small"
#endif
#if REMOTE_ANCHOR_DATA_COUNT < LOCODECK_NR_OF_TDOA2_ANCHORS
  #error "Tdoa engine storage is too small"
#endif

// Config
static lpsTdoa2AlgoOptions_t defaultOptions = {
   .anchorAddress = {
     0xbccf000000000000,
     0xbccf000000000001,
     0xbccf000000000002,
     0xbccf000000000003,
     0xbccf000000000004,
     0xbccf000000000005,
     0xbccf000000000006,
     0xbccf000000000007,
   },
};

static lpsTdoa2AlgoOptions_t* options = &defaultOptions;

// State
typedef struct {
  uint32_t anchorStatusTimeout;
} history_t;

static uint8_t previousAnchor;
// Holds data for the latest packet from all anchors
static history_t history[LOCODECK_NR_OF_TDOA2_ANCHORS];


// LPP packet handling
static lpsLppShortPacket_t lppPacket;
static bool lppPacketToSend;
static int lppPacketSendTryCounter;

static void lpsHandleLppShortPacket(const uint8_t srcId, const uint8_t *data, tdoaAnchorContext_t* anchorCtx);

// Log data
static float logUwbTdoaDistDiff[LOCODECK_NR_OF_TDOA2_ANCHORS];
static float logClockCorrection[LOCODECK_NR_OF_TDOA2_ANCHORS];
static uint16_t logAnchorDistance[LOCODECK_NR_OF_TDOA2_ANCHORS];

static bool rangingOk;
static float stdDev = TDOA_ENGINE_MEASUREMENT_NOISE_STD;

// The default receive time in the anchors for messages from other anchors is 0
// and is overwritten with the actual receive time when a packet arrives.
// That is, if no message was received the rx time will be 0.

// По умолчанию, время получения пакета от анкера = 0
// Актуальное значение перезаписывается
// Если пакет не получен - время останется = 0

static bool isValidTimeStamp(const int64_t anchorRxTime) {
  return anchorRxTime != 0;  // Проверка, что время не 0 (пакет от анкера получен)
}

static bool isConsecutiveIds(const uint8_t previousAnchor, const uint8_t currentAnchor) {
  return (((previousAnchor + 1) & 0x07) == currentAnchor); // Проверка, что айди нового анкера ПОСЛЕДОВАТЕЛЬНО и не больше 7 (0x07 = 00000111)
}

// Обновляет данные для удалённых (remote) якорей в контексте системы измерения времени прибытия (TDoA) на основе полученного пакета данных. 
static void updateRemoteData(tdoaAnchorContext_t* anchorCtx, const rangePacket2_t* packet) {
  const uint8_t anchorId = tdoaStorageGetId(anchorCtx);  // Получение текущего идентификатора якоря
  for (uint8_t i = 0; i < LOCODECK_NR_OF_TDOA2_ANCHORS; i++) {  // Цикл по всем якорям
    if (anchorId != i) {  // Для каждого якоря, кроме текущего (только удаленные)
      uint8_t remoteId = i;  // Получаем айди
      int64_t remoteRxTime = packet->timestamps[i];  // Получаем время получения сигнала для удаленного анкера
      uint8_t remoteSeqNr = packet->sequenceNrs[i] & 0x7f;  // Получаем номер пакета с обнулением страшего бита ???

      if (isValidTimeStamp(remoteRxTime)) {  // Если время валидное
        tdoaStorageSetRemoteRxTime(anchorCtx, remoteId, remoteRxTime, remoteSeqNr);  // Сохраняем временную метку удаленного анкера с инфой о пакете
      }

      bool hasDistance = (packet->distances[i] != 0);  
      if (hasDistance) {  // Если в пакете есть расстояния (tof)
        int64_t tof = packet->distances[i];  // Получаем tof
        if (isValidTimeStamp(tof)) {  // Если валидный tof
          tdoaStorageSetRemoteTimeOfFlight(anchorCtx, remoteId, tof);  // Сохраняем tof для удаленного анкера

          if (isConsecutiveIds(previousAnchor, anchorId)) {  // Если  текущий якорь и предыдщуий удаленный - последовательные 
            logAnchorDistance[anchorId] = packet->distances[previousAnchor];  // Записываем расстояние до предыдущего удаленного в массив
          }
        }
      }
    }
  }
}

// Обрабатывает LPP (Loco Posisioning Protocol) пакеты,
// полученные в контексте системы измерения времени прибытия (TDoA).
// Она анализирует пакет данных, определяет, является ли он коротким LPP пакетом,
// и затем выполняет соответствующую обработку для якоря.

// dataLength: длина данных, полученных в пакете.
// rxPacket: указатель на структуру packet_t, которая содержит информацию о принятом пакете,
// включая полезную нагрузку и адрес источника.
// anchorCtx: указатель на контекст якоря tdoaAnchorContext_t, 
// который используется для дальнейшей обработки данных.
static void handleLppPacket(const int dataLength, const packet_t* rxPacket, tdoaAnchorContext_t* anchorCtx) {
  const int32_t payloadLength = dataLength - MAC802154_HEADER_LENGTH;  // Длина полезной нагрузки. Разница между всей длиной и длиной заголовка
  const int32_t startOfLppDataInPayload = LPS_TDOA2_LPP_HEADER;  // Начало полезной нагрузки в пакете
  const int32_t lppDataLength = payloadLength - startOfLppDataInPayload;  // Длина даты без заголовка LPP

  if (lppDataLength > 0) {  // Если есть LPP данные
    const uint8_t lppPacketHeader = rxPacket->payload[LPS_TDOA2_LPP_HEADER];  // Извлекаем заголовок
    if (lppPacketHeader == LPP_HEADER_SHORT_PACKET) {  // Если соответствует SHORT_PACKET
      int srcId = -1;  // Init со значением 1

      for (int i=0; i < LOCODECK_NR_OF_TDOA2_ANCHORS; i++) {  // Находим адрес.
        if (rxPacket->sourceAddress == options->anchorAddress[i]) {
          srcId = i;  // Сравниваем адрес, указанный в пакете, со всеми известными нам адресами анкеров.
          break;  // При совпадении - запоминаем
        }
      }

      if (srcId >= 0) {  // Если адрес был найден
        lpsHandleLppShortPacket(srcId, &rxPacket->payload[LPS_TDOA2_LPP_TYPE], anchorCtx);  // Обрабатываем SHORT_PACKET
      }
    }
  }
}

// Send an LPP packet, the radio will automatically go back in RX mode
// Отправить LPP пакет. UWB автоматически уйдет в RX mode.
// dev: указатель на структуру dwDevice_t, представляющую устройство, 
// через которое будет отправляться пакет.
// packet: указатель на структуру lpsLppShortPacket_t, содержащую 
// данные для отправки, включая полезную нагрузку и информацию 
// о длине и адресе назначения.
static void sendLppShort(dwDevice_t *dev, lpsLppShortPacket_t *packet)
{
  static packet_t txPacket;  // создаем пакет
  dwIdle(dev);  // uwb уходит в idle

  MAC80215_PACKET_INIT(txPacket, MAC802154_TYPE_DATA);  // Инициализруем пакет макросом под нужный тип

  txPacket.payload[LPS_TDOA2_TYPE_INDEX] = LPP_HEADER_SHORT_PACKET;  // Заголовок указывает на тип SHORT_PACKET (0xF0)
  memcpy(&txPacket.payload[LPS_TDOA2_SEND_LPP_PAYLOAD_INDEX], packet->data, packet->length);  // Содержимое пакета переносится в отправляемый паккет, начиная с нужного индекса

  txPacket.pan = 0xbccf;  // Установка PAN
  txPacket.sourceAddress = 0xbccf000000000000 | 0xff;  // Указывается адрес источника ???
  txPacket.destAddress = options->anchorAddress[packet->dest];  // Адрес назначения (находим через айди анкера)

  dwNewTransmit(dev);  // Перевод UWB в режим TX
  dwSetDefaults(dev);  // Параметры по умолчанию для передачи
  dwSetData(dev, (uint8_t*)&txPacket, MAC802154_HEADER_LENGTH+1+packet->length);  // Переносим пакет в буфер для отправки, +1 - заголовок LPP

  dwWaitForResponse(dev, true);  // Режим ожидания ответа после передачи
  dwStartTransmit(dev);  // Начать передачу
}

// Обрабатывает приём данных с устройства dwDevice_t - UWB-радиомодуля. 
// Функция выполняет несколько задач, связанных с обработкой пакетов TDoA2
//  и взаимодействием с якорями в системе локализации.

// dev: указатель на структуру 
// dwDevice_t, представляющую радиоустройство, через которое получены данные.

// bool: Функция возвращает true, если был отправлен короткий LPP пакет (lppSent = true), 
// и false в противном случае.
static bool rxcallback(dwDevice_t *dev) {
  tdoaStats_t* stats = &tdoaEngineState.stats;
  STATS_CNT_RATE_EVENT(&stats->packetsReceived);  // Обновляем счетчик полученных пакетов

  int dataLength = dwGetDataLength(dev);  // Получаем длину данных
  packet_t rxPacket;

  dwGetData(dev, (uint8_t*)&rxPacket, dataLength);  // Получаем пакет данных
  const rangePacket2_t* packet = (rangePacket2_t*)rxPacket.payload;  // Указатель на полезную нагрузку пакета

  bool lppSent = false;
  if (packet->type == PACKET_TYPE_TDOA2) {  // Если пакет типа TDOA2
    const uint8_t anchor = rxPacket.sourceAddress & 0xff;  // Извлекаем адреся якоря, отправившего пакет

    // Check if we need to send the current LPP packet
    // Проверяем, надо ли отправить ответ
    if (lppPacketToSend && lppPacket.dest == anchor) {
      sendLppShort(dev, &lppPacket);  // Отправляем
      lppSent = true;
    }

    dwTime_t arrival = {.full = 0};
    dwGetReceiveTimestamp(dev, &arrival);  // Извлекаем метку времени приема пакета

    if (anchor < LOCODECK_NR_OF_TDOA2_ANCHORS) {  // Если айди меньше, чем количество анкеров
      uint32_t now_ms = T2M(xTaskGetTickCount());  // Текущее время в мс

      const int64_t rxAn_by_T_in_cl_T = arrival.full;  // Получаем время приема пакета нами
      const int64_t txAn_in_cl_An = packet->timestamps[anchor];  // Получаем время отправки пакета 
      const uint8_t seqNr = packet->sequenceNrs[anchor] & 0x7f;  // Получаем номер пакета

      tdoaAnchorContext_t anchorCtx;
      tdoaEngineGetAnchorCtxForPacketProcessing(&tdoaEngineState, anchor, now_ms, &anchorCtx);  // Обработка пакета
      updateRemoteData(&anchorCtx, packet);  // Обновить инфу о удаленных анкерах
      tdoaEngineProcessPacket(&tdoaEngineState, &anchorCtx, txAn_in_cl_An, rxAn_by_T_in_cl_T);  // Обработка пакета, с передачей временных меток
      tdoaStorageSetRxTxData(&anchorCtx, rxAn_by_T_in_cl_T, txAn_in_cl_An, seqNr);  // Сохраняются временнные метки

      logClockCorrection[anchor] = tdoaStorageGetClockCorrection(&anchorCtx);  // Логирование коррекции часов для текущего удаленного анкера

      previousAnchor = anchor;  // Запоминаем, какой якорь был обработан последним

      handleLppPacket(dataLength, &rxPacket, &anchorCtx);  // Обработка LPP пакета

      rangingOk = true;  // Успешная обработка пакета
    }
  }

  return lppSent;
}

// Устанавливает UWB в RX mode
static void setRadioInReceiveMode(dwDevice_t *dev) {
  dwNewReceive(dev);
  dwSetDefaults(dev);
  dwStartReceive(dev);
}

// Обрабатывает события, возникающие при работе UWB-устройства 
// (например, события приёма, отправки пакетов, ошибки и тайм-ауты).

// dev: указатель на структуру dwDevice_t, представляющую радиоустройство, 
// через которое обрабатываются события.
// event: тип события uwbEvent_t, который нужно обработать.
static uint32_t onEvent(dwDevice_t *dev, uwbEvent_t event) {
  switch(event) {
    case eventPacketReceived:
      if (rxcallback(dev)) {  // Вызыв колбека, проверка. Если пакет обработан и отправлен ответ
        lppPacketToSend = false;  // Флаг lppPacketToSend = false
      } else {
        setRadioInReceiveMode(dev);  // Иначе ставим UWB в RX mode

        // Discard lpp packet if we cannot send it for too long
        if (++lppPacketSendTryCounter >= TDOA2_LPP_PACKET_SEND_TIMEOUT) {  // Если счетчик попыток слишком большой
          lppPacketToSend = false;  // Флаг сбрасывается
        }
      }

      if (!lppPacketToSend) {  // Если флаг сброшен
        // Get next lpp packet
        lppPacketToSend = lpsGetLppShort(&lppPacket);  // Пытаемся получить следующий LPP пакет
        lppPacketSendTryCounter = 0;  // Сброс счетчика попыток
      }
      break;
    case eventTimeout:  // Далее
      // Fall through
    case eventReceiveFailed:  // Далее
      // Fall through
    case eventReceiveTimeout:  // Таймаут, Неудача получения пакета, Таймаут получения
      setRadioInReceiveMode(dev);  // Переходим снова в режим RX
      break;
    case eventPacketSent:  // UWB автоматически перейдет в режим RX после отправки
      // Service packet sent, the radio is back to receive automatically
      break;
    default:  // Любое другое событие - ошибка
      ASSERT_FAILED();
  }

  uint32_t now = xTaskGetTickCount();  // Получаем текущее время
  uint16_t rangingState = 0;
  for (int anchor = 0; anchor < LOCODECK_NR_OF_TDOA2_ANCHORS; anchor++) {  // Перебор всех якорей
    if (now < history[anchor].anchorStatusTimeout) {  // Если текущее меньше, чем таймаут - якорь активный
      rangingState |= (1 << anchor);  // Ставим бит в переменную
    }
  }
  locoDeckSetRangingState(rangingState);  // Уставновить статус общения с анкерами

  return MAX_TIMEOUT;  // Для управления временем ожидания в системе
}

// Предназначена для обработки измерений TDoA и отправки их в систему оценивания положения

// tdoaMeasurement: указатель на структуру tdoaMeasurement_t, 
// содержащую данные измерения TDoA, включая идентификаторы якорей, 
// разницу расстояний и стандартное отклонение.
static void sendTdoaToEstimatorCallback(tdoaMeasurement_t* tdoaMeasurement) {
  // Override the default standard deviation set by the TDoA engine.
  tdoaMeasurement->stdDev = stdDev;  // Устанавливается стандартное отклонение 0.15f

  estimatorEnqueueTDOA(tdoaMeasurement);  // Добавление в очередь эстиматора (локализатор)

  #ifdef CONFIG_DECK_LOCO_2D_POSITION  // Если включена локализация только 2D
  heightMeasurement_t heightData;
  heightData.timestamp = xTaskGetTickCount();
  heightData.height = DECK_LOCO_2D_POSITION_HEIGHT;
  heightData.stdDev = 0.0001;
  estimatorEnqueueAbsoluteHeight(&heightData);
  #endif

  const uint8_t idA = tdoaMeasurement->anchorIds[0];  // Извлекаются айди пары анкеров
  const uint8_t idB = tdoaMeasurement->anchorIds[1];
  if (isConsecutiveIds(idA, idB)) {  // Если последовательны
    logUwbTdoaDistDiff[idB] = tdoaMeasurement->distanceDiff;  // Записываем разницу между ними для второго анкера
  }
}

// Настраивает и инициализирует систему для работы с UWB
static void Initialize(dwDevice_t *dev) {
  uint32_t now_ms = T2M(xTaskGetTickCount());  // Текущее время
  // Инит движка, Статус, текущее время, коллбек, для отправки измерений, частота, алгоритм вычисления
  tdoaEngineInit(&tdoaEngineState, now_ms, sendTdoaToEstimatorCallback, LOCODECK_TS_FREQ, TdoaEngineMatchingAlgorithmYoungest);

  previousAnchor = 0;  // Инит переменной, с последним анкером

  lppPacketToSend = false;  // Инит флага, что пакет надо отправить

  locoDeckSetRangingState(0);  // Нет активных анкеров в начале
  dwSetReceiveWaitTimeout(dev, TDOA2_RECEIVE_TIMEOUT);  // Настройка таймаута на ожидание приема

  dwCommitConfiguration(dev);  // Применение конфигурации

  rangingOk = false;  // Инит переменной и начальное состояние
}

// Возвращает статус rangingOk
static bool isRangingOk()
{
  return rangingOk;
}

// Предназначена для получения позиции якоря по его идентификатору.

// anchorId: идентификатор якоря, для которого нужно получить позицию.
// position: указатель на структуру point_t, куда будет сохранена позиция якоря 
static bool getAnchorPosition(const uint8_t anchorId, point_t* position) {
  tdoaAnchorContext_t anchorCtx;  // Создаем контекст
  uint32_t now_ms = T2M(xTaskGetTickCount());  // Получаем время сейчас

// Ищем контекст в массиве по айдишнику анкера. Если найден, сохраняется в anchorCtx
  bool contextFound = tdoaStorageGetAnchorCtx(tdoaEngineState.anchorInfoArray, anchorId, now_ms, &anchorCtx);
  if (contextFound) {  // Если найден контекст
    tdoaStorageGetAnchorPosition(&anchorCtx, position);  // Извлекает позиция анкера
    return true;  // Возврат тру
  }

  return false;
}

// Получает список айди анкеров
static uint8_t getAnchorIdList(uint8_t unorderedAnchorList[], const int maxListSize) {
  return tdoaStorageGetListOfAnchorIds(tdoaEngineState.anchorInfoArray, unorderedAnchorList, maxListSize);
}

// Получает список активных анкеров
static uint8_t getActiveAnchorIdList(uint8_t unorderedAnchorList[], const int maxListSize) {
  uint32_t now_ms = T2M(xTaskGetTickCount());  // Время сейчас
  return tdoaStorageGetListOfActiveAnchorIds(tdoaEngineState.anchorInfoArray, unorderedAnchorList, maxListSize, now_ms);
}

// Loco Posisioning Protocol (LPP) handling
// Обрабатывает короткие LPP-пакеты (Low Power Protocol) от якорей 
// и обновляет информацию о положении якоря, если пакет содержит данные о позиции.

// srcId: идентификатор источника пакета (якоря), от которого поступил пакет.
// data: указатель на данные пакета
// anchorCtx: указатель на контекст якоря (tdoaAnchorContext_t), 
// который будет обновляться, если пакет содержит информацию о позиции.
static void lpsHandleLppShortPacket(const uint8_t srcId, const uint8_t *data, tdoaAnchorContext_t* anchorCtx)
{
  uint8_t type = data[0];  // Определяем тип пакета по первому байту

  if (type == LPP_SHORT_ANCHORPOS) {  // Если тип SHORT_ANCHORPOS
    if (srcId < LOCODECK_NR_OF_TDOA2_ANCHORS) {  // Проверка, что айди источника меньше общего количества
      struct lppShortAnchorPos_s *newpos = (struct lppShortAnchorPos_s*)&data[1];  // Интерпретирует данные, начиная со второго байта, как структуру lppShortAnchorPos_s, содержащую координаты позиции якоря
      tdoaStorageSetAnchorPosition(anchorCtx, newpos->x, newpos->y, newpos->z);  // Обновляем позицию анкера
    }
  }
}

// Заполнение структуры для инициализации алгоритма
uwbAlgorithm_t uwbTdoa2TagAlgorithm = {  
  .init = Initialize,
  .onEvent = onEvent,
  .isRangingOk = isRangingOk,
  .getAnchorPosition = getAnchorPosition,
  .getAnchorIdList = getAnchorIdList,
  .getActiveAnchorIdList = getActiveAnchorIdList,
};

// Применить новые опции
void lpsTdoa2TagSetOptions(lpsTdoa2AlgoOptions_t* newOptions) {
  options = newOptions;
}

// ЛОГИРОВАНИЕ

LOG_GROUP_START(tdoa2)
LOG_ADD(LOG_FLOAT, d7-0, &logUwbTdoaDistDiff[0])
LOG_ADD(LOG_FLOAT, d0-1, &logUwbTdoaDistDiff[1])
LOG_ADD(LOG_FLOAT, d1-2, &logUwbTdoaDistDiff[2])
LOG_ADD(LOG_FLOAT, d2-3, &logUwbTdoaDistDiff[3])
LOG_ADD(LOG_FLOAT, d3-4, &logUwbTdoaDistDiff[4])
LOG_ADD(LOG_FLOAT, d4-5, &logUwbTdoaDistDiff[5])
LOG_ADD(LOG_FLOAT, d5-6, &logUwbTdoaDistDiff[6])
LOG_ADD(LOG_FLOAT, d6-7, &logUwbTdoaDistDiff[7])

LOG_ADD(LOG_FLOAT, cc0, &logClockCorrection[0])
LOG_ADD(LOG_FLOAT, cc1, &logClockCorrection[1])
LOG_ADD(LOG_FLOAT, cc2, &logClockCorrection[2])
LOG_ADD(LOG_FLOAT, cc3, &logClockCorrection[3])
LOG_ADD(LOG_FLOAT, cc4, &logClockCorrection[4])
LOG_ADD(LOG_FLOAT, cc5, &logClockCorrection[5])
LOG_ADD(LOG_FLOAT, cc6, &logClockCorrection[6])
LOG_ADD(LOG_FLOAT, cc7, &logClockCorrection[7])

LOG_ADD(LOG_UINT16, dist7-0, &logAnchorDistance[0])
LOG_ADD(LOG_UINT16, dist0-1, &logAnchorDistance[1])
LOG_ADD(LOG_UINT16, dist1-2, &logAnchorDistance[2])
LOG_ADD(LOG_UINT16, dist2-3, &logAnchorDistance[3])
LOG_ADD(LOG_UINT16, dist3-4, &logAnchorDistance[4])
LOG_ADD(LOG_UINT16, dist4-5, &logAnchorDistance[5])
LOG_ADD(LOG_UINT16, dist5-6, &logAnchorDistance[6])
LOG_ADD(LOG_UINT16, dist6-7, &logAnchorDistance[7])
LOG_GROUP_STOP(tdoa2)

PARAM_GROUP_START(tdoa2)
/**
 * @brief The measurement noise to use when sending TDoA measurements to the estimator.
 */
PARAM_ADD(PARAM_FLOAT, stddev, &stdDev)

PARAM_GROUP_STOP(tdoa2)
