#ifndef __TDOA_ENGINE_H__
#define __TDOA_ENGINE_H__

#include "tdoaStorage.h"
#include "tdoaStats.h"
#include "autoconf.h"

#if CONFIG_DECK_LOCO_LONGER_RANGE
#define TDOA_ENGINE_MEASUREMENT_NOISE_STD 0.30f
#else
#define TDOA_ENGINE_MEASUREMENT_NOISE_STD 0.15f //Стандартное отклонение шума измерения TDoA. Зависит от конфигурации дальнобойности системы.
#endif

typedef void (*tdoaEngineSendTdoaToEstimator)(tdoaMeasurement_t* tdoaMeasurement);

typedef enum {
  TdoaEngineMatchingAlgorithmNone = 0,
  TdoaEngineMatchingAlgorithmRandom,
  TdoaEngineMatchingAlgorithmYoungest,
} tdoaEngineMatchingAlgorithm_t; //Перечисление, определяющее алгоритм сопоставления якорей для вычисления TDoA.

typedef struct {
  // State
  tdaoAnchorInfoArray_t anchorInfoArray; // Массив, хранящий информацию о каждом обнаруженном якоре.
  tdoaStats_t stats; // Структура, хранящая статистику работы движка TDOA.

// Указатель на функцию, которая будет вызываться для отправки данных TDOA в модуль оценки местоположения.
  tdoaEngineSendTdoaToEstimator sendTdoaToEstimator; 
  double locodeckTsFreq; // Частота таймера Locodeck, используемого для измерения времени.
  tdoaEngineMatchingAlgorithm_t matchingAlgorithm; // Алгоритм сопоставления якорей для вычисления TDOA.

  // Matching algorithm data
  struct {
    uint8_t seqNr[REMOTE_ANCHOR_DATA_COUNT]; // Массив, хранящий номера последовательностей пакетов от удаленных якорей.
    uint8_t id[REMOTE_ANCHOR_DATA_COUNT]; // Массив, хранящий идентификаторы удаленных якорей.
    uint8_t offset;// Смещение, используемое в алгоритме случайного сопоставления якорей.
  } matching;
} tdoaEngineState_t; //Структура, содержащая состояние движка TDoA. Включает информацию о якорях, статистику, конфигурацию и данные для алгоритма сопоставления.

//Функция инициализации движка TDoA.
void tdoaEngineInit(tdoaEngineState_t* state, const uint32_t now_ms, tdoaEngineSendTdoaToEstimator sendTdoaToEstimator, const double locodeckTsFreq, const tdoaEngineMatchingAlgorithm_t matchingAlgorithm);
//Функция получения контекста якоря для обработки пакета.
void tdoaEngineGetAnchorCtxForPacketProcessing(tdoaEngineState_t* engineState, const uint8_t anchorId, const uint32_t currentTime_ms, tdoaAnchorContext_t* anchorCtx);
//Функция обработки пакета данных от якоря.
void tdoaEngineProcessPacket(tdoaEngineState_t* engineState, tdoaAnchorContext_t* anchorCtx, const int64_t txAn_in_cl_An, const int64_t rxAn_by_T_in_cl_T);
//Функция обработки пакета данных от якоря с возможностью фильтрации по ID якоря.
bool tdoaEngineProcessPacketFiltered(tdoaEngineState_t* engineState, tdoaAnchorContext_t* anchorCtx, const int64_t txAn_in_cl_An, const int64_t rxAn_by_T_in_cl_T, const bool doExcludeId, const uint8_t excludedId);

#define TDOA_ENGINE_TRUNCATE_TO_ANCHOR_TS_BITMAP 0x00FFFFFFFF //Маска для усечения временной метки до временной метки якоря.
//Функция усечения временной метки до временной метки якоря.
static inline uint64_t tdoaEngineTruncateToAnchorTimeStamp(uint64_t fullTimeStamp) {
  return fullTimeStamp & TDOA_ENGINE_TRUNCATE_TO_ANCHOR_TS_BITMAP;
}

#endif // __TDOA_ENGINE_H__
