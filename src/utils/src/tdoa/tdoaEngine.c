/*
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie firmware.
 *
 * Copyright 2018, Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * tdoaEngine.c is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with tdoaEngine.c. If not, see <http://www.gnu.org/licenses/>.
 */


/*
Implementation of LPS TDoA Tag functionality

The tag is assumed to move around in a large system of anchors. Any anchor ids
can be used, and the same anchor id can even be used by multiple anchors as long
as they are not visible in the same area. It is assumed that the anchor density
is evenly distributed in the covered volume and that 5-20 anchors are visible
in every point. The tag is attached to a physical object and the expected
velocity is a few m/s, this means that anchors are within range for a time
period of seconds.

The implementation must handle
1. An infinite number of anchors, where around 20 are visible at one time
2. Any anchor ids
3. Dynamically changing visibility of anchors over time
4. Random TX times from anchors with possible packet collisions and packet loss

*/

#include <string.h>

#define DEBUG_MODULE "TDOA_ENGINE"
#include "debug.h"

#include "tdoaEngine.h"
#include "tdoaStats.h"
#include "clockCorrectionEngine.h"
#include "physicalConstants.h"

// **Инициализирует движок TDoA, устанавливая начальные значения и сохраняя важные параметры.**
// 
// `engineState`: Указатель на структуру состояния движка TDoA.
// `now_ms`: Текущее время в миллисекундах.
// `sendTdoaToEstimator`: Указатель на функцию, которая будет вызвана для отправки данных TDoA в модуль оценки местоположения.
// `locodeckTsFreq`: Частота таймера Locodeck.
// `matchingAlgorithm`: Выбранный алгоритм сопоставления якорей.
void tdoaEngineInit(tdoaEngineState_t* engineState, const uint32_t now_ms, tdoaEngineSendTdoaToEstimator sendTdoaToEstimator, const double locodeckTsFreq, const tdoaEngineMatchingAlgorithm_t matchingAlgorithm) {
  // Инициализирует хранилище информации о якорях, обнуляя все слоты в массиве `anchorInfoArray`.
  // Каждый слот хранит информацию о конкретном якоре, такую как время приема/передачи, 
  // коррекция часов, номер последовательности и т.д. 
  // Подробнее см. `tdoaStorage.h` и `tdoaStorage.c`.
  tdoaStorageInitialize(engineState->anchorInfoArray); 
  // Инициализирует модуль сбора статистики TDoA, устанавливая начальное время и сбрасывая счетчики.
  tdoaStatsInit(&engineState->stats, now_ms);
  // Сохраняет указатель на функцию отправки данных TDoA. 
  // Эта функция будет вызвана позже, когда будут готовы данные TDoA, 
  // чтобы передать их в модуль оценки местоположения.
  engineState->sendTdoaToEstimator = sendTdoaToEstimator; 
  // Сохраняет частоту таймера Locodeck. 
  // Эта частота используется для преобразования разницы во времени 
  // (время прихода сигнала от разных якорей) в разницу в расстоянии.
  engineState->locodeckTsFreq = locodeckTsFreq; 
  // Сохраняет выбранный алгоритм сопоставления якорей.
  // Алгоритм сопоставления определяет, как выбрать пару якорей для 
  // вычисления TDoA из всех доступных якорей. 
  // Возможные варианты: случайный выбор (`TdoaEngineMatchingAlgorithmRandom`) 
  // или выбор по самому "молодому" якорю (`TdoaEngineMatchingAlgorithmYoungest`).
  engineState->matchingAlgorithm = matchingAlgorithm; 

  // Инициализирует смещение для алгоритма случайного сопоставления. 
  // Это смещение гарантирует, что при каждом вызове алгоритма 
  // будет выбран другой якорь, обеспечивая случайность выбора.
  engineState->matching.offset = 0; 
}

// **Добавляет данные TDoA в очередь для отправки в модуль оценки местоположения.**
// 
// `anchorACtx`: Контекст первого якоря.
// `anchorBCtx`: Контекст второго якоря.
// `distanceDiff`: Разница в расстоянии между тегом и двумя якорями.
// `engineState`: Указатель на структуру состояния движка TDoA.
static void enqueueTDOA(const tdoaAnchorContext_t* anchorACtx, const tdoaAnchorContext_t* anchorBCtx, double distanceDiff, tdoaEngineState_t* engineState) {
  tdoaStats_t* stats = &engineState->stats; // Получает указатель на структуру статистики.

  // Структура для хранения данных TDoA (разница в расстоянии, стандартное отклонение, ID якорей, позиции якорей).
  tdoaMeasurement_t tdoa = { 
    .stdDev = TDOA_ENGINE_MEASUREMENT_NOISE_STD, // Записывает стандартное отклонение шума измерения (фиксированное значение).
    .distanceDiff = distanceDiff // Записывает разницу в расстоянии, вычисленную ранее.
  };

  // Проверка, доступны ли позиции обоих якорей (т.е. известны ли их координаты).
  // Позиции якорей могут быть получены из внешнего источника или вычислены ранее.
  if (tdoaStorageGetAnchorPosition(anchorACtx, &tdoa.anchorPositions[0]) && tdoaStorageGetAnchorPosition(anchorBCtx, &tdoa.anchorPositions[1])) {
    // Увеличивает счетчик отправленных пакетов TDoA.
    STATS_CNT_RATE_EVENT(&stats->packetsToEstimator);
   
    // Получает ID первого и второго якорей.
    uint8_t idA = tdoaStorageGetId(anchorACtx); 
    uint8_t idB = tdoaStorageGetId(anchorBCtx); 
    // Проверяет, соответствуют ли ID якорей ожидаемым значениям (используется для сбора статистики).
    if (idA == stats->anchorId && idB == stats->remoteAnchorId) {
      stats->tdoa = distanceDiff; // Сохраняет разницу в расстоянии в статистике.
    }
    // Проверяет, соответствуют ли ID якорей ожидаемым значениям в обратном порядке (используется для сбора статистики).
    if (idB == stats->anchorId && idA == stats->remoteAnchorId) {
      stats->tdoa = -distanceDiff; // Сохраняет разницу в расстоянии в статистике (с обратным знаком).
    }
    // Записывает ID первого и второго якорей в структуру TDoA.
    tdoa.anchorIds[0] = idA; 
    tdoa.anchorIds[1] = idB; 

    // Отправляет данные TDoA в модуль оценки местоположения с помощью функции, 
    // указатель на которую был передан при инициализации движка.
    engineState->sendTdoaToEstimator(&tdoa); 
  }
}

// **Обновляет информацию о коррекции часов для якоря, используя алгоритм коррекции часов.**
// 
// `anchorCtx`: Контекст якоря.
// `txAn_in_cl_An`: Время передачи пакета якорем в системе отсчета часов якоря.
// `rxAn_by_T_in_cl_T`: Время приема пакета тегом в системе отсчета часов тега.
// `stats`: Указатель на структуру статистики.
// 
// **Возвращает:** `true`, если измерение считается надежным, `false` в противном случае.
static bool updateClockCorrection(tdoaAnchorContext_t* anchorCtx, const int64_t txAn_in_cl_An, const int64_t rxAn_by_T_in_cl_T, tdoaStats_t* stats) {
  bool sampleIsReliable = false; // Инициализирует флаг надежности измерения.

  // Получает последнее измеренное время приема пакета от якоря и время передачи пакета якорем.
  // Эти значения хранятся в структуре `tdoaAnchorInfo_t`, которая доступна через `anchorCtx`.
  const int64_t latest_rxAn_by_T_in_cl_T = tdoaStorageGetRxTime(anchorCtx); 
  const int64_t latest_txAn_in_cl_An = tdoaStorageGetTxTime(anchorCtx);

  // Проверяет, были ли ранее измерения для этого якоря. 
  // Если нет, то коррекция часов невозможна, и функция возвращает `false`.
  if (latest_rxAn_by_T_in_cl_T != 0 && latest_txAn_in_cl_An != 0) {
    // Вычисляет потенциальную коррекцию часов, используя алгоритм коррекции часов.
    // Алгоритм использует текущее и предыдущее измерение времени передачи и приема пакета 
    // для оценки разницы между часами якоря и тега.
    // `TDOA_ENGINE_TRUNCATE_TO_ANCHOR_TS_BITMAP` -  маска, определяющая, какие биты 
    // временной метки якоря будут использоваться для коррекции часов.
    double clockCorrectionCandidate = clockCorrectionEngineCalculate(rxAn_by_T_in_cl_T, latest_rxAn_by_T_in_cl_T, txAn_in_cl_An, latest_txAn_in_cl_An, TDOA_ENGINE_TRUNCATE_TO_ANCHOR_TS_BITMAP);
    // Обновляет коррекцию часов для якоря, используя вычисленное значение. 
    // Функция `clockCorrectionEngineUpdate` также проверяет надежность измерения, 
    // например, сравнивая новое значение с предыдущими значениями и 
    // отбрасывая выбросы. Возвращает `true`, если измерение считается надежным.
    sampleIsReliable = clockCorrectionEngineUpdate(tdoaStorageGetClockCorrectionStorage(anchorCtx), clockCorrectionCandidate);

    // Если измерение надежное...
    if (sampleIsReliable){
      // ...и если это основной якорь (anchorId указан в статистике)...
      if (tdoaStorageGetId(anchorCtx) == stats->anchorId) {
        // ...то сохраняет коррекцию часов в статистике.
        stats->clockCorrection = tdoaStorageGetClockCorrection(anchorCtx);
        // Увеличивает счетчик успешных коррекций часов.
        STATS_CNT_RATE_EVENT(&stats->clockCorrectionCount);
      }
    }
  }

  return sampleIsReliable; // Возвращает флаг надежности измерения.
}

// **Вычисляет разницу во времени прихода сигнала (TDoA) между двумя якорями.**
// 
// `otherAnchorCtx`: Контекст второго якоря.
// `anchorCtx`: Контекст первого якоря.
// `txAn_in_cl_An`: Время передачи пакета первым якорем в системе отсчета часов первого якоря.
// `rxAn_by_T_in_cl_T`: Время приема пакета от первого якоря тегом в системе отсчета часов тега.
// 
// **Возвращает:** TDoA в тиках часов тега.
static int64_t calcTDoA(const tdoaAnchorContext_t* otherAnchorCtx, const tdoaAnchorContext_t* anchorCtx, const int64_t txAn_in_cl_An, const int64_t rxAn_by_T_in_cl_T) {
  // Получает ID второго якоря.
  const uint8_t otherAnchorId = tdoaStorageGetId(otherAnchorCtx);

  // Получает время пролета сигнала между первым и вторым якорями.
  // Это значение было измерено ранее и хранится в структуре `tdoaAnchorInfo_t` первого якоря.
  const int64_t tof_Ar_to_An_in_cl_An = tdoaStorageGetRemoteTimeOfFlight(anchorCtx, otherAnchorId);
  // Получает время приема пакета от второго якоря первым якорем.
  // Это значение было измерено ранее и хранится в структуре `tdoaAnchorInfo_t` первого якоря.
  const int64_t rxAr_by_An_in_cl_An = tdoaStorageGetRemoteRxTime(anchorCtx, otherAnchorId);
  // Получает коррекцию часов первого якоря.
  const double clockCorrection = tdoaStorageGetClockCorrection(anchorCtx);

  // Получает время приема пакета от второго якоря тегом.
  // Это значение хранится в структуре `tdoaAnchorInfo_t` второго якоря.
  const int64_t rxAr_by_T_in_cl_T = tdoaStorageGetRxTime(otherAnchorCtx);

  // Вычисляет разницу во времени передачи пакетов первым и вторым якорями 
  // в системе отсчета часов первого якоря. 
  // `tdoaEngineTruncateToAnchorTimeStamp` - функция, которая обрезает 
  // временную метку до разрешения часов якоря.
  const int64_t delta_txAr_to_txAn_in_cl_An = (tof_Ar_to_An_in_cl_An + tdoaEngineTruncateToAnchorTimeStamp(txAn_in_cl_An - rxAr_by_An_in_cl_An));
  // Вычисляет разницу во времени прихода сигналов от первого и второго якорей к тегу (TDoA) 
  // в системе отсчета часов тега.
  // Учитывается коррекция часов первого якоря.
  const int64_t timeDiffOfArrival_in_cl_T =  tdoaEngineTruncateToAnchorTimeStamp(rxAn_by_T_in_cl_T - rxAr_by_T_in_cl_T) - delta_txAr_to_txAn_in_cl_An  * clockCorrection;

  return timeDiffOfArrival_in_cl_T; // Возвращает TDoA.
}

// **Вычисляет разницу в расстоянии, соответствующую TDoA.**
// 
// `otherAnchorCtx`: Контекст второго якоря.
// `anchorCtx`: Контекст первого якоря.
// `txAn_in_cl_An`: Время передачи пакета первым якорем в системе отсчета часов первого якоря.
// `rxAn_by_T_in_cl_T`: Время приема пакета от первого якоря тегом в системе отсчета часов тега.
// `locodeckTsFreq`: Частота таймера Locodeck.
// 
// **Возвращает:** Разницу в расстоянии в метрах.
static double calcDistanceDiff(const tdoaAnchorContext_t* otherAnchorCtx, const tdoaAnchorContext_t* anchorCtx, const int64_t txAn_in_cl_An, const int64_t rxAn_by_T_in_cl_T, const double locodeckTsFreq) {
  // Вычисляет TDoA с помощью функции `calcTDoA`.
  const int64_t tdoa = calcTDoA(otherAnchorCtx, anchorCtx, txAn_in_cl_An, rxAn_by_T_in_cl_T); 
  // Преобразует TDoA (в тиках часов тега) в разницу в расстоянии (в метрах), 
  // умножая на скорость света и деля на частоту таймера Locodeck.
  return SPEED_OF_LIGHT * tdoa / locodeckTsFreq; 
}

// **Находит случайный якорь для сопоставления с текущим якорем.**
// 
// `engineState`: Указатель на структуру состояния движка TDoA.
// `otherAnchorCtx`: Указатель на контекст найденного якоря. Будет заполнен, если функция вернет `true`.
// `anchorCtx`: Контекст текущего якоря.
// `doExcludeId`: Флаг, указывающий, нужно ли исключать определенный ID якоря из списка кандидатов.
// `excludedId`: ID якоря, который нужно исключить, если `doExcludeId` равен `true`.
// 
// **Возвращает:** `true`, если найден подходящий якорь, `false` в противном случае.
static bool matchRandomAnchor(tdoaEngineState_t* engineState, tdoaAnchorContext_t* otherAnchorCtx, const tdoaAnchorContext_t* anchorCtx, const bool doExcludeId, const uint8_t excludedId) {
  // Увеличивает смещение для алгоритма случайного сопоставления. 
  // Это гарантирует, что при каждом вызове функции будет выбран другой 
  // начальный индекс в списке кандидатов.
  engineState->matching.offset++;  
  int remoteCount = 0;
  // Получает список номеров последовательностей (`seqNr`) и ID (`id`) 
  // удаленных якорей, видимых текущим якорем. 
  // Эта информация хранится в структуре `tdoaAnchorInfo_t` текущего якоря.
  tdoaStorageGetRemoteSeqNrList(anchorCtx, &remoteCount, engineState->matching.seqNr, engineState->matching.id);

  uint32_t now_ms = anchorCtx->currentTime_ms; // Получает текущее время.

  // Loop over the candidates and pick the first one that is useful
  // An offset (updated for each call) is added to make sure we start at
  // different positions in the list and vary which candidate to choose
  // Перебирает кандидатов на сопоставление, начиная со случайного 
  // индекса, определенного смещением `engineState->matching.offset`.
  for (int i = engineState->matching.offset; i < (remoteCount + engineState->matching.offset); i++) {
    // Вычисляет индекс в списке кандидатов с учетом циклического перебора.
    uint8_t index = i % remoteCount; 
    // Получает ID кандидата.
    const uint8_t candidateAnchorId = engineState->matching.id[index]; 
    // Проверяет, не нужно ли исключить этот ID.
    if (!doExcludeId || (excludedId != candidateAnchorId)) { 
      // Пытается получить или создать контекст кандидата. 
      // Если контекст кандидата уже существует (якорь был виден ранее), 
      // функция `tdoaStorageGetCreateAnchorCtx` вернет `true` и заполнит `otherAnchorCtx`. 
      // Если контекст кандидата не существует, функция вернет `false` и создаст новый контекст.
      if (tdoaStorageGetCreateAnchorCtx(engineState->anchorInfoArray, candidateAnchorId, now_ms, otherAnchorCtx)) {
        // Проверяет, соответствует ли номер последовательности кандидата 
        // номеру последовательности, хранящемуся в списке `seqNr` текущего якоря.
        // Также проверяет, доступно ли время пролета сигнала между 
        // текущим якорем и кандидатом (т.е. было ли оно измерено ранее).
        if (engineState->matching.seqNr[index] == tdoaStorageGetSeqNr(otherAnchorCtx) && tdoaStorageGetRemoteTimeOfFlight(anchorCtx, candidateAnchorId)) {
          return true; // Если кандидат подходит, возвращает `true`.
        }
      }
    }
  }
  // Если подходящий кандидат не найден, сбрасывает контекст 
  // (устанавливает `otherAnchorCtx->anchorInfo` в 0) и возвращает `false`.
  otherAnchorCtx->anchorInfo = 0;
  return false;
}

// **Находит якорь с самым поздним временем обновления для сопоставления с текущим якорем.**
// 
// `engineState`: Указатель на структуру состояния движка TDoA.
// `otherAnchorCtx`: Указатель на контекст найденного якоря. Будет заполнен, если функция вернет `true`.
// `anchorCtx`: Контекст текущего якоря.
// `doExcludeId`: Флаг, указывающий, нужно ли исключать определенный ID якоря из списка кандидатов.
// `excludedId`: ID якоря, который нужно исключить, если `doExcludeId` равен `true`.
// 
// **Возвращает:** `true`, если найден подходящий якорь, `false` в противном случае.
static bool matchYoungestAnchor(tdoaEngineState_t* engineState, tdoaAnchorContext_t* otherAnchorCtx, const tdoaAnchorContext_t* anchorCtx, const bool doExcludeId, const uint8_t excludedId) {
    int remoteCount = 0;
    // Получает список номеров последовательностей (`seqNr`) и ID (`id`) 
    // удаленных якорей, видимых текущим якорем. 
    // Эта информация хранится в структуре `tdoaAnchorInfo_t` текущего якоря.
    tdoaStorageGetRemoteSeqNrList(anchorCtx, &remoteCount, engineState->matching.seqNr, engineState->matching.id);

    uint32_t now_ms = anchorCtx->currentTime_ms; // Получает текущее время.
    uint32_t youmgestUpdateTime = 0;  // Инициализирует время последнего обновления самого "молодого" якоря.
    int bestId = -1; // Инициализирует ID самого "молодого" якоря.

    // Перебирает кандидатов на сопоставление.
    for (int index = 0; index < remoteCount; index++) {
      // Получает ID кандидата.
      const uint8_t candidateAnchorId = engineState->matching.id[index];
      // Проверяет, не нужно ли исключить этот ID.
      if (!doExcludeId || (excludedId != candidateAnchorId)) {
        // Проверяет наличие времени пролета сигнала для кандидата. 
        // Время пролета сигнала должно быть измерено ранее.
        if (tdoaStorageGetRemoteTimeOfFlight(anchorCtx, candidateAnchorId)) {
          // Получает контекст кандидата. 
          // Если контекст кандидата уже существует (якорь был виден ранее), 
          // функция `tdoaStorageGetCreateAnchorCtx` вернет `true` и заполнит `otherAnchorCtx`. 
          // Если контекст кандидата не существует, функция вернет `false` и создаст новый контекст.
          if (tdoaStorageGetCreateAnchorCtx(engineState->anchorInfoArray, candidateAnchorId, now_ms, otherAnchorCtx)) {
            // Получает время последнего обновления кандидата.
            uint32_t updateTime = tdoaStorageGetLastUpdateTime(otherAnchorCtx);
            // Если кандидат "моложе" текущего самого "молодого"...
            if (updateTime > youmgestUpdateTime) {
              // ...и номер последовательности совпадает...
              if (engineState->matching.seqNr[index] == tdoaStorageGetSeqNr(otherAnchorCtx)) {
                // ...то обновляет время последнего обновления и ID самого "молодого" якоря.
                youmgestUpdateTime = updateTime;
                bestId = candidateAnchorId;
              }
            }
          }
        }
      }
    }

    // Если найден "молодой" якорь...
    if (bestId >= 0) { 
      // ...то получает его контекст и возвращает `true`.
      tdoaStorageGetCreateAnchorCtx(engineState->anchorInfoArray, bestId, now_ms, otherAnchorCtx);
      return true; 
    }

    // Если подходящий кандидат не найден, сбрасывает контекст 
    // (устанавливает `otherAnchorCtx->anchorInfo` в 0) и возвращает `false`.
    otherAnchorCtx->anchorInfo = 0;
    return false;
}

// **Находит подходящий якорь для сопоставления с текущим якорем, 
// используя выбранный алгоритм сопоставления.**
// 
// `engineState`: Указатель на структуру состояния движка TDoA.
// `otherAnchorCtx`: Указатель на контекст найденного якоря. Будет заполнен, если функция вернет `true`.
// `anchorCtx`: Контекст текущего якоря.
// `doExcludeId`: Флаг, указывающий, нужно ли исключать определенный ID якоря из списка кандидатов.
// `excludedId`: ID якоря, который нужно исключить, если `doExcludeId` равен `true`.
// 
// **Возвращает:** `true`, если найден подходящий якорь, `false` в противном случае.
static bool findSuitableAnchor(tdoaEngineState_t* engineState, tdoaAnchorContext_t* otherAnchorCtx, const tdoaAnchorContext_t* anchorCtx, const bool doExcludeId, const uint8_t excludedId) {
  bool result = false; // Инициализирует флаг успешного поиска якоря.

  // Проверяет, доступна ли коррекция часов для текущего якоря.
  // Коррекция часов необходима для точного вычисления TDoA.
  if (tdoaStorageGetClockCorrection(anchorCtx) > 0.0) {
    // Выбирает алгоритм сопоставления якорей, 
    // который был установлен при инициализации движка TDoA.
    switch(engineState->matchingAlgorithm) { 
      case TdoaEngineMatchingAlgorithmRandom:
        // Вызывает функцию случайного сопоставления.
        result = matchRandomAnchor(engineState, otherAnchorCtx, anchorCtx, doExcludeId, excludedId);
        break;
      case TdoaEngineMatchingAlgorithmYoungest:
        // Вызывает функцию сопоставления по самому "молодому" якорю.
        result = matchYoungestAnchor(engineState, otherAnchorCtx, anchorCtx, doExcludeId, excludedId);
        break;

      default:
        // Do nothing
        break;
    }
  }

  return result;  // Возвращает флаг успешного поиска якоря.
}

// **Получает контекст якоря для обработки пакета.**
// 
// `engineState`: Указатель на структуру состояния движка TDoA.
// `anchorId`: ID якоря.
// `currentTime_ms`: Текущее время в миллисекундах.
// `anchorCtx`: Указатель на контекст якоря. Будет заполнен, если функция успешно найдет или создаст контекст.
void tdoaEngineGetAnchorCtxForPacketProcessing(tdoaEngineState_t* engineState, const uint8_t anchorId, const uint32_t currentTime_ms, tdoaAnchorContext_t* anchorCtx) {
  // Пытается получить или создать контекст якоря с заданным ID.
  // Если контекст якоря уже существует (якорь был виден ранее), 
  // функция `tdoaStorageGetCreateAnchorCtx` вернет `true` и заполнит `anchorCtx`. 
  // Если контекст якоря не существует, функция вернет `false` и создаст новый контекст.
  if (tdoaStorageGetCreateAnchorCtx(engineState->anchorInfoArray, anchorId, currentTime_ms, anchorCtx)) {
    // Увеличивает счетчик успешных получений контекста.
    STATS_CNT_RATE_EVENT(&engineState->stats.contextHitCount);
  } else {
    // Увеличивает счетчик неудачных получений контекста.
    STATS_CNT_RATE_EVENT(&engineState->stats.contextMissCount);
  }
}

// **Обрабатывает пакет данных от якоря.**
// 
// `engineState`: Указатель на структуру состояния движка TDoA.
// `anchorCtx`: Контекст якоря.
// `txAn_in_cl_An`: Время передачи пакета якорем в системе отсчета часов якоря.
// `rxAn_by_T_in_cl_T`: Время приема пакета тегом в системе отсчета часов тега.
void tdoaEngineProcessPacket(tdoaEngineState_t* engineState, tdoaAnchorContext_t* anchorCtx, const int64_t txAn_in_cl_An, const int64_t rxAn_by_T_in_cl_T) {
  // Вызывает функцию обработки пакета без фильтрации.
  // Фильтрация позволяет исключить определенные якори из 
  // процесса сопоставления для вычисления TDoA.
  tdoaEngineProcessPacketFiltered(engineState, anchorCtx, txAn_in_cl_An, rxAn_by_T_in_cl_T, false, 0);
}

// **Обрабатывает пакет данных от якоря с возможностью фильтрации по ID якоря.**
// 
// `engineState`: Указатель на структуру состояния движка TDoA.
// `anchorCtx`: Контекст якоря.
// `txAn_in_cl_An`: Время передачи пакета якорем в системе отсчета часов якоря.
// `rxAn_by_T_in_cl_T`: Время приема пакета тегом в системе отсчета часов тега.
// `doExcludeId`: Флаг, указывающий, нужно ли исключать определенный ID якоря из списка кандидатов.
// `excludedId`: ID якоря, который нужно исключить, если `doExcludeId` равен `true`.
// 
// **Возвращает:** `true`, если измерение времени считается надежным, `false` в противном случае.
bool tdoaEngineProcessPacketFiltered(tdoaEngineState_t* engineState, tdoaAnchorContext_t* anchorCtx, const int64_t txAn_in_cl_An, const int64_t rxAn_by_T_in_cl_T, const bool doExcludeId, const uint8_t excludedId) {
  // Обновляет коррекцию часов для якоря, используя данные из пакета 
  // (время передачи и приема). 
  // Функция `updateClockCorrection` возвращает `true`, если 
  // измерение времени считается надежным.
  bool timeIsGood = updateClockCorrection(anchorCtx, txAn_in_cl_An, rxAn_by_T_in_cl_T, &engineState->stats);
  // Если измерение времени надежное...
  if (timeIsGood) { 
    // ...то увеличивает счетчик надежных измерений времени.
    STATS_CNT_RATE_EVENT(&engineState->stats.timeIsGood);

    tdoaAnchorContext_t otherAnchorCtx;
    // Находит подходящий якорь для сопоставления с текущим якорем, 
    // используя выбранный алгоритм сопоставления и, 
    // при необходимости, фильтруя по ID.
    if (findSuitableAnchor(engineState, &otherAnchorCtx, anchorCtx, doExcludeId, excludedId)) {
      // Увеличивает счетчик найденных подходящих данных.
      STATS_CNT_RATE_EVENT(&engineState->stats.suitableDataFound);
      // Вычисляет разницу в расстоянии между тегом и двумя якорями, 
      // используя TDoA и частоту таймера Locodeck.
      double tdoaDistDiff = calcDistanceDiff(&otherAnchorCtx, anchorCtx, txAn_in_cl_An, rxAn_by_T_in_cl_T, engineState->locodeckTsFreq);
      // Добавляет данные TDoA (разницу в расстоянии, ID якорей, позиции якорей) 
      // в очередь для отправки в модуль оценки местоположения.
      enqueueTDOA(&otherAnchorCtx, anchorCtx, tdoaDistDiff, engineState);
    }
  }
  // Возвращает флаг надежности измерения времени.
  return timeIsGood;  
}