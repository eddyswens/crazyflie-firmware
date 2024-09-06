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
 * lpsTdoaTagStats.c is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with lpsTdoaTagStats.c. If not, see <http://www.gnu.org/licenses/>.
 */

/*
Log statistics for the tdoa engine
*/

#include <string.h>

#include "tdoaStats.h"

// Интервал обновления статистики (в миллисекундах).
#define STATS_INTERVAL 500


// **Инициализирует структуру статистики TDoA.**
// 
// `tdoaStats`: Указатель на структуру статистики TDoA.
// `now_ms`: Текущее время в миллисекундах.
void tdoaStatsInit(tdoaStats_t* tdoaStats, uint32_t now_ms) {
  // Обнуляет все поля структуры статистики.
  memset(tdoaStats, 0, sizeof(tdoaStats_t));
  // Устанавливает начальные значения ID основного и удаленного якорей.
  tdoaStats->remoteAnchorId = tdoaStats->newRemoteAnchorId = 1;

  // Устанавливает время следующего обновления статистики.
  tdoaStats->nextStatisticsTime = now_ms + STATS_INTERVAL;
  // Устанавливает время предыдущего обновления статистики в 0.
  tdoaStats->previousStatisticsTime = 0;

  // Инициализирует счетчики событий с интервалом обновления `STATS_INTERVAL`.
  STATS_CNT_RATE_INIT(&tdoaStats->packetsReceived, STATS_INTERVAL);
  STATS_CNT_RATE_INIT(&tdoaStats->packetsToEstimator, STATS_INTERVAL);
  STATS_CNT_RATE_INIT(&tdoaStats->clockCorrectionCount, STATS_INTERVAL);
  STATS_CNT_RATE_INIT(&tdoaStats->contextHitCount, STATS_INTERVAL);
  STATS_CNT_RATE_INIT(&tdoaStats->contextMissCount, STATS_INTERVAL);
  STATS_CNT_RATE_INIT(&tdoaStats->timeIsGood, STATS_INTERVAL);
  STATS_CNT_RATE_INIT(&tdoaStats->suitableDataFound, STATS_INTERVAL);
}

// **Обновляет статистику TDoA.**
// 
// `tdoaStats`: Указатель на структуру статистики TDoA.
// `now_ms`: Текущее время в миллисекундах.
void tdoaStatsUpdate(tdoaStats_t* tdoaStats, uint32_t now_ms) {
  // Проверяет, пришло ли время обновить статистику.
  if (now_ms > tdoaStats->nextStatisticsTime) {
    // Если ID основного якоря изменился...
    if (tdoaStats->anchorId != tdoaStats->newAnchorId) {
      // ...то обновляет ID основного якоря...
      tdoaStats->anchorId = tdoaStats->newAnchorId;

      // ...и сбрасывает статистику, связанную с основным якорем.
      tdoaStats->clockCorrection = 0.0;
      tdoaStats->tof = 0;
      tdoaStats->tdoa = 0;
    }

    // Если ID удаленного якоря изменился...
    if (tdoaStats->remoteAnchorId != tdoaStats->newRemoteAnchorId) {
      // ...то обновляет ID удаленного якоря...
      tdoaStats->remoteAnchorId = tdoaStats->newRemoteAnchorId;

      // ...и сбрасывает статистику, связанную с удаленным якорем.
      tdoaStats->tof = 0;
      tdoaStats->tdoa = 0;
    }

    // Обновляет время предыдущего и следующего обновления статистики.
    tdoaStats->previousStatisticsTime = now_ms;
    tdoaStats->nextStatisticsTime = now_ms + STATS_INTERVAL;
  }
}