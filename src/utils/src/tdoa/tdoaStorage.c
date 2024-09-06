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
Data storage encapsulation for the TDoA engine
*/

#include <string.h>

#define DEBUG_MODULE "TDOA_STORAGE"
#include "debug.h"

#include "tdoaStorage.h"

// All times in milli seconds
#define TOF_VALIDITY_PERIOD (2 * 1000)
#define REMOTE_DATA_VALIDITY_PERIOD 30
#define ANCHOR_POSITION_VALIDITY_PERIOD (2 * 1000)
#define ANCHOR_ACTIVE_VALIDITY_PERIOD (2 * 1000)


static tdoaAnchorInfo_t* initializeSlot(tdoaAnchorInfo_t anchorStorage[], const uint8_t slot, const uint8_t anchor);

// Инициализирует хранилище информации о якорях, обнуляя все его данные.
// Функция memset используется для заполнения памяти, занимаемой массивом anchorStorage, нулями.
void tdoaStorageInitialize(tdoaAnchorInfo_t anchorStorage[]) {
  memset(anchorStorage, 0, sizeof(tdoaAnchorInfo_t) * ANCHOR_STORAGE_COUNT);
}

// Ищет информацию о якоре в хранилище и, если не находит, 
// создает новую запись для этого якоря. Она возвращает true, если якорь найден, 
// и false, если пришлось создать новую запись.

// anchorStorage[]: массив структур типа tdoaAnchorInfo_t, 
// представляющий хранилище данных о якорях.
// anchor: идентификатор якоря, который необходимо найти или создать.
// currentTime_ms: текущее время в миллисекундах, которое используется 
// для обновления информации о якоре.
// anchorCtx: указатель на структуру tdoaAnchorContext_t, 
// в которую будет записана информация о найденном или созданном якоре.
bool tdoaStorageGetCreateAnchorCtx(tdoaAnchorInfo_t anchorStorage[], const uint8_t anchor, const uint32_t currentTime_ms, tdoaAnchorContext_t* anchorCtx) {
  anchorCtx->currentTime_ms = currentTime_ms;  // Установка текущего времени в контексте анкера
  uint32_t oldestUpdateTime = currentTime_ms;  // Самое старое время обновления
  int firstUninitializedSlot = -1;  // Первый неинициализированный слот
  int oldestSlot = 0;  // Самый старый слот

  for (int i = 0; i < ANCHOR_STORAGE_COUNT; i++) {  // Проход по всем элементам массива anchorStorage
    if (anchorStorage[i].isInitialized) {  // Если запись уже инициализирована
      if (anchor == anchorStorage[i].id) {  // Проверка, совпадает ли айди с искомым
        anchorCtx->anchorInfo = &anchorStorage[i];  // Записывает ее
        return true;  // Возвращает тру
      }

      if (anchorStorage[i].lastUpdateTime < oldestUpdateTime) {  // Если текущее время больше, чем время последнего обновления у записи
        oldestUpdateTime = anchorStorage[i].lastUpdateTime;  // Обновить время обновления
        oldestSlot = i;  // Занести айди записи как последнюю обновленную
      }
    } else {  // Если слот не инициализирован
      if (firstUninitializedSlot == -1) {  // Сохраняет индекс первого такого слота
        firstUninitializedSlot = i;
      }
    }
  }

  // The anchor was not found in storage
  // Запись анкера НЕ была найдена
  tdoaAnchorInfo_t* newAnchorInfo = 0;
  if (firstUninitializedSlot != -1) {  // Если найден хоть один неициализированный слот
    newAnchorInfo = initializeSlot(anchorStorage, firstUninitializedSlot, anchor);  // Инициализация первого доступного слота
  } else {  // Иначе
    newAnchorInfo = initializeSlot(anchorStorage, oldestSlot, anchor);  // Используем самый старый слот
  }

  anchorCtx->anchorInfo = newAnchorInfo;  // Передаем новую запись в указатель
  return false;  // Возвращаем фолс - пришлось создать запись
}

// Ищет информацию о анкере, но в случае ненахода, просто записывает 0
bool tdoaStorageGetAnchorCtx(tdoaAnchorInfo_t anchorStorage[], const uint8_t anchor, const uint32_t currentTime_ms, tdoaAnchorContext_t* anchorCtx) {
  anchorCtx->currentTime_ms = currentTime_ms;  // Текущее время

  for (int i = 0; i < ANCHOR_STORAGE_COUNT; i++) {  // Перебор элементов массива anchorStorage
    if (anchorStorage[i].isInitialized) {  // Если запись существует (инициализирована)
      if (anchor == anchorStorage[i].id) {  // Если это искомый айди
        anchorCtx->anchorInfo = &anchorStorage[i];  // Записать ссылку на эту запись
        return true;  // Вернуть тру
      }
    }
  }

  anchorCtx->anchorInfo = 0;  // Иначе отдаем 0
  return false;  // Ретурн фолс
}

// Получить список записанных анкеров
uint8_t tdoaStorageGetListOfAnchorIds(tdoaAnchorInfo_t anchorStorage[], uint8_t unorderedAnchorList[], const int maxListSize) {
  int count = 0;  // Счетчик начальное 0

  for (int i = 0; i < ANCHOR_STORAGE_COUNT && count < maxListSize; i++) {  // Перебор всего массива или пока не достигнем заданного предела
    if (anchorStorage[i].isInitialized) {  // Если запись существует
      unorderedAnchorList[count] = anchorStorage[i].id;  // Записываем айдишник в массив без сортировки
      count++;  // Счетчик +1
    }
  }

  return count;  // Возвращаем количество
}

// Получить список записанных активных анкеров
uint8_t tdoaStorageGetListOfActiveAnchorIds(tdoaAnchorInfo_t anchorStorage[], uint8_t unorderedAnchorList[], const int maxListSize, const uint32_t currentTime_ms) {
  int count = 0;  // Счетчик начальное 0

  const uint32_t expiryTime = currentTime_ms - ANCHOR_ACTIVE_VALIDITY_PERIOD;  // Вычисляем время просрочки активности
  for (int i = 0; i < ANCHOR_STORAGE_COUNT && count < maxListSize; i++) {  // Перебор массива, но не больше требуемого количества активных записей
    if (anchorStorage[i].isInitialized && anchorStorage[i].lastUpdateTime > expiryTime) {  // Если запись инициализирована и еще не просрочена по времени
      unorderedAnchorList[count] = anchorStorage[i].id;  // Записать айди анкера в список
      count++;  // счетчик +1
    }
  }

  return count;  // вернуть количество
}

// Возвращает айди анкера из контекста
uint8_t tdoaStorageGetId(const tdoaAnchorContext_t* anchorCtx) {
  return anchorCtx->anchorInfo->id;
}

// Возвращает время получения анкера из контекста
int64_t tdoaStorageGetRxTime(const tdoaAnchorContext_t* anchorCtx) {
  return anchorCtx->anchorInfo->rxTime;
}

// Возвращает время отправки анкера из контекста
int64_t tdoaStorageGetTxTime(const tdoaAnchorContext_t* anchorCtx) {
  return anchorCtx->anchorInfo->txTime;
}

// Возвращает номер пакета анкера из контекста
uint8_t tdoaStorageGetSeqNr(const tdoaAnchorContext_t* anchorCtx) {
  return anchorCtx->anchorInfo->seqNr;
}

// Возвращает последнее время апдейта анкера из контекста
uint32_t tdoaStorageGetLastUpdateTime(const tdoaAnchorContext_t* anchorCtx) {
  return anchorCtx->anchorInfo->lastUpdateTime;
}

// Возвращает структуру коррекции времени анкера из контекста
clockCorrectionStorage_t* tdoaStorageGetClockCorrectionStorage(const tdoaAnchorContext_t* anchorCtx) {
  return &anchorCtx->anchorInfo->clockCorrectionStorage;
}

// Возвращает позицию анкера из контекста
bool tdoaStorageGetAnchorPosition(const tdoaAnchorContext_t* anchorCtx, point_t* position) {
  uint32_t now = anchorCtx->currentTime_ms;  // Текущее время

  int32_t validCreationTime = now - ANCHOR_POSITION_VALIDITY_PERIOD;  // Крайнее время актуальности позиции анкеров
  const tdoaAnchorInfo_t* anchorInfo = anchorCtx->anchorInfo;
  if ((int32_t)anchorInfo->position.timestamp > validCreationTime) {  // Если время больше, чем крайнее время
    position->timestamp = anchorInfo->position.timestamp;  // Возвращаем позицию и это время
    position->x = anchorInfo->position.x;
    position->y = anchorInfo->position.y;
    position->z = anchorInfo->position.z;
    return true;  // рет тру
  }

  return false;  // рет фолс
}

// Устанавливает новую позицию анкера
void tdoaStorageSetAnchorPosition(tdoaAnchorContext_t* anchorCtx, const float x, const float y, const float z) {
  uint32_t now = anchorCtx->currentTime_ms;  // Текущее время
  tdoaAnchorInfo_t* anchorInfo = anchorCtx->anchorInfo;

  anchorInfo->position.timestamp = now;  // Записываем новые значения
  anchorInfo->position.x = x;
  anchorInfo->position.y = y;
  anchorInfo->position.z = z;
}

// Устанаваливает новую rx и tx дату
void tdoaStorageSetRxTxData(tdoaAnchorContext_t* anchorCtx, int64_t rxTime, int64_t txTime, uint8_t seqNr) {
  uint32_t now = anchorCtx->currentTime_ms;  // текущее время
  tdoaAnchorInfo_t* anchorInfo = anchorCtx->anchorInfo;

  anchorInfo->rxTime = rxTime;  // Время получения
  anchorInfo->txTime = txTime;  // Время отправки
  anchorInfo->seqNr = seqNr;  // Номер пакета
  anchorInfo->lastUpdateTime = now;  // Время обнолвения (сейчас)
}

#ifdef CONFIG_DECK_LOCO_TDOA3_HYBRID_MODE
int64_t tdoaStorageGetTimeOfFlight(const tdoaAnchorContext_t* anchorCtx, const uint32_t oldestAcceptableTime_ms) {
  if (anchorCtx->anchorInfo->tofTime_ms < oldestAcceptableTime_ms) {
    return 0;
  }

  return anchorCtx->anchorInfo->tof;
}

void tdoaStorageSetTimeOfFlight(tdoaAnchorContext_t* anchorCtx, const int64_t tof, const uint32_t currentTime_ms) {
  anchorCtx->anchorInfo->tof = tof;
  anchorCtx->anchorInfo->tofTime_ms = currentTime_ms;
}

#endif

// Возвращает коррекцию времени
double tdoaStorageGetClockCorrection(const tdoaAnchorContext_t* anchorCtx) {
  return clockCorrectionEngineGet(&anchorCtx->anchorInfo->clockCorrectionStorage);  // Передается storage
}

// Возвращает время приема сигнала от удаленного анкера в контексте текущего???
int64_t tdoaStorageGetRemoteRxTime(const tdoaAnchorContext_t* anchorCtx, const uint8_t remoteAnchor) {
  int64_t rxTime;
  uint8_t seqNr;

  const bool foundAnchor = tdoaStorageGetRemoteRxTimeSeqNr(anchorCtx, remoteAnchor, &rxTime, &seqNr);
  if (foundAnchor) {
    return rxTime;  // Просто использует функцию, отбрасывая номер пакета
  }

  return 0;
}

// Возвращает время приема сигнала и номер пакета от удаленного анкера в контексте текущего???
bool tdoaStorageGetRemoteRxTimeSeqNr(const tdoaAnchorContext_t* anchorCtx, const uint8_t remoteAnchor, int64_t* rxTime, uint8_t* seqNr) {
  const tdoaAnchorInfo_t* anchorInfo = anchorCtx->anchorInfo;
  bool result = false;

  for (int i = 0; i < REMOTE_ANCHOR_DATA_COUNT; i++) {  // Проход по всем анкерам
    if (remoteAnchor == anchorInfo->remoteAnchorData[i].id) {  // Если айди совпал
      uint32_t now = anchorCtx->currentTime_ms;  // Запись текущего времени
      if (anchorInfo->remoteAnchorData[i].endOfLife > now) {  // Если данные еще не устарели
        *rxTime = anchorInfo->remoteAnchorData[i].rxTime;  // Отдать инфу о времени приема
        *seqNr = anchorInfo->remoteAnchorData[i].seqNr;  // Отдать инфу о номере пакета
        result = true;  // Успешный успех
      }
      break;
    }
  }

  return result;
}

// Обновляет время приема и номер пакета для удаленного анкера в контексте текущего
void tdoaStorageSetRemoteRxTime(tdoaAnchorContext_t* anchorCtx, const uint8_t remoteAnchor, const int64_t remoteRxTime, const uint8_t remoteSeqNr) {
  tdoaAnchorInfo_t* anchorInfo = anchorCtx->anchorInfo;

  int indexToUpdate = 0;  // Начальное состояние
  uint32_t now = anchorCtx->currentTime_ms;  // Текущее время
  uint32_t oldestTime = 0xFFFFFFFF;  // Самое старое время

  for (int i = 0; i < REMOTE_ANCHOR_DATA_COUNT; i++) {  // Проход по всем записям remoteAnchorData
    if (remoteAnchor == anchorInfo->remoteAnchorData[i].id) {  // Если нашли
      indexToUpdate = i;  // Записываем индекс, который надо обновить
      break;  // Выходим из цикла
    }

    if (anchorInfo->remoteAnchorData[i].endOfLife < oldestTime) {  // Поиск самого старого слота, чтобы в случае чего, перезаписать его
      oldestTime = anchorInfo->remoteAnchorData[i].endOfLife;
      indexToUpdate = i;
    }
  }

  anchorInfo->remoteAnchorData[indexToUpdate].id = remoteAnchor;  // Записываем новые данные
  anchorInfo->remoteAnchorData[indexToUpdate].rxTime = remoteRxTime;
  anchorInfo->remoteAnchorData[indexToUpdate].seqNr = remoteSeqNr;
  anchorInfo->remoteAnchorData[indexToUpdate].endOfLife = now + REMOTE_DATA_VALIDITY_PERIOD;  // И новый срок годности
}

// Возвращает последние полученные номер пакетов для каждого из удаленных анкеров
void tdoaStorageGetRemoteSeqNrList(const tdoaAnchorContext_t* anchorCtx, int* remoteCount, uint8_t seqNr[], uint8_t id[]) {
  const tdoaAnchorInfo_t* anchorInfo = anchorCtx->anchorInfo;
  uint32_t now = anchorCtx->currentTime_ms;  // Текущее время

  int count = 0;  // Счетчик начало

  for (int i = 0; i < REMOTE_ANCHOR_DATA_COUNT; i++) {  // Проход по всем анкерам в контексте текущего
    if (anchorInfo->remoteAnchorData[i].endOfLife > now) {  // Если срок годности нормальный
      id[count] = anchorInfo->remoteAnchorData[i].id;  // Записать айди в список
      seqNr[count] = anchorInfo->remoteAnchorData[i].seqNr;  // Записать номер пакета в список
      count++;  // Счетчик +1
    }
  }

  *remoteCount = count;  // Количество удаленных анкеров
}

// Возвращает ToF между текущим анкеромс и другим
int64_t tdoaStorageGetRemoteTimeOfFlight(const tdoaAnchorContext_t* anchorCtx, const uint8_t otherAnchor) {
  const tdoaAnchorInfo_t* anchorInfo = anchorCtx->anchorInfo;

  for (int i = 0; i < TOF_PER_ANCHOR_COUNT; i++) {  // По всем анкерам в контексте
    if (otherAnchor == anchorInfo->remoteTof[i].id) {  // Если Есть доступный ToF для заданного анкера
      uint32_t now = anchorCtx->currentTime_ms;  // Текущее время
      if (anchorInfo->remoteTof[i].endOfLife > now) {  // если срок годности нормальный
        return anchorInfo->remoteTof[i].tof;  // Отдаем значение ToF
      }
      break;
    }
  }

  return 0;
}

// Обновляет ToF для заданного анкера
void tdoaStorageSetRemoteTimeOfFlight(tdoaAnchorContext_t* anchorCtx, const uint8_t remoteAnchor, const int64_t tof) {
  tdoaAnchorInfo_t* anchorInfo = anchorCtx->anchorInfo;

  int indexToUpdate = 0;  // Индекс, который надо будет обновить
  uint32_t now = anchorCtx->currentTime_ms;  // Текущее время
  uint32_t oldestTime = 0xFFFFFFFF;  // Начальное самое старое время

  for (int i = 0; i < TOF_PER_ANCHOR_COUNT; i++) {  // Проход по всем анкерам в контексте текущего
    if (remoteAnchor == anchorInfo->remoteTof[i].id) {  // Если нашли нужную запись
      indexToUpdate = i;  // Записываем ее индекс
      break;  // Выходим с цикла
    }

    if (anchorInfo->remoteTof[i].endOfLife < oldestTime) {  // Находим самую старую запись
      oldestTime = anchorInfo->remoteTof[i].endOfLife;
      indexToUpdate = i;
    }
  }

  anchorInfo->remoteTof[indexToUpdate].id = remoteAnchor;  // Записываем новую инфу
  anchorInfo->remoteTof[indexToUpdate].tof = tof;
  anchorInfo->remoteTof[indexToUpdate].endOfLife = now + TOF_VALIDITY_PERIOD;  // И новый срок годности
}

// Проверяет. находится ли анкер в сторадже
bool tdoaStorageIsAnchorInStorage(tdoaAnchorInfo_t anchorStorage[], const uint8_t anchor) {
  for (int i = 0; i < ANCHOR_STORAGE_COUNT; i++) {  // Проход по всем анкерам
    if (anchorStorage[i].isInitialized) {  // Если запись инициализирована
      if (anchor == anchorStorage[i].id) {  // Если нужный анкер
        return true;  // Рет тру
      }
    }
  }

  return false;  // Рет фолс иначе
}

// Инициализирует слот под новую запись (Делает новую запись)
static tdoaAnchorInfo_t* initializeSlot(tdoaAnchorInfo_t anchorStorage[], const uint8_t slot, const uint8_t anchor) {
  memset(&anchorStorage[slot], 0, sizeof(tdoaAnchorInfo_t));  // Очищает ячейку памяти
  anchorStorage[slot].id = anchor;  // Записывает айди анкера
  anchorStorage[slot].isInitialized = true;  // Ставит флаг, что ячейка инициализированна

  return &anchorStorage[slot];  // Возвращает ссылку на новую запись
}
