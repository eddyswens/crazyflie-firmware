#include "clockCorrectionEngine.h"

// Максимальное допустимое отклонение часов (в частях на миллион).
#define MAX_CLOCK_DEVIATION_SPEC 10e-6
// Минимальное допустимое значение коррекции часов. 
// Вычисляется как 1 - (удвоенное максимальное отклонение), 
// чтобы учесть потенциальные ошибки в обе стороны.
#define CLOCK_CORRECTION_SPEC_MIN (1.0 - MAX_CLOCK_DEVIATION_SPEC * 2)
// Максимальное допустимое значение коррекции часов.
// Вычисляется как 1 + (удвоенное максимальное отклонение), 
// чтобы учесть потенциальные ошибки в обе стороны.
#define CLOCK_CORRECTION_SPEC_MAX (1.0 + MAX_CLOCK_DEVIATION_SPEC * 2)

// Допустимый уровень шума для коррекции часов (в частях на миллион).
// Если разница между новым значением коррекции и текущим значением меньше этого порога, 
// то новое значение считается "близким" к текущему и может быть использовано для 
// уточнения текущей коррекции с помощью фильтра нижних частот.
#define CLOCK_CORRECTION_ACCEPTED_NOISE 0.03e-6
// Коэффициент фильтра нижних частот для сглаживания коррекции часов.
// Чем ближе значение к 1, тем медленнее реагирует фильтр на изменения 
// и тем сильнее сглаживаются колебания.
#define CLOCK_CORRECTION_FILTER 0.1
// Максимальное значение счетчика "дырявого ведра" (leaky bucket).
// "Дырявое ведро" используется для ограничения скорости изменения опорной 
// коррекции часов. Чем больше значение, тем медленнее меняется опорная коррекция.
#define CLOCK_CORRECTION_BUCKET_MAX 4

/**
 Логирование всей информации о коррекции часов требует многократного масштабирования значений, 
 что является интенсивной вычислительной операцией. 
 Таким образом, функциональность логирования включается во время компиляции 
 с помощью флага CLOCK_CORRECTION_ENABLE_LOGGING.
 */
#ifdef CLOCK_CORRECTION_ENABLE_LOGGING
#include "log.h"

// Переменные для хранения значений, которые будут записаны в лог.
static float logMinAcceptedNoiseLimit = 0;
static float logMaxAcceptedNoiseLimit = 0;
static float logMinSpecLimit = 0;
static float logMaxSpecLimit = 0;
static float logClockCorrection = 0;
static float logClockCorrectionCandidate = 0;

// Масштабирует значение коррекции часов для логирования.
// Масштабирование позволяет представить значения коррекции 
// в более удобном для восприятия формате (в промилле).
static float scaleValueForLogging(double value) {
  return (float)((value - 1) * (1 / MAX_CLOCK_DEVIATION_SPEC) * 1000);
}
#endif

/**
 Получает коррекцию часов из объекта clockCorrectionStorage_t. 
 */
double clockCorrectionEngineGet(const clockCorrectionStorage_t* storage) {
  return storage->clockCorrection;
}

/**
 Обрезает временную метку до количества бит, указанных в маске. 
 Это обрезание гарантирует, что возвращаемое значение является 
 допустимым временным событием, даже если счетчик времени 
 в какой-то момент переполнился. 
 
 Например, если счетчик времени 32-битный, а маска равна 0xFFFFFFFF, 
 то функция вернет неизмененную временную метку. 
 Если же маска равна 0xFFFF, то функция вернет только младшие 16 бит 
 временной метки, отбросив старшие 16 бит.
 */
static uint64_t truncateTimeStamp(uint64_t timeStamp, uint64_t mask) {
  return timeStamp & mask;
}

/**
 Реализация алгоритма "дырявого ведра". 
 
 Эта функция увеличивает счетчик "ведра" на 1, 
 если он еще не достиг максимального значения.
 "Ведро" используется для ограничения скорости изменения опорной 
 коррекции часов.
 */
static void fillClockCorrectionBucket(clockCorrectionStorage_t* storage) {
  if (storage->clockCorrectionBucket < CLOCK_CORRECTION_BUCKET_MAX) {
    storage->clockCorrectionBucket++;
  }
}

/**
 Реализация алгоритма "дырявого ведра". 
 
 Эта функция уменьшает счетчик "ведра" на 1, 
 если он больше 0. Возвращает `true`, если счетчик стал равен 0,
 что означает, что "ведро" пустое и можно принимать новое 
 опорное значение коррекции часов.
 */
static bool emptyClockCorrectionBucket(clockCorrectionStorage_t* storage) {
  if (storage->clockCorrectionBucket > 0) {
    storage->clockCorrectionBucket--;
    return false;
  }

  return true;
}

/**
 Вычисляет коррекцию часов между опорными часами и другими часами (x).
 
 Коррекция часов - это коэффициент, на который нужно умножить временную метку, 
 измеренную часами x, чтобы получить ее значение, как если бы измерение 
 было выполнено опорными часами.
 
 Формула для вычисления коррекции часов:
 
 clockCorrection = (tickCount_in_cl_reference) / (tickCount_in_cl_x)
 
 где:
 
 * tickCount_in_cl_reference - количество тиков, прошедших между двумя событиями, 
   измеренное опорными часами.
 * tickCount_in_cl_x - количество тиков, прошедших между двумя событиями, 
   измеренное часами x.
 
 @param new_t_in_cl_reference Новейшее время возникновения события (t), измеренное опорными часами.
 @param old_t_in_cl_reference Более старое время возникновения события (t), измеренное опорными часами.
 @param new_t_in_cl_x Новейшее время возникновения события (t), измеренное часами x.
 @param old_t_in_cl_x Предыдущее время возникновения события (t), измеренное часами x.
 @param mask Маска длиной, равной количеству бит, используемых для представления временных меток. 
   Используется для вычисления допустимой временной метки, даже если произошло переполнение 
   счетчика времени.
 @return Необходимая коррекция часов, которую нужно применить к временным меткам, 
   измеренным часами x, чтобы получить их значение, как если бы измерение было выполнено 
   опорными часами. Или -1, если выполнить вычисление не удалось. 
   Пример: timestamp_in_cl_reference = clockCorrection * timestamp_in_cl_x
 */
double clockCorrectionEngineCalculate(const uint64_t new_t_in_cl_reference, const uint64_t old_t_in_cl_reference, const uint64_t new_t_in_cl_x, const uint64_t old_t_in_cl_x, const uint64_t mask) {
  // Вычисляет количество тиков, прошедших между двумя событиями, 
  // измеренное опорными часами и часами x.
  // Функция `truncateTimeStamp` используется для обработки 
  // возможных переполнений счетчика времени.
  const uint64_t tickCount_in_cl_reference = truncateTimeStamp(new_t_in_cl_reference - old_t_in_cl_reference, mask);
  const uint64_t tickCount_in_cl_x = truncateTimeStamp(new_t_in_cl_x - old_t_in_cl_x, mask);

  // Если количество тиков, измеренное часами x, равно 0, 
  // то коррекцию часов вычислить невозможно, и функция возвращает -1.
  if (tickCount_in_cl_x == 0) {
    return -1;
  }

  // Вычисляет коррекцию часов как отношение количества тиков, 
  // измеренных опорными часами, к количеству тиков, измеренных часами x.
  return (double)tickCount_in_cl_reference / (double)tickCount_in_cl_x;
}

/**
 Обновляет коррекцию часов, только если предоставленное значение 
 соответствует определенным условиям. 
 Это используется для отбрасывания неверных измерений коррекции часов 
 и плавного изменения текущей коррекции.
 
 Алгоритм работы функции:
 
 1. Вычисляет разницу между новым значением коррекции (clockCorrectionCandidate) 
    и текущим значением (currentClockCorrection).
 2. Если разница меньше допустимого уровня шума (CLOCK_CORRECTION_ACCEPTED_NOISE):
    * Применяет фильтр нижних частот для сглаживания коррекции часов. 
      Новое значение коррекции вычисляется как взвешенная сумма текущего значения 
      и нового значения, где веса определяются коэффициентом фильтра 
      (CLOCK_CORRECTION_FILTER).
    * Увеличивает счетчик "дырявого ведра" (fillClockCorrectionBucket).
    * Обновляет текущее значение коррекции часов.
    * Помечает образец как надежный (sampleIsReliable = true).
 3. Если разница больше допустимого уровня шума:
    * Проверяет, нужно ли принять новое значение как опорное значение коррекции. 
      Это делается с помощью алгоритма "дырявого ведра" (emptyClockCorrectionBucket), 
      который ограничивает скорость изменения опорного значения.
    * Если "ведро" пустое (счетчик равен 0) и новое значение коррекции находится 
      в допустимых пределах (CLOCK_CORRECTION_SPEC_MIN < clockCorrectionCandidate < CLOCK_CORRECTION_SPEC_MAX):
        * Принимает новое значение как опорное значение коррекции.
        * "Ведро" не заполняется, и образец не помечается как надежный, 
          т.к. это первое значение в новой серии измерений.
 4. Возвращает флаг надежности образца (sampleIsReliable).
 
 @return `true`, если предоставленный образец коррекции часов надежен, 
 `false` в противном случае. Образец считается надежным, 
 когда он находится в пределах допустимого уровня шума 
 (что означает, что у нас уже есть два или более образца, 
 которые похожи) и был отфильтрован фильтром нижних частот.
 */
bool clockCorrectionEngineUpdate(clockCorrectionStorage_t* storage, const double clockCorrectionCandidate) {
  bool sampleIsReliable = false;

  // Получает текущее значение коррекции часов.
  const double currentClockCorrection = storage->clockCorrection;
  // Вычисляет разницу между новым значением коррекции и текущим значением.
  const double difference = clockCorrectionCandidate - currentClockCorrection;

  // Записывает значения в переменные для логирования 
  // (если логирование включено).
#ifdef CLOCK_CORRECTION_ENABLE_LOGGING
  logMinAcceptedNoiseLimit = scaleValueForLogging(currentClockCorrection - CLOCK_CORRECTION_ACCEPTED_NOISE);
  logMaxAcceptedNoiseLimit = scaleValueForLogging(currentClockCorrection + CLOCK_CORRECTION_ACCEPTED_NOISE);
  logMinSpecLimit = scaleValueForLogging(CLOCK_CORRECTION_SPEC_MIN);
  logMaxSpecLimit = scaleValueForLogging(CLOCK_CORRECTION_SPEC_MAX);
  logClockCorrection = scaleValueForLogging(currentClockCorrection);
  logClockCorrectionCandidate = scaleValueForLogging(clockCorrectionCandidate);
#endif

  // Если разница между новым значением коррекции и текущим значением 
  // меньше допустимого уровня шума...
  if (-CLOCK_CORRECTION_ACCEPTED_NOISE < difference && difference < CLOCK_CORRECTION_ACCEPTED_NOISE) {
    // ...то применяет фильтр нижних частот 
    // для сглаживания коррекции часов.
    const double newClockCorrection = currentClockCorrection * CLOCK_CORRECTION_FILTER + clockCorrectionCandidate * (1.0 - CLOCK_CORRECTION_FILTER);

    // Помечает образец как надежный.
    sampleIsReliable = true;
    // Увеличивает счетчик "дырявого ведра".
    fillClockCorrectionBucket(storage);
    // Обновляет значение коррекции часов.
    storage->clockCorrection = newClockCorrection;
  } else {
    // Если разница между новым значением коррекции и текущим значением 
    // больше допустимого уровня шума...
    
    // ...то проверяет, нужно ли принять новое значение 
    // как опорное значение коррекции. 
    // Это делается с помощью алгоритма "дырявого ведра", 
    // который ограничивает скорость изменения опорного значения.
    const bool shouldAcceptANewClockReference = emptyClockCorrectionBucket(storage);
    // Если "ведро" пустое (т.е. счетчик равен 0)...
    if (shouldAcceptANewClockReference) {
      // ...и новое значение коррекции находится 
      // в допустимых пределах...
      if (CLOCK_CORRECTION_SPEC_MIN < clockCorrectionCandidate && clockCorrectionCandidate < CLOCK_CORRECTION_SPEC_MAX) {
        // ...то принимает новое значение как опорное значение коррекции.
        // "Ведро" не заполняется, и образец не помечается как надежный, 
        // т.к. это первое значение в новой серии измерений.
        storage->clockCorrection = clockCorrectionCandidate;
      }
    }
  }

  // Возвращает флаг надежности образца.
  return sampleIsReliable;
}

// Определяет группу логов для коррекции часов
#ifdef CLOCK_CORRECTION_ENABLE_LOGGING
LOG_GROUP_START(CkCorrection)
LOG_ADD(LOG_FLOAT, minNoise, &logMinAcceptedNoiseLimit)
LOG_ADD(LOG_FLOAT, maxNoise, &logMaxAcceptedNoiseLimit)
LOG_ADD(LOG_FLOAT, minSpec, &logMinSpecLimit)
LOG_ADD(LOG_FLOAT, maxSpec, &logMaxSpecLimit)
LOG_ADD(LOG_FLOAT, actualValue, &logClockCorrection)
LOG_ADD(LOG_FLOAT, candidate, &logClockCorrectionCandidate)
LOG_GROUP_STOP(CkCorrection)
#endif