#include "ntc_table.h"

const temperature_table_entry_type termo_table[] PROGMEM = {
     4036, 4031, 4026, 4021, 4015, 4009, 4002, 3995,
    3988, 3980, 3972, 3963, 3954, 3944, 3934, 3923,
    3912, 3900, 3887, 3873, 3859, 3844, 3829, 3813,
    3796, 3778, 3759, 3739, 3719, 3698, 3676, 3653,
    3629, 3604, 3578, 3551, 3524, 3495, 3466, 3435,
    3404, 3372, 3338, 3304, 3269, 3233, 3197, 3159,
    3121, 3082, 3042, 3001, 2960, 2918, 2875, 2832,
    2789, 2745, 2700, 2656, 2611, 2565, 2519, 2474,
    2428, 2382, 2336, 2290, 2244, 2198, 2152, 2107,
    2062, 2017, 1972, 1928, 1884, 1841, 1798, 1755,
    1714, 1672, 1632, 1591, 1552, 1513, 1475, 1437,
    1401, 1364, 1329, 1294, 1260, 1227, 1194, 1162,
    1131, 1101, 1071, 1042, 1013, 986, 959, 933,
    907, 882, 858, 834, 811, 788, 767, 745,
    725, 705, 685, 666, 648, 630, 612, 596,
    579, 563, 548, 533, 518, 504, 490, 477,
    464, 451, 439, 427, 415, 404, 393, 383,
    373, 363, 353, 344, 335, 326, 317, 309,
    301, 293, 286, 278, 271, 264, 257, 251,
    245, 238, 232, 227, 221, 215, 210, 205,
    200, 195, 190, 186, 181, 177, 172, 168,
    164, 160, 157, 153, 149, 146, 142, 139,
    136, 133, 130, 127, 124, 121, 118, 116,
    113, 111, 108, 106, 104, 101, 99, 97,
    95, 93, 91, 89, 87, 85, 83, 82,
    80, 78, 77, 75, 74, 72, 71, 69,
    68, 66, 65, 64, 63, 61, 60, 59,
    58, 57, 56, 55, 53, 52, 51, 50,
    50, 49, 48, 47, 46, 45, 44, 43,
    43, 42, 41, 40, 40, 39, 38, 38,
    37, 36, 36, 35, 34, 34, 33, 33,
    32, 32, 31, 31, 30, 30, 29, 29,
    28, 28, 27, 27, 26, 26, 25, 25,
    25, 24, 24, 24, 23, 23, 22, 22,
    22, 21, 21, 21, 20, 20, 20, 20,
    19, 19, 19, 18, 18, 18, 18, 17,
    17, 17, 17, 16, 16, 16, 16, 15,
    15, 15, 15, 15, 14, 14, 14, 14,
    14, 13, 13, 13, 13, 13, 13, 12,
    12, 12, 12, 12, 12, 11, 11, 11,
    11, 11, 11, 11, 10, 10, 10, 10,
    10, 10, 10, 10, 9, 9, 9, 9,
    9, 9, 9, 9, 9, 8, 8, 8,
    8, 8, 8, 8, 8, 8, 8

};


int16_t calc_temperature(temperature_table_entry_type adcsum) {
  temperature_table_index_type l = 0;
  temperature_table_index_type r = (sizeof(termo_table) / sizeof(termo_table[0])) - 1;
  temperature_table_entry_type thigh = TEMPERATURE_TABLE_READ(r);
  
  // Проверка выхода за пределы и граничных значений
  if (adcsum <= thigh) {
    #ifdef TEMPERATURE_OVER
      if (adcsum < thigh) 
        return TEMPERATURE_OVER;
    #endif
     return TEMPERATURE_TABLE_STEP * r + TEMPERATURE_TABLE_START;
  }
  temperature_table_entry_type tlow = TEMPERATURE_TABLE_READ(0);
  if (adcsum >= tlow) {
    #ifdef TEMPERATURE_UNDER
      if (adcsum > tlow)
        return TEMPERATURE_UNDER;
    #endif
    return TEMPERATURE_TABLE_START;
  }

  // Двоичный поиск по таблице
  while ((r - l) > 1) {
    temperature_table_index_type m = (l + r) >> 1;
    temperature_table_entry_type mid = TEMPERATURE_TABLE_READ(m);
    if (adcsum > mid) {
      r = m;
    } else {
      l = m;
    }
  }
  temperature_table_entry_type vl = TEMPERATURE_TABLE_READ(l);
  if (adcsum >= vl) {
    return l * TEMPERATURE_TABLE_STEP + TEMPERATURE_TABLE_START;
  }
  temperature_table_entry_type vr = TEMPERATURE_TABLE_READ(r);
  temperature_table_entry_type vd = vl - vr;
  int16_t res = TEMPERATURE_TABLE_START + r * TEMPERATURE_TABLE_STEP; 
  if (vd) {
    // Линейная интерполяция
    res -= ((TEMPERATURE_TABLE_STEP * (int32_t)(adcsum - vr) + (vd >> 1)) / vd);
  }
  return res;
}
