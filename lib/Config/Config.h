

#include <Arduino.h>

const float CMP_KP = 0.8;
const float COORD_KP = 2;
const float COORD_KI = 0;
const float COORD_KD = 1;

const int LIGHT_THRESH[32] = {844, 875, 886, 882, 882, 896, 899, 881, 889, 892, 880, 881, 900, 862, 888, 725,
                             900, 888, 890, 877, 892, 862, 894, 897, 884, 886, 892, 904, 887, 893, 869, 606};
