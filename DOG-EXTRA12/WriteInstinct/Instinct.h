#define WalkingDOF 8
#define NUM_SKILLS 42

//#define I2C_EEPROM
//#include "avr/pgmspace.h"

const char bd[] PROGMEM = { //store in flash memory
31, 0, 0,
 18, 18,-86,-86, 18, 18,  3,  3,
 26, 26,-79,-79, 20, 20, -6, -6,
 32, 32,-72,-72, 27, 27,-14,-14,
 33, 33,-65,-65, 40, 40,-19,-19,
 30, 30,-61,-61, 54, 54,-20,-20,
 33, 33,-58,-58, 56, 56,-17,-17,
 39, 39,-55,-55, 51, 51,-16,-16,
 48, 48,-52,-52, 40, 40,-15,-15,
 57, 57,-49,-49, 29, 29,-14,-14,
 65, 65,-44,-44, 16, 16,-15,-15,
 72, 72,-39,-39,  4,  4,-16,-16,
 76, 76,-35,-35, -5, -5,-18,-18,
 78, 78,-29,-29,-14,-14,-20,-20,
 76, 76,-26,-26,-22,-22,-17,-17,
 69, 69,-26,-26,-32,-32,-10,-10,
 56, 56,-28,-28,-37,-37, -2, -2,
 39, 39,-32,-32,-37,-37,  8,  8,
 20, 20,-36,-36,-32,-32, 17, 17,
  2,  2,-43,-43,-23,-23, 25, 25,
 -9, -9,-51,-51,-15,-15, 32, 32,
-16,-16,-60,-60, -6, -6, 37, 37,
-22,-22,-66,-66,  3,  3, 39, 39,
-27,-27,-72,-72, 15, 15, 40, 40,
-32,-32,-77,-77, 27, 27, 40, 40,
-35,-35,-82,-82, 39, 39, 40, 40,
-38,-38,-87,-87, 49, 49, 39, 39,
-36,-36,-92,-92, 54, 54, 38, 38,
-31,-31,-97,-97, 51, 51, 35, 35,
-19,-19,-99,-99, 38, 38, 29, 29,
 -5, -5,-97,-97, 26, 26, 21, 21,
  8,  8,-93,-93, 20, 20, 13, 13,
};
const char bk[] PROGMEM = { 
37, 0, 0,
 30, 39,-57,-64,  6, -9, -6,  9,
 27, 51,-58,-55,  8,-11, -8, 11,
 24, 61,-60,-43,  9,-10, -9, 10,
 21, 66,-61,-36, 11, -8,-11,  7,
 18, 66,-62,-31, 13, -4,-13,  4,
 14, 64,-63,-28, 16,  1,-16, -1,
 11, 61,-63,-27, 18,  6,-18, -6,
  8, 57,-64,-28, 21,  7,-21, -7,
  4, 56,-64,-31, 24,  6,-24, -6,
  0, 54,-64,-34, 28,  5,-28, -5,
 -3, 52,-64,-37, 31,  4,-32, -4,
 -8, 50,-63,-40, 37,  4,-37, -4,
-10, 48,-64,-43, 38,  3,-38, -3,
-10, 45,-68,-45, 34,  3,-34, -3,
 -6, 43,-72,-48, 26,  3,-26, -3,
 -2, 40,-75,-50, 19,  3,-19, -4,
  6, 37,-75,-52, 12,  4,-12, -4,
 20, 34,-73,-54,  1,  5, -1, -5,
 33, 31,-68,-56, -6,  6,  6, -6,
 45, 28,-59,-58,-10,  7, 10, -7,
 56, 25,-49,-59,-11,  9, 11, -9,
 65, 22,-37,-60, -8, 11,  8,-10,
 66, 19,-33,-61, -6, 13,  6,-13,
 66, 16,-30,-63, -2, 15,  2,-15,
 63, 12,-27,-63,  3, 17, -4,-17,
 59,  9,-27,-64,  7, 20, -7,-20,
 57,  5,-30,-64,  7, 23, -7,-23,
 55,  2,-33,-64,  5, 26, -5,-26,
 53, -2,-36,-64,  5, 30, -5,-30,
 51, -6,-39,-64,  4, 34, -4,-34,
 49,-10,-41,-64,  3, 38, -3,-38,
 46,-10,-44,-67,  3, 35, -3,-35,
 44, -8,-46,-71,  3, 29, -3,-29,
 41, -4,-49,-74,  3, 22, -3,-22,
 39,  1,-51,-75,  4, 16, -4,-16,
 36, 15,-53,-74,  5,  4, -5, -4,
 33, 28,-55,-70,  5, -4, -5,  4,
};
const char bkL[] PROGMEM = { 
37, 0, 0,
 32, 39,-57,-61,  3, -9, -6,  2,
 31, 51,-58,-59,  3,-11, -8,  4,
 30, 61,-60,-56,  3,-10, -9,  5,
 29, 66,-61,-54,  4, -8,-11,  5,
 28, 66,-62,-53,  5, -4,-13,  4,
 27, 64,-63,-51,  5,  1,-16,  3,
 26, 61,-63,-50,  6,  6,-18,  1,
 25, 57,-64,-51,  6,  7,-21,  0,
 24, 56,-64,-51,  7,  6,-24,  0,
 23, 54,-64,-52,  7,  5,-28,  0,
 22, 52,-64,-53,  8,  4,-32,  0,
 20, 50,-63,-53,  9,  4,-37,  0,
 20, 48,-64,-54,  8,  3,-38, -1,
 20, 45,-68,-55,  8,  3,-34, -1,
 21, 43,-72,-56,  6,  3,-26, -1,
 22, 40,-75,-56,  4,  3,-19, -2,
 24, 37,-75,-57,  3,  4,-12, -2,
 29, 34,-73,-58,  0,  5, -1, -2,
 33, 31,-68,-58, -2,  6,  6, -3,
 37, 28,-59,-59, -3,  7, 10, -3,
 41, 25,-49,-60, -4,  9, 11, -3,
 44, 22,-37,-60, -5, 11,  8, -4,
 46, 19,-33,-61, -5, 13,  6, -5,
 45, 16,-30,-61, -4, 15,  2, -5,
 45, 12,-27,-62, -2, 17, -4, -6,
 44,  9,-27,-62,  0, 20, -7, -6,
 43,  5,-30,-63,  0, 23, -7, -7,
 42,  2,-33,-63,  0, 26, -5, -7,
 41, -2,-36,-64,  0, 30, -5, -8,
 40, -6,-39,-64,  0, 34, -4, -9,
 39,-10,-41,-64,  1, 38, -3, -9,
 38,-10,-44,-66,  1, 35, -3, -8,
 37, -8,-46,-67,  1, 29, -3, -6,
 36, -4,-49,-67,  1, 22, -3, -5,
 35,  1,-51,-67,  2, 16, -4, -3,
 34, 15,-53,-65,  2,  4, -5, -1,
 33, 28,-55,-64,  2, -4, -5,  1,
};
const char bkR[] PROGMEM = { 
37, 0, 0,
 30, 35,-58,-64,  6, -3, -3,  9,
 27, 39,-59,-55,  8, -4, -3, 11,
 24, 43,-60,-43,  9, -5, -3, 10,
 21, 45,-60,-36, 11, -5, -4,  7,
 18, 46,-61,-31, 13, -4, -5,  4,
 14, 45,-61,-28, 16, -3, -5, -1,
 11, 44,-62,-27, 18, -1, -6, -6,
  8, 43,-62,-28, 21,  0, -6, -7,
  4, 42,-63,-31, 24,  0, -7, -6,
  0, 42,-63,-34, 28,  0, -7, -5,
 -3, 41,-64,-37, 31,  0, -8, -4,
 -8, 40,-64,-40, 37,  1, -9, -4,
-10, 39,-65,-43, 38,  1, -8, -3,
-10, 38,-66,-45, 34,  1, -8, -3,
 -6, 37,-67,-48, 26,  1, -6, -3,
 -2, 36,-67,-50, 19,  1, -3, -4,
  6, 35,-67,-52, 12,  2, -3, -4,
 20, 34,-65,-54,  1,  2,  0, -5,
 33, 33,-63,-56, -6,  2,  2, -6,
 45, 31,-60,-58,-10,  3,  3, -7,
 56, 30,-57,-59,-11,  3,  4, -9,
 65, 29,-54,-60, -8,  4,  5,-10,
 66, 28,-54,-61, -6,  5,  5,-13,
 66, 27,-52,-63, -2,  5,  3,-15,
 63, 26,-51,-63,  3,  6,  2,-17,
 59, 25,-50,-64,  7,  6,  0,-20,
 57, 24,-51,-64,  7,  7,  0,-23,
 55, 23,-51,-64,  5,  7,  0,-26,
 53, 22,-52,-64,  5,  8,  0,-30,
 51, 21,-53,-64,  4,  9,  0,-34,
 49, 20,-54,-64,  3,  9,  0,-38,
 46, 20,-55,-67,  3,  8, -1,-35,
 44, 20,-55,-71,  3,  6, -1,-29,
 41, 21,-56,-74,  3,  5, -1,-22,
 39, 23,-57,-75,  4,  3, -2,-16,
 36, 27,-57,-74,  5,  1, -2, -4,
 33, 31,-58,-70,  5, -1, -2,  4,
};
const char cr[] PROGMEM = { 
26, 0, -5,
 35, 37,-46,-53,-23,-32, -3, 12,
 40, 28,-42,-59,-24,-28, -4, 12,
 45, 20,-38,-64,-25,-24, -5, 12,
 51, 12,-34,-69,-26,-19, -7, 10,
 56,  4,-30,-72,-27,-13, -9,  8,
 60, -5,-26,-71,-26,  1,-10, -4,
 64,  1,-21,-64,-26, -1,-14, -9,
 68,  6,-17,-62,-24, -5,-17, -6,
 74, 11,-23,-59,-34,-10, -2, -5,
 68, 16,-29,-57,-36,-13,  3, -4,
 60, 21,-36,-54,-37,-16,  6, -3,
 52, 27,-42,-51,-36,-20,  9, -3,
 44, 32,-48,-47,-34,-22, 11, -3,
 35, 37,-54,-44,-31,-23, 12, -3,
 26, 42,-59,-40,-27,-25, 12, -4,
 19, 48,-64,-36,-23,-26, 11, -6,
 11, 53,-69,-32,-17,-27, 10, -7,
  3, 58,-73,-28,-11,-26,  8,-10,
 -4, 62,-69,-23,  2,-26, -7,-13,
  2, 66,-64,-19, -2,-26, -8,-15,
  7, 75,-61,-16, -6,-29, -7,-12,
 12, 71,-59,-25,-10,-35, -5,  0,
 17, 64,-56,-32,-13,-36, -4,  4,
 22, 56,-54,-39,-16,-36, -2,  7,
 28, 48,-50,-45,-19,-35, -3,  9,
 33, 39,-47,-51,-22,-32, -3, 11,
};
const char crL[] PROGMEM = { 
26, 0, -5,
 35, 37,-46,-50,-25,-32, -3,  6,
 37, 28,-42,-52,-26,-28, -4,  6,
 39, 20,-38,-53,-26,-24, -5,  6,
 41, 12,-34,-55,-26,-19, -7,  7,
 43,  4,-30,-57,-27,-13, -9,  6,
 45, -5,-26,-58,-28,  1,-10,  3,
 47,  1,-21,-55,-28, -1,-14,  1,
 48,  6,-17,-54,-28, -5,-17,  1,
 50, 11,-23,-53,-32,-10, -2,  2,
 48, 16,-29,-52,-32,-13,  3,  2,
 45, 21,-36,-51,-31,-16,  6,  2,
 42, 27,-42,-50,-30,-20,  9,  1,
 39, 32,-48,-48,-30,-22, 11,  1,
 37, 37,-54,-47,-28,-23, 12,  1,
 34, 42,-59,-46,-27,-25, 12,  1,
 31, 48,-64,-44,-26,-26, 11,  0,
 29, 53,-69,-43,-25,-27, 10,  0,
 26, 58,-73,-42,-23,-26,  8,  0,
 22, 62,-69,-41,-19,-26, -7,  0,
 24, 66,-64,-40,-19,-26, -8, -1,
 26, 75,-61,-38,-20,-29, -7,  1,
 27, 71,-59,-41,-21,-35, -5,  4,
 29, 64,-56,-43,-22,-36, -4,  4,
 31, 56,-54,-46,-23,-36, -2,  5,
 33, 48,-50,-47,-24,-35, -3,  5,
 35, 39,-47,-49,-24,-32, -3,  5,
};
const char crR[] PROGMEM = { 
26, 0, -5,
 35, 37,-48,-53,-23,-28,  1, 12,
 40, 34,-47,-59,-24,-27,  1, 12,
 45, 31,-46,-64,-25,-26,  1, 12,
 51, 29,-44,-69,-26,-25,  0, 10,
 56, 26,-43,-72,-27,-23,  0,  8,
 60, 22,-42,-71,-26,-20,  0, -4,
 64, 24,-40,-64,-26,-19,  0, -9,
 68, 26,-38,-62,-24,-20, -1, -6,
 74, 27,-40,-59,-34,-21,  3, -5,
 68, 29,-42,-57,-36,-22,  4, -4,
 60, 31,-44,-54,-37,-23,  4, -3,
 52, 33,-47,-51,-36,-24,  5, -3,
 44, 35,-48,-47,-34,-24,  5, -3,
 35, 36,-50,-44,-31,-25,  6, -3,
 26, 38,-52,-40,-27,-26,  6, -4,
 19, 40,-54,-36,-23,-26,  6, -6,
 11, 42,-55,-32,-17,-27,  7, -7,
  3, 44,-57,-28,-11,-27,  6,-10,
 -4, 45,-57,-23,  2,-28,  2,-13,
  2, 47,-55,-19, -2,-29,  1,-15,
  7, 51,-54,-16, -6,-31,  1,-12,
 12, 49,-53,-25,-10,-32,  2,  0,
 17, 47,-52,-32,-13,-31,  2,  4,
 22, 44,-51,-39,-16,-30,  2,  7,
 28, 41,-50,-45,-19,-30,  1,  9,
 33, 38,-48,-51,-22,-29,  1, 11,
};
const char ly[] PROGMEM = { 
20, 0, -20,
114,117,-45,-53, 52, 49,-38,-24,
114,117,-39,-58, 52, 49,-42,-23,
114,117,-34,-62, 52, 49,-46,-21,
114,116,-26,-66, 52, 48,-54,-22,
114,116,-22,-66, 54, 48,-59,-25,
114,116,-25,-64, 54, 48,-50,-30,
115,115,-30,-60, 52, 50,-42,-34,
116,115,-35,-58, 50, 50,-36,-34,
116,115,-42,-54, 50, 50,-31,-34,
116,115,-47,-49, 50, 50,-28,-36,
117,114,-53,-43, 49, 52,-25,-39,
117,114,-58,-37, 49, 52,-22,-44,
117,114,-63,-30, 49, 52,-21,-50,
116,114,-66,-22, 48, 54,-22,-59,
116,114,-66,-24, 48, 54,-25,-54,
116,115,-63,-27, 48, 52,-31,-47,
115,116,-60,-31, 50, 50,-34,-41,
115,116,-58,-38, 50, 50,-33,-34,
115,116,-53,-44, 50, 50,-34,-29,
115,116,-48,-50, 50, 50,-36,-26,
};
const char stair[] PROGMEM = { 
54, 0, 30,
 44, 90,-39,-38, 10,-32,-10, 32,
 45, 90,-32,-46, 16,-38,-16, 38,
 44, 88,-24,-53, 24,-43,-24, 43,
 42, 86,-17,-57, 33,-45,-33, 45,
 42, 83,-13,-62, 37,-46,-37, 46,
 41, 80,-10,-67, 41,-47,-41, 47,
 42, 76, -7,-71, 42,-47,-43, 48,
 42, 72, -4,-75, 45,-47,-45, 48,
 43, 67, -2,-79, 45,-47,-46, 47,
 46, 64, -1,-81, 44,-46,-44, 46,
 48, 59, -1,-84, 42,-44,-42, 44,
 51, 55, -1,-85, 40,-42,-40, 42,
 54, 51, -1,-85, 37,-39,-37, 39,
 58, 48, -2,-85, 32,-36,-32, 36,
 61, 45, -3,-84, 28,-33,-28, 32,
 65, 42, -4,-83, 24,-29,-24, 29,
 68, 40, -5,-81, 19,-25,-19, 25,
 72, 38, -6,-79, 15,-22,-15, 22,
 74, 37, -8,-77, 11,-19,-11, 19,
 77, 36, -9,-74,  7,-16, -7, 16,
 80, 36,-12,-71,  2,-12, -2, 12,
 82, 36,-13,-67, -2, -9,  2,  9,
 85, 37,-16,-64, -6, -7,  6,  7,
 86, 38,-18,-61,-10, -5, 10,  5,
 87, 39,-21,-58,-14, -3, 14,  3,
 89, 42,-23,-51,-17,  1, 17, -1,
 90, 43,-30,-44,-25,  6, 25, -6,
 90, 45,-38,-38,-32, 11, 32,-11,
 90, 45,-46,-31,-38, 17, 38,-17,
 88, 44,-53,-23,-43, 25, 43,-26,
 86, 42,-57,-16,-45, 33, 45,-33,
 83, 42,-62,-12,-46, 38, 46,-38,
 80, 42,-67, -9,-47, 41, 47,-41,
 76, 42,-71, -7,-47, 43, 48,-43,
 72, 42,-75, -4,-47, 45, 48,-45,
 67, 44,-79, -3,-47, 45, 47,-44,
 64, 46,-81, -1,-46, 44, 46,-45,
 59, 48,-84,  0,-44, 43, 44,-43,
 55, 52,-85, -1,-42, 39, 42,-40,
 51, 55,-85, -1,-39, 36, 39,-36,
 48, 59,-85, -2,-36, 32, 36,-32,
 45, 62,-84, -3,-33, 27, 32,-27,
 42, 66,-83, -4,-29, 23, 29,-23,
 40, 69,-81, -5,-25, 18, 25,-18,
 38, 72,-79, -6,-22, 15, 22,-15,
 37, 75,-77, -8,-19, 10, 19,-10,
 36, 78,-74,-10,-16,  6, 16, -6,
 36, 81,-71,-12,-12,  1, 12, -1,
 36, 83,-67,-14, -9, -3,  9,  3,
 37, 85,-64,-16, -7, -7,  7,  7,
 38, 87,-61,-19, -5,-11,  5, 11,
 39, 88,-58,-21, -3,-15,  3, 15,
 42, 90,-51,-25,  1,-19, -1, 19,
 43, 90,-44,-32,  6,-26, -6, 26,
};
const char tr[] PROGMEM = { 
30, 0, 0,
 35, 38,-41,-46, 11,  2,-10, -1,
 39, 23,-37,-57, 11,  9,-11, -5,
 43,  6,-33,-64, 11, 21,-12,-13,
 46,-12,-28,-66, 11, 39,-13,-27,
 50,-17,-23,-63, 12, 49,-16,-34,
 52,-20,-18,-59, 14, 57,-19,-40,
 55,-14,-13,-58, 17, 49,-22,-36,
 57, -7, -7,-59, 20, 41,-27,-30,
 58,  0, -1,-58, 24, 33,-32,-25,
 58,  6,  5,-57, 29, 28,-39,-21,
 61, 12,  7,-55, 27, 23,-38,-17,
 65, 17,  2,-53, 19, 20,-29,-15,
 64, 22, -8,-50, 12, 16,-20,-13,
 57, 27,-24,-47,  4, 14, -8,-11,
 46, 32,-39,-44,  1, 12, -2,-10,
 32, 36,-51,-40,  4, 11, -2,-10,
 16, 40,-60,-36, 13, 11, -7,-11,
 -1, 44,-65,-31, 27, 11,-18,-12,
-13, 48,-65,-26, 41, 12,-29,-14,
-19, 51,-61,-21, 53, 13,-37,-17,
-18, 53,-58,-16, 55, 15,-39,-20,
-11, 56,-59,-10, 45, 18,-34,-24,
 -4, 57,-58, -5, 38, 22,-28,-29,
  2, 58,-58,  1, 31, 26,-23,-35,
  8, 59,-56,  7, 26, 29,-19,-40,
 14, 63,-54,  6, 22, 24,-16,-35,
 19, 65,-52,  1, 18, 18,-14,-27,
 24, 61,-49,-15, 15,  8,-12,-14,
 29, 53,-46,-30, 13,  2,-11, -5,
 34, 40,-42,-44, 12,  2,-10, -1,
};
const char trL[] PROGMEM = { 
25, 0, 0,
 33, 37,-40,-45, 10, -1,-13, -3,
 35, 22,-36,-49, 10,  5,-14, -3,
 36,  6,-31,-52,  9, 17,-15, -4,
 38, -7,-27,-54,  9, 31,-16, -6,
 39,-14,-22,-53,  9, 47,-19, -9,
 40,-10,-17,-51,  8, 45,-21,-10,
 42, -3,-11,-50,  8, 38,-26,-10,
 43,  3, -6,-49,  8, 32,-30, -9,
 44,  9,  1,-48, 10, 27,-37, -9,
 47, 15, -3,-47,  6, 22,-25, -9,
 46, 20,-12,-45,  3, 19,-13, -9,
 42, 25,-28,-44,  3, 16, -3, -9,
 38, 30,-43,-43,  4, 14,  2, -9,
 33, 34,-55,-41,  5, 14,  2, -9,
 29, 38,-64,-40,  7, 13, -3, -9,
 24, 42,-69,-39, 10, 13,-13, -9,
 20, 45,-65,-37, 14, 14,-25, -9,
 20, 48,-57,-36, 16, 15,-33, -9,
 22, 51,-57,-35, 15, 17,-29, -9,
 24, 53,-56,-33, 14, 20,-24,-10,
 25, 55,-54,-31, 13, 23,-20,-10,
 27, 60,-53,-31, 12, 21,-17, -9,
 29, 65,-50,-33, 12,  8,-15, -6,
 30, 60,-47,-37, 11,  1,-14, -4,
 32, 49,-44,-42, 10, -3,-13, -3,
};
const char trR[] PROGMEM = { 
25, 0, 0,
 31, 36,-42,-49, 15,  5, -9,  3,
 35, 31,-41,-61, 14,  6, -9,  1,
 39, 26,-40,-67, 13,  8, -9, -7,
 43, 22,-38,-68, 14, 11, -9,-18,
 47, 20,-37,-61, 14, 15, -9,-31,
 49, 21,-36,-57, 16, 15, -9,-31,
 52, 22,-34,-56, 18, 15,-10,-26,
 54, 25,-32,-55, 21, 13,-10,-22,
 55, 26,-30,-54, 26, 13,-12,-19,
 63, 27,-31,-51, 15, 12, -8,-16,
 64, 30,-34,-49,  5, 11, -5,-14,
 55, 31,-39,-45, -1, 11, -4,-13,
 44, 32,-43,-42, -3, 10, -3,-13,
 30, 34,-47,-37,  2, 10, -3,-14,
 15, 35,-51,-33, 10,  9, -4,-14,
 -2, 37,-54,-29, 25,  9, -5,-16,
-11, 39,-54,-24, 40,  9, -8,-17,
-13, 40,-51,-19, 49,  8,-11,-20,
 -6, 41,-50,-14, 42,  8,-10,-24,
  0, 43,-49, -8, 35,  8,-10,-28,
  6, 44,-49, -2, 29,  8, -9,-33,
 12, 46,-47,  0, 24,  7, -9,-32,
 17, 47,-46, -8, 20,  4, -9,-17,
 23, 44,-45,-22, 17,  3, -9, -6,
 27, 39,-43,-37, 15,  4, -9,  1,
};
const char vt[] PROGMEM = { 
17, 0, 0,
 51, 39,-57,-43,-18,  7, 19, -7,
 42, 39,-47,-43,  1,  7,  0, -7,
 39, 39,-43,-43,  7,  7, -7, -7,
 39, 39,-43,-43,  7,  7, -7, -7,
 39, 42,-43,-47,  7,  0, -7,  0,
 39, 51,-43,-57,  7,-19, -7, 19,
 39, 59,-43,-67,  7,-36, -7, 36,
 39, 59,-43,-66,  7,-35, -7, 36,
 39, 51,-43,-57,  7,-18, -7, 19,
 39, 42,-43,-47,  7,  1, -7,  0,
 39, 39,-43,-43,  7,  7, -7, -7,
 39, 39,-43,-43,  7,  7, -7, -7,
 40, 39,-45,-43,  3,  7, -3, -7,
 50, 39,-56,-43,-16,  7, 16, -7,
 58, 39,-65,-43,-33,  7, 33, -7,
 60, 39,-68,-43,-38,  7, 38, -7,
 52, 39,-59,-43,-21,  7, 22, -7,
};
const char wk[] PROGMEM = { 
43, 0, 0,
 12, 59,-55,-49, 23, 24, -2,-12,
 15, 59,-63,-47, 22, 27, -8,-11,
 18, 59,-67,-45, 20, 30,-20,-11,
 21, 59,-66,-43, 18, 34,-33,-10,
 24, 59,-64,-40, 16, 38,-37,-10,
 27, 58,-62,-37, 15, 43,-41,-11,
 30, 57,-60,-35, 13, 47,-45,-12,
 32, 58,-57,-32, 13, 47,-48,-13,
 35, 60,-57,-29, 12, 45,-47,-14,
 38, 62,-58,-26, 12, 41,-42,-15,
 40, 65,-59,-23, 11, 36,-37,-16,
 43, 66,-59,-20, 11, 32,-33,-18,
 45, 67,-59,-17, 11, 18,-30,-20,
 47, 62,-59,-14, 11,  7,-26,-22,
 49, 53,-59,-12, 12,  1,-24,-24,
 51, 40,-58,-12, 13,  2,-21,-22,
 52, 26,-57,-12, 14,  7,-19,-20,
 54, 17,-55,-14, 15, 13,-18,-16,
 55, 15,-54,-16, 17, 16,-16,-15,
 57, 13,-53,-23, 18, 19,-15, -9,
 58, 12,-51,-38, 21, 22,-13, -2,
 58, 12,-49,-51, 23, 24,-12, -1,
 59, 13,-47,-60, 26, 23,-11, -6,
 59, 17,-45,-66, 29, 20,-11,-15,
 59, 20,-43,-66, 32, 18,-10,-33,
 59, 23,-41,-65, 37, 17,-10,-35,
 58, 26,-38,-63, 41, 15,-11,-40,
 57, 29,-35,-61, 46, 14,-12,-44,
 58, 32,-33,-58, 47, 13,-13,-47,
 59, 34,-30,-57, 47, 12,-14,-48,
 61, 37,-27,-58, 43, 12,-15,-43,
 64, 40,-24,-59, 38, 11,-16,-38,
 65, 42,-21,-59, 34, 11,-17,-34,
 67, 44,-18,-59, 23, 11,-19,-31,
 64, 46,-15,-59, 10, 11,-21,-27,
 56, 48,-12,-59,  3, 12,-23,-24,
 45, 50,-12,-58,  1, 13,-23,-22,
 31, 52,-12,-57,  5, 14,-20,-19,
 18, 53,-14,-56, 13, 15,-17,-17,
 16, 55,-16,-55, 15, 17,-15,-16,
 14, 57,-18,-53, 17, 17,-13,-15,
 12, 57,-33,-52, 21, 20, -4,-14,
 12, 58,-47,-50, 23, 22,  0,-13,
};
const char wkL[] PROGMEM = { 
43, 0, 0,
 40, 59,-55,-51,  8, 24, -2, -9,
 41, 59,-63,-50,  8, 27, -8, -9,
 42, 59,-67,-49,  7, 30,-20, -8,
 42, 59,-66,-48,  7, 34,-33, -8,
 43, 59,-64,-48,  7, 38,-37, -8,
 44, 58,-62,-47,  7, 43,-41, -8,
 45, 57,-60,-46,  8, 47,-45, -8,
 46, 58,-57,-46,  8, 47,-48, -8,
 46, 60,-57,-45,  8, 45,-47, -8,
 47, 62,-58,-44,  8, 41,-42, -8,
 48, 65,-59,-43,  8, 36,-37, -8,
 48, 66,-59,-42,  8, 32,-33, -8,
 49, 67,-59,-41,  8, 18,-30, -8,
 50, 62,-59,-41,  9,  7,-26, -8,
 50, 53,-59,-40,  9,  1,-24, -9,
 51, 40,-58,-40,  9,  2,-21, -8,
 52, 26,-57,-40, 10,  7,-19, -7,
 52, 17,-55,-41, 10, 13,-18, -6,
 53, 15,-54,-41, 10, 16,-16, -6,
 53, 13,-53,-44, 11, 19,-15, -4,
 54, 12,-51,-48, 11, 22,-13, -4,
 54, 12,-49,-51, 12, 24,-12, -5,
 55, 13,-47,-54, 12, 23,-11, -6,
 55, 17,-45,-57, 13, 20,-11, -8,
 56, 20,-43,-60, 13, 18,-10,-11,
 56, 23,-41,-59, 14, 17,-10,-13,
 57, 26,-38,-59, 14, 15,-11,-13,
 57, 29,-35,-59, 15, 14,-12,-14,
 58, 32,-33,-58, 15, 13,-13,-15,
 58, 34,-30,-57, 14, 12,-14,-15,
 59, 37,-27,-57, 14, 12,-15,-15,
 59, 40,-24,-57, 12, 11,-16,-14,
 59, 42,-21,-56, 12, 11,-17,-14,
 59, 44,-18,-56, 10, 11,-19,-13,
 56, 46,-15,-56,  7, 11,-21,-12,
 53, 48,-12,-55,  6, 12,-23,-12,
 50, 50,-12,-54,  5, 13,-23,-12,
 46, 52,-12,-54,  4, 14,-20,-11,
 42, 53,-14,-53,  5, 15,-17,-11,
 41, 55,-16,-53,  6, 17,-15,-10,
 41, 57,-18,-52,  6, 17,-13,-10,
 40, 57,-33,-52,  7, 20, -4,-10,
 40, 58,-47,-51,  8, 22,  0, -9,
};
const char wkR[] PROGMEM = { 
43, 0, 0,
 12, 55,-53,-49, 23, 12, -5,-12,
 15, 55,-55,-47, 22, 12, -7,-11,
 18, 56,-58,-45, 20, 13, -9,-11,
 21, 56,-60,-43, 18, 13,-11,-10,
 24, 57,-59,-40, 16, 14,-13,-10,
 27, 57,-59,-37, 15, 15,-14,-11,
 30, 58,-58,-35, 13, 15,-15,-12,
 32, 58,-58,-32, 13, 15,-15,-13,
 35, 58,-57,-29, 12, 14,-15,-14,
 38, 59,-57,-26, 12, 14,-15,-15,
 40, 59,-56,-23, 11, 12,-14,-16,
 43, 60,-56,-20, 11, 11,-13,-18,
 45, 58,-56,-17, 11,  9,-13,-20,
 47, 55,-55,-14, 11,  7,-13,-22,
 49, 52,-55,-12, 12,  5,-12,-24,
 51, 49,-54,-12, 13,  4,-11,-22,
 52, 45,-54,-12, 14,  4,-11,-20,
 54, 42,-53,-14, 15,  5,-10,-16,
 55, 41,-53,-16, 17,  6,-10,-15,
 57, 40,-52,-23, 18,  7,-10, -9,
 58, 40,-51,-38, 21,  7, -9, -2,
 58, 40,-51,-51, 23,  8, -9, -1,
 59, 40,-50,-60, 26,  8, -9, -6,
 59, 41,-49,-66, 29,  8, -9,-15,
 59, 42,-49,-66, 32,  7, -8,-33,
 59, 43,-48,-65, 37,  7, -8,-35,
 58, 44,-47,-63, 41,  7, -8,-40,
 57, 44,-47,-61, 46,  8, -8,-44,
 58, 45,-46,-58, 47,  8, -8,-47,
 59, 46,-45,-57, 47,  8, -8,-48,
 61, 47,-44,-58, 43,  8, -8,-43,
 64, 47,-43,-59, 38,  8, -8,-38,
 65, 48,-43,-59, 34,  8, -8,-34,
 67, 49,-42,-59, 23,  8, -8,-31,
 64, 49,-41,-59, 10,  8, -8,-27,
 56, 50,-40,-59,  3,  9, -9,-24,
 45, 51,-40,-58,  1,  9, -8,-22,
 31, 51,-40,-57,  5, 10, -7,-19,
 18, 52,-40,-56, 13, 10, -7,-17,
 16, 53,-41,-55, 15, 10, -6,-16,
 14, 53,-42,-53, 17, 11, -5,-15,
 12, 54,-46,-52, 21, 11, -4,-14,
 12, 54,-50,-50, 23, 11, -5,-13,
};

const char balance[] PROGMEM = { 
1, 0, 0,
  0,  0,  0,  0,  0,  0,  0,  0, 30, 30,-30,-30, 30, 30,-30,-30,}; //1st group of 4 = nil, 2nd = hip, 3rd = shoulder, 4th = legs
const char buttUp[] PROGMEM = { 
1, 0, -15,
 20, 40,  0,  0,  5,  5,  3,  3, 90, 90,-45,-45,-60,-60, -5, -5,};
const char calib[] PROGMEM = { 
1, 0, 0,
  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,};
const char cd1[] PROGMEM = { 
1, -15, -15,
 20,-45, 30,  0,  5,  5,  3,  3, 70, 70,-45,-45,-60,-60,  0,  0,};
const char cd2[] PROGMEM = { 
1, 15, -15,
-30,-30,  0,  0,  5,  5,  3,  3, 70, 70,-45,-45,-60,-60,  0,  0,};
const char dropped[] PROGMEM = { 
1, 0, -75,
  0, 30,  0,  0, -5, -5, 15, 15,-75,-75,-60,-60, 60, 60, 30, 30,};
const char lifted[] PROGMEM = { 
1, 0, 75,
  0,-70,  0,  0,  0,  0,  0,  0, 55, 55, 20, 20, 45, 45,  0,  0,};
const char pee[] PROGMEM = { 
1, 0, 0,
 45, 20,  0,  0,   15,-10, 60,-10,   45, 45,-70,-15,   15, 45, 30,-20,};
const char pee1[] PROGMEM = { 
1, 0, 0,
 45, 10,  0,  0, 15,-10, -5, -5, 45, 30,-30,-15, 15, 45,-30,  0,};
const char pu1[] PROGMEM = { 
1, 0, 0,
  0,-30,  0,  0,  0,  0,  0,  0, 20, 20, 60, 60, 60, 60,-55,-55,};
const char pu2[] PROGMEM = { 
1, 0, 0,
  0, 10,  0,  0,  0,  0,  0,  0, 60, 60, 40, 40,-45,-45,-55,-55,};
const char rest[] PROGMEM = { 
1, 0, 0,
-30,-80,-45,  0, -3, -3,  3,  3, 60, 60,-60,-60,-45,-45, 45, 45,};
const char sit[] PROGMEM = { 
1, 0, 30,
  0,  0,-60,  0, -5, -5, 20, 20, 30, 30,-90,-90, 60, 60, 45, 45,};
const char sleep[] PROGMEM = { 
1, 0, 0,
-10,-100,  0,  0, -5, -5,  3,  3, 80, 80,-80,-80,-55,-55, 55, 55,};
const char str[] PROGMEM = { 
1, 0, 15,
  0, 30,  0,  0, -5, -5,  0,  0,-60,-60,-15,-15, 60, 60,-45,-45,};
const char zero[] PROGMEM = { 
1, 0, 0,
  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,};
const char stairs1[] PROGMEM = { 
1, 0, 0,
//45, 20,  0,  0,    -20,60, 15,15,     15, 70,-45,-35,     15, -35, -15,-25,};
 45, 20,  0,  0,    45,-10, 20,0,     70, 25,-55,-55,     -25, 30, 5,-30,};	 //()()()()
const char stairs2[] PROGMEM = { 
1, 0, 0,
45, 20,  0,  0,    45,0, 20,0,     -45, 35,-55,-55,     -25, 30, 5,-30,};
//45, 20,  0,  0,    -20,35, 15,15,     15, -55,-25,-15,     15, -35, -15,-25,};
const char stairs3[] PROGMEM = { 
1, 0, 0,
45, 20,  0,  0,    5,0, 20,0,     -5, 45,-55,-45,     15, 30, -5,-30,};
//45, 20,  0,  0,    -20,5, 15,15,     15, -25,-25,-15,     15, 5, -35,-45,};
const char stairs4[] PROGMEM = { 
1, 0, 0,
45, 20,  0,  0,    -20,35, 15,15,     15, 70,-25,-15,     25, -35, -35,-45,};
const char stairs5[] PROGMEM = { 
1, 0, 0,
45, 20,  0,  0,    -20,35, 15,15,     15, 70,-25,-15,     15, -35, -35,-45,};
const char stairs6[] PROGMEM = { 
1, 0, 0,
45, 20,  0,  0,    -20,35, 15,15,     15, 70,-25,-15,     15, -35, -35,-45,};
const char stairs7[] PROGMEM = { 
1, 0, 0,
45, 20,  0,  0,    -20,35, 15,15,     15, 70,-25,-15,     15, -35, -35,-45,};
const char stairs8[] PROGMEM = { 
1, 0, 0,
45, 20,  0,  0,    -20,35, 15,15,     15, 70,-25,-15,     15, -35, -35,-45,};
const char stairs9[] PROGMEM = { 
1, 0, 0,
45, 20,  0,  0,    -20,35, 15,15,     15, 70,-25,-15,     15, -35, -35,-45,};
const char stairs10[] PROGMEM = { 
1, 0, 0,
45, 20,  0,  0,    -20,35, 15,15,     15, 70,-25,-15,     15, -35, -35,-45,};

#if !defined(MAIN_SKETCH) || !defined(I2C_EEPROM)
		//if it's not the main sketch to save data or there's no external EEPROM, 
		//the list should always contain all information.
  const char* skillNameWithType[]={"bdI","bkI","bkLI","bkRI","crI","crLI","crRI","lyI","stairN","trI","trLI","trRI","vtI","wkI","wkLI","wkRI","balanceI","buttUpI","calibI","cd1I","cd2I","droppedI","liftedI","peeI","pee1I","pu1I","pu2I","restI","sitI","sleepI","strI","zeroI","stairs1I","stairs2I","stairs3I","stairs4I","stairs5I","stairs6I","stairs7I","stairs8I","stairs9I","stairs10I",};
  const char* progmemPointer[] = {bd, bk, bkL, bkR, cr, crL, crR, ly, stair, tr, trL, trR, vt, wk, wkL, wkR, balance, buttUp, calib, cd1, cd2, dropped, lifted, pee, pee1, pu1, pu2, rest, sit, sleep, str, zero, stairs1, stairs2, stairs3, stairs4, stairs5, stairs6, stairs7, stairs8, stairs9, stairs10, };
#else	//only need to know the pointers to newbilities, because the intuitions have been saved onto external EEPROM,
	//while the newbilities on progmem are assigned to new addresses
  const char* progmemPointer[] = {stair, };
#endif
//the total byte of instincts is 4702
//the maximal array size is 436 bytes of stair. 
//Make sure to leave enough memory for SRAM to work properly. Any single skill should be smaller than 400 bytes for safety.
