#ifndef MESSAGE_TYPES_H
#define MESSAGE_TYPES_H

// =============================== MESSAGE TYPES ==============================
// const byte MT_OK = 0;
// const byte MT_WAIT = 1;
// const byte MT_ALONE = 2;
// const byte MT_NETWORK = 3;

const byte MT_REQUEST_FOR_CALIBRATION = 10;
const byte MT_CALIBRATION_LED_ON = 11;
const byte MT_CALIBRATION_LED_OFF = 12;
const byte MT_END_CALIBRATION = 13;
const byte MT_CALIBRATION_VALUE = 14;

const byte MT_REQUEST_JOIN_NETWORK = 20;
const byte MT_REQUEST_JOIN_NETWORK_REPLY_OK = 21;

const byte MT_MAX_NODES_IN_NETWORK_REACHED = 30;

const byte MT_TODO = 50;

// =============================== MESSAGE TYPES ==============================

#endif // MESSAGE_TYPES_H
