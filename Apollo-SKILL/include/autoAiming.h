#ifndef AUTOAIMING_H_
#define AUTOAIMING_H_

extern bool isAiming;
extern int offset;

int autonAiming();
int autonAiming_vision();
// Basic control
void setAimingStatus(bool _input);
void setAimingStatus(bool _input, int _offset);
void setVisionAimingStatus(bool _input);
#endif