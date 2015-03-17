#ifndef SingleWheelEncoders_c
#define SingleWheelEncoders_c
#ifndef PinChangeInt_h
#define LIBCALL_PINCHANGEINT
#include <PinChangeInt.h>
#endif
#include <avr/interrupt.h>
#include "SingleWheelEncoders.h"
 
unsigned int countsM1 = 0;
unsigned int countsM2 = 0;
 
void incrementCountsM1() {
  countsM1++;
}
 
void incrementCountsM2() {
  countsM2++;
}
 
SingleWheelEncoders::SingleWheelEncoders()
{
  init(11,13);
}
 
void SingleWheelEncoders::init(unsigned char m1a, unsigned char m2a)
{
  pinMode(m1a, INPUT);
  digitalWrite(m1a, LOW);
  PCintPort::attachInterrupt(m1a, &incrementCountsM1, CHANGE);  // add more attachInterrupt code as required
  pinMode(m2a, INPUT);
  digitalWrite(m2a, LOW);
  PCintPort::attachInterrupt(m2a, &incrementCountsM2, CHANGE);
}
 
unsigned int SingleWheelEncoders::getCountsM1()
{
  cli();
  unsigned int tmp = countsM1;
  sei();
  return tmp;
}
 
unsigned int SingleWheelEncoders::getCountsM2()
{
  cli();
  unsigned int tmp = countsM2;
  sei();
  return tmp;
}
 
unsigned int SingleWheelEncoders::getCountsAndResetM1()
{
  cli();
  unsigned int tmp = countsM1;
  countsM1 = 0;
  sei();
  return tmp;
}
 
unsigned int SingleWheelEncoders::getCountsAndResetM2()
{
  cli();
  unsigned int tmp = countsM2;
  countsM2 = 0;
  sei();
  return tmp;
}
 
void SingleWheelEncoders::resetM1M2()
{
  countsM1 = 0;
  countsM2 = 0;
}
 
#endif