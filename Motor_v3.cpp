#include "Arduino.h"
#include "Motor_v3.h"

Motor::Motor(int DIRpin, int DIRpin_S, int SPEEDpin)
{
  _DIR = DIRpin; _DIR_S = DIRpin_S; _SPEED = SPEEDpin;
  setProperties(255, -255, 10);
}
void Motor::set(int SPEED)
{
  _tSPEED = SPEED;
}
void Motor::setProperties(int maxSPEED, int minSPEED, int Step)
{
  _maxSPEED = maxSPEED; _minSPEED = minSPEED; _step = Step;
}
void Motor::update()
{
    if( _cSPEED > _tSPEED ){
      _cSPEED = _cSPEED -_step;
      if(_cSPEED < _tSPEED) _cSPEED = _tSPEED;
    }
    else if (_cSPEED < _tSPEED ){
      _cSPEED = _cSPEED + _step; 
      if(_cSPEED > _tSPEED) _cSPEED = _tSPEED;      
    }       
  if( _cSPEED < _minSPEED )
     _cSPEED = _minSPEED;
  if( _cSPEED > _maxSPEED )
     _cSPEED = _maxSPEED;

  // output to pin
  if (_cSPEED >0){    
    digitalWrite(_DIR, HIGH);  digitalWrite(_DIR_S, LOW);
  }else{    
    digitalWrite(_DIR, LOW);  digitalWrite(_DIR_S, HIGH);
  }
  analogWrite(_SPEED, abs(_cSPEED));
}

int Motor::getSPEED()
{
  return _cSPEED;
}
void Motor::Debug() 
{
  Serial.print( " _DIR(pin):  ");
  Serial.print( _DIR, DEC );
  Serial.print( " _SPEED(pin):  ");
  Serial.print( _SPEED, DEC );
  Serial.print( " _maxSPEED:  ");
  Serial.print( _maxSPEED, DEC );
  Serial.print( " _minSPEED:  ");
  Serial.print( _minSPEED, DEC );
  Serial.print( " _step:  ");
  Serial.print( _step, DEC ); 
  Serial.print( "       _cSPEED:  ");
  Serial.print( _cSPEED, DEC );
  Serial.print( " _tSPEED:  ");
  Serial.println( _tSPEED, DEC );
}
