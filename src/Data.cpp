#include <Data.h>

Data::Data()
{
    mph = 0;
    heading = 0;
    rateOfTurn = 0;
    rollAngle = 0;
    dateTime[0] = '\0';
    latitude = 0;
    northOrSouth = 'X';
    longitude = 0;
    eastOrWest = 'X';
}

Data::~Data()
{

}

//getters
float Data::getMph()
{
   return mph; 
}
unsigned int  Data::getHeading()
{
    return heading;
}
float  Data::getRateOfTurn()
{
    return rateOfTurn;
}
float  Data::getRollAngle()
{
    return rollAngle;
}
char * Data::getDateTime()
{
    return dateTime;
}
long Data::getLatitude()
{
    return latitude;
}
uint8_t Data::getNorthOrSouth()
{
    return northOrSouth;
}
long Data::getLongitude()
{
    return longitude;
}
uint8_t Data::getEastOrWest()
{
    return eastOrWest;
}

//setters
void Data::setMph(float speedInMphValue)
{
    mph = speedInMphValue;
}
void Data::setHeading(unsigned int headingValue)
{
    heading = headingValue;
}
void Data::setRateOfTurn(float rateOfTurnValue)
{
    rateOfTurn = rateOfTurnValue;
}
void Data::setRollAngle(float rollAngleValue)
{
    rollAngle = rollAngleValue;
}
void Data::setDate(void* dateValue)
{
    memcpy(dateTime, dateValue, 6);
}
void Data::setTime(void* timeValue)
{
    memcpy(dateTime+6, timeValue, 6);
}
void Data::setLatitude(long latitudeValue)
{
    latitude = latitudeValue;
}
void Data::setNorthOrSouth(uint8_t northOrSouthValue)
{
    northOrSouth = northOrSouthValue;
}
void Data::setLongitude(long longitudeValue)
{
    longitude = longitudeValue;
}
void Data::setEastOrWest(uint8_t eastOrWestValue)
{
    eastOrWest = eastOrWestValue;   
}