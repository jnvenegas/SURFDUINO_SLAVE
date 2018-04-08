#ifndef DATASTRUCT
#define DATASTRUCT

#include <Arduino.h>

class Data{

    private:

    float mph;
    unsigned int heading;
    float rateOfTurn;
    float rollAngle;
    char dateTime[12];
    long latitude;
    uint8_t northOrSouth;
    long longitude;
    uint8_t eastOrWest;

    public:

    Data();
    ~Data();

    //getters
    float getMph();
    unsigned int  getHeading();
    float  getRateOfTurn();
    float getRollAngle();
    char * getDateTime();
    long getLatitude();
    uint8_t getNorthOrSouth();
    long getLongitude();
    uint8_t getEastOrWest();

    //setters
    void setMph(float speedInMphValue);
    void setHeading(unsigned int headingValue);
    void setRateOfTurn(float rateOfTurnValue);
    void setRollAngle(float rollAngleValue);
    void setDate(void* dateValue);
    void setTime(void* timeValue);
    void setLatitude(long latitudeValue);
    void setNorthOrSouth(uint8_t northOrSouthValue);
    void setLongitude(long longitudeValue);
    void setEastOrWest(uint8_t eastOrWestValue);
};

#endif