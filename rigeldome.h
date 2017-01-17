//
//  nexdome.h
//  NexDome
//
//  Created by Rodolphe Pineau on 6/11/2016.
//  NexDome X2 plugin

#ifndef __NEXDOME__
#define __NEXDOME__
#include <math.h>
#include <string.h>
#include "../../licensedinterfaces/sberrorx.h"
#include "../../licensedinterfaces/serxinterface.h"
#include "../../licensedinterfaces/loggerinterface.h"

#define SERIAL_BUFFER_SIZE 20
#define MAX_TIMEOUT 5000
#define ND_LOG_BUFFER_SIZE 256

// error codes
// Error code
enum RigelDomeErrors {RD_OK=0, NOT_CONNECTED, RD_CANT_CONNECT, RD_BAD_CMD_RESPONSE, COMMAND_FAILED};
enum RigelDomeShutterState {OPEN=0, CLOSED, OPENING, CLOSING, SHUTTER_ERROR, UNKNOWN, NOT_FITTED};

class CRigelDome
{
public:
    CRigelDome();
    ~CRigelDome();

    int        Connect(const char *szPort);
    void        Disconnect(void);
    bool        IsConnected(void) { return bIsConnected; }

    void        SetSerxPointer(SerXInterface *p) { pSerx = p; }
    void        setLogger(LoggerInterface *pLogger) { mLogger = pLogger; };

    // Dome commands
    int syncDome(double dAz, double dEl);
    int parkDome(void);
    int unparkDome(void);
    int gotoAzimuth(double newAz);
    int openShutter();
    int closeShutter();
    int getFirmwareVersion(char *version, int strMaxLen);
    int goHome();
    int calibrate();

    // command complete functions
    int isGoToComplete(bool &complete);
    int isOpenComplete(bool &complete);
    int isCloseComplete(bool &complete);
    int isParkComplete(bool &complete);
    int isUnparkComplete(bool &complete);
    int isFindHomeComplete(bool &complete);
    int isCalibratingComplete(bool &complete);

    int abortCurrentCommand();

    // getter/setter
    int getNbTicksPerRev();
    int getBatteryLevel();

    double getHomeAz();
    int setHomeAz(double dAz);

    double getParkAz();
    int setParkAz(double dAz);

    double getCurrentAz();
    double getCurrentEl();

    int getCurrentShutterState();
    int getBatteryLevels(double &domeVolts, double &shutterVolts);

    void setDebugLog(bool enable);

protected:
    
    int             readResponse(char *respBuffer, int bufferLen);
    int             getDomeAz(double &domeAz);
    int             getDomeEl(double &domeEl);
    int             getDomeHomeAz(double &Az);
    int             getDomeParkAz(double &Az);
    int             getShutterState(int &state);
    int             getDomeStepPerRev(int &stepPerRev);

    bool            isDomeMoving();
    bool            isDomeAtHome();
    
    int             domeCommand(const char *cmd, char *result, int resultMaxLen);

    LoggerInterface *mLogger;
    bool            bDebugLog;
    
    bool            bIsConnected;
    bool            mHomed;
    bool            mParked;
    bool            bCalibrating;
    
    int             mNbStepPerRev;
    double          mShutterBatteryVolts;
    double          mHomeAz;
    
    double          mParkAz;

    double          mCurrentAzPosition;
    double          mCurrentElPosition;

    double          mGotoAz;
    
    SerXInterface   *pSerx;
    
    char            firmwareVersion[SERIAL_BUFFER_SIZE];
    int             mShutterState;
    bool            mHasShutter;
    bool            mShutterOpened;

    char            mLogBuffer[ND_LOG_BUFFER_SIZE];

};

#endif
