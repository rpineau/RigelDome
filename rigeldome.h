//
//  rigeldome.h
//
//  Created by Rodolphe Pineau on 6/11/2016.
//  Rigel rotation drive unit for Pulsar Dome X2 plugin

#ifndef __RIGEL_DOME__
#define __RIGEL_DOME__
#include <math.h>
#include <string.h>
#include <time.h>

#include <string>
#include <vector>
#include <sstream>
#include <iostream>

#include "../../licensedinterfaces/sberrorx.h"
#include "../../licensedinterfaces/serxinterface.h"
#include "../../licensedinterfaces/loggerinterface.h"

#define RIGEL_DEBUG 2

#define SERIAL_BUFFER_SIZE 20
#define MAX_TIMEOUT 5000
#define ND_LOG_BUFFER_SIZE 256

// error codes
// Error code
enum RigelDomeErrors {RD_OK=0, NOT_CONNECTED, RD_CANT_CONNECT, RD_BAD_CMD_RESPONSE, COMMAND_FAILED};
enum RigelDomeShutterState {OPEN=0, CLOSED, OPENING, CLOSING, SHUTTER_ERROR, UNKNOWN, NOT_FITTED};
enum RigelMotorState {IDLE=0, MOVING_TO_TARGET, MOVING_TO_VELOCITY, MOVING_AT_SIDEREAL, MOVING_ANTICLOCKWISE, MOVING_CLOCKWISE, CALIBRATIG, GOING_HOME};

class CRigelDome
{
public:
    CRigelDome();
    ~CRigelDome();

    int        Connect(const char *szPort);
    void        Disconnect(void);
    bool        IsConnected(void) { return m_bIsConnected; }

    void        SetSerxPointer(SerXInterface *p) { m_pSerx = p; }
    void        setLogger(LoggerInterface *pLogger) { m_pLogger = pLogger; };

    // Dome commands
    int syncDome(double dAz, double dEl);
    int parkDome(void);
    int unparkDome(void);
    int gotoAzimuth(double newAz);
    int openShutter();
    int closeShutter();
    int getFirmwareVersion(char *version, int strMaxLen);
    int getModel(char *model, int strMaxLen);
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
    int getBatteryLevels(double &shutterVolts, int &percent);

    bool hasShutterUnit();

    void setDebugLog(bool enable);

protected:
    
    int             readResponse(char *pszRespBuffer, int bufferLen);
    int             getDomeAz(double &dDomeAz);
    int             getDomeEl(double &dDomeEl);
    int             getDomeHomeAz(double &dAz);
    int             getDomeParkAz(double &dAz);
    int             getShutterState(int &nState);
    int             getDomeStepPerRev(int &nStepPerRev);

    int             isDomeMoving(bool &bIsMoving);
    int             isDomeAtHome(bool &bAtHome);

    int             connectToShutter();

    int             domeCommand(const char *pszCmd, char *pszResult, int nResultMaxLen);
    int             getExtendedState();
    int             parseFields(const char *pszResp, std::vector<std::string> &svFields, char cSeparator);
    
    LoggerInterface *m_pLogger;
    bool            m_bDebugLog;
    
    bool            m_bIsConnected;
    bool            m_bHomed;
    bool            m_bParked;
    bool            m_bCalibrating;
    
    int             m_nNbStepPerRev;
    double          m_dShutterBatteryVolts;
    double          m_dShutterBatteryPercent;
    double          m_dHomeAz;
    
    double          m_dParkAz;

    double          m_dCurrentAzPosition;
    double          m_dCurrentElPosition;

    double          m_dGotoAz;
    
    SerXInterface   *m_pSerx;
    
    char            m_szFirmwareVersion[SERIAL_BUFFER_SIZE];
    int             m_nShutterState;
    bool            m_bHasShutter;
    bool            m_bShutterOpened;

    char            m_szLogBuffer[ND_LOG_BUFFER_SIZE];
    int             m_nMotorState;

#ifdef RIGEL_DEBUG
    std::string m_sLogfilePath;
    // timestamp for logs
    char *timestamp;
    time_t ltime;
    FILE *Logfile;      // LogFile
#endif

};

#endif
