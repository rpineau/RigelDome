//
//  nexdome.cpp
//  NexDome X2 plugin
//
//  Created by Rodolphe Pineau on 6/11/2016.


#include "rigeldome.h"
#include <stdlib.h>
#include <stdio.h>
#include <ctype.h>
#include <memory.h>
#ifdef SB_MAC_BUILD
#include <unistd.h>
#endif

CRigelDome::CRigelDome()
{
    // set some sane values
    bDebugLog = false;
    
    pSerx = NULL;
    bIsConnected = false;

    mNbStepPerRev = 0;
    mShutterBatteryVolts = 0.0;
    
    mHomeAz = 180;
    mParkAz = 180;

    mCurrentAzPosition = 0.0;
    mCurrentElPosition = 0.0;

    bCalibrating = false;

    mHasShutter = false;
    mShutterOpened = false;
    
    mParked = true;
    mHomed = false;
    memset(firmwareVersion,0,SERIAL_BUFFER_SIZE);
    memset(mLogBuffer,0,ND_LOG_BUFFER_SIZE);
}

CRigelDome::~CRigelDome()
{

}

int CRigelDome::Connect(const char *szPort)
{
    int err;
    int state;

    // 9600 8N1
    if(pSerx->open(szPort, 115200, SerXInterface::B_NOPARITY, "-DTR_CONTROL 1") == 0)
        bIsConnected = true;
    else
        bIsConnected = false;

    if(!bIsConnected)
        return ERR_COMMNOLINK;

    if (bDebugLog) {
        snprintf(mLogBuffer,ND_LOG_BUFFER_SIZE,"[CRigelDome::Connect] Connected.\n");
        mLogger->out(mLogBuffer);

        snprintf(mLogBuffer,ND_LOG_BUFFER_SIZE,"[CRigelDome::Connect] Getting Firmware.\n");
        mLogger->out(mLogBuffer);
    }
    // if this fails we're not properly connected.
    err = getFirmwareVersion(firmwareVersion, SERIAL_BUFFER_SIZE);
    if(err) {
        if (bDebugLog) {
            snprintf(mLogBuffer,ND_LOG_BUFFER_SIZE,"[CRigelDome::Connect] Error Getting Firmware.\n");
            mLogger->out(mLogBuffer);
        }
        bIsConnected = false;
        pSerx->close();
        return FIRMWARE_NOT_SUPPORTED;
    }

    if (bDebugLog) {
        snprintf(mLogBuffer,ND_LOG_BUFFER_SIZE,"[CRigelDome::Connect] Got Firmware.\n");
        mLogger->out(mLogBuffer);
    }
    // assume the dome was parked
    getDomeParkAz(mCurrentAzPosition);

    syncDome(mCurrentAzPosition,mCurrentElPosition);
    err = getShutterState(state);

    if(state != NOT_FITTED && state != UNKNOWN )
        mHasShutter = true;

    return SB_OK;
}


void CRigelDome::Disconnect()
{
    if(bIsConnected) {
        pSerx->purgeTxRx();
        pSerx->close();
    }
    bIsConnected = false;
}


int CRigelDome::readResponse(char *respBuffer, int bufferLen)
{
    int err = RD_OK;
    unsigned long nBytesRead = 0;
    unsigned long totalBytesRead = 0;
    char *bufPtr;

    memset(respBuffer, 0, (size_t) bufferLen);
    bufPtr = respBuffer;

    do {
        err = pSerx->readFile(bufPtr, 1, nBytesRead, MAX_TIMEOUT);
        if(err) {
            if (bDebugLog) {
                snprintf(mLogBuffer,ND_LOG_BUFFER_SIZE,"[CRigelDome::readResponse] readFile error.\n");
                mLogger->out(mLogBuffer);
            }
            return err;
        }

        if (bDebugLog) {
            snprintf(mLogBuffer,ND_LOG_BUFFER_SIZE,"[CRigelDome::readResponse] respBuffer = %s\n",respBuffer);
            mLogger->out(mLogBuffer);
        }
        
        if (nBytesRead !=1) {// timeout
            if (bDebugLog) {
                snprintf(mLogBuffer,ND_LOG_BUFFER_SIZE,"[CRigelDome::readResponse] readFile Timeout.\n");
                mLogger->out(mLogBuffer);
            }
            err = RD_BAD_CMD_RESPONSE;
            break;
        }
        totalBytesRead += nBytesRead;
        if (bDebugLog) {
            snprintf(mLogBuffer,ND_LOG_BUFFER_SIZE,"[CRigelDome::readResponse] nBytesRead = %lu\n",nBytesRead);
            mLogger->out(mLogBuffer);
        }
    } while (*bufPtr++ != 0x0D && totalBytesRead < bufferLen );

    *bufPtr = 0; //remove the \r
    return err;
}


int CRigelDome::domeCommand(const char *cmd, char *result, int resultMaxLen)
{
    int err = RD_OK;
    char resp[SERIAL_BUFFER_SIZE];
    unsigned long  nBytesWrite;

    pSerx->purgeTxRx();
    if (bDebugLog) {
        snprintf(mLogBuffer,ND_LOG_BUFFER_SIZE,"[CRigelDome::domeCommand] Sending %s\n",cmd);
        mLogger->out(mLogBuffer);
    }
    err = pSerx->writeFile((void *)cmd, strlen(cmd), nBytesWrite);
    pSerx->flushTx();
    if(err)
        return err;
    // read response
    if (bDebugLog) {
        snprintf(mLogBuffer,ND_LOG_BUFFER_SIZE,"[CRigelDome::domeCommand] Getting response.\n");
        mLogger->out(mLogBuffer);
    }
    err = readResponse(resp, SERIAL_BUFFER_SIZE);
    if(err)
        return err;

    if(result)
        strncpy(result, &resp[1], resultMaxLen);

    return err;

}

int CRigelDome::getDomeAz(double &domeAz)
{
    int err = RD_OK;
    char resp[SERIAL_BUFFER_SIZE];

    if(!bIsConnected)
        return NOT_CONNECTED;

    if(bCalibrating)
        return err;

    err = domeCommand("ANGLE\r", resp, SERIAL_BUFFER_SIZE);
    if(err)
        return err;
    
    // convert Az string to double
    domeAz = atof(resp);
    mCurrentAzPosition = domeAz;

    return err;
}

int CRigelDome::getDomeEl(double &domeEl)
{
    int err = RD_OK;
    int shutterState;

    if(!bIsConnected)
        return NOT_CONNECTED;

    getShutterState(shutterState);

    if(!mShutterOpened || !mHasShutter)
    {
        domeEl = 0.0;
    }
    else {
        domeEl = 90.0;
    }

    mCurrentElPosition = domeEl;
    
    return err;
}


int CRigelDome::getDomeHomeAz(double &Az)
{
    int err = RD_OK;
    char resp[SERIAL_BUFFER_SIZE];

    if(!bIsConnected)
        return NOT_CONNECTED;

    if(bCalibrating)
        return err;

    err = domeCommand("HOME\r", resp, SERIAL_BUFFER_SIZE);
    if(err)
        return err;

    // convert Az string to double
    Az = atof(resp);
    mHomeAz = Az;
    return err;
}

int CRigelDome::getDomeParkAz(double &Az)
{
    int err = RD_OK;
    char resp[SERIAL_BUFFER_SIZE];

    if(!bIsConnected)
        return NOT_CONNECTED;

    if(bCalibrating)
        return err;

    err = domeCommand("PARK\r", resp, SERIAL_BUFFER_SIZE);
    if(err)
        return err;

    // convert Az string to double
    Az = atof(resp);
    mParkAz = Az;
    return err;
}


int CRigelDome::getShutterState(int &state)
{
    int err = RD_OK;
    char resp[SERIAL_BUFFER_SIZE];
    int shutterState;

    if(!bIsConnected)
        return NOT_CONNECTED;

    if(bCalibrating)
        return err;

    err = domeCommand("SHUTTER\r", resp, SERIAL_BUFFER_SIZE);
    if(err)
        return err;
    err = domeCommand("SHUTTER\r", resp, SERIAL_BUFFER_SIZE);
    if(err)
        return err;

    state = atoi(resp);
    switch(shutterState) {
        case OPEN:
            mShutterOpened = true;
            break;

        case CLOSED:
            mShutterOpened = false;
            break;

        case NOT_FITTED:
            mShutterOpened = false;
            mHasShutter = false;
            break;
        default:
            mShutterOpened = false;
            
    }

    state = atoi(resp);

    return err;
}


int CRigelDome::getDomeStepPerRev(int &stepPerRev)
{
    int err = RD_OK;
    char resp[SERIAL_BUFFER_SIZE];

    if(!bIsConnected)
        return NOT_CONNECTED;

    err = domeCommand("ENCREV\r", resp, SERIAL_BUFFER_SIZE);
    if(err)
        return err;

    stepPerRev = atoi(resp);
    mNbStepPerRev = stepPerRev;
    return err;
}

int CRigelDome::getBatteryLevels(double &shutterVolts, int &percent)
{
    int err = RD_OK;
    int i = 0;
    int j = 0;
    char resp[SERIAL_BUFFER_SIZE];
    char voltData[SERIAL_BUFFER_SIZE];
    
    if(!bIsConnected)
        return NOT_CONNECTED;

    if(bCalibrating)
        return err;
    
    err = domeCommand("BAT\r", resp, SERIAL_BUFFER_SIZE);
    if(err)
        return err;

    
    // convert battery values string
    memset(voltData,0,SERIAL_BUFFER_SIZE);
    // skip the spaces:
    while(resp[j]==' ')
        j++;
    while(resp[j] != ' ' && i < (SERIAL_BUFFER_SIZE-1))
        voltData[i++]=resp[j++];
    percent = atof(voltData);

    // skip the spaces:
    while(resp[j]==' ')
        j++;
    memset(voltData,0,SERIAL_BUFFER_SIZE);
    i = 0;
    while(resp[j] != 0 && i < (SERIAL_BUFFER_SIZE-1))
        voltData[i++]=resp[j++];
    shutterVolts = atof(voltData);

    shutterVolts = shutterVolts / 1000.0;
    return err;
}

bool CRigelDome::hasShutterUnit() {
    return mHasShutter;
}

void CRigelDome::setDebugLog(bool enable)
{
    bDebugLog = enable;
}

bool CRigelDome::isDomeMoving()
{
    bool isMoving;
    int tmp;
    int err = RD_OK;
    char resp[SERIAL_BUFFER_SIZE];

    if(!bIsConnected)
        return NOT_CONNECTED;

    err = domeCommand("MSTATE\r", resp, SERIAL_BUFFER_SIZE);
    if(err) {
        return false;   // Not really correct but will do for now.
    }

    isMoving = false;
    tmp = atoi(resp);
    if(tmp != 0 || tmp != 3)
        isMoving = true;

    return isMoving;
}

bool CRigelDome::isDomeAtHome()
{
    bool athome;
    int tmp;
    int err = RD_OK;
    char resp[SERIAL_BUFFER_SIZE];
    
    if(!bIsConnected)
        return NOT_CONNECTED;
    
    err = domeCommand("HOME ?\r", resp, SERIAL_BUFFER_SIZE);
    if(err)
        return false;

    athome = false;
    tmp = atoi(resp);
    if(tmp)
        athome = true;
    
    return athome;
  
}

int CRigelDome::syncDome(double dAz, double dEl)
{
    int err = RD_OK;
    char buf[SERIAL_BUFFER_SIZE];
    char resp[SERIAL_BUFFER_SIZE];

    if(!bIsConnected)
        return NOT_CONNECTED;

    mCurrentAzPosition = dAz;
    snprintf(buf, SERIAL_BUFFER_SIZE, "ANGLE K %3.1f\r", dAz);
    err = domeCommand(buf, resp, SERIAL_BUFFER_SIZE);
    if(err)
        return err;
    if(strncmp(resp,"A",1) == 0) {
        err = RD_OK;
    }
    else {
        err = RD_BAD_CMD_RESPONSE;
    }
    return err;
}

int CRigelDome::parkDome()
{
    int err = RD_OK;
    char resp[SERIAL_BUFFER_SIZE];

    if(!bIsConnected)
        return NOT_CONNECTED;

    if(bCalibrating)
        return SB_OK;

    err = domeCommand("GO P\r", resp, SERIAL_BUFFER_SIZE);
    if(err)
        return err;

    if(strncmp(resp,"A",1) == 0) {
        err = RD_OK;
    }
    else {
        err = RD_BAD_CMD_RESPONSE;
    }
    return err;
}

int CRigelDome::unparkDome()
{
    mParked = false;
    mCurrentAzPosition = mParkAz;
    syncDome(mCurrentAzPosition,mCurrentElPosition);
    return 0;
}

int CRigelDome::gotoAzimuth(double newAz)
{

    int err = RD_OK;
    char buf[SERIAL_BUFFER_SIZE];
    char resp[SERIAL_BUFFER_SIZE];

    if(!bIsConnected)
        return NOT_CONNECTED;

    snprintf(buf, SERIAL_BUFFER_SIZE, "GO %3.1f\r", newAz);
    err = domeCommand(buf, resp, SERIAL_BUFFER_SIZE);
    if(err)
        return err;
    if(strncmp(resp,"A",1) == 0) {
        err = RD_OK;
    }
    else {
        err = RD_BAD_CMD_RESPONSE;
    }

    mGotoAz = newAz;

    return err;
}

int CRigelDome::openShutter()
{
    int err = RD_OK;
    char resp[SERIAL_BUFFER_SIZE];

    if(!bIsConnected)
        return NOT_CONNECTED;

    if(bCalibrating)
        return SB_OK;

    err = domeCommand("OPEN\r", resp, SERIAL_BUFFER_SIZE);
    if(err)
        return err;

    if(strncmp(resp,"A",1) == 0) {
        err = RD_OK;
    }
    else {
        err = RD_BAD_CMD_RESPONSE;
    }
    return err;
}

int CRigelDome::closeShutter()
{
    int err = RD_OK;
    char resp[SERIAL_BUFFER_SIZE];

    if(!bIsConnected)
        return NOT_CONNECTED;

    if(bCalibrating)
        return SB_OK;

    err = domeCommand("CLOSE\r", resp, SERIAL_BUFFER_SIZE);
    if(err)
        return err;

    if(strncmp(resp,"A",1) == 0) {
        err = RD_OK;
    }
    else {
        err = RD_BAD_CMD_RESPONSE;
    }
    return err;
}

int CRigelDome::getFirmwareVersion(char *version, int strMaxLen)
{
    int err = 0;
    char resp[SERIAL_BUFFER_SIZE];
    float fVersion;

    if(!bIsConnected)
        return NOT_CONNECTED;

    if(bCalibrating)
        return SB_OK;

    err = domeCommand("VER\r", resp, SERIAL_BUFFER_SIZE);
    if(err)
        return err;

    fVersion = atof(resp);
    snprintf(version,strMaxLen, "%.2f",fVersion);
    return err;
}

int CRigelDome::goHome()
{
    int err = RD_OK;
    char resp[SERIAL_BUFFER_SIZE];

    if(!bIsConnected)
        return NOT_CONNECTED;

    if(bCalibrating)
        return SB_OK;

    err = domeCommand("GO H\r", resp, SERIAL_BUFFER_SIZE);
    if(err)
        return err;

    if(strncmp(resp,"A",1) == 0) {
        err = RD_OK;
    }
    else {
        err = RD_BAD_CMD_RESPONSE;
    }
    return err;
}

int CRigelDome::calibrate()
{
    int err = RD_OK;
    char resp[SERIAL_BUFFER_SIZE];

    if(!bIsConnected)
        return NOT_CONNECTED;

    if(bCalibrating)
        return SB_OK;

    err = domeCommand("CALIBRATE\r", resp, SERIAL_BUFFER_SIZE);
    if(err)
        return err;

    if(strncmp(resp,"A",1) == 0) {
        err = RD_OK;
    }
    else {
        err = RD_BAD_CMD_RESPONSE;
        return err;
    }

    bCalibrating = true;
    
    return err;
}

int CRigelDome::isGoToComplete(bool &complete)
{
    int err = 0;
    double domeAz = 0;

    if(!bIsConnected)
        return NOT_CONNECTED;

    if(isDomeMoving()) {
        complete = false;
        getDomeAz(domeAz);
        return err;
    }

    getDomeAz(domeAz);

    if (ceil(mGotoAz) == ceil(domeAz))
        complete = true;
    else {
        // we're not moving and we're not at the final destination !!!
        if (bDebugLog) {
            snprintf(mLogBuffer,ND_LOG_BUFFER_SIZE,"[CRigelDome::isGoToComplete] domeAz = %f, mGotoAz = %f\n", ceil(domeAz), ceil(mGotoAz));
            mLogger->out(mLogBuffer);
        }
        complete = false;
        err = ERR_CMDFAILED;
    }

    return err;
}

int CRigelDome::isOpenComplete(bool &complete)
{
    int err=0;
    int state;

    if(!bIsConnected)
        return NOT_CONNECTED;

    err = getShutterState(state);
    if(err)
        return ERR_CMDFAILED;
    if(state == OPEN){
        mShutterOpened = true;
        complete = true;
        mCurrentElPosition = 90.0;
    }
    else {
        mShutterOpened = false;
        complete = false;
        mCurrentElPosition = 0.0;
    }

    return err;
}

int CRigelDome::isCloseComplete(bool &complete)
{
    int err=0;
    int state;

    if(!bIsConnected)
        return NOT_CONNECTED;

    err = getShutterState(state);
    if(err)
        return ERR_CMDFAILED;
    if(state == CLOSED){
        mShutterOpened = false;
        complete = true;
        mCurrentElPosition = 0.0;
    }
    else {
        mShutterOpened = true;
        complete = false;
        mCurrentElPosition = 90.0;
    }

    return err;
}


int CRigelDome::isParkComplete(bool &complete)
{
    int err = 0;
    double domeAz=0;

    if(!bIsConnected)
        return NOT_CONNECTED;

    getDomeAz(domeAz);

    if(isDomeMoving()) {
        complete = false;
        return err;
    }

    if (ceil(mParkAz) == ceil(domeAz))
    {
        mParked = true;
        complete = true;
    }
    else {
        // we're not moving and we're not at the final destination !!!
        complete = false;
        mHomed = false;
        mParked = false;
        err = ERR_CMDFAILED;
    }

    return err;
}

int CRigelDome::isUnparkComplete(bool &complete)
{
    int err=0;

    if(!bIsConnected)
        return NOT_CONNECTED;

    mParked = false;
    complete = true;

    return err;
}

int CRigelDome::isFindHomeComplete(bool &complete)
{
    int err = 0;

    if(!bIsConnected)
        return NOT_CONNECTED;

    if(isDomeMoving()) {
        mHomed = false;
        complete = false;
        return err;
    }

    if(isDomeAtHome()){
        mHomed = true;
        complete = true;
    }
    else {
        // we're not moving and we're not at the home position !!!
        if (bDebugLog) {
            snprintf(mLogBuffer,ND_LOG_BUFFER_SIZE,"[CRigelDome::isFindHomeComplete] Not moving and not at home !!!\n");
            mLogger->out(mLogBuffer);
        }
        complete = false;
        mHomed = false;
        mParked = false;
        err = ERR_CMDFAILED;
    }

    return err;
}


int CRigelDome::isCalibratingComplete(bool &complete)
{
    int err = 0;
    double domeAz = 0;

    if(!bIsConnected)
        return NOT_CONNECTED;

    if(isDomeMoving()) {
        getDomeAz(domeAz);
        mHomed = false;
        complete = false;
        return err;
    }

    
    err = getDomeAz(domeAz);

    if (ceil(mHomeAz) != ceil(domeAz)) {
        // We need to resync the current position to the home position.
        mCurrentAzPosition = mHomeAz;
        syncDome(mCurrentAzPosition,mCurrentElPosition);
        mHomed = true;
        complete = true;
    }

    err = getDomeStepPerRev(mNbStepPerRev);
    mHomed = true;
    complete = true;
    bCalibrating = false;
    return err;
}


int CRigelDome::abortCurrentCommand()
{
    if(!bIsConnected)
        return NOT_CONNECTED;

    bCalibrating = false;

    return (domeCommand("STOP\r", NULL, SERIAL_BUFFER_SIZE));
}

#pragma mark - Getter / Setter

int CRigelDome::getNbTicksPerRev()
{
    if(bIsConnected)
        getDomeStepPerRev(mNbStepPerRev);
    return mNbStepPerRev;
}


double CRigelDome::getHomeAz()
{
    if(bIsConnected)
        getDomeHomeAz(mHomeAz);
    return mHomeAz;
}

int CRigelDome::setHomeAz(double dAz)
{
    int err = RD_OK;
    char buf[SERIAL_BUFFER_SIZE];
    char resp[SERIAL_BUFFER_SIZE];

    if(!bIsConnected)
        return NOT_CONNECTED;

    snprintf(buf, SERIAL_BUFFER_SIZE, "HOME %3.1f\r", dAz);
    err = domeCommand(buf, resp, SERIAL_BUFFER_SIZE);
    if(err)
        return err;
    if(strncmp(resp,"A",1) == 0) {
        err = RD_OK;
    }
    else {
        err = RD_BAD_CMD_RESPONSE;
    }
    mHomeAz = dAz;
    return err;
}


double CRigelDome::getParkAz()
{
    if(bIsConnected)
        getDomeParkAz(mParkAz);

    return mParkAz;

}

int CRigelDome::setParkAz(double dAz)
{
    int err = RD_OK;
    char buf[SERIAL_BUFFER_SIZE];
    char resp[SERIAL_BUFFER_SIZE];

    if(!bIsConnected)
        return NOT_CONNECTED;

    snprintf(buf, SERIAL_BUFFER_SIZE, "PARK %3.1f\r", dAz);
    err = domeCommand(buf, resp, SERIAL_BUFFER_SIZE);
    if(err)
        return err;
    if(strncmp(resp,"A",1) == 0) {
        err = RD_OK;
    }
    else {
        err = RD_BAD_CMD_RESPONSE;
    }

    mParkAz = dAz;
    return err;
}


double CRigelDome::getCurrentAz()
{
    if(bIsConnected)
        getDomeAz(mCurrentAzPosition);
    
    return mCurrentAzPosition;
}

double CRigelDome::getCurrentEl()
{
    if(bIsConnected)
        getDomeEl(mCurrentElPosition);
    
    return mCurrentElPosition;
}

int CRigelDome::getCurrentShutterState()
{
    if(bIsConnected)
        getShutterState(mShutterState);

    return mShutterState;
}

