//
//  rigeldome.cpp
//  Rigel rotation drive unit for Pulsar Dome X2 plugin
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
    m_bDebugLog = true;
    
    m_pSerx = NULL;
    m_bIsConnected = false;

    m_nNbStepPerRev = 0;
    m_dShutterBatteryVolts = 0.0;
    
    m_dHomeAz = 180;
    m_dParkAz = 180;

    m_dCurrentAzPosition = 0.0;
    m_dCurrentElPosition = 0.0;

    m_bCalibrating = false;

    m_bHasShutter = false;
    m_bShutterOpened = false;
    
    m_bParked = true;
    m_bHomed = false;
    memset(m_szFirmwareVersion,0,SERIAL_BUFFER_SIZE);
    memset(m_szLogBuffer,0,ND_LOG_BUFFER_SIZE);
}

CRigelDome::~CRigelDome()
{

}

int CRigelDome::Connect(const char *pszPort)
{
    int nErr;
    int nState;

    // 9600 8N1
    if(m_pSerx->open(pszPort, 115200, SerXInterface::B_NOPARITY, "-DTR_CONTROL 1") == 0)
        m_bIsConnected = true;
    else
        m_bIsConnected = false;

    if(!m_bIsConnected)
        return ERR_COMMNOLINK;

    if (m_bDebugLog) {
        snprintf(m_szLogBuffer,ND_LOG_BUFFER_SIZE,"[CRigelDome::Connect] Connected.\n");
        m_pLogger->out(m_szLogBuffer);

        snprintf(m_szLogBuffer,ND_LOG_BUFFER_SIZE,"[CRigelDome::Connect] Getting Firmware.\n");
        m_pLogger->out(m_szLogBuffer);
    }
    // if this fails we're not properly connected.
    nErr = getFirmwareVersion(m_szFirmwareVersion, SERIAL_BUFFER_SIZE);
    if(nErr) {
        if (m_bDebugLog) {
            snprintf(m_szLogBuffer,ND_LOG_BUFFER_SIZE,"[CRigelDome::Connect] Error Getting Firmware.\n");
            m_pLogger->out(m_szLogBuffer);
        }
        m_bIsConnected = false;
        m_pSerx->close();
        return FIRMWARE_NOT_SUPPORTED;
    }

    if (m_bDebugLog) {
        snprintf(m_szLogBuffer,ND_LOG_BUFFER_SIZE,"[CRigelDome::Connect] Got Firmware.\n");
        m_pLogger->out(m_szLogBuffer);
    }
    // assume the dome was parked
    getDomeParkAz(m_dCurrentAzPosition);

    syncDome(m_dCurrentAzPosition,m_dCurrentElPosition);
    nErr = getShutterState(nState);

    if(nState != NOT_FITTED && nState != UNKNOWN )
        m_bHasShutter = true;

    return SB_OK;
}


void CRigelDome::Disconnect()
{
    if(m_bIsConnected) {
        m_pSerx->purgeTxRx();
        m_pSerx->close();
    }
    m_bIsConnected = false;
}


int CRigelDome::readResponse(char *pszRespBuffer, int nBufferLen)
{
    int nErr = RD_OK;
    unsigned long ulBytesRead = 0;
    unsigned long ulTotalBytesRead = 0;
    char *pszBufPtr;

    memset(pszRespBuffer, 0, (size_t) nBufferLen);
    pszBufPtr = pszRespBuffer;

    do {
        nErr = m_pSerx->readFile(pszBufPtr, 1, ulBytesRead, MAX_TIMEOUT);
        if(nErr) {
            if (m_bDebugLog) {
                snprintf(m_szLogBuffer,ND_LOG_BUFFER_SIZE,"[CRigelDome::readResponse] readFile error.\n");
                m_pLogger->out(m_szLogBuffer);
            }
            return nErr;
        }

        if (m_bDebugLog) {
            snprintf(m_szLogBuffer,ND_LOG_BUFFER_SIZE,"[CRigelDome::readResponse] respBuffer = %s\n",pszRespBuffer);
            m_pLogger->out(m_szLogBuffer);
        }
        
        if (ulBytesRead !=1) {// timeout
            if (m_bDebugLog) {
                snprintf(m_szLogBuffer,ND_LOG_BUFFER_SIZE,"[CRigelDome::readResponse] readFile Timeout.\n");
                m_pLogger->out(m_szLogBuffer);
            }
            nErr = RD_BAD_CMD_RESPONSE;
            break;
        }
        ulTotalBytesRead += ulBytesRead;
        if (m_bDebugLog) {
            snprintf(m_szLogBuffer,ND_LOG_BUFFER_SIZE,"[CRigelDome::readResponse] nBytesRead = %lu\n",ulBytesRead);
            m_pLogger->out(m_szLogBuffer);
        }
    } while (*pszBufPtr++ != 0x0D && ulTotalBytesRead < nBufferLen );

    if(ulTotalBytesRead)
        *(pszBufPtr-1) = 0; //remove the \r

    return nErr;
}


int CRigelDome::domeCommand(const char *pszCmd, char *pszResult, int nResultMaxLen)
{
    int nErr = RD_OK;
    char szResp[SERIAL_BUFFER_SIZE];
    unsigned long  ulBytesWrite;

    m_pSerx->purgeTxRx();
    if (m_bDebugLog) {
        snprintf(m_szLogBuffer,ND_LOG_BUFFER_SIZE,"[CRigelDome::domeCommand] Sending %s\n",pszCmd);
        m_pLogger->out(m_szLogBuffer);
    }
    nErr = m_pSerx->writeFile((void *)pszCmd, strlen(pszCmd), ulBytesWrite);
    m_pSerx->flushTx();
    if(nErr)
        return nErr;
    // read response
    if (m_bDebugLog) {
        snprintf(m_szLogBuffer,ND_LOG_BUFFER_SIZE,"[CRigelDome::domeCommand] Getting response.\n");
        m_pLogger->out(m_szLogBuffer);
    }
    nErr = readResponse(szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    if(pszResult)
        strncpy(pszResult, &szResp[1], nResultMaxLen);

    return nErr;

}

int CRigelDome::getDomeAz(double &dDomeAz)
{
    int nErr = RD_OK;
    char szResp[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    if(m_bCalibrating)
        return nErr;

    nErr = domeCommand("ANGLE\r", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;
    
    // convert Az string to double
    dDomeAz = atof(szResp);
    m_dCurrentAzPosition = dDomeAz;

    return nErr;
}

int CRigelDome::getDomeEl(double &dDomeEl)
{
    int nErr = RD_OK;
    int nShutterState;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    getShutterState(nShutterState);

    if(!m_bShutterOpened || !m_bHasShutter)
    {
        dDomeEl = 0.0;
    }
    else {
        dDomeEl = 90.0;
    }

    m_dCurrentElPosition = dDomeEl;
    
    return nErr;
}


int CRigelDome::getDomeHomeAz(double &dAz)
{
    int nErr = RD_OK;
    char szResp[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    if(m_bCalibrating)
        return nErr;

    nErr = domeCommand("HOME\r", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    // convert Az string to double
    dAz = atof(szResp);
    m_dHomeAz = dAz;
    return nErr;
}

int CRigelDome::getDomeParkAz(double &dAz)
{
    int nErr = RD_OK;
    char szResp[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    if(m_bCalibrating)
        return nErr;

    nErr = domeCommand("PARK\r", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    // convert Az string to double
    dAz = atof(szResp);
    m_dParkAz = dAz;
    return nErr;
}


int CRigelDome::getShutterState(int &nState)
{
    int nErr = RD_OK;
    char szResp[SERIAL_BUFFER_SIZE];
    int nShutterState;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    if(m_bCalibrating)
        return nErr;

    nErr = domeCommand("SHUTTER\r", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;
    nErr = domeCommand("SHUTTER\r", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

	nShutterState = atoi(szResp);
    switch(nShutterState) {
        case OPEN:
            m_bShutterOpened = true;
            break;

        case CLOSED:
            m_bShutterOpened = false;
            break;

        case NOT_FITTED:
            m_bShutterOpened = false;
            m_bHasShutter = false;
            break;
        default:
            m_bShutterOpened = false;
            
    }

    nState = atoi(szResp);

    return nErr;
}


int CRigelDome::getDomeStepPerRev(int &nStepPerRev)
{
    int nErr = RD_OK;
    char szResp[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    nErr = domeCommand("ENCREV\r", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    nStepPerRev = atoi(szResp);
    m_nNbStepPerRev = nStepPerRev;
    return nErr;
}

int CRigelDome::getBatteryLevels(double &dShutterVolts, int &nPercent)
{
    int nErr = RD_OK;
    int rc = 0;
    char szResp[SERIAL_BUFFER_SIZE];
    
    if(!m_bIsConnected)
        return NOT_CONNECTED;

    if(m_bCalibrating)
        return nErr;
    
    nErr = domeCommand("BAT\r", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    rc = sscanf(szResp, "%d %lf", &nPercent, &dShutterVolts);
    if(rc == 0) {
        return COMMAND_FAILED;
    }

    dShutterVolts = dShutterVolts / 1000.0;
    return nErr;
}

bool CRigelDome::hasShutterUnit() {
    return m_bHasShutter;
}

void CRigelDome::setDebugLog(bool bEnable)
{
    m_bDebugLog = bEnable;
}

int CRigelDome::isDomeMoving(bool &bIsMoving)
{
    int tmp;
    int nErr = RD_OK;
    char szResp[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    nErr = domeCommand("MSTATE\r", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    bIsMoving = false;
    tmp = atoi(szResp);
    if(tmp != 0 || tmp != 3)
        bIsMoving = true;

    return nErr;
}

int CRigelDome::isDomeAtHome(bool &bAtHome)
{
    int tmp;
    int nErr = RD_OK;
    char resp[SERIAL_BUFFER_SIZE];
    
    if(!m_bIsConnected)
        return NOT_CONNECTED;
    
    nErr = domeCommand("HOME ?\r", resp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return false;

    bAtHome = false;
    tmp = atoi(resp);
    if(tmp)
        bAtHome = true;
    
    return nErr;
  
}

int CRigelDome::syncDome(double dAz, double dEl)
{
    int nErr = RD_OK;
    char szBuf[SERIAL_BUFFER_SIZE];
    char szResp[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    m_dCurrentAzPosition = dAz;
    snprintf(szBuf, SERIAL_BUFFER_SIZE, "ANGLE K %3.1f\r", dAz);
    nErr = domeCommand(szBuf, szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;
    if(strncmp(szResp,"A",1) == 0) {
        nErr = RD_OK;
    }
    else {
        nErr = RD_BAD_CMD_RESPONSE;
    }
    return nErr;
}

int CRigelDome::parkDome()
{
    int nErr = RD_OK;
    char szResp[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    if(m_bCalibrating)
        return SB_OK;

    nErr = domeCommand("GO P\r", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    if(strncmp(szResp,"A",1) == 0) {
        nErr = RD_OK;
    }
    else {
        nErr = RD_BAD_CMD_RESPONSE;
    }
    return nErr;
}

int CRigelDome::unparkDome()
{
    m_bParked = false;
    m_dCurrentAzPosition = m_dParkAz;
    syncDome(m_dCurrentAzPosition,m_dCurrentElPosition);
    return 0;
}

int CRigelDome::gotoAzimuth(double dNewAz)
{

    int nErr = RD_OK;
    char szBuf[SERIAL_BUFFER_SIZE];
    char szResp[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    snprintf(szBuf, SERIAL_BUFFER_SIZE, "GO %3.1f\r", dNewAz);
    nErr = domeCommand(szBuf, szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    if(strncmp(szResp,"A",1) == 0) {
        nErr = RD_OK;
    }
    else {
        nErr = RD_BAD_CMD_RESPONSE;
    }

    m_dGotoAz = dNewAz;

    return nErr;
}

int CRigelDome::openShutter()
{
    int nErr = RD_OK;
    char szResp[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    if(m_bCalibrating)
        return SB_OK;

    nErr = domeCommand("OPEN\r", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    if(strncmp(szResp,"A",1) == 0) {
        nErr = RD_OK;
    }
    else {
        nErr = RD_BAD_CMD_RESPONSE;
    }
    return nErr;
}

int CRigelDome::closeShutter()
{
    int nErr = RD_OK;
    char szResp[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    if(m_bCalibrating)
        return SB_OK;

    nErr = domeCommand("CLOSE\r", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    if(strncmp(szResp,"A",1) == 0) {
        nErr = RD_OK;
    }
    else {
        nErr = RD_BAD_CMD_RESPONSE;
    }
    return nErr;
}

int CRigelDome::getFirmwareVersion(char *pszVersion, int nStrMaxLen)
{
    int nErr = RD_OK;
    char szResp[SERIAL_BUFFER_SIZE];
    double dVersion;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    if(m_bCalibrating)
        return SB_OK;

    nErr = domeCommand("VER\r", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    dVersion = atof(szResp);
    snprintf(pszVersion, nStrMaxLen, "%.2f",dVersion);
    return nErr;
}

int CRigelDome::getModel(char *pszModel, int nStrMaxLen)
{
    int nErr = RD_OK;
    char szResp[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    if(m_bCalibrating)
        return SB_OK;

    nErr = domeCommand("PULSAR\r", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    strncpy(pszModel, szResp, nStrMaxLen);
    return nErr;
}

int CRigelDome::goHome()
{
    int nErr = RD_OK;
    char szResp[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    if(m_bCalibrating)
        return SB_OK;

    nErr = domeCommand("GO H\r", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    if(strncmp(szResp,"A",1) == 0) {
        nErr = RD_OK;
    }
    else {
        nErr = RD_BAD_CMD_RESPONSE;
    }
    return nErr;
}

int CRigelDome::calibrate()
{
    int nErr = RD_OK;
    char resp[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    if(m_bCalibrating)
        return SB_OK;

    nErr = domeCommand("CALIBRATE\r", resp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    if(strncmp(resp,"A",1) == 0) {
        nErr = RD_OK;
    }
    else {
        nErr = RD_BAD_CMD_RESPONSE;
        return nErr;
    }

    m_bCalibrating = true;
    
    return nErr;
}

int CRigelDome::isGoToComplete(bool &bComplete)
{
    int nErr = 0;
    double dDomeAz = 0;
    bool bIsMoving = false;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    nErr = isDomeMoving(bIsMoving);
    if(nErr) {
        return nErr;
        }

    if(bIsMoving) {
        bComplete = false;
        getDomeAz(dDomeAz);
        return nErr;
    }

    getDomeAz(dDomeAz);

    if (ceil(m_dGotoAz) == ceil(dDomeAz))
        bComplete = true;
    else {
        // we're not moving and we're not at the final destination !!!
        if (m_bDebugLog) {
            snprintf(m_szLogBuffer,ND_LOG_BUFFER_SIZE,"[CRigelDome::isGoToComplete] domeAz = %f, mGotoAz = %f\n", ceil(dDomeAz), ceil(m_dGotoAz));
            m_pLogger->out(m_szLogBuffer);
        }
        bComplete = false;
        nErr = ERR_CMDFAILED;
    }

    return nErr;
}

int CRigelDome::isOpenComplete(bool &bComplete)
{
    int nErr = 0;
    int nState;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    nErr = getShutterState(nState);
    if(nErr)
        return ERR_CMDFAILED;
    if(nState == OPEN){
        m_bShutterOpened = true;
        bComplete = true;
        m_dCurrentElPosition = 90.0;
    }
    else {
        m_bShutterOpened = false;
        bComplete = false;
        m_dCurrentElPosition = 0.0;
    }

    return nErr;
}

int CRigelDome::isCloseComplete(bool &bComplete)
{
    int err=0;
    int nState;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    err = getShutterState(nState);
    if(err)
        return ERR_CMDFAILED;
    if(nState == CLOSED){
        m_bShutterOpened = false;
        bComplete = true;
        m_dCurrentElPosition = 0.0;
    }
    else {
        m_bShutterOpened = true;
        bComplete = false;
        m_dCurrentElPosition = 90.0;
    }

    return err;
}


int CRigelDome::isParkComplete(bool &bComplete)
{
    int nErr = 0;
    double dDomeAz=0;
    bool bIsMoving = false;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    getDomeAz(dDomeAz);
    nErr = isDomeMoving(bIsMoving);
    if(nErr)
        return nErr;

    if(bIsMoving) {
        bComplete = false;
        return nErr;
    }

    if (ceil(m_dParkAz) == ceil(dDomeAz))
    {
        m_bParked = true;
        bComplete = true;
    }
    else {
        // we're not moving and we're not at the final destination !!!
        bComplete = false;
        m_bHomed = false;
        m_bParked = false;
        nErr = ERR_CMDFAILED;
    }

    return nErr;
}

int CRigelDome::isUnparkComplete(bool &bComplete)
{
    int nErr = 0;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    m_bParked = false;
    bComplete = true;

    return nErr;
}

int CRigelDome::isFindHomeComplete(bool &bComplete)
{
    int nErr = 0;
    bool bIsMoving = false;
    bool bIsAtHome = false;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    nErr = isDomeMoving(bIsMoving);
    if(nErr)
        return nErr;

    if(bIsMoving) {
        m_bHomed = false;
        bComplete = false;
        return nErr;
    }

    nErr = isDomeAtHome(bIsAtHome);
    if(nErr)
        return nErr;

    if(bIsAtHome){
        m_bHomed = true;
        bComplete = true;
    }
    else {
        // we're not moving and we're not at the home position !!!
        if (m_bDebugLog) {
            snprintf(m_szLogBuffer,ND_LOG_BUFFER_SIZE,"[CRigelDome::isFindHomeComplete] Not moving and not at home !!!\n");
            m_pLogger->out(m_szLogBuffer);
        }
        bComplete = false;
        m_bHomed = false;
        m_bParked = false;
        nErr = ERR_CMDFAILED;
    }

    return nErr;
}


int CRigelDome::isCalibratingComplete(bool &bComplete)
{
    int nErr = 0;
    double dDomeAz = 0;
    bool bIsMoving = false;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    nErr = isDomeMoving(bIsMoving);
    if(nErr)
        return nErr;

    if(bIsMoving) {
        getDomeAz(dDomeAz);
        m_bHomed = false;
        bComplete = false;
        return nErr;
    }
    
    nErr = getDomeAz(dDomeAz);

    if (ceil(m_dHomeAz) != ceil(dDomeAz)) {
        // We need to resync the current position to the home position.
        m_dCurrentAzPosition = m_dHomeAz;
        syncDome(m_dCurrentAzPosition,m_dCurrentElPosition);
        m_bHomed = true;
        bComplete = true;
    }

    nErr = getDomeStepPerRev(m_nNbStepPerRev);
    m_bHomed = true;
    bComplete = true;
    m_bCalibrating = false;
    return nErr;
}


int CRigelDome::abortCurrentCommand()
{
    if(!m_bIsConnected)
        return NOT_CONNECTED;

    m_bCalibrating = false;

    return (domeCommand("STOP\r", NULL, SERIAL_BUFFER_SIZE));
}

#pragma mark - Getter / Setter

int CRigelDome::getNbTicksPerRev()
{
    if(m_bIsConnected)
        getDomeStepPerRev(m_nNbStepPerRev);
    return m_nNbStepPerRev;
}


double CRigelDome::getHomeAz()
{
    if(m_bIsConnected)
        getDomeHomeAz(m_dHomeAz);

    return m_dHomeAz;
}

int CRigelDome::setHomeAz(double dAz)
{
    int nErr = RD_OK;
    char szBuf[SERIAL_BUFFER_SIZE];
    char szResp[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    snprintf(szBuf, SERIAL_BUFFER_SIZE, "HOME %3.1f\r", dAz);
    nErr = domeCommand(szBuf, szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    if(strncmp(szResp,"A",1) == 0) {
        nErr = RD_OK;
    }
    else {
        nErr = RD_BAD_CMD_RESPONSE;
    }
    m_dHomeAz = dAz;
    return nErr;
}


double CRigelDome::getParkAz()
{
    if(m_bIsConnected)
        getDomeParkAz(m_dParkAz);

    return m_dParkAz;

}

int CRigelDome::setParkAz(double dAz)
{
    int nErr = RD_OK;
    char szBuf[SERIAL_BUFFER_SIZE];
    char szResp[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    snprintf(szBuf, SERIAL_BUFFER_SIZE, "PARK %3.1f\r", dAz);
    nErr = domeCommand(szBuf, szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    if(strncmp(szResp,"A",1) == 0) {
        nErr = RD_OK;
    }
    else {
        nErr = RD_BAD_CMD_RESPONSE;
    }

    m_dParkAz = dAz;
    return nErr;
}


double CRigelDome::getCurrentAz()
{
    if(m_bIsConnected)
        getDomeAz(m_dCurrentAzPosition);
    
    return m_dCurrentAzPosition;
}

double CRigelDome::getCurrentEl()
{
    if(m_bIsConnected)
        getDomeEl(m_dCurrentElPosition);
    
    return m_dCurrentElPosition;
}

int CRigelDome::getCurrentShutterState()
{
    if(m_bIsConnected)
        getShutterState(m_nShutterState);

    return m_nShutterState;
}

