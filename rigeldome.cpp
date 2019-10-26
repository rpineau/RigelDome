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
    m_pLogger = NULL;
    
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

#ifdef RIGEL_DEBUG
#if defined(SB_WIN_BUILD)
    m_sLogfilePath = getenv("HOMEDRIVE");
    m_sLogfilePath += getenv("HOMEPATH");
    m_sLogfilePath += "\\RigelLog.txt";
#elif defined(SB_LINUX_BUILD)
    m_sLogfilePath = getenv("HOME");
    m_sLogfilePath += "/RigelLog.txt";
#elif defined(SB_MAC_BUILD)
    m_sLogfilePath = getenv("HOME");
    m_sLogfilePath += "/RigelLog.txt";
#endif
    Logfile = fopen(m_sLogfilePath.c_str(), "w");
#endif

#if defined RIGEL_DEBUG && RIGEL_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] CRigelDome Constructor Called\n", timestamp);
    fflush(Logfile);
#endif

}

CRigelDome::~CRigelDome()
{

}

int CRigelDome::Connect(const char *pszPort)
{
    int nErr;
    int nState;

#if defined RIGEL_DEBUG && RIGEL_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CRigelDome::Connect] Connect called.\n", timestamp);
    fflush(Logfile);
#endif

    // 115200 8N1
    if(m_pSerx->open(pszPort, 115200, SerXInterface::B_NOPARITY, "-DTR_CONTROL 1") == 0)
        m_bIsConnected = true;
    else
        m_bIsConnected = false;

    if(!m_bIsConnected)
        return ERR_COMMNOLINK;

#if defined RIGEL_DEBUG && RIGEL_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CRigelDome::Connect] Connected.\n", timestamp);
    fprintf(Logfile, "[%s] [CRigelDome::Connect] Getting Firmware.\n", timestamp);
    fflush(Logfile);
#endif

    // if this fails we're not properly connected.
    nErr = getFirmwareVersion(m_szFirmwareVersion, SERIAL_BUFFER_SIZE);
    if(nErr) {
#if defined RIGEL_DEBUG && RIGEL_DEBUG >= 2
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CRigelDome::Connect] Error Getting Firmware : err = %d\n", timestamp, nErr);
        fflush(Logfile);
#endif
        m_bIsConnected = false;
        m_pSerx->close();
        return ERR_CMDFAILED;
    }

#if defined RIGEL_DEBUG && RIGEL_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CRigelDome::Connect] Got Firmware : %s\n", timestamp, m_szFirmwareVersion);
    fflush(Logfile);
#endif
    nErr = connectToShutter();
    // nErr = btForce();
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
#if defined RIGEL_DEBUG && RIGEL_DEBUG >= 2
            ltime = time(NULL);
            timestamp = asctime(localtime(&ltime));
            timestamp[strlen(timestamp) - 1] = 0;
            fprintf(Logfile, "[%s] [CRigelDome::readResponse] readFile error.\n", timestamp);
            fflush(Logfile);
#endif
            return nErr;
        }

        if (ulBytesRead !=1) {// timeout
#if defined RIGEL_DEBUG && RIGEL_DEBUG >= 2
            ltime = time(NULL);
            timestamp = asctime(localtime(&ltime));
            timestamp[strlen(timestamp) - 1] = 0;
            fprintf(Logfile, "[%s] [CRigelDome::readResponse] readFile Timeout.\n", timestamp);
            fflush(Logfile);
#endif
            nErr = RD_BAD_CMD_RESPONSE;
            break;
        }
        ulTotalBytesRead += ulBytesRead;
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

#if defined RIGEL_DEBUG && RIGEL_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CRigelDome::domeCommand] Sending %s\n", timestamp, pszCmd);
    fflush(Logfile);
#endif

    nErr = m_pSerx->writeFile((void *)pszCmd, strlen(pszCmd), ulBytesWrite);
    m_pSerx->flushTx();
    if(nErr)
        return nErr;

    // read response
    nErr = readResponse(szResp, SERIAL_BUFFER_SIZE);
#if defined RIGEL_DEBUG && RIGEL_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CRigelDome::domeCommand] response  code is %d with data : %s\n", timestamp, nErr, szResp);
    fflush(Logfile);
#endif

    if(nErr)
        return nErr;

    if(pszResult)
        strncpy(pszResult, szResp, nResultMaxLen);

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
    bool bShutterConnected;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    if(m_bCalibrating)
        return nErr;

    nErr = isConnectedToShutter(bShutterConnected);
    if(nErr)
        return nErr;

    if(!bShutterConnected)
        return NOT_CONNECTED;

    nErr = domeCommand("SHUTTER\r", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

	nShutterState = atoi(szResp);
    m_bHasShutter = true;

    switch(nShutterState) {
        case OPEN:
            m_bShutterOpened = true;
            break;

        case CLOSED:
            m_bShutterOpened = false;
            break;

        case OPENING:
        case CLOSING:
            break;

        //case NOT_FITTED:
        //    m_bShutterOpened = false;
            // m_bHasShutter = false;
            // temp fix
        //    m_bHasShutter = true;
        //    break;
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
    if(tmp != 0 && tmp != 3)
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

    bAtHome = false;

    nErr = domeCommand("HOME ?\r", resp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return ERR_CMDFAILED;

    tmp = atoi(resp);
    if(tmp)
        bAtHome = true;
    
    return nErr;
  
}

int CRigelDome::connectToShutter()
{
    int nErr = RD_OK;
    char resp[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    nErr = domeCommand("BBOND 1\r", resp, SERIAL_BUFFER_SIZE);
    return nErr;
}

int CRigelDome::isConnectedToShutter(bool &bConnected)
{
    int tmp;
    int nErr = RD_OK;
    char resp[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return NOT_CONNECTED;
    bConnected = false;

    nErr = domeCommand("BBOND\r", resp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    tmp = atoi(resp);
    if(tmp)
        bConnected = true;

    return nErr;
}

int CRigelDome::btForce()
{
    int nErr = RD_OK;
    char resp[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    nErr = domeCommand("BTFORCE\r", resp, SERIAL_BUFFER_SIZE);
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

    if(m_bDebugLog && m_pLogger) {
        char szEventLogMsg[256];
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        sprintf(szEventLogMsg, "[%s] Opening Shutter", timestamp);
        m_pLogger->out(szEventLogMsg);
    }
    
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

    if(m_bDebugLog && m_pLogger) {
        char szEventLogMsg[256];
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        sprintf(szEventLogMsg, "[%s] Closing Shutter", timestamp);
        m_pLogger->out(szEventLogMsg);
    }

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

    getDomeAz(dDomeAz);

    if(bIsMoving) {
#if defined RIGEL_DEBUG && RIGEL_DEBUG >= 2
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CRigelDome::isGoToComplete] Dome is moving, domeAz = %f, mGotoAz = %f\n", timestamp, ceil(dDomeAz), ceil(m_dGotoAz));
        fflush(Logfile);
#endif
        bComplete = false;
        return nErr;
    }

#if defined RIGEL_DEBUG && RIGEL_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CRigelDome::isGoToComplete] Dome is NOT moving, domeAz = %f, mGotoAz = %f\n", timestamp, ceil(dDomeAz), ceil(m_dGotoAz));
    fflush(Logfile);
#endif

    if ((floor(m_dGotoAz) <= floor(dDomeAz)+1) && (floor(m_dGotoAz) >= floor(dDomeAz)-1)) {
#if defined RIGEL_DEBUG && RIGEL_DEBUG >= 2
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CRigelDome::isGoToComplete] GOTO completed.\n", timestamp);
        fflush(Logfile);
#endif
        bComplete = true;
    }
    else {
        // we're not moving and we're not at the final destination !!!
#if defined RIGEL_DEBUG && RIGEL_DEBUG >= 2
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CRigelDome::isGoToComplete] GOTO ERROR, not moving but not at target\n", timestamp);
        fflush(Logfile);
#endif
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
        if(m_bDebugLog && m_pLogger) {
            char szEventLogMsg[256];
            ltime = time(NULL);
            timestamp = asctime(localtime(&ltime));
            timestamp[strlen(timestamp) - 1] = 0;
            sprintf(szEventLogMsg, "[%s] Shutter Openned", timestamp);
            m_pLogger->out(szEventLogMsg);
        }
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
        if(m_bDebugLog && m_pLogger) {
            char szEventLogMsg[256];
            ltime = time(NULL);
            timestamp = asctime(localtime(&ltime));
            timestamp[strlen(timestamp) - 1] = 0;
            sprintf(szEventLogMsg, "[%s] Shutter Closed", timestamp);
            m_pLogger->out(szEventLogMsg);
        }
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

    if ((floor(m_dParkAz) <= floor(dDomeAz)+1) && (floor(m_dGotoAz) >= floor(dDomeAz)-1)) {
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

    bComplete = false;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    // new version of the test
    nErr = getExtendedState();
    if(nErr)
        return nErr;

    if(m_nMotorState == CALIBRATIG) {
        bComplete = false;
        return nErr;
    }
    else if (m_nMotorState == IDLE){
        bComplete = true;
        return nErr;
    }
    else {
        // probably still moving
        bComplete = false;
        return nErr;
    }

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

int CRigelDome::getExtendedState()
{
    int nErr = RD_OK;
    char szResp[SERIAL_BUFFER_SIZE];
    std::vector<std::string> vFields;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    nErr = domeCommand("V\r", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;
    // szResp contains the 13 state fields.
    nErr = parseFields(szResp, vFields, '\t');
    if(vFields.size()>=13) {
        m_dCurrentAzPosition = atof(vFields[0].c_str());
        m_nMotorState = atoi(vFields[1].c_str());
        m_nShutterState = atoi(vFields[5].c_str());
        switch(m_nShutterState) {
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
    }
    return nErr;
}


int CRigelDome::parseFields(const char *pszResp, std::vector<std::string> &svFields, char cSeparator)
{
    int nErr = RD_OK;
    std::string sSegment;
    std::stringstream ssTmp(pszResp);

    svFields.clear();
    // split the string into vector elements
    while(std::getline(ssTmp, sSegment, cSeparator))
    {
        svFields.push_back(sSegment);
    }

    if(svFields.size()==0) {
        nErr = ERR_CMDFAILED;
    }
    return nErr;
}
