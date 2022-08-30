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

	m_cmdDelayCheckTimer.Reset();

#ifdef PLUGIN_DEBUG
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

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 1
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CRigelDome::CRigelDome] Version %3.2f build %s %s\n", timestamp, DRIVER_VERSION, __DATE__, __TIME__);
    fprintf(Logfile, "[%s] [CRigelDome Constructor] Called\n", timestamp);
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

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 1
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CRigelDome::Connect] Called.\n", timestamp);
    fflush(Logfile);
#endif

    // 115200 8N1
    if(m_pSerx->open(pszPort, 115200, SerXInterface::B_NOPARITY, "-DTR_CONTROL 1") == 0)
        m_bIsConnected = true;
    else
        m_bIsConnected = false;

    if(!m_bIsConnected)
        return ERR_COMMNOLINK;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
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
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
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

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
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

    getDomeAz(m_dCurrentAzPosition);
    getDomeAz(m_dGotoAz);
    
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
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 3
            ltime = time(NULL);
            timestamp = asctime(localtime(&ltime));
            timestamp[strlen(timestamp) - 1] = 0;
            fprintf(Logfile, "[%s] [CRigelDome::readResponse] readFile error.\n", timestamp);
            fflush(Logfile);
#endif
            return nErr;
        }

        if (ulBytesRead !=1) {// timeout
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 3
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

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 3
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
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 3
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

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 1
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CRigelDome::getDomeAz] Called.\n", timestamp);
    fflush(Logfile);
#endif


    if(m_bCalibrating)
        return nErr;

    nErr = domeCommand("ANGLE\r", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;
    
    // convert Az string to double
    dDomeAz = atof(szResp);
    m_dCurrentAzPosition = dDomeAz;

	// Check Shutter state from time to time and if it changed, log the change.
	if(m_cmdDelayCheckTimer.GetElapsedSeconds()>SHUTTER_CHECK_WAIT) {
		m_cmdDelayCheckTimer.Reset();
		nErr = getShutterState(m_nShutterState);
		if(nErr)
			return nErr;
		if(m_nPreviousShutterState != m_nShutterState) {
			m_nPreviousShutterState = m_nShutterState;
			switch(m_nShutterState) {
				case OPEN :
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
                    ltime = time(NULL);
                    timestamp = asctime(localtime(&ltime));
                    timestamp[strlen(timestamp) - 1] = 0;
                    fprintf(Logfile, "[%s] [CRigelDome::getDomeAz] Shutter Opened\n", timestamp);
                    fflush(Logfile);
#endif
					if(m_bDebugLog && m_pLogger) {
						char szEventLogMsg[256];
						ltime = time(NULL);
						timestamp = asctime(localtime(&ltime));
						timestamp[strlen(timestamp) - 1] = 0;
						snprintf(szEventLogMsg, 256, "[%s] [CRigelDome::getDomeAz] Shutter Opened", timestamp);
						m_pLogger->out(szEventLogMsg);
					}
					break;
				case CLOSED :
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
                    ltime = time(NULL);
                    timestamp = asctime(localtime(&ltime));
                    timestamp[strlen(timestamp) - 1] = 0;
                    fprintf(Logfile, "[%s] [CRigelDome::getDomeAz] Shutter Closed\n", timestamp);
                    fflush(Logfile);
#endif
					if(m_bDebugLog && m_pLogger) {
						char szEventLogMsg[256];
						ltime = time(NULL);
						timestamp = asctime(localtime(&ltime));
						timestamp[strlen(timestamp) - 1] = 0;
						snprintf(szEventLogMsg, 256, "[%s] [CRigelDome::getDomeAz] Shutter Closed", timestamp);
						m_pLogger->out(szEventLogMsg);
					}
					break;
				default:
					break;
			}
		}
	}

    return nErr;
}

int CRigelDome::getDomeEl(double &dDomeEl)
{
    int nErr = RD_OK;
    int nShutterState;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 1
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CRigelDome::getDomeEl] Called.\n", timestamp);
    fflush(Logfile);
#endif

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

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 1
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CRigelDome::getDomeHomeAz] Called.\n", timestamp);
    fflush(Logfile);
#endif


    if(m_bCalibrating)
        return nErr;

    nErr = domeCommand("HOME\r", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    // convert Az string to double
    dAz = atof(szResp);
    m_dHomeAz = dAz;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CRigelDome::getDomeHomeAz] Home Az = %3.1f\n", timestamp,m_dHomeAz);
    fflush(Logfile);
#endif

    return nErr;
}

int CRigelDome::getDomeParkAz(double &dAz)
{
    int nErr = RD_OK;
    char szResp[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return NOT_CONNECTED;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 1
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CRigelDome::getDomeParkAz] Called.\n", timestamp);
    fflush(Logfile);
#endif

    if(m_bCalibrating)
        return nErr;

    nErr = domeCommand("PARK\r", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    // convert Az string to double
    dAz = atof(szResp);
    m_dParkAz = dAz;
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CRigelDome::getDomeParkAz] Park Az = %3.1f\n", timestamp,m_dHomeAz);
    fflush(Logfile);
#endif
    return nErr;
}


int CRigelDome::getShutterState(int &nState)
{
    int nErr = RD_OK;
    char szResp[SERIAL_BUFFER_SIZE];
	bool bShutterConnected;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 1
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CRigelDome::getShutterState] Called.\n", timestamp);
    fflush(Logfile);
#endif

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

	nState = atoi(szResp);
    m_bHasShutter = true;

    switch(nState) {
        case OPEN:
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
            ltime = time(NULL);
            timestamp = asctime(localtime(&ltime));
            timestamp[strlen(timestamp) - 1] = 0;
            fprintf(Logfile, "[%s] [CRigelDome::getShutterState] Shutter is open, state = %d\n", timestamp, nState);
            fflush(Logfile);
#endif
            m_bShutterOpened = true;
            break;

        case CLOSED:
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
            ltime = time(NULL);
            timestamp = asctime(localtime(&ltime));
            timestamp[strlen(timestamp) - 1] = 0;
            fprintf(Logfile, "[%s] [CRigelDome::getShutterState] Shutter is closed, state = %d\n", timestamp, nState);
            fflush(Logfile);
#endif
            m_bShutterOpened = false;
            break;

        case OPENING:
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
            ltime = time(NULL);
            timestamp = asctime(localtime(&ltime));
            timestamp[strlen(timestamp) - 1] = 0;
            fprintf(Logfile, "[%s] [CRigelDome::getShutterState] Shutter is opening, state = %d\n", timestamp, nState);
            fflush(Logfile);
#endif
            break;

        case CLOSING:
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
            ltime = time(NULL);
            timestamp = asctime(localtime(&ltime));
            timestamp[strlen(timestamp) - 1] = 0;
            fprintf(Logfile, "[%s] [CRigelDome::getShutterState] Shutter is closing, state = %d\n", timestamp, nState);
            fflush(Logfile);
#endif
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

    return nErr;
}


int CRigelDome::getDomeStepPerRev(int &nStepPerRev)
{
    int nErr = RD_OK;
    char szResp[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return NOT_CONNECTED;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 1
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CRigelDome::getDomeStepPerRev] Called.\n", timestamp);
    fflush(Logfile);
#endif

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

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 1
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CRigelDome::getBatteryLevels] Called.\n", timestamp);
    fflush(Logfile);
#endif

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

    if(m_pLogger) {
        char szEventLogMsg[256];
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
		if(m_bDebugLog)
			snprintf(szEventLogMsg, 256, "[%s] Shuttem event logging enabled", timestamp);
		else
			snprintf(szEventLogMsg, 256, "[%s] Shuttem event logging disabled", timestamp);
        m_pLogger->out(szEventLogMsg);
    }
}

void CRigelDome::logString(const char *message)
{
    #if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CRigelDome::logString]  %s\n", timestamp, message);
        fflush(Logfile);
    #endif

    if(m_bDebugLog && m_pLogger) {
        char szEventLogMsg[256];
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        snprintf(szEventLogMsg, 256, "[%s] %s", timestamp, message);
        m_pLogger->out(szEventLogMsg);
    }

}


int CRigelDome::isDomeMoving(bool &bIsMoving)
{
    int tmp;
    int nErr = RD_OK;
    char szResp[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return NOT_CONNECTED;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 1
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CRigelDome::isDomeMoving] Called.\n", timestamp);
    fflush(Logfile);
#endif

    nErr = domeCommand("MSTATE\r", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    bIsMoving = false;
    tmp = atoi(szResp);
    if(tmp != 0 && tmp != 3)
        bIsMoving = true;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CRigelDome::isDomeMoving] Dome moving = %s\n", timestamp, bIsMoving?"Yes":"No");
    fflush(Logfile);
#endif

    return nErr;
}

int CRigelDome::isDomeAtHome(bool &bAtHome)
{
    int tmp;
    int nErr = RD_OK;
    char resp[SERIAL_BUFFER_SIZE];
    
    if(!m_bIsConnected)
        return NOT_CONNECTED;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 1
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CRigelDome::isDomeAtHome] Called.\n", timestamp);
    fflush(Logfile);
#endif

    bAtHome = false;

    nErr = domeCommand("HOME ?\r", resp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return ERR_CMDFAILED;

    tmp = atoi(resp);
    if(tmp)
        bAtHome = true;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CRigelDome::isDomeAtHome] Is dome at home = %s\n", timestamp, bAtHome?"Yes":"No");
    fflush(Logfile);
#endif

    return nErr;
  
}

int CRigelDome::isDomeAtPark(bool &bAtPArk)
{
    int tmp;
    int nErr = RD_OK;
    char resp[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return NOT_CONNECTED;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 1
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CRigelDome::isDomeAtPark] Called.\n", timestamp);
    fflush(Logfile);
#endif

    bAtPArk = false;

    nErr = domeCommand("PARK ?\r", resp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return ERR_CMDFAILED;

    tmp = atoi(resp);
    if(tmp)
        bAtPArk = true;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CRigelDome::isDomeAtPark] Is dome at park = %s\n", timestamp, bAtPArk?"Yes":"No");
    fflush(Logfile);
#endif

    return nErr;
}


int CRigelDome::connectToShutter()
{
    int nErr = RD_OK;
    char resp[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return NOT_CONNECTED;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 1
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CRigelDome::connectToShutter] Called.\n", timestamp);
    fflush(Logfile);
#endif

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

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 1
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CRigelDome::isConnectedToShutter] Called.\n", timestamp);
    fflush(Logfile);
#endif

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

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 1
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CRigelDome::btForce] Called.\n", timestamp);
    fflush(Logfile);
#endif

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

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 1
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CRigelDome::syncDome] Called.\n", timestamp);
    fflush(Logfile);
#endif

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

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 1
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CRigelDome::parkDome] Called.\n", timestamp);
    fflush(Logfile);
#endif

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

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 1
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CRigelDome::unparkDome] Called.\n", timestamp);
    fflush(Logfile);
#endif

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

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 1
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CRigelDome::gotoAzimuth] Called.\n", timestamp);
    fflush(Logfile);
#endif

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CRigelDome::gotoAzimuth] GoTo %3.1f\n", timestamp, dNewAz);
    fflush(Logfile);
#endif

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

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 1
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CRigelDome::openShutter] Called.\n", timestamp);
    fflush(Logfile);
#endif

    if(m_bCalibrating)
        return SB_OK;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CRigelDome::openShutter] Opening Shutter\n", timestamp);
    fflush(Logfile);
#endif

    if(m_bDebugLog && m_pLogger) {
        char szEventLogMsg[256];
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        snprintf(szEventLogMsg, 256, "[%s] Opening Shutter", timestamp);
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

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 1
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CRigelDome::closeShutter] Called.\n", timestamp);
    fflush(Logfile);
#endif

    if(m_bCalibrating)
        return SB_OK;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CRigelDome::closeShutter] Closing Shutter\n", timestamp);
    fflush(Logfile);
#endif

    if(m_bDebugLog && m_pLogger) {
        char szEventLogMsg[256];
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        snprintf(szEventLogMsg, 256, "[%s] Closing Shutter", timestamp);
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

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 1
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CRigelDome::getFirmwareVersion] Called.\n", timestamp);
    fflush(Logfile);
#endif

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

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 1
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CRigelDome::getModel] Called.\n", timestamp);
    fflush(Logfile);
#endif

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

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 1
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CRigelDome::goHome] Called.\n", timestamp);
    fflush(Logfile);
#endif

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

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 1
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CRigelDome::calibrate] Called.\n", timestamp);
    fflush(Logfile);
#endif

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
    
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CRigelDome::calibrate] Calibrating\n", timestamp);
    fflush(Logfile);
#endif

    return nErr;
}

int CRigelDome::isGoToComplete(bool &bComplete)
{
    int nErr = 0;
    double dDomeAz = 0;
    bool bIsMoving = false;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 1
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CRigelDome::isGoToComplete] called\n", timestamp);
    fflush(Logfile);
#endif

    nErr = isDomeMoving(bIsMoving);
    if(nErr) {
        return nErr;
        }

    getDomeAz(dDomeAz);

    if(bIsMoving) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CRigelDome::isGoToComplete] Dome is moving, domeAz = %f, mGotoAz = %f\n", timestamp, ceil(dDomeAz), ceil(m_dGotoAz));
        fflush(Logfile);
#endif
        bComplete = false;
        return nErr;
    }

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CRigelDome::isGoToComplete] Dome is NOT moving, domeAz = %f, mGotoAz = %f\n", timestamp, ceil(dDomeAz), ceil(m_dGotoAz));
    fflush(Logfile);
#endif

    if ((floor(m_dGotoAz) <= floor(dDomeAz)+1) && (floor(m_dGotoAz) >= floor(dDomeAz)-1)) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
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
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
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

    if(!m_bIsConnected)
        return NOT_CONNECTED;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 1
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CRigelDome::isOpenComplete] called\n", timestamp);
    fflush(Logfile);
#endif


    nErr = getShutterState(m_nShutterState);
    if(nErr)
        return ERR_CMDFAILED;

    if(m_nShutterState == OPEN){
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CRigelDome::isOpenComplete] Shutter Opened\n", timestamp);
        fflush(Logfile);
#endif
        m_bShutterOpened = true;
        bComplete = true;
        m_dCurrentElPosition = 90.0;

        if(m_bDebugLog && m_pLogger) {
			if(m_nPreviousShutterState != m_nShutterState) {
				m_nPreviousShutterState = m_nShutterState;
				char szEventLogMsg[256];
				ltime = time(NULL);
				timestamp = asctime(localtime(&ltime));
				timestamp[strlen(timestamp) - 1] = 0;
				snprintf(szEventLogMsg, 256, "[%s] Shutter Opened", timestamp);
				m_pLogger->out(szEventLogMsg);
			}
        }
    }
    else {
        m_bShutterOpened = false;
        bComplete = false;
        m_dCurrentElPosition = 0.0;
    }

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CRigelDome::isOpenComplete] Open complete = %s\n", timestamp, bComplete?"Yes":"No");
    fflush(Logfile);
#endif

    return nErr;
}

int CRigelDome::isCloseComplete(bool &bComplete)
{
    int err=0;

	if(!m_bIsConnected)
        return NOT_CONNECTED;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 1
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CRigelDome::isCloseComplete] called\n", timestamp);
    fflush(Logfile);
#endif

    err = getShutterState(m_nShutterState);
    if(err)
        return ERR_CMDFAILED;

    if(m_nShutterState == CLOSED){
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CRigelDome::isCloseComplete] Shutter Closed\n", timestamp);
        fflush(Logfile);
#endif
        m_bShutterOpened = false;
        bComplete = true;
        m_dCurrentElPosition = 0.0;
        if(m_bDebugLog && m_pLogger) {
			if(m_nPreviousShutterState != m_nShutterState) {
				m_nPreviousShutterState = m_nShutterState;
				char szEventLogMsg[256];
				ltime = time(NULL);
				timestamp = asctime(localtime(&ltime));
				timestamp[strlen(timestamp) - 1] = 0;
				snprintf(szEventLogMsg, 256, "[%s] Shutter Closed", timestamp);
				m_pLogger->out(szEventLogMsg);
			}
        }
    }
    else {
        m_bShutterOpened = true;
        bComplete = false;
        m_dCurrentElPosition = 90.0;
    }

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CRigelDome::isCloseComplete] Close complete = %s\n", timestamp, bComplete?"Yes":"No");
    fflush(Logfile);
#endif

    return err;
}


int CRigelDome::isParkComplete(bool &bComplete)
{
    int nErr = 0;
    bool bIsMoving = false;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 1
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CRigelDome::isParkComplete] called\n", timestamp);
    fflush(Logfile);
#endif

    nErr = isDomeMoving(bIsMoving);
    if(nErr)
        return nErr;

    if(bIsMoving) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 1
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CRigelDome::isParkComplete] Dome still moving\n", timestamp);
        fflush(Logfile);
#endif
        bComplete = false;
        return nErr;
    }

    nErr = isDomeAtPark(bComplete);

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 1
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CRigelDome::isParkComplete] Park complete = %s\n", timestamp, bComplete?"Yes":"No");
    fflush(Logfile);
#endif

    return nErr;
}

int CRigelDome::isUnparkComplete(bool &bComplete)
{
    int nErr = 0;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 1
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CRigelDome::isUnparkComplete] called\n", timestamp);
    fflush(Logfile);
#endif

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

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 1
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CRigelDome::isFindHomeComplete] called\n", timestamp);
    fflush(Logfile);
#endif

    nErr = isDomeMoving(bIsMoving);
    if(nErr)
        return nErr;

    if(bIsMoving) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CRigelDome::isFindHomeComplete] Dome is still moving\n");
        fflush(Logfile);
#endif
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

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CRigelDome::isFindHomeComplete] Find home complete = %s\n", timestamp, bComplete?"Yes":"No");
    fflush(Logfile);
#endif

    return nErr;
}


int CRigelDome::isCalibratingComplete(bool &bComplete)
{
    int nErr = 0;

    bComplete = false;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 1
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CRigelDome::isCalibratingComplete] called\n", timestamp);
    fflush(Logfile);
#endif

    // new version of the test
    nErr = getExtendedState();
    if(nErr)
        return nErr;

    if(m_nMotorState == CALIBRATIG) {
        bComplete = false;
    }
    else if (m_nMotorState == IDLE){
        bComplete = true;
    }
    else {
        // probably still moving
        bComplete = false;
    }

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CRigelDome::isCalibratingComplete] Calibration complete = %s\n", timestamp, bComplete?"Yes":"No");
    fflush(Logfile);
#endif

     return nErr;
}


int CRigelDome::abortCurrentCommand()
{
    if(!m_bIsConnected)
        return NOT_CONNECTED;

    m_bCalibrating = false;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 1
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CRigelDome::abortCurrentCommand] Called.\n", timestamp);
    fflush(Logfile);
#endif

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

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 1
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CRigelDome::setHomeAz] Called.\n", timestamp);
    fflush(Logfile);
#endif

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

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 1
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CRigelDome::setParkAz] Called.\n", timestamp);
    fflush(Logfile);
#endif

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

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 1
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CRigelDome::getExtendedState] Called.\n", timestamp);
    fflush(Logfile);
#endif

    nErr = domeCommand("V\r", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;
    // szResp contains the 13 state fields.
    nErr = parseFields(szResp, vFields, '\t');
    if(vFields.size()>=13) {
        m_dCurrentAzPosition = atof(vFields[0].c_str());
        m_nMotorState = atoi(vFields[1].c_str());
		m_nPreviousShutterState = m_nShutterState;
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
