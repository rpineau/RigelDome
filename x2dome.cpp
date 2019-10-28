#include <stdio.h>
#include <string.h>
#include "x2dome.h"
#include "../../licensedinterfaces/sberrorx.h"
#include "../../licensedinterfaces/basicstringinterface.h"
#include "../../licensedinterfaces/serxinterface.h"
#include "../../licensedinterfaces/basiciniutilinterface.h"
#include "../../licensedinterfaces/theskyxfacadefordriversinterface.h"
#include "../../licensedinterfaces/sleeperinterface.h"
#include "../../licensedinterfaces/loggerinterface.h"
#include "../../licensedinterfaces/basiciniutilinterface.h"
#include "../../licensedinterfaces/mutexinterface.h"
#include "../../licensedinterfaces/tickcountinterface.h"
#include "../../licensedinterfaces/serialportparams2interface.h"


X2Dome::X2Dome(const char* pszSelection, 
							 const int& nISIndex,
					SerXInterface*						pSerX,
					TheSkyXFacadeForDriversInterface*	pTheSkyXForMounts,
					SleeperInterface*					pSleeper,
					BasicIniUtilInterface*			pIniUtil,
					LoggerInterface*					pLogger,
					MutexInterface*						pIOMutex,
					TickCountInterface*					pTickCount)
{

    m_nPrivateISIndex				= nISIndex;
	m_pSerX							= pSerX;
	m_pTheSkyXForMounts				= pTheSkyXForMounts;
	m_pSleeper						= pSleeper;
	m_pIniUtil						= pIniUtil;
	m_pLogger						= pLogger;	
	m_pIOMutex						= pIOMutex;
	m_pTickCount					= pTickCount;

	m_bLinked = false;
    m_bCalibratingDome = false;
    m_bBattRequest = 0;
    
    m_RigelDome.SetSerxPointer(pSerX);
    m_RigelDome.setLogger(pLogger);

    if (m_pIniUtil) {
        m_bShutterEventLog = m_pIniUtil->readInt(PARENT_KEY, CHILD_KEY_LOG_EVENT, 0);
        m_RigelDome.setDebugLog( m_bShutterEventLog );
        m_RigelDome.setHomeAz( m_pIniUtil->readDouble(PARENT_KEY, CHILD_KEY_HOME_AZ, 180) );
        m_RigelDome.setParkAz( m_pIniUtil->readDouble(PARENT_KEY, CHILD_KEY_PARK_AZ, 180) );
    }
}


X2Dome::~X2Dome()
{
	if (m_pSerX)
		delete m_pSerX;
	if (m_pTheSkyXForMounts)
		delete m_pTheSkyXForMounts;
	if (m_pSleeper)
		delete m_pSleeper;
	if (m_pIniUtil)
		delete m_pIniUtil;
	if (m_pLogger)
		delete m_pLogger;
	if (m_pIOMutex)
		delete m_pIOMutex;
	if (m_pTickCount)
		delete m_pTickCount;

}


int X2Dome::establishLink(void)					
{
    int nErr;
    char szPort[DRIVER_MAX_STRING];

    X2MutexLocker ml(GetMutex());
    // get serial port device name
    portNameOnToCharPtr(szPort,DRIVER_MAX_STRING);
    nErr = m_RigelDome.Connect(szPort);
    if(nErr)
        m_bLinked = false;
    else
        m_bLinked = true;

    //m_bHasShutterControl = m_RigelDome.hasShutterUnit();
    // temp fix
    m_bHasShutterControl = true;
	return nErr;
}

int X2Dome::terminateLink(void)					
{
    X2MutexLocker ml(GetMutex());
    m_RigelDome.Disconnect();
	m_bLinked = false;
	return SB_OK;
}

 bool X2Dome::isLinked(void) const				
{
	return m_bLinked;
}


int X2Dome::queryAbstraction(const char* pszName, void** ppVal)
{
    *ppVal = NULL;

    if (!strcmp(pszName, LoggerInterface_Name))
        *ppVal = GetLogger();
    else if (!strcmp(pszName, ModalSettingsDialogInterface_Name))
        *ppVal = dynamic_cast<ModalSettingsDialogInterface*>(this);
    else if (!strcmp(pszName, X2GUIEventInterface_Name))
        *ppVal = dynamic_cast<X2GUIEventInterface*>(this);
    else if (!strcmp(pszName, SerialPortParams2Interface_Name))
        *ppVal = dynamic_cast<SerialPortParams2Interface*>(this);
    
    return SB_OK;
}

#pragma mark - UI binding

int X2Dome::execModalSettingsDialog()
{
    int nErr = SB_OK;
    X2ModalUIUtil uiutil(this, GetTheSkyXFacadeForDrivers());
    X2GUIInterface*					ui = uiutil.X2UI();
    X2GUIExchangeInterface*			dx = NULL;//Comes after ui is loaded
    bool bPressedOK = false;
    char szTmpBuf[SERIAL_BUFFER_SIZE];
    double dHomeAz;
    double dParkAz;
    int nShutterBatteryPercent;
    double dShutterBattery;

    if (NULL == ui)
        return ERR_POINTER;

    if ((nErr = ui->loadUserInterface("RigelDome.ui", deviceType(), m_nPrivateISIndex)))
        return nErr;

    if (NULL == (dx = uiutil.X2DX()))
        return ERR_POINTER;


    memset(szTmpBuf,0,SERIAL_BUFFER_SIZE);
    if(m_bHasShutterControl)
    {
        dx->setChecked("hasShutterCtrl",true);
    }
    else
    {
        dx->setChecked("hasShutterCtrl",false);
    }

	if(m_bShutterEventLog)
		dx->setChecked("enableEventLog",true);
	else
		dx->setChecked("enableEventLog",false);

	// set controls state depending on the connection state
    if(m_bLinked) {
        snprintf(szTmpBuf,16,"%d",m_RigelDome.getNbTicksPerRev());
        dx->setPropertyString("ticksPerRev","text", szTmpBuf);
        if(m_bHasShutterControl) {
            m_RigelDome.getBatteryLevels(dShutterBattery, nShutterBatteryPercent);
            snprintf(szTmpBuf,16,"%d%% ( %3.2f V )",nShutterBatteryPercent, dShutterBattery);
            dx->setPropertyString("shutterBatteryLevel","text", szTmpBuf);
        }
        else {
            snprintf(szTmpBuf,16,"NA");
            dx->setPropertyString("shutterBatteryLevel","text", szTmpBuf);
        }

        dx->setEnabled("pushButton",true);
    }
    else {
        snprintf(szTmpBuf,16,"NA");
        dx->setPropertyString("ticksPerRev","text", szTmpBuf);
        dx->setPropertyString("shutterBatteryLevel","text", szTmpBuf);
        dx->setEnabled("pushButton",false);
        dx->setEnabled("pushButton_2",false);
    }
    dx->setPropertyDouble("homePosition","value", m_RigelDome.getHomeAz());
    dx->setPropertyDouble("parkPosition","value", m_RigelDome.getParkAz());

    m_bBattRequest = 0;
    m_bCalibratingDome = false;
    
    X2MutexLocker ml(GetMutex());

    //Display the user interface
    if ((nErr = ui->exec(bPressedOK)))
        return nErr;

    //Retreive values from the user interface
    if (bPressedOK)
    {
        dx->propertyDouble("homePosition", "value", dHomeAz);
        dx->propertyDouble("parkPosition", "value", dParkAz);
        m_bShutterEventLog = dx->isChecked("enableEventLog");
        m_RigelDome.setDebugLog(m_bShutterEventLog);
        if(m_bLinked)
        {
            m_RigelDome.setHomeAz(dHomeAz);
            m_RigelDome.setParkAz(dParkAz);
        }

        // save the values to persistent storage
        nErr |= m_pIniUtil->writeInt(PARENT_KEY, CHILD_KEY_LOG_EVENT, m_bShutterEventLog);
        nErr |= m_pIniUtil->writeDouble(PARENT_KEY, CHILD_KEY_HOME_AZ, dHomeAz);
        nErr |= m_pIniUtil->writeDouble(PARENT_KEY, CHILD_KEY_PARK_AZ, dParkAz);
    }
    return nErr;

}

void X2Dome::uiEvent(X2GUIExchangeInterface* uiex, const char* pszEvent)
{
    bool bComplete = false;
    int nErr;
    int nShutterBatteryPercent;
    double dShutterBattery;
    char szTmpBuf[SERIAL_BUFFER_SIZE];
    char szErrorMessage[LOG_BUFFER_SIZE];
    
    if (!strcmp(pszEvent, "on_pushButtonCancel_clicked"))
        m_RigelDome.abortCurrentCommand();

    if (!strcmp(pszEvent, "on_timer"))
    {
        if(m_bLinked) {
            if(m_bCalibratingDome) {
                // are we still calibrating ?
                bComplete = false;
                nErr = m_RigelDome.isCalibratingComplete(bComplete);
                if(nErr) {
                    uiex->setEnabled("pushButton",true);
                    uiex->setEnabled("pushButtonOK",true);
                    snprintf(szErrorMessage, LOG_BUFFER_SIZE, "Error calibrating dome : Error %d", nErr);
                    uiex->messageBox("rigelDome Calibrate", szErrorMessage);
                    m_bCalibratingDome = false;
                    return;;
                }
                
                if(!bComplete) {
                    return;
                }
                
                // enable "ok" and "calibrate"
                uiex->setEnabled("pushButton",true);
                uiex->setEnabled("pushButtonOK",true);
                // read step per rev from dome
                snprintf(szTmpBuf,16,"%d",m_RigelDome.getNbTicksPerRev());
                uiex->setPropertyString("ticksPerRev","text", szTmpBuf);
                m_bCalibratingDome = false;
                
            }
            
            if(m_bHasShutterControl && !m_bCalibratingDome) {
                // don't ask to often
                if (!(m_bBattRequest%4)) {
                    if(m_bHasShutterControl) {
                        m_RigelDome.getBatteryLevels(dShutterBattery, nShutterBatteryPercent);
                        snprintf(szTmpBuf,16,"%d%% ( %3.2f V )",nShutterBatteryPercent, dShutterBattery);
                        uiex->setPropertyString("shutterBatteryLevel","text", szTmpBuf);
                    }
                    else {
                        snprintf(szTmpBuf,16,"NA");
                        uiex->setPropertyString("shutterBatteryLevel","text", szTmpBuf);
                    }
                }
                m_bBattRequest++;
            }

            
        }
    }

    if (!strcmp(pszEvent, "on_pushButton_clicked"))
    {
        if(m_bLinked) {
            // disable "ok" and "calibrate"
            uiex->setEnabled("pushButton",false);
            uiex->setEnabled("pushButtonOK",false);
            m_RigelDome.calibrate();
            m_bCalibratingDome = true;
        }
    }

    if (!strcmp(pszEvent, "on_pushButton_2_clicked"))
    {
        if(m_bLinked) {
            m_RigelDome.btForce();
        }
    }
}

//
//HardwareInfoInterface
//
#pragma mark - HardwareInfoInterface

void X2Dome::deviceInfoNameShort(BasicStringInterface& str) const					
{
	str = "RigelDome";
}

void X2Dome::deviceInfoNameLong(BasicStringInterface& str) const					
{
    str = "Rigel rotation drive unit";
}

void X2Dome::deviceInfoDetailedDescription(BasicStringInterface& str) const		
{
    str = "Rigel rotation drive unit for Pulsar Dome";
}

 void X2Dome::deviceInfoFirmwareVersion(BasicStringInterface& str)					
{
    if(m_bLinked) {
        char cFirmware[SERIAL_BUFFER_SIZE];
        m_RigelDome.getFirmwareVersion(cFirmware, SERIAL_BUFFER_SIZE);
        str = cFirmware;

    }
    else
        str = "N/A";
}

void X2Dome::deviceInfoModel(BasicStringInterface& str)
{
    if(m_bLinked) {
        char cModel[SERIAL_BUFFER_SIZE];
        m_RigelDome.getModel(cModel, SERIAL_BUFFER_SIZE);
        str = cModel;

    }
    else
        str = "N/A";
}

//
//DriverInfoInterface
//
#pragma mark - DriverInfoInterface

 void	X2Dome::driverInfoDetailedInfo(BasicStringInterface& str) const	
{
    str = "Rigel rotation drive unit X2 plugin by Rodolphe Pineau";
}

double	X2Dome::driverInfoVersion(void) const
{
	return DRIVER_VERSION;
}

//
//DomeDriverInterface
//
#pragma mark - DomeDriverInterface

int X2Dome::dapiGetAzEl(double* pdAz, double* pdEl)
{
    X2MutexLocker ml(GetMutex());

    if(!m_bLinked)
        return ERR_NOLINK;

    *pdAz = m_RigelDome.getCurrentAz();
    *pdEl = m_RigelDome.getCurrentEl();
    return SB_OK;
}

int X2Dome::dapiGotoAzEl(double dAz, double dEl)
{
    int nErr;

    X2MutexLocker ml(GetMutex());

    if(!m_bLinked)
        return ERR_NOLINK;

    nErr = m_RigelDome.gotoAzimuth(dAz);
    if(nErr)
        return ERR_CMDFAILED;

    else
        return SB_OK;
}

int X2Dome::dapiAbort(void)
{

    X2MutexLocker ml(GetMutex());

    if(!m_bLinked)
        return ERR_NOLINK;

    m_RigelDome.abortCurrentCommand();

    return SB_OK;
}

int X2Dome::dapiOpen(void)
{
    int nErr;
    X2MutexLocker ml(GetMutex());

    if(!m_bLinked)
        return ERR_NOLINK;

    if(!m_bHasShutterControl)
        return SB_OK;

    nErr = m_RigelDome.openShutter();
    if(nErr)
        return ERR_CMDFAILED;

	return SB_OK;
}

int X2Dome::dapiClose(void)
{
    int nErr;
    X2MutexLocker ml(GetMutex());

    if(!m_bLinked)
        return ERR_NOLINK;

    if(!m_bHasShutterControl)
        return SB_OK;

    nErr = m_RigelDome.closeShutter();
    if(nErr)
        return ERR_CMDFAILED;

	return SB_OK;
}

int X2Dome::dapiPark(void)
{
    int nErr;
    X2MutexLocker ml(GetMutex());

    if(!m_bLinked)
        return ERR_NOLINK;

    if(m_bHasShutterControl)
    {
        nErr = m_RigelDome.closeShutter();
        if(nErr)
            return ERR_CMDFAILED;
    }

    nErr = m_RigelDome.parkDome();
    if(nErr)
        return ERR_CMDFAILED;

	return SB_OK;
}

int X2Dome::dapiUnpark(void)
{
    int nErr;
    X2MutexLocker ml(GetMutex());

    if(!m_bLinked)
        return ERR_NOLINK;

    if(m_bHasShutterControl)
    {
        nErr = m_RigelDome.openShutter();
        if(nErr)
            return ERR_CMDFAILED;
    }

    nErr = m_RigelDome.unparkDome();
    if(nErr)
        return ERR_CMDFAILED;

	return SB_OK;
}

int X2Dome::dapiFindHome(void)
{
    int nErr;
    X2MutexLocker ml(GetMutex());

    if(!m_bLinked)
        return ERR_NOLINK;

    nErr = m_RigelDome.goHome();
    if(nErr)
        return ERR_CMDFAILED;

    return SB_OK;
}

int X2Dome::dapiIsGotoComplete(bool* pbComplete)
{
    int nErr;
    X2MutexLocker ml(GetMutex());

    if(!m_bLinked)
        return ERR_NOLINK;

    nErr = m_RigelDome.isGoToComplete(*pbComplete);
    if(nErr)
        return ERR_CMDFAILED;
    return SB_OK;
}

int X2Dome::dapiIsOpenComplete(bool* pbComplete)
{
    int nErr;
    X2MutexLocker ml(GetMutex());

    if(!m_bLinked)
        return ERR_NOLINK;
    
    if(!m_bHasShutterControl)
    {
        *pbComplete = true;
        return SB_OK;
    }

    nErr = m_RigelDome.isOpenComplete(*pbComplete);
    if(nErr)
        return ERR_CMDFAILED;

    return SB_OK;
}

int	X2Dome::dapiIsCloseComplete(bool* pbComplete)
{
    int nErr;
    X2MutexLocker ml(GetMutex());

    if(!m_bLinked)
        return ERR_NOLINK;

    if(!m_bHasShutterControl)
    {
        *pbComplete = true;
        return SB_OK;
    }

    nErr = m_RigelDome.isCloseComplete(*pbComplete);
    if(nErr)
        return ERR_CMDFAILED;

    return SB_OK;
}

int X2Dome::dapiIsParkComplete(bool* pbComplete)
{
    int nErr;
    X2MutexLocker ml(GetMutex());

    if(!m_bLinked)
        return ERR_NOLINK;

    nErr = m_RigelDome.isParkComplete(*pbComplete);
    if(nErr)
        return ERR_CMDFAILED;

    return SB_OK;
}

int X2Dome::dapiIsUnparkComplete(bool* pbComplete)
{
    int nErr;
    X2MutexLocker ml(GetMutex());

    if(!m_bLinked)
        return ERR_NOLINK;

    nErr = m_RigelDome.isUnparkComplete(*pbComplete);
    if(nErr)
        return ERR_CMDFAILED;

    return SB_OK;
}

int X2Dome::dapiIsFindHomeComplete(bool* pbComplete)
{
    int nErr;
    X2MutexLocker ml(GetMutex());

    if(!m_bLinked)
        return ERR_NOLINK;

    nErr = m_RigelDome.isFindHomeComplete(*pbComplete);
    if(nErr)
        return ERR_CMDFAILED;

    return SB_OK;
}

int X2Dome::dapiSync(double dAz, double dEl)
{
    int nErr;

    X2MutexLocker ml(GetMutex());

    if(!m_bLinked)
        return ERR_NOLINK;

    nErr = m_RigelDome.syncDome(dAz, dEl);
    if(nErr)
        return ERR_CMDFAILED;
	return SB_OK;
}

//
// SerialPortParams2Interface
//
#pragma mark - SerialPortParams2Interface

void X2Dome::portName(BasicStringInterface& str) const
{
    char szPortName[DRIVER_MAX_STRING];

    portNameOnToCharPtr(szPortName, DRIVER_MAX_STRING);

    str = szPortName;

}

void X2Dome::setPortName(const char* pszPort)
{
    if (m_pIniUtil)
        m_pIniUtil->writeString(PARENT_KEY, CHILD_KEY_PORTNAME, pszPort);
    
}


void X2Dome::portNameOnToCharPtr(char* pszPort, const int& nMaxSize) const
{
    if (NULL == pszPort)
        return;

    snprintf(pszPort, nMaxSize,DEF_PORT_NAME);

    if (m_pIniUtil)
        m_pIniUtil->readString(PARENT_KEY, CHILD_KEY_PORTNAME, pszPort, pszPort, nMaxSize);
    
}



