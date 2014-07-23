#-- '''
#--   Tittle:....... Logger
#-- '''
from synapse.snapsys import *
from synapse.platforms import *
if platform != "SM700":
    compileError  # script valid only on SM700

from synapse.switchboard import *

#-------------------------------------------------------------------------------
#--
#-- Module pins
#--
#-------------------------------------------------------------------------------

# Debug Generation for Controller #20 with JTAG Connector
from OptiV2Io import *


#-------------------------------------------------------------------------------
#-- DBG aid
#-------------------------------------------------------------------------------
from hexSupport3 import *

#--from DsProtoMsg import *
#-- Declarations section
#--
#--SnaPy maximum string length
MAX_LEN = 62
#------------------------------------ Non-Volatile memory ----------------------

#--- Snap Node Config NV_Params
NETWORK_ID      = 3
ENCRYPT_ENABLE  = 50
ENCRYPT_KEY     = 51

#--- User Defined NV_Params
MASTER_ADDR   = 243

# SYS_FLAG: b0-Printf_Enable, b3-LensPower, b4:6-Mode
#          |  Mode2   |  Mode1   |  Mode0   |  LensPwr  |  xxx  |  xxx  |  Printf  |           
SYS_FLAG      = 244
INIT_OK       = 245

#--
DUMP_SEL    = 250
LAST_CRC    = 251

SNAP_CRC    =  40

#--
#-- UART #1 connected to pc
#--

#------------------------------------------------------------------------------#
#--                                STARTUP                                   --#
#------------------------------------------------------------------------------#
@setHook(HOOK_STARTUP)
def startupEvent():
    global OK_Flag, led1St, NodeAddr, State22, T_1msec
    global TimeM10, MasterAddr, lastEH_mq, WirlesGrnLed

    checkNVParams()

    setPinDir( POWER_BT, False)                  #--
    setPinPullup(POWER_BT, True)                 #--
    
    setPinDir( POWER_ON, True)                  #--
    writePin(POWER_ON, True)

    setPinDir( PWRRED_LED, True)                 #--
    writePin( PWRRED_LED, False)             #--
    setPinDir( PWRGRN_LED, True)                 #--
    writePin( PWRGRN_LED, True)
    
    setPinDir(BACKL_LED1, True)                  #-- 
    writePin(BACKL_LED1, False)                  
    setPinDir(BACKL_LED2, True)
    writePin(BACKL_LED2, False)  

    setPinDir( RANGE_BT, False)                  #--
    setPinPullup(RANGE_BT, True)                 #--
    
    setPinDir( RANGEYEL_LED, True)              # Range Yellow Led, Output  
    writePin( RANGEYEL_LED, False)                
    setPinDir( RANGEGRN_LED, True)              # Range Green Led, Output  
    writePin( RANGEGRN_LED, False)                
    
    setPinDir( INIT_BT, False)                   #--
    setPinPullup(INIT_BT, True)                  #--
    
    setPinDir( INITGRN_LED, True)                 #--
    writePin( INITGRN_LED, False)                
    
    setPinDir( INITYEL_LED, True)                 #--
    writePin( INITYEL_LED, False)                   #--

    setPinDir( WIRLES_BT, False)                 #-- WirelessOn Button-Input
    setPinPullup(WIRLES_BT, True)                #-- Pullup On
    
    setPinDir( WIRLESRED_LED, True)              #-- WirelessOn Led-Output
    writePin( WIRLESRED_LED, False)            
    setPinDir( WIRLESGRN_LED, True) 
    WirlesGrnLed = True
    writePin( WIRLESGRN_LED, WirlesGrnLed)            
    
    setPinDir( SETUP_F1_BT, False)                 #-- -Input
       
    initUart( 0, 1)                              #-- Birger interface   UART #0
    crossConnect(1, 4)                           #-- UART0 <--> STDIO
    stdinMode(0, False)                          # Line Mode, no Echo
                                                 #-- debug port
    initUart( 1, 1)                              #-- PC - interface     UART #1
    crossConnect(2, 0)                           #-- UART1 <--> null

    OK_Flag = chkAsmOK()

    State22 = False
    writePin(OUTPUT_22, State22)
    
    CurFocVal = 0
    LastFocVal = 0 # Leon
    
    rx(True)
    
    MasterAddr = loadNvParam(MASTER_ADDR)
    NodeAddr = localAddr()
    
    T_1msec = 0
    TimeM10 = 0
    lastEH_mq = 0
    lastEH_mq_1msec = 0 # Leon


def checkNVParams():
    crc = loadNvParam(SNAP_CRC)
    if ( crc != loadNvParam(LAST_CRC)):
        saveNvParam( LAST_CRC, crc)
        testAsmStrings()

    if ( loadNvParam(INIT_OK) != 123 ):
        saveNvParam( INIT_OK, 123 )
        saveNvParam( SYS_FLAG, 0 )

@setHook(HOOK_1MS)
def int_1ms():
    ''' every 1 ms '''
    global T_1msec

    T_1msec += 1
    

@setHook(HOOK_10MS) # Leon comment > might have to change the main part
def main():
    ''' every 10 ms '''
    global TimeM10

    TimeM10 += 1
    
    
@setHook(HOOK_1S)
def everySecond():
    global TimeM10, lastEH_mq, T_1msec

    toggleLed1()
    
    if ( dTFocVal > 1000 ): # Leon -> Changed from: TimeM10 > 3000
        TimeM10 = 0
        T_1msec = 0
        lastEH_mq = 0
        lastEH_mq_1msec = 0 # Leon
        LastFocVal = 0 # Leon
        
            
def showMiliSecQ():
    s = ' mQ=' + str(TimeM10) 
    return s    

                
##-------------------------------------------------------------------
#-- RECEIVER
#-- works in line mode
#-------------------------------------------------------------------
@setHook(HOOK_STDIN)
def receiver(line):
    ''' Process response line received from lenses '''
    global waitCntr, lineCnt

    if ( len(line) > 0):
        
        if ( line[0:3]=='XYZ' ):
            TTY_AckOK = 1
            mode = ord(line[3])&0x07
            return

        lineCnt += 1
        

def toggleLed1():
    global WirlesGrnLed
    
    if ( WirlesGrnLed == True ):
        writePin( WIRLESGRN_LED, False)
        WirlesGrnLed = False
    else:
        writePin( WIRLESGRN_LED, True)
        WirlesGrnLed = True
       
    return WirlesGrnLed    


 #------------------------------------------------------------------
 #--  Check if string(C\Asm Routine) starts at div by 4 boundary
 #--
def isCallStrOK( callStr):
    Length = len(callStr)

    addr_lo = 0x4a00 + 3
    while ( addr_lo < 0x5c00):
        tLen = peek( 0x0040, addr_lo, 0)
        if ( tLen == Length):
            CodeOK = True
            i = 0
            while ( i < tLen ):
                memByte = peek( 0x0040, addr_lo+i+1, 0 )
                if ( ord(callStr[i]) != memByte ):
                    CodeOK = False
                    break;
                i = i + 1
            if ( CodeOK == True ):
                return (addr_lo+1)
        addr_lo = addr_lo + 4
    return -1
    
#--
def checkDDump1():
    return isCallStrOK(DUMP2MB_BODYa)

def checkDDump2():
    return isCallStrOK(DUMP2MB_BODYb)


def testAsmStrings():
    #--
    if ( checkDDump1() > 0 ):
        saveNvParam(DUMP_SEL, 1)
    elif ( checkDDump2() > 0 ):
        saveNvParam(DUMP_SEL, 2)
    else:
        saveNvParam(DUMP_SEL, 0)

#--------------------------------------------------------------------
#-- ARM 7 Assembly language routines
#-- debug dump --
#-------------------------------------------------------------------
# Debug Serial Dump (Portal UART2-2MBaud) Asm ARM/Thumb Routine
DUMP2MB_BODYa = '\x00\x00\x0F\xE1\xC0\x20\x80\xE3\x02\xF0\x21\xE1\x15\x30\x83\xE2\x13\xFF\x2F\xE1\x01\xB4\xF2\xB4\x8F\x1E\x3D\x78\x7F\x78\x3F\x02\xED\x19\x40\x20\x00\x04\x2D\x18\x4F\x1F\x3B\x78\x7F\x78\x3F\x02\xDE\x19\x2A\x78\x01\x32\xAF\x18\x0D\x20\x38\x70\x0F\x4B\x10\x48\x18\x60\x10\x48\x98\x61\x14\x1C\x01\x32\x1E\x72\x18\x69\x00\x28\xFC\xD0\x68\x78\x18\x72\x01\x35\x01\x3C\xF7\xD1\xF2\xBC\x0C\x20\x0B\x1A\x00\x20\x98\x70\x5A\x70\x01\x20\x18\x70\x01\xBC\x7B\x46\x02\x33\x18\x47\x00\xF0\x21\xE1\x1E\xFF\x2F\xE1\x00\xB0\x00\x80\x03\x64\x00\x00\x0F\x27\xFF\x4FVVVV'
DUMP2MB_BODYb = '\x00\x00\x0F\xE1\xC0\x20\x80\xE3\x02\xF0\x21\xE1\x15\x30\x83\xE2\x13\xFF\x2F\xE1\x01\xB4\xF2\xB4\x8F\x1E\x3D\x78\x7F\x78\x3F\x02\xED\x19\x40\x20\x00\x04\x2D\x18\x4F\x1F\x3B\x78\x7F\x78\x3F\x02\xDE\x19\x2A\x78\x01\x32\xAF\x18\x0D\x20\x38\x70\x0F\x4B\x10\x48\x18\x60\x10\x48\x98\x61\x14\x1C\x01\x32\x1E\x72\x18\x69\x00\x28\xFC\xD0\x68\x78\x18\x72\x01\x35\x01\x3C\xF7\xD1\xF2\xBC\x0C\x20\x0B\x1A\x00\x20\x98\x70\x5A\x70\x01\x20\x18\x70\x01\xBC\x7B\x46\x02\x33\x18\x47\x00\xF0\x21\xE1\x1E\xFF\x2F\xE1\x00\xB0\x00\x80\x03\x64\x00\x00\x0F\x27\xFF\x4FZZZZ'


def exeDump(line):
    if ( loadNvParam(DUMP_SEL) == 1):
        return call(DUMP2MB_BODYa, line[:])
    return call(DUMP2MB_BODYb, line[:])


def xdbg(line):
    if ( loadNvParam(DUMP_SEL) == 1):
        return call(DUMP2MB_BODYa, 0xA4, line[:])
    return call(DUMP2MB_BODYb, 0xA4, line[:])
    

def chkAsmOK ():
    if ( loadNvParam( DUMP_SEL) == 0 ):
        return False
    return True


def showStatus():
    s = showMiliSecQ()
    xdbg(s)
    return s


def getEH(EHstr):
    global CurFocVal, IrStep, LastEhStr, lastEH_mq, lastSync, lastEH_mq_1msec, dT_1msec, dTFocVal # Leon > lastEH_mq_1msec, dTFocVal
    
    toggle_Out22()
    
    LastEhStr = EHstr[0:2]
    dTFocVal = CurFocVal-LastFocVal # Leon
    CurFocVal = str2int(EHstr[0], EHstr[1])
    LastFocVal = CurFocVal # Leon
    
    IrStep = ord(EHstr[2])
    SyncNo = ord(EHstr[3])
    
    if ( lastSync == None ):
        lastSync = (SyncNo - 1) & 0xff
    
    dT = TimeM10-lastEH_mq
    lastEH_mq = TimeM10
    
    dT_1msec = T_1msec-lastEH_mq_1msec # Leon
    lastEH_mq_1msec = T_1msec # Leon
    
    if ( dT>120 ):
        dtStr = ' ---- '
    else:    
        dtStr = ' ' + str(dT) + ' :' + str(T_1msec) + ' :' + str(dT_1msec) # Leon >  + ' :' + str(dT_1msec)
        
    if ( SyncNo > lastSync ):
        dSync = SyncNo - lastSync
    else:    
        dSync = SyncNo + ( 255 - lastSync) + 1
        
    lastSync = SyncNo    
        
    if ( dSync > 1 ):   
        syncStr = ' *' + str(dSync-1)
    else:    
        syncStr = '   ' 
    
    s = str(CurFocVal) + ' ' + str(IrStep) + ' ' + str(SyncNo) + syncStr + dtStr 
    
    #s = str(CurFocVal) + ' ' + str(IrStep) + ' ' + str(SyncNo) + dtStr 
    
    #s = str(CurFocVal) + dtStr + ' ' + str(SyncNo)
    #s = '-F:' + str(CurFocVal) + showMiliSecQ() + dtStr
    xdbg(s)
    

def toggle_Out22():
    global State22
    
    State22 = not (State22)   
    writePin(OUTPUT_22, State22)
    

def a_Go2Doba():
    saveNvParam( NETWORK_ID, 0xD0BA)
    saveNvParam( ENCRYPT_ENABLE, 1)
    saveNvParam( ENCRYPT_KEY, 'DS@Kaszana#Szma?')
    
    resetVm()
                                  
