dbLoadDatabase("../dbd/ETH32AsynPortDriver.dbd")
ETH32AsynPortDriver_registerRecordDeviceDriver(pdbbase)

# Turn on asynTraceFlow and asynTraceError for global trace, i.e. no connected asynUser.
#asynSetTraceMask("", 0, 17)

ETH32AsynPortDriverConfigure("testETH32")

dbLoadRecords("../db/ETH32AsynPortDriver.db","P=testETH32:,R=motors:,PORT=testETH32,ADDR=0,TIMEOUT=1")
dbLoadRecords("../db/asynRecord.db","P=testETH32:,R=asyn1,PORT=testETH32,ADDR=0,OMAX=80,IMAX=80")
#asynSetTraceMask("testAPD",0,0xff)
asynSetTraceIOMask("testETH32",0,0x2)
iocInit()

## Run this to get something
## caget testETH32:motors:M1Pos_RBV
## caput testETH32:motors:Run 1
## caget testETH32:motors:M1Pos_RBV
## caput testETH32:motors:M1For 1
## caget testETH32:motors:M1Pos_RBV
## caput testETH32:motors:M1For 0
## caget testETH32:motors:M1Pos_RBV
