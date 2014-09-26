#!/usr/bin/lppmon -e 

# LOAD TIMEGEN USING  LINK 2
RMAPPlugin0.setValueSpaceWireLinkNumber( 2 )
RMAPPlugin0.setValueTargetLogicalAddress( 254 )

dsu3plugin0.openFile("/opt/DEV_PLE/timegen-qt/bin/timegen")
dsu3plugin0.loadFile()
dsu3plugin0.run()

# LOAD FSW USING LINK 1
RMAPPlugin0.setValueSpaceWireLinkNumber( 1 )
RMAPPlugin0.setValueTargetLogicalAddress( 254 )

dsu3plugin0.openFile("/opt/DEV_PLE/FSW-qt/bin/fsw")
dsu3plugin0.loadFile()
dsu3plugin0.run()

# START SENDING TIMECODES AT 1 Hz
RMAPPlugin0.sig_sendTimecodePeriodically( 1 )

# it is possible to change the time code frequency
#RMAPPlugin0.changeTimecodeFrequency(2)
