#!/usr/bin/lppmon -e 

RMAPPlugin0.setValueSpaceWireLinkNumber( 1 )
RMAPPlugin0.setValueTargetLogicalAddress( 254 )

dsu3plugin0.openFile("/opt/DEV_PLE/FSW-qt/bin/fsw")
dsu3plugin0.loadFile()
dsu3plugin0.run()

