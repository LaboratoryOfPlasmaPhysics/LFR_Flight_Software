#!/usr/bin/lppmon -e 

RMAPPlugin0.setValueSpaceWireLinkNumber( 2 )
RMAPPlugin0.setValueTargetLogicalAddress( 254 )

dsu3plugin0.openFile("/opt/DEV_PLE/timegen-qt/bin/timegen")
dsu3plugin0.loadFile()
dsu3plugin0.run()

RMAPPlugin0.setValueSpaceWireLinkNumber( 1 )
