#!/usr/bin/lppmon -e 

import time

proxy.loadSysDriver("SpwPlugin","SpwPlugin0")
SpwPlugin0.selectBridge("STAR-Dundee Spw USB Brick")

proxy.loadSysDriverToParent("dsu3plugin","SpwPlugin0")
proxy.loadSysDriverToParent("LFRControlPlugin","SpwPlugin0")

proxy.loadSysDriverToParent("APB_UART_PLUGIN","SpwPlugin0")
APB_UART_PLUGIN0.setFifoDebugEnabled( 0 )

availableBrickCount = SpwPlugin0.StarDundeeGetAvailableBrickCount()
print "availableBrickCount = ", availableBrickCount

SpwPlugin0.StarDundeeSelectBrick(1)
SpwPlugin0.StarDundeeSetBrickAsARouter(1)
SpwPlugin0.connectBridge()

#SpwPlugin0.TCPServerSetIP("127.0.0.1")
SpwPlugin0.TCPServerConnect()

#LFRControlPlugin0.SetSpwServerIP(129,104,27,164)
LFRControlPlugin0.TCPServerConnect()

dsu3plugin0.openFile("/opt/DEV_PLE/EQM/bin/eqm")
dsu3plugin0.loadFile()
dsu3plugin0.run()

APB_UART_PLUGIN0.setFifoDebugEnabled( 1 )

LFRControlPlugin0.TMEchoBridgeOpenPort()

