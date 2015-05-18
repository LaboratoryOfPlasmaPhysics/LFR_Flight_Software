import time

proxy.loadSysDriver("SpwPlugin","SpwPlugin0")
SpwPlugin0.selectBridge("STAR-Dundee Spw USB Brick")

proxy.loadSysDriverToParent("dsu3plugin","SpwPlugin0")
proxy.loadSysDriverToParent("LFRControlPlugin","SpwPlugin0")

availableBrickCount = SpwPlugin0.StarDundeeGetAvailableBrickCount()
print str(availableBrickCount) + " SpaceWire brick(s) found"

SpwPlugin0.StarDundeeSelectBrick(1)
SpwPlugin0.StarDundeeSetBrickAsARouter(1)
SpwPlugin0.connectBridge()

#SpwPlugin0.TCPServerSetIP("127.0.0.1")
SpwPlugin0.TCPServerConnect()

# OPEN SPACEWIRE SERVER
#LFRControlPlugin0.SetSpwServerIP(129,104,27,164)
LFRControlPlugin0.TCPServerConnect()

# OPEN TM ECHO BRIDGE SERVER
LFRControlPlugin0.TMEchoBridgeOpenPort()

# LOAD TIMEGEN USING  LINK 2
SpwPlugin0.StarDundeeSelectLinkNumber( 2 )
dsu3plugin0.openFile("/opt/LFR/TIMEGEN/0.0.0.1/timegen")
dsu3plugin0.loadFile()
dsu3plugin0.run()

# LOAD FSW USING LINK 1
SpwPlugin0.StarDundeeSelectLinkNumber( 1 )
dsu3plugin0.openFile("/opt/DEV_PLE/FSW-qt/bin/fsw")
dsu3plugin0.loadFile()
dsu3plugin0.run()

# START SENDING TIMECODES AT 1 Hz
SpwPlugin0.StarDundeeStartTimecodes( 1 )

# it is possible to change the time code frequency
#RMAPPlugin0.changeTimecodeFrequency(2)
