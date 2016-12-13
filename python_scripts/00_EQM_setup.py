import time

proxy.loadSysDriver("SpwPlugin","SpwPlugin0")
SpwPlugin0.selectBridge("STAR-Dundee Spw USB Brick")

proxy.loadSysDriverToParent("dsu3plugin","SpwPlugin0")
proxy.loadSysDriverToParent("LFRControlPlugin","SpwPlugin0")

availableBrickCount = SpwPlugin0.StarDundeeGetAvailableBrickCount()
print str(availableBrickCount) + " SpaceWire brick(s) found"

SpwPlugin0.StarDundeeSelectBrick(1)
SpwPlugin0.StarDundeeSetBrickAsARouter(1)
SpwPlugin0.StarDundeeSelectLinkNumber( 1 )
SpwPlugin0.connectBridge()

#SpwPlugin0.TCPServerSetIP("127.0.0.1")
SpwPlugin0.TCPServerConnect()

# OPEN SPACEWIRE SERVER
#LFRControlPlugin0.SetSpwServerIP(129,104,27,164)
LFRControlPlugin0.TCPServerConnect()

# OPEN TM ECHO BRIDGE SERVER
LFRControlPlugin0.TMEchoBridgeOpenPort()

# START SENDING TIMECODES AT 1 Hz
SpwPlugin0.StarDundeeStartTimecodes( 1 )

# it is possible to change the time code frequency
#RMAPPlugin0.changeTimecodeFrequency(2)
