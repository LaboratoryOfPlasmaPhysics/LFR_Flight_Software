
FSW="/opt/LFR/BENCHMARKS/matrix_bench"
elffile=PySocExplorer.ElfFile(FSW)

duration_address=elffile.getSymbolAddress(elffile.getSymbolIndex("duration"))
duration2_address=elffile.getSymbolAddress(elffile.getSymbolIndex("duration2"))
 
def loadSW(SW):
    dsu3plugin0.openFile(SW)
    dsu3plugin0.loadFile()
    dsu3plugin0.run()

def wait_for_bench(address):
	timeout=10000
	duration=0
	while duration == 0 and timeout > 0:
		duration=SpwPlugin0.Read(address,1)[0]
		timeout-=1
	return duration

proxy.loadSysDriver("SpwPlugin")
SpwPlugin0.selectBridge("STAR-Dundee Spw USB Brick")
proxy.loadSysDriverToParent("dsu3plugin","SpwPlugin0")
availableBrickCount = SpwPlugin0.StarDundeeGetAvailableBrickCount()
print( str(availableBrickCount) + " SpaceWire brick(s) found")
SpwPlugin0.StarDundeeSelectBrick(1)
SpwPlugin0.StarDundeeSetBrickAsARouter(1)
SpwPlugin0.StarDundeeSelectLinkNumber( 1 )
SpwPlugin0.connectBridge()
SpwPlugin0.TCPServerConnect()
loadSW(FSW)

duration=wait_for_bench(duration_address)
duration2=wait_for_bench(duration2_address)

print(duration*1./25.e6)
print(duration2*1./25.e6)
