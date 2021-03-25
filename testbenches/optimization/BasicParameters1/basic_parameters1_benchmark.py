
FSW="/opt/LFR/BENCHMARKS/bp1_bench"
elffile=PySocExplorer.ElfFile(FSW)

compute_BP1_duration_address=elffile.getSymbolAddress(elffile.getSymbolIndex("compute_BP1_duration"))
BP1_set_duration_address=elffile.getSymbolAddress(elffile.getSymbolIndex("BP1_set_duration"))
 
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

compute_BP1_duration=wait_for_bench(compute_BP1_duration_address)
BP1_set_duration=wait_for_bench(BP1_set_duration_address)

print("compute_BP1_duration=" + str(compute_BP1_duration*1./25.e6))
print("BP1_set_duration=" + str(BP1_set_duration*1./25.e6))
