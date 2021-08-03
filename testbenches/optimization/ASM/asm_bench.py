import time
from datetime import timedelta

FSW="/opt/LFR/BENCHMARKS/asm_bench"
elffile=PySocExplorer.ElfFile(FSW)

duration_symbols = [ elffile.getSymbolName(i) for i in range(elffile.getSymbolCount()) if '_duration' in elffile.getSymbolName(i) ]
 
def loadSW(SW):
    dsu3plugin0.openFile(SW)
    dsu3plugin0.loadFile()
    dsu3plugin0.run()

def wait_for_bench():
        done_address=elffile.getSymbolAddress(elffile.getSymbolIndex("done"))
        timeout=1000
        done=0
        while done == 0 and timeout > 0:
                done=SpwPlugin0.Read(done_address,1)[0]
                time.sleep(.1)
                timeout-=1

def print_duration(symbol):
    address = elffile.getSymbolAddress(elffile.getSymbolIndex(symbol))
    cycles = SpwPlugin0.Read(address,1)[0]
    duration = float(SpwPlugin0.Read(address,1)[0])/25.e6
    td=timedelta(seconds=float(SpwPlugin0.Read(address,1)[0])/25.e6)
    print(symbol + " = " + str(td) + "s or "+ str(duration*1e6) + "us\t (or " + str(cycles) + " CPU cycles)")

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

time.sleep(5.)
wait_for_bench()

for symbol in duration_symbols:
    print_duration(str(symbol))

