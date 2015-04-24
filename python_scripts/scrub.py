#from PyQt4 import QtGui
#from PyQt4 import QtCore
#from PyQt4 import Qt
import sys
def elfSize(FileName,section):
    bashCommand = "/usr/bin/size "+ FileName
    import subprocess
    process = subprocess.Popen(bashCommand.split(), stdout=subprocess.PIPE)
    result =   process.communicate()[0].split("\n")
    header = result[0].lstrip()
    line1 = result[1].lstrip()
    hcolumns=header.split()
    columns=line1.split()
    for i in range(0,len(hcolumns)):
        if(hcolumns[i].find(section) != -1):
            return int(columns[i])
    return 0;


def elfAddress(FileName,section):
    bashCommand = "readelf -S " + FileName 
    import subprocess
    process = subprocess.Popen(bashCommand.split(), stdout=subprocess.PIPE)
    result =   process.communicate()[0].split()
    for i in range(0,len(result)):
        if(result[i].find('.'+section) != -1):
            return int("0x"+result[i+2],16)
    return 0;

def compare(data1,data2):
    if len(data1)!=len(data2):
        return [-1,-1]
    for i in range(len(data1)):
        if data1[i]!=data2[i]:
            return [-1,i]
    return [1,-1]
    
def cycles(rootPlugin,textAddress,textSize,orgiData,count):
    for i in range(count):
        rootPlugin.dumpMemory(textAddress,textSize/4,"/opt/dump"+str(i)+".srec","srec")
        data = rootPlugin.Read(textAddress,textSize/4)
        result = compare(data,orgiData)
        if(result[0]==1):
            print("test number "+str(i)+" = success")
        else:
            print("test number "+str(i)+" = error @0x" + hex(textAddress + result[1]))


#app = QtGui.QApplication(sys.argv)
fileName = QtGui.QFileDialog.getOpenFileName()

if PySocExplorer.ElfFile.isElf(fileName):
    proxy.loadSysDriver("SpwPlugin","SpwPlugin0")
    SpwPlugin0.selectBridge("STAR-Dundee Spw USB Brick")

    proxy.loadSysDriverToParent("dsu3plugin","SpwPlugin0")
    proxy.loadSysDriverToParent("LFRControlPlugin","SpwPlugin0")
    SpwPlugin0.TCPServerConnect()
    LFRControlPlugin0.TCPServerConnect()
    proxy.loadSysDriverToParent("APB_UART_PLUGIN","SpwPlugin0")
    availableBrickCount = SpwPlugin0.StarDundeeGetAvailableBrickCount()
    print(str(availableBrickCount) + " SpaceWire brick(s) found")

    SpwPlugin0.StarDundeeSelectBrick(1)
    SpwPlugin0.StarDundeeSetBrickAsARouter(1)
    SpwPlugin0.connectBridge()
    APB_UART_PLUGIN0.setUARTPortNane("/dev/ttyUSB1")
    APB_UART_PLUGIN0.setUARTPortSpeed(38400)
    APB_UART_PLUGIN0.openUart()
    textSize= elfSize(fileName,"text")
    textAddress= elfAddress(fileName,"text")
    print "Found text section@" + hex(textAddress)+ " of " + str(textSize) +" bytes"
    print "loading software"
    dsu3plugin0.openFile(fileName)
    dsu3plugin0.loadFile()
    SpwPlugin0.dumpMemory(textAddress,textSize/4,"/opt/dumpOrig.srec","srec")
    dsu3plugin0.run()
    orgiData = SpwPlugin0.Read(textAddress,textSize/4)
    orgiData = SpwPlugin0.Read(textAddress,textSize/4)




