#!/usr/bin/lppmon -e 

proxy.loadSysDriver("AHBUARTplugin","AHBUART");
proxy.loadSysDriverToParent("dsu3plugin","AHBUART");
proxy.loadSysDriverToParent("APB UART PLUGIN","AHBUART");
AHBUART.open("/dev/ttyUSB0",30000000)
dsu3plugin0.openFile("/opt/DEV_PLE/FSW-qt/bin/fsw")
dsu3plugin0.loadFile()
dsu3plugin0.run()
