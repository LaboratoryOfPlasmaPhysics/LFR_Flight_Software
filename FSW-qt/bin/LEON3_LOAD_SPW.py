#!/usr/bin/lppmon -e 

proxy.loadSysDriver("RMAPPlugin","RMAPplugin0");
proxy.loadSysDriverToParent("dsu3plugin","RMAPplugin0");

#BUTTON_selectStarDundee.click()
BUTTON_selectGRESB.click()

BUTTON_rmapOpenCommunication.click()
dsu3plugin0.openFile("/opt/DEV_PLE/FSW-qt/bin/fsw-vhdl-dev")
dsu3plugin0.loadFile()
dsu3plugin0.run()

