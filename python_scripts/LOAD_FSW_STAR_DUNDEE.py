#!/usr/bin/lppmon -e 

proxy.loadSysDriver("RMAPPlugin","RMAPPlugin0");
proxy.loadSysDriverToParent("dsu3plugin","RMAPPlugin0");

BUTTON_selectStarDundee.click()
#BUTTON_selectGRESB.click()
BUTTON_rmapOpenCommunication.click()

dsu3plugin0.openFile("/opt/DEV_PLE/FSW-qt/bin/fsw")
dsu3plugin0.loadFile()
dsu3plugin0.run()

