# -*- coding: utf-8 *-*

# setting up LPPMON plugins
from __main__ import proxy
if not(proxy.instanceExists("RMAPPlugin0")):
	proxy.loadSysDriver("RMAPPlugin", "RMAPPlugin0")
	proxy.loadSysDriverToParent("dsu3plugin", "RMAPPlugin0")

	dsu3plugin0.openFile("/opt/DEV_PLE/FSW-qt/bin/fsw")
	dsu3plugin0.loadFile()
	dsu3plugin0.run()

from __main__ import RMAPPlugin0

