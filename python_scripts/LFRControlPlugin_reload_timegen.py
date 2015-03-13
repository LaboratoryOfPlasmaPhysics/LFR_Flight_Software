# LOAD FSW USING LINK 1
SpwPlugin0.StarDundeeSelectLinkNumber( 2 )

dsu3plugin0.openFile("/opt/DEV_PLE/timegen-qt/bin/timegen")
dsu3plugin0.loadFile()

dsu3plugin0.run()

# START SENDING TIMECODES AT 1 Hz
SpwPlugin0.StarDundeeStartTimecodes( 1 )

SpwPlugin0.StarDundeeSelectLinkNumber( 1 )
