# LOAD FSW USING LINK 1
SpwPlugin0.StarDundeeSelectLinkNumber( 1 )

dsu3plugin0.openFile("/home/pleroy/DEV/build-DEV_PLE-Desktop-Default/src/fsw")
#dsu3plugin0.openFile("/opt/LFR/LFR-FSW/2.0.2.3/fsw")
dsu3plugin0.loadFile()

dsu3plugin0.run()

# START SENDING TIMECODES AT 1 Hz
#SpwPlugin0.StarDundeeStartTimecodes( 1 )

# it is possible to change the time code frequency
#RMAPPlugin0.changeTimecodeFrequency(2)
