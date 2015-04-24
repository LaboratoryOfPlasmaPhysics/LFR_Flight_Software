# LOAD FSW USING LINK 1
SpwPlugin0.StarDundeeSelectLinkNumber( 1 )

APB_UART_PLUGIN0.setFifoDebugEnabled( 0 )

dsu3plugin0.openFile("/opt/DEV_PLE/EQM/bin/eqm")
dsu3plugin0.loadFile()

dsu3plugin0.run()

APB_UART_PLUGIN0.setFifoDebugEnabled( 1 )

# START SENDING TIMECODES AT 1 Hz
SpwPlugin0.StarDundeeStartTimecodes( 1 )

# it is possible to change the time code frequency
#RMAPPlugin0.changeTimecodeFrequency(2)
