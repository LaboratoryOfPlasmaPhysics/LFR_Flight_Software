#!/usr/bin/lppmon -e 
import math

address_to_read = 0x80000f08
val = RMAPPlugin0.Read( address_to_read, 1)
matrixF0_Address0 = val[0]
print hex(matrixF0_Address0)

teta = []
for i in range(128):
	teta.append( i * 2 * math.pi / 128 )
amplitude = 10000

# BUILD THE DATA
dataToWrite = []
for frequencyBin in range(128):
	for component in range (25):
		dataToWrite.append( amplitude * math.sin( teta[frequencyBin] * component ) )

# WRITE THE DATA
print len(dataToWrite)
RMAPPlugin0.Write( matrixF0_Address0, dataToWrite )
