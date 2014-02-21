#!/usr/bin/lppmon -e 

address_to_read = 0x80000f08
val = RMAPPlugin0.Read( address_to_read, 1)
matrixF0_Address0 = val[0]
print hex(matrixF0_Address0)

# BUILD THE DATA
dataToWrite = []
for frequencyBin in range(128):
	for component in range (25):
		dataToWrite.append( component )

#for frequencyBin in range(64):
#	for component in range (25):
#		dataToWrite.append( 2 * component )

# WRITE THE DATA
print len(dataToWrite)
RMAPPlugin0.Write( matrixF0_Address0, dataToWrite )
