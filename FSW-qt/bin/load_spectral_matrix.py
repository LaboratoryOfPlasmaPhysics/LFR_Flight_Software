#!/usr/bin/lppmon -e 

address_to_read = 0x80000f08
val = RMAPPlugin0.Read( address_to_read, 1)
matrixF0_Address0 = val[0]
print hex(matrixF0_Address0)

# BUILD THE DATA
dataToWrite = []
dataToWrite.append( 0x1234abcd )
dataToWrite.append( 0x4321dcba )

for component in range(25):
	for frequencyBin in range (64):
		dataToWrite.append( component + frequencyBin )
	for frequencyBin in range (64):
		dataToWrite.append( - (component + frequencyBin) )

# WRITE THE DATA
print len(dataToWrite)
RMAPPlugin0.Write( matrixF0_Address0, dataToWrite )
