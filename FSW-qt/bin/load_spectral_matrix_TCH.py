#!/usr/bin/lppmon -e 

####################
## BUILD THE DATA ##
####################

with open('/opt/DEV_PLE/FSW-qt/bin/spectralmatrix/asm_f0_test_20140403_case2.txt', 'r') as f:
    listOfLines = []
    for line in f.readlines():
    	listOfLines.append(line)

data = listOfLines[10]	# line 10 contains the data
data =  data.split()	# get the data from the global string

dataInFloat = []
nbData = len(data)
print "nbData = " + str(nbData)
for i in range( nbData ):
	dataInFloat.append( float( data[i] ) )	# convert each string into a float

# reorganize the data to have a matrix in the VHDL format
# INPUT		(ICD format)
# matrix_0[0 .. 24] matrix_1[0 .. 24] .. matrix_127[0 .. 24]
# OUTPUT	(VHDL format)
# component_0[0 .. 127] component_1[0 .. 127] .. component_24[0 .. 127]

dataInFloatReorganized = []
dataInIntReorganized = []
nbComponentsByMatrix  = 25
nbFrequencyBins = 128
for indexComponent in range(nbComponentsByMatrix):
	for frequencyBin in range(nbFrequencyBins):
		dataInFloatReorganized.append( 
			dataInFloat[ indexComponent + frequencyBin * nbComponentsByMatrix ] 
			)
		dataInIntReorganized.append( 
			int( dataInFloat[ indexComponent + frequencyBin * nbComponentsByMatrix ] ) 
			)

####################
## WRITE THE DATA ##
####################

# F0 buffer address
address_to_read = 0x80000f08
val = RMAPPlugin0.Read( address_to_read, 1)
matrixF0_Address0 = val[0]

# F1 buffer address
address_to_read = 0x80000f10
val = RMAPPlugin0.Read( address_to_read, 1)
matrixF1_Address = val[0]

# F2 buffer address
address_to_read = 0x80000f14
val = RMAPPlugin0.Read( address_to_read, 1)
matrixF2_Address = val[0]

print str( len(dataInIntReorganized) ) + " data to write"
RMAPPlugin0.Write( matrixF0_Address0, dataInIntReorganized )
print str( len(dataInIntReorganized) ) + " data written @" + hex(matrixF0_Address0)
RMAPPlugin0.Write( matrixF1_Address, dataInIntReorganized )
print str( len(dataInIntReorganized) ) + " data written @" + hex(matrixF1_Address)
RMAPPlugin0.Write( matrixF2_Address, dataInIntReorganized )
print str( len(dataInIntReorganized) ) + " data written @" + hex(matrixF2_Address)







