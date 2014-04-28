with open('asm_f0_test_paul_1.txt', 'r') as f:
    listOfLines = []
    for line in f.readlines():
    	listOfLines.append(line)

data = listOfLines[10]	# line 9 contains the data
data =  data.split()	# get the data from the global string

dataInFloat = []
nbData = len(data)
print "nbData = " + str(nbData)
for i in range( nbData ):
	dataInFloat.append( float( data[i] ) )	# convert each string into a float

# reorganize the data to have a matrix in the VHDL format
# input format	(ICD format)
# matrix_0[0 .. 24] matrix_1[0 .. 24] .. matrix_127[0 .. 127]
# output format	(VHDL format)
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

print dataInIntReorganized
