import os

# find the opensim environment on your computer
Paths = os.getenv('Path')

# location opensim in path string
iOsim = Paths.find('OpenSim')

# find the last ;  before opensim
Paths.find(';')
index_start = iOsim
statement = True
while statement:
	index_start = index_start-1
	firstPcomm = Paths.find(';', index_start)
	statement = firstPcomm>iOsim

# find first ; after opsim
lastPcomm= Paths.find(';', iOsim)

# test opensim path
print(index_start)
print(iOsim)
print(lastPcomm)
OsimPath = Paths[index_start+1:lastPcomm]