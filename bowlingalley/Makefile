INC_BULLET=-I/usr/include/bullet
INC_OSG=-I/usr/include/osg
LD_OSG=-losg -losgViewer -losgSim -l osgDB -l osgGA -l OpenThreads -losgText -losgShadow -losgUtil
BASE_DIR=http://ave.dee.isep.ipp.pt/~jml/intmu/lab9
	

bowling: bowling.o 
	g++ -O2 -o $@ $^ -l BulletDynamics -l BulletCollision -l LinearMath ${LD_OSG} -l stdc++ -lm

bowling.o: bowling.cpp
	gcc -Wall -O2 -c ${INC_BULLET} ${INC_OSG} $<

get:
	wget -q ${BASE_DIR}/bowling.cpp
	wget -q ${BASE_DIR}/bullet_hello.cpp
	wget -q ${BASE_DIR}/bowling.mtl
	wget -q ${BASE_DIR}/bowling.obj
	wget -q ${BASE_DIR}/plano.mtl
	wget -q ${BASE_DIR}/plano.obj
	 wget -q ${BASE_DIR}/terra.png
	
