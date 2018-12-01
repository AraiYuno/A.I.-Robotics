###############################################################
#
# Purpose: Makefile for "FIRA Template"
# Author.: robotis
# Version: 0.1
# License: GPL
#
###############################################################

TARGET = Template

INCLUDE_DIRS = -I../../include -I../../../Framework/include -I/usr/local/include/opencv -I/usr/local/include

CXX = g++
CXXFLAGS += -O2 -DLINUX -Wall $(INCLUDE_DIRS)
#CXXFLAGS += -O2 -DDEBUG -DLINUX -Wall $(INCLUDE_DIRS)
LFLAGS += -lpthread -ljpeg -lrt -L/usr/local/lib -lopencv_calib3d -lopencv_core -lopencv_features2d -lopencv_flann -lopencv_highgui -lopencv_imgproc -lopencv_imgcodecs -lopencv_ml -lopencv_objdetect -lopencv_photo -lopencv_stitching -lopencv_superres -lopencv_video -lopencv_videostab -lrt -lpthread -lm -ldl

OBJECTS = StatusCheck.o Template.o GenericVision.o Follower.o

all: $(TARGET)

clean:
	rm -f *.a *.o $(TARGET) core *~ *.so *.lo

libclean:
	make -C ../../build clean
collect2: ld returned 1 exit status

distclean: clean libclean

darwin.a:
	make -C ../../build

$(TARGET): darwin.a $(OBJECTS)
	$(CXX) $(CFLAGS) $(OBJECTS) ../../lib/darwin.a -o $(TARGET) $(LFLAGS)
	chmod 755 $(TARGET)

# useful to make a backup "make tgz"
tgz: clean
	mkdir -p backups
	tar czvf ./backups/DARwIn_Template_`date +"%Y_%m_%d_%H.%M.%S"`.tgz --exclude backups *
