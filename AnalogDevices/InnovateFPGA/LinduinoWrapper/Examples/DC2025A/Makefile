MAKEFILE      = Makefile

CC            = gcc
CXX           = g++
DEFINES       = -Dsoc_cv_av
CFLAGS        = -pipe -O2 -Wall -W -D_REENTRANT -fPIC $(DEFINES)
CXXFLAGS      = -pipe -O2 -std=c++0x -Wall -W -D_REENTRANT -fPIC $(DEFINES)
INCPATH       = -I. -I../../Arduino -I../../De10Nano -I../../LT_I2C -I../../LT_SPI -I../../QuikEval_EEPROM -I../../../intel-socfpga-hwlib/armv7a/hwlib/include -I../../../intel-socfpga-hwlib/armv7a/hwlib/include/soc_cv_av -I../../../Linduino/LTSketchbook/libraries/Linduino -I../../../Linduino/LTSketchbook/libraries/USE_WIRE -I../../../Linduino/LTSketchbook/libraries/LTC2668
DEL_FILE      = rm -f
CHK_DIR_EXISTS= test -d
MKDIR         = mkdir -p
COPY          = cp -f
COPY_FILE     = cp -f
COPY_DIR      = cp -f -R
INSTALL_FILE  = install -m 644 -p
INSTALL_PROGRAM = install -m 755 -p
INSTALL_DIR   = cp -f -R
DEL_FILE      = rm -f
SYMLINK       = ln -f -s
DEL_DIR       = rmdir
MOVE          = mv -f
TAR           = tar -cf
COMPRESS      = gzip -9f
DISTNAME      = DC2025A
DISTDIR       = ./
LINK          = g++
LFLAGS        = -Wl,-O1
LIBS          = $(SUBLIBS) -lpthread 
AR            = ar cqs
RANLIB        = 
SED           = sed
STRIP         = strip

OBJECTS_DIR   = ./

SOURCES = DC2025A.cpp \
		../../De10Nano/De10Nano.cpp \
		../../Arduino/Arduino.cpp \
		../../Arduino/WireWrapper.cpp \
		../../Arduino/SpiWrapper.cpp \
		../../LT_I2C/LT_I2C.cpp \
		../../LT_SPI/LT_SPI.cpp \
		../../QuikEval_EEPROM/QuikEval_EEPROM.cpp \
		../../../Linduino/LTSketchbook/libraries/LTC2668/LTC2668.cpp

OBJECTS = DC2025A.o \
		De10Nano.o \
		Arduino.o \
		WireWrapper.o \
		SpiWrapper.o \
		LT_I2C.o \
		LT_SPI.o \
		QuikEval_EEPROM.o \
		LTC2668.o 

DIST    = ../../De10Nano/De10Nano.h \
		../../Arduino/Arduino.h \
		../../Arduino/Wire.h \
		../../Arduino/WireWrapper.h \
		../../Arduino/SPI.h \
		../../Arduino/SpiWrapper.h \
		../../LT_I2C/LT_I2C.h \
		../../LT_SPI/LT_SPI.h \
		../../QuikEval_EEPROM/QuikEval_EEPROM.h \
		../../../Linduino/LTSketchbook/libraries/Linduino/Linduino.h \
		../../../Linduino/LTSketchbook/libraries/LTC2668/LTC2668.h \
		../../De10Nano/De10Nano.cpp \
		../../Arduino/Arduino.cpp \
		../../Arduino/WireWrapper.cpp \
		../../Arduino/SpiWrapper.cpp \
		../../LT_I2C/LT_I2C.cpp \
		../../LT_SPI/LT_SPI.cpp \
		../../QuikEval_EEPROM/QuikEval_EEPROM.cpp \
		../../../Linduino/LTSketchbook/libraries/LTC2668/LTC2668.cpp 
		

DESTDIR       = build

TARGET        = DC2025A

first: all

.SUFFIXES: .o .c .cpp .cc .cxx .C

.cpp.o:
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o "$@" "$<"

.cc.o:
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o "$@" "$<"

.cxx.o:
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o "$@" "$<"

.C.o:
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o "$@" "$<"

.c.o:
	$(CC) -c $(CFLAGS) $(INCPATH) -o "$@" "$<"

$(TARGET):  $(OBJECTS)  
	$(LINK) $(LFLAGS) -o $(TARGET) $(OBJECTS) $(OBJCOMP) $(LIBS)

all: Makefile $(TARGET)

dist: distdir FORCE
	(cd `dirname $(DISTDIR)` && $(TAR) $(DISTNAME).tar $(DISTNAME) && $(COMPRESS) $(DISTNAME).tar) && $(MOVE) `dirname $(DISTDIR)`/$(DISTNAME).tar.gz . && $(DEL_FILE) -r $(DISTDIR)

distdir: FORCE
	@test -d $(DISTDIR) || mkdir -p $(DISTDIR)
	$(COPY_FILE) --parents $(DIST) $(DISTDIR)/
	$(COPY_FILE) --parents ../../De10Nano/De10Nano.h ../../Arduino/Arduino.h ../../Arduino/Wire.h ../../Arduino/WireWrapper.h ../../Arduino/SPI.h ../../Arduino/SpiWrapper.h ../../LT_I2C/LT_I2C.h ../../LT_SPI/LT_SPI.h ../../QuikEval_EEPROM/QuikEval_EEPROM.h ../../../Linduino/LTSketchbook/libraries/Linduino/Linduino.h ../../../Linduino/LTSketchbook/libraries/LTC2668/LTC2668.h $(DISTDIR)/
	$(COPY_FILE) --parents DC2025A.cpp ../../De10Nano/De10Nano.cpp ../../Arduino/Arduino.cpp ../../Arduino/WireWrapper.cpp ../../Arduino/SpiWrapper.cpp ../../LT_I2C/LT_I2C.cpp ../../LT_SPI/LT_SPI.cpp ../../QuikEval_EEPROM/QuikEval_EEPROM.cpp ../../../Linduino/LTSketchbook/libraries/LTC2668/LTC2668.cpp $(DISTDIR)/

clean: compiler_clean
	-$(DEL_FILE) $(OBJECTS)
	-$(DEL_FILE) *~ core *.core

distclean: clean 
	-$(DEL_FILE) $(TARGET) 
	-$(DEL_FILE) Makefile

mocclean: compiler_moc_header_clean compiler_moc_source_clean

mocables: compiler_moc_header_make_all compiler_moc_source_make_all

check: first

compiler_rcc_make_all:
compiler_rcc_clean:
compiler_moc_header_make_all:
compiler_moc_header_clean:
compiler_moc_source_make_all:
compiler_moc_source_clean:
compiler_yacc_decl_make_all:
compiler_yacc_decl_clean:
compiler_yacc_impl_make_all:
compiler_yacc_impl_clean:
compiler_lex_make_all:
compiler_lex_clean:
compiler_clean: 

DC2025A.o: DC2025A.cpp \
        ../../Arduino/Arduino.h \
        ../../Arduino/Wire.h \
		../../Arduino/SPI.h \
		../../Arduino/WireWrapper.h \
        ../../Arduino/SpiWrapper.h \
		../../De10Nano/De10Nano.h \
		../../../intel-socfpga-hwlib/armv7a/hwlib/include/hwlib.h \
		../../../intel-socfpga-hwlib/armv7a/hwlib/include/soc_cv_av/socal/socal.h \
		../../../intel-socfpga-hwlib/armv7a/hwlib/include/soc_cv_av/socal/hps.h \
		../../../intel-socfpga-hwlib/armv7a/hwlib/include/soc_cv_av/socal/alt_gpio.h \
		../../LT_I2C/LT_I2C.h \
		../../LT_SPI/LT_SPI.h \
		../../QuikEval_EEPROM/QuikEval_EEPROM.h \
		../../../Linduino/LTSketchbook/libraries/LTC2668/LTC2668.h 
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o DC2025A.o DC2025A.cpp

De10Nano.o: ../../De10Nano/De10Nano.cpp \
        ../../De10Nano/De10Nano.h \
		../../../intel-socfpga-hwlib/armv7a/hwlib/include/hwlib.h \
		../../../intel-socfpga-hwlib/armv7a/hwlib/include/soc_cv_av/socal/socal.h \
		../../../intel-socfpga-hwlib/armv7a/hwlib/include/soc_cv_av/socal/hps.h \
		../../../intel-socfpga-hwlib/armv7a/hwlib/include/soc_cv_av/socal/alt_gpio.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o De10Nano.o ../../De10Nano/De10Nano.cpp

Arduino.o: ../../Arduino/Arduino.cpp \
        ../../Arduino/Arduino.h \
		../../Arduino/SpiWrapper.h \
		../../Arduino/WireWrapper.h \
		../../De10Nano/De10Nano.h \
		../../../intel-socfpga-hwlib/armv7a/hwlib/include/hwlib.h \
		../../../intel-socfpga-hwlib/armv7a/hwlib/include/soc_cv_av/socal/socal.h \
		../../../intel-socfpga-hwlib/armv7a/hwlib/include/soc_cv_av/socal/hps.h \
		../../../intel-socfpga-hwlib/armv7a/hwlib/include/soc_cv_av/socal/alt_gpio.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o Arduino.o ../../Arduino/Arduino.cpp

WireWrapper.o: ../../Arduino/WireWrapper.cpp \
        ../../Arduino/WireWrapper.h \
		../../De10Nano/De10Nano.h \
		../../../intel-socfpga-hwlib/armv7a/hwlib/include/hwlib.h \
		../../../intel-socfpga-hwlib/armv7a/hwlib/include/soc_cv_av/socal/socal.h \
		../../../intel-socfpga-hwlib/armv7a/hwlib/include/soc_cv_av/socal/hps.h \
		../../../intel-socfpga-hwlib/armv7a/hwlib/include/soc_cv_av/socal/alt_gpio.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o WireWrapper.o ../../Arduino/WireWrapper.cpp

SpiWrapper.o: ../../Arduino/SpiWrapper.cpp \
        ../../Arduino/SpiWrapper.h \
		../../De10Nano/De10Nano.h \
		../../../intel-socfpga-hwlib/armv7a/hwlib/include/hwlib.h \
		../../../intel-socfpga-hwlib/armv7a/hwlib/include/soc_cv_av/socal/socal.h \
		../../../intel-socfpga-hwlib/armv7a/hwlib/include/soc_cv_av/socal/hps.h \
		../../../intel-socfpga-hwlib/armv7a/hwlib/include/soc_cv_av/socal/alt_gpio.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o SpiWrapper.o ../../Arduino/SpiWrapper.cpp

LT_I2C.o: ../../LT_I2C/LT_I2C.cpp \
        ../../LT_I2C/LT_I2C.h \
		../../Arduino/WireWrapper.h \
		../../De10Nano/De10Nano.h \
		../../../intel-socfpga-hwlib/armv7a/hwlib/include/hwlib.h \
		../../../intel-socfpga-hwlib/armv7a/hwlib/include/soc_cv_av/socal/socal.h \
		../../../intel-socfpga-hwlib/armv7a/hwlib/include/soc_cv_av/socal/hps.h \
		../../../intel-socfpga-hwlib/armv7a/hwlib/include/soc_cv_av/socal/alt_gpio.h \
		../../Arduino/Arduino.h \
		../../Arduino/SpiWrapper.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o LT_I2C.o ../../LT_I2C/LT_I2C.cpp

LT_SPI.o: ../../LT_SPI/LT_SPI.cpp \
        ../../LT_SPI/LT_SPI.h \
		../../Arduino/SpiWrapper.h \
		../../De10Nano/De10Nano.h \
		../../../intel-socfpga-hwlib/armv7a/hwlib/include/hwlib.h \
		../../../intel-socfpga-hwlib/armv7a/hwlib/include/soc_cv_av/socal/socal.h \
		../../../intel-socfpga-hwlib/armv7a/hwlib/include/soc_cv_av/socal/hps.h \
		../../../intel-socfpga-hwlib/armv7a/hwlib/include/soc_cv_av/socal/alt_gpio.h \
		../../Arduino/Arduino.h \
		../../Arduino/WireWrapper.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o LT_SPI.o ../../LT_SPI/LT_SPI.cpp

QuikEval_EEPROM.o: ../../QuikEval_EEPROM/QuikEval_EEPROM.cpp \
        ../../QuikEval_EEPROM/QuikEval_EEPROM.h \
		../../Arduino/WireWrapper.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o QuikEval_EEPROM.o ../../QuikEval_EEPROM/QuikEval_EEPROM.cpp

LTC2668.o: ../../../Linduino/LTSketchbook/libraries/LTC2668/LTC2668.cpp \
        ../../Arduino/Arduino.h \
        ../../Arduino/Wire.h \
		../../Arduino/WireWrapper.h \
		../../Arduino/SpiWrapper.h \
		../../LT_I2C/LT_I2C.h \
		../../../Linduino/LTSketchbook/libraries/Linduino/Linduino.h \
		../../../Linduino/LTSketchbook/libraries/USE_WIRE/USE_WIRE.h \
		../../../Linduino/LTSketchbook/libraries/LTC2668/LTC2668.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o LTC2668.o ../../../Linduino/LTSketchbook/libraries/LTC2668/LTC2668.cpp


install:  FORCE

uninstall:  FORCE

FORCE:

