SRCS = putest2.c pudmalib2.c
#OBJS = putest2.o pudmalib2.o

TARGET = putest

TARGET32 = $(TARGET)32-static
TARGET64 = $(TARGET)64-static
TARGET64D = $(TARGET)64-dynamic

BIN = ../bin/x86/linux/
BIN2 = ../


all: $(TARGET32) $(TARGET64) $(TARGET64D)

$(TARGET64): $(SRCS) dockcross-x64
	./dockcross-x64 bash -c '$$CC $(SRCS) -o $@ -static -O2'
	cp $@ $(BIN); mv $@ $(BIN2)

$(TARGET64D): $(SRCS) dockcross-x64
	./dockcross-x64 bash -c '$$CC $(SRCS) -o $@ -O2'
	cp $@ $(BIN); mv $@ $(BIN2)


$(TARGET32): $(SRCS) dockcross-x86
	./dockcross-x86 bash -c '$$CC $(SRCS) -o $@ -static -O2'
	cp $@ $(BIN)

dockcross-x64:
	docker run --rm dockcross/linux-x64 > ./dockcross-x64
	chmod +x ./dockcross-x64

dockcross-x86:
	docker run --rm dockcross/linux-x86 > ./dockcross-x86
	chmod +x ./dockcross-x86

clean:
	rm -f dockcross-* $(TARGET32) $(TARGET64) $(TARGET64D) $(BIN)/*
