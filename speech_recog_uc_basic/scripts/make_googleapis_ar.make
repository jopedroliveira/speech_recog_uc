#GOOGLEAPIS_GENS_PATH ?= $(HOME)/speech/googleapis/gens
GOOGLEAPIS_API_CCS = $(shell find $(GOOGLEAPIS_GENS_PATH)/google/api \
	-name '*.pb.cc')
GOOGLEAPIS_RPC_CCS = $(shell find $(GOOGLEAPIS_GENS_PATH)/google/rpc \
	-name '*.pb.cc')
GOOGLEAPIS_SPEECH_CCS = $(shell find \
	$(GOOGLEAPIS_GENS_PATH)/google/cloud/speech -name '*.pb.cc')
GOOGLEAPIS_LONGRUNNING_CCS = $(shell find \
	$(GOOGLEAPIS_GENS_PATH)/google/longrunning -name '*.pb.cc')
GOOGLEAPIS_CCS = $(GOOGLEAPIS_API_CCS) $(GOOGLEAPIS_RPC_CCS) \
	$(GOOGLEAPIS_LONGRUNNING_CCS) $(GOOGLEAPIS_SPEECH_CCS)

HOST_SYSTEM = $(shell uname | cut -f 1 -d_)
SYSTEM ?= $(HOST_SYSTEM)
CXX = g++
CPPFLAGS += -I/usr/local/include -I/usr/include -pthread -g -I$(GOOGLEAPIS_GENS_PATH)
CXXFLAGS += -std=c++11
ifeq ($(SYSTEM),Darwin)
LDFLAGS += -L/usr/local/lib -L/usr/lib/x86_64-linux-gnu `pkg-config --libs grpc++ grpc`       \
           -lgrpc++_reflection                                    \
           -lprotobuf -lpthread -ldl -lasound -ljack -lportaudio
else
LDFLAGS += -L/usr/local/lib `pkg-config --libs grpc++ grpc`       \
           -Wl,--no-as-needed -lgrpc++_reflection -Wl,--as-needed \
           -lprotobuf -lpthread -ldl -lasound -lportaudio
endif

.PHONY: all
all: googleapis.ar

googleapis.ar: $(GOOGLEAPIS_CCS:.cc=.o)
	ar r $@ $?

clean:
	rm -f *.o googleapis.ar \
		$(GOOGLEAPIS_CCS:.cc=.o)

