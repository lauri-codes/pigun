


INC = -I/opt/vc/include/
LIB = -L/opt/vc/lib/
LNK = -lbcm_host -lvcos -lvchiq_arm -lpthread -lrt -lmmal -lmmal_core -lmmal_util -lm -ldl
#LDFLAGS += -L$(SDKSTAGE)/opt/vc/lib/ -lbrcmGLESv2 -lbrcmEGL -lopenmaxil -lbcm_host -lvcos -lvchiq_arm -lpthread -lrt -lm -L$(SDKSTAGE)/opt/vc/src/hello_pi/libs/ilclient -L$(SDKSTAGE)/opt/vc/src/hello_pi/libs/vgfont

all:
#	gcc -c $(INC) RaspiCLI.c
#	gcc -c $(INC) RaspiPreview.c
#	gcc -c $(INC) RaspiCamControl.c
#	gcc -c $(INC) utils.c main.c
#	gcc $(LIB) *.o $(LNK)
	gcc $(LIB) $(INC) buffertest.c $(LNK)
	
