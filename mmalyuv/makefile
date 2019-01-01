INC = -I/opt/vc/include/
LIB = -L/opt/vc/lib/
LNK = -lmmal_core -lmmal_util -lmmal_vc_client -lvcos -lbcm_host -lvcsm -lvchiq_arm -lpthread -ldl

all:
	gcc $(LIB) $(INC) RaspiVidYUV.c RaspiPreview.c RaspiCamControl.c RaspiCommonSettings.c RaspiHelpers.c RaspiCLI.c RaspiGPS.c libgps_loader.c $(LNK) -o myRaspiVidYUV

