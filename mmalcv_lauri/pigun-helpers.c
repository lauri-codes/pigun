
#include "pigun.h"


int pigun_camera_exposuremode(MMAL_COMPONENT_T *camera, int on) {

  MMAL_PARAM_EXPOSUREMODE_T mode;
  
  if(on == 1) {
    mode = MMAL_PARAM_EXPOSUREMODE_OFF;
  } else {
    mode = MMAL_PARAM_EXPOSUREMODE_AUTO;
  }
  
  //raspicamcontrol_set_exposure_mode(camera, MMAL_PARAM_EXPOSUREMODE_AUTO);
  MMAL_PARAMETER_EXPOSUREMODE_T exp_mode = {{MMAL_PARAMETER_EXPOSURE_MODE, sizeof(exp_mode)}, mode};
  mmal_port_parameter_set(camera->control, &exp_mode.hdr);
  
  return 0;
}



/**
 * Set the aWB (auto white balance) mode for images
 * @param camera Pointer to camera component
 * @param on 0=MMAL_PARAM_AWBMODE_OFF, 1=MMAL_PARAM_AWBMODE_AUTO
 * @return 0 if successful, non-zero if something went wrong
 */
int pigun_camera_awb(MMAL_COMPONENT_T *camera, int on) {

	if (!camera)
		return 1;

	MMAL_PARAMETER_AWBMODE_T param;

	if(on == 1) {
		param = {{MMAL_PARAMETER_AWB_MODE, sizeof(param)}, MMAL_PARAM_AWBMODE_AUTO};
	} else {
		param = {{MMAL_PARAMETER_AWB_MODE, sizeof(param)}, MMAL_PARAM_AWBMODE_OFF};
	}

	mmal_port_parameter_set(camera->control, &param.hdr);
	return 0;
	
}


int pigun_camera_awb_gains(MMAL_COMPONENT_T *camera, float r_gain, float b_gain) {


	if(!camera)
		return 1;

	MMAL_PARAMETER_AWB_GAINS_T param = {{MMAL_PARAMETER_CUSTOM_AWB_GAINS,sizeof(param)}, {0,0}, {0,0}};

	if (!r_gain || !b_gain)
		return 0;

	param.r_gain.num = (unsigned int)(r_gain * 65536);
	param.b_gain.num = (unsigned int)(b_gain * 65536);
	param.r_gain.den = param.b_gain.den = 65536;

	mmal_port_parameter_set(camera->control, &param.hdr);
	return 0;
}


/**
 * Set the image effect for the images
 * @param camera Pointer to camera component
 * @param on 0=MMAL_PARAM_IMAGEFX_NONE, 1=MMAL_PARAM_IMAGEFX_BLUR
 * @return 0 if successful, non-zero if any parameters out of range
 */
int pigun_camera_blur(MMAL_COMPONENT_T *camera, int on) {
	
	if (!camera)
		return 1;

	MMAL_PARAMETER_IMAGEFX_T imgFX;

	if(on == 1)
		imgFX = {{MMAL_PARAMETER_IMAGE_EFFECT,sizeof(imgFX)}, MMAL_PARAM_IMAGEFX_BLUR};
	else
		imgFX = {{MMAL_PARAMETER_IMAGE_EFFECT,sizeof(imgFX)}, MMAL_PARAM_IMAGEFX_NONE};

	mmal_port_parameter_set(camera->control, &imgFX.hdr);
	return 0;
}


