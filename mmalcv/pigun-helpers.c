
#include "pigun.h"



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

	return 0;
	//return mmal_status_to_int(mmal_port_parameter_set(camera->control, &param.hdr));
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

	//return mmal_status_to_int(mmal_port_parameter_set(camera->control, &param.hdr));
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

	//return mmal_status_to_int(mmal_port_parameter_set(camera->control, &imgFX.hdr));
	return 0;
}


