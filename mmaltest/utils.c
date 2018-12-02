
#include "pigun.h"



void pigun_camera_setup(MMAL_COMPONENT_T *cam, unsigned int resx, unsigned int resy) {
	
	
	// setup camera parameters
    {
        MMAL_PARAMETER_CAMERA_CONFIG_T cam_config = {
            { MMAL_PARAMETER_CAMERA_CONFIG, sizeof (cam_config)},
            .max_stills_w = resx,
            .max_stills_h = resy,
            .stills_yuv422 = 0,
            .one_shot_stills = 1,
            .max_preview_video_w = resx,
            .max_preview_video_h = resy,
            .num_preview_video_frames = 3,
            .stills_capture_circular_buffer_height = 0,
            .fast_preview_resume = 0,
            .use_stc_timestamp = MMAL_PARAM_TIMESTAMP_MODE_RESET_STC
        };
        mmal_port_parameter_set(cam->control, &cam_config.hdr);
    }
    
}


void pigun_port_setformat(MMAL_PORT_T *port, unsigned int resx, unsigned int resy) {
	
	MMAL_ES_FORMAT_T *format;
	
	format = port->format;
	
	format->encoding = MMAL_ENCODING_I420;
    format->encoding_variant = MMAL_ENCODING_I420;

    format->es->video.width = resx;
    format->es->video.height = resy;
	format->es->video.frame_rate.num = PIGUN_FPS;
	format->es->video.frame_rate.den = 1;
    format->es->video.crop.x = 0;
    format->es->video.crop.y = 0;
    format->es->video.crop.width = resx;
    format->es->video.crop.height = resy;
	
	
	mmal_port_format_commit(port);
}


void pigun_port_setformat_buffered(MMAL_PORT_T *port, unsigned int resx, unsigned int resy, unsigned int nbuf) {
	
	MMAL_ES_FORMAT_T *format;
	
	format = port->format;
	
	format->encoding = MMAL_ENCODING_I420;
    format->encoding_variant = MMAL_ENCODING_I420;

    format->es->video.width = resx;
    format->es->video.height = resy;
	format->es->video.frame_rate.num = PIGUN_FPS;
	format->es->video.frame_rate.den = 1;
    format->es->video.crop.x = 0;
    format->es->video.crop.y = 0;
    format->es->video.crop.width = resx;
    format->es->video.crop.height = resy;
	
	port->buffer_size = port->buffer_size_recommended;
	port->buffer_num = nbuf;
	
	mmal_port_format_commit(port);
}




// called when a frame is ready on the port
void pigun_video_buffer_callback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer) {
	
	printf("got buffer!\n");
	
	mmal_buffer_header_release(buffer);
	
}

