#include "ch.h"
#include "hal.h"
#include <main.h>
#include <usbcfg.h>
#include <chprintf.h>

#include <motors.h>
#include <audio/microphone.h>
#include <traitement_son.h>
#include <arm_math.h>
#include <arm_const_structs.h>

//semaphore
static BSEMAPHORE_DECL(traitement, TRUE);

//2 times FFT_SIZE because these arrays contain complex numbers (real + imaginary)
static float micLeft_cmplx_input[2 * FFT_SIZE];
static float micRight_cmplx_input[2 * FFT_SIZE];
static float micFront_cmplx_input[2 * FFT_SIZE];
static float micBack_cmplx_input[2 * FFT_SIZE];
//Arrays containing the computed magnitude of the complex numbers
static float micLeft_output[FFT_SIZE];
static float micRight_output[FFT_SIZE];
static float micFront_output[FFT_SIZE];
static float micBack_output[FFT_SIZE];
//buffer to not write on data
static float micLeft_cmplx_input_buf[2 * FFT_SIZE];
static float micRight_cmplx_input_buf[2 * FFT_SIZE];
static float micFront_cmplx_input_buf[2 * FFT_SIZE];
static float micBack_cmplx_input_buf[2 * FFT_SIZE];

static uint8_t samples_count=0;

/*pour intensity deph
 *
 static float intensity_const=10;
static float l_x=3;*/
static float angle=0.;

#define FREQ_TRAITEMENT		1
#define MIN_VALUE_THRESHOLD	1000


#define MIN_FREQ		10	//we don't analyze before this index to not use resources for nothing
#define MAX_FREQ		130	//we don't analyze after this index to not use resources for nothing

/* Define complex multiplication and its conjugate */
#define  rmul(x,y)      (x.real * y.real - x.imag * y.imag)
#define  imul(x,y)      (x.imag * y.real + x.real * y.imag)
#define rcmul(x,y)      (x.real * y.real + x.imag * y.imag)
#define icmul(x,y)      (x.imag * y.real - x.real * y.imag)

/*
*	Wrapper to call a very optimized fft function provided by ARM
*	which uses a lot of tricks to optimize the computations
*/
void doFFT_optimized(uint16_t size, float* complex_buffer){
	if(size == 1024)
		arm_cfft_f32(&arm_cfft_sR_f32_len1024, complex_buffer, 0, 1);

}

/*test fonction, a adapter plus tard*/
/*void position_control(float angle, int freq_indice){
	float error=0. ,speed=0.;
	static float sum_error=0;

	error = angle - ANGLE_MARGIN;
	//pour eviter des emballements
	if (error > MAX_ERROR)
	{
		error = MAX_ERROR;
	}
	else if (error < -MAX_ERROR)
	{
		error = -MAX_ERROR;
	}
	//we set a maximum and a minimum for the sum to avoid an uncontrolled growth
	if(sum_error > MAX_SUM_ERROR){
		sum_error = MAX_SUM_ERROR;
	}else if(sum_error < -MAX_SUM_ERROR){
		sum_error = -MAX_SUM_ERROR;
	}
	speed = KP * error + KI * sum_error;
	if(micLeft_output[freq_indice] < micRight_output[freq_indice]){
		right_motor_set_speed(speed);
		left_motor_set_speed(-speed);
	}
	else {
		right_motor_set_speed(-speed);
		left_motor_set_speed(speed);
	}
}*/

void traitement_data(void){
	float max_norm[4] = {MIN_VALUE_THRESHOLD,MIN_VALUE_THRESHOLD,MIN_VALUE_THRESHOLD,MIN_VALUE_THRESHOLD};
	int16_t max_norm_index[4] = {-1,-1,-1,-1};
	float test_angle1,test_angle2,test,deph_buf;
	static float deph =0.;

	//search for the highest peak
	for(uint16_t i = MIN_FREQ ; i <= MAX_FREQ ; i++){
		if(micRight_output[i] > max_norm[MIC_RIGHT_I]){
			max_norm[MIC_RIGHT_I] = micRight_output[i];
			max_norm_index[MIC_RIGHT_I] = i;
		}
		if(micLeft_output[i] > max_norm[MIC_LEFT_I]){
			max_norm[MIC_LEFT_I] = micLeft_output[i];
			max_norm_index[MIC_LEFT_I] = i;
		}
		if(micFront_output[i] > max_norm[MIC_FRONT_I]){
			max_norm[MIC_FRONT_I] = micFront_output[i];
			max_norm_index[MIC_FRONT_I] = i;
		}
		if(micBack_output[i] > max_norm[MIC_BACK_I]){
			max_norm[MIC_BACK_I] = micBack_output[i];
			max_norm_index[MIC_BACK_I] = i;
		}
	}

	test_angle1= atan2f(micFront_cmplx_input_buf[2*max_norm_index[MIC_FRONT_I]+1], micFront_cmplx_input_buf[2*max_norm_index[MIC_FRONT_I]]);
	test_angle2= atan2f(micBack_cmplx_input_buf[2*max_norm_index[MIC_BACK_I]+1], micBack_cmplx_input_buf[2*max_norm_index[MIC_BACK_I]]);
	deph_buf=(test_angle1-test_angle2);
	if ((-3 < deph_buf) && (deph_buf<3)){
		deph = a*deph+b*deph_buf;
		test = SOUND_CONST*deph/(max_norm_index[MIC_BACK_I]*AUDIO_RESOLUTION);
		angle = asinf(test);
		if((max_norm_index[MIC_BACK_I] == max_norm_index[MIC_FRONT_I]) && (max_norm_index[MIC_BACK_I] >MIN_FREQ)){
			chprintf((BaseSequentialStream *) &SDU1, "\n indice = %6f | deph = %6f test %6f  || angle = %4f || deph_buf %f \n ",
					(float)(max_norm_index[MIC_BACK_I]*AUDIO_RESOLUTION),deph,test,angle*180/3.14,deph_buf);
			/*chprintf((BaseSequentialStream *) &SDU1, "left output %f right output %f \n\n",micLeft_output[max_norm_index[MIC_LEFT_I]],
						micRight_output[max_norm_index[MIC_LEFT_I]]);*/

		}
	}
	//position_control(angle, max_norm_index[MIC_LEFT_I]);

	/*float emit_intensity = intensity_const*((micFront_output[max_norm_index[MIC_FRONT_I]]*micFront_output[max_norm_index[MIC_FRONT_I]]*
												micLeft_output[max_norm_index[MIC_LEFT_I]]*micLeft_output[max_norm_index[MIC_LEFT_I]])/
												(micFront_output[max_norm_index[MIC_FRONT_I]]*micFront_output[max_norm_index[MIC_FRONT_I]]-
												micLeft_output[max_norm_index[MIC_LEFT_I]]*micLeft_output[max_norm_index[MIC_LEFT_I]]));
		float test_x = emit_intensity*(1/micRight_output[max_norm_index[MIC_RIGHT_I]]-1/micLeft_output[max_norm_index[MIC_LEFT_I]])/(l_x);
		chprintf((BaseSequentialStream *) &SDU1, "\n test x %f emit int %f \n", test_x,emit_intensity);

		test_x=asin(test_x);*/
}


/*
*	Callback called when the demodulation of the four microphones is done.
*	We get 160 samples per mic every 10ms (16kHz)
*
*	params :
*	int16_t *data			Buffer containing 4 times 160 samples. the samples are sorted by micro
*							so we have [micRight1, micLeft1, micBack1, micFront1, micRight2, etc...]
*	uint16_t num_samples	Tells how many data we get in total (should always be 640)
*/
void processAudioData(int16_t *data, uint16_t num_samples){

	/*
	*
	*	We get 160 samples per mic every 10ms
	*	So we fill the samples buffers to reach
	*	1024 samples, then we compute the FFTs.
	*
	*/

	static uint16_t nb_samples = 0;


	//loop to fill the buffers
	for(uint16_t i = 0 ; i < num_samples ; i+=4){
		//construct an array of complex numbers. Put 0 to the imaginary part
		micRight_cmplx_input[nb_samples] = (float)data[i + MIC_RIGHT];
		micLeft_cmplx_input[nb_samples] = (float)data[i + MIC_LEFT];
		micBack_cmplx_input[nb_samples] = (float)data[i + MIC_BACK];
		micFront_cmplx_input[nb_samples] = (float)data[i + MIC_FRONT];

		nb_samples++;

		micRight_cmplx_input[nb_samples] = 0;
		micLeft_cmplx_input[nb_samples] = 0;
		micBack_cmplx_input[nb_samples] = 0;
		micFront_cmplx_input[nb_samples] = 0;

		nb_samples++;


		//stop when buffer is full
		if(nb_samples >= (2 * FFT_SIZE)){
			break;
		}
	}
	if(nb_samples >= (2 * FFT_SIZE) ){
		samples_count++;
		nb_samples = 0;
		if (samples_count<FREQ_TRAITEMENT) return;
		samples_count=0;
		/*	FFT proccessing
		 *
		 *	This FFT function stores the results in the input buffer given.
		 *	This is an "In Place" function.
		 */
		arm_copy_f32(micRight_cmplx_input, micRight_cmplx_input_buf, 2*FFT_SIZE);
		arm_copy_f32(micLeft_cmplx_input, micLeft_cmplx_input_buf, 2*FFT_SIZE);
		arm_copy_f32(micFront_cmplx_input, micFront_cmplx_input_buf, 2*FFT_SIZE);
		arm_copy_f32(micBack_cmplx_input, micBack_cmplx_input_buf, 2*FFT_SIZE);

		doFFT_optimized(FFT_SIZE, micRight_cmplx_input_buf);
		doFFT_optimized(FFT_SIZE, micLeft_cmplx_input_buf);
		doFFT_optimized(FFT_SIZE, micFront_cmplx_input_buf);
		doFFT_optimized(FFT_SIZE, micBack_cmplx_input_buf);

		/*	Magnitude processing
		 *
		 *	Computes the magnitude of the complex numbers and
		 *	stores them in a buffer of FFT_SIZE because it only contains
		 *	real numbers.
		 *
		 */
		arm_cmplx_mag_f32(micRight_cmplx_input_buf, micRight_output, FFT_SIZE);
		arm_cmplx_mag_f32(micLeft_cmplx_input_buf, micLeft_output, FFT_SIZE);
		arm_cmplx_mag_f32(micFront_cmplx_input_buf, micFront_output, FFT_SIZE);
		arm_cmplx_mag_f32(micBack_cmplx_input_buf, micBack_output, FFT_SIZE);

		chBSemSignal(&traitement);

	}
}

void wait_traitement_data(void){
	chBSemWait(&traitement);
}
