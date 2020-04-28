#include "ch.h"
#include "hal.h"
#include <main.h>
#include <usbcfg.h>
#include <chprintf.h>

#include <leds.h>
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

static uint8_t samples_count=0,freq_samples=0;
static float angle=0., freq_buf;

#define FREQ_TRAITEMENT		1
#define MIN_VALUE_THRESHOLD	3000

#define MIN_FREQ			10		//we don't analyze before this index to not use resources for nothing
#define AVANT_FREQ_MIN		17		//258hz
#define AVANT_FREQ_MAX		27		//411hz
#define ARRIERE_FREQ_MIN	27		//411hz
#define ARRIERE_FREQ_MAX	37		//563hz
#define DROITE_FREQ_MIN		37		//563
#define DROITE_FREQ_MAX		47		//715
#define GAUCHE_FREQ_MIN		47		//715
#define GAUCHE_FREQ_MAX		57		//868
#define MAX_FREQ			130		//we don't analyze after this index to not use resources for nothing


/*
 * Allume les leds pour indiquer la direction de provenance du son
 */
void set_direction_led(void){
	if ((angle<2*PI/8) || (angle>14*PI/8))   set_led(LED1,1);
	if ((PI/8<angle) && (3*PI/8>angle))		 set_rgb_led(LED8,255,0,0);
	if ((2*PI/8<angle) && (6*PI/8>angle))    set_led(LED7,1);
	if ((5*PI/8<angle) && (7*PI/8>angle))	 set_rgb_led(LED6,255,0,0);
	if ((6*PI/8<angle) && (10*PI/8>angle))   set_led(LED5,1);
	if ((9*PI/8<angle) && (11*PI/8>angle))	 set_rgb_led(LED4,255,0,0);
	if ((10*PI/8<angle) && (14*PI/8>angle))  set_led(LED3,1);
	if ((13*PI/8<angle) && (15*PI/8>angle))	 set_rgb_led(LED2,255,0,0);
}
/*Oriente le robot dans la direction du son
 *a revoir quelque modif
 */
void sound_response(int sound_index){
	int16_t vitesse_d,vitesse_g;
	float c_angle,s_angle;
	static int16_t sum_error;
	c_angle=cosf(angle);
	s_angle=sinf(angle);
	// dirige le robot vers la source de son pour les fréquences [260,]Hz
	if ((sound_index>AVANT_FREQ_MIN)&&(sound_index<AVANT_FREQ_MAX)){
		sum_error += (COS_AVANT-c_angle);
		if (abs(sum_error)> MAX_SUM_ERROR) sum_error=MAX_SUM_ERROR;
		vitesse_d=(COS_AVANT-c_angle)*KP+(sum_error)*KI+KA*c_angle;
		vitesse_g=-((COS_AVANT-c_angle)*KP+(sum_error)*KI)+KA*c_angle;
	    if (s_angle > NUL){
			right_motor_set_speed(vitesse_d);
			left_motor_set_speed(vitesse_g);
		}
		else {
			right_motor_set_speed(vitesse_g);
			left_motor_set_speed(vitesse_d);
		}
	    set_direction_led();
	    set_front_led(1);
	}
	//dirige le robot à l'inverse de la source de son pour les frequence []Hz
	else if ((sound_index>ARRIERE_FREQ_MIN)&&(sound_index<ARRIERE_FREQ_MAX)){
		sum_error += (COS_ARRIERE-c_angle);
		if (abs(sum_error)> MAX_SUM_ERROR) sum_error=-MAX_SUM_ERROR;
		vitesse_d=(COS_ARRIERE-c_angle)*KP+(sum_error)*KI-KA*c_angle;
		vitesse_g=-((COS_ARRIERE-c_angle)*KP+(sum_error)*KI)-KA*c_angle;
		if (s_angle > NUL){
			right_motor_set_speed(vitesse_d);
			left_motor_set_speed(vitesse_g);
		}
		else {
			right_motor_set_speed(vitesse_g);
			left_motor_set_speed(vitesse_d);
		}
		set_direction_led();
		set_front_led(1);
	}
	//pas de mouvement sinon
	else {
		right_motor_set_speed(NUL);
		left_motor_set_speed(NUL);
	}
	/*chprintf((BaseSequentialStream *) &SD3,"front %6f back %6f delta %f\n",micFront_output[sound_index],micFront_output[sound_index],
																		 micFront_output[sound_index]-micFront_output[sound_index]);
	*/
}

/*fonction determinant l'angle d'incidence du son par rapport à l'axe x du robot
 *Définie positif dans le sens inverse des aiguilles d'une montre de [0,2*PI]
 *
 */
void set_localisation(float angle_x,float angle_y){
	if (angle_x>NUL && angle_y>NUL){
		angle=(angle_x+(PI/2-angle_y))/2;
	}
	if (angle_x>NUL && angle_y<NUL){
		angle=PI/2+(-angle_y+(PI/2-angle_x))/2;
	}
	if (angle_x<NUL && angle_y<NUL){
		angle=PI+(-angle_x+(PI/2+angle_y))/2;
	}
	if (angle_x<NUL && angle_y>NUL){
		angle=1.5*PI+(PI/2+angle_x+angle_y)/2;
	}
}


void traitement_data(void){
	float max_norm[4] = {MIN_VALUE_THRESHOLD,MIN_VALUE_THRESHOLD,MIN_VALUE_THRESHOLD,MIN_VALUE_THRESHOLD};
	int16_t max_norm_index[4] = {-1,-1,-1,-1};
	float deph_right,deph_left,test,deph_buf_x, deph_front, deph_back, deph_buf_y;
	static float deph_x=0., deph_y=0.;
	static float angle_y=0., angle_x=0.;

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
	//if (freq_buf==max_norm[MIC_BACK_I]) freq_samples++;
	//if (freq_buf>FREQ_STAB_IND) freq_buf=FREQ_STAB_IND;

	if((max_norm_index[MIC_LEFT_I] == max_norm_index[MIC_RIGHT_I]) && (max_norm_index[MIC_LEFT_I] >MIN_FREQ)
			/*&& (freq_samples== FREQ_STAB_IND)*/ && (micLeft_output[max_norm_index[MIC_LEFT_I]]>SOUND_THREESHOLD)){
		deph_left= atan2f(micLeft_cmplx_input_buf[2*max_norm_index[MIC_LEFT_I]+1], micLeft_cmplx_input_buf[2*max_norm_index[MIC_LEFT_I]]);
		deph_right= atan2f(micRight_cmplx_input_buf[2*max_norm_index[MIC_RIGHT_I]+1], micRight_cmplx_input_buf[2*max_norm_index[MIC_RIGHT_I]]);
		deph_buf_x=(deph_left-deph_right);
		if ((-3 < deph_buf_x) && (deph_buf_x<3)){
			deph_x = A*deph_x+B*deph_buf_x;
			test = SOUND_CONST*deph_x/(max_norm_index[MIC_LEFT_I]);
			if (test > 1) test=1;
			if (test < -1) test=-1;
			angle_x = asinf(test);
		}
	}
	if((max_norm_index[MIC_BACK_I] == max_norm_index[MIC_FRONT_I]) && (max_norm_index[MIC_BACK_I] >MIN_FREQ)
			/*&& (freq_samples== FREQ_STAB_IND)*/ && (micFront_output[max_norm_index[MIC_FRONT_I]]>SOUND_THREESHOLD)){
		deph_front= atan2f(micFront_cmplx_input_buf[2*max_norm_index[MIC_FRONT_I]+1], micFront_cmplx_input_buf[2*max_norm_index[MIC_FRONT_I]]);
		deph_back= atan2f(micBack_cmplx_input_buf[2*max_norm_index[MIC_BACK_I]+1], micBack_cmplx_input_buf[2*max_norm_index[MIC_BACK_I]]);
		deph_buf_y=(deph_front-deph_back);
		if ((-3 < deph_buf_y) && (deph_buf_y<3)){
			deph_y = A*deph_y+B*deph_buf_y;
			test = SOUND_CONST*deph_y/(max_norm_index[MIC_LEFT_I]);
			if (test > 1) test=1;
			if (test < -1) test=-1;
			angle_y = asinf(test);
		}
	}
	/*if ((max_norm_index[MIC_LEFT_I] == max_norm_index[MIC_RIGHT_I])&&(max_norm_index[MIC_BACK_I] == max_norm_index[MIC_FRONT_I])){
			freq_buf=max_norm_index[MIC_LEFT_I];
	}*/
	//if (freq_samples == FREQ_STAB_IND){
	set_localisation(angle_x,angle_y);
	sound_response(max_norm_index[MIC_LEFT_I]);
	//}
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
		/*
		 * DoFFT optimised
		 */
		arm_cfft_f32(&arm_cfft_sR_f32_len1024, micRight_cmplx_input_buf, 0, 1);
		arm_cfft_f32(&arm_cfft_sR_f32_len1024, micLeft_cmplx_input_buf, 0, 1);
		arm_cfft_f32(&arm_cfft_sR_f32_len1024, micFront_cmplx_input_buf, 0, 1);
		arm_cfft_f32(&arm_cfft_sR_f32_len1024, micBack_cmplx_input_buf, 0, 1);

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
