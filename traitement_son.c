#include <audio/microphone.h>
#include <traitement_son.h>
#include <arm_math.h>
#include <arm_const_structs.h>


//intensity min du son pour ne pas etre traité comme bruit
#define MIN_VALUE_THREESHOLD		10000

//definie la fréquence du traitement sonore, 1 est la fréquence maximum.
#define FREQ_TRAITEMENT		1

#define FFT_SIZE 			1024
#define AUDIO_RESOLUTION   	15.23f
// SOUND_CONST=340/(2*PI*lx*AUDIO_RESOLUTION), lx la distance entre les deux micros sur l'axe x lx=6cm
#define SOUND_CONST			59.21
//paramètre de la moyenne mobile : angle = a*angle +b*angle_buf
#define A					0.7f
#define B					0.3f

#define MIN_FREQ			29		//we don't analyze before this index to not use resources for nothing
#define MAX_FREQ			66		//we don't analyze after this index to not use resources for nothing


typedef enum {
	//2 times FFT_SIZE because these arrays contain complex numbers (real + imaginary)
	LEFT_CMPLX_INPUT = 0,
	RIGHT_CMPLX_INPUT,
	FRONT_CMPLX_INPUT,
	BACK_CMPLX_INPUT,
	//Arrays containing the computed magnitude of the complex numbers
	LEFT_OUTPUT,
	RIGHT_OUTPUT,
	FRONT_OUTPUT,
	BACK_OUTPUT
} BUFFER_NAME_t;

typedef enum {
	MIC_RIGHT_I = 0,
	MIC_LEFT_I,
	MIC_BACK_I,
	MIC_FRONT_I
} INDEX_MICROPHONE;


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

static float angle=0., amp=0., freq=0.;

/*
 *fonction determinant l'angle d'incidence du son par rapport à l'axe x du robot
 *Définie positif dans le sens inverse des aiguilles d'une montre de [0,2*PI]
 */
void set_localisation(float angle_x,float angle_y);

void set_localisation(float angle_x,float angle_y){
	if (angle_x>0 && angle_y>0){
		angle=(angle_x+(PI/2-angle_y))/2;
	}
	if (angle_x>0 && angle_y<0){
		angle=PI/2+(-angle_y+(PI/2-angle_x))/2;
	}
	if (angle_x<0 && angle_y<0){
		angle=PI+(-angle_x+(PI/2+angle_y))/2;
	}
	if (angle_x<0 && angle_y>0){
		angle=3*PI/2+(PI/2+angle_x+angle_y)/2;
	}
}


void traitement_data(void){
	float max_norm[4] = {MIN_VALUE_THREESHOLD,MIN_VALUE_THREESHOLD,MIN_VALUE_THREESHOLD,MIN_VALUE_THREESHOLD};
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

	if((max_norm_index[MIC_LEFT_I] == max_norm_index[MIC_RIGHT_I]) && (max_norm_index[MIC_LEFT_I] >MIN_FREQ)
			&& (micLeft_output[max_norm_index[MIC_LEFT_I]]>MIN_VALUE_THREESHOLD))
	{
		deph_left= atan2f(micLeft_cmplx_input_buf[2*max_norm_index[MIC_LEFT_I]+1], micLeft_cmplx_input_buf[2*max_norm_index[MIC_LEFT_I]]);
		deph_right= atan2f(micRight_cmplx_input_buf[2*max_norm_index[MIC_RIGHT_I]+1], micRight_cmplx_input_buf[2*max_norm_index[MIC_RIGHT_I]]);
		deph_buf_x=(deph_left-deph_right);
		//filtre des pics
		if ((-PI < deph_buf_x) && (deph_buf_x<PI)){
			//moyenne mobile pour eviter les changements trop abrute
			deph_x = A*deph_x+B*deph_buf_x;
			test = SOUND_CONST*deph_x/(max_norm_index[MIC_LEFT_I]);
			if (test > 1) test=1;
			if (test < -1) test=-1;
			angle_x = asinf(test);
		}
	}
	if((max_norm_index[MIC_BACK_I] == max_norm_index[MIC_FRONT_I]) && (max_norm_index[MIC_BACK_I] >MIN_FREQ)
			 && (micFront_output[max_norm_index[MIC_FRONT_I]]>MIN_VALUE_THREESHOLD))
	{
		deph_front= atan2f(micFront_cmplx_input_buf[2*max_norm_index[MIC_FRONT_I]+1], micFront_cmplx_input_buf[2*max_norm_index[MIC_FRONT_I]]);
		deph_back= atan2f(micBack_cmplx_input_buf[2*max_norm_index[MIC_BACK_I]+1], micBack_cmplx_input_buf[2*max_norm_index[MIC_BACK_I]]);
		deph_buf_y=(deph_front-deph_back);
		if ((-PI < deph_buf_y) && (deph_buf_y<PI)){
			deph_y = A*deph_y+B*deph_buf_y;
			test = SOUND_CONST*deph_y/(max_norm_index[MIC_LEFT_I]);
			if (test > 1) test=1;
			if (test < -1) test=-1;
			angle_y = asinf(test);
		}
	}

	if ((max_norm_index[MIC_LEFT_I] == max_norm_index[MIC_RIGHT_I])&&(max_norm_index[MIC_BACK_I] == max_norm_index[MIC_FRONT_I])){
			freq=max_norm_index[MIC_LEFT_I];
	}
	amp=micFront_output[max_norm_index[MIC_FRONT_I]];
	//défini l'angle global d'incidence du son
	set_localisation(angle_x,angle_y);
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
	 *	We get 160 samples per mic every 10ms
	 *	So we fill the samples buffers to reach
	 *	1024 samples, then we compute the FFTs.
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

		/*
		 * copie des input pour ne pas les overwright pdt le traitement
		 */
		arm_copy_f32(micRight_cmplx_input, micRight_cmplx_input_buf, 2*FFT_SIZE);
		arm_copy_f32(micLeft_cmplx_input, micLeft_cmplx_input_buf, 2*FFT_SIZE);
		arm_copy_f32(micFront_cmplx_input, micFront_cmplx_input_buf, 2*FFT_SIZE);
		arm_copy_f32(micBack_cmplx_input, micBack_cmplx_input_buf, 2*FFT_SIZE);

		arm_cfft_f32(&arm_cfft_sR_f32_len1024, micRight_cmplx_input_buf, 0, 1);
		arm_cfft_f32(&arm_cfft_sR_f32_len1024, micLeft_cmplx_input_buf, 0, 1);
		arm_cfft_f32(&arm_cfft_sR_f32_len1024, micFront_cmplx_input_buf, 0, 1);
		arm_cfft_f32(&arm_cfft_sR_f32_len1024, micBack_cmplx_input_buf, 0, 1);

		/*	Magnitude processing
		 */
		arm_cmplx_mag_f32(micRight_cmplx_input_buf, micRight_output, FFT_SIZE);
		arm_cmplx_mag_f32(micLeft_cmplx_input_buf, micLeft_output, FFT_SIZE);
		arm_cmplx_mag_f32(micFront_cmplx_input_buf, micFront_output, FFT_SIZE);
		arm_cmplx_mag_f32(micBack_cmplx_input_buf, micBack_output, FFT_SIZE);

		chBSemSignal(&traitement);

	}
}

float get_angle(void){
	return angle;
}

float get_freq(void){
	return freq;
}

float get_amp(void){
	return amp;
}

void wait_traitement_data(void){
	chBSemWait(&traitement);
}
