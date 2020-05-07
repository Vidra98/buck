#ifndef TRAITEMENT_SON
#define TRAITEMENT_SON

#define FFT_SIZE 			1024
#define AUDIO_RESOLUTION   	15.23f
#define SOUND_CONST			59.21 // =340/(2*PI*lx*AUDIO_RESOLUTION), lx=6cm
#define SOUND_THREESHOLD    10000.0f
//param�tre de la moyenne mobile : angle = a*angle +b*angle_buf
#define A					0.7f
#define B					0.3f

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

#define MIN_FREQ			10		//we don't analyze before this index to not use resources for nothing
#define AVANT_FREQ_MIN		17		//258hz
#define AVANT_FREQ_MAX		27		//411hz
#define ARRIERE_FREQ_MIN	27		//411hz
#define ARRIERE_FREQ_MAX	37		//563hz
#define MAX_FREQ			40		//we don't analyze after this index to not use resources for nothing

//intensity min du son pour ne pas etre trait� comme bruit
#define MIN_VALUE_THREESHOLD		5000
//intensity min du son pour etre track�
#define MIN_INTENSITY_TRACKING	10000



//Traite les informations provenant des micro
 void processAudioData(int16_t *data, uint16_t num_samples);

//put the invoking thread into sleep until it can process the audio datas
void wait_traitement_data(void);

//localise le son selon l'axe x et y
void traitement_data(void);

float get_angle(void);

float get_freq(void);

float get_amp(void);

#endif
