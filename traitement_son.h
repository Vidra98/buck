#ifndef TRAITEMENT_SON
#define TRAITEMENT_SON

#define FFT_SIZE 			1024
#define AUDIO_RESOLUTION   	15.23f
#define SOUND_CONST			59.21 // =340/(2*PI*lx*AUDIO_RESOLUTION), lx=6cm
#define SOUND_THREESHOLD    10000.0f
//paramètre de la moyenne mobile : angle = a*angle +b*angle_buf
#define A					0.5f
#define B					0.5f

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
#define DROITE_FREQ_MIN		37		//563
#define DROITE_FREQ_MAX		47		//715
#define GAUCHE_FREQ_MIN		47		//715
#define GAUCHE_FREQ_MAX		57		//868
#define MAX_FREQ			130		//we don't analyze after this index to not use resources for nothing

#define MIN_VALUE_THRESHOLD		5000
#define MIN_INTENSITY_TRACKING	10000


/*comme la vitesse du régulateur s'ajoute à celle de l'idle (à savoir 7.5 cm/s) et que l'on souhaite pas dépasser la vitesse max
de 13 cm/s on définit une vitesse limite sur le regulateur que l'on pourra jamais dépasser. ceci évite ainsi que le régulateur
s'emballe avec des valeurs d'erreur qui sont trop élevées (surtout quand on voit les valeurs des capteurs)
Tu peux essayer avec plusieurs valeurs de KP et KI pour voir ce que ça donne*/



#define FREQ_STAB_IND				3

/*Traite les informations provenant des micro
 *
 */
void processAudioData(int16_t *data, uint16_t num_samples);

/*
*	put the invoking thread into sleep until it can process the audio datas
*/
void wait_traitement_data(void);

/*
*	Returns the pointer to the BUFFER_NAME_t buffer asked
*/
float* get_audio_buffer_ptr(BUFFER_NAME_t name);

void traitement_data(void);

float get_angle(void);

float get_freq(void);

#endif
