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

typedef struct complex_float{
	float real;
	float imag;
}complex_float;

typedef enum {
	MIC_RIGHT_I = 0,
	MIC_LEFT_I,
	MIC_BACK_I,
	MIC_FRONT_I
} INDEX_MICROPHONE;

/*define provenant de parcours.h, a supprimer quand on mergera les fichier
 *
 */
// pour une vitesse d'environ 7.5 cm/s qui correspond à l'idle (tant que pas d'obstacle, buck avance à une vitesse de 7.5cm/s)
#define VITESSE_IDLE 				577

//etapes pour definir la progression du contournement d'obstacles
#define MVT_IDLE 					0
#define MVT_CONTOURNEMENT_DROITE	1
#define MVT_CONTOURNEMENT_GAUCHE 	2
#define ANGLE_MARGIN				6.
/*comme la vitesse du régulateur s'ajoute à celle de l'idle (à savoir 7.5 cm/s) et que l'on souhaite pas dépasser la vitesse max
de 13 cm/s on définit une vitesse limite sur le regulateur que l'on pourra jamais dépasser. ceci évite ainsi que le régulateur
s'emballe avec des valeurs d'erreur qui sont trop élevées (surtout quand on voit les valeurs des capteurs)
Tu peux essayer avec plusieurs valeurs de KP et KI pour voir ce que ça donne*/

#define VITESSE_LIM					600
#define KP 							300
#define KI							10
#define MAX_SUM_ERROR 				VITESSE_LIM/8
#define MAX_ERROR 					VITESSE_LIM/3

#define COS_AVANT					1.0f
#define COS_ARRIERE					-1.0f
#define COS_MARGE					0.1f

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

#endif
