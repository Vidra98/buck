#ifndef TRAITEMENT_SON
#define TRAITEMENT_SON

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
