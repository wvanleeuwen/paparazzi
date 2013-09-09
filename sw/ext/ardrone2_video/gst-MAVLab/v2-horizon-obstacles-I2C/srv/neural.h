void nninit_network();
void nninit_pattern();
void nnset_pattern(int);
int f_logic(int);
void nncalculate_network();
void nncalculat_errors();
void nntrain_network(int);
void nndisplay(int);
void nnscale8x8(unsigned char *, unsigned int, unsigned int, unsigned int, unsigned int, unsigned int, unsigned int, unsigned int);
void nnpack8x8(int);

extern int weights[], neurons[], teach_neurons[], nerror[];
extern unsigned char npattern[], nmask[];

#define NUM_INPUT    64
#define NUM_OUTPUT   16
#define NUM_HIDDEN   16    
#define NUM_NPATTERNS 16

// For accessing weight from input 2 to hidden 3 use:
//    W_IN_HIDDEN(2,3)

#define W_IN_HIDDEN(i, h)  weights[i*NUM_HIDDEN + h]
#define W_HIDDEN_OUT(h, o) weights[NUM_INPUT*NUM_HIDDEN + h*NUM_OUTPUT + o]
#define N_IN(i)     neurons[i]
#define N_HIDDEN(h) neurons[NUM_INPUT + h]
#define N_OUT(o)    neurons[NUM_INPUT + NUM_HIDDEN + o]
#define N_TEACH(o)  teach_neurons[o]
#define E_HIDDEN(h) nerror[h]
#define E_OUT(o)    nerror[NUM_HIDDEN + o]


