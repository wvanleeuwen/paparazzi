#define PRINTF_SUPPORT
#define ROWS 48
#define COLUMNS 96
#define SS 9
#define SR (SS + (SS>>1) + (SS>>2) + 1)
#define WINWIDTH 48         //(2*SR+16)

short exp_vmv_4[4] = {0,-15,-15,15};
short exp_hmv_4[4] = {0,-15,-15,0};

short exp_vmv_9[4] = {0,-31,-31,31};
short exp_hmv_9[4] = {0,0,-31,0};

typedef struct
{
    short vmv[9];
    short hmv[9];
    short modifier[25];
} tss_struct;
typedef struct
{
    unsigned char *ptr_target;      // Target block address (16x16)
    unsigned char *ptr_reference;   // Reference window address
    int winwidth;                   // Width of the reference window
    int step_size;                  // Initial step size
    tss_struct *ptr_tss;            // Pointer to the initialized tss structure
    short mv_x;                     // Address of the horizontal MV
    short mv_y;                     // Address of the vertical MV
} tss_par;

segment("mydata1") unsigned char target[256];
segment("mydata2") unsigned char reference[5000],ref_window[48*48], *ptr;
segment("mydata2") tss_par tss_in_out;
segment("mydata2") tss_struct tss;

segment ("mydata1") short hhpel[9] = {0,-1,0,1,-1,1,-1,0,1};
segment ("mydata1") short vhpel[] = {0,-1,-1,-1,0,0,1,1,1};

