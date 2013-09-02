/* stereo correspondence */

//#define SVS_MAPPING_BY_DEFAULT
//#define SVS_FOOTLINE
//#define SVS_ENABLE_MAPPING
#define SVS_MAX_FEATURES         2000
#define SVS_MAX_MATCHES          2000
#define SVS_MAX_IMAGE_WIDTH      1280
#define SVS_MAX_IMAGE_HEIGHT     1024
#define SVS_VERTICAL_SAMPLING    2
#define SVS_HORIZONTAL_SAMPLING  8
#define SVS_DESCRIPTOR_PIXELS    30
#define SVS_STEER_DEADBAND       2
//#define SVS_SHOW_STEERING
#define SVS_SECTOR               3
#define SVS_FILTER_SAMPLING      40

#define SVS_BASELINE_MM          107
#define SVS_FOCAL_LENGTH_MMx100  360
#define SVS_FOV_DEGREES          90

//#define SVS_VERBOSE
//#define SVS_PROFILE

/* multipliers used during calibration map update */
#define SVS_MULT                 1
#define SVS_MULT_COEFF           10000000

#define pixindex(xx, yy)  (((yy) * imgWidth + (xx)) * 2) & 0xFFFFFFFC  // always a multiple of 4
#define pixindex2(xx, yy)  ((yy) * imgWidth + (xx))

/* Two structures are created:
 * svs_data stores features obtained from this camera
 * svs_data_received stores features received from the opposite camera */
typedef struct {
        unsigned int head;  // use 0xAAAAAAAA
        
        /* number of detected features */
        unsigned int no_of_features;

        /* array storing x coordinates of detected features */
        short int feature_x[SVS_MAX_FEATURES];
        
        /* array storing the number of features detected on each row */
        unsigned short int features_per_row[SVS_MAX_IMAGE_HEIGHT/SVS_VERTICAL_SAMPLING];
    
        /* Array storing a binary descriptor, 32bits in length, for each detected feature.
         * This will be used for matching purposes.*/
        unsigned int descriptor[SVS_MAX_FEATURES];

        /* mean luminance for each feature */
        unsigned char mean[SVS_MAX_FEATURES];
        
        /* checksum */
        unsigned short crc;
        
        unsigned char received_bit;
        
        unsigned int tail;  // use 0x55555555
} svs_data_struct;

extern int svs_update_sums(int cols, int y, unsigned char* rectified_frame_buf, int segment);
extern void svs_non_max(int cols, int inhibition_radius, unsigned int min_response);
extern int svs_compute_descriptor(int px, int py, unsigned char* rectified_frame_buf, int no_of_features, int row_mean);
extern int svs_get_features_vertical(unsigned char* rectified_frame_buf, int inhibition_radius, unsigned int minimum_response, int calibration_offset_x, int calibration_offset_y, int segment);
extern int svs_get_features_horizontal(unsigned char* rectified_frame_buf, int inhibition_radius, unsigned int minimum_response, int calibration_offset_x, int calibration_offset_y, int segment);
void svs_send_features();
int svs_receive_features();
extern int svs_match(int ideal_no_of_matches, int max_disparity_percent, int learnDesc, int learnLuma, int learnDisp, int learnGrad, int groundPrior, int use_priors);

extern void svs_filter_plane(int no_of_possible_matches, int max_disparity_pixels);
extern void svs_rectify(unsigned char* raw_image, unsigned char* rectified_frame_buf);

void svs_master(unsigned short *outbuf16, unsigned short *inbuf16, int bufsize);
void svs_slave(unsigned short *inbuf16, unsigned short *outbuf16, int bufsize);
extern int svs_grab(int calibration_offset_x, int calibration_offset_y, int left_camera, int show_feats);
extern void init_svs();
extern void init_svs_matching_data();
extern int svs_read_calib_params();
void svs_make_map(long centre_of_distortion_x, long centre_of_distortion_y, long* coeff, int degree, long scale_num, long scale_denom);

extern void svs_show_features(unsigned char *outbuf, int no_of_feats);
extern void svs_show_matches(unsigned char *outbuf, int no_of_matches);
extern void svs_show_ground_plane(unsigned char *outbuf);

extern void svs_ground_plane();
#ifdef SVS_FOOTLINE
extern void svs_footline_slope();
extern void svs_show_footline(unsigned char *outbuf);
#endif
extern void svs_send_disparities (int no_of_matches);
extern void svs_stereo(int send_disparities);

extern void svs_footline_update(int max_disparity_percent);

/* mapping */

#ifdef SVS_ENABLE_MAPPING

#define MAP_WIDTH_CELLS      200
#define MAP_CELL_SIZE_MM     40
#define MAP_MIN_RANGE_MM     200
#define EMPTY_CELL           999999

extern void init_map();
extern void map_update(int no_of_matches, int max_disparity_percent, unsigned int* svs_matches, unsigned short int** footline);
extern void map_recenter();
extern void scan_match(int right_turn_pixels, int tollerance, svs_data_struct *svs_data, svs_data_struct *svs_data_previous);
extern void show_map(unsigned char* outbuf);
extern void update_pose(svs_data_struct *svs_data, svs_data_struct *svs_data_previous);

#endif

