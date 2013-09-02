char *malloc(unsigned int);
void free(char *);
void init_heap();

typedef int ALIGN;
union header {              // free block header
    struct {
        union header *ptr;  // next free block
        unsigned int size;  // size of this free block
    } s;
    ALIGN x;                // forces alignment of blocks
};

typedef union header HEADER;
extern char *heap_start, *heap_ptr, *heap_end;
extern HEADER base;         // pointer to beginning of free block list
extern HEADER *allocp;      // last allocated block
HEADER *morecore(unsigned int);
