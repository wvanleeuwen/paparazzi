/* malloc() functions derived from Kernigahan & Ritchie "The C Programming Language" (c) 1978 */

#include "srv.h"
#include "string.h"
#include "malloc.h"

char *heap_start, *heap_end, *heap_ptr;
HEADER base;
HEADER *allocp = 0;

#define NALLOC 128      // number of units to allocate at once

void init_heap() {
    int *ix;
    
    heap_start = (char *)HEAPSTART;
    heap_end = heap_start + HEAPSIZE;
    heap_ptr = heap_start;
    allocp = 0;
    for (ix=(int *)heap_start; ix<(int*)heap_end; ix++)
        *ix = 0;
}

char *sbrk(unsigned int size) {  // need to add check for head_ptr vs heap_end
    heap_ptr += size;
    return (heap_ptr-size);  // return pointer to beginning of newly allocated block
}

char *malloc(unsigned int nbytes) {
    HEADER *p, *q;
    int nunits;
    
    nunits = 1+(nbytes+sizeof(HEADER)-1)/sizeof(HEADER);
    if ((q = allocp) == 0) {            // no free list yet
        base.s.ptr = allocp = q = &base;
        base.s.size = 0;
    }
    for (p=q->s.ptr; ; q=p, p=p->s.ptr) {    
        if (p->s.size >= nunits) {      // big enough
            if (p->s.size == nunits)    // exactly
                q->s.ptr = p->s.ptr;
            else {                      // allocate tail end
                p->s.size -= nunits;
                p += p->s.size;
                p->s.size = nunits;
            }
            allocp = q;
            return((char *)(p+1));
        }
        if (p == allocp)                // wrapped around free list
            if ((p = morecore(nunits)) == 0)
                return(0);              // none left
    }
}

HEADER *morecore(unsigned int nu) {  // ask system for memory
    char *cp;
    HEADER *up;
    int rnu;
    
    rnu = NALLOC * ((nu+NALLOC-1) / NALLOC);
    cp = sbrk(rnu * sizeof(HEADER));
    if ((int)cp == -1)      // no space available
        return (0);
    up = (HEADER *)cp;
    up->s.size = rnu;
    free((char *)(up+1));
    return(allocp);
}

void free(char *ap) {       // put block ap in free list
    HEADER *p, *q;
    
    p = (HEADER *)ap - 1;   // point to header
    for (q=allocp; !(p > q && p < q->s.ptr); q=q->s.ptr)
        if (q >= q->s.ptr && (p > q || p < q->s.ptr))
            break;          // at one end of other
            
    if (p+p->s.size == q->s.ptr) {  // join to upper nbr
        p->s.size += q->s.ptr->s.size;
        p->s.ptr = q->s.ptr->s.ptr;
    } else
        p->s.ptr = q->s.ptr;
    if (q+q->s.size == p) {         // join to lower nbr
        q->s.size += p->s.size;
        q->s.ptr = p->s.ptr;
    } else
        q->s.ptr = p;
    allocp = q;
}

