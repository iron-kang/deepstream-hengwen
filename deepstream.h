#ifndef __DEEPSTREAM_H__
#define __DEEPSTREAM_H__

#define MAX_LANES 10

typedef struct line {
    int a;
    int b;
    int c;
} Line;

void deepstream_init(const char *config);
void deepstream_exit();

#endif
