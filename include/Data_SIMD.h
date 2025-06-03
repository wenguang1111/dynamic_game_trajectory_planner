#define BLOCK_SIZE 4 
#define NUM_State 21

struct Block_U
{
    float d[BLOCK_SIZE];
    float F[BLOCK_SIZE];
};

struct Block_X
{
    float x[BLOCK_SIZE];
    float y[BLOCK_SIZE];
    float psi[BLOCK_SIZE];
    float v[BLOCK_SIZE];
    float s[BLOCK_SIZE];
    float l[BLOCK_SIZE];
};
