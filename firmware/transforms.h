// clarke transform
void clarke(float a, float b, float c, float *alpha, float *beta);


// park transform

void park(float alpha, float beta, float *d, float *q, float theta);

// inverse park transform

void inv_park(float d, float q, float *alpha, float *beta, float theta);

// inverse clarke transform

void inv_clarke(float alpha, float beta, float *a, float *b, float *c);