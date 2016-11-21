#include <ml.h>
class Mysvm: public CvSVM
{
public:
    int get_alpha_count();
    int get_sv_dim();
    int get_sv_count();
    double* get_alpha();
    float** get_sv();
    float get_rho();
};
