#include "mySVM.h"
int Mysvm::get_alpha_count()
{
    return this->sv_total;
}
int Mysvm::get_sv_dim()
{
    return this->var_all;
}
int Mysvm::get_sv_count()
{
    return this->decision_func->sv_count;
}
double* Mysvm::get_alpha()
{
    return this->decision_func->alpha;
}
float** Mysvm::get_sv()
{
    return this->sv;
}
float Mysvm::get_rho()
{
    return this->decision_func->rho;
}
