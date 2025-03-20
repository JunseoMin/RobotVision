#ifndef OPTIMA_H
#define OPTIMA_H

#include "kfc.h"
#include "optima.h"

#define  _OPTIMA_GOLD     1.618034
#define  _OPTIMA_CGOLD    0.3819660
#define  _OPTIMA_GLIMIT   100
#define  _OPTIMA_TINY     1.0e-20
#define  _OPTIMA_ZEPS     1.0e-10
#define  _OPTIMA_SHIFT(a,b,c,d) (a)=(b); (b)=(c);(c)=(d);

class KOptima
{
protected:
    int         _nMaxItr, _nMaxItr2;
    KVector    	_vP;          //starting point
    KVector    	_vDir; //direction of minimization
    KMatrix    	_mDir;

    KVector     _vHistoryErf;

public:
    KOptima(int nMaxItr=15,int nMaxItr2=30){ _nMaxItr = nMaxItr; _nMaxItr2 = nMaxItr2; }
    virtual ~KOptima(){ };

public:
    bool		Powell(KVector& vP,double dTol,int nItr=0){ return ExecuteWithoutGradients(vP,dTol,nItr); }
    bool		ExecuteWithoutGradients(KVector& vP,double dTol,int nItr=0); //Powell method
    //bool		ExecuteWithGradients(KVector& vP,double dTol,int nItr=0);    //Fletcher-Reeves and Polak-Ribiere

    KVector&     HistoryErf(){ return _vHistoryErf; }

protected:
    virtual double   Erf(const KVector& vX) = 0;
    virtual void		OnIterationFront(){ }
    virtual void		OnIterationEnd(){ }

    //minimization along a line
    double           LineMin();
    //from 10.1 of "Numerical Recipes in C"
    void             GoldenSection(double& dA,double& dB,double& dC,double& dFa,double& dFb,double& dFc);
    //from 10.2 of "Numerical Recipes in C"
    double           Brent(double dA,double dB,double dC,double dTol,double& dXmin);
    //one dimensional function to be minimized
    double           Fun1D(double);

};
/////////////////////////////////////////////////////////////////////////////////


#endif // OPTIMA_H
