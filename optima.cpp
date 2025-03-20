#include "optima.h"

//reference : 10.5 in  Nemerical Recipes in C
//functions : - Nonlinear Optimization by Powell's Method
//return value : true if success(<= the specified max iteration number)
//output : is overwritten on vP
//

bool KOptima::ExecuteWithoutGradients(KVector& vP,double dTol,int nItr)
{
    //starting point
    int nDim =  vP.Dim();
    _vP      =  vP;

    //initial matrix, each column is initial direction vector
    _mDir.Create(nDim,nDim,_IDENTITY);
    _vDir.Create(nDim);

    //history for return values of Erf(..)
    _vHistoryErf.Tailed(Erf(_vP));

    //Optimaization
    int      	nIdx;
    static int 	nID;
    double   	dDel,dFp,dFptt;
    double   	dMin = _vHistoryErf[0];
    KVector  	vPt(_vP);
    KVector  	vPtt(nDim);

    _nMaxItr = (nItr ? nItr : _nMaxItr);

    for(int i=0; i<_nMaxItr; i++)
    {
        //do something at every iteration if necessary
        OnIterationFront();

        //update the estimation vector
        dFp   = dMin;
        nIdx  = 0;
        dDel  = 0.0;

        for(int j=0; j<nDim; j++)
        {
            _mDir.Column(j,_vDir);
            dFptt  = dMin;
            dMin   = LineMin();

            if(_ABS(dFptt-dMin) > dDel){
                dDel = _ABS(dFptt-dMin);
                nIdx = j;
            }
        }

        //record history
        _vHistoryErf.Tailed(dMin);

        //check convergence - dTol : allowable relative error of Erf(--)
        if(2.0*_ABS(dFp-dMin) <= dTol*(_ABS(dFp)+_ABS(dMin)))
        {
            vP = _vP;
            _vHistoryErf.WriteText(KString::Format("errorHistory%02d.txt",nID).Address());

            return true;
        }

        //comput extrapolation point
        vPtt = _vP*2.0 - vPt;
        //compute everage direction vection
        _vDir = _vP - vPt;
        //store the starting point
        vPt  = _vP;
        //function value at the extrapolation point
        dFptt= Erf(vPtt);

        if(dFptt < dFp){
            double dTmp = 2.0*(dFp-2.0*dMin+dFptt)+_SQR(dFp-dMin-dDel)-dDel*_SQR(dFp-dFptt);
            if(dTmp < 0.0){
                dMin = LineMin();
                for(int j=0; j<nDim; j++){
                    _mDir[j][nIdx]   = _mDir[j][nDim-1];
                    _mDir[j][nDim-1] = _vDir[j];
                }
            }
        }

        //do something at every iteration if necessary
        OnIterationEnd();
    }

    vP = _vP;

    _vHistoryErf.Tailed(Erf(vP));
    _vHistoryErf.WriteText(KString::Format("errorHistory%02d.txt",nID).Address());

    return false;
}
//---------------------------------------------------------------------------

double KOptima::LineMin()
{
    //golden section search
    double dA=0.0,dX=1.0,dB,dFa,dFx,dFb;
    GoldenSection(dA,dX,dB,dFa,dFx,dFb);

    //apply Brent method
    double dXmin;
    double dMin = Brent(dA,dX,dB,2.0e-4,dXmin);

    //set results
    _vDir *= dXmin;
    _vP   += _vDir;

    return dMin;
}
//---------------------------------------------------------------------------


double KOptima::Fun1D(double dX)
{
    return Erf(_vP + _vDir*dX);
}

void KOptima::GoldenSection(double& dA,double& dB,double& dC,double& dFa,double& dFb,double& dFc)
{
    //to make dFb lower than dFa
    double dDummy;

    dFa = Fun1D(dA);
    dFb = Fun1D(dB);

    if(dFb > dFa){
        _OPTIMA_SHIFT(dDummy,dA,dB,dDummy)
        _OPTIMA_SHIFT(dDummy,dFa,dFb,dDummy)
    }

    //init. dC
    dC    = dB+_OPTIMA_GOLD*(dB-dA);
    dFc   = Fun1D(dC);

    //iterate
    double dU,dFu,dUlim,dR,dQ;

    while(dFb > dFc){
        //inverse parabolic interpoloation
        dR = (dB-dA)*(dFb-dFc);
        dQ = (dB-dC)*(dFb-dFa);
        dU = dB - ((dB-dC)*dQ-(dB-dA)*dR)/(2.*_SIGN(dQ-dR)*_MAX(_ABS(dQ-dR),_OPTIMA_TINY));
        dUlim = dB+_OPTIMA_GLIMIT*(dC-dB);

        //the case that dU is between dB and dC
        if((dB-dU)*(dU-dC) > 0.0){
            dFu = Fun1D(dU);

            if(dFu < dFc){
                dA = dB;
                dB = dU;
                dFa= dFb;
                dFb= dFu; return;
            }else if(dFu > dFc){
                dC = dU;
                dFc= dFu; return;
            }

            //in the case that inverse parabolic inter. is useless
            dU = dC+_OPTIMA_GOLD*(dC-dB);
            dFu= Fun1D(dU);
        }
        //the case that dU is between dUlim and dC
        else if((dC-dU)*(dU-dUlim)>0.0){
            dFu = Fun1D(dU);
            if(dFu < dFc){
                _OPTIMA_SHIFT(dB,dC,dU,(dC+_OPTIMA_GOLD*(dC-dB)))
                _OPTIMA_SHIFT(dFb,dFc,dFu,Fun1D(dU))
            }
        }
        //
        else if((dU-dUlim)*(dUlim-dC) >= 0.0){
            dU = dUlim;
            dFu= Fun1D(dU);
        }
        else{
            dU = dC+_OPTIMA_GOLD*(dC-dB);
            dFu= Fun1D(dU);
        }
        _OPTIMA_SHIFT(dA,dB,dC,dU)
        _OPTIMA_SHIFT(dFa,dFb,dFc,dFu)
    }
}
//---------------------------------------------------------------------------

double KOptima::Brent(double dA,double dB,double dC,double dTol,double& dXmin)
{
    //to make dAA < dBB
    double dAA,dBB;

    dAA = (dA<dC ? dA:dC);
    dBB = (dA>dC ? dA:dC);

    //init.
    double dX,dW,dV,dU,dFu,dFx,dFw,dFv;
    dX = dW = dV = dB;
    dFw= dFv= dFx= Fun1D(dX);

    //iterate
    double dP,dQ,dR,dD=0.0;
    double dXm,dTol1,dTol2,dE=0.0,dEtmp;

    for(int i=0; i<_nMaxItr2; i++){
        dXm   = 0.5*(dAA+dBB);
        dTol2 = 2.0*(dTol1=dTol*fabs(dX)+_OPTIMA_ZEPS);

        if(fabs(dX-dXm) <= (dTol2-0.5*(dBB-dAA))){
            dXmin = dX;
            return dFx;
        }
        if(fabs(dE)>dTol1){
            dR = (dX-dW)*(dFx-dFv);
            dQ = (dX-dV)*(dFx-dFw);
            dP = (dX-dV)*dQ - (dX-dW)*dR;
            dQ = 2.0*(dQ-dR);

            if(dQ>0.0)
                dP = -dP;
            dQ = fabs(dQ);
            dEtmp = dE;
            dE    = dD;

            if(fabs(dP)>= fabs(0.5*dQ*dEtmp) || dP<=dQ*(dAA-dX) || dP>=dQ*(dBB-dX))
                dD = _OPTIMA_CGOLD*(dE=(dX>=dXm ? dAA-dX:dBB-dX));
            else{
                dD = dP/dQ;
                dU = dX+dD;
                if(dU-dAA < dTol2 || dBB-dU<dTol2)
                    dD = _SIGN(dXm-dX)*dTol1;
            }
        }
        else
            dD = _OPTIMA_CGOLD*(dE=(dX>=dXm ? dAA-dX : dBB-dX));

        dU = (_ABS(dD) >= dTol1 ? dX+dD : dX+_SIGN(dD)*dTol1);
        dFu= Fun1D(dU);

        if(dFu <= dFx){
            if(dU>=dX)
                dAA = dX;
            else
                dBB = dX;
            _OPTIMA_SHIFT(dV,dW,dX,dU)
            _OPTIMA_SHIFT(dFv,dFw,dFx,dFu)
        }
        else{
            if(dU<dX)
                dAA = dU;
            else
                dBB = dU;
            if(dFu <= dFw || dW == dX){
                dV = dW;  dW = dU;
                dFv= dFw; dFw = dFu;
            }
            else if(dFu <= dFv || dV==dX || dV==dW){
                dV = dU;
                dFv= dFu;
            }
        }
    }

    //warning !!!
    dXmin = dX;
    return dFx;
}
//---------------------------------------------------------------------------

