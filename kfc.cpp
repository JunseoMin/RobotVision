﻿//---------------------------------------------------------------------------
#include "kfc.h"

using namespace std;

bool KPoint::Read(FILE* fp)
{
	if(fread(&_dX,sizeof(double),1,fp)==1 && fread(&_dY,sizeof(double),1,fp)==1)
		return true;
	return false;
}

int KPoint::Write(FILE* fp)
{
	int nBytes;

    nBytes  = sizeof(double)*fwrite(&_dX,sizeof(double),1,fp);                   
	nBytes += sizeof(double)*fwrite(&_dY,sizeof(double),1,fp);

	return nBytes;
}                                               

void KPoint::Rotated(const double& dRad,const double& dXo,const double& dYo)
{
    double dCos = cos(dRad);
    double dSin = sin(dRad);
    double dXn  = dCos*(_dX-dXo) - dSin*(_dY-dYo) + dXo;
    double dYn  = dSin*(_dX-dXo) + dCos*(_dY-dYo) + dYo;

    _dX = dXn;
    _dY = dYn;
}

void KPoint::Rotated(const double& dRad,const KPoint& ptO)
{
    double dCos = cos(dRad);
    double dSin = sin(dRad);
    double dXn  = dCos*(_dX-ptO._dX) - dSin*(_dY-ptO._dY) + ptO._dX;
    double dYn  = dSin*(_dX-ptO._dX) + dCos*(_dY-ptO._dY) + ptO._dY;

    _dX = dXn;
    _dY = dYn;
}

KPoint KPoint::Rotate(const double& dRad,const KPoint& ptO)
{
    double dCos = cos(dRad);
    double dSin = sin(dRad);
    double dXn  = dCos*(_dX-ptO._dX) - dSin*(_dY-ptO._dY) + ptO._dX;
    double dYn  = dSin*(_dX-ptO._dX) + dCos*(_dY-ptO._dY) + ptO._dY;

    return KPoint(dXn,dYn);
}

bool KPoint::InRegion(const KPoints& psR, const int& nType) //원점에서 +z 방향으로 보는 것을 기준으로 함
{																 //점은 nType(CW, CCW)으로 정렬되어 있음		
	double dArea;

    for(unsigned int i=0,j; i<psR.size(); i++)
	{		
        j = (i+1)%psR.size();
		
		dArea = psR[i]._dX*psR[j]._dY + psR[j]._dX*_dY + _dX*psR[i]._dY - psR[j]._dX*psR[i]._dY - _dX*psR[j]._dY - psR[i]._dX*_dY;

		if(nType == _CW){
			if(dArea < 0.0)
				return false;
		}
		else if(dArea > 0.0) //_CCW
			return false;		
	}

	return true;
}

KPoint& KPoint::operator +=(const KPoint& pt)
{
    _dX += pt._dX;
    _dY += pt._dY;

    return *this;
}

KPoint& KPoint::operator +=(const KVector& v)
{
#ifdef _DEBUG
	assert(v.Address() != 0 && v.Dim() == 2);
#endif

    _dX += v[0];
    _dY += v[1];

    return *this;
}

KPoint& KPoint::operator *=(const double& d)
{
    _dX *= d;
    _dY *= d;    

    return *this;
}

KPoint& KPoint::operator -=(const KPoint& pt)
{
    _dX -= pt._dX;
    _dY -= pt._dY;

    return *this;
}

KPoint& KPoint::operator =(const KPoint& pt)
{
    _dX = pt._dX;
    _dY = pt._dY;

    return *this;
}

KPoint& KPoint::operator =(const double& d)
{
    _dX = _dY = d;

    return *this;
}

KPoint KPoint::operator +(const KPoint& pt) const
{
    KPoint ptOut;
    ptOut._dX = _dX + pt._dX;
    ptOut._dY = _dY + pt._dY;

	return ptOut;
}

KPoint KPoint::operator *(const double& d) const
{
    KPoint ptOut;
    ptOut._dX = _dX*d;
    ptOut._dY = _dY*d;

    return ptOut;
}

KPoint KPoint::operator /(const double& d) const
{
    KPoint ptOut;
    ptOut._dX = _dX/d;
    ptOut._dY = _dY/d;

    return ptOut;
}
KPoint KPoint::operator -(const KPoint& pt) const
{
    KPoint ptOut;
    ptOut._dX = _dX - pt._dX;
    ptOut._dY = _dY - pt._dY;

    return ptOut;
}

KPoint& KPoint::operator =(const KVector& v)
{
    _dX = v[0];
    _dY = v[1];

	return *this;
}

KPoint KPoint::operator +(const KVector& v) const
{
    KPoint ptOut;
    ptOut._dX = _dX + v[0];
    ptOut._dY = _dY + v[1];

	return ptOut;
}

KPoint KPoint::operator -(const KVector& v) const
{
    KPoint ptOut;
	ptOut._dX = _dX - v[0];
    ptOut._dY = _dY - v[1];

    return ptOut;
}

KPoint KPoint::operator -() const
{
	return KPoint(-_dX,-_dY);
}
//-----------------------------------------------------------------------------------------------------------------------------


KRect KPoints::Rect() const
{
    KRect rcOut(this->front(),this->front());

    for(auto& item : *this)
    {
        rcOut._nLeft   = _MIN(rcOut._nLeft,  item._dX);
        rcOut._nTop    = _MIN(rcOut._nTop,   item._dY);
        rcOut._nRight  = _MAX(rcOut._nRight, item._dX);
        rcOut._nBottom = _MAX(rcOut._nBottom,item._dY);
    }

    return rcOut;
}

KPoints KPoints::Transform(const KMatrix& mH)
{    
    KPoints     psOut;
    KVector     vUV;    

    for(auto& point : *this)
    {
        vUV = mH * KVector(point._dX, point._dY, 1.0);
        if(vUV[2] == 0.0)
            psOut.push_back(KPoint(0.0, 0.0));
        else
            psOut.push_back(KPoint(vUV[0]/vUV[2],vUV[1]/vUV[2]));
    }

    return psOut;
}

void KPoints::Transformed(const KMatrix& mH)
{
    KVector     vUV;

    for(auto& point : *this)
    {
        vUV = mH * KVector(point._dX, point._dY, 1.0);
        if(vUV[2] == 0.0)
            point = 0.0;
        else
        {
            point._dX = vUV[0]/vUV[2];
            point._dY = vUV[1]/vUV[2];
        }
    }

}

KPoints::KPoints(const KPoints &psInput)
{    
    this->resize(psInput.size());
    copy(psInput.begin(), psInput.end(), this->begin());
}

KPoints& KPoints::operator =  ( const KPoints& psInput)
{
    this->resize(psInput.size());
    copy(psInput.begin(), psInput.end(), this->begin());
	
	return *this;
}
//-----------------------------------------------------------------------------------------------------------------------------

bool KPoint3D::Read(FILE* fp)
{
    if(fread(&_dX,sizeof(double),1,fp)==1 && fread(&_dY,sizeof(double),1,fp)==1 && fread(&_dZ,sizeof(double),1,fp)==1)
        return true;
    return false;
}

void KPoint3D::Write(FILE* fp)
{
	fwrite(&_dX,sizeof(double),1,fp);
	fwrite(&_dY,sizeof(double),1,fp);
	fwrite(&_dZ,sizeof(double),1,fp);
}

KPoint3D& KPoint3D::operator =(const KVector& v)
{
    _dX = v[0];
    _dY = v[1];
    _dZ = v[2];

    return *this;
}

KPoint3D& KPoint3D::operator =(const KPoint3D& pt)
{
	_dX = pt._dX;
	_dY = pt._dY;
	_dZ = pt._dZ;

	return *this;
}

KPoint3D& KPoint3D::operator =(const double& d)
{
    _dX = _dY = _dZ = d;

    return *this;
}


KPoint3D KPoint3D::operator +(const KVector& v)
{
    KPoint3D ptOut;
    ptOut._dX = _dX + v[0];
    ptOut._dY = _dY + v[1];
    ptOut._dZ = _dZ + v[2];

    return ptOut;
}

KPoint3D KPoint3D::operator -(const KVector& v)
{
    KPoint3D ptOut;
    ptOut._dX = _dX - v[0];
    ptOut._dY = _dY - v[1];
    ptOut._dZ = _dZ - v[2];

    return ptOut;
}
//---------------------------------------------------------------------------


bool KRect::Intersect(const KRect& rc, KRect* rcpInt) const
{
    if(this->Width() + rc.Width() > (_MAX(this->_nRight, rc._nRight) -  _MIN(this->_nLeft, rc._nLeft) + 1) &&
       this->Height() + rc.Height() > (_MAX(this->_nBottom, rc._nBottom) -  _MIN(this->_nTop, rc._nTop) + 1)) //겹치면
    {
        if(rcpInt)
        {
            rcpInt->_nLeft  = _MAX(this->_nLeft, rc._nLeft);
            rcpInt->_nRight = _MIN(this->_nRight, rc._nRight);
            rcpInt->_nTop   = _MAX(this->_nTop, rc._nTop);
            rcpInt->_nBottom= _MIN(this->_nBottom, rc._nBottom);
        }
        return true;
    }

    return false;
}

KRect& KRect::operator =(const KRect& rc)
{
	_nLeft=rc._nLeft;   _nTop=rc._nTop;
	_nRight=rc._nRight; _nBottom=rc._nBottom;

	return *this;
}

KRect& KRect::operator +=(const KPoint& pt)
{
   _nLeft   += (int)(pt._dX);
   _nRight  += (int)(pt._dX);
   _nTop    += (int)(pt._dY);
   _nBottom += (int)(pt._dY);

   return *this;
}

KRect& KRect::operator -=(const KPoint& pt)
{
   _nLeft   -= (int)(pt._dX);
   _nRight  -= (int)(pt._dX);
   _nTop    -= (int)(pt._dY);
   _nBottom -= (int)(pt._dY);

   return *this;
}

KRect KRect::Transform(const double& dRatio,const int& nType, bool bSquare) const
{
	KRect rcOut(*this);

	if(nType == _ORIGIN)
	{
		rcOut._nLeft   *= dRatio;
		rcOut._nRight  *= dRatio;
		rcOut._nTop    *= dRatio;
		rcOut._nBottom *= dRatio;

		return rcOut;
	}
	else if(dRatio>0)
	{
		if(nType == _COL)
            rcOut._nRight  = (int)(_nLeft + (int)(Width()*dRatio +0.5) - 1);
		else if(nType == _ROW)
            rcOut._nBottom = (int)(_nTop  + (int)(Width()*dRatio +0.5) - 1);
		else if(nType == _BOTH){
            rcOut._nRight  = (int)(_nLeft + (int)(Width()*dRatio +0.5) - 1);
            rcOut._nBottom = (int)(_nTop  + (int)(Height()*dRatio+0.5) - 1);
		}
		else if(nType == _CENTER){
            int nT = (int)(Width()*dRatio + 0.5);
            rcOut._nLeft   += (Width()-nT)/2;
            rcOut._nRight   = rcOut._nLeft + nT - 1;
            nT              = (int)(Height()*dRatio + 0.5);
            rcOut._nTop    += (Height()-nT)/2;
            rcOut._nBottom  = rcOut._nTop + nT - 1;

            if(bSquare)
            {
                double dS = (double)(rcOut.Height()) / (double)(rcOut.Width());
                if(dS > 1.0) //높이가 더 큰 경우
                {
                    int nT = (int)(Width()*dS + 0.5);
                    rcOut._nLeft   += (Width()-nT)/2;
                    rcOut._nRight   = rcOut._nLeft + rcOut.Height() - 1;
                }
                else        //폭이 더 큰 경우
                {
                    int nT = (int)(Height()/dS + 0.5);
                    rcOut._nTop   += (Height()-nT)/2;
                    rcOut._nBottom = rcOut._nTop + rcOut.Width() - 1;
                }
            }

		}
	}
	else
	{
		if(nType == _COL)
            rcOut._nLeft= (int)(_nRight + (int)(Width()*dRatio +0.5));
		else if(nType == _ROW)
            rcOut._nTop = (int)(_nBottom+ (int)(Height()*dRatio+0.5));
		else if(nType == _BOTH){
            rcOut._nLeft= (int)(_nRight + (int)(Width()*dRatio +0.5));
            rcOut._nTop = (int)(_nBottom+ (int)(Height()*dRatio+0.5));
		}
	}
	return rcOut;
}
//---------------------------------------------------------------------------


KRect& KRect::Transformed(const double& dRatio,const int& nType)
{
	if(nType == _ORIGIN)
	{
		_nLeft   *= dRatio;
		_nRight  *= dRatio;
		_nTop    *= dRatio;
		_nBottom *= dRatio;
	}
	else if(dRatio>0)
	{
		if(nType == _COL)
			_nRight  = (int)(_nLeft + (double)(_nRight-_nLeft+1)*dRatio + 0.5);
		else if(nType == _ROW)
			_nBottom = (int)(_nTop  + (double)(_nBottom-_nTop+1)*dRatio + 0.5);
		else if(nType == _BOTH){
			_nRight  = (int)(_nLeft + (double)(_nRight-_nLeft+1)*dRatio + 0.5);
			_nBottom = (int)(_nTop  + (double)(_nBottom-_nTop+1)*dRatio + 0.5);
		}
		else if(nType == _CENTER)
		{
			int nWidth, nHeight;
			nWidth      = (int)((double)(_nRight-_nLeft + 1)*dRatio + 0.5);
			_nLeft     += (_nRight-_nLeft + 1 - nWidth)/2;
			_nRight     = _nLeft + nWidth - 1;
			nHeight     = (int)((double)(_nBottom-_nTop + 1)*dRatio + 0.5);
			_nTop      += (_nBottom-_nTop + 1 - nHeight)/2;
			_nBottom    = _nTop + nHeight - 1;
		}
	}
	else
	{
		if(nType == _COL)
			_nLeft= (int)(_nRight + (double)(_nRight-_nLeft+1)*dRatio + 0.5);
		else if(nType == _ROW)
			_nTop = (int)(_nBottom+ (double)(_nBottom-_nTop+1)*dRatio + 0.5);
		else if(nType == _BOTH)
		{
			_nLeft= (int)(_nRight + (double)(_nRight-_nLeft+1)*dRatio + 0.5);
			_nTop = (int)(_nBottom+ (double)(_nBottom-_nTop+1)*dRatio + 0.5);
		}
	}

	return *this;
}
//---------------------------------------------------------------------------

KRectD& KRectD::Transformed(const double& dRatio, const int& nType)
{
	if (nType == _ORIGIN)
	{
		_dLeft *= dRatio;
		_dRight *= dRatio;
		_dTop *= dRatio;
		_dBottom *= dRatio;
	}
	else if (dRatio>0)
	{
		if (nType == _COL)
			_dRight = _dLeft + (_dRight - _dLeft)*dRatio;
		else if (nType == _ROW)
			_dBottom = _dTop + (_dBottom - _dTop)*dRatio;
		else if (nType == _BOTH){
			_dRight = _dLeft + (_dRight - _dLeft)*dRatio;
			_dBottom = _dTop + (_dBottom - _dTop)*dRatio;
		}
		else if (nType == _CENTER)
		{
			double dWidth, dHeight;
			dWidth = (_dRight - _dLeft)*dRatio;
			_dLeft += (_dRight - _dLeft - dWidth)* 0.5;
			_dRight = _dLeft + dWidth;
			dHeight = (_dBottom - _dTop)*dRatio;
			_dTop += (_dBottom - _dTop - dHeight)*0.5;
			_dBottom = _dTop + dHeight;
		}
	}
	else
	{
		if (nType == _COL)
			_dLeft = _dRight + (_dRight - _dLeft)*dRatio;
		else if (nType == _ROW)
			_dTop = _dBottom + (_dBottom - _dTop)*dRatio;
		else if (nType == _BOTH)
		{
			_dLeft = _dRight + (_dRight - _dLeft)*dRatio;
			_dTop = _dBottom + (_dBottom - _dTop)*dRatio;
		}
	}

	return *this;
}


KCircle& KCircle::operator =(const KCircle& cc)

{
	_dCx 		= cc._dCx;
	_dCy 		= cc._dCy;
	_dRadius 	= cc._dRadius;

	return *this;
}

KCircle& KCircle::Transformed(const double& dR,int nType)
{
	if(nType == _ORIGIN)
    {
		_dCx 		*= dR;
    	_dCy 		*= dR;
	    _dRadius	*= dR;
	}
    else if(nType == _CENTER)
    	_dRadius	*= dR;

    return *this;
}


KCircle KCircle::Transform(const double& dR,int nType)
{
	KCircle oOut;
    
	if(nType == _ORIGIN)
    {
		oOut._dCx 		= _dCx*dR;
    	oOut._dCy 		= _dCy*dR;
	    oOut._dRadius	= _dRadius*dR;
	}
    else if(nType == _CENTER)
    {
		oOut._dCx 		= _dCx;
    	oOut._dCy 		= _dCy;
        oOut._dRadius	= _dRadius*dR;
	}

    return oOut;
}
//---------------------------------------------------------------------------

void KString::Create(const char* szIn,const int& nSize)
{
	if(_szA)
    {
		delete[] _szA;
        _szA = 0;
    }

	_nSize = (nSize != 0 ? nSize : (szIn ? strlen(szIn) : 0));

	if(_nSize)
	{
		_szA   = new char[_nSize + 1];
		if(szIn)
		{
			memcpy(_szA,szIn,_nSize);
			_szA[_nSize] = 0;
		}
		else
			memset(_szA,0,_nSize+1);
	}
}


KString& KString::Formatted(const char* szFormat, ...)
{
	int     nLength;
	char    szBuf[_MAX_STRING_LENGTH+1];
	va_list args;

	va_start(args, szFormat);
	nLength = vsprintf(szBuf, szFormat, args);
	va_end(args);

	if(nLength != EOF)
		Create(szBuf);

	return *this;
}

void KString::Parscing(const char* szFormat, ...)
{
    int     nLength;
    va_list args;

    va_start(args, szFormat);
    nLength = vsscanf(_szA, szFormat, args);
    va_end(args);
}



KString KString::Format(const char* szFormat, ...)
{
    int     nLength;
    char    szBuf[_MAX_STRING_LENGTH+1];
    KString stOut;
    va_list args;

    va_start(args, szFormat);
    nLength = vsprintf(szBuf, szFormat, args);
    va_end(args);

    if(nLength != EOF)
        stOut.Create(szBuf);

    return stOut;
}

string KString::FormatStd(const char* szFormat, ...)
{
    int     nLength;
    char    szBuf[_MAX_STRING_LENGTH+1];
    string  stOut;
    va_list args;

    va_start(args, szFormat);
    nLength = vsprintf(szBuf, szFormat, args);
    va_end(args);

    if(nLength != EOF)
        stOut = szBuf;

    return stOut;
}

bool KString::Write(FILE* fp,const unsigned int& nSize)
{
	unsigned int nLength = (nSize ? nSize : strlen(_szA)+1);
	return (fwrite(_szA,sizeof(char),nLength,fp) == nLength);
}


bool KString::Read(FILE* fp,const unsigned int& nSize)       
{

#ifdef _DEBUG                                                
	assert(!(nSize > _MAX_STRING_LENGTH));                   
#endif
 
	//기존 스트링 삭제
	Release();
		
	//읽어들이기
	int  i=0;   
	char p[_MAX_STRING_LENGTH+1];

	if(nSize)
	{        
		if(fread(p,sizeof(char),nSize,fp) != nSize)
			return false;                          
		p[nSize] = 0;                              
	}                                              
	else                                           
		do{                                        
			if(fread(&p[i],sizeof(char),1,fp) != 1)
				return false;                      
		}while(p[i++] != 0);                       

	*this =  p;

	return true;
}


KString& KString::operator += (const char& cIn)
{
	char* szNew = new char[_nSize + 2];

	if(_nSize)
	{
		strcpy(szNew,_szA);
		delete[] _szA;
	}

	szNew[_nSize]   = cIn;
	szNew[_nSize+1] = 0;
	_szA            = szNew;
	_nSize ++;

	return *this;

}

KString operator + (const char* szIn, const KString& stIn)
{
	int     nSize = strlen(szIn) + stIn.Size();
	char*   szNew = new char[nSize+1];
	KString stOut;

	strcpy(szNew,szIn);
	strcat(szNew,stIn.Address());
	stOut._nSize = nSize;
	stOut._szA   = szNew;

	return stOut;
}
//---------------------------------------------------------------------------

KString KString::operator + (const char* szIn)
{
	int     nSize = _nSize + strlen(szIn);
	KString stOut;

	if(_szA == 0)
		stOut.Create(szIn);
	else if(nSize)
	{
		char* szNew = new char[nSize + 1];
		strcpy(szNew,_szA);
		strcat(szNew,szIn);

		stOut._nSize = nSize;
		stOut._szA   = szNew;
	}

	return stOut;

}


KString& KString::operator += (const char* szIn)
{
	int nSize = _nSize + strlen(szIn);
	if(_szA == 0)
		Create(szIn);
	else if(nSize)
	{
		char* szNew = new char[nSize + 1];
		strcpy(szNew,_szA);
		strcat(szNew,szIn);

		delete[] _szA;
		_szA   = szNew;
		_nSize = nSize;
	}

	return *this;
}

KString& KString::operator = (const char* szIn)
{
	if(_szA)
		delete[] _szA;
	_szA = 0;

	_nSize  = strlen(szIn);
	if(_nSize)
	{
		_szA = new char[_nSize+1];
		strcpy(_szA,szIn);
	}

	return *this;
}

KString KString::operator + (const KString& stIn)
{
	int     nSize = _nSize + stIn._nSize;
    KString stOut;

    if(nSize)
	{
		char* szNew = new char[nSize + 1];

        if(_szA)
            strcpy(szNew,_szA);
        if(stIn._nSize)
            strcat(szNew,stIn._szA);

		stOut._nSize = nSize;
		stOut._szA   = szNew;
	}


	return stOut;
}


KString& KString::operator += (const KString& stIn)
{
	int nSize = _nSize + stIn._nSize;


	if(_szA == 0)
		Create(stIn._szA);
	else if(nSize)
	{
		char* szNew = new char[nSize + 1];
		strcpy(szNew,_szA);
		strcat(szNew,stIn._szA);

		delete[] _szA;
		_szA   = szNew;
		_nSize = nSize;
	}

	return *this;
}

KString& KString::operator = (const KString& stIn)
{
	if(_szA)
		delete[] _szA;
	_szA = 0;

	_nSize  = stIn._nSize;
	if(_nSize)
	{
		_szA = new char[_nSize+1];
		strcpy(_szA,stIn._szA);
	}

	return *this;
}

bool KString::operator == (const KString& stIn)
{
	return ( _szA ? (strcmp(_szA,stIn._szA) ? false : true) : (stIn._szA ? false : true) );
}

bool KString::operator == (const char* szIn)
{
	return ( _szA ? (strcmp(_szA,szIn) ? false : true) : (szIn ? false : true) );
}


bool KString::operator != (const KString& stIn)
{
	return !( _szA ? (strcmp(_szA,stIn._szA) ? false : true) : (stIn._szA ? false : true) );
}

bool KString::operator != (const char* szIn)
{
	return !( _szA ? (strcmp(_szA,szIn) ? false : true) : (szIn ? false : true) );
}

int KString::Find(const KString& stSub, int nType) const
{
    if(nType == _LEFT)
    {
        for(int i=0,ic=_nSize; ic; ic--,i++)
            if(strncmp(&_szA[i],stSub._szA,stSub._nSize) == 0)
                return i;
    }
    else
    {
        for(int i=_nSize-1; i>=0; i--)
            if(strncmp(&_szA[i],stSub._szA,stSub._nSize) == 0)
                return i;
    }

	return -1;
}

int KString::Find(const char* szSub, int nType) const
{
	int nLen = strlen(szSub);

    if(nType == _LEFT)
    {
        for(int i=0,ic=_nSize; ic; ic--,i++)
            if(strncmp(&_szA[i],szSub,nLen) == 0)
                return i;
    }
    else
    {
        for(int i=_nSize-1; i>=0; i--)
            if(strncmp(&_szA[i],szSub,nLen) == 0)
                return i;
    }
	return -1;
}

KString KString::Remove(int nS, int nE) const
{
    KString stOut;

    if(!(nS<0 || nE+1 >_nSize || nS > nE))
    {
        int   nCut  = nE - nS + 1;
        char* szNew = new char[_nSize - nCut + 1];

        memcpy(szNew,_szA,nS);

        for(int i=nE+1, ii=0; i<=_nSize; i++, ii++)
            szNew[nS + ii] = _szA[i];

        stOut.Create(szNew);
    }

    return stOut;
}

KString KString::RemoveFrom(int nS) const
{
    KString stOut;
    int     nE = _nSize -1;

    if(!(nS<0 || nE+1 >_nSize || nS > nE))
    {
        int   nCut  = nE - nS + 1;
        char* szNew = new char[_nSize - nCut + 1];

        memcpy(szNew,_szA,nS);

        for(int i=nE+1, ii=0; i<=_nSize; i++, ii++)
            szNew[nS + ii] = _szA[i];

        stOut.Create(szNew);
    }

    return stOut;
}

KString KString::RemoveFrom(const char* szSub, int nType) const
{
    KString stOut;
    int     nS = Find(szSub,nType);
    int     nE = _nSize -1;

    if(!(nS<0 || nE+1 >_nSize || nS > nE))
    {
        int   nCut  = nE - nS + 1;
        char* szNew = new char[_nSize - nCut + 1];

        memcpy(szNew,_szA,nS);

        for(int i=nE+1, ii=0; i<=_nSize; i++, ii++)
            szNew[nS + ii] = _szA[i];

        stOut.Create(szNew);
    }

    return stOut;
}

KString& KString::Removed(int nS, int nE)
{
    if(!(nS<0 || nE+1 >_nSize || nS > nE))
    {
        int   nCut  = nE - nS + 1;
        char* szNew = new char[_nSize - nCut + 1];

        memcpy(szNew,_szA,nS);

        for(int i=nE+1, ii=0; i<=_nSize; i++, ii++)
            szNew[nS + ii] = _szA[i];

        delete[] _szA;
        _szA    = szNew;
        _nSize -= nCut;
    }

    return *this;
}


KString KString::Remove(const char* szCut) const
{
    int nIdx = Find(szCut);

    if(nIdx != -1)
        return Remove(nIdx, nIdx + strlen(szCut) - 1);

    return *this;
}

KString& KString::Removed(const char* szCut)
{
    int nIdx = Find(szCut);

    if(nIdx != -1)
        Removed(nIdx, nIdx + strlen(szCut) - 1);

    return *this;
}

KString KString::ParentDir() const
{
	//set a model file name
	int nSf=0;
	for(int i=_nSize-1; i; i--)
	{
		if(_szA[i] == '\\' || _szA[i] == '/')
		{
			nSf = i;
			break;
		}
	}

	return (nSf ? KString(_szA,0,nSf-1) : KString());
}

KString KString::FilePathOnly() const
{
#ifdef _DEBUG
	assert(_szA);
#endif

	//set a model file name
	int nSf=0;

	for(int i=_nSize-1; i; i--)
	{
		if(_szA[i] == '\\' || _szA[i] == '/')
		{
			nSf = i;
			break;
		}
	}

	return (nSf ? KString(_szA,0,nSf-1) : KString());
}


KString KString::FileNameWithoutExt(const char* szFile)
{
	//set a model file name
	int nLength = strlen(szFile),nSi=0,nSf=0;

	for(int i=nLength-1; i; i--)
	{
		if(nSf == 0 && szFile[i] == '.')
			nSf = i;
		else if(szFile[i] == '\\' || szFile[i] == '/')
		{   			
            if((nSi = i+1) != nLength)            	
				break;
            nSf = i;
		}
	}

	return (nSf ? KString(&szFile[nSi],0,nSf-nSi-1) : KString(&szFile[nSi],0,nLength-nSi-1));
}

KString KString::FileNameWithExt(const char* szFile)
{
	//set a model file name
	int nLength = strlen(szFile),nSi=0;

	for(int i=nLength-1; i; i--)
	{
		if(szFile[i] == '\\' || szFile[i] == '/')
		{
			nSi = i+1;
			break;
		}
	}

	return (nSi ? KString(&szFile[nSi],0,nLength-nSi-1) : KString(szFile));
}

KString KString::FileExt(const char* szFile)
{
	//set a model file name
	int nLength = strlen(szFile),nSi=0;

	for(int i=nLength-2; i; i--)
	{
		if(szFile[i] == '.')
		{
			nSi = i+1;
			break;
		}
	}

	return (nSi ? KString(&szFile[nSi],0,nLength-nSi-1) : KString());
}

KString KString::FileNameWithoutExt() const
{
	//set a model file name
	int nSi=0,nSf=0;

	for(int i=_nSize-1; i; i--)
	{
		if(nSf == 0 && _szA[i] == '.')
			nSf = i;
		else if(_szA[i] == '\\' || _szA[i] == '/')
		{   			
            if((nSi = i+1) != _nSize)            	
				break;
            nSf = i;
		}
	}

	return (nSf ? KString(&_szA[nSi],0,nSf-nSi-1) : KString(&_szA[nSi],0,_nSize-nSi-1));
}

KString KString::FileNameWithExt() const
{
	//set a model file name
	int nSi=0;

	for(int i=_nSize-2; i; i--)
	{
		if(_szA[i] == '\\' || _szA[i] == '/')
		{
			nSi = i+1;
			break;
		}
	}

	return (nSi ? KString(&_szA[nSi],0,_nSize-nSi-1) : KString(_szA));
}

KString KString::FileExt() const
{
	//set a model file name
	int nSi=0;

	for(int i=_nSize-2; i; i--)
	{
		if(_szA[i] == '.')
		{
			nSi = i+1;
			break;
		}
	}

	return (nSi ? KString(&_szA[nSi],0,_nSize-nSi-1) : KString());
}
//---------------------------------------------------------------------------


void KMatrix::Create(const KMatrix& mMat)
{
	KArray<double>::Create(mMat.Row(),mMat.Col(),mMat._ppA[0]);
}

void KMatrix::Create(const int& nRow,const int& nCol,const int& type)
{
	//memory alloc.
	KArray<double>::Create(nRow, nCol);

	if(type == _IDENTITY)
		for(int i=0; i<Row(); i++)
			_ppA[i][i] = 1.;

}
//---------------------------------------------------------------------------

void KMatrix::Write(FILE* file) const
{
	if(_ppA)
	{
		int nRow = Row();
		int nCol = Col();

		fwrite(&nRow,sizeof(int),1,file);
		fwrite(&nCol,sizeof(int),1,file);
		fwrite(_ppA[0],sizeof(double),Size(),file);
	}
}

void KMatrix::Write(const char* szFile,const char* szMode) const
{
	if(_ppA)
	{
		FILE* file;
		int   nRow = Row();
		int   nCol = Col();
		char  szModeA[5] = "wb";

		if(szMode)
			strcpy(szModeA,szMode);

		if((file = fopen(szFile,szModeA)) != 0)
		{
			fwrite(&nRow,sizeof(int),1,file);
			fwrite(&nCol,sizeof(int),1,file);
			fwrite(_ppA[0],sizeof(double),Size(),file);
			fclose(file);
		}
	}
}

void KMatrix::WriteText(FILE* file,const char* szFormat) const
{
	//set writing format
    char szDefault[20] = "%12.4f,";
	if(szFormat)
	{
		strcpy(szDefault,szFormat);
        strcat(szDefault,",");
	}

    for(unsigned int i=0; i<Row(); i++){
        for(unsigned int j=0; j<Col(); j++)
			fprintf(file,szDefault,_ppA[i][j]);
		fprintf(file,"\n");
    }
}


bool KMatrix::WriteText(const char* szFile,const char* szFormat,const char* szMode) const
{
	//set writing format
    char szFormatA[20] = "%12.4f,";
	char szModeA[5]    = "wt";

	if(szFormat)
	{
		strcpy(szFormatA,szFormat);
        strcat(szFormatA,",");
	}
	if(szMode)
		strcpy(szModeA,szMode);

	FILE* fp;
	if((fp=fopen(szFile,szModeA)) != 0)
	{
		for(int ic=Row(),i=0; ic; ic--,i++){
			for(int jc=Col(),j=0; jc; jc--,j++)
				fprintf(fp,szFormatA,_ppA[i][j]);
			fprintf(fp,"\n");
        }
		fclose(fp);
		return true;
	}
	return false;
}
//---------------------------------------------------------------------------

bool KMatrix::Read(FILE* file)
{
	//read dimension
	int nRow,nCol;
	if(fread(&nRow,sizeof(int),1,file) != 1)
		return false;
	if(fread(&nCol,sizeof(int),1,file) != 1)
		return false;

	//memory allocation
	if(nRow != 0 && nCol != 0)
	{
		Create(nRow,nCol);
		//read matrix elements
		if(fread(_ppA[0],sizeof(double),Size(),file) == Size())
			return true;
	}

	return false;
}
//---------------------------------------------------------------------------

KMatrix& KMatrix::Place(int nRow,int nCol,const KMatrix& mMat)
{
#ifdef _DEBUF
    assert(!(Col()-nCol < 0 || Row()-nRow < 0));
#endif
    long lSize = sizeof(double) * _MIN(mMat.Col(), Col()-nCol);
    int  nRowA = _MIN(mMat.Row(), Row()-nRow);

    for(int i=nRow,ii=0; ii<nRowA; i++,ii++)
		memcpy(&_ppA[i][nCol],mMat._ppA[ii],lSize);

	return *this;
}

KMatrix& KMatrix::Place(int nRow,int nCol,const KPoint& ptIn)
{
#ifdef _DEBUF
	assert(1 + nRow < Row() && nCol < Col());
#endif

	_ppA[nRow][nCol]    = ptIn._dX;
	_ppA[++nRow][nCol]  = ptIn._dY;

	return *this;
}

KMatrix& KMatrix::Place(int nRow,int nCol,const KPoint3D& ptIn)
{
#ifdef _DEBUF
	assert(2 + nRow < Row() && nCol < Col());
#endif
	_ppA[nRow][nCol]    = ptIn._dX;
	_ppA[++nRow][nCol]  = ptIn._dY;
	_ppA[++nRow][nCol]  = ptIn._dZ;

	return *this;
}
//---------------------------------------------------------------------------

KVector KMatrix::Vector() const
{
	KVector vOut(Size(),_ppA[0]);

	return vOut;
}

double KMatrix::Trace() const
{
#ifdef _DEBUG
	assert(_ppA!=0 && Row()==Col());
#endif

	KMatrix mA = *this;
	KMatrix mU,mV;
	KVector vS;

	mA.SVD(mU,vS,mV);

	return vS.Sum();
}
//---------------------------------------------------------------------------

KVector KMatrix::Sum(const int& nType) const
{
#ifdef _DEBUF
    assert(_ppA != 0);
#endif

    KVector vOut;

    if(nType == _ROW)
    {
        vOut.Create(Col());

        for(int j=0,jj=Col(); jj; j++,jj--)
            for(int i=0,ii=Row(); ii; i++,ii--)
                vOut._ppA[j][0] += _ppA[i][j];
    }
    else if(nType == _COL)
    {
        vOut.Create(Row());

        for(int i=0,ii=Row(); ii; i++,ii--)
            for(int j=0,jj=Col(); jj; j++,jj--)
                vOut._ppA[i][0] += _ppA[i][j];
    }

    return vOut;
}
//---------------------------------------------------------------------------

KVector KMatrix::Min(const int& nType) const
{
#ifdef _DEBUF
    assert(_ppA != 0);
#endif

    KVector vOut;

    if(nType == _ROW)
    {
        vOut.Create(Col());

        for(int j=0,jj=Col(); jj; j++,jj--)
        {
            vOut._ppA[j][0] = _ppA[0][j];
            for(int i=1,ii=Row()-1; ii; i++,ii--)
                vOut._ppA[j][0] = (_ppA[i][j] < vOut._ppA[j][0] ? _ppA[i][j] : vOut._ppA[j][0]);
        }
    }
    else if(nType == _COL)
    {
        vOut.Create(Row());

        for(int i=0,ii=Row(); ii; i++,ii--)
        {
            vOut._ppA[i][0] = _ppA[i][0];
            for(int j=1,jj=Col()-1; jj; j++,jj--)
                vOut._ppA[i][0] = (_ppA[i][j] < vOut._ppA[i][0] ? _ppA[i][j] : vOut._ppA[i][0]);
        }
    }

    return vOut;
}

KVector KMatrix::Max(const int& nType) const
{
#ifdef _DEBUF
    assert(_ppA != 0);
#endif

    KVector vOut;

    if(nType == _ROW)
    {
        vOut.Create(Col());

        for(int j=0,jj=Col(); jj; j++,jj--)
        {
            vOut._ppA[j][0] = _ppA[0][j];
            for(int i=1,ii=Row()-1; ii; i++,ii--)
                vOut._ppA[j][0] = (_ppA[i][j] > vOut._ppA[j][0] ? _ppA[i][j] : vOut._ppA[j][0]);
        }
    }
    else if(nType == _COL)
    {
        vOut.Create(Row());

        for(int i=0,ii=Row(); ii; i++,ii--)
        {
            vOut._ppA[i][0] = _ppA[i][0];
            for(int j=1,jj=Col()-1; jj; j++,jj--)
                vOut._ppA[i][0] = (_ppA[i][j] > vOut._ppA[i][0] ? _ppA[i][j] : vOut._ppA[i][0]);
        }
    }

    return vOut;
}
//---------------------------------------------------------------------------


KVector KMatrix::Column(int c) const
{
	int     nRow = Row();
	KVector vOut(nRow);

	for(int i=0; i<nRow; i++)
		vOut._ppA[i][0] = _ppA[i][c];

	return vOut;
}

void KMatrix::Column(int c,KVector& vOut) const
{
	int nRow = Row();
	vOut.Create(nRow);

	for(int i=0; i<nRow; i++)
		vOut._ppA[i][0] = _ppA[i][c];
}
//---------------------------------------------------------------------------

KMatrix KMatrix::Sign() const
{
#ifdef _DEBBUG
    assert(_ppA);
#endif
    
    KMatrix mOut(Row(),Col());
    for(int i=0,ii=Row(); ii; i++,ii--)
        for(int j=0,jj=Col(); jj; j++,jj--)
            mOut._ppA[i][j] = _SIGN(_ppA[i][j]);

    return mOut;        
}
//---------------------------------------------------------------------------

KMatrix KMatrix::Remove(int idx,int nType) const
{
    KMatrix mOut;

    if(nType == _ROW)
    {
        mOut.Create(Row()-1, Col());
        for(int row=0,r=0; row<Row(); row++)
            if(row != idx)
                memcpy(mOut[r++], _ppA[row], sizeof(double)*Col());
    }
    else //_COL
    {
        mOut.Create(Row(), Col()-1);

        for(int col=0,c=0; col<Col(); col++)
        {
            if(col == idx) continue;

            for(int row=0; row<Row(); row++)
                mOut[row][c] = _ppA[row][col];
            c++;
        }
    }

    return mOut;
}

KMatrix KMatrix::Cut(int nRow,int nCol,int nRow2,int nCol2) const
{
    long     lSize = sizeof(double)*(nCol2-nCol+1);
    KMatrix mOut(nRow2-nRow+1,nCol2-nCol+1);

    for(int i=nRow,ii=0; i<=nRow2; i++,ii++)
      memcpy(mOut._ppA[ii],&_ppA[i][nCol],lSize);
    return mOut;
}



KMatrix KMatrix::Cut(int idx, int nType) const
{
    int     nSize;
    KMatrix mOut;

    if(nType == _COL)
    {
        mOut.Create((nSize = Row()),1);

        for(int i=0; i<nSize; i++)
            mOut._ppA[i][0] = _ppA[i][idx];
    }
    else if(nType == _ROW)
        mOut.Create(1,Col(),_ppA[idx]);

    return mOut;
}

void KMatrix::Cut(int idx, int nType, KVector& vOut) const
{
    int     nSize;

    if(nType == _COL)
    {
        vOut.Create((nSize = Row()));

        for(int i=0; i<nSize; i++)
            vOut._ppA[i][0] = _ppA[i][idx];
    }
    else if(nType == _ROW)
        vOut.Create(Col(), _ppA[idx]);
}
//---------------------------------------------------------------------------

KMatrix KMatrix::From(int nRow,int nCol) const
{
//	ASSERT(_ppA);
   long     lSize = sizeof(double)*(Col()-nCol);
	KMatrix mOut(Row()-nRow,Col()-nCol);

	for(int i=nRow,ii=0; i<Row(); i++,ii++)
	  memcpy(mOut._ppA[ii],&_ppA[i][nCol],lSize);
	return mOut;
}
//---------------------------------------------------------------------------

KMatrix KMatrix::To(int nRow2,int nCol2) const
{
//	ASSERT(_ppA);
   long     lSize = sizeof(double)*(nCol2+1);
	KMatrix mOut(nRow2+1,nCol2+1);

	for(int i=0; i<=nRow2; i++)
	  memcpy(mOut._ppA[i],_ppA[i],lSize);
	return mOut;
}
//---------------------------------------------------------------------------

KMatrix KMatrix::Diag(const int& n,const double* dpValues)
{
	KMatrix mOut(n,n);

	for(int i=0; i<n; i++)
		mOut._ppA[i][i] = dpValues[i];

	return mOut;
}

KMatrix KMatrix::Diag(const KVector& v)
{
	int     n = v.Dim();
	KMatrix mOut(n,n);

	for(int i=0; i<n; i++)
		mOut._ppA[i][i] = v._ppA[i][0];

	return mOut;
}

KMatrix KMatrix::Diag(const double& d1,const double& d2)
{
   KMatrix mOut(2,2);
   mOut._ppA[0][0] = d1;
   mOut._ppA[1][1] = d2;

   return mOut;
}

KMatrix KMatrix::Diag(const double& d1,const double& d2,const double& d3)
{
   KMatrix mOut(3,3);
   mOut._ppA[0][0] = d1;
   mOut._ppA[1][1] = d2;
   mOut._ppA[2][2] = d3;

   return mOut;
}

KMatrix KMatrix::Diag(const KMatrix& mMat1,const KMatrix& mMat2)
{
   KMatrix mOut(mMat1.Row()+mMat2.Row(),mMat1.Col()+mMat2.Col());
   mOut.Place(0,0,mMat1);
   mOut.Place(mMat1.Row(), mMat1.Col(), mMat2);

   return mOut;
}

KMatrix KMatrix::Diag(const KMatrix& mMat1,const KMatrix& mMat2,const KMatrix& mMat3)
{
   KMatrix mOut(mMat1.Row()+mMat2.Row()+mMat3.Row(),mMat1.Col()+mMat2.Col()+mMat3.Col());
   mOut.Place(0,0,mMat1);
   mOut.Place(mMat1.Row(), mMat1.Col(), mMat2);
   mOut.Place(mMat1.Row()+mMat2.Row(), mMat1.Col()+mMat2.Col(), mMat3);

   return mOut;
}
//---------------------------------------------------------------------------

KMatrix KMatrix::Abs()
{
#ifdef _DEBUG
	assert(_ppA);
#endif

	KMatrix mOut(Row(),Col());

	for(int i=0,ii=mOut.Row(); ii; i++,ii--)
		for(int j=0,jj=mOut.Col(); jj; j++,jj--)
			mOut._ppA[i][j] = (_ppA[i][j] >0 ? _ppA[i][j] : -_ppA[i][j]);

	return mOut;
}
//---------------------------------------------------------------------------

KMatrix KMatrix::RepMat(const KMatrix& mMat,const int& nRow,const int& nCol)
{
	int		nStepRow = mMat.Row(), nStepCol = mMat.Col();
	KMatrix mOut(nRow*nStepRow, nCol*nStepCol);

	for(int i=0,ii=0; i<nRow; i++,ii+=nStepRow)
		for(int j=0,jj=0; j<nCol; j++,jj+=nStepCol)
			mOut.Place(ii,jj,mMat);

	return mOut;
}

KMatrix KMatrix::RepMat(const double& dValue,const int& nRow,const int& nCol)
{
	KMatrix mOut(nRow,nCol);

	for(int i=0,ii=nRow; ii; i++,ii--)
		for(int j=0,jj=nCol; jj; j++,jj--)
			mOut._ppA[i][j] = dValue;

	return mOut;
}
//---------------------------------------------------------------------------

KMatrix KMatrix::Ones(const int& nRow,const int& nCol)
{
	KMatrix mOut(nRow,nCol);

	mOut = 1.0;

	return mOut;
}
//---------------------------------------------------------------------------

KMatrix KMatrix::Kron(const KMatrix& mX, const KMatrix& mY)
{
    KMatrix mOut(mX.Row()*mY.Row(),mX.Col()*mY.Col());
    for(int r=0; r<mX.Row(); r++)
        for(int c=0; c<mX.Col(); c++)
            mOut.Place(r*mY.Row(),c*mY.Col(),mX[r][c]*mY);

    return mOut;
}
//---------------------------------------------------------------------------

//m*m.Tr()
KMatrix KMatrix::Cor() const
{
	double   dSum;
	KMatrix oOut(Row(),Row());

	for(int i=0; i<Row(); i++){
		for(int j=0; j<Row(); j++){

			dSum=0;
			for(int jj=0; jj<Col(); jj++)
				dSum += _ppA[i][jj]*_ppA[j][jj];

			oOut._ppA[i][j] = dSum;
		}
	}

	return oOut;

}
//---------------------------------------------------------------------------

//m*m.Tr()
KMatrix KMatrix::Sc1() const
{
	double   dSum;
	KMatrix oOut(Row(),Row());

	for(int i=0; i<Row(); i++){
		for(int j=0; j<Row(); j++){

			dSum=0;
			for(int jj=0; jj<Col(); jj++)
				dSum += _ppA[i][jj]*_ppA[j][jj];

			oOut._ppA[i][j] = dSum;
		}
	}

	return oOut;
}
//---------------------------------------------------------------------------

KMatrix KMatrix::Tr() const
{
	KMatrix oOut(Col(),Row());

	for(int i=0; i<Row(); i++){
		for(int j=0; j<Col(); j++)
			oOut._ppA[j][i] = _ppA[i][j];
	}

	return oOut;
}
//---------------------------------------------------------------------------

//Porperties : - LU Decomposition
double KMatrix::LU(int* nIp)
{
    int		i,j,k,ii,ik;
	double	dDet = 1.;
	double*	dpWeight = new double[Row()];
	double	dT, dU;

	for(k=0; k<Row(); k++){
		nIp[k]	= k;
		dU		= 0.;

		for(j=0; j<Row(); j++){
			dT = fabs(_ppA[k][j]);

			if(dT > dU)	dU = dT;
		}

		//check if LU is possible
		if(dU == 0.){
			delete[] dpWeight;
			return 0.;
		}

		dpWeight[k] = 1. / dU;
	}

	for(k=0; k<Row(); k++){
		dU	= -1.;

		for(i=k; i<Row(); i++){
			ii	= nIp[i];
			dT	= fabs(_ppA[ii][k]) * dpWeight[ii];

			if(dT>dU){
				dU = dT;
				j = i;
			}
		}
		ik = nIp[j];

		if(j != k){
			nIp[j]	= nIp[k];
			nIp[k]	= ik;
			dDet	= -dDet;
		}

		dU		= _ppA[ik][k];
		dDet	*= dU;

		if(dU == 0.){
			delete[] dpWeight;
			return 0.;
		}

		for(i=k+1; i<Row(); i++){
			ii	= nIp[i];
			dT	= (_ppA[ii][k] /= dU);

			for(j=k+1; j<Row(); j++)
				_ppA[ii][j] -= dT*_ppA[ik][j];
		}
	}
	delete[] dpWeight;

	return dDet;

}
//---------------------------------------------------------------------------

//Porperties : - use of LU Decomposition
double KMatrix::Det() const
{
	int		i,j,k,ii,ik;
    int*    npIp  = new int[Row()];
	double	dDet = 1.0;
	double*	dpWeight = new double[Row()];
	double	dT, dU;
    KMatrix mA(*this);

	for(k=0; k<Row(); k++){
		npIp[k]	= k;
		dU		= 0.0;

		for(j=0; j<Row(); j++){
			dT = fabs(mA[k][j]);

			if(dT > dU)	dU = dT;
		}

		//check if LU is possible
		if(dU == 0.0){
			delete[] dpWeight;
            delete[] npIp;
			return 0.0;
		}

		dpWeight[k] = 1.0 / dU;
	}

	for(k=0; k<Row(); k++){
		dU	= -1.0;

		for(i=k; i<Row(); i++){
			ii	= npIp[i];
			dT	= fabs(mA[ii][k]) * dpWeight[ii];

			if(dT>dU){
				dU = dT;
				j = i;
			}
		}
		ik = npIp[j];

		if(j != k){
			npIp[j]	= npIp[k];
			npIp[k]	= ik;
			dDet	= -dDet;
		}

		dU		= mA[ik][k];
		dDet	*= dU;

		if(dU == 0.0){
			delete[] dpWeight;
            delete[] npIp;
			return 0.0;
		}

		for(i=k+1; i<Row(); i++){
			ii	= npIp[i];
			dT	= (mA[ii][k] /= dU);

			for(j=k+1; j<Row(); j++)
				mA[ii][j] -= dT*mA[ik][j];
		}
	}
	delete[] dpWeight;
    delete[] npIp;

	return dDet;
}
//---------------------------------------------------------------------------


//Reference  :   Nemerical recipes in C 2nd
//Porperties : - Cholesky Decomposition
//			   - return the lower matrix,mL : mA = mL*mL'

KMatrix KMatrix::Cholesky()
{
	//Check
//	ASSERT(Row() == Col());

	//Cholesky decomposition
	KMatrix mL(*this);
	int      nDim = Row();
	double*  dpDen = new double[nDim+1];
	double   dSum;

	for(int i=1; i<=nDim; i++){
		for(int j=i; j<=nDim; j++){
			dSum = mL[i-1][j-1];
			for(int k=i-1; k>=1; k--)
				dSum -= mL[i-1][k-1]*mL[j-1][k-1];
			if(i == j){
				if(dSum <= 0){
					//AfxMessageBox("Error in Cholesky Decomposition : not positive definite");
					return *this;
				}
				dpDen[i] = sqrt(dSum);
			}
			else
				mL[j-1][i-1] = dSum / dpDen[i];
		}
	}

	//set lower triangle and diagonal elements of oOut
	for(int j=1; j<=nDim; j++){
		mL[j-1][j-1] = dpDen[j];
		for(int k=j+1; k<=nDim; k++)
			mL[j-1][k-1] = 0.0;
	}
	delete[] dpDen;

	return mL;
}

//reference : Nemerical Recipes in C
// return value : true if singularity is encountered, but decomposition is still completed
//
bool KMatrix::QR(KMatrix& mQ,KMatrix& mR) const
{
	int     i,j,k;
    int     nDim = Row();
	double  dScale,dSigma,dSum,dTau;
    bool    bSingularity = false;
    KVector vC(nDim-1),vD(nDim);
    KMatrix mTmp = *this;

	for(k=0;k<nDim-1;k++)
    {
        dScale = 0.0;
		for(i=k; i<nDim; i++)
            dScale = _MAX(dScale,_ABS(mTmp[i][k]));
		if(dScale == 0.0)
        {
			bSingularity = true;
			vC[k]=vD[k]=0.0;
		}
		else
        {
			for(i=k; i<nDim; i++)
				mTmp[i][k] /= dScale;
			for(dSum=0.0,i=k; i<nDim; i++)
                dSum += _SQR(mTmp[i][k]);

			dSigma      =  _SIGNEX(sqrt(dSum),mTmp[k][k]);
			mTmp[k][k] +=  dSigma;
			vC[k]       =  dSigma*mTmp[k][k];
			vD[k]       = -dScale*dSigma;

			for(j=k+1; j<nDim; j++)
            {
				for(dSum=0.0,i=k; i<nDim; i++)
                    dSum += mTmp[i][k]*mTmp[i][j];
				dTau = dSum/vC[k];

				for(i=k; i<nDim; i++)
                    mTmp[i][j] -= dTau*mTmp[i][k];
			}
		}
	}
	vD[nDim-1] = mTmp[nDim-1][nDim-1];

	if(vD[nDim-1] == 0.0)
        bSingularity = true;

	//set mR
    mR.Create(nDim,nDim);

	for(i=0; i<nDim; i++)
    {
        mR[i][i] = vD[i];
		for(j=i+1; j<nDim; j++)
			mR[i][j]   = mTmp[i][j];
	}

	//set mQ
	KMatrix  mI(nDim,nDim,_IDENTITY);
	KVector  vTmp(nDim);

	mQ.Create(nDim,nDim,_IDENTITY);

	for(j=0; j<nDim-1; j++)
	{
		vTmp = 0.0;
		for(i=j; i<nDim; i++)
			vTmp[i] = mTmp[i][j];

		mQ *= (mI - vTmp.Cor()/vC[j]);
	}

	return bSingularity;
}

bool KMatrix::RQ(KMatrix& mR,KMatrix& mQ) const
{
	//QR decomposition
	bool bSingular = this->Tr().QR(mQ,mR);
	mQ = mQ.Tr();
	mR = mR.Tr();

	KMatrix mQu;
	KVector vTmp;

	vTmp = mR.Cut(1,0,1,2).Vector().Skew() * mR.Cut(2,0,2,2).Vector();
	mQu = vTmp.Normalized(_SIZE_NORMALIZE).Tr();

	vTmp = mQu.Vector().Skew() * mR.Cut(2,0,2,2).Vector();
	mQu ^= vTmp.Normalized(_SIZE_NORMALIZE).Tr();

	vTmp = mQu.Cut(0,0,0,2).Vector().Skew() * mQu.Cut(1,0,1,2).Vector();
	mQu ^= vTmp.Tr();

	mR   = mR  * mQu.Tr();
	mQ	 = mQu * mQ;

	return bSingular;
}


KMatrix KMatrix::IvDefault()
{
	//ASSERT(Row() == Col());

	KMatrix mOut(Row(),Row());

	for(int i=0; i<Row(); i++){
//		ASSERT(_ppA[i][i] != 0);
		mOut._ppA[i][i] = 1./_ppA[i][i];
	}

	return mOut;
}


KMatrix KMatrix::Iv() const
{
#ifdef _DEBUG
	assert(Row() == Col());
#endif

    int		    i, j, k, r, iw;
	double	    w, wmax, pivot, api, w1;
	double  	eps	= 1.0e-10;
	int		    dim	= Row();
	int*		work	= new int[dim];
	KMatrix     a = *this;

	w1 = 1.0;
	for(i=0; i<dim; i++)
		work[i] = i;

	for(k=0; k<dim; k++){
		wmax = 0.0;
		for(i=k; i<dim; i++){
			w = fabs(a[i][k]);
			if(w > wmax){
				 wmax = w;	 r = i;
			}
		}
		pivot = a[r][k];
		api	  = fabs(pivot);
		if(api <= eps){
            delete[] work;
            a.Release();
#ifdef _DEBUG
            assert(0);
#endif
            return a;
		}
		w1 *= pivot;
		if(r != k){
			 w1 = -w; iw = work[k];
			 work[k] = work[r];	 work[r] = iw;
			 for(j=0; j<dim; j++){
				w = a[k][j];
				a[k][j] = a[r][j];
				a[r][j] = w;
			 }
		}
		for(i = 0; i < dim; i++) a[k][i] /= pivot;
		for(i = 0; i < dim; i++) {
			if(i != k){
				 w = a[i][k];
				 if(w != 0.0) {
					for(j = 0; j < dim; j++)
						if(j != k) a[i][j] -= w * a[k][j];
					a[i][k] = -w / pivot;
				 }
			}
		}                                              
		a[k][k] = 1.0 / pivot;
	}

	for(i = 0; i < dim; i++){
		while(1){
			 k = work[i];
			 if(k == i) break;
			 iw = work[k]; work[k] = work[i]; work[i] = iw;
			 for(j = 0; j < dim; j++){
				w = a[j][i];
				a[j][i] = a[j][k];
				a[j][k] = w;
			 }
		}
	}
	delete[] work;

	return a;

}

void KMatrix::Iv(KMatrix& a)
{
    int		    i, j, k, r, iw;
	double	    w, wmax, pivot, api, w1;
	double  	eps	= 1.0e-10;
	int		    dim	= Row();
	int*		work	= new int[dim];

	a  = *this;
	w1 = 1.0;

	for(i=0; i<dim; i++)
		work[i] = i;
	for(k=0; k<dim; k++){
		wmax = 0.0;
		for(i=k; i<dim; i++){
			w = fabs(a[i][k]);
			if(w > wmax){
				 wmax = w;	 r = i;
			}
		}
		pivot = a[r][k];
		api	  = fabs(pivot);
		if(api <= eps){
             delete[] work;
             a.Release();
             return;
		}
		w1 *= pivot;
		if(r != k){
			 w1 = -w; iw = work[k];
			 work[k] = work[r];	 work[r] = iw;
			 for(j=0; j<dim; j++){
				w = a[k][j];
				a[k][j] = a[r][j];
				a[r][j] = w;
			 }
		}
		for(i = 0; i < dim; i++) a[k][i] /= pivot;
		for(i = 0; i < dim; i++) {
			if(i != k){
				 w = a[i][k];
				 if(w != 0.0) {
					for(j = 0; j < dim; j++)
						if(j != k) a[i][j] -= w * a[k][j];
					a[i][k] = -w / pivot;
				 }
			}
		}
		a[k][k] = 1.0 / pivot;
	}

	for(i = 0; i < dim; i++){
		while(1){
			 k = work[i];
			 if(k == i) break;
			 iw = work[k]; work[k] = work[i]; work[i] = iw;
			 for(j = 0; j < dim; j++){
				w = a[j][i];
				a[j][i] = a[j][k];
				a[j][k] = w;
			 }
		}
	}
	delete[] work;
}

KMatrix KMatrix::Diagonalize()
{
#ifdef _DEBUG
	assert(_ppA !=0  && Row() == Col());
#endif

	KMatrix mU,mV;
	KVector vS;

	this->SVD(mU,vS,mV);

	return KMatrix::Diag(vS);
}

void KMatrix::Normalized(const int& nType)
{
#ifdef _DEBUG
	assert(_ppA !=0);
#endif

	int 	nRow = Row();
	int		nCol = Col();
	double 	dNorm;

	if(nType == _COL)
	{
		for(int j=0,jj=nCol; jj; j++,jj--)
		{
			dNorm = 0.0;
			for(int i=0,ii=nRow; ii; i++,ii--)
				dNorm += _ppA[i][j] * _ppA[i][j];
			dNorm = sqrt(dNorm);

			for(int i=0,ii=nRow; ii; i++,ii--)
				_ppA[i][j] /= dNorm;
		}
	}
	if(nType == _ROW)
	{
		for(int i=0,ii=nRow; ii; i++,ii--)
		{
			dNorm = 0.0;
			for(int j=0,jj=nCol; jj; j++,jj--)
				dNorm += _ppA[i][j] * _ppA[i][j];
			dNorm = sqrt(dNorm);

			for(int j=0,jj=nCol; jj; j++,jj--)
				_ppA[i][j] /= dNorm;
		}
	}
}


void KMatrix::Swap(int i,int j,int nType)
{
	//Check
	if(nType == _ROW){
		if(i<0 || i>=Row() || j<0 || j>=Row()){
			//AfxMessageBox("There a invalid swap operation");
			return;
		}
		double dTmp;
		for(int c=0; c<Col(); c++){

			dTmp = _ppA[i][c];
			_ppA[i][c] = _ppA[j][c];
			_ppA[j][c] = dTmp;
		}
	}
	else if(nType == _COL){
		if(i<0 || i>=Col() || j<0 || j>=Col()){
			//AfxMessageBox("There a invalid swap operation");
			return;
		}
		double dTmp;
		for(int r=0; r<Row(); r++){
			dTmp = _ppA[r][i];
			_ppA[r][i] = _ppA[r][j];
			_ppA[r][j] = dTmp;
		}
	}
//	else
//		AfxMessageBox("Invalid swap operation : not specified type");
}

void KMatrix::Set(double dVal,int nIdx,int nType)
{
	if(nType == _COL){
		for(int i=0; i<Row(); i++)
			_ppA[i][nIdx] = dVal;
	}
	else if(nType == _ROW){
		for(int i=0; i<Col(); i++)
			_ppA[nIdx][i] = dVal;
	}
}

void KMatrix::Set(int nType)
{
	if(nType == _IDENTITY){
		if(Row() != 0 && Row() == Col()){
			for(int i=0; i<Row(); i++){
				for(int j=0; j<Col(); j++){
					if(i==j)
						 _ppA[i][j] = 1;
					else _ppA[i][j] = 0;
				}
			}
		}
	}
	else if(nType == _ZERO){
		for(int i=0; i<Row(); i++)
			for(int j=0; j<Col(); j++)
				_ppA[i][j] = nType;
	}
}


bool KMatrix::Eigen(KMatrix& mEigen, KVector& vEigen)
{

	///////////////////////////////////////////////////////
	//
	int      i,j,l,k;
	int      n = Row();
	double   scale,hh,h,g,f;
	double*  e = new double[n+1];
	double*  d = new double[n+1];
    KMatrix mA(Row()+1,Row()+1);

	for(i=0; i<Row(); i++)
		for(int j=0; j<Col(); j++)
			mA[i+1][j+1] = _ppA[i][j];

	for(i=n; i>=2; i--){
		l = i -1;
		h = scale = 0;
		if(l>1){
			for(k=1; k<=l; k++)
				scale += fabs(mA[i][k]);
			if(scale == 0.0)
				e[i] = mA[i][l];
			else{
				for(k=1; k<=l; k++){
					mA[i][k] /= scale;
					h += mA[i][k]*mA[i][k];
				}
				f=mA[i][l];
				g=(f>=0 ? -sqrt(h) : sqrt(h));
				e[i] = scale*g;
				h -= f*g;

				mA[i][l] = f-g;
				f=0.0;

				for(j=1; j<=l; j++){
					mA[j][i] = mA[i][j] / h;
					g = 0.0;

					for(k=1; k<=j; k++)
						g += mA[j][k]*mA[i][k];
					for(k=j+1; k<=l; k++)
						g += mA[k][j]*mA[i][k];
					e[j] = g/h;
					f += e[j]*mA[i][j];
				}
				hh = f/(h+h);

				for(j=1; j<=l; j++){
					f = mA[i][j];
					e[j] = g = e[j] - hh*f;

					for(k=1; k<=j; k++)
						mA[j][k] -= (f*e[k]+g*mA[i][k]);
				}
			}
		}
		else
			e[i] = mA[i][l];
		d[i] = h;
	}

	d[1] = e[1] = 0.0;
	for(i=1; i<=n; i++){
		l = i-1;
		if(d[i]){
			for(j=1; j<=l; j++){
				g=0;
				for(k=1; k<=l; k++)
					g += mA[i][k]*mA[k][j];
				for(k=1; k<=l; k++)
					mA[k][j] -= g*mA[k][i];
			}
		}
		d[i] = mA[i][i];
		mA[i][i] = 1.0;
		for(j=1; j<=l; j++){
			mA[j][i] = 0.0;
			mA[i][j] = 0.0;
		}
	}

	////////////////////////////////////////////////
	int    m,iter;
	double s,r,p,dd,c,b;

	for(i=2; i<=n; i++)
		e[i-1] = e[i];
	e[n] = 0.0;

	for(l=1; l<=n; l++){
		iter = 0;
		do{
			for(m=l; m<=n-1; m++){
				dd=fabs(d[m]) + fabs(d[m+1]);
				if((float)(fabs(e[m])+dd) == dd)
					break;
			}
			if(m != l){
				if(iter++ == 30){
                	delete[] d; delete[] e;
					return false;
				}
				g = (d[l+1]-d[l]) / (2.0*e[l]);
				r = sqrt(g*g+1.0);
				g =d[m]-d[l]+e[l]/(g + _SIGNEX(r,g));
				s=c=1.0;
				p=0.0;

				for(i=m-1; i>=l; i--){
					f=s*e[i];
					b=c*e[i];
					e[i+1] = (r=sqrt(f*f+g*g));

					if(r==0.0){
						d[i+1] -= p;
						e[m]=0.0;
						break;
					}
					s=f/r;
					c=g/r;
					g=d[i+1]-p;
					r=(d[i]-g)*s+(float)(2.0*c*b);
					d[i+1] = g+(p=s*r);
					g=c*r-b;

					for(k=1; k<=n; k++){
						f=mA[k][i+1];
						mA[k][i+1] = s*mA[k][i]+c*f;
						mA[k][i] = c*mA[k][i]-s*f;
					}
				}

				if(r==0.0 && i>= 1)
					continue;
				d[l] -= p;
				e[l] = g;
				e[m] = 0.0;
			}
		}while(m != l);
	}

	//set output
	mEigen.Create(n,n);
	for(i=0; i<n; i++)
		for(j=0; j<n; j++)
			mEigen[i][j] = mA[i+1][j+1];

  	double* pd = new double[n];
	for(i=0; i<n; i++)
		pd[i] = d[i+1];
	vEigen.Create(n,pd);

    delete[] pd;
	delete[] d;
	delete[] e;

	//Sorting in decending way of eingen values
	int nIdx;
	double dT;
	for(i=0; i<n-1; i++){
		dT = vEigen[nIdx=i];

		for(int j=i+1; j<n; j++){
			if(vEigen[j] > dT)
				dT = vEigen[nIdx=j];
		}
		if(nIdx != i){
			vEigen.Swap(nIdx,i,_ROW);
			mEigen.Swap(nIdx,i,_COL);
		}
	}

    return true;
}


//
////////////////////////////////////////////////////////////////////////////////
//
//Reference  : - Gene H. Golub, Matrix 3nd, p463
//             - William H. Press, Numerical Recipesb in C 2rd, pp474-475, pp480-481
//properties : - solve generalized eigen problem
//                mA * X = x * mB * X
//             - input
//                mA : n*n symetric matrix
//                mB : n*n symetric positive definite matrix
//             - output
//                store eigen matrix whose column is a normalized eigen vector
//                return a vector composed of eigen values in decending order
//
KVector KMatrix::Eigen(KMatrix& mA,KMatrix& mB)
{
	//Cholesky decomposition
	KMatrix mL    = mB.Cholesky();
	KMatrix mL_Iv = mL.Iv();

	//make symetric matrix mC
	KMatrix mC = mL_Iv*mA*mL_Iv.Tr();

	//compute eigen values and eigen vectors
	KMatrix mEigen,mV;
	KVector vEigen;
	mC.SVD(mEigen,vEigen,mV); // refer to "Numerical Recipesb in C" 2rd, pp474-475, pp480-481

	//compute the eigen vectors of this generalized eigen problem
	//eigen values are the same as ones of mC
	Create(mL_Iv.Tr()*mEigen);

	//normalize each column(=eigen vector)
	int    i,j;
	double dNorm;
	for(i=0; i<Col(); i++){
		dNorm = 0;
		for(j=0; j<Row(); j++)
			dNorm += _SQR(_ppA[j][i]);
		dNorm = sqrt(dNorm);

		for(j=0; j<Row(); j++)
			_ppA[j][i] /= dNorm;
	}

	//return eigen values on decending order of eigen values
	return vEigen;
}

/*
//
////////////////////////////////////////////////////////////////////////////////
//
//Reference  : - Gene H. Golub, Matrix 3nd, p463
//             - William H. Press, section 2.7 in Numerical Recipesb in C 2rd,
//properties : - transform a sparse matrix into sparse storage mode
//
//             - input
//                dThreshold : only elements of the matrix with magnitude ≥ dThresh are retained
//             - output
//                dpS   : row-indexed array
//                npIdx : indics to point non-zero elements
//
void KMatrix::ToSparseMode(KList<double>& lA, KList<unsigned int>& lIdx, const double& dThresh)
{
	//reset
	lA.RemoveAll();
	lIdx.RemoveAll();

	//into sparse storage mode
	unsigned int dwDim = Row();

	lIdx.Add(dwDim+1);
	for(unsigned int i=0,ii=dwDim; ii; i++,ii--)
	{
		lA.Add(_ppA[i][i]);
		lIdx.Add(0);
	}
	lA.Add(0.0);

	for(unsigned int i=0,ii=dwDim; ii; i++,ii--){
		for(unsigned int j=0,jj=dwDim; jj; j++,jj--)
		{
			if(fabs(_ppA[i][j]) <= dThresh || i==j)
				continue;
			lA.Add(_ppA[i][j]);
			lIdx.Add(j);
		}
		lIdx[i+1] = lIdx.Count();
	}
}


double KMatrix::LinBCG(KList<double>& lA, KList<unsigned int>& lIdx,const KVector& vB,KVector& vX,const int& nITR,const double& dTOL)
{
	unsigned int nDim = lIdx[0]-1;
	KVector 	 vP(nDim);
	KVector 	 vPP(nDim);

	// initial guess : all zeros
	vX.Create(nDim);

	//calculate initial residual
	KVector vR  = vB - SparseAX(lA,lIdx,vX);
	KVector vRR = vR;

	double  dAkden,dAk,dErr;
	double  dBnorm = sqrt(vB.Norm2()),dBknum,dBkden,dBk;
	KVector vZ     = SparseAsolve(lA,lIdx,vR), vZZ;
	int		nItr = 0;

	//loop for vX
	while(nItr++ < nITR)
	{
		vZZ 	= SparseAsolve(lA,lIdx,vRR);
		dBknum 	= (vZ & vRR);

		if(nItr == 1){
			vP  = vZ;
			vPP = vZZ;
		}
		else{
			dBk = dBknum / dBkden;
			for(unsigned int j=0; j<nDim; j++)
			{
				vP[j]  = dBk*vP[j] + vZ[j];
				vPP[j] = dBk*vPP[j] + vZZ[j];
			}
		}

		dBkden  = dBknum;
		vZ      = SparseAX(lA,lIdx,vP);
		dAkden  = vZ & vPP;
		dAk     = dBknum / dAkden;
		vZZ		= SparseATX(lA,lIdx,vPP);

		vX	   += (vP*dAk);
		vR	   -= (vZ*dAk);
		vRR    -= (vZZ*dAk);

		vZ 		= SparseAsolve(lA,lIdx,vR);

		dErr    = sqrt(vR.Norm2()) / dBnorm;

		if(dErr <= dTOL) break;
	}

	//output
	return dErr;
}

//
////////////////////////////////////////////////////////////////////////////////
//
//Reference  : - Gene H. Golub, Matrix 3nd, p463
//             - William H. Press, section 2.7 in Numerical Recipesb in C 2rd,
//properties : - multiply the matrix in sparse mode by a vector vX, giving vB           - SparseAX( )
//             - multiply its transpose matrix in sparse mode by a vector vX, giving vB - SparseATX( )
//             - input
//                lA, lIdx : matrix in Sparse mode
//				  vX       : to be multiplied by the sparse-mode matrix
//             - output
//                vB  =  (lA,lIdx) x vX
//
//
KVector KMatrix::SparseAX(const KList<double>& lA, const KList<unsigned int>& lIdx,const KVector& vX)
{
#ifdef _DEBUG
	assert(lIdx[0] == vX.Dim()+1);
#endif

	unsigned int dwDim = vX.Dim();
	KVector vB(dwDim);

	for(unsigned int i=0,ii=dwDim; ii; i++, ii--)
	{
		vB[i] = lA[i] * vX[i];

		for(unsigned int k=lIdx[i]; k<=lIdx[i+1]-1; k++)
			vB[i] += lA[k] * vX[lIdx[k]];
	}

	return vB;
}

KVector KMatrix::SparseATX(const KList<double>& lA, const KList<unsigned int>& lIdx,const KVector& vX)
{
#ifdef _DEBUG
	assert(lIdx[0] == vX.Dim()+1);
#endif

	unsigned int dwDim = vX.Dim();
	KVector      vB(dwDim);

	for(unsigned int i=0,ii=dwDim; ii; i++, ii--)
		vB[i] = lA[i] * vX[i];

	for(unsigned int i=0,j,ii=dwDim; ii; i++, ii--){
		for(unsigned int k=lIdx[i]; k<= lIdx[i+1]-1; k++)
		{
			j = lIdx[k];
			vB[j] += lA[k] * vX[i];
		}
	}

	return vB;
}

//
KVector KMatrix::SparseAsolve(const KList<double>& lA, const KList<unsigned int>& lIdx,const KVector& vB)
{
	KVector  vX(lIdx[0]-1);
	for(unsigned int i=0, ii=vX.Dim(); ii; i++,ii--)
		vX[i] = (lA[i] != 0 ? vB[i]/lA[i] : vB[i]);

	return vX;
}
*/


int KMatrix::operator==(const KMatrix& mMat)
{
	if(Row() != mMat.Row() || Col() != mMat.Col())
		return 0;
	for(int i=0; i<Row(); i++){
		for(int j=0; j<Col(); j++){
			if(_ppA[i][j] != mMat._ppA[i][j])
				return 0;
		}
	}
	return 1;
}


int KMatrix::operator!=(const KMatrix& mMat)
{
	if(Row() != mMat.Row() || Col() != mMat.Col())
		return 1;
	for(int i=0; i<Row(); i++){
		for(int j=0; j<Col(); j++)
			if(_ppA[i][j] != mMat._ppA[i][j])
				return 1;
	}

	return 0;
}

KMatrix& KMatrix::operator+=(const KMatrix& mMat)
{
	if(_ppA == 0)
		Create(mMat.Row(),mMat.Col());

#ifdef _DEBUG
	assert(Row() == mMat.Row() && Col() == mMat.Col());
#endif

	for(int i=0; i<Row(); i++){
		for(int j=0; j<Col(); j++)
			_ppA[i][j] += mMat._ppA[i][j];
	}

	return *this;
}

KMatrix& KMatrix::operator +=(const double& dVal)
{
#ifdef _DEBUG
    assert(mMat.Address());
#endif

    for(int i=0; i<Row(); i++){
        for(int j=0; j<Col(); j++)

            _ppA[i][j] += dVal;
    }

    return *this;
}

KMatrix& KMatrix::operator=(const KMatrix& mMat)
{
    if(mMat.Pointer())
        KArray<double>::Create(mMat.Row(),mMat.Col(),mMat._ppA[0]);
	return *this;
}


KMatrix& KMatrix::operator-=(const KMatrix& mMat)
{
#ifdef _DEBUG
    assert(_ppA != 0 && Row() == mMat.Row() && Col() == mMat.Col());
#endif
	for(int i=0; i<Row(); i++){
		for(int j=0; j<Col(); j++)
			_ppA[i][j] -= mMat._ppA[i][j];
	}
	return *this;
}

KMatrix& KMatrix::operator *=(const KMatrix& mIn)
{
#ifdef _DEBUG
    assert(_ppA != 0 && Col() == mIn.Row() );
#endif

	double  dSum;
    int     nRow = Row();
    int     nCol = mIn.Col();
    double* dpTmp = new double[mIn.Row()];

	for(int i=0; i<nRow; i++){
		for(int j=0,jj=0; j<nCol; j++,jj++){
			dSum = 0.0;
			for(int k=0; k<Col(); k++)
				dSum += _ppA[i][k] * mIn._ppA[k][j];
			dpTmp[jj] = dSum;
		}
        memcpy(_ppA[i],dpTmp,sizeof(double)*mIn.Row());
	}
    delete[] dpTmp;

	return *this;
}

KMatrix& KMatrix::operator *=(const double& dS)
{
#ifdef _DEBUG
    assert(_ppA != 0 );
#endif
	for(int i=0,ii=Row(); ii; i++,ii--)
		for(int j=0,jj=Col(); jj; j++,jj--)
			_ppA[i][j] *= dS;

	return *this;
}

KMatrix& KMatrix::operator = (const KImageGray& igImg)
{
    KArray<double>::Create(igImg.Row(),igImg.Col());

    for(int i=0; i<Row(); i++)
        for(int j=0; j<Col(); j++)
            _ppA[i][j] = (double)igImg._ppA[i][j];

    return *this;
}

KMatrix& KMatrix::operator=(const double& dS)
{
#ifdef _DEBUG
	assert(_ppA != 0);
#endif
	if(dS){
		for(int i=0,ii=Row(); ii; i++,ii--)
			for(int j=0,jj=Col(); jj; j++,jj--)
				_ppA[i][j] = dS;
	}
	else
		memset(_ppA[0],0,Size()*sizeof(double));

	return *this;
}

KMatrix& KMatrix::operator -=(const double& dVal)
{
#ifdef _DEBUG
    assert(mMat.Address());
#endif

    for(int i=0; i<Row(); i++){
        for(int j=0; j<Col(); j++)

            _ppA[i][j] -= dVal;
    }

    return *this;
}


KMatrix& KMatrix::operator/=(const double& dS)
{
#ifdef _DEBUG
	assert(dS != 0.0 && _ppA != 0);
#endif

	double dIvS = 1.0/dS;

	for(int i=0,ii=Row(); ii; i++,ii--)
		for(int j=0,jj=Col(); jj; j++,jj--)
			_ppA[i][j] *= dIvS;

	return *this;
}

KMatrix& KMatrix::operator/=(const KMatrix& mDen)
{
#ifdef _DEBUG
	assert(_ppA != 0 && Row() == mDen.Row() && Col() == mDen.Col());
#endif

	for(int i=0,ii=Row(); ii; i++,ii--)
		for(int j=0,jj=Col(); jj; j++,jj--)
			_ppA[i][j] /= mDen._ppA[i][j];

	return *this;
}


KMatrix KMatrix::operator*(const double&  dV) const
{
#ifdef _DEBUG
    assert(_ppA != 0);
#endif

	//Create a object for return
	KMatrix mOut(*this);
	for(int i=0, ii=Size(); ii; i++,ii--)
		mOut._ppA[0][i] *= dV;

	return mOut;
}
//---------------------------------------------------------------------------

KMatrix KMatrix::operator+(const double&  dV) const
{
#ifdef _DEBUG
	assert(_ppA != 0);
#endif

	//Create a object for return
	KMatrix mOut(*this);
	for(int i=0, ii=Size(); ii; i++,ii--)
		mOut._ppA[0][i] += dV;

	return mOut;
}
//---------------------------------------------------------------------------

KMatrix KMatrix::operator-(const double&  dV) const
{
#ifdef _DEBUG
	assert(_ppA != 0);
#endif

	//Create a object for return
	KMatrix mOut(*this);
	for(int i=0, ii=Size(); ii; i++,ii--)
		mOut._ppA[0][i] -= dV;

	return mOut;
}
//---------------------------------------------------------------------------

KMatrix operator * (const double& dV, const KMatrix& mMat)
{
#ifdef _DEBUG
	assert(mMat._ppA != 0);
#endif

	KMatrix mOut(mMat);
	for(int i=0; i<mMat.Row(); i++){
		for(int j=0; j<mMat.Col(); j++)
			mOut._ppA[i][j] *= dV;
	}

	return mOut;
}
//---------------------------------------------------------------------------

KMatrix KMatrix::operator / (const double&  dV) const
{
#ifdef _DEBUG
	assert(_ppA != 0 && dV != 0.0);
#endif

	//Create a object for return
	KMatrix oOut(*this);
	for(int i=0; i<Row(); i++){
		for(int j=0; j<Col(); j++)
			oOut._ppA[i][j] /= dV;
	}

	return oOut;
}
//---------------------------------------------------------------------------

KMatrix KMatrix::operator *(const KMatrix& mMat) const
{
#ifdef _DEBUG
	assert(Col() == mMat.Row());
#endif
	KMatrix mOut(Row(),mMat.Col());
	double  dSum;
	for(int i=0; i<Row(); i++){
		for(int j=0; j<mMat.Col(); j++){
			dSum=0.0;
			for(int k=0; k<Col(); k++)
				dSum += _ppA[i][k] * mMat._ppA[k][j];
			mOut._ppA[i][j] = dSum;
		}
	}

	return mOut;
}

//element-wise multiplication
KMatrix KMatrix::operator &(const KMatrix& mMat) const
{
#ifdef _DEBUG
	assert(Col() == mMat.Col() && Row() == mMat.Row());
#endif

	KMatrix mOut(Row(),Col());
	double  dSum;

	for(int i=0,ii=Row(); ii; i++,ii--)
		for(int j=0,jj=Col(); jj; j++,jj--)
			mOut._ppA[i][j] = _ppA[i][j] * mMat._ppA[i][j];

	return mOut;
}
//---------------------------------------------------------------------------

KVector KMatrix::operator *(const KPoint& ptIn) const
{
#ifdef _DEBUG
	assert(Col() == 2);
#endif
    int     nRow = Row();
	KVector vOut(nRow);

	for(int i=0; i<nRow; i++)
		vOut._ppA[i][0] = _ppA[i][0]*ptIn._dX +  _ppA[i][1]*ptIn._dY;
	return vOut;
}
//---------------------------------------------------------------------------

KVector KMatrix::operator *(const KVector& vIn) const
{
    int     nCol = Col();
	int     nRow = Row();
#ifdef _DEBUG
    assert(nCol == vIn.Dim());
#endif

	KVector vOut(nRow);
	double  dSum;

	for(int i=0; i<nRow; i++){
	    dSum=0.0;
		for(int k=0; k<nCol; k++)
		    dSum += _ppA[i][k] * vIn._ppA[k][0];
        vOut._ppA[i][0] = dSum;
	}

	return vOut;
}
//---------------------------------------------------------------------------


KMatrix KMatrix::operator +(const KMatrix& mMat) const
{
#ifdef _DEBUG
	assert(Row() == mMat.Row() && Col() == mMat.Col());
#endif
	KMatrix mOut(*this);
	for(int i=0; i<Row(); i++){
		for(int j=0; j<Col(); j++)
			mOut._ppA[i][j] += mMat._ppA[i][j];
	}

	return mOut;
}
//---------------------------------------------------------------------------

KMatrix KMatrix::operator /(const KMatrix& mMat) const
{
#ifdef _DEBUG
	assert(Row() == mMat.Row() && Col() == mMat.Col());
#endif
	KMatrix mOut(*this);
	for(int i=0,ii=Row(); ii; i++,ii--){
		for(int j=0,jj=Col(); jj; j++,jj--)
		{
#ifdef _DEBUG
			assert(mMat._ppA[i][j] != 0.0);
#endif
			mOut._ppA[i][j] /= mMat._ppA[i][j];
		}
	}

	return mOut;
}
//---------------------------------------------------------------------------


KMatrix KMatrix::operator -(const KMatrix& mMat) const
{
#ifdef _DEBUG
    assert(Row() == mMat.Row() && Col() == mMat.Col());
#endif
	KMatrix mOut(*this);
	for(int i=0; i<Row(); i++){
		for(int j=0; j<Col(); j++)
			mOut._ppA[i][j] -= mMat._ppA[i][j];
	}

	return mOut;
}
//---------------------------------------------------------------------------

KMatrix KMatrix::operator |(const KMatrix& mMat) const
{
#ifdef _DEBUG
	assert(Row() == mMat.Row() || _ppA == 0);
#endif

	KMatrix mOut(Row(),Col()+mMat.Col());
	for(int i=0; i<mOut.Row(); i++)
		for(int j=0; j<mOut.Col(); j++)
			mOut._ppA[i][j] = ( j<Col() ? _ppA[i][j] : mMat._ppA[i][j-Col()]);

	return mOut;
}


KMatrix& KMatrix::operator |= (const KMatrix& mMat)
{
	if(_ppA == 0)
		*this = mMat;
	else
	{
#ifdef _DEBUG
		assert(mMat.Row() == Row());
#endif

		//set output matrix
		KMatrix mOut(Row(),Col()+mMat.Col());

		mOut.Place(0,0,*this);
		mOut.Place(0,Col(),mMat);

		*this = mOut;
	}

	return *this;
}
//---------------------------------------------------------------------------


KMatrix KMatrix::operator ^(const KMatrix& mMat) const
{
	if(_ppA == 0)
        return mMat;

    //check
    if(_nCol != mMat._nCol)
    {
        throw("incorect dimension in (KMatrix::operator ^)");
    }

	//Create output matrix
	KMatrix mOut(Row()+mMat.Row(),Col());

	//Set output matrix
	memcpy(mOut._ppA[0],_ppA[0],sizeof(double)*Size());
	memcpy(mOut._ppA[Row()],mMat._ppA[0],sizeof(double)*mMat.Size());

	return mOut;
}

KMatrix& KMatrix::operator ^= (const KMatrix& mMat)
{
	if(_ppA == 0)
		*this = mMat;
	else
	{
#ifdef _DEBUG
		assert(mMat.Col() == Col());
#endif

		//set output matrix
		KMatrix mOut(Row()+mMat.Row(),Col());

		memcpy(mOut._ppA[0],_ppA[0],sizeof(double)*Size());
		memcpy(mOut._ppA[Row()],mMat._ppA[0],sizeof(double)*mMat.Size());

		*this = mOut;
	}

	return *this;
}
//---------------------------------------------------------------------------

KMatrix& KMatrix::operator ^= (const double& dExp)
{
#ifdef _DEBUG
	assert(_ppA != 0);
#endif

	for(int i=0,ii=Row(); ii; i++,ii--)
		for(int j=0,jj=Col(); jj; j++,jj--)
			_ppA[i][j] = pow(_ppA[i][j],dExp);

	return *this;
}

KMatrix KMatrix::operator ^(const double& dExp) const
{
#ifdef _DEBUG
	assert(_ppA != 0);
#endif

	KMatrix mOut(Row(),Col());

	for(int i=0,ii=Row(); ii; i++,ii--)
		for(int j=0,jj=Col(); jj; j++,jj--)
			mOut._ppA[i][j] = pow(_ppA[i][j],dExp);

	return mOut;
}
//---------------------------------------------------------------------------


KMatrix KMatrix::operator-() const
{
	//Create output matrix
	KMatrix oOut(Row(),Col());

	//Set output matrix
	for(int i=0; i<Row(); i++){
		for(int j=0; j<Col(); j++)
			oOut._ppA[i][j] = -_ppA[i][j];
	}

	return oOut;
}
//---------------------------------------------------------------------------

KMatrix KMatrix::operator ~() const
{
	int		    i, j, k, r, iw;
	double	    w, wmax, pivot, api, w1;
	double  	eps	= 1.0e-10;
	int		    dim	= Row();
	int*		work	= new int[dim];
	KMatrix    a = *this;

	w1 = 1.0;
	for(i=0; i<dim; i++)
		work[i] = i;

	for(k=0; k<dim; k++){
		wmax = 0.0;
		for(i=k; i<dim; i++){
			w = fabs(a[i][k]);
			if(w > wmax){
				 wmax = w;	 r = i;
			}
		}
		pivot = a[r][k];
		api	  = fabs(pivot);
		if(api <= eps){
            delete[] work;
            a.Release();
            assert(0);
            return a;
		}
		w1 *= pivot;
		if(r != k){
			 w1 = -w; iw = work[k];
			 work[k] = work[r];	 work[r] = iw;
			 for(j=0; j<dim; j++){
				w = a[k][j];
				a[k][j] = a[r][j];
				a[r][j] = w;
			 }
		}
		for(i = 0; i < dim; i++) a[k][i] /= pivot;
		for(i = 0; i < dim; i++) {
			if(i != k){
				 w = a[i][k];
				 if(w != 0.0) {
					for(j = 0; j < dim; j++)
						if(j != k) a[i][j] -= w * a[k][j];
					a[i][k] = -w / pivot;
				 }
			}
		}
		a[k][k] = 1.0 / pivot;
	}

	for(i = 0; i < dim; i++){
		while(1){
			 k = work[i];
			 if(k == i) break;
			 iw = work[k]; work[k] = work[i]; work[i] = iw;
			 for(j = 0; j < dim; j++){
				w = a[j][i];
				a[j][i] = a[j][k];
				a[j][k] = w;
			 }
		}
	}
	delete[] work;

	return a;
}
//---------------------------------------------------------------------------


//
//////////////////////////////////////////////////////////////////////////////
//
// class KVector
void KVector::Create(const int& nDim)
{
	KMatrix::Create(nDim,1);
}
//---------------------------------------------------------------------------

void KVector::Create(const int& nDim,double* dpV,int nMode)
{
	KArray<double>::Create(nDim,1,dpV,nMode);
}
//---------------------------------------------------------------------------

bool KVector::Read(FILE* fp)
{
	//read the dimension of a vector
	int nRow;
	if(fread(&nRow,sizeof(int),1,fp) != 1)
		return false;

	//create KVector object
	if(nRow)
	{
		KArray<double>::Create(nRow,1);
		//set elements of vector
		if(fread(_ppA[0],sizeof(double),nRow,fp) == (unsigned int)nRow)
			return true;
	}

	return false;
}

bool KVector::Read(char* szFile)
{
	FILE* fp;
	if((fp=fopen(szFile,"rb")) == 0)
		return false;

	//read the dimension of a vector
	int nRow;
	if(fread(&nRow,sizeof(int),1,fp) != 1)
		return false;

	//create KVector object
	if(nRow)
	{
		KArray<double>::Create(nRow,1);
		//set elements of vector
		if(fread(_ppA[0],sizeof(double),nRow,fp) == (unsigned int)nRow)
			return true;
	}

	return false;
}

//---------------------------------------------------------------------------

int KVector::Write(FILE* fp) const
{
	int nBytes = 0;
	int nRow   = Row();

	if(_ppA)
	{		
		nBytes += sizeof(int)*fwrite(&nRow,sizeof(int),1,fp);
		nBytes += sizeof(double)*fwrite(_ppA[0],sizeof(double),nRow,fp);
	}

	return nBytes;
}

void KVector::Write(const char* szFile,const char* szMode) const
{
	FILE* fp;	
	int   nRow       = Row();
	char  szModeA[5] = "wb";

	if(szMode)
    	strcpy(szModeA,szMode);

	if((fp=fopen(szFile,szModeA)) != 0)
	{
		fwrite(&nRow, sizeof(int), 1, fp);
		fwrite(_ppA[0], sizeof(double), nRow, fp);
	}	

	fclose(fp);	
}
//---------------------------------------------------------------------------


void KVector::WriteText(FILE* fp,const char* szFormat) const
{
	//set writing format
	char szFormatA[20] = "%12.4f ";

	if(szFormat)
	{
		strcpy(szFormatA,szFormat);
		strcat(szFormatA," ");
	}

	for(int i=0; i<Row(); i++)
		fprintf(fp,szFormatA,_ppA[i][0]);
	fprintf(fp,"\n");
}
//---------------------------------------------------------------------------

void KVector::WriteText(const char* szFile,const char* szFormat,const char* szMode) const
{
	FILE* fp;
	char szFormatA[20] = "%12.4f ";
	char szModeA[5]    = "wt";

	if(szFormat)
	{
		strcpy(szFormatA,szFormat);
		strcat(szFormatA," ");
	}
	if(szMode)
		strcpy(szModeA,szMode);

	if((fp=fopen(szFile,szModeA)) != 0){
		for(int ic=Row(),i=0; ic; ic--,i++)
			fprintf(fp,szFormatA,_ppA[i][0]);
		fprintf(fp,"\n");
		fclose(fp);
	}
}

void KVector::Angle(double& a,double& b,double& c,int nType)
{
#ifdef _DEBUG
	assert(Row() == 3);
#endif

	a = atan2(_ppA[2][0],_ppA[1][0]);
	b = atan2(_ppA[0][0],_ppA[2][0]);
	c = atan2(_ppA[1][0],_ppA[0][0]);

	if(nType == _DEG){
		a = _DEGREE(a);
		b = _DEGREE(b);
		c = _DEGREE(c);
	}
}
//---------------------------------------------------------------------------

double KVector::Angle(int nType)
{
#ifdef _DEBUG
	assert(Row() == 2);
#endif
	double dAng = atan2(_ppA[1][0],_ppA[0][0]);

    return (nType == _DEG ? _DEGREE(dAng) : dAng);
}
//---------------------------------------------------------------------------


KVector KVector::From(int nRow) const
{
#ifdef _DEBUG
	assert(_ppA && nRow >= 0 && nRow < Row());
#endif

	KVector vOut(Row() - nRow);
    memcpy(vOut._ppA[0],_ppA[nRow],(Row() - nRow)*sizeof(double));
    return vOut;
}
//---------------------------------------------------------------------------

KVector KVector::To(int nRow) const
{
#ifdef _DEBUG
	assert(_ppA && nRow >= 0 && nRow < Row() );
#endif

	KVector vOut(nRow+1);
    memcpy(vOut._ppA[0],_ppA[0],(nRow+1)*sizeof(double));

	return vOut;
}
//---------------------------------------------------------------------------

void KVector::QuickSorted(std::vector<int>* lpIdx)
{
    // 정렬 인덱스 초기화
    if(lpIdx != 0)
        for(int i=0, ii=Dim(); ii; i++, ii--)
            lpIdx->push_back(i);

    QuickSorted(Address(), 0, Dim()-1, lpIdx);
}

void KVector::QuickSorted(double dpA[], int first, int last, std::vector<int>* lpIdx)
{
    if(first >= last)
        return;

    int     low = first+1, high = last;
    double  tmp, pivot = dpA[first];

    while(low <= high)
    {
        while(low <= high  && pivot >= dpA[low]) low++;
        while(high > first && pivot <  dpA[high]) high--;

        if (low < high)
        {
            //value swap
            _SWAP(dpA[low], dpA[high], double);
            //index swap
            if(lpIdx != 0)
                _SWAP((*lpIdx)[low], (*lpIdx)[high], int);
        }
    }

    //value swap
    _SWAP(dpA[first], dpA[high], double);
    //index swap
    if(lpIdx != 0)
        _SWAP((*lpIdx)[first], (*lpIdx)[high], int);

    //recursive call
    QuickSorted(dpA, first, high-1, lpIdx);
    QuickSorted(dpA, high+1, last, lpIdx);
}

void KVector::Sorted(int nType,int nNum)
{
    double* pS   = Pointer();
    double  dTmp;
    int     nRow = Row();

    //number of elements to be sorted
    nNum = (nNum == 0 || nNum >= nRow ? nRow-1 : nNum);

    //sorting
    if(nType == _ASCEND){
        for(int i=0; i<nNum; i++)
            for(int j=i+1; j<nRow; j++)
                if(pS[i] > pS[j]){
                    dTmp   = pS[i];
                    pS[i]  = pS[j];
                    pS[j]  = dTmp;
    }}
    else{  //_DESCEND
        for(int i=0; i<nNum; i++)
            for(int j=i+1; j<nRow; j++)
                if(pS[i] < pS[j]){
                    dTmp   = pS[i];
                    pS[i]  = pS[j];
                    pS[j]  = dTmp;
    }}
}

void KVector::SortedBy(std::vector<int> lIdx)
{
    int     nDim  = Dim();
    double* dpTmp = new double[nDim];
    memcpy(dpTmp, _ppA[0], sizeof(double)*nDim);

    for(int i=0, ii=nDim; ii; i++, ii--)
        _ppA[i][0] = dpTmp[lIdx[i]];

    delete[] dpTmp;
}

KVector KVector::Cut(int nFrom, int nTo) const
{
    KVector vOut(nTo-nFrom+1);
    memcpy(vOut._ppA[0],_ppA[nFrom],(nTo-nFrom+1)*sizeof(double));
	return vOut;
}
//---------------------------------------------------------------------------

void KVector::Find(const double& dValue, KVector& vIndex) const
{
    int pos = 0;
    vIndex.Create(Dim());

    for(int i=0,ii=Dim(); ii; i++, ii--)
        if(_ppA[i][0] == dValue)
            vIndex._ppA[pos++][0] = (double)(i);

    if(pos)
        vIndex = vIndex.To(pos-1);
    else
        vIndex.Release();
}

KVector KVector::Insert(int r,double d) const
{
	KVector vOut(Row()+1);
	for(int i=0,j=0; i<Row()+1; i++)
		vOut._ppA[i][0] = ( i==r ? d : _ppA[j++][0] );
	return vOut;
}
//---------------------------------------------------------------------------

KVector KVector::Tail(double d) const
{
	KVector vOut;
	if(_ppA)
    {
    	vOut.Create(Row()+1);
		memcpy(vOut._ppA[0],_ppA[0],Row()*sizeof(double));
		vOut._ppA[Row()][0]= d;
	}
	else{
		vOut.Create(1);
		vOut[0] = d;
	}
	return vOut;
}

KVector& KVector::Tailed(double d)
{
	if(_ppA)
	{
		KVector vTmp(*this);

		Create(Row()+1);
		memcpy(_ppA[0],vTmp._ppA[0],vTmp.Row()*sizeof(double));
		_ppA[Row()-1][0]= d;
	}
	else{
		Create(1);
		_ppA[0][0] = d;
	}
	return *this;
}

KVector KVector::Tail(const KVector& v) const
{
	KVector vOut;
	if(_ppA)
	{
		vOut.Create(Row()+v.Row());
		memcpy(vOut._ppA[0],_ppA[0],Row()*sizeof(double));
		memcpy(vOut._ppA[Row()],v._ppA[0],v.Row()*sizeof(double));
	}
	else
		vOut = v;

	return vOut;
}

KVector& KVector::Tailed(const KVector& v)
{
	if(_ppA)
	{
		KVector vTmp(*this);

		Create(Row()+v.Row());
		memcpy(_ppA[0],vTmp._ppA[0],vTmp.Row()*sizeof(double));
		memcpy(_ppA[vTmp.Row()],v._ppA[0],v.Row()*sizeof(double));
	}
	else
		*this = v;

	return *this;
}
//---------------------------------------------------------------------------

KVector KVector::Floor() const
{
#ifdef _DEBUG
	assert(_ppA);
#endif

	KVector vOut(Dim());
	for(int i=0,ii=Dim(); ii; i++,ii--)
		vOut._ppA[i][0] = floor(_ppA[i][0]);

	return vOut;
}

KVector& KVector::Floored()
{
#ifdef _DEBUG
	assert(_ppA);
#endif

	for(int i=0,ii=Dim(); ii; i++,ii--)
		_ppA[i][0] = floor(_ppA[i][0]);

	return *this;
}
//---------------------------------------------------------------------------

double KVector::Sum() const
{
	double dSum=0.0;
	for(int i=0; i<Row(); i++)
		dSum += _ppA[i][0];
	return dSum;
}

double KVector::SumAbs() const
{
    double dSum=0.0;
    for(int i=0; i<Row(); i++)
        dSum += (_ppA[i][0] > 0 ? _ppA[i][0] : -_ppA[i][0]);
    return dSum;
}
//---------------------------------------------------------------------------

double KVector::Max(int* npIdx,int nMode)
{
	double dMax;
	int    nRow = Row();

	if(npIdx)
	{
		*npIdx = 0;
		if(nMode == _ABSOLUTE)
		{
			dMax   = _ABS(_ppA[0][0]);
			for(int i=1,ic=nRow-1; ic; ic--,i++)
				if(_ABS(_ppA[i][0]) > dMax)
				{
					dMax   = _ABS(_ppA[i][0]);
					*npIdx = i;
				}
		}
		else
		{
			dMax   = _ppA[0][0];
			for(int i=1,ic=nRow-1; ic; ic--,i++)
				if(_ppA[i][0] > dMax)
				{
					dMax   = _ppA[i][0];
					*npIdx = i;
				}
		}
	}
	else if(nMode == _ABSOLUTE)
	{
		dMax = _ABS(_ppA[0][0]);
		for(int i=1,ic=nRow-1; ic; ic--,i++)
			dMax = (_ABS(_ppA[i][0]) > dMax ? _ABS(_ppA[i][0]):dMax);
	}
	else
	{
		dMax = _ppA[0][0];
		for(int i=1,ic=nRow-1; ic; ic--,i++)
			dMax = (_ppA[i][0] > dMax ? _ppA[i][0]:dMax);
	}
	return dMax;
}
//---------------------------------------------------------------------------

double KVector::Min(int* npIdx,int nMode)
{
#ifdef _DEBUG
    assert(_ppA != 0);
#endif

	double dMin;
	int    nRow = Row();

    if(npIdx)
	{
		*npIdx = 0;
		if(nMode == _ABSOLUTE)
		{
			dMin   = _ABS(_ppA[0][0]);
			for(int i=1,ic=nRow-1; ic; ic--,i++)
				if(_ABS(_ppA[i][0]) < dMin)
				{
					dMin   = _ABS(_ppA[i][0]);
					*npIdx = i;
				}
		}
		else
		{
			dMin   = _ppA[0][0];
            for(int i=1,ic=nRow-1; ic; ic--,i++)
				if(_ppA[i][0] < dMin)
				{
					dMin   = _ppA[i][0];
					*npIdx = i;
				}
		}
	}
	else if(nMode == _ABSOLUTE)
	{
		dMin = _ABS(_ppA[0][0]);
		for(int i=1,ic=nRow-1; ic; ic--,i++)
			dMin = (_ABS(_ppA[i][0]) < dMin ? _ABS(_ppA[i][0]):dMin);
	}
	else
	{
		dMin = _ppA[0][0];
		for(int i=1,ic=nRow-1; ic; ic--,i++)
			dMin = (_ppA[i][0] < dMin ? _ppA[i][0]:dMin);
	}
	return dMin;
}
//---------------------------------------------------------------------------

double KVector::Average()
{
	double dSum=0.0;
	for(int i=0; i<Row(); i++)
		dSum += _ppA[i][0];

	return dSum/(double)(Row());
}
//---------------------------------------------------------------------------

double KVector::Variance()
{
    //mean
    double dAve = 0.0;
	for(int i=0; i<Row(); i++)
        dAve += _ppA[i][0];
	dAve /= (double)Row();

	//variance
    double dVar = 0.0;
    for(int j=0; j<Row(); j++)
        dVar += _SQR(_ppA[j][0]-dAve);
    return (Row()>2 ? dVar/(double)(Row()-1) : 0.0);
}
//---------------------------------------------------------------------------

double KVector::Nearest(const double& dValue, int* npIdx)
{
    int    nDim = Row();
    double dMin = INFINITY, dDiff;

    for(int i=0; i<nDim; i++)
        if((dDiff = _DIFF(dValue, _ppA[i][0])) < dMin)
        {
            dMin = dDiff;

            if(npIdx)
                *npIdx = i;
        }

    return dMin;
}
//----------------------------------------------------------------------------

void KVector::Statistics(double& dMean, double& dVar)
{
#ifdef _DEBUG
	assert(Row());
#endif

	//mean
	int nDim = Row();

	dMean = 0.0;
	for(int i=0,ic=nDim; ic; ic--,i++)
		dMean += _ppA[i][0];
	dMean /= (double)nDim;

	//variance
	dVar = 0.0;
	for(int j=0,jc=nDim; jc; jc--,j++)
		dVar += _SQR(_ppA[j][0]-dMean);
	dVar = (nDim == 1 ? 0.0 : dVar/(double)(nDim-1));
}
//---------------------------------------------------------------------------


KVector& KVector::MedianFiltered(const int& nWindow)
{
    int     nHalf = nWindow/2;
    KVector vOld(*this);
    KVector vWindow(2*nHalf+1);

    for(int i=nHalf; i<Dim()-nHalf; i++)
    {
        memcpy(vWindow.Address(),vOld.Address()+(i-nHalf),vWindow.Dim()*sizeof(double));

        _ppA[i][0] = vWindow.Median(_MEDIAN_RANK);
    }

    return *this;
}

double KVector::Median(const int& nType,const int& nTo)
{
    double  dTmp;
    double  dMedian;
    int	 nRow   = (nTo ? nTo+1 : Row());
	KVector			 vTmp(*this);

	if(nType == _MEDIAN_RANK)
	{
		for(int ic=nRow,i=0; ic; ic--, i++)
		{
			for(int jc=nRow-i-1,j=i+1; jc; jc--,j++)
			{
				if(vTmp._ppA[i][0] < vTmp._ppA[j][0])
				{
					dTmp        	= vTmp._ppA[i][0];
					vTmp._ppA[i][0] = vTmp._ppA[j][0];
					vTmp._ppA[j][0] = dTmp;
				}
			}
		}

		dMedian = vTmp._ppA[nRow/2][0];
	}
	else if(nType == _MEDIAN_VALUE)
	{
		KVector	vSort(nRow);

		for(int ic=nRow,i=0,ii=0; ic; ic--, i++)
		{
			for(int jc=nRow-i-1,j=i+1; jc; jc--,j++)
			{
				if(vTmp._ppA[i][0] < vTmp._ppA[j][0])
				{
					dTmp        	= vTmp._ppA[i][0];
					vTmp._ppA[i][0] = vTmp._ppA[j][0];
					vTmp._ppA[j][0] = dTmp;
				}
			}

			if(vSort._ppA[ii][0] != vTmp._ppA[i][0])
			{
				vSort._ppA[ii+1][0] = vTmp._ppA[i][0];
				ii++;
				dMedian				= vSort._ppA[(ii+1)/2][0];
			}
		}
	}

	return dMedian;
}
//---------------------------------------------------------------------------

double KVector::Norm2() const
{
    double dNorm2=0.0;
	for(int i=0; i<Row(); i++)
		dNorm2 += _ppA[i][0]*_ppA[i][0];

	return dNorm2;
}

double KVector::Norm2(const KMatrix& mA) const
{
    double dOut=0.0, dTmp;

	for(int i=0; i<Row(); i++){
		dTmp = 0.0;

		for(int ii=0; ii<Row(); ii++)
			dTmp += _ppA[ii][0]*mA[ii][i];
		dOut += dTmp*_ppA[i][0];
	}

	return dOut;

}
//---------------------------------------------------------------------------

KVector& KVector::Normalized(int nMode,double* dpOut)
{
    int i,ic,nRow = Row();

	if(nMode == _DTB_NORMALIZE){
		double dAve=0.0, dStd=0.0;
		//mean
		for(i=0,ic=nRow; ic; ic--,i++)
			dAve += _ppA[i][0];
		dAve /= (double)nRow;
		//std
		for(i=0,ic=nRow; ic; ic--,i++)
		   dStd += _SQR(_ppA[i][0]-dAve);
		dStd = (nRow>2 ? sqrt(dStd/(nRow-1)) : 0.0);
		if(dpOut)
			*dpOut = dStd;

		if(dStd == 0.0)
			return *this;

		//normalize
		for(i=0,ic=nRow; ic; ic--,i++)
			_ppA[i][0] = (_ppA[i][0]-dAve)/dStd;
		return *this;
	}
	else if(nMode == _SIZE_NORMALIZE){
		double dNorm = 0.0;
		for(i=0,ic=nRow; ic; ic--,i++)
			dNorm += _ppA[i][0]*_ppA[i][0];
		dNorm = sqrt(dNorm);
		if(dpOut)
			*dpOut = dNorm;
        if(dNorm != 0.0)
            for(i=0,ic=nRow; ic; ic--,i++)
                _ppA[i][0] /= dNorm;
		return *this;
	}
	else if(nMode == _HEQ_NORMALIZE){//vector object should be from image gray object
		//init. histogram
		int npHisto[256];
		memset(npHisto,0,sizeof(int)*256);

		//histograming according to intensities
		for(i=0,ic=nRow; ic; ic--,i++)
			npHisto[(int)(_ppA[i][0])] ++;

		//accumulated histogram
		for(i=1,ic=255; ic; ic--,i++)
			npHisto[i] += npHisto[i-1];

		//Histogram Equalization
		for(i=0,ic=256; ic; ic--,i++)
			npHisto[i] =(int)(npHisto[i]*255.0/(double)(nRow) + 0.5);

		//transform the original image
		for(i=0,ic=nRow; ic; ic--,i++)
			_ppA[i][0] = npHisto[(int)_ppA[i][0]];
		return *this;
	}
	else if(nMode == _MEAN_NORMALIZE){
		double dAve=0.0;
		//mean
		for(i=0,ic=nRow; ic; ic--,i++)
			dAve += _ppA[i][0];
		dAve /= (double)(nRow);
		//normalize
		if(dpOut)
			*dpOut = dAve;
		for(i=0,ic=nRow; ic; ic--,i++)
			_ppA[i][0] -= dAve;

		return *this;
	}
	else if(nMode == _STD_NORMALIZE){
		double dAve=0.0, dStd=0.0;
		//mean
		for(i=0,ic=nRow; ic; ic--,i++)
			dAve += _ppA[i][0];
		dAve /= nRow;
		//std
		for(i=0,ic=nRow; ic; ic--,i++)
		   dStd += _SQR(_ppA[i][0]-dAve);
		dStd = (nRow>2 ? sqrt(dStd/(nRow-1)) : 0.0);
		//normalize
		if(dpOut)
			*dpOut = dStd;
		if(dStd == 0.0)
			return *this;
		for(i=0,ic=nRow; ic; ic--,i++)
			_ppA[i][0] = _ppA[i][0]/dStd;
		return
			*this;
	}
	else if(nMode == _UNITSUM_NORMALIZE){
	   double dSum = 0.0;
	   for(i=0,ic=nRow; ic; ic--,i++)
		   dSum += _ppA[i][0];
	   //normalize
	   for(i=0,ic=nRow; ic; ic--,i++)
			_ppA[i][0] /= dSum;
	   if(dpOut)
			*dpOut = dSum;

	   return *this;
	}

	return *this;
}
//---------------------------------------------------------------------------

KVector KVector::Normalize(int nMode) const
{
    int i,ic,nRow = Row();
	KVector		 vOut(Dim());

	if(nMode == _DTB_NORMALIZE){
		double dAve=0.0, dStd=0.0;
		//mean
		for(i=0,ic=nRow; ic; ic--,i++)
			dAve += _ppA[i][0];
		dAve /= (double)nRow;
		//std
		for(i=0,ic=nRow; ic; ic--,i++)
		   dStd += _SQR(_ppA[i][0]-dAve);
		dStd = (nRow>2 ? sqrt(dStd/(nRow-1)) : 0.0);
#ifdef _DEBUG
		assert(dStd != 0.0);
#endif

		//normalize
		for(i=0,ic=nRow; ic; ic--,i++)
			vOut._ppA[i][0] = (_ppA[i][0]-dAve)/dStd;
	}
	else if(nMode == _SIZE_NORMALIZE){
		double dNorm = 0.0;
		for(i=0; i<nRow; i++)
			dNorm += _ppA[i][0]*_ppA[i][0];
		dNorm = sqrt(dNorm);
#ifdef _DEBUG
		assert(dNorm != 0.0);
#endif
		for(i=0,ic=nRow; ic; ic--, i++)
			vOut._ppA[i][0] = _ppA[i][0]/dNorm;
	}
	else if(nMode == _HEQ_NORMALIZE){//vector object should be from image gray object
		//init. histogram
		int npHisto[256];
		memset(npHisto,0,sizeof(int)*256);

		//histograming according to intensities
		for(i=0,ic=nRow; ic; ic--,i++)
			npHisto[(int)(_ppA[i][0])] ++;

		//accumulated histogram
		for(i=1,ic=255; ic; ic--,i++)
			npHisto[i] += npHisto[i-1];

		//Histogram Equalization
		for(i=0,ic=256; ic; ic--,i++)
			npHisto[i] =(int)(npHisto[i]*255.0/(double)(nRow) + 0.5);

		//transform the original image
		for(i=0,ic=nRow; ic; ic--,i++)
			vOut._ppA[i][0] = npHisto[(int)_ppA[i][0]];
	}
	else if(nMode == _MEAN_NORMALIZE){
		double dAve=0.0;
		//mean
		for(i=0,ic=nRow; ic; ic--,i++)
			dAve += _ppA[i][0];
		dAve /= (double)(nRow);
		//normalize
		for(i=0,ic=nRow; ic; ic--,i++)
			vOut._ppA[i][0] = _ppA[i][0] - dAve;
	}
	else if(nMode == _STD_NORMALIZE){
		double dAve=0.0, dStd=0.0;
		//mean
		for(i=0,ic=nRow; ic; ic--,i++)
			dAve += _ppA[i][0];
		dAve /= nRow;
		//std
		for(i=0,ic=nRow; ic; ic--,i++)
		   dStd += _SQR(_ppA[i][0]-dAve);
		dStd = (nRow>2 ? sqrt(dStd/(nRow-1)) : 0.0);
		//normalize
#ifdef _DEBUG
		assert(dStd != 0.0);
#endif
		for(i=0,ic=nRow; ic; ic--,i++)
			vOut._ppA[i][0] = _ppA[i][0]/dStd;
	}
	else if(nMode == _UNITSUM_NORMALIZE){
	   double dSum = 0.0;
	   for(i=0,ic=nRow; ic; ic--,i++)
		   dSum += _ppA[i][0];
	   //normalize
	   for(i=0,ic=nRow; ic; ic--,i++)
			vOut._ppA[i][0] = _ppA[i][0] / dSum;
	}

	return vOut;
}
//---------------------------------------------------------------------------

KVector KVector::Exp() const
{
    KVector vExp(this->Dim());
    for(int i=0, ii=vExp.Dim(); ii; i++, ii--)
        vExp[i] = exp(_ppA[i][0]);

    return vExp;
}

KMatrix KVector::Skew()
{
#ifdef _DEBUG
    assert(Dim() == 3);
#endif

    KMatrix mOut(3,3);

    mOut[2][1] =  _ppA[0][0];
    mOut[0][2] =  _ppA[1][0];
    mOut[1][0] =  _ppA[2][0];
    mOut[1][2] = -_ppA[0][0];
    mOut[2][0] = -_ppA[1][0];
    mOut[0][1] = -_ppA[2][0];

    return mOut;
}

KVector KVector::operator - (const KVector& vVec) const
{
    int nDim = Dim();
#ifdef _DEBUG
    assert(nDim == vVec.Dim());
#endif
	//Create a object for return
	KVector vOut(nDim);

	//substraction
	for(int i=0; i<nDim; i++)
		vOut[i] = _ppA[i][0] - vVec[i];

	return vOut;
}

KVector KVector::operator + (const KVector& vVec) const
{
    int nDim = Dim();
#ifdef _DEBUG
    assert(nDim == vVec.Dim());
#endif
	//Create a object for return
	KVector vOut(Row());

	//substraction
	for(int i=0; i<nDim; i++)
		vOut[i] = _ppA[i][0] + vVec[i];

	return vOut;
}

KVector KVector::operator-() const
{
	//Create output matrix
	KVector vOut(Row());

	//Set output matrix
    for(int i=0; i<Row(); i++)
	    vOut._ppA[i][0] = -_ppA[i][0];

	return vOut;
}
//---------------------------------------------------------------------------


KVector KVector::operator - (const KMatrix& mMat) const
{
	int nDim = Dim();
#ifdef _DEBUG
	assert(nDim == mMat.Row() && mMat.Col()==1);
#endif

	//Create a object for return
	KVector vOut(nDim);

	//substraction
	for(int i=0; i<nDim; i++)
		vOut[i] = _ppA[i][0] - mMat._ppA[i][0];

	return vOut;
}

KVector KVector::operator + (const KMatrix& mMat) const
{
    int nDim = Dim();
#ifdef _DEBUG
    assert(nDim == mMat.Row() && mMat.Col()==1);
#endif
	//Create a object for return
	KVector vOut(nDim);

	//substraction
	for(int i=0; i<nDim; i++)
		vOut[i] = _ppA[i][0] + mMat._ppA[i][0];

	return vOut;
}

KVector operator * (const double& dV, const KVector& mMat)
{
#ifdef _DEBUG
	assert(mMat._ppA != 0);
#endif

	KMatrix mOut(mMat);
	for(int i=0; i<mMat.Row(); i++){
		for(int j=0; j<mMat.Col(); j++)
			mOut._ppA[i][j] *= dV;
	}

	return mOut;
}

KMatrix KVector::operator * (const KMatrix& mMat) const
{
	//Create a object for return
	KMatrix mOut(Row(),mMat.Col());

	//substraction
	for(int i=0; i<Row(); i++)
	  for(int j=0; j<mMat.Col(); j++)
		 mOut[i][j] = _ppA[i][0] * mMat._ppA[0][j];

	return mOut;
}

KVector& KVector::operator = (const KImageGray& oImg)
{
	unsigned char*   pImg = oImg.Pointer();
	int              nSize = oImg.Size();

	KArray<double>::Create(nSize,1);

	for(int ic=nSize,i=0; ic; ic--,i++)
		_ppA[i][0] = pImg[i];
	return *this;
}

KVector& KVector::operator = (const KPoint& ptPoint)
{
   Create(2);

   _ppA[0][0] = ptPoint._dX;
   _ppA[1][0] = ptPoint._dY;

	return *this;
}

KVector& KVector::operator = (const KCOLOR32& oPixel)
{
   Create(3);

   _ppA[0][0] = oPixel.r;
   _ppA[1][0] = oPixel.g;
   _ppA[2][0] = oPixel.b;

	return *this;
}

KVector& KVector::operator = (const KCOLOR24& oPixel)
{
   Create(3);

   _ppA[0][0] = oPixel.r;
   _ppA[1][0] = oPixel.g;
   _ppA[2][0] = oPixel.b;

	return *this;
}

double KVector::operator & (const KVector& vVec) const
{
#ifdef _DEBUG
    assert(Dim() == vVec.Dim());
#endif
    double dInner = 0.0;
    for(int i=0; i<Row(); i++)
        dInner += _ppA[i][0]*vVec._ppA[i][0];

	return dInner;
}


KVector& KVector::operator = (const double& dV)
{
    if(_ppA){
        int nRow = Row();
        for(int i=0; i<nRow; i++)
            _ppA[i][0] = dV;
    }

	return *this;
}

KVector KVector::operator * (const double&  dV) const
{
	//Create a object for return
	int     nRow = Row();
	KVector vOut(nRow);

	//Multiplication
	for(int i=0; i<nRow; i++)
		vOut[i] = _ppA[i][0]*dV;

	return vOut;
}

KVector KVector::operator / (const double&  dV) const
{
	//Create a object for return
	int     nRow = Row();
	KVector vOut(nRow);

	//Multiplication
	for(int i=0,ic=nRow; ic; ic--,i++)
		vOut[i] = _ppA[i][0]/dV;

	return vOut;
}


KVector KVector::operator / (const KVector&  vIn) const

{
#ifdef _DEBUG
	assert(Dim() == vIn.Dim());
#endif

	//create a object for return
	KVector vOut(*this);

	//devide
	for(int i=0,ic=vOut.Row(); ic; ic--,i++)
		vOut._ppA[i][0] /= vIn._ppA[i][0];

	return vOut;
}


double KVector::Distance2(const KVector& v1,const KVector& v2)
{
#ifdef _DEBUG
	assert(v1.Dim() == v2.Dim());
#endif

	double dOut = 0.0;

	for(int i=0,ic=v1.Dim(); ic; ic--,i++)
		dOut += _SQR(v1._ppA[i][0]-v2._ppA[i][0]);

	return dOut;
}

KVector KVector::Min(const KVector& v1,const KVector& v2)
{
#ifdef _DEBUG
	assert(v1.Dim() == v2.Dim());
#endif

	KVector vOut(v1.Dim());

	for(int i=0,ic=v1.Dim(); ic; ic--,i++)
		vOut._ppA[i][0] = (v1._ppA[i][0] < v2._ppA[i][0] ? v1._ppA[i][0] : v2._ppA[i][0]);
	return vOut;
}

KVector KVector::Max(const KVector& v1,const KVector& v2)
{
#ifdef _DEBUG
	assert(v1.Dim() == v2.Dim());
#endif

	KVector vOut(v1.Dim());

	for(int i=0,ic=v1.Dim(); ic; ic--,i++)
		vOut._ppA[i][0] = (v1._ppA[i][0] > v2._ppA[i][0] ? v1._ppA[i][0] : v2._ppA[i][0]);
	return vOut;
}

KVectors::KVectors(const KVectors &vsIn)
{
    this->resize(vsIn.size());
    copy(vsIn.begin(), vsIn.end(), this->begin());
}

KVectors& KVectors::operator = ( const KVectors& vsIn)
{
    this->resize(vsIn.size());
    copy(vsIn.begin(), vsIn.end(), this->begin());

    return *this;
}
//---------------------------------------------------------------------------


KImageDouble::KImageDouble(const KImageGray& igImg,const int& nOption)
{
    int nRow = igImg.Row(), nCol = igImg.Col();
    KArray<double>::Create(nRow,nCol);

    //write to the KImageGray object on its option
    if(nOption == _NO_SCALING)
    {
        for(int i=0,ii=nRow; ii; i++,ii--)
            for(int j=0,jj=nCol; jj; j++,jj--)
                _ppA[i][j] = (double)igImg._ppA[i][j];
    }
    else if(nOption == _DEFAULT_SCALING)  //scaling : 0.0 - 1.0
    {
        for(int i=0,ii=nRow; ii; i++,ii--)
            for(int j=0,jj=nCol; jj; j++,jj--)
                _ppA[i][j] = (double)(igImg._ppA[i][j]) / 255.0;
    }
}
//---------------------------------------------------------------------------

KImageDouble KImageDouble::Integral() const
{
    //first row
    KImageDouble idInt(Row(),Col());
    idInt[0][0]  = _ppA[0][0];

    for(int j=1, jj=Col()-1; jj; j++, jj--)
        idInt[0][j]  = idInt[0][j-1]  + _ppA[0][j];

    //recursion
    double  dAccu;
    for(int i=1,ii=Row()-1; ii; i++, ii--)
    {
        dAccu = 0;
        for(int j=0, jj=Col(); jj; j++, jj--)
        {
            dAccu       +=  _ppA[i][j];
            idInt[i][j] =   idInt[i-1][j] + dAccu;
        }
    }

    return idInt;
}

void KImageDouble::Integral(KImageDouble& idInt, KImageDouble& idInt2) const
{

    //first row    
    idInt.Create(Row(),Col());
    idInt2.Create(Row(),Col());
    idInt[0][0]  = _ppA[0][0];
    idInt2[0][0] = _SQR(_ppA[0][0]);

    for(int j=1, jj=Col()-1; jj; j++, jj--)
    {
        idInt[0][j]  = idInt[0][j-1]  + _ppA[0][j];
        idInt2[0][j] = idInt2[0][j-1] + _SQR(_ppA[0][j]);
    }

    //recursion
    double  dAccu, dAccu2;
    for(int i=1,ii=Row()-1; ii; i++, ii--)
    {
        dAccu = dAccu2 = 0;
        for(int j=0, jj=Col(); jj; j++, jj--)
        {
            dAccu       +=  _ppA[i][j];
            dAccu2      +=  _SQR(_ppA[i][j]);
            idInt[i][j]  =   idInt[i-1][j]  + dAccu;
            idInt2[i][j] =   idInt2[i-1][j] + dAccu2;
        }
    }
}

double KImageDouble::Pixel(const double& dX,const double& dY) const
{
	if(dX < 0 || dY < 0 || dX > Col()-1 || dY > Row()-1)
		return 0.0;

	int x0 = (int)dX;
	int x1 = x0 + 1;
	int y0 = (int)dY;
	int y1 = y0 + 1;

	double wx0 = dX - (double)x0;
	double wx1 = 1.0 - wx0;
	double wy0 = dY - (double)y0;
	double wy1 = 1.0 - wy0;

	/*
	double dOut;

	dOut  = wx1*wy1*(_ppA[y0][x0]);
	dOut += (wx0 ? wx0*wy1*_ppA[y0][x1] : 0.0);
	dOut += (wy0 ? wx1*wy0*_ppA[y1][x0] : 0.0);
	dOut += (wx0 && wy0 ? wx0*wy0*_ppA[y1][x1] : 0.0);

	return dOut;
	*/

	return ( wx1*wy1*(_ppA[y0][x0]) + (wx0 ? wx0*wy1*_ppA[y0][x1] : 0.0) + (wy0 ? wx1*wy0*_ppA[y1][x0] : 0.0) +
			 (wx0 && wy0 ? wx0*wy0*_ppA[y1][x1] : 0.0) );
}

void KImageDouble::Write(KImageGray& igImg,int nOption,int nSi,int nSf,double dMin,double dMax)
{
	//init. the output memory
	int nRow = Row(), nCol = Col();
	igImg.Create(nRow,nCol);

	//write to the KImageGray object on its option
	int    i,j;
	double dTmp;
	if(nOption == _NO_SCALING)
	{
		for(i=0; i<nRow; i++)
			for(j=0; j<nCol; j++)
				igImg._ppA[i][j] = (unsigned char)(_ppA[i][j] > 255.0 ? 255 : (_ppA[i][j]<0.0 ? 0 :_ppA[i][j]));
	}
	//scaling
	else if(nOption == _DEFAULT_SCALING)
	{
		for(i=0; i<nRow; i++)
			for(j=0; j<nCol; j++){
				dTmp =  _ppA[i][j] * 255.0;
				igImg._ppA[i][j] = (unsigned char)(dTmp > 255.0 ? 255 : (dTmp < 0.0 ? 0 : dTmp));
			}
	}
	else if(nOption == _USER_SCALING)
	{
		//range
		double dRange,dSize;
		if(dMin == 0.0 && dMax == 0.0)
		{
			dMax = _ppA[0][0];
			dMin = _ppA[0][0];
			for(i=0; i<nRow; i++)
			{
				for(j=0; j<nCol; j++)
				{
					dMax = (_ppA[i][j] > dMax ? _ppA[i][j] : dMax);
					dMin = (_ppA[i][j] < dMin ? _ppA[i][j] : dMin);
				}
			}
		}
		dRange = dMax - dMin;
		dSize  = nSf-nSi;

#ifdef _DEBUG
		assert(dRange > 0 && dSize > 0);
#endif

		for(i=0; i<nRow; i++)
			for(j=0; j<nCol; j++)
			{
				dTmp = (double)nSi + (_ppA[i][j]-dMin)/dRange*dSize;
				igImg._ppA[i][j] = (unsigned char)(dTmp < 0.0 ? 0 : (dTmp > 255.0 ? 255 : (unsigned char)(dTmp)));
			}
	}
}

KImageDouble& KImageDouble::Scaled(const double& dFrom, const double& dTo)
{
    //scaling factor
    int     nRow = Row(), nCol = Col();
    double  dFactor;
    double  dMax = _ppA[0][0];
    double  dMin = _ppA[0][0];

    for(int i=0; i<nRow; i++)
        for(int j=0; j<nCol; j++)
        {
            dMax = (_ppA[i][j] > dMax ? _ppA[i][j] : dMax);
            dMin = (_ppA[i][j] < dMin ? _ppA[i][j] : dMin);
        }
    dFactor = 1.0 / (dMax  - dMin) * (dTo - dFrom);

    //scaling
    for(int i=0; i<nRow; i++)
        for(int j=0; j<nCol; j++)
            _ppA[i][j] = dFrom + (_ppA[i][j]-dMin) * dFactor;

    return *this;
}


void KImageDouble::Convolution(const KVector& vKernel,const int& nAxis, KImageDouble& idOut, const bool& bExcludeBound)
{
#ifdef _DEBUG
    assert(vKernel.Dim()%2);
#endif

    int     i,ii, j,jj, k,kk;
    int     nSy,nEy,nRow  = Row();
    int     nSx,nEx,nCol  = Col();
    int     nHalf = vKernel.Dim()/2;
    double  dConv;

    idOut.Create(nRow,nCol);

    nSx = (nAxis == _X_AXIS && bExcludeBound ? nHalf : 0);
    nEx = (nAxis == _X_AXIS && bExcludeBound ? nCol-nHalf : nCol);
    nSy = (nAxis == _Y_AXIS && bExcludeBound ? nHalf : 0);
    nEy = (nAxis == _Y_AXIS && bExcludeBound ? nRow-nHalf : nRow);

    if(nAxis == _X_AXIS)
    {
        for(i=nSy; i<nEy; i++)
        {
            for(j=nSx; j<nEx; j++)
            {
                dConv = 0.0;
                for(k=-nHalf,kk=0; k<=nHalf; k++,kk++)
                {
                    jj       = ((jj=j+k) < 0 ? 0 : (jj>=nCol ? nCol-1 : jj));
                    dConv   += _ppA[i][jj] * vKernel[kk];
                }
                idOut._ppA[i][j] = dConv;
            }
        }
    }
    else if(nAxis == _Y_AXIS)
	{
        for(i=nSy; i<nEy; i++)
        {
            for(j=nSx; j<nEx; j++)
            {
                dConv = 0.0;
                for(k=-nHalf,kk=0; k<=nHalf; k++,kk++)
                {
                    ii       = ((ii=i+k) < 0 ? 0 : (ii>=nRow ? nRow-1 : ii));
                    dConv   += _ppA[ii][j] * vKernel[kk];
                }
                idOut._ppA[i][j] = dConv;
            }
        }
    }
}

void KImageDouble::Convolution(const double& dSigma,const int& nAxis,KImageDouble& idOut, const bool& bExcludeBound)
{
    KVector vKernel = KGaussianMulti::Kernel_1D(dSigma);

    Convolution(vKernel, nAxis, idOut, bExcludeBound);
}

KImageDouble KImageDouble::Convolution(const KVector& vKernel,const int& nAxis, const bool& bExcludeBound)
{
    KImageDouble idOut;
    Convolution(vKernel, nAxis, idOut, bExcludeBound);

    return idOut;
}

KImageDouble KImageDouble::Convolution(const double& dSigma,const int& nAxis, const bool& bExcludeBound)
{
    KVector      vKernel = KGaussianMulti::Kernel_1D(dSigma);
    KImageDouble idOut;

    Convolution(vKernel, nAxis, idOut, bExcludeBound);

    return idOut;
}

void KImageDouble::Convolution(const KMatrix& mKernel, KImageDouble& idOut)
{
#ifdef _DEBUG
	assert(mKernel.Row()%2 && mKernel.Row()==mKernel.Col());
#endif

    int     r,rr, c,cc;
    int     nSy,nEy,nRow  = Row();
    int     nSx,nEx,nCol  = Col();
    int     nHalf = mKernel.Row()/2;
    double  dConv;

    idOut.Create(nRow,nCol);

    nSx = nSy =  nHalf;
    nEx = nCol - nHalf;
    nEy = nRow - nHalf;

    for(int i=nSy; i<nEy; i++)
        for(int j=nSx; j<nEx; j++)
        {
            dConv = 0.0;

            for(r = -nHalf,rr=0; r <= nHalf; r++,rr++)            
                for(c = -nHalf,cc=0; c <= nHalf; c++,cc++)
                    dConv  += _ppA[i+r][j+c] * mKernel[rr][cc];

            idOut._ppA[i][j] = dConv;
        }    
}

KImageDouble KImageDouble::HalfSize()
{
	int             nRow = (Row()>>1);
	int             nCol = (Col()>>1);
    KImageDouble    idOut(nRow,nCol);

	for(int i=0; i<nRow; i++)
    {
		for(int j=0; j<nCol; j++)
		    idOut._ppA[i][j] = _ppA[i<<1][j<<1];
	}

    return idOut;
}

KImageDouble KImageDouble::DoubleSize()
{
    int             i,j;
	int             nRow = (Row() << 1);
	int             nCol = (Col() << 1);
    KImageDouble    idOut(nRow,nCol);

	for(i=0; i<nRow; i++)
    {
		for(j=0; j<nCol; j++)
		    idOut._ppA[i][j] = _ppA[i>>1][j>>1];
	}

	/*
	  A B C
	  E F G
	  H I J

	  pixels A C H J are pixels from original image
	  pixels B E G I F are interpolated pixels

	*/

	// interpolate pixels B and I
	for(i=0; i<nRow; i += 2)
		for(j=1; j<nCol-1; j += 2)
			idOut._ppA[i][j] = (_ppA[i>>1][j>>1] + _ppA[i>>1][(j>>1) + 1]) / 2.0;


	// interpolate pixels E and G
	for(i=1; i<nRow-1; i += 2)
		for(j=0; j<nCol; j += 2)
            idOut._ppA[i][j] = (_ppA[i>>1][j>>1] + _ppA[(i>>1)+1][j>>1]) / 2.0;


	// interpolate pixel F
	// interpolate pixels E and G
	for(i=1; i<nRow-1; i += 2)
		for(j=1; j<nCol-1; j += 2)
           idOut._ppA[i][j] = (_ppA[i>>1][j>>1] + _ppA[(i>>1)+1][j>>1] + _ppA[i>>1][(j>>1)+1] + _ppA[(i>>1)+1][(j>>1)+1])/4.0;

    return idOut;
}


KImageDouble KImageDouble::Log()
{
    int          nRow = Row();
    int          nCol = Col();
    KImageDouble idOut(nRow,nCol);

    for(int i=0; i<nRow; i++)
        for(int j=0; j<nCol; j++)
            idOut._ppA[i][j] = log(1.0 + _ppA[i][j]);

    return idOut;
}

KImageDouble& KImageDouble::operator = (const KImageGray& igImg)
{
    KArray<double>::Create(igImg.Row(),igImg.Col());

    for(int i=0; i<Row(); i++)
        for(int j=0; j<Col(); j++)
            _ppA[i][j] = (double)igImg._ppA[i][j];

    return *this;
}

KImageDouble KImageDouble::operator - (const KImageDouble& idImg)
{
    int             nRow = Row(), nCol = Col();
    KImageDouble    idOut(*this);

    for(int i=0; i<nRow; i++)
        for(int j=0; j<nCol; j++)
            _ppA[i][j] -= idImg._ppA[i][j];

    return idOut;
}

KImageDouble& KImageDouble::operator -= (const KImageDouble& idImg)
{
    int  nRow = Row(), nCol = Col();

    for(int i=0; i<nRow; i++)
        for(int j=0; j<nCol; j++)
            _ppA[i][j] -= idImg._ppA[i][j];
    return *this;
}

KImageDouble KImageTriplet::RedBand()
{
    int          nSize = Size();
	KImageDouble idOut(Row(),Col());

    for(int i=0; i<nSize; i++)
            idOut._ppA[0][i] =  _ppA[0][i].r;

    return idOut;
}

KImageDouble KImageTriplet::GreenBand()
{
    int          nSize = Size();
    KImageDouble idOut(Row(),Col());

    for(int i=0; i<nSize; i++)
            idOut._ppA[0][i] =  _ppA[0][i].g;

    return idOut;
}

KImageDouble KImageTriplet::BlueBand()
{
    int          nSize = Size();
    KImageDouble idOut(Row(),Col());

    for(int i=0; i<nSize; i++)
            idOut._ppA[0][i] =  _ppA[0][i].b;

    return idOut;
}
//---------------------------------------------------------------------------

KImageTriplet KImageTriplet::Convolution(const KArray<double>& arMask)
{
    KImageTriplet itOut;
    Convolution(arMask,itOut);
    return itOut;
}

void KImageTriplet::Convolution(const KArray<double>& arMask,KImageTriplet& itOut)
{
    itOut.Create(Row(),Col());

    //compute magnitude and direction of edge
    int     nHalf = arMask.Col()/2;
    int     nEy   = Row() - nHalf;
    int     nEx   = Col() - nHalf;
    double  dTmpR,dTmpG,dTmpB;

    for(int i=nHalf; i<nEy; i++)
        for(int j=nHalf; j<nEx; j++)
        {
             //convolution
             dTmpR = dTmpG = dTmpB = 0.0;
		     for(int r=-nHalf,rr=0; r<=nHalf; r++,rr++)
             {
                for(int c=-nHalf,cc=0; c<=nHalf; c++,cc++)
                {
					dTmpR += _ppA[i+r][j+c].r*arMask._ppA[rr][cc];
                    dTmpG += _ppA[i+r][j+c].g*arMask._ppA[rr][cc];
                    dTmpB += _ppA[i+r][j+c].b*arMask._ppA[rr][cc];
                }
             }
             itOut._ppA[i][j].r = (float)dTmpR;
             itOut._ppA[i][j].g = (float)dTmpG;
             itOut._ppA[i][j].b = (float)dTmpB;
        }
}
//---------------------------------------------------------------------------

KImageTriplet KImageTriplet::Log()
{
    KImageTriplet itOut;
    Log(itOut);
    return itOut;
}

void KImageTriplet::Log(KImageTriplet& itOut)
{
	itOut.Create(Row(),Col());

    //compute magnitude and direction of edge
    int nRow = Row();
    int nCol = Col();
    for(int i=0; i<nRow; i++)
        for(int j=0; j<nCol; j++)
		{
            itOut._ppA[i][j].r = log(1.0 + _ppA[i][j].r);
            itOut._ppA[i][j].g = log(1.0 + _ppA[i][j].g);
            itOut._ppA[i][j].b = log(1.0 + _ppA[i][j].b);
        }
}
//---------------------------------------------------------------------------
KImageTriplet KImageTriplet::Exp()
{
    KImageTriplet itOut;
    Exp(itOut);
    return itOut;
}

void KImageTriplet::Exp(KImageTriplet& itOut)
{
    itOut.Create(Row(),Col());

    //compute magnitude and direction of edge
    int nRow = Row();
    int nCol = Col();
    for(int i=0; i<nRow; i++)
        for(int j=0; j<nCol; j++)
        {
            itOut._ppA[i][j].r = exp(_ppA[i][j].r);
            itOut._ppA[i][j].g = exp(_ppA[i][j].g);
            itOut._ppA[i][j].b = exp(_ppA[i][j].b);
        }
}
//---------------------------------------------------------------------------

void KImageTriplet::Write(KImageGray& igOut,int nBand,int nOption,int nSi,int nSf,double dMin,double dMax)
{
    //init. the output memory
    igOut.Create(Row(),Col());

    //write to the KImageGray object on its option
    int i,j;

    if(nBand == _RED)
    {
        if(nOption == _NO_SCALING)
        {
            for(i=0; i<Row(); i++)
                for(j=0; j<Col(); j++)
					igOut._ppA[i][j] = (unsigned char)(_ppA[i][j].r > 255. ? 255 : (_ppA[i][j].r<0. ? 0 :(unsigned char)(_ppA[i][j].r)));
            return;
        }
        //range
        double dRange;
        if(dMin == 0.0 && dMax == 0.0)
        {
            dMax = -1e100;
            dMin =  1e100;
            for(i=0; i<Row(); i++)
            {
                for(j=0; j<Col(); j++)
                {
                    dMax = (_ppA[i][j].r > dMax ? _ppA[i][j].r : dMax);
                    dMin = (_ppA[i][j].r < dMin ? _ppA[i][j].r : dMin);
                }
            }
        }
        dRange = dMax - dMin;

        //scaling
        if(nOption == _DEFAULT_SCALING)
		{
            for(i=0; i<Row(); i++)
                for(j=0; j<Col(); j++)
                    igOut._ppA[i][j] = (unsigned char)((_ppA[i][j].r-dMin) / dRange * 255.0);
        }
        else if(nOption == _USER_SCALING)
        {
            //scaling
            double dSize = (double)(nSf - nSi),dTmp;

            for(i=0; i<Row(); i++)
                for(j=0; j<Col(); j++)
                {
                    dTmp = (double)nSi + (_ppA[i][j].r-dMin)/dRange*dSize;
					igOut._ppA[i][j] = (unsigned char)(dTmp < 0.0 ? 0 : (dTmp > 255.0 ? 255 : (unsigned char)(dTmp)));
                }
        }
    }
    else if(nBand == _GREEN)
    {
        if(nOption == _NO_SCALING)
        {
            for(i=0; i<Row(); i++)
                for(j=0; j<Col(); j++)
					igOut._ppA[i][j] = (unsigned char)(_ppA[i][j].g > 255. ? 255 : (_ppA[i][j].g<0. ? 0 :(unsigned char)(_ppA[i][j].g)));
            return;
        }
        //range
        double dRange;
        if(dMin == 0.0 && dMax == 0.0)
        {
            dMax = -1e100;
            dMin =  1e100;
            for(i=0; i<Row(); i++)
            {
                for(j=0; j<Col(); j++)
                {
					dMax = (_ppA[i][j].g > dMax ? _ppA[i][j].g : dMax);
                    dMin = (_ppA[i][j].g < dMin ? _ppA[i][j].g : dMin);
                }
            }
        }
        dRange = dMax - dMin;

        //scaling
        if(nOption == _DEFAULT_SCALING)
        {
			for(i=0; i<Row(); i++)
                for(j=0; j<Col(); j++)
                    igOut._ppA[i][j] = (unsigned char)((_ppA[i][j].g-dMin) / dRange * 255.0);
        }
        else if(nOption == _USER_SCALING)
        {
            //scaling
            double dSize = (double)(nSf - nSi),dTmp;

            for(i=0; i<Row(); i++)
                for(j=0; j<Col(); j++)
                {
                    dTmp = (double)nSi + (_ppA[i][j].g-dMin)/dRange*dSize;
					igOut._ppA[i][j] = (unsigned char)(dTmp < 0.0 ? 0 : (dTmp > 255.0 ? 255 : (unsigned char)(dTmp)));
				}
		}
	}
	else if(nBand == _BLUE)
	{
		if(nOption == _NO_SCALING)
		{
			for(i=0; i<Row(); i++)
				for(j=0; j<Col(); j++)
					igOut._ppA[i][j] = (unsigned char)(_ppA[i][j].b > 255. ? 255 : (_ppA[i][j].b<0. ? 0 :(unsigned char)(_ppA[i][j].b)));
            return;
        }
        //range
		double dRange;
        if(dMin == 0.0 && dMax == 0.0)
        {
            dMax = -1e100;
            dMin =  1e100;
            for(i=0; i<Row(); i++)
            {
                for(j=0; j<Col(); j++)
                {
                    dMax = (_ppA[i][j].b > dMax ? _ppA[i][j].b : dMax);
                    dMin = (_ppA[i][j].b < dMin ? _ppA[i][j].b : dMin);
				}
            }
        }
        dRange = dMax - dMin;

        //scaling
        if(nOption == _DEFAULT_SCALING)
        {
            for(i=0; i<Row(); i++)
                for(j=0; j<Col(); j++)
                    igOut._ppA[i][j] = (unsigned char)((_ppA[i][j].b-dMin) / dRange * 255.0);
        }
        else if(nOption == _USER_SCALING)
        {
            //scaling
            double dSize = (double)(nSf - nSi),dTmp;

            for(i=0; i<Row(); i++)
                for(j=0; j<Col(); j++)
                {
					dTmp = (double)nSi + (_ppA[i][j].b-dMin)/dRange*dSize;
					igOut._ppA[i][j] = (unsigned char)(dTmp < 0.0 ? 0 : (dTmp > 255.0 ? 255 : (unsigned char)(dTmp)));
                }
        }
    }
}
//---------------------------------------------------------------------------

void KImageTriplet::Tmp(KImageColor& icImg,KImageTriplet& itIni,KImageTriplet& itLpc)
{
    int i,j;
    int nSx = 1, nEx = itIni.Col()-1;
    int nSy = 1, nEy = itIni.Row()-1;

    KImageTriplet itPrev = itIni;

    Create(icImg.Row(),icImg.Col());

	for(int itr=0; itr<10; itr++)
    {
        for(i=nSy; i<nEy; i++)
        {
            for(j=nSx; j<nEx; j++)
            {
                _ppA[i][j].r = (itPrev[i+1][j].r+itPrev[i][j+1].r+itPrev[i-1][j].r+itPrev[i][j-1].r)/5.0
                                   + (itPrev[i+1][j+1].r+itPrev[i-1][j+1].r+itPrev[i-1][j-1].r+itPrev[i+1][j-1].r)/20.0
                                   -  0.3*itLpc[i][j].r;
                _ppA[i][j].g = (itPrev[i+1][j].g+itPrev[i][j+1].g+itPrev[i-1][j].g+itPrev[i][j-1].g)/5.0
                                   + (itPrev[i+1][j+1].g+itPrev[i-1][j+1].g+itPrev[i-1][j-1].g+itPrev[i+1][j-1].g)/20.0
								   -  0.3*itLpc[i][j].g;
                _ppA[i][j].b = (itPrev[i+1][j].b+itPrev[i][j+1].b+itPrev[i-1][j].b+itPrev[i][j-1].b)/5.0
                                   + (itPrev[i+1][j+1].b+itPrev[i-1][j+1].b+itPrev[i-1][j-1].b+itPrev[i+1][j-1].b)/20.0
                                   -  0.3*itLpc[i][j].b;
            }
        }
        itPrev = *this;
    }  
}
//---------------------------------------------------------------------------


KImageColor KImageGray::GrayToRGB()
{
	KImageColor icOut(this->Row(),this->Col());

	for(int i=0,ii=icOut.Row(); ii; i++,ii--)
		for(int j=0,jj=icOut.Col(); jj; j++,jj--)
			icOut._ppA[i][j].r = icOut._ppA[i][j].g = icOut._ppA[i][j].b = this->_ppA[i][j];

	return icOut;
}

KImageGray KImageGray::Solarize()
{
	KImageGray igOut(Row(),Col());

	for(int ii=Size(),i=0; ii; ii--,i++)
		igOut._ppA[0][i] = ~(_ppA[0][i]);

	return igOut;
}

KImageGray& KImageGray::Solarized()
{
	if(_ppA[0])
	{
		for(int ii=Size(),i=0; ii; ii--,i++)
			_ppA[0][i] = ~(_ppA[0][i]);
	}

	return *this;
}

KImageGray& KImageGray::Scaled(const unsigned char& ucFrom, const unsigned char& ucTo)
{
    //scaling factor
    int           nRow = Row(), nCol = Col();
    double        dFactor;
    unsigned char ucMax = _ppA[0][0];
    unsigned char ucMin = _ppA[0][0];

    for(int i=0; i<nRow; i++)
        for(int j=0; j<nCol; j++)
        {
            ucMax = (_ppA[i][j] > ucMax ? _ppA[i][j] : ucMax);
            ucMin = (_ppA[i][j] < ucMin ? _ppA[i][j] : ucMin);
        }
    dFactor = 1.0 / (double)(ucMax  - ucMin) * (double)(ucTo - ucFrom);

    //scaling
    for(int i=0; i<nRow; i++)
        for(int j=0; j<nCol; j++)
            _ppA[i][j] = (unsigned char)(ucFrom + (_ppA[i][j]-ucMin) * dFactor);

    return *this;
}



void KImageGray::GrayToRGB(KImageColor& icOut)
{
	icOut.Create(this->Row(),this->Col());

	for(int i=0,ii=icOut.Row(); ii; i++,ii--)
		for(int j=0,jj=icOut.Col(); jj; j++,jj--)
			icOut._ppA[i][j].r = icOut._ppA[i][j].g = icOut._ppA[i][j].b = this->_ppA[i][j];

}

KImageDouble KImageGray::Normalize(int nType) const
{
    KImageDouble idImg(Row(),Col());
    KVector      vImg(idImg.Size(), idImg.Address(), _LOCK);


    for(int i=0,idx=0; i<Row(); i++)
        for(int j=0; j<Col(); j++)
            vImg._ppA[idx++][0] = (double)_ppA[i][j];

    if(nType == _DTB_NORMALIZE)
        vImg.Normalized(_DTB_NORMALIZE);
    else if(nType == _HEQ_NORMALIZE)
        vImg.Normalized(_HEQ_NORMALIZE);
    else if(nType == _SIZE_NORMALIZE)
        vImg.Normalized(_SIZE_NORMALIZE);

    return idImg;
}

void KImageGray::Normalize(int nType, KImageDouble& idImg) const
{
    idImg.Create(Row(),Col());

    KVector vImg(idImg.Size(), idImg.Address(), _LOCK);

    for(int i=0,idx=0; i<Row(); i++)
        for(int j=0; j<Col(); j++)
            vImg._ppA[idx++][0] = (double)_ppA[i][j];

    if(nType == _DTB_NORMALIZE)
        vImg.Normalized(_DTB_NORMALIZE);
    else if(nType == _HEQ_NORMALIZE)
        vImg.Normalized(_HEQ_NORMALIZE);
    else if(nType == _SIZE_NORMALIZE)
        vImg.Normalized(_SIZE_NORMALIZE);
}

KImageGray& KImageGray::HistoEqualized()
{
    //histograming according to intensities
    int             npHisto[256] = { 0,  };
    unsigned char*  upAdd = this->Address();
    int             nSize = this->Size();

    for(int i=0, ic=nSize; ic; i++, ic--)
        npHisto[upAdd[i]] ++;

    //accumulated histogram
    for(int i=1, ic=255; ic; i++, ic--)
        npHisto[i] += npHisto[i-1];

    //Histogram Equalization
    for(int i=0, ic=256; ic; i++, ic--)
        npHisto[i] = (int)((double)(npHisto[i]) / (double)(nSize) * 255.0);

    //transform the original image
    for(int i=0, ic=nSize; ic; i++, ic--)
        upAdd[i] = (unsigned char)(npHisto[ upAdd[i] ]);

    return *this;
}

KImageGray KImageGray::HistoEqualize() const
{
    //histograming according to intensities
    int             npHisto[256] = { 0,  };
    unsigned char*  upAdd = this->Address();
    int             nSize = this->Size();
    KImageGray      igOut(*this);

    for(int i=0, ic=nSize; ic; i++, ic--)
        npHisto[upAdd[i]] ++;

    //accumulated histogram
    for(int i=1, ic=255; ic; i++, ic--)
        npHisto[i] += npHisto[i-1];

    //Histogram Equalization
    for(int i=0, ic=256; ic; i++, ic--)
        npHisto[i] = (int)((double)(npHisto[i]) / (double)(nSize) * 255.0);

    //transform the original image
    upAdd = igOut.Address();
    for(int i=0, ic=nSize; ic; i++, ic--)
        upAdd[i] = (unsigned char)(npHisto[ upAdd[i] ]);

    return igOut;
}

void KImageGray::HistoEqualize(KImageGray& igOut) const
{
    //histograming according to intensities
    int             npHisto[256] = { 0,  };
    unsigned char*  upAdd = this->Address();
    int             nSize = this->Size();

    igOut.Create(Row(), Col());

    for(int i=0, ic=nSize; ic; i++, ic--)
        npHisto[upAdd[i]] ++;

    //accumulated histogram
    for(int i=1, ic=255; ic; i++, ic--)
        npHisto[i] += npHisto[i-1];

    //Histogram Equalization
    for(int i=0, ic=256; ic; i++, ic--)
        npHisto[i] = (int)((double)(npHisto[i]) / (double)(nSize) * 255.0);

    //transform the original image
    upAdd = igOut.Address();

    for(int i=0, ic=nSize; ic; i++, ic--)
        upAdd[i] = (unsigned char)(npHisto[ upAdd[i] ]);
}

double KImageGray::TotalVariation(const KRect& rcWindow) const
{
    double dSum = 0.0;    

    for(int r=rcWindow._nTop+1; r<rcWindow._nBottom - 1; r++)
        for(int c=rcWindow._nLeft+1; c<rcWindow._nRight-1; c++)
            dSum += _ABS(_ppA[r][c+1] - _ppA[r][c-1]) + _ABS(_ppA[r+1][c] - _ppA[r-1][c]);

    dSum /= ( (rcWindow.Width() - 1.0)*(rcWindow.Height()-1.0) );

    return dSum;
}


KImageGray& KImageGray::Cleared(const KRect& rcArea,const int& nDir)
{
	if(nDir == _INWARD)
	{
		int nBytes = rcArea.Width();
#ifdef _DEBUG
		assert(_ppA != 0 && nBytes+rcArea._nLeft <= Col() && rcArea._nBottom < Row());
#endif
		for(int i=rcArea._nTop,ic=rcArea.Height(); ic; ic--,i++)
			memset(&_ppA[i][rcArea._nLeft],0,nBytes);
		return *this;
	}

	KImageGray igMask(Row(),Col());
	igMask.Activated(rcArea);

	for(int i=0,ii=Row(); ii; i++,ii--)
		for(int j=0,jj=Col(); jj; j++,jj--)
			_ppA[i][j] &= igMask._ppA[i][j];

	return *this;
}

KImageGray& KImageGray::Activated(const KRect& rcArea,const int& nDir)
{
	if(nDir == _INWARD)
	{
		int nBytes = rcArea.Width();
	#ifdef _DEBUG
		assert(_ppA != 0 && nBytes+rcArea._nLeft <= Col() && rcArea._nBottom < Row());
	#endif
		for(int i=rcArea._nTop,ic=rcArea.Height(); ic; ic--,i++)
			memset(&_ppA[i][rcArea._nLeft],0xFF,nBytes);
		return *this;
	}

	KImageGray igMask(Row(),Col());
	igMask.Activated();
	igMask.Cleared(rcArea);

	for(int i=0,ii=Row(); ii; i++,ii--)
		for(int j=0,jj=Col(); jj; j++,jj--)
			_ppA[i][j] |= igMask._ppA[i][j];

	return *this;
}

void KImageGray::Place(int nX,int nY,const KImageGray& igImg)
{
#ifdef _DEBUF
	assert(igImg.Col() + nX <= Col() && igImg.Row() + nY <= Row());
#endif
	long lSize = sizeof(unsigned char)*igImg.Col();

	for(int i=nY,ii=0; ii<igImg.Row(); i++,ii++)
		memcpy(&_ppA[i][nX],igImg._ppA[ii],lSize);
}

KImageGray KImageGray::Crop(int jx, int iy, int nWidth, int nHeight) const

{
	KImageGray igOut(nHeight,nWidth);
	for(int i=iy,ii=0; ii<nHeight; i++,ii++)
		memcpy(igOut[ii],&(_ppA[i][jx]),nWidth);

	return igOut;
}

KImageGray KImageGray::Rotate(const double& dRad,const double& dCx, const double& dCy) const
{
	//bilinear interpolation
    int    x0, x1, y0, y1;
    int    nWidth = Col(), nHeight = Row();
    double dTmpX,dTmpY,x,y,dx0,dy0,dx1,dy1;
    double dSin = sin(dRad), dCos = cos(dRad);
    double dUo = dCos * dCx - dSin*dCy - dCx;
    double dVo = dSin * dCx + dCos*dCy - dCy;

	 //init. output
	KImageGray igOut(nHeight,nWidth);

	for(int i=0; i<nHeight; i++)
	{
		dTmpX  = dSin*(dVo+i);
		dTmpY  = dCos*(dVo+i);

		for(int j=0; j<nWidth; j++)
		{
			x =  dCos*(dUo+j) + dTmpX;
			y = -dSin*(dUo+j) + dTmpY;

			if(x<0 || x>nWidth-1 || y<0 || y>nHeight-1)
			{
				igOut[i][j] = 0;
				continue;
			}

			igOut[i][j] = BilinearInterpolation(x,y);
		}
	}
	return igOut;
}

KImageGray KImageGray::Rotate(const double& dRad,const KPoint& ptCen) const
{
	//bilinear interpolation
    int    x0, x1, y0, y1;
    int    nWidth = Col(), nHeight = Row();
    double dTmpX,dTmpY,x,y,dx0,dy0,dx1,dy1;
    double dSin = sin(dRad), dCos = cos(dRad);
    double dUo = dCos * ptCen._dX - dSin*ptCen._dY - ptCen._dX;
    double dVo = dSin * ptCen._dX + dCos*ptCen._dY - ptCen._dY;

	 //init. output
	KImageGray igOut(nHeight,nWidth);

	for(int i=0; i<nHeight; i++)
	{
		dTmpX  = dSin*(dVo+i);
		dTmpY  = dCos*(dVo+i);

		for(int j=0; j<nWidth; j++)
		{
			x =  dCos*(dUo+j) + dTmpX;
			y = -dSin*(dUo+j) + dTmpY;

			if(x<0 || x>nWidth-1 || y<0 || y>nHeight-1)
			{
				igOut[i][j] = 0;
				continue;
			}

			igOut[i][j] = BilinearInterpolation(x,y);
		}
	}
	return igOut;
}

KImageGray KImageGray::Transform(const KMatrix& mHomography) const
{
    int         nRow = Row(), nCol = Col();
    KImageGray  igOut(nRow, nCol);
    KVector     vUV;
    KMatrix     mHinv = ~mHomography;

    for(int r=0, rr=nRow; rr; r++, rr--)
        for(int c=0, cc=nCol; cc; c++, cc--)
        {
            vUV = mHinv * KVector((double)c,(double)r,1.0);
            if(vUV[2] == 0.0) continue;
            vUV /= vUV[2];

            if(vUV[0] < 0.0 || vUV[1] < 0.0 || vUV[0] > nCol-1 || vUV[1] > nRow-1)
                continue;

            igOut._ppA[r][c] = BilinearInterpolation(vUV[0],vUV[1]);
        }

    return igOut;
}

KImageGray& KImageGray::Transformed(const KMatrix& mHomography)
{
    int         nRow = Row(), nCol = Col();
    KImageGray  igOut(nRow, nCol);
    KVector     vUV;
    KMatrix     mHinv = ~mHomography;

    for(int r=0, rr=nRow; rr; r++, rr--)
    {
        for(int c=0, cc=nCol; cc; c++, cc--)
        {
            vUV = mHinv * KVector((double)c,(double)r,1.0);
            if(vUV[2] == 0.0) continue;
            vUV /= vUV[2];

            if(vUV[0] < 0.0 || vUV[1] < 0.0 || vUV[0] > nCol-1 || vUV[1] > nRow-1)
                continue;

            igOut._ppA[r][c] = (unsigned char)BilinearInterpolation(vUV[0],vUV[1]);
        }
    }
    memcpy(_ppA[0], igOut.Address(), Size()*sizeof(unsigned char));

    return *this;
}

/*
void KImageGray::IntegralImage(KArrayDWord& arInt) const
{
    int nRow = Row(), nCol = Col();

    //first row
    arInt.Create(nRow,nCol);
    arInt._ppA[0][0]  = _ppA[0][0];

    for(int j=1; j<nCol; j++)
        arInt._ppA[0][j]  = arInt[0][j-1]  + _ppA[0][j];


    //recursion
    unsigned int uiAccu;
    for(int i=1; i<nRow; i++)
    {
        uiAccu =  0;

        for(int j=0; j<nCol; j++)
        {
            uiAccu       +=  _ppA[i][j];
            arInt[i][j]  =   arInt[i-1][j] + uiAccu;
        }
    }
}

void KImageGray::IntegralImage2(KArrayDWord& arInt, KArrayDWord& arInt2) const
{
    int nRow = Row(), nCol = Col();

    //first row
    arInt.Create(nRow,nCol);
    arInt2.Create(nRow,nCol);

    arInt._ppA[0][0]  = _ppA[0][0];
    arInt2._ppA[0][0] = _SQR(_ppA[0][0]);

    for(int j=1; j<nCol; j++)
    {
        arInt._ppA[0][j]  = arInt[0][j-1]  + _ppA[0][j];
        arInt2._ppA[0][j] = arInt2[0][j-1] + _SQR(_ppA[0][j]);
    }

    //recursion
    unsigned int uiAccu, uiAccu2;
    for(int i=1; i<nRow; i++)
    {
        uiAccu = uiAccu2 = 0;

        for(int j=0; j<nCol; j++)
        {
            uiAccu       +=  _ppA[i][j];
            uiAccu2      +=  _SQR(_ppA[i][j]);
            arInt[i][j]  =   arInt[i-1][j] + uiAccu;
            arInt2[i][j] =   arInt2[i-1][j] + uiAccu2;
        }
    }
}
*/

void KImageGray::Crop(const KRect& rcCrop,KImageGray& igOut) const
{
	int nRow = rcCrop.Height();
	int nCol = rcCrop.Width();

	igOut.Create(nRow,nCol);
	for(int i=rcCrop._nTop,ii=0; i<=rcCrop._nBottom; i++,ii++)
		memcpy(igOut[ii],&(_ppA[i][rcCrop._nLeft]),nCol);
}

KImageGray KImageGray::Crop(const KRect& rcCrop) const
{
	int nRow = rcCrop.Height();
	int nCol = rcCrop.Width();

	KImageGray igOut(nRow,nCol);
	for(int i=rcCrop._nTop,ii=0; i<=rcCrop._nBottom; i++,ii++)
		memcpy(igOut[ii],&(_ppA[i][rcCrop._nLeft]),nCol);
	return igOut;
}

void KImageGray::Crop(int nXi,int nYi, int nWidth,int nHeight,KImageGray& igOut) const
{
	igOut.Create(nHeight,nWidth);
	 for(int i=nYi,ii=0; ii<nHeight; i++,ii++)
		memcpy(igOut[ii],&(_ppA[i][nXi]),nWidth);
}

void KImageGray::Crop(int nXi,int nYi, int nWidth,int nHeight,KVector& vOut) const
{
	vOut.Create(nHeight*nWidth);
	for(int i=nYi,ii=0,idx=0; ii<nHeight; i++,ii++)
		for(int j=nXi,jj=0; jj<nWidth; j++,jj++)
			vOut._ppA[idx++][0] = _ppA[i][j];
}

void KImageGray::HalfSize(KImageGray& igHalf)
{
	int             nRow = (Row()>>1);
	int             nCol = (Col()>>1);

	igHalf.Create(nRow,nCol);

	for(int i=0,ii=nRow; ii; i++,ii--)
		for(int j=0,jj=nCol; jj; j++,jj--)
			igHalf._ppA[i][j] = _ppA[i<<1][j<<1];
}

KImageGray KImageGray::HalfSize()
{
	int             nRow = (Row()>>1);
	int             nCol = (Col()>>1);

	KImageGray		igHalf(nRow,nCol);

	for(int i=0,ii=nRow; ii; i++,ii--)
		for(int j=0,jj=nCol; jj; j++,jj--)
			igHalf._ppA[i][j] = _ppA[i<<1][j<<1];

	return igHalf;
}
//---------------------------------------------------------------------------


KVector KImageGray::Vectorize(bool bMedian)

{
#ifdef _DEBUG
	assert(_ppA !=0 );
#endif

	int i,ii,j,idx;
	int nRow = Row();
	int nCol = Col();

	//create an output vector
	KVector vOut(Size());

    //set the output vector
	if(!bMedian){
        for(i=0,idx=0; i<nRow; i++)
            for(j=0; j<nCol; j++)
                vOut._ppA[idx++][0] = (double)_ppA[i][j];
    }

	//median filtering
	else{
		double  dpItems[9],dTmp;
		for(i=1,ii=nCol; i<nRow-1; i++,ii+=nCol){
			for(j=1; j<nCol-1; j++){
				//get 3X3 patch
				dpItems[0]=_ppA[i-1][j-1]; dpItems[1]=_ppA[i-1][j]; dpItems[2]=_ppA[i-1][j+1];
				dpItems[3]=_ppA[i][j-1];   dpItems[4]=_ppA[i][j];   dpItems[5]=_ppA[i][j+1];
				dpItems[6]=_ppA[i+1][j-1]; dpItems[7]=_ppA[i+1][j]; dpItems[8]=_ppA[i+1][j+1];
				//sort
				for(int m=0; m<5; m++)
					for(int n=m+1; n<9; n++)
						if(dpItems[m] < dpItems[n]){
							dTmp       = dpItems[m];
							dpItems[m] = dpItems[n];
							dpItems[n] = dTmp;
						}
				//set median value
				vOut._ppA[ii+j][0] = dpItems[4];
            }
        }
        for(j=0,ii=nCol*(nRow-1); j<nCol; j++){
            vOut._ppA[j][0]    = _ppA[0][j];
            vOut._ppA[ii+j][0] = _ppA[nRow-1][j];
        }
        for(i=0,ii=0; i<nRow; i++,ii+=nCol){
            vOut._ppA[ii][0]        = _ppA[i][0];
			vOut._ppA[ii+nCol-1][0] = _ppA[i][nCol-1];
		}
	}

    return vOut;
}

void KImageGray::Vectorize(KVector& vOut,bool bMedian)
{
#ifdef _DEBUG
	assert(_ppA !=0 );
#endif

	int i,j,idx,ii;
	int nRow = Row();
	int nCol = Col();

	//create an output vector
	vOut.Create(Size());

	//set the output vector
	if(!bMedian){
		for(i=0,idx=0; i<nRow; i++)
			for(j=0; j<nCol; j++)
				vOut._ppA[idx++][0] = (double)(_ppA[i][j]);
	}
	//median filtering
	else{
		double  dpItems[9],dTmp;
		for(i=1,ii=nCol; i<nRow-1; i++,ii+=nCol){
			for(j=1; j<nCol-1; j++){
				//get 3X3 patch
                dpItems[0]=_ppA[i-1][j-1]; dpItems[1]=_ppA[i-1][j]; dpItems[2]=_ppA[i-1][j+1];
                dpItems[3]=_ppA[i][j-1];   dpItems[4]=_ppA[i][j];   dpItems[5]=_ppA[i][j+1];
                dpItems[6]=_ppA[i+1][j-1]; dpItems[7]=_ppA[i+1][j]; dpItems[8]=_ppA[i+1][j+1];
                //sort
				for(int m=0; m<5; m++)
                    for(int n=m+1; n<9; n++)
                        if(dpItems[m] < dpItems[n]){
                            dTmp       = dpItems[m];
							dpItems[m] = dpItems[n];
                            dpItems[n] = dTmp;
                        }
				//set median value
                vOut._ppA[ii+j][0] = dpItems[4];
            }
        }
        for(j=0,ii=nCol*(nRow-1); j<nCol; j++){
            vOut._ppA[j][0]    = _ppA[0][j];
            vOut._ppA[ii+j][0] = _ppA[nRow-1][j];
        }
        for(i=0,ii=0; i<nRow; i++,ii+=nCol){
            vOut._ppA[ii][0]        = _ppA[i][0];
			vOut._ppA[ii+nCol-1][0] = _ppA[i][nCol-1];
        }
    }
}
//---------------------------------------------------------------------------


double KImageGray::LoGFiltered(int nWinSize,double dSigma,double dOffset)
{
#ifdef _DEBUG
    assert(_ppA !=0 && nWinSize%2 != 0);
#endif

    //create exponential tables
    int         nHalf  = nWinSize/2;
	double      dVar   = _SQR(dSigma);
    double      dConst = -1.0/(_PI*_SQR(dVar));

    KMatrix     mMask(nWinSize,nWinSize);
    int         i,j,r,c,rr,cc;

	for(r=-nHalf,rr=0; r<=nHalf; r++,rr++)
        for(c=-nHalf,cc=0; c<=nHalf; c++,cc++)
            mMask[rr][cc] = dConst*( 1.0-(r*r+c*c)*0.5/dVar )*exp(-(r*r+c*c)*0.5/dVar);
#ifdef _DEBUG
    KMatrix mTmp = mMask*1.0/mMask[0][1];
    mTmp.WriteText("mMask.txt","%12.6f");
#endif

    //convolution
    KImageDouble idOut;
	double dTmp,dMin=10000000;

    Convolution(mMask,idOut);
	*this = (unsigned char)0;
              

	for(i=nHalf; i<Row()-nHalf; i++)
		for(j=nHalf; j<Col()-nHalf; j++)
		{
			dMin       = _MIN(idOut[i][j],dMin);
			dTmp       = idOut[i][j] + dOffset;
			_ppA[i][j] = (unsigned char)(dTmp > 255.0 ? 255 : (dTmp<0.0 ? 0 : dTmp));
		}

	return dMin;
}
//---------------------------------------------------------------------------

KImageGray KImageGray::ContrastTransform(const int& nA, const int& nB,const int& nAp, const int& nBp)
{
#ifdef _DEBUG
    assert(nB > nA && nB<256 && nA<256);
#endif

    KImageGray igOut(*this);
    double       dXin, dSlope = (double)(nBp-nAp) / (double)(nB-nA); 

    for(int i=0,ic=Row(); ic; i++,ic--)
        for(int j=0,jc=Col(); jc; j++,jc--)
        {
            dXin = (_ppA[i][j] < nA ? nA : (_ppA[i][j]>nB ? nB : _ppA[i][j]));
            igOut._ppA[i][j] = (unsigned char)(dSlope * (dXin-nA) + nAp);
        }

    return igOut;
}
//---------------------------------------------------------------------------

void KImageGray::BilateralFiltered(const double& dSigmaS, const double& dSigmaR)
{
#ifdef _DEBUG
	assert(_ppA !=0);
#endif

	//create exponential tables
	int         nHalfS = ( 3.0*dSigmaS < 2.5 ? 2:(int)(3.0*dSigmaS+0.5) );
	double      dVarS  = _SQR(dSigmaS);
	int         nHalfR = ( 3.0*dSigmaR < 2.5 ? 2:(int)(3.0*dSigmaR+0.5) );
	double      dVarR  = _SQR(dSigmaR);
	KMatrix     mKernelS(2*nHalfS+1,2*nHalfS+1);
	KVector     vKernelR(nHalfR+1);

	 for(int r=-nHalfS,rr=0; r<=nHalfS; r++,rr++)
		for(int c=-nHalfS,cc=0; c<=nHalfS; c++,cc++)
			mKernelS[rr][cc] = exp(-0.5*(r*r+c*c)/dVarS);
	for(int i=0; i<=nHalfR; i++)
		vKernelR[i] = exp(-0.5*i*i/dVarR);

	//filtering
	KImageGray  igTmp = *this;
	double      dScale,dWeight,dSum,dDiff;
	int         nSy   = nHalfS, nEy = Row() - nHalfS;
	int         nSx   = nHalfS, nEx = Col() - nHalfS;

	for(int i=nSy; i<nEy; i++)
		for(int j=nSx; j<nEx; j++)
		{
			//weighted mean
			dScale = dSum = 0.0;
			for(int r=-nHalfS,rr=0; r<=nHalfS; r++,rr++){
				for(int c=-nHalfS,cc=0; c<=nHalfS; c++,cc++)
				{
					if((dDiff=_DIFF(_ppA[i+r][j+c],_ppA[i][j])) > nHalfR + 1)
						dWeight = 0.0;
					else
						dWeight  = mKernelS[rr][cc] * vKernelR[(int)dDiff];
					dSum    += _ppA[i+r][j+c]*dWeight;
					dScale  += dWeight;
				}
			}
			igTmp[i][j] = (unsigned char)(dSum / dScale + 0.5);
		}

	//overwrite the filtering result
	memcpy(_ppA[0],igTmp.Pointer(),igTmp.Size());
}
//---------------------------------------------------------------------------

//bilinear interpolation
unsigned char KImageGray::BilinearInterpolation(const double& dU,const double& dV) const
{
	double	dWx0,dWx1,dWy0,dWy1;;
	int		nX0,nX1,nY0,nY1;

	nX0 = (int)dU;  nY0 = (int)dV;
	nX1 = nX0 + 1;  nY1 = nY0 + 1;
	dWx0 = dU - nX0; dWx1 = nX1 - dU;
	dWy0 = dV - nY0; dWy1 = nY1 - dV;

	if(dWx0 && dWy0)
		return (unsigned char)(dWx1*(dWy1*_ppA[nY0][nX0] + dWy0*_ppA[nY1][nX0]) + dWx0*(dWy1*_ppA[nY0][nX1] + dWy0*_ppA[nY1][nX1]));
	else if(dWx0 && dWy0==0.0)
		return (unsigned char)(dWx1*_ppA[nY0][nX0] + dWx0*_ppA[nY0][nX1]);
	else if(dWx0==0.0 && dWy0)
		return (unsigned char)(dWy1*_ppA[nY0][nX0] + dWy0*_ppA[nY1][nX0]);
	else
		return _ppA[nY0][nX0];
}

KImageGray& KImageGray::LogarithmTransformed()
{
    double dMax = 0.0, dC;

    //determination of scale factor
    for(int i=0,ic=Row(); ic; i++,ic--)
        for(int j=0,jc=Col(); jc; j++,jc--)
            dMax = (_ppA[i][j] > dMax ? _ppA[i][j] : dMax);
    dC = 255.0/(1.0 + dMax);

    for(int i=0,ic=Row(); ic; i++,ic--)
        for(int j=0,jc=Col(); jc; j++,jc--)
            _ppA[i][j] = (unsigned char)( dC * log(1.0+(double)_ppA[i][j]) );

    return *this;
}

void KImageGray::MaxValueFiltered(int nMaskSize)
{
	int           nHalf = (nMaskSize%2 == 1 ? nMaskSize/2 : (nMaskSize+1)/2);
	int           nSx = nHalf, nEx = Col()-nHalf;
	int           nSy = nHalf, nEy = Row()-nHalf;
	unsigned char uMax;
	KImageGray    igTmp(*this);

	for(int i=nSy; i<nEy; i++)
		for(int j=nSx; j<nEx; j++)
		{
			uMax = 0;
			for(int m=-nHalf; m<=nHalf; m++)
                for(int n=-nHalf; n<=nHalf; n++)
                    if(igTmp[i][j] > uMax)
                        uMax = igTmp[i+m][j+n];
            //set maximum value
            _ppA[i][j] = uMax;
        }
}

void KImageGray::MinValueFiltered(int nMaskSize)
{
    int           nHalf = (nMaskSize%2 == 1 ? nMaskSize/2 : (nMaskSize+1)/2);
    int           nSx = nHalf, nEx = Col()-nHalf;
    int           nSy = nHalf, nEy = Row()-nHalf;
    unsigned char uMin;
	KImageGray    igTmp(*this);

    for(int i=nSy; i<nEy; i++)
        for(int j=nSx; j<nEx; j++)
        {
            uMin = 255;
            for(int m=-nHalf; m<=nHalf; m++)
                for(int n=-nHalf; n<=nHalf; n++)
                    if(igTmp[i][j] < uMin)
						uMin = igTmp[i+m][j+n];
            //set maximum value
            _ppA[i][j] = uMin;
        }
}
//---------------------------------------------------------------------------

KImageGray& KImageGray::GaussianSmoothed(const double& dSigma)
{
    int          nSy,nEy, nRow = Row();
    int          nSx,nEx, nCol = Col();
    KVector      vKernel = KGaussianMulti::Kernel_1D(dSigma);
    KImageDouble idOut(nRow,nCol);
    int          nHalf = vKernel.Dim()/2;
    double       dConv;

    nSx = nSy = nHalf;
    nEx = nCol - nHalf;
    nEy = nRow - nHalf;

    for(int i=0; i<nRow; i++)
        for(int j=nSx; j<nEx; j++)
        {
            dConv = 0.0;
            for(int k=-nHalf,kk=0; k<=nHalf; k++,kk++)
                dConv   += _ppA[i][j+k] * vKernel[kk]; //X 방향으로 미분

            idOut._ppA[i][j] = dConv;
        }

    //출력 이미지 초기화
    memset(_ppA[0], 0, Size()*sizeof(unsigned char));


    for(int i=nSy; i<nEy; i++)
        for(int j=nSx; j<nEx; j++)
        {
            dConv = 0.0;
            for(int k=-nHalf,kk=0; k<=nHalf; k++,kk++)
                dConv   += idOut._ppA[i+k][j] * vKernel[kk]; //Y 방향으로 미분

            _ppA[i][j] = (unsigned char)dConv;
        }

    //결과 반환
    return *this;
}

KImageGray KImageGray::GaussianSmoothing(const double& dSigma)
{
    return KImageGray(*this).GaussianSmoothed(dSigma);
}
//---------------------------------------------------------------------------

KImageGray& KImageGray::MedianFiltered(int nWindow)
{
#ifdef _DEBUG
    assert(_ppA !=0 );
#endif

    //buffering
    KImageGray igTmp(*this);

    //median filtering
    int		nHalf = nWindow/2;
    int		nMid  = _SQR((nHalf<<1)+1)/2;
    vector<double> lItem;
    
    for(unsigned int i=nHalf; i<Row()-nHalf; i++)
        for(unsigned int j=nHalf; j<Col()-nHalf; j++)
        {
            lItem.clear();
            for(int m=-nHalf; m<=nHalf; m++)
                for(int n=-nHalf; n<=nHalf; n++)
                    lItem.push_back(igTmp._ppA[i+m][j+n]);
            //sort
            std::sort(lItem.begin(),lItem.end());

            //set the median value
            _ppA[i][j] = (unsigned char)lItem[nMid];
    	}

    return *this;
}

KImageGray KImageGray::MedianFiltering(int nWindow)
{
#ifdef _DEBUG
    assert(_ppA !=0 );
#endif

    KImageGray igOut(*this);

    return igOut.MedianFiltered(nWindow);
}

//---------------------------------------------------------------------------


void KImageGray::Convolution(const KArray<double>& arMask,KImageDouble& idOut)
{
    //output
    idOut.Create(Row(),Col());

    //compute magnitude and direction of edge
	int     nHalf = arMask.Col()/2;
    int     nEy   = Row() - nHalf;
    int     nEx   = Col() - nHalf;
    double  dTmp;

    for(int i=nHalf; i<nEy; i++)
        for(int j=nHalf; j<nEx; j++){
             //convolution
			 dTmp = 0.0;
		     for(int r=-nHalf,rr=0; r<=nHalf; r++,rr++)
                for(int c=-nHalf,cc=0; c<=nHalf; c++,cc++)
					dTmp  += _ppA[i+r][j+c]*arMask._ppA[rr][cc];
             idOut._ppA[i][j] = dTmp;
        }
}

KImageGray& KImageGray::Convoluted(const KVector& vKernel, const int& nAxis)
{
    int     nHalf = vKernel.Dim()/2;
    int     nSx = nHalf, nEx = Col() - nHalf;
    int     nSy = nHalf, nEy = Row() - nHalf;
    double  dConv;
    unsigned char* upTmp = new unsigned char[_MAX(Row(), Col())];

    if(nAxis == _X_AXIS)
    {
        for(int i=nSy; i<nEy; i++)
        {
            memcpy(upTmp, _ppA[i], Col());
            for(int j=nSx; j<nEx; j++)
            {
                dConv = 0.0;
                for(int k=-nHalf,kk=0; k<=nHalf; k++,kk++)
                    dConv += upTmp[j+k] * vKernel[kk];

                _ppA[i][j] = (unsigned char)dConv;
            }
        }
    }
    else if(nAxis == _Y_AXIS)
    {
        for(int j = nSx; j < nEx; j++)
        {
            for(unsigned int r = 0; r < Row(); r++)
                upTmp[r] = _ppA[r][j];

            for(int i = nSy; i < nEy; i++)
            {
                dConv = 0.0;
                for(int k=-nHalf,kk=0; k<=nHalf; k++,kk++)
                    dConv += upTmp[i+k] * vKernel[kk];

                _ppA[i][j] = (unsigned char)dConv;
            }
        }
    }
    delete[] upTmp;

    return *this;
}
//---------------------------------------------------------------------------

KImageGray KImageGray::BinaryDilate(const int& nType)
{
	KImageGray  igOut = *this;

    if(nType == _FOREGROUND)
    {
		for(int i=1,ii=Row()-2; ii; i++,ii--)
			for(int j=1,jj=Col()-2; jj; j++,jj--)
			{
				if(_ppA[i][j] == 0)
					if(_ppA[i-1][j-1] || _ppA[i-1][j] || _ppA[i-1][j+1] || _ppA[i][j-1]
							|| _ppA[i][j+1] || _ppA[i+1][j-1] || _ppA[i+1][j] || _ppA[i+1][j+1])

						igOut._ppA[i][j] = 255;
			}
    }
    else if(nType == _BACKGROUND)
    {
		for(int i=1,ii=Row()-2; ii; i++,ii--)
			for(int j=1,jj=Col()-2; jj; j++,jj--)
			{
				if(_ppA[i][j])
					if(_ppA[i-1][j-1]==0 || _ppA[i-1][j]==0 || _ppA[i-1][j+1]==0 || _ppA[i][j-1]==0
							|| _ppA[i][j+1]==0 || _ppA[i+1][j-1]==0 || _ppA[i+1][j]==0==0 || _ppA[i+1][j+1]==0)

						igOut._ppA[i][j] = 0;
			}
    }

	return igOut;
}


void KImageGray::BinaryDilated(const int& nType)
{
	KImageGray  igOut = *this;

    if(nType == _FOREGROUND)
    {
		for(int i=1,ii=Row()-2; ii; i++,ii--)
			for(int j=1,jj=Col()-2; jj; j++,jj--)
			{
				if(_ppA[i][j] == 0)
					if(_ppA[i-1][j-1] || _ppA[i-1][j] || _ppA[i-1][j+1] || _ppA[i][j-1]
							|| _ppA[i][j+1] || _ppA[i+1][j-1] || _ppA[i+1][j] || _ppA[i+1][j+1])
						igOut._ppA[i][j] = 255;
			}
    }
    else if(nType == _BACKGROUND)
    {
		for(int i=1,ii=Row()-2; ii; i++,ii--)
			for(int j=1,jj=Col()-2; jj; j++,jj--)
			{
				if(_ppA[i][j])
					if(_ppA[i-1][j-1]==0 || _ppA[i-1][j]==0 || _ppA[i-1][j+1]==0 || _ppA[i][j-1]==0
							|| _ppA[i][j+1]==0 || _ppA[i+1][j-1]==0 || _ppA[i+1][j]==0==0 || _ppA[i+1][j+1]==0)
						igOut._ppA[i][j] = 0;
			}
    }

	*this = igOut;
}

KImageGray KImageGray::BinaryErode(const int& nType)
{
	KImageGray  igOut = *this;

	if(nType == _FOREGROUND)
    {
		for(int i=1,ii=Row()-2; ii; i++,ii--)
			for(int j=1,jj=Col()-2; jj; j++,jj--)
			{
				if(_ppA[i][j])
					if(_ppA[i-1][j-1]==0 || _ppA[i-1][j]==0 || _ppA[i-1][j+1]==0 || _ppA[i][j-1]==0
							|| _ppA[i][j+1]==0 || _ppA[i+1][j-1]==0 || _ppA[i+1][j]==0 || _ppA[i+1][j+1]==0)

						igOut._ppA[i][j] = 0;
			}
    }
    else if(nType == _BACKGROUND)
    {
		for(int i=1,ii=Row()-2; ii; i++,ii--)
			for(int j=1,jj=Col()-2; jj; j++,jj--)
			{
				if(_ppA[i][j]==0)
					if(_ppA[i-1][j-1] || _ppA[i-1][j] || _ppA[i-1][j+1] || _ppA[i][j-1]
							|| _ppA[i][j+1] || _ppA[i+1][j-1] || _ppA[i+1][j] || _ppA[i+1][j+1])

						igOut._ppA[i][j] = 255;
			}
    }

    return igOut;
}

void KImageGray::BinaryEroded(const int& nType)
{
	KImageGray  igOut = *this;

    if(nType == _FOREGROUND)
    {
		for(int i=1,ii=Row()-2; ii; i++,ii--)
			for(int j=1,jj=Col()-2; jj; j++,jj--)
			{
				if(_ppA[i][j])
					if(_ppA[i-1][j-1]==0 || _ppA[i-1][j]==0 || _ppA[i-1][j+1]==0 || _ppA[i][j-1]==0
							|| _ppA[i][j+1]==0 || _ppA[i+1][j-1]==0 || _ppA[i+1][j]==0 || _ppA[i+1][j+1]==0)
						igOut._ppA[i][j] = 0;
			}
	}
	else if(nType == _BACKGROUND)
    {
		for(int i=1,ii=Row()-2; ii; i++,ii--)
			for(int j=1,jj=Col()-2; jj; j++,jj--)
			{
				if(_ppA[i][j]==0)
					if(_ppA[i-1][j-1] || _ppA[i-1][j] || _ppA[i-1][j+1] || _ppA[i][j-1]
							|| _ppA[i][j+1] || _ppA[i+1][j-1] || _ppA[i+1][j] || _ppA[i+1][j+1])
						igOut._ppA[i][j] = 255;
			}
    }

	*this = igOut;
}

void KImageGray::Threshold(const int& nThresh, KImageGray& igOut)
{
    igOut.Create(Row(),Col());

    unsigned char* pPixel = igOut.Address();
    for(int i=0,ii=Size(); ii; i++,ii--)
        *(pPixel++) = (_ppA[0][i] > nThresh ? 255 : 0);
}

KImageGray& KImageGray::Thresholded(const int& nThresh)
{
	for(int i=0,ii=Row(); ii; i++,ii--)
		for(int j=0,jj=Col(); jj; j++,jj--)
            _ppA[i][j] = (_ppA[i][j] > nThresh ? 255 : 0);

	return *this;
}

/*
double KEllipseActive::WG_Feature(const KImageGray& igImg,const double& dX, const double& dY)
{
	//check if the point is within the image area
}

*/



KImageGray& KImageGray::operator =  ( const unsigned char& 	c)
{
#ifdef _DEBUG
	assert(_ppA != 0);
#endif

	memset(_ppA[0],c,Size());
	return *this;
}

KImageGray& KImageGray::operator  =(const KImageColor24& icImg)
{
	if(icImg.Address())
		icImg.ColorToGray(*this);
	return *this;
}

KImageGray& KImageGray::operator  =(const KImageColor& icImg)
{
	if(icImg.Address())
		icImg.ColorToGray(*this);

    return *this;
}

/*#ifdef _WINDOWS
void KImageColor::SetBitmapInfo()
{
	_opBmpInfo = (BITMAPINFO*) new char[sizeof(BITMAPINFOHEADER)];//+ m_nColorTableEntries*sizeof(RGBQUAD) );
	memset(_opBmoInfo, 0, sizeof(BITMAPINFOHEADER) );

	_opBmpInfo->bmiHeader.biSize        = sizeof(BITMAPINFOHEADER);
	_opBmpInfo->bmiHeader.biWidth       = Col();
	_opBmpInfo->bmiHeader.biHeight      = Row();
	_opBmpInfo->bmiHeader.biPlanes      = 1;
	_opBmpInfo->bmiHeader.biBitCount    = 32;
	_opBmpInfo->bmiHeader.biCompression = BI_RGB;
	_opBmpInfo->bmiHeader.biClrUsed     = 0;
	_opBmpInfo->bmiHeader.biClrImportant= 0;
    _opBmpInfo->bmiHeader.biSizeImage   = Size()*4
}

void KImageColor::Draw(HDC hDC,int nX,int nY)
{
#ifdef _DEBUF
	assert(_opBmpInfo != 0);
#endif

	::StretchDIBits(
		hDC, nX, nY,
		_opBmoInfo->bmiHeader.biWidth, m_pBMI->bmiHeader.biHeight,
		0, 0,
		_opBmoInfo->bmiHeader.biWidth, m_pBMI->bmiHeader.biHeight,
		Pointer(),
		_opBmoInfo,
		DIB_RGB_COLORS,
		SRCCOPY);
}
#endif
*/
//---------------------------------------------------------------------------

KImageColor& KImageColor::FromRGB(const KImageGray& igR, const KImageGray& igG, const KImageGray& igB)
{
#ifdef _DEBUG_EVENT
    assert(igR.Size() == igG.Size() && igG.Col() == igB.Col() && igG.Row() == igB.Row());
#endif

    Create(igR.Row(), igR.Col());

    for(int i=0, ii=igR.Row(); ii; i++,ii--)
        for(int j=0, jj=igR.Col(); jj; j++,jj--)
        {
            _ppA[i][j].r = igR._ppA[i][j];
            _ppA[i][j].g = igG._ppA[i][j];
            _ppA[i][j].b = igB._ppA[i][j];
        }

    return *this;
}

//bilinear interpolation
KCOLOR32 KImageColor::BilinearInterpolation(const double& dU,const double& dV) const
{
    double	 dWx0,dWx1,dWy0,dWy1;;
    int      nX0,nX1,nY0,nY1;
    KCOLOR32 oRGB;

    nX0  = (int)dU;  nY0  = (int)dV;
    nX1  = nX0 + 1;  nY1  = nY0 + 1;
    dWx0 = dU - nX0; dWx1 = nX1 - dU;
    dWy0 = dV - nY0; dWy1 = nY1 - dV;

    if(dWx0 && dWy0)
    {
        oRGB.r = (unsigned char)(dWx1*(dWy1*_ppA[nY0][nX0].r + dWy0*_ppA[nY1][nX0].r) + dWx0*(dWy1*_ppA[nY0][nX1].r + dWy0*_ppA[nY1][nX1].r));
        oRGB.g = (unsigned char)(dWx1*(dWy1*_ppA[nY0][nX0].g + dWy0*_ppA[nY1][nX0].g) + dWx0*(dWy1*_ppA[nY0][nX1].g + dWy0*_ppA[nY1][nX1].g));
        oRGB.b = (unsigned char)(dWx1*(dWy1*_ppA[nY0][nX0].b + dWy0*_ppA[nY1][nX0].b) + dWx0*(dWy1*_ppA[nY0][nX1].b + dWy0*_ppA[nY1][nX1].b));
    }
    else if(dWx0 && dWy0==0.0)
    {
        oRGB.r = (unsigned char)(dWx1*_ppA[nY0][nX0].r + dWx0*_ppA[nY0][nX1].r);
        oRGB.g = (unsigned char)(dWx1*_ppA[nY0][nX0].g + dWx0*_ppA[nY0][nX1].g);
        oRGB.b = (unsigned char)(dWx1*_ppA[nY0][nX0].b + dWx0*_ppA[nY0][nX1].b);
    }
    else if(dWx0==0.0 && dWy0)
    {
        oRGB.r = (unsigned char)(dWy1*_ppA[nY0][nX0].r + dWy0*_ppA[nY1][nX0].r);
        oRGB.g = (unsigned char)(dWy1*_ppA[nY0][nX0].g + dWy0*_ppA[nY1][nX0].g);
        oRGB.b = (unsigned char)(dWy1*_ppA[nY0][nX0].b + dWy0*_ppA[nY1][nX0].b);
    }
    else{
        oRGB.r =  _ppA[nY0][nX0].r;
        oRGB.g = _ppA[nY0][nX0].g;
        oRGB.b = _ppA[nY0][nX0].b;
    }
    return oRGB;
}

void KImageColor::Convolution(const KArray<double>& arMask,KImageTriplet& itOut)
{
	//output
	itOut.Create(Row(),Col());

	//compute magnitude and direction of edge
	int     nHalf = arMask.Col()/2;
	int     nEy   = Row() - nHalf;
	int     nEx   = Col() - nHalf;
	double  dTmpR,dTmpG,dTmpB;

	for(int i=nHalf; i<nEy; i++)
		for(int j=nHalf; j<nEx; j++)
		{
			 //convolution
			 dTmpR = dTmpG = dTmpB  = 0.0;

			 for(int r=-nHalf,rr=0; r<=nHalf; r++,rr++)
			 {
				for(int c=-nHalf,cc=0; c<=nHalf; c++,cc++)
				{
					dTmpR  += (double)(_ppA[i+r][j+c].r)*arMask._ppA[rr][cc];
					dTmpG  += (double)(_ppA[i+r][j+c].g)*arMask._ppA[rr][cc];
					dTmpB  += (double)(_ppA[i+r][j+c].b)*arMask._ppA[rr][cc];
				}
			 }
			 itOut._ppA[i][j].r = (float)dTmpR;
			 itOut._ppA[i][j].g = (float)dTmpG;
			 itOut._ppA[i][j].b = (float)dTmpB;
		}
}
//---------------------------------------------------------------------------

void KImageColor::Log(KImageTriplet& itOut)
{
	itOut.Create(Row(),Col());

	//compute magnitude and direction of edge
	int nRow = Row();
	int nCol = Col();
	for(int i=0; i<nRow; i++)
		for(int j=0; j<nCol; j++)
		{
			itOut._ppA[i][j].r = (float)log((double)_ppA[i][j].r);
			itOut._ppA[i][j].g = (float)log((double)_ppA[i][j].g);
			itOut._ppA[i][j].b = (float)log((double)_ppA[i][j].b);
		}
}
//---------------------------------------------------------------------------

KImageColor KImageColor::HalfSize()
{
    int             nRow = (Row()>>1);
    int             nCol = (Col()>>1);
    KImageColor     icOut(nRow,nCol);

    for(int i=0; i<nRow; i++)
    {
        for(int j=0; j<nCol; j++)
            icOut._ppA[i][j] = _ppA[i<<1][j<<1];
    }

    return icOut;
}

void KImageColor::Stretch(int nRow,int nCol,KImageColor& icOut)
{
	//obatin the adjusted image
	double dDx = (double)(Col())/(double)(nCol);
	double dDy = (double)(Row())/(double)(nRow);
	double dDxy= dDx*dDy;

	//create an output image array
	icOut.Create(nRow,nCol);

	//stretch
	int         nSy,nEy,nSx,nEx,nWex,nWey,idx=0;
	double      dW,dWx,dWy,dEx,dEy;
	double      dR,dG,dB;
	KCOLOR32*   pImg = icOut.Pointer();

	for(double vv=0,yy=-1.0; vv<nRow; vv++,yy+=dDy){
		nSy = (int)(yy + 1);
		nSy = (_DIFF(nSy,yy) < 1e-10 ? nSy+1 : nSy);
		nEy = (int)(yy + dDy + 1);
		dEy = yy + dDy;
		if(_DIFF(nEy,yy+dDy) < 1e-10)
			  nWey = nEy++;
		else  nWey = (int)(dEy);

		for(double uu=0.0,xx=-1.0; uu<nCol; uu++,xx+=dDx){
			nSx = (int)(xx + 1);
			nSx = (_DIFF(nSx,xx) < 1e-10 ? nSx+1 : nSx);
            nEx = (int)(xx + dDx + 1);
            dEx = xx + dDx;
            if(_DIFF(nEx,xx+dDx) < 1e-10)
                  nWex = nEx++;
            else  nWex = (int)(dEx);

            dR = dG = dB = 0.0;
			for(int ii=nSy; ii<=nEy; ii++){
                dWy = (nSy==nEy ? dDy : (ii==nSy ? nSy-yy : (ii==nEy ? dEy-nWey : 1.0)));
                if(dWy < 0.000001) continue;
                for(int jj=nSx; jj<=nEx; jj++){
                    dWx = (nSx==nEx ? dDx : (jj==nSx ? nSx-xx : (jj==nEx ? dEx-nWex : 1.0)));
                    if(dWx < 0.000001)
                        continue;
                    dW  = dWx*dWy;
                    dR += _ppA[ii][jj].r*dW;
                    dG += _ppA[ii][jj].g*dW;
                    dB += _ppA[ii][jj].b*dW;
            }}
            pImg[idx].r = (unsigned char)(dR/dDxy+0.5);
            pImg[idx].g = (unsigned char)(dG/dDxy+0.5);
            pImg[idx].b = (unsigned char)(dB/dDxy+0.5);
            idx ++;
    }}
}
//---------------------------------------------------------------------------

void KImageColor::ColorToGray(KImageGray& igGray) const
{
    int nRow = Row();
    int nCol = Col();

    igGray.Create(nRow,nCol);

    for(int i=0; i<nRow; i++)
        for(int j=0; j<nCol; j++)
            igGray[i][j] = (unsigned char)(0.299*_ppA[i][j].r+0.587*_ppA[i][j].g+0.114*_ppA[i][j].b);
}

KImageGray KImageColor::ColorToGray() const
{
    int         nRow = Row();
    int         nCol = Col();
    KImageGray  igGray(nRow,nCol);

    for(int i=0; i<nRow; i++)
        for(int j=0; j<nCol; j++)
            igGray._ppA[i][j] = (unsigned char)(0.299*_ppA[i][j].r+0.587*_ppA[i][j].g+0.114*_ppA[i][j].b);
    return igGray;
}

//---------------------------------------------------------------------------

void KImageColor::SplitInto(KImageGray& igR,KImageGray& igG,KImageGray& igB) const
{
    int nRow = Row();
    int nCol = Col();

    igR.Create(nRow,nCol);
    igG.Create(nRow,nCol);
    igB.Create(nRow,nCol);

	for(int i=0,ii=nRow; ii; i++,ii--)
        for(int j=0,jj=nCol; jj; j++,jj--)
        {
            igR._ppA[i][j] =  _ppA[i][j].r;
            igG._ppA[i][j] =  _ppA[i][j].g;
            igB._ppA[i][j] =  _ppA[i][j].b;
        }
}
//---------------------------------------------------------------------------

KImageColor KImageColor::Transform(const KMatrix& mHomography) const
{
    int         nRow = Row(), nCol = Col();
    KImageColor icOut(nRow, nCol);
    KVector     vUV;
    KCOLOR32    oColor;
    KMatrix     mHinv = ~mHomography;

    for(int r=0, rr=nRow; rr; r++, rr--)
        for(int c=0, cc=nCol; cc; c++, cc--)
        {
            vUV = mHinv * KVector((double)c,(double)r,1.0);
            if(vUV[2] == 0.0) continue;
            vUV /= vUV[2];

            if(vUV[0] < 0.0 || vUV[1] < 0.0 || vUV[0] > nCol-1 || vUV[1] > nRow-1)
                continue;

            icOut._ppA[r][c] = BilinearInterpolation(vUV[0],vUV[1]);
        }

    return icOut;
}

KImageColor& KImageColor::Transformed(const KMatrix& mHomography)
{
    int         nRow = Row(), nCol = Col();
    KImageColor icOut(nRow, nCol);
    KVector     vUV;
    KCOLOR32    oColor;
    KMatrix     mHinv = ~mHomography;

    for(int r=0, rr=nRow; rr; r++, rr--)
        for(int c=0, cc=nCol; cc; c++, cc--)
        {
            vUV = mHinv * KVector((double)c,(double)r,1.0);
            if(vUV[2] == 0.0) continue;
            vUV /= vUV[2];

            if(vUV[0] < 0.0 || vUV[1] < 0.0 || vUV[0] > nCol-1 || vUV[1] > nRow-1)
                continue;

            icOut._ppA[r][c] = BilinearInterpolation(vUV[0],vUV[1]);
        }

    memcpy(_ppA[0], icOut.Address(), Size()*sizeof(KCOLOR32));

    return *this;
}
//---------------------------------------------------------------------------

KImageGray KImageColor::RedBand()
{
    int        nSize = Size();
    KImageGray igOut(Row(),Col());

	for(int i=0; i<nSize; i++)
            igOut._ppA[0][i] =  _ppA[0][i].r;

    return igOut;
}

KImageGray KImageColor::GreenBand()
{
    int        nSize = Size();
    KImageGray igOut(Row(),Col());

    for(int i=0; i<nSize; i++)
            igOut._ppA[0][i] =  _ppA[0][i].g;

    return igOut;
}

KImageGray KImageColor::BlueBand()
{
    int        nSize = Size();
    KImageGray igOut(Row(),Col());

    for(int i=0; i<nSize; i++)
            igOut._ppA[0][i] =  _ppA[0][i].b;

    return igOut;
}
//---------------------------------------------------------------------------

void KImageColor::RGBtoIRG(KArray<KIRG>& arIRG) const
{
    int     nRow = Row();
    int     nCol = Col();
    double  dSum;
	arIRG.Create(nRow,nCol);

	for(int i=0; i<nRow; i++)
        for(int j=0; j<nCol; j++)
        {
			dSum          = (double)(_ppA[i][j].r + _ppA[i][j].g + _ppA[i][j].b);
			arIRG[i][j].i = (unsigned char)(dSum/3.0+0.5);
			arIRG[i][j].r = (float)((double)(_ppA[i][j].r) / dSum);
			arIRG[i][j].g = (float)((double)(_ppA[i][j].g) / dSum);
		}
}
//---------------------------------------------------------------------------


void KImageColor::RGBtoIRG(const int& j, const int& i, KIRG& oIRG) const
{
    int     nRow = Row();
    int     nCol = Col();
    double  dSum;

    dSum   = (double)(_ppA[i][j].r + _ppA[i][j].g + _ppA[i][j].b);

    oIRG.i = (unsigned char)(dSum/3.0+0.5);
    oIRG.r = (float)((double)(_ppA[i][j].r) / dSum);
    oIRG.g = (float)((double)(_ppA[i][j].g) / dSum);
}
//---------------------------------------------------------------------------


void KImageColor::RGBtoYUV(KArray<KYUV>& arYUV) const
{
	int nRow = Row();
	int nCol = Col();
	arYUV.Create(nRow,nCol);

	for(int i=0,ii=nRow; ii; i++,ii--)
		for(int j=0,jj=nCol; jj; j++,jj--)
		{
			arYUV[i][j].y = (unsigned char)(0.299*_ppA[i][j].r + 0.587*_ppA[i][j].g + 0.114*_ppA[i][j].b);
			arYUV[i][j].u = (unsigned char)(128.0 - 0.16874*_ppA[i][j].r - 0.3313*_ppA[i][j].g + 0.5*_ppA[i][j].b);
			arYUV[i][j].v = (unsigned char)(128.0 + 0.5*_ppA[i][j].r - 0.4187*_ppA[i][j].g - 0.0813*_ppA[i][j].b);
		}
}
//---------------------------------------------------------------------------

void KImageColor::RGBtoYUV(const int& j, const int& i, KYUV& oYUV) const
{
	oYUV.y = (unsigned char)(0.299*_ppA[i][j].r + 0.587*_ppA[i][j].g + 0.114*_ppA[i][j].b);
	oYUV.u = (unsigned char)(128.0 - 0.16874*_ppA[i][j].r - 0.3313*_ppA[i][j].g + 0.5*_ppA[i][j].b);
	oYUV.v = (unsigned char)(128.0 + 0.5*_ppA[i][j].r - 0.4187*_ppA[i][j].g - 0.0813*_ppA[i][j].b);
}
//---------------------------------------------------------------------------


void KImageColor::RGBtoYUV(KImageGray& igY,KImageGray& igU,KImageGray& igV) const
{
	int nRow = Row();
	int nCol = Col();

	igY.Create(nRow,nCol);
	igU.Create(nRow,nCol);
	igV.Create(nRow,nCol);

	for(int i=0,ii=nRow; ii; i++,ii--)
		for(int j=0,jj=nCol; jj; j++,jj--)
		{
			igY._ppA[i][j] = (unsigned char)(0.299*_ppA[i][j].r + 0.587*_ppA[i][j].g + 0.114*_ppA[i][j].b);
			igU._ppA[i][j] = (unsigned char)(128.0 - 0.16874*_ppA[i][j].r - 0.3313*_ppA[i][j].g + 0.5*_ppA[i][j].b);
			igV._ppA[i][j] = (unsigned char)(128.0 + 0.5*_ppA[i][j].r - 0.4187*_ppA[i][j].g - 0.0813*_ppA[i][j].b);
		}
}
//---------------------------------------------------------------------------


void KImageColor::RGBtoHSI(const int& x,const int& y,KHSI& oHSI) const
{
	double dR   = (double)(_ppA[y][x].r)/255.0;
	double dG   = (double)(_ppA[y][x].g)/255.0;
    double dB   = (double)(_ppA[y][x].b)/255.0;
    double dSum = dR + dG + dB;
    double dTheta;

	oHSI.i = (float)(0.299*dR+0.587*dG+0.114*dB);

	if(dR!=dG || dG!=dB)
	{
		dTheta = acos((dR-(dG+dB)*0.5) / sqrt(_SQR(dR-dG)+ (dR-dB)*(dG-dB)));
		oHSI.h = (float)((dB > dG ? _2PI-dTheta : dTheta)/_2PI);
		oHSI.s = (float)(1.0 - 3.0*_MIN(dR,_MIN(dG,dB))/dSum);
	}
	else
		oHSI.h = oHSI.s = (float)0.0;
}

void KImageColor::RGBtoHue(KImageGray& igHue) const
{
    //create a HSV image
    igHue.Create(Row(),Col());

    //conversion
    double dMin, dMax,dDiff, dHue;

    for(int i=Row(),ii=0; i; i--,ii++)
        for(int j=Col(),jj=0; j; j--,jj++)
        {
            dMin  = _MIN(_MIN(_ppA[ii][jj].r,_ppA[ii][jj].g),_ppA[ii][jj].b);
            dMax  = _MAX(_MAX(_ppA[ii][jj].r,_ppA[ii][jj].g),_ppA[ii][jj].b);
            dDiff = dMax - dMin;

            //undefined case
            if(dDiff == 0.0)
            {
                igHue._ppA[ii][jj] = 0.0;
                continue;
            }

            //hue
            if(dMax == (double)(_ppA[ii][jj].r))
                dHue = 60.0 * (double)(_ppA[ii][jj].g-_ppA[ii][jj].b)/dDiff;
            else if(dMax == (double)(_ppA[ii][jj].g))
                dHue = 60.0 * (double)(_ppA[ii][jj].b-_ppA[ii][jj].r)/dDiff + 120.0;
            else
                dHue = 60.0 * (double)(_ppA[ii][jj].r-_ppA[ii][jj].g)/dDiff + 240.0;

            dHue += (dHue < 0.0 ? 360.0 : 0.0);

            igHue._ppA[ii][jj]  = (unsigned char)((dHue / 360.0) * 255.0);
        }

}

void KImageColor::RGBtoHSI(KArray<KHSI>& arHSI) const
{
	int     nRow = Row();
	int     nCol = Col();
	double  dTheta,dSum;
	double  dR,dG,dB;

	arHSI.Create(nRow,nCol);

	for(int i=0; i<nRow; i++)
		for(int j=0; j<nCol; j++){
			dR   = (double)(_ppA[i][j].r)/255.0;
			dG   = (double)(_ppA[i][j].g)/255.0;
			dB   = (double)(_ppA[i][j].b)/255.0;
			dSum = dR + dG + dB;

			arHSI[i][j].i = (float)(0.299*dR+0.587*dG+0.114*dB);
			if(dR!=dG || dG!=dB)
			{
				dTheta = acos((dR-(dG+dB)*0.5) / sqrt(_SQR(dR-dG)+ (dR-dB)*(dG-dB)));
				arHSI[i][j].h = (float)((dB > dG ? _2PI-dTheta : dTheta)/_2PI);
				arHSI[i][j].s = (float)(1.0 - 3.0*_MIN(dR,_MIN(dG,dB))/dSum);
			}
			else
				arHSI[i][j].h = arHSI[i][j].s = (float)0.0;
		}
}
//---------------------------------------------------------------------------

void KImageColor::RGBtoHSV(KArray<KHSV>& arHSV) const
{
	//create a HSV image
	arHSV.Create(Row(),Col());

	//conversion
	double dMin, dMax,dDiff;

	for(int i=Row(),ii=0; i; i--,ii++)
		for(int j=Col(),jj=0; j; j--,jj++)
		{
			dMin  = _MIN(_MIN(_ppA[ii][jj].r,_ppA[ii][jj].g),_ppA[ii][jj].b);
			dMax  = _MAX(_MAX(_ppA[ii][jj].r,_ppA[ii][jj].g),_ppA[ii][jj].b);
			dDiff = dMax - dMin;

			//value
			arHSV[ii][jj].v = dMax;

			//undefined case
			if(dDiff == 0.0)
			{
				arHSV[ii][jj].h = -1.0;
				arHSV[ii][jj].s = 0.0;
				continue;
			}

			//saturation
			arHSV[ii][jj].s = dDiff/dMax;

			//hue
			if(dMax == (double)(_ppA[ii][jj].r))
				arHSV[ii][jj].h = 60.0 * (double)(_ppA[ii][jj].g-_ppA[ii][jj].b)/dDiff;
			else if(dMax == (double)(_ppA[ii][jj].g))
				arHSV[ii][jj].h = 60.0 * (double)(_ppA[ii][jj].b-_ppA[ii][jj].r)/dDiff + 120.0;
			else
				arHSV[ii][jj].h = 60.0 * (double)(_ppA[ii][jj].r-_ppA[ii][jj].g)/dDiff + 240.0;

			arHSV[ii][jj].h += (arHSV[ii][jj].h < 0.0 ? 360.0 : 0.0);

		}
}
//---------------------------------------------------------------------------

KImageColor& KImageColor::FromHSV(KArray<KHSV>& arHSV)
{
	float h,s,v,f,p,q,t;
	int   idx;

	Create(arHSV.Row(),arHSV.Col());

	for(int i=0, ii=arHSV.Row(); ii; i++,ii--)
		for(int j=0, jj=arHSV.Col(); jj; j++,jj--)
		{
			h = arHSV[i][j].h;
			s = arHSV[i][j].s;
			v = arHSV[i][j].v;

			if(s == 0.0)
			{
				_ppA[i][j].r = _ppA[i][j].g = _ppA[i][j].b = (h == -1.0 ? (unsigned char)v : 0);
				continue;
			}

			if(h == 360.0)
				h = 0.0;
			h /= 60.0;
			idx = (int)h;
			f = h-idx;
			p = v*(1.0-s);
			q = v*(1.0-s*f);
			t=v*(1.0-s*(1.0-f));

			switch(idx)
			{
				case 0: _ppA[i][j].r=v; _ppA[i][j].g=t; _ppA[i][j].b=p; break;
				case 1: _ppA[i][j].r=q; _ppA[i][j].g=v; _ppA[i][j].b=p; break;
				case 2: _ppA[i][j].r=p; _ppA[i][j].g=v; _ppA[i][j].b=t; break;
				case 3: _ppA[i][j].r=p; _ppA[i][j].g=q; _ppA[i][j].b=v; break;
				case 4: _ppA[i][j].r=t; _ppA[i][j].g=p; _ppA[i][j].b=v; break;
				case 5: _ppA[i][j].r=v; _ppA[i][j].g=p; _ppA[i][j].b=q; break;
			}
		}

	return *this;
}

void KImageColor::Crop(const KRect& rcCrop,KImageColor& icOut) const
{
	int nLength = rcCrop.Width()*sizeof(KCOLOR32);

	icOut.Create(rcCrop.Height(),rcCrop.Width());
	for(int i=rcCrop._nTop,ii=0; i<=rcCrop._nBottom; i++,ii++)
		memcpy(icOut[ii],&(_ppA[i][rcCrop._nLeft]),nLength);
}

void KImageColor::Crop(const KRect& rcCrop,KImageTriplet& itOut,bool bNormalize) const
{
	itOut.Create(rcCrop.Height(),rcCrop.Width());

	for(int i=rcCrop._nTop,ii=0; i<=rcCrop._nBottom; i++,ii++)
		for(int j=rcCrop._nLeft,jj=0; j<=rcCrop._nRight; j++,jj++)
		{
			if(bNormalize)
			{
				itOut._ppA[ii][jj].r = (double)(_ppA[i][j].r)/255.0;
				itOut._ppA[ii][jj].g = (double)(_ppA[i][j].g)/255.0;
				itOut._ppA[ii][jj].b = (double)(_ppA[i][j].b)/255.0;
			}
			else
			{
				itOut._ppA[ii][jj].r = (double)(_ppA[i][j].r);
				itOut._ppA[ii][jj].g = (double)(_ppA[i][j].g);
				itOut._ppA[ii][jj].b = (double)(_ppA[i][j].b);
			}
		}
}

void KImageColor::Crop(const KRect& rcCrop,KImageGray& igOut,int nType) const
{

	igOut.Create(rcCrop.Height(),rcCrop.Width());

	if(nType == _GRAY)
	{
		for(int ic=rcCrop.Height(),i=rcCrop._nTop,ii=0; ic; ic--,i++,ii++)
			for(int jc=rcCrop.Width(),j=rcCrop._nLeft,jj=0; jc; jc--,j++,jj++)
				igOut[ii][jj] = (unsigned char)(0.299*_ppA[i][j].r+0.587*_ppA[i][j].g+0.114*_ppA[i][j].b);
	}
	else if(nType == _RGB_ONEBYTE)//r:3bit | g:3bit | b:2bit
	{
		for(int ic=rcCrop.Height(),i=rcCrop._nTop,ii=0; ic; ic--,i++,ii++)
			for(int jc=rcCrop.Width(),j=rcCrop._nLeft,jj=0; jc; jc--,j++,jj++)
				igOut[ii][jj] = (unsigned char)((_ppA[i][j].r&0xE0) | ((_ppA[i][j].g&0xE0)>>3) | ((_ppA[i][j].b&0xC0)>>6));
	}
	else if(nType == _RED)
	{
		for(int ic=rcCrop.Height(),i=rcCrop._nTop,ii=0; ic; ic--,i++,ii++)
			for(int jc=rcCrop.Width(),j=rcCrop._nLeft,jj=0; jc; jc--,j++,jj++)
				igOut[ii][jj] = _ppA[i][j].r;
	}
	else if(nType == _GREEN)
	{
		for(int ic=rcCrop.Height(),i=rcCrop._nTop,ii=0; ic; ic--,i++,ii++)
			for(int jc=rcCrop.Width(),j=rcCrop._nLeft,jj=0; jc; jc--,j++,jj++)
				igOut[ii][jj] = _ppA[i][j].g;
	}
	else if(nType == _BLUE)
	{
		for(int ic=rcCrop.Height(),i=rcCrop._nTop,ii=0; ic; ic--,i++,ii++)
			for(int jc=rcCrop.Width(),j=rcCrop._nLeft,jj=0; jc; jc--,j++,jj++)
				igOut[ii][jj] = _ppA[i][j].b;
	}

}
//---------------------------------------------------------------------------

KImageColor& KImageColor::GaussianSmoothed(const double& dSigma)
{
	int     i,j,r,c,rr,cc;

	//create the conv. mask
	int     nHalf = (int)(3.0*dSigma+0.3);
	KMatrix mMask(nHalf*2 + 1,nHalf*2 + 1);

	//obtain the mask
	double  dScale = 0.0, dSigma2 = 2.0*_SQR(dSigma);
	double  dConst = 1.0 / _2PI / _SQR(dSigma);

    for(r = -nHalf,i=0; r <= nHalf; r++,i++)
    {
         for(c = -nHalf,j=0; c <= nHalf; c++,j++)
         {
            mMask[i][j] = dConst*exp(-(r*r+c*c)/dSigma2);
            dScale     += mMask[i][j];
         }
    }
    mMask /= dScale;

    //convolution
    KImageColor icTmp(*this);
    double      dR,dG,dB;
    int         nSy = nHalf, nEy = Row() - nHalf;
    int         nSx = nHalf, nEx = Col() - nHalf;

    for(i=nSy; i<nEy; i++)
    {
        for(j=nSx; j<nEx; j++)
        {
			//convolution
		    dR = dG = dB = 0.0;
			for(r = -nHalf,rr=0; r <= nHalf; r++,rr++)
			{
                for(c = -nHalf,cc=0; c <= nHalf; c++,cc++)
                {
                    dR += icTmp._ppA[i+r][j+c].r  * mMask[rr][cc];
                    dG += icTmp._ppA[i+r][j+c].g * mMask[rr][cc];
                    dB += icTmp._ppA[i+r][j+c].b * mMask[rr][cc];
                }
            }
            _ppA[i][j].r = (unsigned char)(dR);
            _ppA[i][j].g = (unsigned char)(dG);
            _ppA[i][j].b = (unsigned char)(dB);
        }
    }    

    return *this;
}
//---------------------------------------------------------------------------


KImageColor KImageColor::GaussianSmoothing(const double& dSigma)
{
    return KImageColor(*this).GaussianSmoothed(dSigma);
}
//---------------------------------------------------------------------------


void KImageColor::ToColor24(KImageColor24& icImg) const

{
    int nRow = Row();
    int nCol = Col();

    icImg.Create(nRow,nCol);
    for(int i=0; i<nRow; i++)
        for(int j=0; j<nCol; j++){
            icImg._ppA[i][j].r = _ppA[i][j].r;
            icImg._ppA[i][j].g = _ppA[i][j].g;
            icImg._ppA[i][j].b = _ppA[i][j].b;
		}
}
//---------------------------------------------------------------------------



void KImageColor24::Create(int row,int col)
{
#ifdef _DEBUG
//    assert(col%4 == 0);
#endif
    KArray<KCOLOR24>::Create(row,col);
}

void KImageColor24::Create(int row,int col,KCOLOR24* p,int t)
{
#ifdef _DEBUG
//    assert(col%4 == 0);
#endif
    KArray<KCOLOR24>::Create(row,col,p,t);
}
//---------------------------------------------------------------------------

//bilinear interpolation
KTRIPLET KImageColor24::BilinearInterpolation(const double& dU,const double& dV) const
{
    double	 dWx0,dWx1,dWy0,dWy1;;
    int                 nX0,nX1,nY0,nY1;
    KTRIPLET oRGB;

    nX0  = (int)dU;  nY0  = (int)dV;
    nX1  = nX0 + 1;  nY1  = nY0 + 1;
    dWx0 = dU - nX0; dWx1 = nX1 - dU;
    dWy0 = dV - nY0; dWy1 = nY1 - dV;

    if(dWx0 && dWy0)
    {
        oRGB.r = (dWx1*(dWy1*_ppA[nY0][nX0].r + dWy0*_ppA[nY1][nX0].r) + dWx0*(dWy1*_ppA[nY0][nX1].r + dWy0*_ppA[nY1][nX1].r));
        oRGB.g =(dWx1*(dWy1*_ppA[nY0][nX0].g + dWy0*_ppA[nY1][nX0].g) + dWx0*(dWy1*_ppA[nY0][nX1].g + dWy0*_ppA[nY1][nX1].g));
        oRGB.b = (dWx1*(dWy1*_ppA[nY0][nX0].b + dWy0*_ppA[nY1][nX0].b) + dWx0*(dWy1*_ppA[nY0][nX1].b + dWy0*_ppA[nY1][nX1].b));
    }
    else if(dWx0 && dWy0==0.0)
    {
        oRGB.r = (dWx1*_ppA[nY0][nX0].r + dWx0*_ppA[nY0][nX1].r);
        oRGB.g =(dWx1*_ppA[nY0][nX0].g + dWx0*_ppA[nY0][nX1].g);
        oRGB.b = (dWx1*_ppA[nY0][nX0].b + dWx0*_ppA[nY0][nX1].b);
    }
    else if(dWx0==0.0 && dWy0)
    {
        oRGB.r = (dWy1*_ppA[nY0][nX0].r + dWy0*_ppA[nY1][nX0].r);
        oRGB.g = (dWy1*_ppA[nY0][nX0].g + dWy0*_ppA[nY1][nX0].g);
        oRGB.b = (dWy1*_ppA[nY0][nX0].b + dWy0*_ppA[nY1][nX0].b);
    }
    else{
        oRGB.r =  _ppA[nY0][nX0].r;
        oRGB.g = _ppA[nY0][nX0].g;
        oRGB.b = _ppA[nY0][nX0].b;
    }
    return oRGB;
}


void KImageColor24::ToColor32(KImageColor& icImg) const
{
	int nRow = Row();
	int nCol = Col();

    icImg.Create(nRow,nCol);
    for(int i=0; i<nRow; i++)
        for(int j=0; j<nCol; j++){
            icImg._ppA[i][j].r = _ppA[i][j].r;
            icImg._ppA[i][j].g = _ppA[i][j].g;
            icImg._ppA[i][j].b = _ppA[i][j].b;
		}
}

KImageColor KImageColor24::ToColor32() const
{
    KImageColor  icOut;
    int               nRow = Row();
    int               nCol = Col();

    icOut.Create(nRow,nCol);

    for(int i=0; i<nRow; i++)
        for(int j=0; j<nCol; j++){
            icOut._ppA[i][j].r = _ppA[i][j].r;
            icOut._ppA[i][j].g = _ppA[i][j].g;
            icOut._ppA[i][j].b = _ppA[i][j].b;
        }
    return icOut;
}
//---------------------------------------------------------------------------


void KImageColor24::SplitInto(KImageGray& igR,KImageGray& igG,KImageGray& igB) const
{
    int nRow = Row();
    int nCol = Col();

    igR.Create(nRow,nCol);
    igG.Create(nRow,nCol);
    igB.Create(nRow,nCol);

	for(int i=0,ii=nRow; ii; i++,ii--)
        for(int j=0,jj=nCol; jj; j++,jj--)
        {
            igR._ppA[i][j] =  _ppA[i][j].r;
            igG._ppA[i][j] =  _ppA[i][j].g;
            igB._ppA[i][j] =  _ppA[i][j].b;
        }
}
//---------------------------------------------------------------------------

void KImageColor24::ColorToGray(KImageGray& igGray) const
{
    int nRow = Row();
    int nCol = Col();
	igGray.Create(nRow,nCol);

	for(int i=0; i<nRow; i++)
		for(int j=0; j<nCol; j++)
			igGray[i][j] = (unsigned char)(0.299*_ppA[i][j].r+0.587*_ppA[i][j].g+0.114*_ppA[i][j].b);
}

KImageGray KImageColor24::ColorToGray() const
{
    int         nRow = Row();
    int         nCol = Col();
    KImageGray  igGray(nRow,nCol);

    for(int i=0; i<nRow; i++)
        for(int j=0; j<nCol; j++)
            igGray._ppA[i][j] = (unsigned char)(0.299*_ppA[i][j].r+0.587*_ppA[i][j].g+0.114*_ppA[i][j].b);
    return igGray;
}
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

void KImageColor24::RGBtoYUV(KArray<KYUV>& arYUV) const
{
	int nRow = Row();
	int nCol = Col();
	arYUV.Create(nRow,nCol);

	for(int i=0,ii=nRow; ii; i++,ii--)
		for(int j=0,jj=nCol; jj; j++,jj--)
		{
			arYUV[i][j].y = (unsigned char)(0.299*_ppA[i][j].r + 0.587*_ppA[i][j].g + 0.114*_ppA[i][j].b);
			arYUV[i][j].u = (unsigned char)(128.0 - 0.16874*_ppA[i][j].r - 0.3313*_ppA[i][j].g + 0.5*_ppA[i][j].b);
			arYUV[i][j].v = (unsigned char)(128.0 + 0.5*_ppA[i][j].r - 0.4187*_ppA[i][j].g - 0.0813*_ppA[i][j].b);
		}
}
//---------------------------------------------------------------------------

void KImageColor24::RGBtoYUV(KImageGray& igY,KImageGray& igU,KImageGray& igV) const
{
	int nRow = Row();
	int nCol = Col();

	igY.Create(nRow,nCol);
	igU.Create(nRow,nCol);
	igV.Create(nRow,nCol);

	for(int i=0,ii=nRow; ii; i++,ii--)
		for(int j=0,jj=nCol; jj; j++,jj--)
		{
			igY._ppA[i][j] = (unsigned char)(0.299*_ppA[i][j].r + 0.587*_ppA[i][j].g + 0.114*_ppA[i][j].b);
			igU._ppA[i][j] = (unsigned char)(128.0 - 0.16874*_ppA[i][j].r - 0.3313*_ppA[i][j].g + 0.5*_ppA[i][j].b);
			igV._ppA[i][j] = (unsigned char)(128.0 + 0.5*_ppA[i][j].r - 0.4187*_ppA[i][j].g - 0.0813*_ppA[i][j].b);
		}
}
//---------------------------------------------------------------------------

void KImageColor24::RGBtoIRG(KArray<KIRG>& arIRG) const
{
	int     nRow = Row();
	int     nCol = Col();
	double  dSum;
	arIRG.Create(nRow,nCol);

	for(int i=0; i<nRow; i++)
		for(int j=0; j<nCol; j++){
			dSum          = (double)(_ppA[i][j].r + _ppA[i][j].g + _ppA[i][j].b);
			arIRG[i][j].i = (unsigned char)(dSum/3.0+0.5);
            arIRG[i][j].r = (float)((double)(_ppA[i][j].r) / dSum);
            arIRG[i][j].g = (float)((double)(_ppA[i][j].g) / dSum);
        }
}

void KImageColor24::RGBtoHSI(const int& x,const int& y,KHSI& oHSI) const
{
    double dR   = (double)(_ppA[y][x].r)/255.0;
    double dG   = (double)(_ppA[y][x].g)/255.0;
    double dB   = (double)(_ppA[y][x].b)/255.0;
    double dSum = dR + dG + dB;
    double dTheta;

    oHSI.i = (float)(dSum / 3.0);
    if(dR!=dG || dG!=dB)
    {
        dTheta = acos((dR-(dG+dB)*0.5) / sqrt(_SQR(dR-dG)+ (dR-dB)*(dG-dB)));
        oHSI.h = (float)((dB > dG ? _2PI-dTheta : dTheta)/_2PI);
        oHSI.s = (float)(1.0 - 3.0*_MIN(dR,_MIN(dG,dB))/dSum);
	}
    else
		oHSI.h = -1.0;
}

void KImageColor24::RGBtoHSI(KArray<KHSI>& arHSI) const
{
    int     nRow = Row();
    int     nCol = Col();
    double  dTheta,dSum;
    double  dR,dG,dB;

    arHSI.Create(nRow,nCol);

    for(int i=0; i<nRow; i++)
        for(int j=0; j<nCol; j++)
		{
			dR   = (double)(_ppA[i][j].r)/255.0;
            dG   = (double)(_ppA[i][j].g)/255.0;
            dB   = (double)(_ppA[i][j].b)/255.0;
            dSum = dR + dG + dB;

            arHSI[i][j].i = (float)(dSum / 3.0);
            if(dR!=dG || dG!=dB)
            {
                dTheta = acos((dR-(dG+dB)*0.5) / sqrt(_SQR(dR-dG)+ (dR-dB)*(dG-dB)));
                arHSI[i][j].h = (float)((dB > dG ? _2PI-dTheta : dTheta)/_2PI);
                arHSI[i][j].s = (float)(1.0 - 3.0*_MIN(dR,_MIN(dG,dB))/dSum);
            }
            else            
				arHSI[i][j].h = -1.0;
        }
}
//---------------------------------------------------------------------------

void KImageColor24::RGBtoHSV(KArray<KHSV>& arHSV) const
{
	//create a HSV image
    arHSV.Create(Row(),Col());

    //conversion
	double dMin, dMax,dDiff;

    for(int i=Row(),ii=0; i; i--,ii++)
        for(int j=Col(),jj=0; j; j--,jj++)
        {
            dMin  = _MIN(_MIN(_ppA[ii][jj].r,_ppA[ii][jj].g),_ppA[ii][jj].b);
            dMax  = _MAX(_MAX(_ppA[ii][jj].r,_ppA[ii][jj].g),_ppA[ii][jj].b);
            dDiff = dMax - dMin;

            //Value
            arHSV[ii][jj].v = dMax;

            //undefined case
            if(dDiff == 0.0)
            {
				arHSV[ii][jj].h = -1.0;
                arHSV[ii][jj].s = 0.0;
                continue;
			}
			
			//Saturation
			arHSV[ii][jj].s = dDiff/dMax;  
			
			//Hue
			if(dMax == (double)(_ppA[ii][jj].r))
				arHSV[ii][jj].h = 60.0 * (double)(_ppA[ii][jj].g-_ppA[ii][jj].b)/dDiff;
			else if(dMax == (double)(_ppA[ii][jj].g))
				arHSV[ii][jj].h = 60.0 * (double)(_ppA[ii][jj].b-_ppA[ii][jj].r)/dDiff + 120.0;            
			else
				arHSV[ii][jj].h = 60.0 * (double)(_ppA[ii][jj].r-_ppA[ii][jj].g)/dDiff + 240.0;
				
			arHSV[ii][jj].h += (arHSV[ii][jj].h < 0.0 ? 360.0 : 0.0);
			
		}
}
//---------------------------------------------------------------------------

void KImageColor24::Crop(const KRect& rcCrop,KImageColor24& icOut) const
{
	int nLength = rcCrop.Width()*sizeof(KCOLOR24);

	icOut.Create(rcCrop.Height(),rcCrop.Width());
	for(int i=rcCrop._nTop,ii=0; i<=rcCrop._nBottom; i++,ii++)
		memcpy(icOut[ii],&(_ppA[i][rcCrop._nLeft]),nLength);
}

//---------------------------------------------------------------------------


void KImageColor24::Crop(const KRect& rcCrop,KImageGray& igOut) const

{
	igOut.Create(rcCrop.Height(),rcCrop.Width());
	for(int i=rcCrop._nTop,ii=0; i<=rcCrop._nBottom; i++,ii++)
		for(int j=rcCrop._nLeft,jj=0; j<=rcCrop._nRight; j++,jj++)
			igOut[ii][jj] = (unsigned char)(0.299*_ppA[i][j].r+0.587*_ppA[i][j].g+0.114*_ppA[i][j].b);
}
//---------------------------------------------------------------------------

void KImageWord::ScaleTo(KImageGray& igImg,const unsigned short& wMax)
{
	//init. the output object
    int nRow = Row(), nCol = Col();
	igImg.Create(nRow,nCol);

	//scaling
	unsigned short* wpSource = Pointer();
	unsigned char*  upTarget = igImg.Pointer();

	if(wMax)
	{
		double dIvMax   = 255.0/(double)(wMax);

		for(int ic=igImg.Size(),i=0; ic; ic--,i++)
			upTarget[i] = (unsigned char)( _MIN(wpSource[i]*dIvMax, 255) );
	}
	else
	{
		for(int ic=igImg.Size(),i=0; ic; ic--,i++)
			upTarget[i] = (unsigned char)( _MIN(wpSource[i],255) );
	}
}

KImageWord&	KImageWord::HybridFiltered(const double& dSigmaS, const double& dSigmaR)
{
#ifdef _DEBUG
	assert(_ppA !=0);
#endif

	//create exponential tables
	int         nHalfS = ( 3.0*dSigmaS < 2.5 ? 2:(int)(3.0*dSigmaS+0.5) );
	double      dVarS  = _SQR(dSigmaS);
	int         nHalfR = ( 3.0*dSigmaR < 2.5 ? 2:(int)(3.0*dSigmaR+0.5) );
	double      dVarR  = _SQR(dSigmaR);
	KMatrix     mKernelS(2*nHalfS+1,2*nHalfS+1);
	KVector     vKernelR(nHalfR+1);

	 for(int r=-nHalfS,rr=0; r<=nHalfS; r++,rr++)
		for(int c=-nHalfS,cc=0; c<=nHalfS; c++,cc++)
			mKernelS[rr][cc] = exp(-0.5*(r*r+c*c)/dVarS);
	for(int i=0; i<=nHalfR; i++)
		vKernelR[i] = exp(-0.5*i*i/dVarR);

	//hybrid filtering
	KImageWord  	iwTmp = *this;
	KVector			vWin(mKernelS.Size()-1);
	double      	dScale,dWeight,dSum,dDiff;
	int         	nSy   = nHalfS, nEy = Row() - nHalfS;
	int         	nSx   = nHalfS, nEx = Col() - nHalfS;
	unsigned short  wMedian,wBilateral;

	for(int i=nSy; i<nEy; i++)
	{
		for(int j=nSx; j<nEx; j++)
		{
			if(_ppA[i][j] == 0)	continue;

			//bilateral filtering
			dScale = dSum = 0.0;
			for(int r=-nHalfS,rr=0; r<=nHalfS; r++,rr++)
			{
				for(int c=-nHalfS,cc=0; c<=nHalfS; c++,cc++)
				{
					if(_ppA[i+r][j+c] == 0 || (dDiff=_DIFF(_ppA[i+r][j+c],_ppA[i][j])) >= nHalfR + 1)
						continue;

					dWeight  = mKernelS[rr][cc] * vKernelR[(int)dDiff];
					dSum    += _ppA[i+r][j+c]*dWeight;
					dScale  += dWeight;
				}
			}
			wBilateral = (unsigned short)(dSum / dScale + 0.5);

			//median filtering
			for(int r=-nHalfS,n=0; r<=nHalfS; r++)
				for(int c=-nHalfS; c<=nHalfS; c++)
					if(r == c)
						continue;
					else
						vWin[n++] = _ppA[i+r][j+c];
			wMedian = (unsigned short)vWin.Median();

			//hybrid filtering
			iwTmp[i][j] = (_DIFF(wBilateral,wMedian) > (nHalfR<<4) ? wMedian : wBilateral);
		}
	}

	//overwrite the filtering result
	memcpy(_ppA[0],iwTmp.Pointer(),sizeof(unsigned short)*iwTmp.Size());

	return *this;
}

KImageWord&	KImageWord::MedianFiltered(const double& dSigmaS)
{
#ifdef _DEBUG
	assert(_ppA !=0);
#endif

	//create exponential tables
	int		nHalfS = ( 3.0*dSigmaS < 2.5 ? 2:(int)(3.0*dSigmaS+0.5) );

	//hybrid filtering
	KImageWord  	iwTmp = *this;
	KVector			vWin(_SQR((nHalfS<<1)+1)-1);
	double      	dScale,dWeight,dSum,dDiff;
	int         	nSy   = nHalfS, nEy = Row() - nHalfS;
	int         	nSx   = nHalfS, nEx = Col() - nHalfS;

	for(int i=nSy; i<nEy; i++)
		for(int j=nSx; j<nEx; j++)
		{
			//median filtering
			for(int r=-nHalfS,n=0; r<=nHalfS; r++)
				for(int c=-nHalfS; c<=nHalfS; c++)
					if(r == c)
						continue;
					else
						vWin[n++] = _ppA[i+r][j+c];

			iwTmp[i][j] = (unsigned short)vWin.Median();
		}

	//overwrite the filtering result
	memcpy(_ppA[0],iwTmp.Address(),sizeof(unsigned short)*iwTmp.Size());

	return *this;
}


KImageWord& KImageWord::BilateralFiltered(const double& dSigmaS, const double& dSigmaR)
{
#ifdef _DEBUG
	assert(_ppA !=0);
#endif

	//create exponential tables
	int         nHalfS = ( 3.0*dSigmaS < 2.5 ? 2:(int)(3.0*dSigmaS+0.5) );
	double      dVarS  = _SQR(dSigmaS);
	int         nHalfR = ( 3.0*dSigmaR < 2.5 ? 2:(int)(3.0*dSigmaR+0.5) );
	double      dVarR  = _SQR(dSigmaR);
	KMatrix     mKernelS(2*nHalfS+1,2*nHalfS+1);
	KVector     vKernelR(nHalfR+1);

	 for(int r=-nHalfS,rr=0; r<=nHalfS; r++,rr++)
		for(int c=-nHalfS,cc=0; c<=nHalfS; c++,cc++)
			mKernelS[rr][cc] = exp(-0.5*(r*r+c*c)/dVarS);
	for(int i=0; i<=nHalfR; i++)
		vKernelR[i] = exp(-0.5*i*i/dVarR);

	//filtering
	KImageWord  iwTmp = *this;
	double      dScale,dWeight,dSum,dDiff;
	int         nSy   = nHalfS, nEy = Row() - nHalfS;
	int         nSx   = nHalfS, nEx = Col() - nHalfS;

	for(int i=nSy; i<nEy; i++)
		for(int j=nSx; j<nEx; j++)
		{
			if(_ppA[i][j] == 0)	continue;

			//weighted mean
			dScale = dSum = 0.0;
			for(int r=-nHalfS,rr=0; r<=nHalfS; r++,rr++)
			{
				for(int c=-nHalfS,cc=0; c<=nHalfS; c++,cc++)
				{
					if(_ppA[i+r][j+c] == 0 || (dDiff=_DIFF(_ppA[i+r][j+c],_ppA[i][j])) >= nHalfR + 1)
						continue;

					dWeight  = mKernelS[rr][cc] * vKernelR[(int)dDiff];
					dSum    += _ppA[i+r][j+c]*dWeight;
					dScale  += dWeight;
				}
			}
			iwTmp[i][j] = (unsigned short)(dSum / dScale + 0.5);
		}

	//overwrite the filtering result
	memcpy(_ppA[0],iwTmp.Pointer(),sizeof(unsigned short)*iwTmp.Size());

	return *this;
}


KImageWord& KImageWord::operator = (const unsigned short& wIn)
{
#ifdef _DEBUG
	assert(_ppA);
#endif

	unsigned short* pA = _ppA[0];
	for(int ic=this->Size(),i=0; ic; ic--,i++)
		pA[i] = wIn;

	return *this;
}
//////////////////////////////////////////////////////////////////////////////////////////////




//reference : Nemerical Recipes in C
//functions : - Singular Value Decomposition
//return value : convergency within 30 interations
//related equation : *this = mU_(m x n) * mW_(n x n) * mV'_(n x n)
//

double KMatrix::Pythag(double a, double b)
{
	double absa,absb;
	absa=fabs(a);
	absb=fabs(b);
	if (absa > absb) return absa*sqrt(1.0+_SQR(absb/absa));
	else return (absb == 0.0 ? 0.0 : absb*sqrt(1.0+_SQR(absa/absb)));
}


bool KMatrix::SVD(KMatrix& mU,KVector& vW,KMatrix& mV)
{
	int flag,i,its,j,jj,k,l,nm;
	double anorm,c,f,g,h,s,scale,x,y,z;
	bool    bConvergency = true;
    int     nRow = Row();
	int     nCol = Col();
    KVector vRv1(nCol);

    mU = *this;
    vW.Create(nCol);
    mV.Create(nCol,nCol);

	g = scale = anorm = 0.0;

	for(i=0; i<nCol; i++)
	{
		l=i+1;
		vRv1[i]=scale*g;
		g=s=scale=0.0;
		if (i < nRow) {
			for (k=i;k<nRow;k++) scale += _ABS(mU[k][i]);
			if(scale)
			{
				for(k=i;k<nRow;k++)
                {
					mU[k][i] /= scale;
					s += mU[k][i]*mU[k][i];
				}

				f=mU[i][i];
				g = -_SIGNEX(sqrt(s),f);
				h=f*g-s;
				mU[i][i]=f-g;

				for (j=l;j<nCol;j++) {
					for (s=0.0,k=i;k<nRow;k++) s += mU[k][i]*mU[k][j];
					f=s/h;
					for (k=i;k<nRow;k++) mU[k][j] += f*mU[k][i];
				}
				for (k=i;k<nRow;k++) mU[k][i] *= scale;
			}
		}
		vW[i]=scale *g;
		g=s=scale=0.0;

		if(i <nRow && i != nCol-1)
        {
			for(k=l;k<nCol;k++)
                scale += _ABS(mU[i][k]);

			if(scale)
            {
				for (k=l;k<nCol;k++) {
					mU[i][k] /= scale;
					s += mU[i][k]*mU[i][k];
				}

				f=mU[i][l];
				g = -_SIGNEX(sqrt(s),f);
				h=f*g-s;
				mU[i][l]=f-g;

				for(k=l;k<nCol;k++)
                    vRv1[k] = mU[i][k]/h;
				for(j=l;j<nRow;j++)
                {
					for(s=0.0,k=l;k<nCol;k++)   s        += mU[j][k]*mU[i][k];
					for(k=l;k<nCol;k++)         mU[j][k] += s*vRv1[k];
				}
				for (k=l;k<nCol;k++)
                    mU[i][k] *= scale;
			}
		}
		anorm=_MAX(anorm,(_ABS(vW[i])+_ABS(vRv1[i])));
	}
	for(i=nCol-1; i>=0; i--)
    {
		if(i < nCol-1)
        {
			if(g)
            {
				for (j=l;j<nCol;j++)
                    mV[j][i]=(mU[i][j]/mU[i][l])/g;
				for (j=l;j<nCol;j++)
                {
					for(s=0.0,k=l;k<nCol;k++)
                        s += mU[i][k]*mV[k][j];
					for(k=l; k<nCol; k++)
                        mV[k][j] += s*mV[k][i];
				}
			}
			for (j=l;j<nCol;j++)
                mV[i][j]=mV[j][i]=0.0;
		}
		mV[i][i]=1.0;
		g=vRv1[i];
		l=i;
	}
	for(i=_MIN(nRow-1,nCol-1); i>=0; i--)
    {
		l=i+1;
		g=vW[i];

		for(j=l;j<nCol;j++)
            mU[i][j]=0.0;
		if(g)
        {
			g=1.0/g;
			for(j=l; j<nCol; j++)
            {
				for(s=0.0,k=l; k<nRow; k++)
					s += mU[k][i]*mU[k][j];
				f=(s/mU[i][i])*g;

				for(k=i;k<nRow;k++)
                    mU[k][j] += f*mU[k][i];
			}
			for(j=i;j<nRow;j++) mU[j][i] *= g;
		}
        else
            for(j=i; j<nRow; j++)
                mU[j][i]=0.0;
		mU[i][i]++;
	}
	for(k=nCol-1; k>=0; k--)
    {
		for(its=0; its<30; its++)
        {
			flag=1;
			for(l=k; l>=0; l--)
            {
				nm=l-1;
				if((double)(_ABS(vRv1[l])+anorm) == anorm)
				{
					flag=0;
					break;
				}
				if((double)(_ABS(vW[nm])+anorm) == anorm)
					break;
			}
			if(flag)
            {
				c=0.0;
				s=1.0;
				for(i=l; i<=k; i++)
                {
					f=s*vRv1[i];
					vRv1[i]=c*vRv1[i];
					if((double)(_ABS(f)+anorm) == anorm)
                        break;
					g=vW[i];
					h=Pythag(f,g);
					vW[i]=h;
					h=1.0/h;
					c=g*h;
					s = -f*h;
					for(j=0; j<nRow; j++)
                    {
						y=mU[j][nm];
						z=mU[j][i];
						mU[j][nm]=y*c+z*s;
						mU[j][i]=z*c-y*s;
					}
				}
			}
			z=vW[k];
			if(l == k)
            {
				if(z < 0.0)
                {
					vW[k] = -z;
					for (j=0;j<nCol;j++)
                        mV[j][k] = -mV[j][k];
				}
				break;
			}
			if(its == 29)
                bConvergency = false;

			x=vW[l];
			nm=k-1;
			y=vW[nm];
			g=vRv1[nm];
			h=vRv1[k];
			f=((y-z)*(y+z)+(g-h)*(g+h))/(2.0*h*y);
			g=Pythag(f,1.0);
			f=((x-z)*(x+z)+h*((y/(f+_SIGNEX(g,f)))-h))/x;
			c=s=1.0;

			for (j=l;j<=nm;j++)
            {
				i=j+1;
				g=vRv1[i];
				y=vW[i];
				h=s*g;
				g=c*g;
				z=Pythag(f,h);
				vRv1[j]=z;
				c=f/z;
				s=h/z;
				f=x*c+g*s;
				g = g*c-x*s;
				h=y*s;
				y *= c;

				for(jj=0; jj<nCol; jj++)
                {
					x=mV[jj][j];
					z=mV[jj][i];
					mV[jj][j]=x*c+z*s;
					mV[jj][i]=z*c-x*s;
				}
				z=Pythag(f,h);
				vW[j]=z;

				if(z)
                {
					z=1.0/z;
					c=f*z;
					s=h*z;
				}
				f=c*g+s*y;
				x=c*y-s*g;

				for(jj=0; jj<nRow; jj++)
				{
					y=mU[jj][j];
					z=mU[jj][i];
					mU[jj][j]=y*c+z*s;
					mU[jj][i]=z*c-y*s;
				}
			}
			vRv1[l]=0.0;
			vRv1[k]=f;
			vW[k]=x;
		}
	}

  	//Sorting in decending way of eingen values
	int nIdx;
	double dT;
	for(i=0; i<nCol-1; i++){
		dT = vW[nIdx=i];

		for(j=i+1; j<nCol; j++){
			if(vW[j] > dT)
				dT = vW[nIdx=j];
		}
		if(nIdx != i){
			vW.Swap(nIdx,i,_ROW);
			mU.Swap(nIdx,i,_COL);
            mV.Swap(nIdx,i,_COL);
		}
	}


    return bConvergency;
}
////////////////////////////////////////////////////////////////////

KRotation::KRotation(const double& a,const double& b,const double& c,const int& nT)
    : KMatrix(3,3)
{
    if(nT == _DEG || nT == _RAD)
        FromEuler(a,b,c,nT);
    else
        FromRodrigues(a,b,c);
}

KRotation& KRotation::FromAxisRotation(const int& nAxis,const double& dRad)
{
	//init.
	memset(_ppA[0],0,sizeof(double)*9);
	_ppA[0][0] = _ppA[1][1] = _ppA[2][2] = 1.0;

	if(nAxis == _X_AXIS){
		_ppA[1][1] = _ppA[2][2] = cos(dRad);
		_ppA[2][1] = sin(dRad);
		_ppA[1][2] = -_ppA[2][1];
	}
	else if(nAxis == _Y_AXIS){
		_ppA[0][0] = _ppA[2][2] = cos(dRad);
		_ppA[2][0] = -sin(dRad);
		_ppA[0][2] = -_ppA[2][0];
	}
	else if(nAxis == _Z_AXIS){
		_ppA[0][0] = _ppA[1][1] = cos(dRad);
		_ppA[1][0] = sin(dRad);
		_ppA[0][1] = -_ppA[1][0];
	}

	return *this;
}

//Reference  : K.S.Fu, R.C.Conzalez and C.S.G.Lee,
//             Robotics:Control,Sensing,Vision, and Intelligence, pp.21-22,
//             McGraw-Hill International Ed., 1987
//Porperties : set rotation matrix given rotation axis(vR,should be a normalized vector) and angle
//
KRotation& KRotation::FromAxisRotation(const double& nx,const double& ny,const double& nz,const double& dPhi)
{
	double dCphi= cos(dPhi);
	double dSphi= sin(dPhi);
	double dVphi= 1.0 - dCphi;

	_ppA[0][0] = nx*nx*dVphi + dCphi;    _ppA[0][1] = nx*ny*dVphi - nz*dSphi; _ppA[0][2] = nx*nz*dVphi + ny*dSphi;
	_ppA[1][0] = ny*nx*dVphi + nz*dSphi; _ppA[1][1] = ny*ny*dVphi + dCphi;    _ppA[1][2] = ny*nz*dVphi - nx*dSphi;
	_ppA[2][0] = nz*nx*dVphi - ny*dSphi; _ppA[2][1] = nz*ny*dVphi + nx*dSphi; _ppA[2][2] = nz*nz*dVphi + dCphi;

	return *this;
}

void KRotation::AxisRotation(KVector& vAxis,double& dPhi)
{
	vAxis.Create(3);
	double dC  = 0.5*(_ppA[0][0]+_ppA[1][1]+_ppA[2][2]-1.0);
	double dS  = sqrt(1.0-dC*dC)*2.0;

	vAxis[0] = (_ppA[2][1] - _ppA[1][2])/dS;
	vAxis[1] = (_ppA[0][2] - _ppA[2][0])/dS;
	vAxis[2] = (_ppA[1][0] - _ppA[0][1])/dS;
	dPhi     = acos(dC); //0 < dPhi < _PI
}


//Reference  : Slama,C.C. ed., Manual of Photogrammetry,
//             American Soc. of Photogrammetry, fourth ed., 1980
//Porperties : set rotation matrix from Rodrigues parameters
//
KRotation& KRotation::FromRodrigues(const double& a,const double& b,const double& c)
{
   //set
   double dA2     = a*a/4., dB2 = b*b/4.;
   double dC2     = c*c/4., dAB = a*b/2.;
   double dAC     = a*c/2., dBC = b*c/2.;
   double dScale  = 1.0 + dA2 + dB2 + dC2;

   _ppA[0][0] = 1.0 + dA2 - dB2 - dC2;   _ppA[0][1] = dAB - c;
   _ppA[0][2] = dAC + b;                 _ppA[1][0] = dAB + c;
   _ppA[1][1] = 1.0 - dA2 + dB2 - dC2;   _ppA[1][2] = dBC - a;
   _ppA[2][0] = dAC - b; _ppA[2][1] = dBC + a;
   _ppA[2][2] = 1.0 - dA2 - dB2 + dC2;

   for(int i=0; i<3; i++)
	  for(int j=0; j<3; j++)
		 _ppA[i][j] /= dScale;

   return *this;
}

void KRotation::Rodrigues(double& a,double& b,double& c)
{
   double dB = 8.*(_ppA[0][0]+_ppA[1][1]+_ppA[2][2])-4.*(_ppA[1][0]+_ppA[0][2]+_ppA[2][1]+
				   _ppA[0][1]+_ppA[2][0]+_ppA[1][2]);
   double dC = _SQR(_ppA[1][0]+_ppA[0][2]+_ppA[2][1]-_ppA[0][1]-_ppA[2][0]-_ppA[1][2]);
   double dS = (dB+sqrt(dB*dB+48.0*dC))/24.0;

   a = (_ppA[2][1]-_ppA[1][2]) / dS;
   b = (_ppA[0][2]-_ppA[2][0]) / dS;
   c = (_ppA[1][0]-_ppA[0][1]) / dS;
}

KVector KRotation::Rodrigues()
{
   double dB = 8.*(_ppA[0][0]+_ppA[1][1]+_ppA[2][2])-4.*(_ppA[1][0]+_ppA[0][2]+_ppA[2][1]+
				   _ppA[0][1]+_ppA[2][0]+_ppA[1][2]);
   double dC = _SQR(_ppA[1][0]+_ppA[0][2]+_ppA[2][1]-_ppA[0][1]-_ppA[2][0]-_ppA[1][2]);
   double dS = (dB+sqrt(dB*dB+48.0*dC))/24.0;

   return KVector((_ppA[2][1]-_ppA[1][2])/dS, (_ppA[0][2]-_ppA[2][0])/dS,(_ppA[1][0]-_ppA[0][1])/dS);
}


//Reference  : Dam,E.B., Koch,M. and Lillholm,M., "Quarternion,Interpolation
//             and Animation", Technical Report DIKU-TR-98/5,Dept. Computer
//             Science, Univ. of Copenhagen, Denmark
//Porperties : set rotation matrix from Euler algles as "R(Z,roll) * R(Y,pitch) * R(X,yaw)"
//
//
KRotation& KRotation::FromEuler(const double& dYaw,const double& dPitch,const double& dRoll,const int& nType)
{
   //set
   double dCosA,dCosB,dCosG,dSinA,dSinB,dSinG;

   if(nType == _DEG){
	  dCosA=cos(_RADIAN(dYaw));  dCosB=cos(_RADIAN(dPitch));
	  dCosG=cos(_RADIAN(dRoll)); dSinA=sin(_RADIAN(dYaw));
	  dSinB=sin(_RADIAN(dPitch));dSinG=sin(_RADIAN(dRoll));
   }
   else{
	  dCosA=cos(dYaw);  dCosB=cos(dPitch);
	  dCosG=cos(dRoll); dSinA=sin(dYaw);
	  dSinB=sin(dPitch);  dSinG=sin(dRoll);
   }

   _ppA[0][0] = dCosB*dCosG;
   _ppA[0][1] = dCosG*dSinA*dSinB - dCosA*dSinG;
   _ppA[0][2] = dCosA*dCosG*dSinB + dSinA*dSinG;
   _ppA[1][0] = dCosB*dSinG;
   _ppA[1][1] = dCosA*dCosG + dSinA*dSinB*dSinG;
   _ppA[1][2] = -dCosG*dSinA + dCosA*dSinB*dSinG;
   _ppA[2][0] = -dSinB;
   _ppA[2][1] = dCosB*dSinA;
   _ppA[2][2] = dCosA*dCosB;

   return *this;
}

void KRotation::Euler(double& dYaw,double& dPitch,double& dRoll,const int& nType)
{
	dPitch = asin(-_ppA[2][0]); // -PI/2 < dPitch < PI/2

	if(_DIFF(cos(dPitch),0.0) < 1.0e-10){  //impossible to distinguish dAlpa from dRoll
		dRoll = atan2(_ppA[1][2]+_ppA[0][1],_ppA[0][2]-_ppA[1][1]);
		dYaw  = dRoll - atan2(_ppA[1][2]-_ppA[0][1],_ppA[0][2]+_ppA[1][1]);
	}
	else
	{
		dRoll = atan2(_ppA[1][0],_ppA[0][0]);
		dYaw  = atan2(_ppA[2][1],_ppA[2][2]);
	}

	if(nType == _DEG)
	{
		dYaw   = _DEGREE(dYaw);
		dPitch = _DEGREE(dPitch);
		dRoll  = _DEGREE(dRoll);
	}
/*
   dPitch = asin(-_ppA[2][0]); // -PI/2 < dPitch < PI/2
   if(_DIFF(cos(dPitch),0.0) < 1.0e-10){  //impossible to distinguish dAlpa from dRoll
	  dRoll = 0.0;                      //thus, define dRoll=0
	  if(_ppA[1][1]>0) dYaw = asin(-_ppA[1][2]);
	  else             dYaw = (_ppA[1][2]<0 ? acos(_ppA[1][1]):-acos(_ppA[1][1]));
   }else{
	  //dYaw
	  if(_ppA[2][2]>0) dYaw = asin(_ppA[2][1]/cos(dPitch));
	  else             dYaw = (_ppA[2][1]>0 ? acos(_ppA[2][2]/cos(dPitch)):-acos(_ppA[2][2]/cos(dPitch)));
	  //dRoll
	  if(_ppA[0][0]>0) dRoll = asin(_ppA[1][0]/cos(dPitch));
	  else             dRoll = (_ppA[1][0]>0 ? acos(_ppA[0][0]/cos(dPitch)):-acos(_ppA[0][0]/cos(dPitch)));
   }
*/
}

KVector KRotation::Euler(const int& nType)
{
	double dYaw,dPitch,dRoll;

	dPitch = asin(-_ppA[2][0]); // -PI/2 < dPitch < PI/2

	if(_DIFF(cos(dPitch),0.0) < 1.0e-10){  //impossible to distinguish dAlpa from dRoll
		dRoll = atan2(_ppA[1][2]+_ppA[0][1],_ppA[0][2]-_ppA[1][1]);
		dYaw  = dRoll - atan2(_ppA[1][2]-_ppA[0][1],_ppA[0][2]+_ppA[1][1]);
	}
	else
	{
		dRoll = atan2(_ppA[1][0],_ppA[0][0]);
		dYaw  = atan2(_ppA[2][1],_ppA[2][2]);
	}

	if(nType == _DEG){
	  dYaw   = _DEGREE(dYaw);
	  dPitch = _DEGREE(dPitch);
	  dRoll  = _DEGREE(dRoll);
	}

	return KVector(dYaw,dPitch,dRoll);
}

KRotation& KRotation::FromQuarternion(const double& s,const double& q1,const double& q2,const double& q3)
{
   double q00=s*s, q11=q1*q1, q22=q2*q2, q33=q3*q3;
   double q01=s*q1, q02=s*q2, q03=s*q3;
   double q12=q1*q2, q13=q1*q3, q23=q2*q3;

   _ppA[0][0] = q00+q11-q22-q33; _ppA[0][1] = (q12-q03)*2.;     _ppA[0][2] = (q13+q02)*2.;
   _ppA[1][0] = (q12+q03)*2.;    _ppA[1][1] = q00-q11+q22-q33; _ppA[1][2] = (q23-q01)*2.;
   _ppA[2][0] = (q13-q02)*2.;    _ppA[2][1] = (q23+q01)*2.;     _ppA[2][2] = q00-q11-q22+q33;

   return *this;
}

void KRotation::Quarternion(double& s,double& q1,double& q2,double& q3)
{
	s = 0.5*sqrt(_ppA[0][0]+_ppA[1][1]+_ppA[2][2]+1.0);
	q1= (_ppA[2][1] - _ppA[1][2])/4.0/s;
	q2= (_ppA[0][2] - _ppA[2][0])/4.0/s;
	q3= (_ppA[1][0] - _ppA[0][1])/4.0/s;
}

void KRotation::Quarternion(KVector& vQ)
{
	vQ.Create(4);
	vQ[0] = 0.5*sqrt(_ppA[0][0]+_ppA[1][1]+_ppA[2][2]+1.0);
	vQ[1] = (_ppA[2][1] - _ppA[1][2])/4.0/vQ[0];
	vQ[2] = (_ppA[0][2] - _ppA[2][0])/4.0/vQ[0];
	vQ[3] = (_ppA[1][0] - _ppA[0][1])/4.0/vQ[0];
}

void KRotation::Angle(int nCol,double& a,double& b,double& c,int nType)
{
	a = atan2(_ppA[2][nCol],_ppA[1][nCol]);
	b = atan2(_ppA[0][nCol],_ppA[2][nCol]);
	c = atan2(_ppA[1][nCol],_ppA[0][nCol]);

	if(nType == _DEG)
	{
		a = _DEGREE(a);
		b = _DEGREE(b);
		c = _DEGREE(c);
	}
}

KRotation KRotation::Orthogonalize()
{
	KRotation rNorm;

	KMatrix mU,mV;
	KVector vW;

	SVD(mU,vW,mV);

	return KRotation(mU*mV.Tr());
}

KRotation& KRotation::Orthogonalized()
{
	KMatrix mU,mV;
	KVector vW;

	SVD(mU,vW,mV);


	return (*this = mU*mV.Tr());
}

bool KRotation::IsRightHand()
{
   double dA = (Column(0).Skew() * Column(1)).Vector() & Column(2);
   double dB = (Column(1).Skew() * Column(2)).Vector() & Column(0);

   return (dA>0.0 && dB>0.0 ? true : false);
}

KRotation&  KRotation::operator =(const KMatrix& mMat)
{
	memcpy(_ppA[0],mMat._ppA[0],9*sizeof(double));
	return *this;
}

KVector KRotation::operator*(const KPoint3D& pt)
{
	KVector vOut(3);
	vOut._ppA[0][0] = _ppA[0][0]*pt._dX + _ppA[0][1]*pt._dY + _ppA[0][2]*pt._dZ;
	vOut._ppA[1][0] = _ppA[1][0]*pt._dX + _ppA[1][1]*pt._dY + _ppA[1][2]*pt._dZ;
	vOut._ppA[2][0] = _ppA[2][0]*pt._dX + _ppA[2][1]*pt._dY + _ppA[2][2]*pt._dZ;

	return vOut;
}

KRotation KRotation::operator*(const KRotation& rRot)
{
	//Create a object for return
	KRotation rOut;

	//Multiplication
	double  dSum;
	for(int i=0; i<3; i++){
		for(int j=0; j<3; j++){
			dSum=0.0;
			for(int k=0; k<3; k++)
				dSum += _ppA[i][k] * rRot._ppA[k][j];
			rOut._ppA[i][j] = dSum;
		}
	}

	return rOut;
}

KMatrix KRotation::operator*(const KMatrix& mMat)
{
	//Create a object for return
	int nCol = mMat.Col();
	KMatrix mOut(3,nCol);

	//Multiplication
	double  dSum;
	for(int i=0; i<3; i++){
		for(int j=0; j<nCol; j++){
			dSum=0.0;
			for(int k=0; k<3; k++)
				dSum += _ppA[i][k] * mMat._ppA[k][j];
			mOut._ppA[i][j] = dSum;
		}
	}

	return mOut;
}

KVector KRotation::operator*(const KVector& vMat)
{
    //Create a object for return
    KVector vOut(3);

    //Multiplication
    for(int i=0; i<3; i++)
        vOut._ppA[i][0] = _ppA[i][0]*vMat._ppA[0][0] + _ppA[i][1]*vMat._ppA[1][0] + _ppA[i][2]*vMat._ppA[2][0];


    return vOut;
}

void KHomogeneous::Create(const KVector& vRT, int nType)
{
    if(nType == _RODRIGUES)
        Place(0,0,KRotation(vRT[0],vRT[1],vRT[2],_RODRIGUES));
    else //_EULER
        Place(0,0,KRotation(vRT[0],vRT[1],vRT[2], _RAD));

    Place(0,3,vRT.Cut(3,5));
}

KVector KHomogeneous::RT(int nType)
{
    if(nType == _EULER)
        return KRotation(this->To(2,2)).Euler(_RAD).Tailed(this->Column(3).To(2));
    else // _RODRIGUES
        return KRotation(this->To(2,2)).Rodrigues().Tailed(this->Column(3).To(2));
}

KHomogeneous& KHomogeneous::operator =(const KHomogeneous& hT)
{
	memcpy(_ppA[0],hT.Address(),sizeof(double)*16);
	return *this;
}

KHomogeneous KHomogeneous::Iv() const
{	
    KRotation    rR = this->R().Tr();
    KVector      vT = -rR*this->t();

    return KHomogeneous(rR, vT);
}

KHomogeneous KHomogeneous::operator ~() const
{
    KRotation    rR = this->R().Tr();
    KVector      vT = -rR*this->t();

    return KHomogeneous(rR, vT);
}


//////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////////


// quaternion representing the rotation of dTheta along vAxis

// dTheta : radian
// vAxis  : 3x1 unit vector
//
void KQuaternion::Create(const KVector& vAxis,const double& dTheta)
{
	double dSine = sin(dTheta*0.5);

	_ppA[0][0] = cos(dTheta*0.5);
	_ppA[1][0] = dSine * vAxis[0];
	_ppA[2][0] = dSine * vAxis[1];
	_ppA[3][0] = dSine * vAxis[2];
}

//dTheta : <0,PI>
//vAxis  : unit vector
//
void KQuaternion::AxisRotation(KVector& vAxis,double& dTheta)
{
	vAxis.Create(_ppA[1][0],_ppA[2][0],_ppA[3][0]);

	dTheta  = acos(_ppA[0][0]); // acos -> <0,PI>
	vAxis  /= sin(dTheta);
	dTheta *= 2.0;

	if(dTheta > _PI){
		dTheta = 2.0*_PI - dTheta;
		vAxis[0] = -vAxis[0];
		vAxis[1] = -vAxis[1];
		vAxis[2] = -vAxis[2];
	}
}

KRotation KQuaternion::Rotation()
{
	KRotation rThis;

    double q00=_SQR(_ppA[0][0]), q11=_SQR(_ppA[1][0]), q22=_SQR(_ppA[2][0]), q33=_SQR(_ppA[3][0]);
    double q01=_ppA[0][0]*_ppA[1][0], q02=_ppA[0][0]*_ppA[2][0], q03=_ppA[0][0]*_ppA[3][0];
    double q12=_ppA[1][0]*_ppA[2][0], q13=_ppA[1][0]*_ppA[3][0], q23=_ppA[2][0]*_ppA[3][0];

    rThis[0][0] = q00+q11-q22-q33; rThis[0][1] = (q12-q03)*2.;     rThis[0][2] = (q13+q02)*2.;
    rThis[1][0] = (q12+q03)*2.;    rThis[1][1] = q00-q11+q22-q33;  rThis[1][2] = (q23-q01)*2.;
    rThis[2][0] = (q13-q02)*2.;    rThis[2][1] = (q23+q01)*2.;     rThis[2][2] = q00-q11-q22+q33;

    return rThis;
}
//---------------------------------------------------------------------------

KQuaternion& KQuaternion::operator =(const KVector& vIn)
{
#ifdef _DEBUG
    assert(vIn.Dim() == 4);
#endif

    _ppA[0][0] = vIn[0];
    _ppA[1][0] = vIn[1];
    _ppA[2][0] = vIn[2];
    _ppA[3][0] = vIn[3];

    return *this;
}

KQuaternion KQuaternion::operator *(const KQuaternion& qIn)
{
    KQuaternion qOut;

    qOut[0] = _ppA[0][0]*qIn[0]-_ppA[1][0]*qIn[1]-_ppA[2][0]*qIn[2]-_ppA[3][0]*qIn[3];
    qOut[1] = _ppA[0][0]*qIn[1]+_ppA[1][0]*qIn[0]+_ppA[2][0]*qIn[3]-_ppA[3][0]*qIn[2];
	qOut[2] = _ppA[0][0]*qIn[2]-_ppA[1][0]*qIn[3]+_ppA[2][0]*qIn[0]+_ppA[3][0]*qIn[1];
    qOut[3] = _ppA[0][0]*qIn[3]+_ppA[1][0]*qIn[2]-_ppA[2][0]*qIn[1]+_ppA[3][0]*qIn[0];

    return qOut;
}

KQuaternion KQuaternion::operator ~() const
{
    KQuaternion qInv;

    qInv[0] = _ppA[0][0];
    qInv[1] = _ppA[1][0];
    qInv[2] = _ppA[2][0];
    qInv[3] = _ppA[3][0];

    return qInv;
}
//////////////////////////////////////////////////////////////////////////////////////////////

KPGM::KPGM(const KImageWord & iwImg)
{
	// memory alloc.
	_iwImg = iwImg;
	KImageGray::Create(iwImg.Row(),iwImg.Col());

	//for display
	unsigned short wMax = 0;
	for(int r=0,rr=iwImg.Row(); rr; r++,rr--)
		for(int c=0, cc=iwImg.Col(); cc; c++,cc--)
			wMax = (_iwImg._ppA[r][c] > wMax ? _iwImg._ppA[r][c] : wMax);

	_dScale = 255.0/(double)(wMax);
	for(int r=0,rr=_iwImg.Row(); rr; r++,rr--)
		for(int c=0, cc=_iwImg.Col(); cc; c++,cc--)
			_ppA[r][c]= (unsigned char)(_iwImg._ppA[r][c]*_dScale);
}


bool KPGM::Load(const char* szFile) //tag: gm
{
	FILE *fp;
	int  dummy;
	char buf[10];

	//create a file pointer
	if((fp=fopen(szFile,"rb")) == 0)
		return false;
	//read its ID
	fscanf(fp,"%s",buf);
	if((buf[0]!='P')|| buf[1]!='5') {
		fclose(fp);
		return false;
	}
	fgetc(fp);

	//skip comment lines if exists
	while((dummy=fgetc(fp)) == '#')
		while(fgetc(fp) != '\n');
	ungetc(dummy,fp);

	//read dimensions
	int nCol,nRow,nMax;
    fscanf(fp,"%d %d\n", &nCol,&nRow);
    fscanf(fp,"%d\n", &nMax);


	//memory alloc.
	KImageGray::Create(nRow,nCol);

	if(nMax > 255)
	{
		// memory alloc.
		_iwImg.Create(nRow,nCol);

		//read data
		if(fread(_iwImg.Address(),sizeof(unsigned short),Size(),fp) != Size()){ fclose(fp); return false; }

		unsigned short wMax = 0;
		for(int r=0,rr=nRow; rr; r++,rr--)
			for(int c=0, cc=nCol; cc; c++,cc--)
				wMax = (_iwImg._ppA[r][c] > wMax ? _iwImg._ppA[r][c] : wMax);

		_dScale = 255.0/(double)(wMax);
		for(int r=0,rr=nRow; rr; r++,rr--)
			for(int c=0, cc=nCol; cc; c++,cc--)
				_ppA[r][c]= (unsigned char)(_iwImg._ppA[r][c]*_dScale);
	}
	else
    {
        _iwImg.Release();
		fread(_ppA[0],sizeof(unsigned char),Size(),fp);
    }
	fclose(fp);

	return true;
}

void KPGM::Save(const char* szName)
{
    FILE*   fp;
    int     nMax = (_iwImg.Address() ? (unsigned short)(0xffff) : (unsigned char)(0xff));
    KString stFile(szName);

    if(stFile.FileExt() != "pgm" && stFile.FileExt() != "PGM")
        stFile += ".pgm";
	
    if((fp=fopen(stFile,"wb")) != 0)
	{
        fprintf(fp,"P5\n%d %d\n%d\n",Col(),Row(),nMax);

		if(_iwImg.Address())
			fwrite(_iwImg.Address(),sizeof(unsigned short),Size(),fp);
		else
			fwrite(_ppA[0],sizeof(unsigned char),Size(),fp);
		fclose(fp);
	}
}

void KPGM::UpdateData()
{
	if(_iwImg.Address() == 0)
        return;

	for(int r=0,rr=_iwImg.Row(); rr; r++,rr--)
		for(int c=0, cc=_iwImg.Col(); cc; c++,cc--)
			_ppA[r][c]= (unsigned char)(_iwImg._ppA[r][c]*_dScale);
}

KPGM& KPGM::operator=(const KImageGray& igImg)
{
    if(_iwImg.Address())
        _iwImg.Release();

	KImageGray::Create(igImg.Row(),igImg.Col(),igImg.Pointer());
	return *this;
}

KPGM& KPGM::operator=(const KImageWord& iwImg)
{
    // memory alloc.
	_iwImg = iwImg;
	KImageGray::Create(iwImg.Row(),iwImg.Col());

	//for display
	unsigned short wMax = 0;
	for(int r=0,rr=_iwImg.Row(); rr; r++,rr--)
		for(int c=0, cc=_iwImg.Col(); cc; c++,cc--)
			wMax = (_iwImg._ppA[r][c] > wMax ? _iwImg._ppA[r][c] : wMax);

	_dScale = 255.0/(double)(wMax);
	for(int r=0,rr=_iwImg.Row(); rr; r++,rr--)
		for(int c=0, cc=_iwImg.Col(); cc; c++,cc--)
			_ppA[r][c]= (unsigned char)(_iwImg._ppA[r][c]*_dScale);

	return *this;
}

KPGM& KPGM::operator =(KPGM& gmImg)
{
	if(gmImg.ImageWord().Address() == 0)
	{
        if(_iwImg.Address())
            _iwImg.Release();

		KImageGray::Create(gmImg.Row(),gmImg.Col(),gmImg.Address());        
		return *this;
	}

	// memory alloc.
	_iwImg = gmImg.ImageWord();
	KImageGray::Create(_iwImg.Row(),_iwImg.Col());

	//for display
	unsigned short wMax = 0;
	for(int r=0,rr=_iwImg.Row(); rr; r++,rr--)
		for(int c=0, cc=_iwImg.Col(); cc; c++,cc--)
			wMax = (_iwImg._ppA[r][c] > wMax ? _iwImg._ppA[r][c] : wMax);

	_dScale = 255.0/(double)(wMax);
	for(int r=0,rr=_iwImg.Row(); rr; r++,rr--)
		for(int c=0, cc=_iwImg.Col(); cc; c++,cc--)
			_ppA[r][c]= (unsigned char)(_iwImg._ppA[r][c]*_dScale);

	return *this;
}
//--------------------------------------------------------------------------------------------

bool KPPM::Load(const char* szFile)  //tag:pm
{
	FILE *fp;
	int  dummy;
	char buf[10];

	//create a file pointer
	if((fp=fopen(szFile,"rb")) == 0)
		return false;
	//read its ID
	fscanf(fp,"%s",buf);
	if((buf[0]!='P')|| buf[1]!='6') {
		fclose(fp);
		return false;
	}
	dummy=fgetc(fp);
	//skip comment lines if exists
	while((dummy=fgetc(fp)) == '#')
		while(fgetc(fp) != '\n');
	ungetc(dummy,fp);
	//read dimensions
	int nRow,nCol;
	fscanf(fp,"%d%d%d",&nCol,&nRow,&dummy);
	fgetc(fp);
	// memory alloc.
	KImageColor::Create(nRow,nCol);
	//read data
	KCOLOR32* p = _ppA[0];
	for(int i=0,ii=(int)Size(); ii; i++,ii--)
	{
			fread(&(p[i].r),1,1,fp);
			fread(&(p[i].g),1,1,fp);
			fread(&(p[i].b),1,1,fp);
	}
	fclose(fp);

	return true;
}

void KPPM::Save(const char* szName)
{
    FILE*   fp;
    int     dummy = 255;
    KString stFile(szName);

    if(stFile.FileExt() != "ppm" && stFile.FileExt() != "PPM")
        stFile += ".ppm";

    if((fp=fopen(stFile,"wb")) != 0){
		fprintf(fp,"P6\n%d %d\n%d\n",Col(),Row(),dummy);
		KCOLOR32* p = _ppA[0];
		for(int i=0,ii=(int)Size(); ii; i++,ii--){
			fwrite(&(p[i].r),1,1,fp);
			fwrite(&(p[i].g),1,1,fp);
			fwrite(&(p[i].b),1,1,fp);
		}
		fclose(fp);
	}
}


KPPM& KPPM::operator=(const KImageColor& oImg)
{
	KImageColor::Create(oImg.Row(),oImg.Col(),oImg.Pointer());
	return *this;
}
//--------------------------------------------------------------------------------------------


void KRandom::OnRandom(int nNum,int nType)
{
	int			 	nJ,nK,nY=0;
	int			 	nV[_RANDOM_NTAB];
	double	 		dTemp;
	struct timeb	time;

	//set seed number using timer and random function
	ftime(&time);

	int				nSeed	= -((int)time.millitm + rand());
	unsigned int	uiRan	= time.millitm + rand();

	_nNum		= nNum;
	_nIdx		= 0;

	if(_fpRandom)
		delete[] _fpRandom;
	_fpRandom	= new float[nNum];

//   nType = 0
//
//Reference  : Numerical recipes in C 2nd Edition, pp284-285
//Properties : 32-bit linear congruential generator
//             return uniform random variables in [0,1)

	if(nType == 0){

		for(int i=0; i<nNum; i++){
			uiRan        = 1664525*uiRan + 1013904223;
			dTemp        = (double)(uiRan) / ( (double)(0xffffffff)+1.0 );
			_fpRandom[i] = (float)dTemp;
		}
	}

//   nType = 1
//
//Reference  : Numerical recipes in C 2nd Edition, pp280
//Properties :
//          - "Minimal" random number generator of Park and Miller with
//			   	Bays-Durham shuffle and added safeguards
//          -  Return uniform random variables in (0,1)

	else if(nType == 1){
		for(int i=0; i<nNum; i++){
			if(nSeed <= 0 || nY != 0){
				if(-nSeed < 1)
					nSeed = 1;
				else
					nSeed = -nSeed;

				for(nJ=_RANDOM_NTAB+7; nJ>=0; nJ--){
					nK	  = nSeed/_RANDOM_IQ;
					nSeed = _RANDOM_IA * (nSeed-nK*_RANDOM_IQ) - _RANDOM_IR*nK;

					if(nSeed < 0)
						nSeed += _RANDOM_IM;
					if(nJ < _RANDOM_NTAB)
						nV[nJ] = nSeed;
				}
				nY = nV[0];
			}

			nK    = nSeed / _RANDOM_IQ;
			nSeed = _RANDOM_IA * (nSeed-nK*_RANDOM_IQ) - _RANDOM_IR*nK;

			if(nSeed < 0)
				nSeed += _RANDOM_IM;
			nJ     = nY / _RANDOM_NDIV;
			nY     = nV[nJ];
			nV[nJ] = nSeed;

			dTemp        = _RANDOM_AM * nY;
			_fpRandom[i] = (float)(dTemp > _RANDOM_RNMX ? _RANDOM_RNMX : dTemp);
		}
	}
}
//---------------------------------------------------------------------------


KGaussian::KGaussian()
{
	_dMean      = 0;
	_dVar       = 1;
    _fpRandom	= 0;
    _opRandom	= 0L;
}

KGaussian::KGaussian(const double& dMean,const double& dVar)
{
    _fpRandom	= 0;
    _opRandom	= 0;

    Create(dMean, dVar);
}

KGaussian::~KGaussian(){
	if(_fpRandom)
		delete[] _fpRandom;
	if(_opRandom)
		delete _opRandom;
}

void KGaussian::Create(const double& dMean,const double& dVar)
{
    _dConst = 1.0 / sqrt(_2PI*dVar);
    _dMean  = dMean;
	_dVar   = dVar;
	_dSigma = sqrt(dVar);
	_dVar2  = 2.0*_dVar;
}

bool KGaussian::Read(FILE* file)
{
    if(fread(&_dMean, sizeof(double), 1, file) != 1 ||  fread(&_dVar, sizeof(double), 1, file) != 1)
        return false;

	_dSigma = sqrt(_dVar);
	_dVar2  = 2.0*_dVar;
	_dConst = 1.0 / sqrt(_2PI*_dVar);

}

void KGaussian::Write(FILE* file)
{
    fwrite(&_dMean, sizeof(double), 1, file);
    fwrite(&_dVar, sizeof(double), 1, file);
}

void KGaussian::WriteText(FILE* file)
{
    fprintf(file, "mean : %lf\n", _dMean);
    fprintf(file, "variance : %lf\n", _dVar);
}

double KGaussian::Pdf(const double& dX)
{
	return  _dConst * exp( -_SQR(dX-_dMean)/_dVar2 );
}

double KGaussian::Exp(const double& dX)
{
	return  exp( -_SQR(dX-_dMean)/_dVar2 );
}


double KGaussian::Pdf(const double& dX,const double& dMean,const double& dVar)
{
	double dConst = 1.0 / 2.5066 / sqrt(dVar);

	return  dConst * exp( _SQR(dX-dMean) / (-2.0*dVar) );
}

//Reference    :   nemerical recipes in C 2nd Edition, pp288-289
//Properties   : - random variates using Box-Muller method
//               - return random variates with mean 0 and variance 1
//
bool KGaussian::OnRandom(int nNum)
{
	double dV1,dV2,dR,dFac;

	_nNum  = (nNum+1)/2*2; //to produce an even nember

	if(_fpRandom)
		delete[] _fpRandom;
	if(_opRandom)
		delete _opRandom;
	if( (_fpRandom=new float[_nNum]) == 0)
		return false;
	if( (_opRandom=new KRandom(_nNum)) == 0)
		return false;

	for(int i=0; i<_nNum; i+=2){
		do{
			dV1 = 2.0 * _opRandom->Generate() - 1.0;
			dV2 = 2.0 * _opRandom->Generate() - 1.0;
			dR  = dV1 * dV1 + dV2 * dV2;
		}while(dR>=1.0 || dR==0);

		dFac  = sqrt(-2.0*log(dR)/dR);

		_fpRandom[i]   = (float)(dV1*dFac);
		_fpRandom[i+1] = (float)(dV2*dFac);
	}

	//set index of ready-made random numbers
	_nIdx	= 0;

	return true;
}

void KGaussian::OffRandom( )
{
	if(_fpRandom)
		delete[] _fpRandom;
	if(_opRandom)
		delete _opRandom;

	_fpRandom	= 0;
	_opRandom	= 0;
}

//generate random numbers with a Gaussian distribution
double& KGaussian::Generate()
{
#ifdef _DEBUG
	assert(_fpRandom != 0);
#endif
	_nIdx    = _nIdx % _nNum;
	_dRandom = _dMean + _dSigma*_fpRandom[_nIdx++];

	return _dRandom;
}

double& KGaussian::Generate(const double& dMean, const double& dSigma)
{
#ifdef _DEBUG
	assert(_fpRandom != 0);
#endif
	_nIdx    = _nIdx % _nNum;
	_dRandom = dMean + dSigma*_fpRandom[_nIdx++];

	return _dRandom;
}
//---------------------------------------------------------------------------

KGaussian& KGaussian::operator=(const KGaussian& gIn)
{
	_dMean = gIn._dMean;
	_dVar  = gIn._dVar;
	_dVar2 = gIn._dVar2;
	_dConst= gIn._dConst;

	return *this;
}


KGaussianMulti::KGaussianMulti()
{
	_nDim		 = 0;
	_gpGaussians = 0;

}

KGaussianMulti::KGaussianMulti(const KVector& vMean, const KMatrix& mCov)
{
	_nDim		 = 0;
	_gpGaussians = 0;

	Create(vMean,mCov);
}

KGaussianMulti::KGaussianMulti(int nDim)
{
	_nDim		 = 0;
	_gpGaussians = 0;

	Create(nDim);
}

bool KGaussianMulti::Create(int nDim)
{
	//Set size of dimension
	_nDim = nDim;

	//mean vector is set to zero vector
	_vMean.Create(nDim);

	//covariance matrix is set by Identity matrix as a default
	_mCov.Create(nDim,nDim,_IDENTITY);
	_mLowCov.Create(nDim,nDim,_IDENTITY);
	_dConst	= 1.0 / pow(_2PI,_vMean.Dim()/2.0) / sqrt(fabs(_mCov.Det( )));

    //switch off random generator
    OffRandom();

    return true;
}

bool KGaussianMulti::Create(const KVector& vMean, const KMatrix& mCov)
{
	_nDim   = vMean.Dim();
	_vMean	= vMean;

	_mCov	= mCov;
	_mIvCov	= _mCov.Iv();
	_dConst	= 1.0 / pow(_2PI,_vMean.Dim()/2.0) / sqrt(fabs(_mCov.Det()));
    _dConstNormal = 1.0 / pow(_2PI,_vMean.Dim()/2.0);

    //the lower matrix in Cholesky decomposition of oSigma
	_mLowCov= _mCov.Cholesky();

    //switch off random generator
    OffRandom();

    return true;
}

KVector KGaussianMulti::Kernel_1D(const double& dSigma,const int& nSize)
{
    //set dimension
    int nDim = nSize;
    if(nDim == 0)
        nDim = 2*( 2.54*dSigma < 1.0 ? 1:(int)(2.54*dSigma+0.5)) + 1;
    else if(nDim%2 == 0)
        nDim++;

    //compute Gaussian Mask
    int     nHalf   = nDim/2;
    double  dSigma2 = 2.0*_SQR(dSigma);
    KVector vOut(nDim);

    for(int i=0; i<=nHalf; i++)
    {
        vOut[nHalf + i] = exp(-(i*i) / dSigma2);
        vOut[nHalf - i] = vOut[nHalf + i];
    }
	vOut.Normalized(_UNITSUM_NORMALIZE);

    return vOut;
}

KVector KGaussianMulti::Kernel_Derivative(const double& dSigma,const int& nSize)
{
	//set dimension
	int nDim = nSize;
    if(nDim == 0)
	    nDim = 2*( 2.54*dSigma < 1.0 ? 1:(int)(2.54*dSigma)) + 1;
    else if(nDim%2 == 0)
        nDim++;
	
	//convolution mask for edge
	int 	nHalf  = nDim/2;
    double  dVar   = _SQR(dSigma);
	KVector	vMask(nDim);

	for(int j=-nHalf,jj=0; j<=nHalf; j++,jj++)
	{
		vMask[jj] = -j/(2.5*dVar*dSigma)*exp(-(j*j)/2./dVar);
	}
    vMask *= (-1.0/vMask.To(nHalf).Sum());

    return vMask;
}    

KMatrix KGaussianMulti::Kernel_2D(const double& dSigma,const int& nSize)
{
    //set dimension
    int nDim = nSize;
    if(nDim == 0)
	    nDim = 2*( 3.0*dSigma < 1.5 ? 1:(int)(3.0*dSigma+0.5) ) + 1;
    else if(nDim%2 == 0)
        nDim++;

	//compute Gaussian Mask
	int     nHalf   = nDim/2;
	double  dScale = 0.0;
	double  dSigma2 = 2.0*_SQR(dSigma);
	KMatrix mMask(nDim,nDim);

	for(int r = -nHalf,i=0; r <= nHalf; r++,i++)
	{
		 for(int c = -nHalf,j=0; c <= nHalf; c++,j++)
		 {
            mMask[i][j] = exp(-(r*r+c*c)/dSigma2);
			dScale     += mMask[i][j];
		 }
	}
	mMask /= dScale;

    return mMask;
}


//Reference  : Ripley,B.D.(1987),chap4.2 Multivariate distributions in Stochastic Simultion,
//             pp98-99
//Properties : return random vector
//
KVector& KGaussianMulti::Generate()
{
	for(int i=0; i<_nDim; i++)
		_vRandom[i] = _gpGaussians[i].Generate();
	_vRandom = _vMean + _mLowCov * _vRandom;

	return _vRandom;
}

KVector& KGaussianMulti::Generate(KMatrix& mLowSigma)
{
   //ASSERT(_vRandom.Pointer() == 0);

	for(int i=0; i<_nDim; i++)
		_vRandom[i] = _gpGaussians[i].Generate();
	_vRandom = _vMean + mLowSigma * _vRandom;

	return _vRandom;
}

bool KGaussianMulti::Read(FILE* file)
{
	if(!_vMean.Read(file))
		return false;
	if(!_mCov.Read(file))
		return false;

    Create(_vMean,_mCov);
    OffRandom();
	return true;
}

bool KGaussianMulti::Read(char* szFile)
{
    FILE* fp;
    if((fp=fopen(szFile,"rb")) == 0)
        return false;

	if(!_vMean.Read(fp) || !_mCov.Read(fp)){
        fclose(fp);
		return false;
    }
    fclose(fp);

    Create(_vMean,_mCov);
    OffRandom();
	return true;
}


void KGaussianMulti::Write(FILE* file)
{
	_vMean.Write(file);
	_mCov.Write(file);
}

void KGaussianMulti::WriteText(FILE* file)
{
	_vMean.WriteText(file); fprintf(file,"\n");
	_mCov.WriteText(file);
}

bool KGaussianMulti::OnRandom(int nNum)
{
	//check
	if(_nDim == 0)
		return false;

	//create single Gaussian objects for generating random number
	if(_gpGaussians == NULL) _gpGaussians = new KGaussian[_nDim];

	//call the OnRandom( ) function of the Gaussian object for each dimension
	for(int i=0; i<_nDim; i++){
		if( !_gpGaussians[i].OnRandom(nNum) )
			return false;
	}

	//create an object for a random vector to be drawn from this distribution
	_vRandom.Create(_nDim);

	return	true;
}

void KGaussianMulti::OffRandom( )
{
	if(_gpGaussians == NULL)
		return;

	for(int i=0; i<_nDim; i++)
		_gpGaussians[i].OffRandom( );

   _vRandom.Release();
}

double KGaussianMulti::Pdf(const KVector& vX)
{
    return _dConst * exp((vX-_vMean).Norm2(_mIvCov)*(-0.5));
}

double KGaussianMulti::PdfNormal(const KVector& vX)
{
    return _dConstNormal * exp((vX-_vMean).Norm2(_mIvCov)*(-0.5));
}


double KGaussianMulti::Pdf(const double& dX, const double& dY)
{
#ifdef _DEBUG
	assert(_nDim == 2);
#endif
	double dA = dX - _vMean[0];
	double dB = dY - _vMean[1];
	return _dConst * exp((-0.5)*(dA*dA*_mIvCov[0][0] + dA*dB*(_mIvCov[1][0] + _mIvCov[0][1]) + dB*dB*_mIvCov[1][1]));
}

double KGaussianMulti::LogPdf(const double& dX, const double& dY)
{
#ifdef _DEBUG
	assert(_nDim == 2);
#endif
	double dA = dX - _vMean[0];
	double dB = dY - _vMean[1];
	return log(_dConst) + (-0.5)*(dA*dA*_mIvCov[0][0] + dA*dB*(_mIvCov[1][0] + _mIvCov[0][1]) + dB*dB*_mIvCov[1][1]);
}

double KGaussianMulti::Exp(const KVector& vX)
{
	return exp((vX-_vMean).Norm2(_mIvCov)*(-0.5));
}

double KGaussianMulti::LogExp(const KVector& vX)
{
	return (vX-_vMean).Norm2(_mIvCov) * (-0.5);
}

double KGaussianMulti::LogPdf(const KVector& vX)
{
	return ((vX-_vMean).Norm2(_mIvCov) * (-0.5) + log(_dConst));
}

double KGaussianMulti::Pdf(const KVector& vX,const KVector& vMean,const KMatrix& mCov)
{
	double dExp = (vX-vMean).Norm2(~mCov) * (-0.5);
	return exp(dExp) / pow(_2PI,vX.Dim()/2.0) / sqrt(fabs(mCov.Det()));
}

double KGaussianMulti::LogPdf(const KVector& vX,const KVector& vMean,const KMatrix& mCov)
{
	double dDen   = pow(_2PI,vX.Dim()/2.0) * sqrt(fabs(mCov.Det()));
	double dExp   = (vX-vMean).Norm2(~mCov) * (-0.5);

	return dExp - log(dDen);
}

double KGaussianMulti::Mahalanobis(const KVector& vX,const KVector& vMean,const KMatrix& mIvCov)
{
	return (vX-vMean).Norm2(mIvCov);
}


double KGaussianMulti::Mahalanobis(const KVector& vX)

{
	return (vX-_vMean).Norm2(_mIvCov);
}



KGaussianMulti& KGaussianMulti::operator=(const KGaussianMulti& gIn)
{
    _nDim   = gIn._mCov.Col();
	_vMean	= gIn._vMean;

    _mCov	= gIn._mCov;
	_mIvCov	= gIn._mIvCov;
    _mLowCov= gIn._mLowCov; //the lower matrix in Cholesky decomposition of oSigma

    _dConst	= gIn._dConst;

	return *this;
}
/////////////////////////////////////////////////////////////////////////////////

KGaussianMixture::KGaussianMixture(const KGaussianMixture& oGmx)
{
    _gpMulti = 0;
    _gpSingle = 0;

    if(oGmx.Type() == 0)
        Create(oGmx._vWeight,oGmx._gpSingle);
    else
        Create(oGmx._vWeight,oGmx._gpMulti);
}

KGaussianMixture::KGaussianMixture(const KVector& vWeight, KGaussianMulti* gpGaussians)
{
    _gpMulti = 0;
    _gpSingle = 0;

    Create(vWeight,gpGaussians);
}

KGaussianMixture::KGaussianMixture(const KVector& vWeight, KGaussian* gpGaussians)
{
    _gpMulti = 0;
    _gpSingle = 0;

    Create(vWeight, gpGaussians);
}

void KGaussianMixture::Create(const KVector& vWeight, KGaussianMulti* gpGaussians)
{
    //delete the previous distributions
    if(_gpMulti)
        delete[] _gpMulti;
    if(_gpSingle)
        delete[] _gpSingle;
    _gpMulti  = 0;
    _gpSingle = 0;

    _vWeight = vWeight;
    _nNum    = _vWeight.Dim();
    _gpMulti  = new KGaussianMulti[_nNum];
    _nType    = 2;

    for(int i=0; i<_vWeight.Dim(); i++)
        _gpMulti[i] = gpGaussians[i];
}

void KGaussianMixture::Create(const KVector& vWeight, KGaussian* gpGaussians)
{
    //delete the previous distributions
    if(_gpMulti)
        delete[] _gpMulti;
    if(_gpSingle)
        delete[] _gpSingle;
    _gpMulti  = 0;
    _gpSingle = 0;

    //
    _vWeight    = vWeight;
    _nNum        = _vWeight.Dim();
    _gpSingle   = new KGaussian[_nNum];
    _nType       = 1;

    for(int i=0; i<_vWeight.Dim(); i++)
        _gpSingle[i] = gpGaussians[i];
}


KGaussianMixture& KGaussianMixture::operator = (const KGaussianMixture& oGmx)
{
    //delete the previous distributions
    if(_gpMulti)
        delete[] _gpMulti;
    if(_gpSingle)
        delete[] _gpSingle;
    _gpMulti  = 0;
    _gpSingle = 0;

    //copy
    _nNum    = oGmx._nNum;
    _vWeight = oGmx._vWeight;
    _nType   = oGmx.Type();
    memcpy(_szID,oGmx._szID,200);

    if(oGmx.Type() == 2)    //multi-variate Gaussian
    {
        _gpMulti   = new KGaussianMulti[_nNum];
        for(int i=0; i<_nNum; i++)
            _gpMulti[i] = oGmx._gpMulti[i];
    }
    else                    //single-variate Gaussian
    {
        _gpSingle   = new KGaussian[_nNum];
        for(int i=0; i<_nNum; i++)
            _gpSingle[i] = oGmx._gpSingle[i];
    }

    return *this;
}

void KGaussianMixture::Save(const char* szFile)
{
    FILE* fp;
    if((fp = fopen(szFile,"wb")) == 0)
        return;

    fwrite(_szID,sizeof(char),200,fp);
    _vWeight.Write(fp);
    fwrite(&_nType,sizeof(int),1,fp);

    if(_gpMulti)
        for(int i=0; i<_nNum; i++)
            _gpMulti[i].Write(fp);
    else
        for(int i=0; i<_nNum; i++)
            _gpSingle[i].Write(fp);

    fclose(fp);
}

void KGaussianMixture::SaveText(const char* szFile)
{
    FILE* fp;
    if((fp = fopen(szFile,"wt")) == 0)
        return;

    fprintf(fp, "%s\n",  _szID);

    _vWeight.WriteText(fp, "%5.2f");
    fprintf(fp, "type : %s\n", (_nType == 1 ? "single" : "multivariate"));

    if(_gpMulti)
        for(int i=0; i<_nNum; i++)
            _gpMulti[i].WriteText(fp);
    else
        for(int i=0; i<_nNum; i++)
            _gpSingle[i].WriteText(fp);

    fclose(fp);
}
//---------------------------------------------------------------------------

void KGaussianMixture::Write(FILE* fp)
{
    fwrite(_szID,sizeof(char),200,fp);
    _vWeight.Write(fp);
    fwrite(&_nType,sizeof(int),1,fp);

    if(_gpMulti)
        for(int i=0; i<_nNum; i++)
            _gpMulti[i].Write(fp);
    else
        for(int i=0; i<_nNum; i++)
            _gpSingle[i].Write(fp);
}
//---------------------------------------------------------------------------


bool KGaussianMixture::Read(FILE* fp)
{
    //delete the previous distributions    
    if(_gpMulti)
        delete[] _gpMulti;
    if(_gpSingle)
        delete[] _gpSingle;
    _gpMulti  = 0;
    _gpSingle = 0;

    //set ID
    fread(_szID,sizeof(char),200,fp);
    //set weights
    if(_vWeight.Read(fp) == false)
        return false;

    //set gaussians    
    fread(&_nType,sizeof(int),1,fp);

    _nNum   = _vWeight.Dim();
    if(_nType == _SINGLE)
    {
        _gpSingle = new KGaussian[_nNum];
        for(int i=0; i<_nNum; i++)
        {
            if(_gpSingle[i].Read(fp) == false){
                delete[] _gpSingle;
                _gpSingle = 0;
                return false;
    }}}
    else
    {
        _gpMulti = new KGaussianMulti[_nNum];
        for(int i=0; i<_nNum; i++)
        {
            if(_gpMulti[i].Read(fp) == false){
                delete[] _gpMulti;
                _gpMulti = 0;
                return false;
    }}}

    return true;
}
//---------------------------------------------------------------------------

bool KGaussianMixture::Load(const char* szFile)
{
    //file open
    FILE* fp;
    if((fp = fopen(szFile,"rb")) == 0)
        return false;

    //delete the previous distributions
    if(_gpMulti)
        delete[] _gpMulti;
    if(_gpSingle)
        delete[] _gpSingle;
    _gpMulti  = 0;
    _gpSingle = 0;

    //set ID
    fread(_szID,sizeof(char),200,fp);
    //set weights
    if(_vWeight.Read(fp) == false)
        return false;

    //set gaussians
    fread(&_nType,sizeof(int),1,fp);

    _nNum   = _vWeight.Dim();
    if(_nType == _SINGLE)
    {
        _gpSingle = new KGaussian[_nNum];
        for(int i=0; i<_nNum; i++)
        {
            if(_gpSingle[i].Read(fp) == false){
                delete[] _gpSingle;
                _gpSingle = 0;
                return false;
    }}}
    else
    {
        _gpMulti = new KGaussianMulti[_nNum];
        for(int i=0; i<_nNum; i++)
        {
            if(_gpMulti[i].Read(fp) == false){
                delete[] _gpMulti;
                _gpMulti = 0;
                return false;
    }}}

    return true;
}

double KGaussianMixture::Pdf(const KVector& vX)
{
    double dOut = 0.0;

    for(int i=0; i<_nNum; i++)
        dOut += _vWeight[i]*_gpMulti[i].Pdf(vX);
    return dOut;
}

double KGaussianMixture::Pdf(const double& vX)
{
    double dOut = 0.0;

    for(int i=0; i<_nNum; i++)
        dOut += _vWeight[i]*_gpSingle[i].Pdf(vX);
    return dOut;
}

double KGaussianMixture::PdfNormal(const KVector& vX)
{
    double dOut = 0.0;
    for(int i=0; i<_nNum; i++)
        dOut += _vWeight[i]*_gpMulti[i].PdfNormal(vX);

    return dOut;
}



double KGaussianMixture::Pdf(const double& a,const double& b)
{
#ifdef _DEBUG
    assert(_gpMulti->Dim());
#endif
    double dOut = 0.0;
    for(int i=0; i<_nNum; i++)
        dOut += _vWeight[i]*_gpMulti[i].Pdf(a,b);
	return dOut;
}


double KGaussianMixture::LogPdf(const KVector& vX)
{
    double dOut = 0.0;
    for(int i=0; i<_nNum; i++)
        dOut += _vWeight[i]*_gpMulti[i].Pdf(vX);

    if(dOut < 1e-300)
        return -1e10;
    return log(dOut);
}

double KGaussianMixture::LogPdf(const double& dX)
{
    double dOut = 0.0;
    for(int i=0; i<_nNum; i++)
        dOut += _vWeight[i]*_gpSingle[i].Pdf(dX);

    if(dOut < 1e-300)
        return -1e10;
    return log(dOut);
}



double KGaussianMixture::Learning(int nNum,const KMatrix& mSample)
{
    //set
    _nNum = nNum;
    _vWeight.Create(nNum);
    _nType = 2; //multivariate Gaussian pdf

    if(_gpMulti) delete[] _gpMulti;
    _gpMulti = new KGaussianMulti[nNum];

    //initial learning
    bool bInit = false;
    for(int itr=0; itr<10 && !bInit; itr++)
        bInit =  InitLearning(nNum,mSample);
    if(!bInit) return -1e30;

    //EM learning
    double dLike;
    double dLikeOld = -1e30;

	for(int itr=0; itr<50; itr++){
        //E-step
        double  dDen;
        KMatrix mW(nNum,mSample.Col());
        for(int i=0; i<mSample.Col(); i++){
            dDen = 0.0;
            for(int j=0; j<nNum; j++){
                mW[j][i] = _vWeight[j] * _gpMulti[j].Pdf(mSample.Column(i));
                dDen    += mW[j][i];
            }
            for(int j=0; j<nNum; j++)
                mW[j][i] = (dDen == 0.0 ? 0.0 : mW[j][i]/dDen);
        }

        //M-step		
        KVector vMean(mSample.Row());
        KMatrix mCov(mSample.Row(),mSample.Row());

        for(int j=0; j<nNum; j++){
            //update weight and mean
            _vWeight[j] = 0.0;
            vMean           = 0.0;
            for(int i=0; i<mSample.Col(); i++){
                _vWeight[j] += mW[j][i];
                vMean           += mSample.Column(i)*mW[j][i];
            }
#ifdef _DEBUG
            assert(_vWeight[j] != 0.0);
#endif            
            vMean   /=  _vWeight[j];

            //update covariance
            mCov = 0.0;
            for(int i=0; i<mSample.Col(); i++)
                mCov += (mSample.Column(i)-vMean).Cor()*mW[j][i];
            mCov /=  _vWeight[j];
            if(_gpMulti[j].Create(vMean,mCov) == false)
                return -1e30;

            //weight normalization
            _vWeight[j] /= (double)(mSample.Col());
        }

        //check convergence
        dLike = 0.0;
        for(int i=0; i<mSample.Col(); i++){
            double tmp = Pdf(mSample.Column(i));
#ifdef _DEBUG
			assert(tmp);
#endif
            if(tmp != 0.0)
                dLike += log(tmp);
        }
        if(_DIFF(dLike,dLikeOld) < 1e-3)
            break;
        dLikeOld = dLike;
    }

    return dLike;
}

double KGaussianMixture::Learning(int nNum, const KVector& vSample)
{
    //set
    _nNum = nNum;
    _vWeight.Create(nNum);
    _nType  = 1;  //single Gaussian pdf

    if(_gpSingle) delete[] _gpSingle;
    if(_gpMulti)  delete[] _gpMulti;
    _gpSingle = new KGaussian[nNum];

    //initial learning
    bool bInit = false;
    for(int itr=0; itr<10 && !bInit; itr++)
        bInit =  InitLearning(nNum,vSample);
    if(!bInit) return -1e30;

    //EM learning
    double dLike;
    double dLikeOld = -1e30;

    for(int itr=0; itr<50; itr++){
        //E-step
        double  dDen;
        KMatrix mW(nNum,vSample.Dim());
        for(int i=0; i<vSample.Dim(); i++){
            dDen = 0.0;
            for(int j=0; j<nNum; j++){
                mW[j][i] = _vWeight[j] * _gpSingle[j].Pdf(vSample[i]);
                dDen    += mW[j][i];
            }
            for(int j=0; j<nNum; j++)
                mW[j][i] = (dDen == 0.0 ? 0.0 : mW[j][i]/dDen);
        }

        //M-step
        double    dMean;
        double    dVar;

        for(int j=0; j<nNum; j++){
            //update weight and mean
            _vWeight[j] = 0.0;
            dMean          = 0.0;
            for(int i=0; i<vSample.Dim(); i++)
            {
                _vWeight[j] += mW[j][i];
                dMean          += vSample[i] * mW[j][i];
            }
#ifdef _DEBUG
            assert(_vWeight[j] != 0.0);
#endif
            dMean  /= _vWeight[j];

            //update covariance
            dVar = 0.0;
            for(int i=0; i<vSample.Dim(); i++)
                dVar += _SQR(vSample[i]-dMean)*mW[j][i];
            dVar /= _vWeight[j];

            _gpSingle[j].Create(dMean, dVar);

            //weight normalization
            _vWeight[j] /= (double)(vSample.Dim());
        }

        //check convergence
        dLike = 0.0;
        for(int i=0; i<vSample.Dim(); i++){
            double tmp = Pdf(vSample[i]);
#ifdef _DEBUG
            assert(tmp);
#endif
            if(tmp != 0.0)
                dLike += log(tmp);
        }
        if(_DIFF(dLike,dLikeOld) < 1e-3)
            break;
        dLikeOld = dLike;
    }

    return dLike;
}



//Initialization by K-means algorithm
bool KGaussianMixture::InitLearning(int nNum, const KMatrix& mSample)
{
    //
    int         i,j,itr;
	KVector*    vpMeans      = new KVector[nNum];
    int*        npMembership = new int[mSample.Col()];
    int*        npMembers    = new int[nNum];
    KRandom     rnSeed;

    //initial state
    memset(npMembership,0xff,sizeof(int)*mSample.Col());
	for(int i=0; i<nNum; i++){
        vpMeans[i].Create(mSample.Row());
		vpMeans[i] = mSample.Column((int)(rnSeed.Generate()*(mSample.Col()-1) + 0.5));
    }

    //
	for(itr=0; itr<10; itr++){
        //update membership
		bool    bChanged = false;
        int     nID;
		double  dMin,dDist;
		for(i=0; i<mSample.Col(); i++){
            dMin = 1e10;
            for(j=0; j<nNum; j++){
                if((dDist=KVector::Distance2(vpMeans[j],mSample.Column(i))) < dMin){
                    dMin = dDist;
                    nID  = j;
            }}
            if(npMembership[i] != nID)
                bChanged = true;
            npMembership[i] = nID;
        }
        if(bChanged == false)
            break;
        //update means
        for(i=0; i<nNum; i++){
            vpMeans[i]   = 0.0;
            npMembers[i] = 0;
        }
        for(i=0; i<mSample.Col(); i++){
            vpMeans[npMembership[i]]   += mSample.Column(i);
            npMembers[npMembership[i]] ++;
        }
        for(i=0; i<nNum; i++){
			if(npMembers[i] < 1){
                delete[] npMembers;
                delete[] npMembership;
                delete[] vpMeans;
                return false;
            }
            vpMeans[i] *= (1.0/(double)(npMembers[i]));
        }
    }

    //
    KMatrix     mCov(mSample.Row(),mSample.Row());
    KMatrix*    mpCov = new KMatrix[nNum];
    KVector     vMeanTmp(mSample.Row());

    for(i=0; i<nNum; i++)
        mpCov[i].Create(mSample.Row(),mSample.Row());

    for(i=0; i<mSample.Col(); i++)
        vMeanTmp += mSample.Column(i);
    vMeanTmp /= (double)(mSample.Col());

    for(i=0; i<mSample.Col(); i++)
    {
        mCov += (mSample.Column(i) - vMeanTmp).Cor();
        mpCov[npMembership[i]] += (mSample.Column(i) - vpMeans[npMembership[i]]).Cor();
    }
    mCov /= (double)(mSample.Col()-1);

    bool bOk;
    for(i=0; i<nNum; i++)
    {
        _vWeight[i] = (double)(npMembers[i]) / (double)(mSample.Col());
        if(npMembers[i] > mSample.Row())
            bOk = _gpMulti[i].Create(vpMeans[i],mpCov[i]/(double)(npMembers[i]-1));
        else
            bOk = _gpMulti[i].Create(vpMeans[i],mCov);
        if(bOk == false){
            delete[] npMembers;
            delete[] npMembership;
            delete[] vpMeans;
            delete[] mpCov;
            return false;
        }
    }

    //free
    delete[] npMembers;
    delete[] npMembership;
    delete[] vpMeans;
    delete[] mpCov;

    return true;
}

bool KGaussianMixture::InitLearning(int nNum, const KVector& vSample)
{
    //
    int               i,j,itr;
    double*     dpMeans           = new double[nNum];
    int*            npMembership = new int[vSample.Dim()];
    int*            npMembers      = new int[nNum];
    KRandom   rnSeed;

    //initial state
    memset(npMembership,0xff,sizeof(int)*vSample.Dim());
    for(int i=0; i<nNum; i++)
        dpMeans[i] = vSample[(int)(rnSeed.Generate()*(vSample.Dim()-1) + 0.5)];

    //
    for(itr=0; itr<10; itr++)
    {
        //update membership
        bool    bChanged = false;
        int     nID;
        double  dMin,dDist;
        for(i=0; i<vSample.Dim(); i++){
            dMin = 1e10;
            for(j=0; j<nNum; j++){
                if((dDist=_SQR(dpMeans[j]-vSample[i])) < dMin){
                    dMin = dDist;
                    nID  = j;
            }}
            if(npMembership[i] != nID)
                bChanged = true;
            npMembership[i] = nID;
        }
        if(bChanged == false)
            break;
        //update means
        for(i=0; i<nNum; i++){
            dpMeans[i]   = 0.0;
            npMembers[i] = 0;
        }
        for(i=0; i<vSample.Dim(); i++){
            dpMeans[npMembership[i]]   += vSample[i];
            npMembers[npMembership[i]] ++;
        }
        for(i=0; i<nNum; i++){
            if(npMembers[i] < 1){
                delete[] npMembers;
                delete[] npMembership;
                delete[] dpMeans;
                return false;
            }
            dpMeans[i] *= (1.0/(double)(npMembers[i]));
        }
    }

    //
    double          dVar = 0.0;
    KVector       vVar(nNum);
    double          dMeanTmp = 0.0;

    for(i=0; i<vSample.Dim(); i++)
        dMeanTmp += vSample[i];
    dMeanTmp /= (double)(vSample.Dim());

    for(i=0; i<vSample.Dim(); i++)
    {
        dVar += _SQR(vSample[i] - dMeanTmp);
        vVar[npMembership[i]] += _SQR(vSample[i] - dpMeans[npMembership[i]]);
    }
    dVar /= (double)(vSample.Dim()-1);

    for(i=0; i<nNum; i++)
    {
        _vWeight[i] = (double)(npMembers[i]) / (double)(vSample.Dim());
        if(npMembers[i] > 1)
            _gpSingle[i].Create(dpMeans[i], vVar[i]/(double)(npMembers[i]-1));
        else
            _gpSingle[i].Create(dpMeans[i],dVar);
    }

    //free
    delete[] npMembers;
    delete[] npMembership;
    delete[] dpMeans;

    return true;
}
//////////////////////////////////////////////////////////////////////////////////////////////
