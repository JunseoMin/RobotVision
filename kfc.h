//---------------------------------------------------------------------------
#ifndef kfcH
#define kfcH


////////////////////////////////////////////////////////////////////////////////////
//
// This library can be freely used and reproduced by anyone.
//
// KFC library by Jeong, Mun-Ho since 2004.05.01
// latest update : 2023.08.17
//
///////////////////////////pa////////////////////////////////////////////////////////

#include <stdio.h>
#include <cstdlib>
#include <ctime>
#include <stdarg.h>
#include <assert.h>
#include <memory>
#include <cstring>
#include <cmath>
#include <sys/timeb.h>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>

#ifdef _WIN32
    #include <direct.h>  // Windows
#else
    #include <sys/stat.h>
    #include <unistd.h>  // Linux/macOS
#endif

#include <algorithm>


//sorting
#define _DESCEND    0
#define _ASCEND     1

//for matrix operations
#define _REPLACE     0
#define _ADD         1
#define _ZERO        0
#define _ONE		 1
#define _IDENTITY    2
#define _DIAGONAL    3

//position
#define _LEFT        0
#define _RIGHT       1
#define _UPP         2
#define _DOWN        3
#define _BACK		 4
#define _FRONT		 5
#define	_TOP		 6
#define _BOTTOM		 7

//rotation
#define	_CW			 0
#define	_CCW		 1

//for matrix, rect object
#define _ROW         0
#define _COL         1
#define _BOTH        2
#define _CENTER      3
#define _ORIGIN		 4

//degree and radian
#define _DEG         0
#define _RAD         1
#define _EULER       8
#define _RODRIGUES   9


//memory alloc. of array object
#define _ALLOC       0
#define _LOCK        1

//smoothing
#define _MEDIAN		 	0
#define _MEDIAN_RANK 	0
#define _MEDIAN_VALUE 	1
#define _GAUSSIAN    	2

//transformation
#define _RIGID              0
#define _SIMILARITY         1
#define _AFFINE             2
#define _PERSPECTIVE        3
#define _RECT_GENERIC       4
#define _ELLIPSE_GENERIC    5

//return value
#define _INVALID    -1

//image format
#define _NRG         0
#define _YUV         1
#define _HSI         2
#define _RGB         3
#define _CS          4
#define _GRAY		 5
#define _RGB_ONEBYTE 6  //r:3bit | g:3bit | b:2bit
#define	_BINARY      7


//color
#define _RED         0
#define _GREEN       1
#define _BLUE        2

//axis
#define _X_AXIS     0
#define _Y_AXIS     1
#define _Z_AXIS     2

#define	_START		0
#define _END		1

#define _DEFAULT	0
#define _ABSOLUTE	1

//circle detection
#define _OUTWARD_EDGE		0
#define _OUTWARD			0
#define _INWARD_EDGE		1
#define _INWARD     		1
#define _INOUTWARD_EDGE     2
#define _OUT_SCAN			1
#define _OUT_SCAN_MAX       2
#define _OUT_SCAN_FULL     	3
#define _IN_SCAN			4
#define _IN_SCAN_MAX        5
#define _IN_SCAN_FULL      	6
#define _INOUT_SCAN     	0

//KRandom
#define	_RANDOM_IA          16807
#define	_RANDOM_IM          2147483647
#define _RANDOM_AM          (1.0/_RANDOM_IM)
#define _RANDOM_IQ          127773
#define _RANDOM_IR          2836
#define _RANDOM_NTAB        32
#define _RANDOM_NDIV        (1+(_RANDOM_IM-1)/_RANDOM_NTAB)
#define _RANDOM_EPS         1.2e-7
#define _RANDOM_RNMX        (1.0-_RANDOM_EPS)


//vector normalization
#define _NO_NORMALIZE       0
#define _DTB_NORMALIZE      1 //mean 0 and std 1
#define _SIZE_NORMALIZE     2 //size 1
#define _HEQ_NORMALIZE      3 //histogram equalization
#define _MEAN_NORMALIZE     4 //mean 0
#define _STD_NORMALIZE      5 //std 1
#define _UNITSUM_NORMALIZE  6 //sum 1

//전처리
#define _SIZE_PREPROC       0 //size 1
#define _DTB_PREPROC        1 //mean 0 and std 1
#define _HEQ_PREPROC        2 //histogram equalization
#define _LAP_PREPROC        3 //Laplacian
#define _TV_PREPROC         4 //total variation

//matrix
#define _NO_SCALING         0
#define _DEFAULT_SCALING    1
#define _USER_SCALING       2

#define _PI                                 3.1415926535897932385
#define _2PI                                6.283185307179586477
#define	_SQR(a)                             ((a)*(a))
#define	_MAX(a,b)                           (((a)>(b)) ? (a):(b))
#define	_MIN(a,b)                           (((a)<(b)) ? (a):(b))
#define _DIFF(a,b)                          ((a)>(b)   ? (a)-(b):(b)-(a))
#define _ABS(a)                             ((a)>0.0   ? (a):-(a))
#define	_SIGN(a)                            ((a)<0.0 ? -1 : 1)
#define	_SIGNEX(a,b)                        ((b)<0.0 ? -(a) : (a))
#define _DEGREE(a)                          ((a)*57.2957795130823)
#define _RADIAN(a)                          ((a)/57.2957795130823)
#define _INTENSITY(r,g,b)                   (0.299*(r) + 0.587*(g) + 0.114*(b))
#define _PARABOLICCENTER(x1,y1,x2,y2,x3,y3) (((x2)+(x3))*((y2)-(y1))-((x1)+(x2))*((y3)-(y2)))/(2*(2*(y2)-(y1)-(y3)))
#define _SWAP(x, y, type) do{ \
    type tmp; \
    tmp = x; \
    x = y; \
    y = tmp; \
}while(0)

/////for Atan()
#define _ATAN_CF0  (-15.8131890796f)
#define _ATAN_CF1  (61.0941945596f)
#define _ATAN_CF2  (-0.140500406322f)

//
#define	_BACKGROUND		0
#define	_FOREGROUND		1
#define _BLACK          0
#define _WHITE          1

//배수
#define _SINGLE         1
#define _DOUBLE         2
#define _TRIPLE         3
#define _QUADRUPLE      4
#define _MULTI          5


struct KHSI
{
	float i;
	float s;
	float h;
};

struct KHSV
{
	double v;
	double s;
	double h;
};

struct KTRIPLET
{
	double r;
	double g;
	double b;
};

struct KTRIPLETF
{
	float r;
	float g;
	float b;
};

struct KYUV
{
	unsigned char y;
	unsigned char u;
	unsigned char v;
};

struct KIRG
{
	unsigned char i;
	float         r;
    float         g;
};

struct KCOLOR32
{
	unsigned char b;//or V
	unsigned char g;//or U
	unsigned char r;//or Y
	unsigned char dummy;

    KCOLOR32(){ }
    KCOLOR32(const unsigned char& rr, const unsigned char& gg,const unsigned char& bb) : r(rr),g(gg),b(bb) { }
};

struct KCOLOR24{
	unsigned char b;
	unsigned char g;
	unsigned char r;

    KCOLOR24(){ }
    KCOLOR24(const unsigned char& rr, const unsigned char& gg,const unsigned char& bb) : r(rr),g(gg),b(bb) { }
};

struct KPOINT
{
	int nX;
	int nY;

    KPOINT(){ }
    KPOINT(const int& nXo, const int& nYo) : nX(nXo), nY(nYo){ }    
};

struct KPOINT3
{
    int nX;
    int nY;
    int nZ;

    KPOINT3() { }
    KPOINT3(const int& nXo, const int& nYo, const int& nZo) : nX(nXo), nY(nYo), nZ(nZo){  }
} ;


struct KPOINTW
{
	unsigned short wX;
	unsigned short wY;

};

struct KPOINTD
{
	double dX;
	double dY;

    KPOINTD() { }
    KPOINTD(const double& dXo, const double& dYo) : dX(dXo), dY(dYo){  }
};


struct KPOINTF
{
	float fX;
	float fY;	
};

struct KPOINT3W
{
	unsigned short wX;
	unsigned short wY;
	unsigned short wZ;
};

struct KPOINT3D
{
	double dX;
	double dY;
	double dZ;

    KPOINT3D() { }
    KPOINT3D(const double& dXo, const double& dYo, const double& dZo) : dX(dXo), dY(dYo), dZ(dZo){  }
};

struct KPOINT3F
{
	float fX;
	float fY;
	float fZ;
};

union KTRIANGLEF

{
	KPOINT3F	ptpTri[3];
	float		fpTri[9];
	
};

struct KRECT
{
    int     nLeft,  nTop;
    int     nWidth, nHeight;
};

struct KPAIR
{
    unsigned int nA;
    unsigned int nB;

    KPAIR() { }
    KPAIR(const unsigned int& nID1, const unsigned int& nID2) : nA(nID1), nB(nID2){ }
};

struct KPIXEL
{
	unsigned short	u;
	unsigned short	v;
	unsigned short 	i;
	unsigned short  id;
};

struct KPIXEL24
{
	int 			u;
	int 			v;
	unsigned char 	b;
	unsigned char 	g;
	unsigned char 	r;
};

class KObject
{
public:
    KObject(){}
    ~KObject(){}

    virtual int     Init(void* pInfo) = 0;
    virtual void*   Run(void* p1,void* p2,void* p3,void* p4,void* p5) = 0;
    virtual void*   Info() = 0;
};


///////////////////
// functions of "Numerical Recipes in C"
//
/*
double Pythag(double a, double b)
{
	double absa,absb;
	absa=fabs(a);
	absb=fabs(b);
	if (absa > absb) return absa*sqrt(1.0+_SQR(absb/absa));
	else return (absb == 0.0 ? 0.0 : absb*sqrt(1.0+_SQR(absa/absb)));
}
*/
////////////////////

//
////////////////////////////////////////////////////////////
//
//class KPoint
class KVector;
class KPoints;

class KPoint //tag : pt
{
public:
    double _dX;
    double _dY;

    KPoint(const double& x = 0.0, const double& y = 0.0) : _dX(x), _dY(y) { }
    KPoint(const KPoint& pt){ _dX=pt._dX; _dY=pt._dY; }
    KPoint(const KPOINTD& pt){ _dX=pt.dX; _dY=pt.dY; }
    KPoint(const KPOINT& pt){ _dX=pt.nX; _dY=pt.nY; }
    virtual ~KPoint(){ }

public:
    bool        Read(FILE* fp);
    int         Write(FILE* fp);
    double      Distance(const KPoint& pt){ return sqrt(_SQR(pt._dX-_dX)+_SQR(pt._dY-_dY)); }
    double      Distance2(const KPoint& pt){ return (_SQR(pt._dX-_dX)+_SQR(pt._dY-_dY)); }
    double      Distance(const double& x,const double& y){ return sqrt(_SQR(_dX-x)+_SQR(_dY-y)); }
    double      Distance2(const double& x,const double& y){ return (_SQR(_dX-x)+_SQR(_dY-y)); }
    void        Rotated(const double& dRad,const double& dXo=0.0, const double& dYo=0.0);
    void        Rotated(const double& dRad,const KPoint& pt = KPoint(0.0,0.0));
    KPoint      Rotate(const double& dRad,const KPoint& pt = KPoint(0.0,0.0));

    bool		InRegion(const KPoints& psRegion,const int& nType=_CW);

    KPoint&     operator =(const KPoint& pt);
    KPoint&     operator =(const double&);
    KPoint&     operator +=(const KPoint& pt);
    KPoint&     operator +=(const KVector& v);
    KPoint&     operator *=(const double& d);
    KPoint&     operator -=(const KPoint& pt);
    KPoint      operator +(const KPoint& pt) const;
    KPoint      operator -(const KPoint& pt) const;
    KPoint      operator -() const;
    bool        operator ==(const KPoint& pt){ return !(_dX != pt._dX || _dY != pt._dY); }

    KPoint&     operator  =(const KVector& v);
    KPoint      operator  +(const KVector& v) const;
    KPoint      operator  -(const KVector& v) const;
    KPoint      operator  /(const double& d)  const;
    KPoint      operator  *(const double& d)  const;

    static double  Distance(const KPoint& ptO,const KPoint& ptE){ return sqrt(_SQR(ptO._dX-ptE._dX)+_SQR(ptO._dY-ptE._dY)); }
    static double  Distance2(const KPoint& ptO,const KPoint& ptE){ return _SQR(ptO._dX-ptE._dX)+_SQR(ptO._dY-ptE._dY); }
    static double  Distance2(const KPOINTD& ptO,const KPOINTD& ptE){ return _SQR(ptO.dX-ptE.dX)+_SQR(ptO.dY-ptE.dY); }
};

class KMatrix;
class KRect;
class KPoints : public std::vector<KPoint>
{

public:

    KPoints(int nNum=0) { if(nNum) Create(nNum); }
    KPoints(const KPoints& psIn);
    virtual ~KPoints(){ }

    void		Create(int nNum) { vector<KPoint>::resize(nNum); }

    KPoints     Transform(const KMatrix& mHgp);
    void        Transformed(const KMatrix& mHgp);
    KRect       Rect() const;
    KPoints&	operator =  ( const KPoints& psInput);
};


//
////////////////////////////////////////////////////////////
//
// class KPoint3D
class KPoint3D //tag : pt
{
public:
    double _dX;
    double _dY;
    double _dZ;

    KPoint3D(){  };
    KPoint3D(const KPoint3D& pt){ _dX=pt._dX; _dY=pt._dY; _dZ=pt._dZ; }
    KPoint3D(const double& x, const double&y, const double& z){ _dX=x; _dY=y; _dZ=z; }
    virtual ~KPoint3D(){ }

public:
    bool    Read(FILE* fp);
    void    Write(FILE* fp);
    double  Distance(const KPoint3D& pt){ return sqrt(_SQR(pt._dX-_dX)+_SQR(pt._dY-_dY)+_SQR(pt._dZ-_dZ)); }
    double  Distance2(const KPoint3D& pt){ return (_SQR(pt._dX-_dX)+_SQR(pt._dY-_dY)+_SQR(pt._dZ-_dZ)); }
    double  Distance(const int& x,const int& y, const double& z){  return sqrt(_SQR(_dX-x)+_SQR(_dY-y)+_SQR(_dZ-z)); }
    double  Distance2(const int& x,const int& y, const double& z){ return (_SQR(_dX-x)+_SQR(_dY-y)+_SQR(_dZ-z)); }

    KPoint3D&    operator  =(const KPoint3D& pt);
    KPoint3D&    operator  =(const double& d);
    KPoint3D&    operator +=(const KPoint3D& pt);
    KPoint3D&    operator -=(const KPoint3D& pt);
    KPoint3D     operator  +(const KPoint3D& pt);
    KPoint3D     operator  -(const KPoint3D& pt);
    KPoint3D	 operator  -() const;

    KPoint3D&    operator  =(const KVector& v);
    KPoint3D     operator  +(const KVector& v);
    KPoint3D     operator  -(const KVector& v);
};

class KIDPoint : public KPoint
{
public:
    int _nID;

    KIDPoint(){ }
    KIDPoint(const int nID,const KPoint& ptIn) : KPoint(ptIn){ _nID = nID; }
    KIDPoint(const int nID,const double& x,const double& y) : KPoint(x,y){ _nID = nID; }
    virtual ~KIDPoint(){ }
};

class KIDPoint3D : public KPoint3D
{
public:
    int _nID;

    KIDPoint3D(){ }
    KIDPoint3D(const int nID,const KPoint3D& ptIn) : KPoint3D(ptIn){ _nID = nID; }
    KIDPoint3D(const int nID,const double& x,const double& y,const double& z) : KPoint3D(x,y,z){ _nID = nID; }
    virtual ~KIDPoint3D(){ }
};

//
////////////////////////////////////////////////////////////
//
// class KRect

class KRect //tag : rc
{
public:
    int _nLeft;
    int _nTop;
    int _nRight;
    int _nBottom;

    KRect() : _nLeft(0), _nTop(0), _nRight(0), _nBottom(0){ }
    KRect(const KRect& rc)
    {
          _nLeft = rc._nLeft; _nTop = rc._nTop; _nRight = rc._nRight; _nBottom = rc._nBottom;
    }
    KRect(const KRECT& rc)
    {
        _nLeft = rc.nLeft; _nTop = rc.nTop; _nRight = rc.nLeft + rc.nWidth - 1; _nBottom = rc.nTop + rc.nHeight - 1;
    }
    KRect(const int& x1, const int&y1,const int& x2, const int&y2)
    {
          _nLeft = x1; _nTop = y1; _nRight = x2; _nBottom = y2;
    }
    KRect(const KPoint& ptLT,const KPoint& ptRB)
    {
          _nLeft  = (int)(ptLT._dX+0.5); _nTop    = (int)(ptLT._dY+0.5);
          _nRight = (int)(ptRB._dX+0.5); _nBottom = (int)(ptRB._dY+0.5);
    }
    virtual ~KRect(){ }

    void    Create(const int& x1, const int&y1,const int& x2, const int&y2)
            {
                _nLeft = x1; _nTop = y1; _nRight = x2; _nBottom = y2;
            }

public:
    bool	IsValid(){ return (_nRight < _nLeft || _nBottom < _nTop ? false:true); }
    int     Area() const { return (_nRight-_nLeft+1) * (_nBottom-_nTop+1); }
    void	Invalidate(){ _nLeft = _nRight = _nTop = _nBottom = 0; }
    bool    Intersect(const KRect& rc, KRect* rcpInt = 0) const;
    bool    Intersect(const int& x1,const int& y1,const int& x2,const int& y2, KRect* rcpInt = 0) const
            {
                return Intersect(KRect(x1,y1,x2,y2), rcpInt);
            }
    bool    InRect(const KPoint& pt) const
            {
                return !(pt._dX<_nLeft || pt._dX>_nRight || pt._dY<_nTop || pt._dY>_nBottom);
            }
    bool    InRect(const double& dX,const double& dY) const
            {
                return !(dX<_nLeft || dX>_nRight || dY<_nTop || dY>_nBottom);
            }
    bool	InRect(const KRect& rcIp) const
            {
                return !(rcIp._nLeft>_nLeft || rcIp._nTop>_nTop ||rcIp._nRight<_nRight || rcIp._nBottom<_nBottom);
            }

    int     		Width()  const { return  (_nRight-_nLeft+1); }
    int     		Height() const { return (_nBottom-_nTop+1); }
    unsigned int    Size()   const { return (unsigned int)((_nRight-_nLeft+1)*(_nBottom-_nTop+1)); }
    KPoint  Center()     const { return KPoint( (_nRight+_nLeft)*0.5,(_nBottom+_nTop)*0.5 ); }
    double  CenterX()    const { return (_nRight+_nLeft)*0.5; }
    double  CenterY()    const { return (_nBottom+_nTop)*0.5; }
    KPoint  LeftTop()    const { return KPoint(_nLeft,_nTop);}
    KPoint  RightBottom()const { return KPoint(_nRight,_nBottom);}

    KRect&  MovedBy(const int& nDx,const int& nDy){ _nLeft += nDx; _nTop += nDy; _nRight += nDx; _nBottom += nDy; return *this; }
    KRect&  MovedTo(const int& nCx,const int& nCy){ MovedBy(nCx-(_nRight+_nLeft)/2,nCy-(_nTop+_nBottom)/2);  return *this; }
    KRect&  Transformed(const double& dRatio,const int& nType);
    KRect   MoveBy(const int& nDx,const int& nDy) const { return KRect(_nLeft+nDx,_nTop+nDy,_nRight+nDx,_nBottom+nDy); }
    KRect   MoveTo(const int& nCx,const int& nCy) const { return MoveBy(nCx-(_nRight+_nLeft)/2,nCy-(_nTop+_nBottom)/2); }
    KRect   Transform(const double& dRatio,const int& nType, bool bSquare = false)  const ;

    KRect&  operator  = (const KRect& rc);
    KRect&  operator += (const KPoint& pt);
    KRect&  operator -= (const KPoint& pt);

};

class KRectD //tag : rc
{
public:
    double	_dLeft;
    double	_dTop;
    double	_dRight;
    double	_dBottom;

    KRectD(){};
    KRectD(const KRectD& rc){
        _dLeft = rc._dLeft; _dTop = rc._dTop; _dRight = rc._dRight; _dBottom = rc._dBottom;
    }
    KRectD(const double& x1, const double&y1, const double& x2, const double&y2){
        _dLeft = x1; _dTop = y1; _dRight = x2; _dBottom = y2;
    }
    KRectD(const KPoint& ptLT, const KPoint& ptRB){
        _dLeft = ptLT._dX; _dTop = ptLT._dY;
        _dRight = ptRB._dX; _dBottom = ptRB._dY;
    }
    virtual ~KRectD(){}

    void    Create(const double& x1, const double&y1, const double& x2, const double&y2){ _dLeft = x1; _dTop = y1; _dRight = x2; _dBottom = y2; }

public:
    bool    InRect(const KPoint& pt) const {
        return !(pt._dX<_dLeft || pt._dX>_dRight || pt._dY<_dTop || pt._dY>_dBottom);
    }
    bool    InRect(const double& dX, const double& dY) const {
        return !(dX<_dLeft || dX>_dRight || dY<_dTop || dY>_dBottom);
    }
    bool	InRect(const KRectD& rcIp) const {
        return !(rcIp._dLeft<_dLeft || rcIp._dTop<_dTop || rcIp._dRight>_dRight || rcIp._dBottom>_dBottom);
    }

    KRectD&	Transformed(const double& dRatio, const int& nType);
    int 	Width()const { return  (_dRight - _dLeft); }
    int 	Height()const { return (_dBottom - _dTop); }
};


//

////////////////////////////////////////////////////////////
//
// class KCircle
class KCircle //tag : cc
{
public:
    double	_dCx;
    double	_dCy;
    double	_dRadius;

    KCircle(){ _dCx = _dCy = _dRadius = 0.0; }
    KCircle(const KCircle& ccT){ _dCx = ccT._dCx; _dCy = ccT._dCy; _dRadius = ccT._dRadius; }
    KCircle(const KPoint& ptCen,const double& dR){ _dCx = ptCen._dX; _dCy = ptCen._dY; _dRadius = dR; }
    KCircle(const double& dCx,const double& dCy,const double& dR){ _dCx = dCx; _dCy = dCy; _dRadius = dR; }
    virtual ~KCircle(){}

    void    Create(const KPoint& ptCen,const double& dR){ _dCx = ptCen._dX; _dCy = ptCen._dY; _dRadius = dR; }
    void    Create(const double& dCx, const double& dCy,const double& dR){ _dCx = dCx; _dCy = dCy; _dRadius = dR;	}


public:
    KPoint		Center(){ return KPoint(_dCx,_dCy); }
    double&		Radius(){ return _dRadius; }
    KRect		Rect(){ return KRect((int)(_dCx-_dRadius),(int)(_dCy-_dRadius),(int)(_dCx+_dRadius),(int)(_dCy+_dRadius)); }
    KRect		RectInner(){ return KRect((int)(_dCx-_dRadius*0.707),(int)(_dCy-_dRadius*0.707),(int)(_dCx+_dRadius*0.707),(int)(_dCy+_dRadius*0.707)); }

    KCircle&	Transformed(const double& dR,int nType);
    KCircle		Transform(  const double& dR,int nType);
    bool		InCircle(const KPoint& ptTest){ return !(_SQR(ptTest._dX-_dCx)+_SQR(ptTest._dY-_dCy) > _SQR(_dRadius)); }
    bool		InCircle(const double& dX,const double& dY){ return !(_SQR(dX-_dCx)+_SQR(dY-_dCy) 	> _SQR(_dRadius)); }
    bool		InInnerRect(const double& dX,const double& dY){ return (_ABS(_dCx-dX) + _ABS(_dCy-dY) > _dRadius); }

    KCircle&  	operator =(const KCircle& cc);
};


//
////////////////////////////////////////////////////////////
//
// class KRandom

//

class KRandom   //tag : rn

{

protected:

    int		_nNum;
    int		_nIdx;
    float*	_fpRandom;

public:
    KRandom(int nNum=1000, int nType=0){ _fpRandom = 0; OnRandom(nNum,nType); }
    virtual ~KRandom(){ delete[] _fpRandom; }
public:
    void	OnRandom(int nNum,int nType=0);
    double	Generate(){ return (double)_fpRandom[_nIdx++ % _nNum]; }
};


//

////////////////////////////////////////////////////////////
//
// class KString

//

#define     _MAX_STRING_LENGTH      4097

class KString

{
public:
    int     _nSize;
    char*   _szA;

    KString(){ _nSize = 0; _szA = 0; }
    KString(const KString& stIn){ _nSize=0; _szA=0; Create(stIn._szA); }
    KString(const std::string& stIn){ _nSize=0; _szA=0; Create(stIn.c_str()); }
    KString(const char* szIn){ _nSize=0; _szA=0; Create(szIn,0); }
    KString(const char* szIn,const int& nFrom, const int& nTo){ _nSize=0; _szA=0; Create(&szIn[nFrom],nTo-nFrom+1); }
    virtual ~KString(){ if(_szA) delete[] _szA; }

    void		Create(const char* szIn,const int& nSize=0);
    void		Create(const KString& stIn){ Create(stIn._szA); }
    void		Release(){ if(_szA){ delete[] _szA; _szA=0; _nSize=0; } }

    void                Parscing(const char* szFormat, ...);
    KString&            Formatted(const char* szFormat, ...);
    static KString      Format(const char* szFormat, ...);
    static std::string  FormatStd(const char* szFormat, ...);

public:
    //retrieve data members
    char*       Pointer() const { return _szA; }
    char*       Address() const { return _szA; }
    bool		IsEmpty() const { return (_szA ? false:true); }
    int         Size()    const {    return _nSize; }

    //string operations
    int         Find(const char*    szSub, int nType=_LEFT) const;
    int         Find(const KString& stSub, int nType=_LEFT) const;
    KString&    Removed(int nS, int nE);
    KString&    Removed(const char*);
    KString     Remove(int nS, int nE) const;
    KString     RemoveFrom(int nS) const;
    KString     RemoveFrom(const char*, int nType=_LEFT) const;
    KString     Remove(const char*) const;


    //file name operations
    KString     ParentDir() const;
    KString     FilePathOnly() const;
    KString     FileNameWithoutExt() const;
    KString     FileNameWithExt() const;
    KString     FileExt() const;
    int         ToInt(){    return atoi(_szA); }
    double      ToDouble(){ return atof(_szA); }

    static KString      FileNameWithoutExt(const char* szFile);
    static KString      FileNameWithExt(const char* szFile);
    static KString      FileExt(const char* szFile);

    //file IO
    bool                Write(FILE* fp,const unsigned int& nSize=0);
    bool                Read(FILE* fp,const unsigned int& nSize=0);

    //operators
    char*		        operator  ~() const { return _szA; }
                        operator char*() const { return _szA; }
    KString             operator  + (const char* szStr);
    KString&            operator += (const char* szStr);
    KString&            operator += (const char& cIn);
    KString&            operator  = (const char* szStr);
    KString             operator  + (const KString& stIn);
    KString&            operator += (const KString& stIn);
    KString&            operator  = (const KString& stIn);
    bool                operator == (const KString& stIn);
    bool                operator == (const char* szIn);
    bool                operator != (const KString& stIn);
    bool                operator != (const char* szIn);
    char                operator [](const int& i) const { return _szA[i]; }
    char&               operator [](const int& i){ return _szA[i]; }

    friend KString	    operator + (const char*, const KString&);
};

//
////////////////////////////////////////////////////////////
//
// class KArray

template< class T > class KArray //tag : ar

{
protected:
    int     _nRow;
    int     _nCol;
    int   	_nSize;
    int     _nMode;

public:
	T**		_ppA;
	char    _szNote[100];

public:
	KArray();
	KArray(const KArray<T>& arFrom){ _ppA=0; Create(arFrom._nRow,arFrom._nCol,arFrom._ppA[0]); }
	KArray(const int& nRow, const int& nCol);
	KArray(const int& nRow, const int& nCol,T* p,int nMode=_ALLOC);
	virtual ~KArray();

	void   Create(const int& nRow, const int& nCol);
	void   Create(const int& nRow, const int& nCol,T* p,int nMode=_ALLOC);

    void   Release();
    void   Clear(){ if(_ppA) memset(_ppA[0],0,sizeof(T)*Size()); }
    T*     Address() const { return (_ppA ? _ppA[0]:0); }
    T*	   Pointer() const { return (_ppA ? _ppA[0]:0); }
	T**    PPointer() const { return _ppA; }

public:
	unsigned int   	Col()  const { return _nCol; }
	unsigned int   	Row()  const { return _nRow; }
	unsigned int 	Size() const { return (_ppA ? _nSize:0); }
    KRect		  	Rect() const { return KRect(0,0,_nCol-1,_nRow-1); }
    void		    Fliped();

	T*			  operator [] ( const int& i ) const;
	KArray<T>&	  operator =  ( const T& 	t);
	KArray<T>&	  operator =  ( const KArray<T>& a);
};


class KPoint;
class KPoint3D;
class KRect;
class KVector;
class KImageGray;
class KMatrix : public KArray<double> //tag : m
{

public:
	KMatrix(){ }
	KMatrix(const KMatrix& mMat) : KArray<double>(mMat.Row(),mMat.Col(),mMat.Pointer()){ }
	KMatrix(const int& r,const int& c) : KArray<double>(r,c){  }
	KMatrix(const int& r,const int& c,const int& t){ Create(r,c,t); }
	KMatrix(const int& r,const int& c,double* p,int m=_ALLOC) : KArray<double>(r,c,p,m){ }
    virtual ~KMatrix() { }

public:
	//memory allocation
    void    Create(const KMatrix& oMat);
    void    Create(const int& r,const int& c,const int& t=0);
	void    Create(const int& r,const int& c,double* p,int m=_ALLOC){ KArray<double>::Create(r,c,p,m); }
	//read and write the contents of the matrix
    bool    Read(FILE *fp);
    void    Write(FILE *fp) const;
	void    Write(const char *szFile,const char* szMode=0) const;
	void    WriteText(FILE *fp,const char* szFormat=0) const;
	bool    WriteText(const char* szFile,const char* szFormat=0,const char* szMode=0) const;


///////////////////////////////////////////////////////////
//setting operations
//
	//swap i row(col) and  j row(col)
    void    Swap(int i,int j,int nType);
	//set matrix according to a given type
    void    Set(int nType);
    void    Set(double nVal,int nIdx,int nType);

	//place a given matrix on (nRow,nCol)
	KMatrix& Place(int nRow,int nCol, const KMatrix& oMat);
	KMatrix& Place(int nRow,int nCol, const KPoint& pt);
	KMatrix& Place(int nRow,int nCol, const KPoint3D& pt);

	//transform into the vector form
    KVector  Vector() const;


//////////////////////////////////////////////////////////
//getting operations
//
	//extraction
    KMatrix     Sign() const;
    void        Column(int c,KVector& vOut) const;
    KVector     Column(int c) const;                
    KMatrix     Cut(int r,int c,int r2,int c2) const;
    KMatrix     Cut(int i,int nType) const;
    void        Cut(int i,int nType, KVector& vOut) const;

    KMatrix     From(int r,int c) const;
    KMatrix     To(int r2,int c2) const;
    KMatrix     Remove(int idx,int nType) const;
    KVector		Min(const int& nType) const;
    KVector     Max(const int& nType) const;

	//get diagonalized forms
    KMatrix        Diag();
	KMatrix 	   Abs();
	static KMatrix Diag(const double& d1,const double& d2);
	static KMatrix Diag(const double& d1,const double& d2,const double& d3);
	static KMatrix Diag(const KVector& v);
	static KMatrix Diag(const int& n,const double* );
	static KMatrix Diag(const KMatrix& mMat1,const KMatrix& mMat2);
	static KMatrix Diag(const KMatrix& mMat1,const KMatrix& mMat2,const KMatrix& mMat3);
	static KMatrix RepMat(const KMatrix& mMat,const int& nRow,const int& nCol);
	static KMatrix RepMat(const double&,const int& nRow,const int& nCol);
	static KMatrix Ones(const int& r, const int& c);
    static KMatrix Kron(const KMatrix& mX, const KMatrix& mY);
    //static KMatrix MeshGrid(const KVector& vX, const KVector& vY);
    //static KMatrix MeshGrid(const KVector& vX, const KVector& vY);

//////////////////////////////////////////////////////////
//mathmatical operations
//
	//inversion
    void        Iv(KMatrix&);
    KMatrix     Iv() const;
	KMatrix     IvDefault();
	//diagonalize
    KMatrix		Diagonalize();
	//normalize
    void        Normalized(const int& nType); //행방향 혹은 열방향 벡터의 정규화
	//tranpose
    KMatrix     Tr() const;
	//speed calculation of mA*mA.Tr()
    KMatrix     Cor() const;
    KMatrix     Sc1() const;
	//decomposition
	KMatrix	    Cholesky();
    bool        QR(KMatrix& mQ,KMatrix& mR) const; //returns a 3x3 upper triangular mR and a unitary mQ so that X = Q*R
    bool		RQ(KMatrix& mR,KMatrix& mQ) const; //returns a 3x3 upper triangular mR and a unitary mQ so that X = R*Q
	double	    LU(int* nIp);
    bool        SVD(KMatrix& mU,KVector& vW,KMatrix& mV);


	//etc.
	double      Det()   const; 	//determinant
	double      Trace() const; 	//trace	
    KVector     Sum(const int& nType = _ROW) const;

	//eigen values and vectors for real and symetric matrics
	bool		Eigen(KMatrix& mEigen, KVector& vEigen);
	//generalized eigen problem( mA*vEigen = dEigen*mB*vEigen )
	KVector     Eigen(KMatrix& mA,KMatrix& mB);
	double		Pythag(double a, double b);
/*
	//sparse matrix - referenced to chap.2 of "Numerical Recipes in C"
	void		    ToSparseMode(KList<double>& lA, KList<unsigned int>& lIdx, const double& dThresh=0.0);
	static KVector  SparseAX(const KList<double>& lA,  const KList<unsigned int>& lIdx,const KVector& vX);
	static KVector  SparseATX(const KList<double>& lA, const KList<unsigned int>& lIdx,const KVector& vX);
	static KVector  SparseAsolve(const KList<double>& lA, const KList<unsigned int>& lIdx,const KVector& vB);
	static double  	LinBCG(KList<double>& lA, KList<unsigned int>& lIdx,const KVector& vB,KVector& vX,const int& nITR,const double& dTOL);
*/

//operators
//
	int			operator ==(const KMatrix&);
	int			operator !=(const KMatrix&);
	KMatrix&	operator  =(const KMatrix&);
    KMatrix&    operator  =(const KImageGray&);
	KMatrix&	operator +=(const KMatrix&);
    KMatrix&	operator +=(const double&);
	KMatrix&	operator -=(const KMatrix&);
    KMatrix&	operator -=(const double&);
	KMatrix&	operator *=(const double&);
	KMatrix&	operator *=(const KMatrix&);
	KMatrix&	operator /=(const double&);
	KMatrix&	operator /=(const KMatrix&);      // ./ as in MatLab
	KMatrix&    operator  =(const double&);
	KMatrix	    operator  *(const KMatrix&) const;
	KVector     operator  *(const KVector&) const;
	KVector	    operator  *(const KPoint&)  const;
	KMatrix	    operator  +(const KMatrix&) const;
	KMatrix	    operator  &(const KMatrix&) const;  //element-wise multiplication

	KMatrix	    operator  +(const double&) const;
	KMatrix	    operator  -(const double&) const;
	KMatrix	    operator  *(const double&) const;
	KMatrix	    operator  /(const double&) const;

	KMatrix 	operator  /(const KMatrix&) const;       // ./ as in MatLab
	KMatrix	    operator  -(const KMatrix&) const;
    KMatrix&    operator |=(const KMatrix&);      // add a column
    KMatrix	    operator  |(const KMatrix&) const;      // add a column
    KMatrix	    operator  ^(const KMatrix&) const;      // add a row
    KMatrix&	operator ^=(const KMatrix&);
    KMatrix&	operator ^=(const double&);       //element-wise exponential computation
    KMatrix	    operator  ^(const double&) const;
    KMatrix	    operator  -() const;              // * (-1)
    KMatrix	    operator  ~() const;              // inverse


	friend KMatrix	operator * (const double&, const KMatrix&);
};

//
////////////////////////////////////////////////////////////
//
// class KVector
//
class KImageGray;
class KVector : public KMatrix //tag : v
{
public:
	KVector(){ }
    KVector(const int& nDim) : KMatrix(nDim, 1){ }
    KVector(const int& nDim,double* pAdd,int nMode=_ALLOC) : KMatrix(nDim,1,pAdd,nMode){ }
    KVector(const KPoint& pt)   : KMatrix(2,1){ _ppA[0][0]=pt._dX; _ppA[1][0]=pt._dY; }
    KVector(const KPoint3D& pt) : KMatrix(3,1){ _ppA[0][0]=pt._dX; _ppA[1][0]=pt._dY; _ppA[2][0]=pt._dZ; }
    KVector(const KPOINT3F& pt) : KMatrix(3,1){ _ppA[0][0]=pt.fX;  _ppA[1][0]=pt.fY;  _ppA[2][0]=pt.fZ; }
    KVector(const KCOLOR32& pt) : KMatrix(3,1){ _ppA[0][0]=pt.r;   _ppA[1][0]=pt.g;   _ppA[2][0]=pt.b; }
    KVector(const KCOLOR24& pt) : KMatrix(3,1){ _ppA[0][0]=pt.r;   _ppA[1][0]=pt.g;   _ppA[2][0]=pt.b; }
    KVector(const KRect& rc) : KMatrix(4,1){ _ppA[0][0]=rc._nLeft; _ppA[1][0]=rc._nTop;_ppA[2][0]=rc._nRight;_ppA[3][0]=rc._nBottom; }
	KVector(const double& d1, const double& d2) : KMatrix(2,1){ _ppA[0][0] = d1; _ppA[1][0] = d2;}
	KVector(const double& d1, const double& d2, const double& d3) : KMatrix(3,1){
		_ppA[0][0] = d1; _ppA[1][0] = d2; _ppA[2][0] = d3;
	}
	KVector(const double& d1, const double& d2, const double& d3,const double& d4) : KMatrix(4,1){
		_ppA[0][0] = d1; _ppA[1][0] = d2; _ppA[2][0] = d3; _ppA[3][0] = d4;
	}
	KVector(const double& d1, const double& d2, const double& d3,const double& d4,const double& d5) : KMatrix(5,1){
		_ppA[0][0] = d1; _ppA[1][0] = d2; _ppA[2][0] = d3; _ppA[3][0] = d4; _ppA[4][0] = d5;
	}

	KVector(const KMatrix& mMat) : KMatrix(mMat.Size(),1,mMat.Pointer()) { }
    KVector(const KVector& oVec) : KMatrix(oVec.Row(),1,oVec.Pointer())  { }
    virtual ~KVector() { }

public:
	//getting functions
    int     Dim() const { return Row(); }  //dimension
	void    Angle(double& a,double& b,double& c,int nType=_DEG); //direction angle(-180 ~ 180)
	double  Angle(int nType = _DEG); // -180 ~ 180

	//memory allocation
	void	Create(const int& nDim);
	void	Create(const int& nDim,double* pV,int nMode=_ALLOC);
    void    Create(const double& d1, const double& d2){
                KMatrix::Create(2,1); _ppA[0][0] = d1; _ppA[1][0] = d2;
            }
    void    Create(const double& d1, const double& d2, const double& d3){
				KMatrix::Create(3,1);
				_ppA[0][0] = d1; _ppA[1][0] = d2; _ppA[2][0] = d3;
			}
    void    Create(const double& d1, const double& d2, const double& d3,const double& d4){
				KMatrix::Create(4,1);
				_ppA[0][0] = d1; _ppA[1][0] = d2; _ppA[2][0] = d3; _ppA[3][0] = d4;
			}
    void    Create(const double& d1, const double& d2, const double& d3,const double& d4,const double& d5){
				KMatrix::Create(5,1);
				_ppA[0][0] = d1; _ppA[1][0] = d2; _ppA[2][0] = d3; _ppA[3][0] = d4; _ppA[4][0] = d5;
			}

	//file io
	bool		Read(FILE* fp);
    bool        Read(char* szFile);
	int			Write(FILE* fp) const;
	void 		Write(const char *szFile,const char* szMode=0) const;
	void		WriteText(FILE* fp,const char* szFormat=0) const;
    void    	WriteText(const char* szFile,const char* szFormat=0,const char* szMode=0) const;
   //operations
	KVector     From(int nRow) const;
	KVector     To(int nRow) const;    
    void        QuickSorted(std::vector<int>* lpIdx = 0);
    void        Sorted(int nType,int nNum = 0); //처음부터 nNum개수의 요소만 정렬, "0"은 전체 정렬    
    void        SortedBy(std::vector<int> lpIdx);
    void        Sorted(int nType,KVector& vX); // vX is sorted accordingly
    void        Sorted(int nType,KVector& vX, KVector& vY);// vX and vY are sorted accordingly
	KVector		Cut(int nFrom,int nTo) const;
    void        Find(const double& value, KVector& vIndex) const;
    KVector		Insert(int r,double d) const;
	KVector		Tail(double d) const;
    KVector&	Tailed(double d);
	KVector		Tail(const KVector& v) const;
    KVector&    Tailed(const KVector& v);
    KVector		Floor() const;
    KVector&	Floored();

    void        Place(int r,const KMatrix& v){ KMatrix::Place(r,0,v); }
//    void        Trim(int nDummy){ Row() = (Row()-1 > nDummy ? nDummy : Row()); } //neglect elements from the nDummy time
	KMatrix     Skew(); //skew symetric matrix:  a x b = a.Skew() * b
	//math    
    double      Average();
    double      Variance();
    double      Nearest(const double& dValue, int* npIdx=0);
    void 		Statistics(double& dMean, double& dVar);
	double      Median(const int& nType=_MEDIAN_RANK,const int& nTo=0);
    KVector&    MedianFiltered(const int& nWindow);
	double      Max(int* npIdx = 0,int nMode = _DEFAULT);  //nMode: _DEFAULT, _ABSOLUTE
	double      Min(int* npIdx = 0,int nMode = _DEFAULT);
	double      Sum() const;
    double      SumAbs() const;
    double		Norm2() const;              // V'* V
    double	    Norm2(const KMatrix& mA) const;  // V'* mA * V    
    KVector&    Normalized(int nMode,double* dpOut=0);
    KVector     Normalize(int nMode) const;
    KVector     Exp() const; // return KVector( exp(_ppA[0][0]),.., exp(_ppA[Dim-1][0]) )

	static double 	Distance2(const KVector& v1,const KVector& v2);
	static KVector  Min(const KVector& v1,const KVector& v2);
	static KVector  Max(const KVector& v1,const KVector& v2);
    static void     QuickSorted(double dpA[], int first, int last, std::vector<int>* lpIdx = 0);

	//operator
    KVector&    operator  =(const KImageGray&);
	KVector&    operator  =(const KPoint&);
	KVector&    operator  =(const KCOLOR32&);
	KVector&    operator  =(const KCOLOR24&);
	KVector&    operator  =(const double&);
	KVector     operator  +(const KMatrix&) const;
	KVector     operator  -(const KMatrix&) const;
	KVector     operator  +(const KVector&) const;
	KVector     operator  -(const KVector&) const;
	KMatrix     operator  *(const KMatrix&) const;
	KVector     operator  /(const KVector&) const;
	KVector     operator  *(const double&)  const;
	KVector     operator  /(const double&)  const;
	KVector	    operator  -() const;                // * (-1)
    double      operator  &(const KVector&) const; //inner product
    double      operator [](const int& i) const { return _ppA[i][0];}
    double&     operator [](const int& i){ return _ppA[i][0];}

	friend KVector	operator * (const double&, const KVector&);
};

class KVectors : public std::vector<KVector>
{

public:
    KVectors(int nNum = 0) { if(nNum) Create(nNum); }
    KVectors(const KVectors& vsIn);
	virtual ~KVectors(){ }

    void		Create(int nNum) { std::vector<KVector>::resize(nNum); }
    KVectors&	operator =  ( const KVectors& vsInput);
};

class KIDVector : public KVector
{
    size_t _nID;

public:
	KIDVector(){ }
    KIDVector(const int nID,const KVector& vIn) : KVector(vIn) { _nID = nID; }
    KIDVector(const KIDVector& vIn) : KVector(vIn) { _nID = vIn._nID;   }
	virtual ~KIDVector(){ }

    size_t  ID() const { return _nID; }
	void	Write(FILE* fp){ fwrite(&_nID,sizeof(int),1,fp); KVector::Write(fp); }
	bool    Read(FILE* fp){ if(fread(&_nID,sizeof(int),1,fp) != 1) return false; return KVector::Read(fp); }

};



class KImageDouble;
class KImageColor;
class KImageColor24;

class KImageGray : public KArray<unsigned char>
{

public:
    KImageGray(){ }
	KImageGray(int row,int col) : KArray<unsigned char>(row,col){ }
    KImageGray(const KImageGray& igImg) : KArray<unsigned char>(igImg.Row(), igImg.Col(), igImg.Address()){ }
	KImageGray(int row,int col,unsigned char* p,int t=_ALLOC) : KArray<unsigned char>(row,col,p,t){ }
    virtual ~KImageGray(){ }

    KImageGray&  Cleared(const KRect& rcArea,const int& nType = _INWARD); //set the given area to 0
    KImageGray&  Cleared(){ if(_ppA) memset(_ppA[0],0,Size()); return *this; }
    KImageGray&  Activated(const KRect& rcArea,const int& nType = _INWARD); //set the given area to 255
    KImageGray&  Activated(){ if(_ppA) memset(_ppA[0],0xFF,Size()); return *this; }


    //KRect        Rect() const { return KRect(0,0,Col()-1, Row()-1); }
    KImageGray   Crop(int jx, int iy, int nWidth, int nHeight) const;
    KImageGray   Crop(const KRect& rcCrop) const;
    void         Crop(const KRect& rcCrop,KImageGray& igOut) const;
    void         Crop(int nXi,int nYi, int nWidth,int nHeight,KImageGray& iaOut) const;
    void         Crop(int nXi,int nYi, int nWidth,int nHeight,KVector& vOut) const;
    void         Place(int nXi,int nYi,const KImageGray&);

    bool         InImage(const double& dX, const double& dY) const { if(dX<0.0 || dX > Col()-1 || dY<0.0 || dY > Row()-1 ) return false; return true; }
    bool         InImage(const KPoint& ptP) const { if(ptP._dX<0.0 || ptP._dX > Col()-1 || ptP._dY<0.0 || ptP._dY > Row()-1 ) return false; return true; }
    KImageGray   Rotate(const double& dRad,const double& dXo=0.0, const double& dYo=0.0) const;
    KImageGray   Rotate(const double& dRad,const KPoint& pt=KPoint(0.0,0.0)) const;
    KImageGray   Transform(const KMatrix& mHomography)   const;
    KImageGray&  Transformed(const KMatrix& mHomography);
//    void         IntegralImage(KArrayDWord& arInt) const;
//    void         IntegralImage2(KArrayDWord& arInt, KArrayDWord& arIntSquare) const;

    void         HalfSize(KImageGray& igHalf);
    KImageGray   HalfSize();

    KImageGray&  HistoEqualized();
    KImageGray   HistoEqualize() const;
    void         HistoEqualize(KImageGray&) const;
    KImageDouble Normalize(int nType) const;
    void         Normalize(int nType, KImageDouble& idImg) const;

    KImageColor  GrayToRGB();
    void         GrayToRGB(KImageColor& icOut);


    void         Threshold(const int& nThresh, KImageGray& igOut);
    KImageGray&  Thresholded(const int& nThresh);
    KImageGray   BinaryDilate(const int& nType);
    KImageGray   BinaryErode(const int& nType);
    void         BinaryDilated(const int& nType);
    void         BinaryEroded(const int& nType);
    KImageGray   GrayDilate(){ return KImageGray(); }
    void         GrayDilated(){ }
//    KBlobList*   BlobColoring(int nType, KRect* rcpScan=0,KImageColor* icpColor=0);
//    KBlobNList*  Boundaries(int nType, KRect* rcpScan=0) const;
    KImageGray   Solarize(); //픽셀값 반전
    KImageGray&  Solarized();
    KImageGray&  Scaled(const unsigned char& ucFrom, const unsigned char& ucTo);

    void         Vectorize(KVector& vOut,bool bMedian = false);
    KVector      Vectorize(bool bMedian = false);

    unsigned char   BilinearInterpolation(const double& dU,const double& dV) const;
    void            BilateralFiltered(const double& dSigmaS, const double& dSigmaR);
    double          LoGFiltered(int nWinSize,double dSigma,double dOffset);
    double          TotalVariation(const KRect& rcWindow) const;
    KImageGray      ContrastTransform(const int& nA, const int& nB,const int& nAp, const int& nBp);
    KImageGray&     LogarithmTransformed();
    void            MaxValueFiltered(int nMaskSize);
    void            MinValueFiltered(int nMaskSize);
    void            Convolution(const KArray<double>& arMask,KImageDouble& idOut);
    KImageGray&     Convoluted(const KVector& vKernel,const int& nAxis);
    KImageGray&     GaussianSmoothed(const double& dSigma);
    KImageGray      GaussianSmoothing(const double& dSigma);
    KImageGray&     MedianFiltered(int nWindow=3);
    KImageGray      MedianFiltering(int nWindow=3);

    KImageGray&     operator  =(const unsigned char&);
    KImageGray&     operator  =(const KImageColor24&);
    KImageGray&     operator  =(const KImageColor&);    
};


class KImageTriplet;
class KImageColor24;
class KImageColor : public KArray<KCOLOR32>
{

public:
	KImageColor(){ }
	KImageColor(int row,int col) : KArray<KCOLOR32>(row,col){ }
    KImageColor(const KImageColor& icImg) : KArray<KCOLOR32>(icImg.Row(),icImg.Col(),icImg.Address()){ }
	KImageColor(int row,int col,KCOLOR32* p,int t=_ALLOC) : KArray<KCOLOR32>(row,col,p,t){ }
	virtual ~KImageColor(){ }

    void            Create(int row,int col){ KArray<KCOLOR32>::Create(row,col); }
    void            Create(int row,int col,KCOLOR32* p,int t=_ALLOC){ KArray<KCOLOR32>::Create(row,col,p,t); }

    void            SetIndex(int id){ sprintf(_szNote,"%d",id); }
    int             Index(){ return atoi(_szNote); }
    void            Stretch(int row,int col,KImageColor& icOut);
    void            SplitInto(KImageGray& igR,KImageGray& igG,KImageGray& igB) const;
    KImageColor     HalfSize();
    KImageGray      RedBand();
    KImageGray      GreenBand();
    KImageGray      BlueBand();
    KCOLOR32        BilinearInterpolation(const double& dU,const double& dV) const;
    KImageColor     Transform(const KMatrix& mHomography)   const;
    KImageColor&    Transformed(const KMatrix& mHomography);

    bool            InImage(const double& dX, const double& dY) const { if(dX<0.0 || dX > Col()-1 || dY<0.0 || dY > Row()-1 ) return false; return true; }

    void            RGBtoYUV(KArray<KYUV>& arYUV) const;
    void            RGBtoYUV(const int& x,const int& y,KYUV& oYUV) const;
    void            RGBtoYUV(KImageGray& igY,KImageGray& igU,KImageGray& igV) const;
    void            RGBtoHSI(KArray<KHSI>& arHSI) const;
    void            RGBtoHue(KImageGray& igHue) const;
    void            RGBtoHSV(KArray<KHSV>& arHSV) const;
    KImageColor&    FromHSV(KArray<KHSV>& arHSV);
    KImageColor&    FromRGB(const KImageGray& igR, const KImageGray& igG, const KImageGray& igB);
    void            RGBtoHSI(const int& x,const int& y,KHSI& oHSI) const;
    void            RGBtoIRG(KArray<KIRG>& arIRG) const;
    void            RGBtoIRG(const int& j, const int& i, KIRG& oIRG) const;
    void            ColorToGray(KImageGray& igOut) const;
    KImageGray      ColorToGray() const;
    double          Intensity(const int& x, const int& y) const
	{
					return (0.299*_ppA[y][x].r + 0.587*_ppA[y][x].g + 0.114*_ppA[y][x].b);
	}
    void            ToColor24(KImageColor24& icImg) const;
    void            Crop(const KRect& rcCrop,KImageTriplet& itOut,bool bNormalize=false) const;
    void            Crop(const KRect& rcCrop,KImageColor& icOut) const;
    void            Crop(const KRect& rcCrop,KImageGray&  igOut,int nType=_GRAY) const;
    void            Log(KImageTriplet& itOut);
    void            Convolution(const KArray<double>& arMask,KImageTriplet& itOut);
    KImageColor&    GaussianSmoothed(const double& dSigma);
    KImageColor     GaussianSmoothing(const double& dSigma);

};

class KImageColor24 : public KArray<KCOLOR24>
{
public:
    KImageColor24(){ }
    KImageColor24(const KImageColor24& icImg) : KArray<KCOLOR24>(icImg.Row(),icImg.Col(),icImg.Address()){ }
	KImageColor24(int row,int col) : KArray<KCOLOR24>(row,col){ }
	KImageColor24(int row,int col,KCOLOR24* p,int t=_ALLOC) : KArray<KCOLOR24>(row,col,p,t){ }

    virtual ~KImageColor24(){ }

    void            Create(int row,int col);
    void            Create(int row,int col,KCOLOR24* p,int t=_ALLOC);
    void            SetIndex(int id){ sprintf(_szNote,"%d",id); }
    int             Index( ){ return atoi(_szNote); }
    void            SplitInto(KImageGray& igR,KImageGray& igG,KImageGray& igB) const;
    KTRIPLET        BilinearInterpolation(const double& dU,const double& dV) const;

    bool            InImage(const double& dX, const double& dY){ if(dX<0.0 || dX > Col()-1 || dY<0.0 || dY > Row()-1 ) return false; return true; }

    void            RGBtoYUV(KArray<KYUV>& arYUV) const;
    void            RGBtoYUV(KImageGray& igY,KImageGray& igU,KImageGray& igV) const;
    void            RGBtoHSI(KArray<KHSI>& arHSI) const;
    void            RGBtoHSV(KArray<KHSV>& arHSV) const;
    void            RGBtoHSI(const int& x,const int& y,KHSI& oHSI) const;
    void            RGBtoIRG(KArray<KIRG>& arIRG) const;
    void            ColorToGray(KImageGray& igOut) const;
    void            ToColor32(KImageColor& icImg) const;
    KImageColor     ToColor32() const;
    void            Crop(const KRect& rcCrop,KImageColor24& icOut) const;
    void            Crop(const KRect& rcCrop,KImageGray&  igOut) const;
    KImageGray      ColorToGray() const;
};

class KImageDouble : public KArray<double>
{
public:

	KImageDouble(){};
	KImageDouble(int row,int col) : KArray<double>(row,col){ }
	KImageDouble(int row,int col,double* p,int t=_ALLOC) : KArray<double>(row,col,p,t){ }
    KImageDouble(const KImageDouble& idImg) : KArray<double>(idImg.Row(),idImg.Col(),idImg.Address()){ }
	KImageDouble(const KImageGray& igImg,const int& nOption =_NO_SCALING);
	virtual ~KImageDouble(){};

                    // _NO_SCALING         0
                    // _DEFAULT_SCALING    1
                    // _USER_SCALING       2
    void            Write(KImageGray& igImg,int nOption=_NO_SCALING,int nSi=0,int nSf=255,double dMin=0.0,double dMax=0.0);
    void            ScaleTo(KImageGray& igImg,int nOption=_NO_SCALING,int nSi=0,int nSf=255,double dMin=0.0,double dMax=0.0)
                        {  Write(igImg,nOption,nSi,nSf,dMin,dMax); }
    KImageDouble&   Scaled(const double& dFrom, const double& dTo);
    void            ToGray(KImageGray& igImg,int nOption=_NO_SCALING,int nSi=0,int nSf=255,double dMin=0.0,double dMax=0.0)
                        {  Write(igImg,nOption,nSi,nSf,dMin,dMax); }
    KImageGray      ToGray(int nOption=_NO_SCALING,int nSi=0,int nSf=255,double dMin=0.0,double dMax=0.0)
                        {  KImageGray igOut; Write(igOut,nOption,nSi,nSf,dMin,dMax); return igOut; }

                    //parameters
                    // - nType : _ROW, _COL
                    // - vKernel : dimension is odd
                    // - mKernel : dimension is odd, column size = row size
    void            Convolution(const KMatrix& mKernel,KImageDouble& idOut);
    void            Convolution(const double& dSigma,  const int& nAxis,KImageDouble& idOut,const bool& bExcludeBound=true);
    KImageDouble    Convolution(const double& dSigma,  const int& nAxis,const bool& bExcludeBound=true);
    void            Convolution(const KVector& vKernel,const int& nAxis,KImageDouble& idOut,const bool& bExcludeBound=true);
    KImageDouble    Convolution(const KVector& vKernel,const int& nAxis,const bool& bExcludeBound=true);
    KImageDouble    Integral() const ;
    void            Integral(KImageDouble& idInt, KImageDouble& idInt2) const;

    KImageDouble    DoubleSize();
    KImageDouble    HalfSize();
    double          Pixel(const double& dX,const double& dY) const;

    KImageDouble    Log();
	KImageDouble&   operator  = (const KImageGray& igImg);
    KImageDouble&   operator -= (const KImageDouble& idImg);
    KImageDouble    operator  - (const KImageDouble& idImg);

};


class KImageTriplet : public KArray<KTRIPLET>
{
public:
    KImageTriplet(){ }
	KImageTriplet(int row,int col) : KArray<KTRIPLET>(row,col){  }
    virtual ~KImageTriplet(){ }

	void            Create(int row,int col){ KArray<KTRIPLET>::Create(row,col); }

	KImageDouble    RedBand();
	KImageDouble    GreenBand();
	KImageDouble    BlueBand();

	void            Convolution(const KArray<double>& arMask,KImageTriplet& itOut);
	void            Log(KImageTriplet& itOut);
	void            Exp(KImageTriplet& itOut);
	KImageTriplet   Convolution(const KArray<double>& arMask);
	KImageTriplet   Log();
	KImageTriplet   Exp();

	void            Write(KImageGray& igImg,int nBand,int nOption,int nSi=0,int nSf=0,double dMin=0.0,double dMax=0.0);
						 //nBand : _RED, _GREEN, _BLUE
						 //nOption : _NO_SCALING, _DEFALT_SCALING, _USER_SCALING
	//
	void            Tmp(KImageColor& icImg,KImageTriplet& itIni,KImageTriplet& itLpc);
};

class KImageWord : public KArray<unsigned short>
{
public:

    KImageWord(){ }
	KImageWord(int row,int col) : KArray<unsigned short>(row,col){  }
    KImageWord(const KImageWord& iwImg) : KArray<unsigned short>(iwImg.Row(),iwImg.Col(),iwImg.Address()){ }
	KImageWord(int row,int col,unsigned short* p) : KArray<unsigned short>(row,col,p){  }
    virtual ~KImageWord(){ }

	void            Create(int row,int col){ KArray<unsigned short>::Create(row,col); }
	void            Create(int row,int col,unsigned short* p){ KArray<unsigned short>::Create(row,col,p); }

	void			ScaleTo(KImageGray& igImg,const unsigned short& wMax=0);
	KImageWord&		BilateralFiltered(const double& dSigmaS, const double& dSigmaR);
	KImageWord&		HybridFiltered(const double& dSigmaS, const double& dSigmaR);
	KImageWord&		MedianFiltered(const double& dSigmaS);

	KImageWord&		operator = (const unsigned short& wIn);
};
/////////////////////////////////////////////////////////////////////////////////

class KPGM : public KImageGray //tag : gm
{
	KImageWord	_iwImg;  //for a 16bit PGM file
	double		_dScale;

public:
	KPGM( ){ }
	KPGM(int row,int col) : KImageGray(row,col){ }
    KPGM(const KImageGray& igImg){ KImageGray::Create(igImg.Row(),igImg.Col(),igImg.Address()); }
    KPGM(const KImageWord& iwImg);
    KPGM(const char* szName){ Load(szName); }
	virtual ~KPGM(){ }

public:
	virtual bool   	Load(const char* szName);
	void	        Save(const char* szName);

	KImageWord&		ImageWord(){ return _iwImg; }
	void			UpdateData();

	KPGM&   operator  =(const KImageGray&);
	KPGM&   operator  =(const KImageWord&);
	KPGM&   operator  =(KPGM&);
};



class KPPM : public KImageColor //tag : pm
{
public:
	KPPM( ){ };
    KPPM(const char* szName){ Load(szName); }
	KPPM(int nRow,int nCol) : KImageColor(nRow,nCol){ }
	KPPM(const KImageColor& oImg) : KImageColor(oImg.Row(),oImg.Col(),oImg.Pointer()){ }
	~KPPM(){ };

public:
	virtual bool   	Load(const char* szName);
	void	        Save(const char* szName);

	KPPM&  operator  =(const KImageColor&);
};


////////////////////////////////////////////////////////////
//
// class KRotation

class KRotation : public KMatrix //tag : r
{
public:
   KRotation() : KMatrix(3,3,_IDENTITY){ }
   KRotation(const KMatrix& mMat):KMatrix(mMat.Row(),mMat.Col(),mMat.Pointer()){  }   
   KRotation(const double& a,const double& b,const double& c,const int& nT);
   KRotation(const double& a, const double& b,const double& c,const double& d) : KMatrix(3,3){ FromQuarternion(a,b,c,d); }
   KRotation(const int& nAxis,const double& dRad) : KMatrix(3,3){ FromAxisRotation(nAxis,dRad); }
   virtual ~KRotation(){}

   KRotation&	FromAxisRotation(const int& nAxis,const double& dRad);
   KRotation&	FromAxisRotation(const double& nx,const double& ny,const double& nz,const double& dRad);
   KRotation&	FromAxisRotation(const KVector& vR,const double& dRad){ return FromAxisRotation(vR[0],vR[1],vR[2],dRad); }
   KRotation&	FromRodrigues(const double& a,const double& b,const double& c);
   KRotation&	FromEuler(const double& yaw,const double& pitch,const double& roll,const int& nT);
   KRotation&	FromQuarternion(const double& s,const double& q1,const double& q2,const double& q3);
   KRotation&	FromQuarternion(const KVector& vQ){ return FromQuarternion(vQ[0],vQ[1],vQ[2],vQ[3]); }
   void     	Rodrigues(double& a,double& b,double& c);
   void     	Euler(double& a,double& b,double& c,const int& nT);
   void     	Quarternion(double& s,double& q1,double& q2,double& q3);
   void     	AxisRotation(KVector& vAxis,double& dPhi);
   void     	Angle(int nCol,double& a,double& b,double& c,int nType);

   KVector  	Rodrigues();
   KVector  	Euler(const int& nT=_DEG);
   KVector  	Quarternion();
   void     	Quarternion(KVector& vQ);

   KRotation&   Orthogonalized(); //find the nearest orthogonal rotation matrix from itself
   KRotation	Orthogonalize();
   bool			IsRightHand();

   KRotation&  operator =(const KMatrix&);
   KRotation   operator *(const KRotation&);
   KVector	   operator *(const KPoint3D&);
   KMatrix     operator *(const KMatrix&);
   KVector     operator *(const KVector&);
};

///////////////////////////////////////////////////////////
//
// class KHomogeneous
//
class KHomogeneous : public KMatrix //tag : h
{
public:
    KHomogeneous() : KMatrix(4,4,_IDENTITY){ }
    KHomogeneous(const KHomogeneous& hT) : KMatrix(4,4,hT.Address()){ }
    KHomogeneous(const KRotation& rR,const KVector& vT) : KMatrix(4,4,_IDENTITY){ Place(0,0,rR); Place(0,3,vT); }
    KHomogeneous(const KMatrix& mT) : KMatrix(4,4,mT.Address()){ }
    KHomogeneous(const KVector& vRT, int nType = _EULER) : KMatrix(4,4,_IDENTITY) {  Create(vRT, nType); }


    void 		 Create(const KVector& vRT, int nType);
    KHomogeneous Iv() const;
    KRotation    R()  const  { return KRotation(this->To(2,2)); }
    KVector 	 t()  const  { return KVector(_ppA[0][3],_ppA[1][3],_ppA[2][3]); }    
    KVector      RT(int nType=_EULER);
    KHomogeneous& operator =(const KHomogeneous&);
    KHomogeneous  operator ~() const;
};


////////////////////////////////////////////////////////////
//
// class KQuaternion

class KQuaternion : public KVector //tag:q
{

public:
	KQuaternion() : KVector(4){ }
	KQuaternion(const KVector& vAxis,const double& dTheta) : KVector(4){ Create(vAxis,dTheta); }
	virtual ~KQuaternion(){ }

	void            Create(const KVector& vAxis,const double& dTheta);

	KRotation       Rotation();
	void            AxisRotation(KVector& vAxis,double& dTheta);

    KQuaternion&    operator =(const KVector&);
    KQuaternion     operator *(const KQuaternion&);
    KQuaternion     operator ~() const;
};



class KGaussian   //tag : g
{
protected:
    double      	_dMean;
    double			_dSigma;

	int				_nNum;	
	double      	_dVar,_dVar2;	
	int				_nIdx;
	float*      	_fpRandom;
	KRandom*    	_opRandom;
	double          _dRandom;
	double          _dConst;

public:
	KGaussian();
    KGaussian(const double& dMean,const double& dVar);
	virtual ~KGaussian();

public:
	void		Create(const double& dMean,const double& dVar);
	double		Pdf(const double& dX);
	double      Exp(const double& dX);
	double		Pdf(const double& dX,const double& dMean,const double& dVar);
	double	    MahaDistance(const double& dX){ return _SQR(_dMean-dX)/_dVar; }
	double&     Mean(){ return _dMean; }
	double&     Var(){  return _dVar; }
	int&        RandomSize(){ return _nNum; }
	double&		Generate( );
	double&		Generate(const double& dMean,const double& dSigma);

	bool		OnRandom(int nNum=1000);
	void		OffRandom( );

    bool        Read(FILE* file);
    void        Write(FILE* file);
	void		WriteText(FILE* file);
	KGaussian&  operator = (const KGaussian& gIn);
};
//---------------------------------------------------------------------------

class KGaussianMulti  //tag : mg
{
    int         _nDim;		//size of dimension
    KVector     _vMean;
    KMatrix     _mCov;		//covariance matrix

	KMatrix	    _mLowCov;	    //the lower matrix from Cholesky decomposition of oSigma
	KGaussian*	_gpGaussians;	//the list of single gaussian distibution
	KVector     _vRandom;       //to be used in generating random vertor
							    //drawn from this distribution
public:
	double		_dConst;
	double      _dConstNormal;
	KMatrix	    _mIvCov;		//inverse of the covariance matrix

public:
	KGaussianMulti();
	KGaussianMulti(const KVector& vMean, const KMatrix& mCov);
	KGaussianMulti(int nDim);
	virtual ~KGaussianMulti(){	if(_gpGaussians) delete[] _gpGaussians; }

	bool        Create(int nDim);    
	bool        Create(const KVector& vMean, const KMatrix& mCov);
    void		Release(){ _vMean.Release(); _mCov.Release();  if(_gpGaussians) delete[] _gpGaussians; }

    bool		Read(FILE*);
	bool        Read(char* szFile);
    void		Write(FILE*);
    void		WriteText(FILE*);
	double      Pdf(const KVector& vX);
	double      PdfNormal(const KVector& vX);
	double      Pdf(const double& d1,const double& d2); //2-d case    
	double      LogPdf(const double& d1,const double& d2); //2-d case
	double      Exp(const KVector& vX);
	double      LogPdf(const KVector& vX);
    double      LogExp(const KVector& vX);
    double	    Mahalanobis(const KVector& vX);

	static double	Pdf(const KVector& vX,const KVector& vMean,const KMatrix& mCov);
	static double	LogPdf(const KVector& vX,const KVector& vMean,const KMatrix& mCov);
	static double	Mahalanobis(const KVector& vX,const KVector& vMean,const KMatrix& mCov);
	static KVector  Kernel_1D(const double& dSigma,const int& nSize=0);
	static KVector  Kernel_Derivative(const double& dSigma,const int& nSize=0);    
	static KMatrix  Kernel_2D(const double& dSigma,const int& nSize=0);

public:
    KMatrix&        Cov() { return  _mCov; }
    KVector&        Mean() { return _vMean; }
    int             Dim() { return _nDim;  }
    bool            Enabled(){ return (bool)_mCov.Pointer(); }
    void            Update(){ Create(_vMean,_mCov); }

public:
    bool            OnRandom(int nNum = 1000);
    void            OffRandom();

	KVector&	    Generate();
	KVector&	    Generate(KMatrix& mLowCov);

	KGaussianMulti& operator = (const KGaussianMulti& gIn);
};

class KGaussianMixture //tag: gm
{
    char            _szID[200];
    int             _nNum;      //number of Gaussians
    KVector         _vWeight;   //weight of each Gaussian
    KGaussianMulti*	_gpMulti;   //pointers of Gaussian distributions
    KGaussian*      _gpSingle;  //pointers of Gaussian distributions
    int             _nType;     //_SINGLE: single-variate gaussian, _MULTI: multivariate gaussian

public:   
    KGaussianMixture(){ _gpMulti = 0; _gpSingle = 0; }
	//외부에서 가우시안 정보를 가지고 초기화 할 경우
	KGaussianMixture(const KGaussianMixture& oGmx);
	KGaussianMixture(const KVector& vWeight, KGaussianMulti* gpDtb);
    KGaussianMixture(const KVector& vWeight, KGaussian* gpDtb);
    virtual ~KGaussianMixture(){ if(_gpMulti) delete[] _gpMulti; if(_gpSingle) delete[] _gpSingle; }

    void        Create(const KVector& vWeight, KGaussianMulti* gpDtb);
    void        Create(const KVector& vWeight, KGaussian* gpDtb);
    bool        Read(FILE* fp);
    bool        Load(const char* szFile);

    //입력데이터로부터 학습할 경우에 사용
    double      Learning(int nNum,const KMatrix& mSample);//each column is a feature vector
    double      Learning(int nNum,const KVector& vSample);


	//공통
    void        Save(const char* szFile);
    void        SaveText(const char* szFile);
    void        Write(FILE* fp);

    double      Pdf(const double&);
    double      Pdf(const KVector&);
    double      PdfNormal(const KVector&);
    double      Pdf(const double& a,const double& b);
    double      LogPdf(const KVector&);
    double      LogPdf(const double&);

	//get & set
    void            ID(const char* szName){ strcpy(_szID,szName); }
    char*           ID() { return _szID; }
    int             Count() const  { return _nNum; }
    bool            Enabled(){ return (_gpMulti || _gpSingle); }
    int             Type()  const { return _nType; }
    KGaussianMulti* Gaussian(int i){ return &_gpMulti[i]; }
    KGaussianMulti* Gaussians(){ return _gpMulti; }
	KVector&        Weight(){ return _vWeight; }

	KGaussianMixture& operator = (const KGaussianMixture&);

protected:
    bool            InitLearning(int nNum,const KMatrix& mSample);
    bool            InitLearning(int nNum,const KVector& vSample);

};


template< class T > KArray<T>::KArray( )
{
	_ppA   = 0;
	_nRow  = _nCol = 0;
	_nMode = _ALLOC;
}

template< class T > KArray<T>::KArray(const int& nRow, const int& nCol)
{
	_ppA   = 0;
	_nMode = _ALLOC;
	_nRow  = _nCol = 0;

	Create(nRow,nCol);
}

template< class T > KArray<T>::KArray(const int& nRow, const int& nCol,T* pINIT,int nMode)
{
	_ppA   = 0;
	_nMode = _ALLOC;
	_nRow  = _nCol = 0;

	Create(nRow,nCol,pINIT,nMode);
}

template< class T > KArray<T>::~KArray()
{
	if(_ppA){
		if(_nMode == _ALLOC)
			delete[] _ppA[0];
		delete[] _ppA;
	}
}

template< class T > void KArray<T>::Release( )
{
	if(_ppA){
		if(_nMode == _ALLOC)
			delete[] _ppA[0];
		delete[] _ppA;
		_ppA = 0;
		_nRow = _nCol = 0;
	}
}

template< class T > void KArray<T>::Create(const int& nRow, const int& nCol,T* pINIT,int nMode)
{
#ifdef _DEBUG
	assert(!(pINIT==0 || nRow==0 || nCol==0));
#endif

	//delete the allocated memory
	if(_ppA)
	{
		if(_nMode == _ALLOC){
			if(_nMode != nMode || (_nRow != nRow || _nCol != nCol)){ delete[] _ppA[0]; delete[] _ppA; _ppA = 0; }
			else{  memcpy(_ppA[0],pINIT,_nSize*sizeof(T)); return; }
		}
		else{ //_LOCK
			if(_nMode != nMode || (_nRow != nRow || _nCol != nCol)){ delete[] _ppA; _ppA = 0; }
			else{
				_ppA[0] = pINIT;
				for(int i=1; i<nRow; i++)
					_ppA[i] = _ppA[i-1] + nCol;
				return;
			}
		}
	}

	//set
	_nMode  = nMode;
	_nRow   = nRow;
	_nCol   = nCol;
	_nSize  = (unsigned int)(nRow)*(unsigned int)(nCol);
	_ppA    = new T* [nRow];

	if(nMode == _ALLOC)
	{
        _ppA[0] = new T[_nSize];
		memcpy(_ppA[0],pINIT,_nSize*sizeof(T));
	}
	else
		_ppA[0] = pINIT;

	for(int i=1,ic=nRow-1; ic; ic--, i++)
		_ppA[i] = _ppA[i-1] + nCol;
}

template< class T > void KArray<T>::Create(const int& nRow, const int& nCol)
{
#ifdef _DEBUG
	assert(!(nRow==0 || nCol==0));
#endif

	//delete the allocated memory
	if(_ppA){
		if(_nMode==_ALLOC && (_nRow != nRow || _nCol != nCol))
		{
			delete[] _ppA[0];
			delete[] _ppA; _ppA = 0;
		}
		else if(_nMode==_LOCK && (_nRow != nRow || _nCol != nCol))
		{
            delete[] _ppA;
            _ppA = 0;
		}
		else{
			memset(_ppA[0],0,_nSize*sizeof(T));
			return;
		}
	}

	//memory alloc.
	_nMode  = _ALLOC;
	_nRow	= nRow;
	_nCol   = nCol;
	_nSize  = (unsigned int)(nRow)*(unsigned int)(nCol);

	_ppA    = new T* [nRow];
	_ppA[0] = new T[_nSize];
	memset(_ppA[0],0,_nSize*sizeof(T));

	//set pointers
	for(int i=1,ic=nRow-1; ic; ic--, i++)
		_ppA[i] = _ppA[i-1] + nCol;
}

template<class T> void KArray<T>::Fliped()
{
	T* pTmp = new T[_nCol];

	int  nTo 	= _nRow/2;
	int  nSize	= sizeof(T)*_nCol;

	for(int i=0,ii=_nRow-1; i<nTo; i++,ii--)
	{
		memcpy(pTmp,_ppA[ii],nSize);
		memcpy(_ppA[ii],_ppA[i],nSize);
		memcpy(_ppA[i],pTmp,nSize);
	}
	delete[] pTmp;
}

template<class T> T* KArray<T>::operator [] (const int& i) const
{
	return _ppA[i];
}

template<class T> KArray<T>& KArray<T>::operator = ( const KArray<T>& 	arFrom)
{
	if(arFrom._ppA)
	{
        if(_ppA == 0 || _nRow != arFrom._nRow || _nCol != arFrom._nCol)
			Create(arFrom._nRow,arFrom._nCol,arFrom._ppA[0],_ALLOC);
		else
			memcpy(_ppA[0],arFrom._ppA[0],_nSize*sizeof(T));
	}

	return *this;
}

template<class T> KArray<T>& KArray<T>::operator =  ( const T& 	t)
{
#ifdef _DEBUG
	assert(_ppA != 0);
#endif

	//copy
    for(int i=0,ic=_nRow; ic; ic--, i++)
        for(int j=0,jc=_nCol; jc; jc--, j++)
            _ppA[i][j] = t;

	return *this;
}


#endif
