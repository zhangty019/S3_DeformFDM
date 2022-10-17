// GLKGeometry.cpp: implementation of the GLKGeometry class.
//
//////////////////////////////////////////////////////////////////////

#include <stdio.h>
#include <math.h>
//#include <malloc.h>

#include "GLKLib.h"
#include "GLKMatrixLib.h"
#include "GLKGeometry.h"

//////////////////////////////////////////////////////////////////////
// Implementation
//////////////////////////////////////////////////////////////////////

bool GLKGeometry::Normalize(double n[])
{
	double tt=sqrt(n[0]*n[0]+n[1]*n[1]+n[2]*n[2]);

	if (tt<EPS) {
		n[0]=0.0;	n[1]=0.0;	n[2]=0.0;
		return false;
	}
	else{
		n[0]=n[0]/tt;	n[1]=n[1]/tt;	n[2]=n[2]/tt;
	}

	return true;
}

bool GLKGeometry::CalPlaneEquation( double & A, double & B, double & C, double & D, 
			double p1[], double p2[], double l, double m, double n)
{
	A = ( p2[1] - p1[1] ) * n - ( p2[2] - p1[2] ) * m;
	B = ( p2[2] - p1[2] ) * l - ( p2[0] - p1[0] ) * n;
	C = ( p2[0] - p1[0] ) * m - ( p2[1] - p1[1] ) * l;
	D = - ( p1[0] * A + p1[1] * B + p1[2] * C );

	double  tt = A*A + B*B + C*C;
	tt = sqrt(tt);
	if(tt < EPS)    return false;
	A = A/tt;   B = B/tt;   C = C/tt;   D = D/tt;

	return true;
}

void GLKGeometry::CalArealCoordinate(double p1[], double p2[], double p3[], double pp[], 
									  double &u, double &v, double &w)
{
	double area=SpatialTriangleArea(p1,p2,p3);

	u=SpatialTriangleArea(pp,p2,p3)/area;
	v=SpatialTriangleArea(pp,p3,p1)/area;
	w=SpatialTriangleArea(pp,p1,p2)/area;
}

void GLKGeometry::CalLineEquation( double & A, double & B, double & C, double x1, double y1, double x2, double y2)
{
	A=y2-y1;
	B=x1-x2;
	if (fabs(B)<EPS)
	{
		A=1;
		B=0;
		C=-x1;
		return;
	}
	C=-(B*y1+A*x1);
}

bool GLKGeometry::ApproximatePlaneEquation( int n, float** p,
		double & A, double & B, double & C, double & D )
{
	double**pp;	int i,j;	bool bReturn;
	GLKMatrixLib::CreateMatrix(pp,n,3);
	for(i=0;i<n;i++)
		for(j=0;j<3;j++)
			pp[i][j]=p[i][j];
	bReturn=ApproximatePlaneEquation(n,pp,A,B,C,D);
	GLKMatrixLib::DeleteMatrix(pp,n,3);

	return bReturn;
}

bool GLKGeometry::ApproximatePlaneEquation( int n, double** p,
		double & A, double & B, double & C, double & D )
{
	double **M,**U,**V;
	double cp[3],minSingularValue,ll;
	int i;
	bool rc;

	cp[0]=0.0;	cp[1]=0.0;	cp[2]=0.0;
	for(i=0;i<n;i++) {cp[0]+=p[i][0];	cp[1]+=p[i][1];	cp[2]+=p[i][2];}
	cp[0]=cp[0]/(double)n;	cp[1]=cp[1]/(double)n;	cp[2]=cp[2]/(double)n;

	GLKMatrixLib::CreateMatrix(M,n,3);
	GLKMatrixLib::CreateMatrix(V,3,3);
	GLKMatrixLib::CreateMatrix(U,n,n);
	for(i=0;i<n;i++) {M[i][0]=p[i][0]-cp[0]; M[i][1]=p[i][1]-cp[1]; M[i][2]=p[i][2]-cp[2];}
	rc=GLKMatrixLib::SingularValueDecomposition(M,n,3,U,V);
	if (rc) {
		minSingularValue=M[0][0];	A=V[0][0];	B=V[0][1];	C=V[0][2];
		if (M[1][1]<minSingularValue) {
			minSingularValue=M[1][1];	A=V[1][0];	B=V[1][1];	C=V[1][2];
		}
		if (M[2][2]<minSingularValue) {A=V[2][0];	B=V[2][1];	C=V[2][2];}

		ll=sqrt(A*A+B*B+C*C);
		if (ll>EPS) {
			A=A/ll;	B=B/ll;	C=C/ll;
			D=-(A*cp[0]+B*cp[1]+C*cp[2]);
		}
		else 
			rc=false;
	}

	GLKMatrixLib::DeleteMatrix(M,n,3);
	GLKMatrixLib::DeleteMatrix(V,3,3);
	GLKMatrixLib::DeleteMatrix(U,n,n);

	return rc;
/*
	double lumda,delta[5],bb[5],ll;
	double **T,**E;
	int i,j,k;
	bool bRC;

	GLKMatrixLib::CreateMatrix(T,5,5);
	GLKMatrixLib::CreateMatrix(E,4,4);

	A=1.0;	B=0.0;	C=0.0;	D=0.0;	lumda=1.0;

	for(k=0;k<n;k++) {
		E[0][0]+=2.0*p[k][0]*p[k][0];	E[0][1]+=2.0*p[k][0]*p[k][1];	E[0][2]+=2.0*p[k][0]*p[k][2];	E[0][3]+=2.0*p[k][0];
		E[1][0]+=2.0*p[k][1]*p[k][0];	E[1][1]+=2.0*p[k][1]*p[k][1];	E[1][2]+=2.0*p[k][1]*p[k][2];	E[1][3]+=2.0*p[k][1];
		E[2][0]+=2.0*p[k][2]*p[k][0];	E[2][1]+=2.0*p[k][2]*p[k][1];	E[2][2]+=2.0*p[k][2]*p[k][2];	E[2][3]+=2.0*p[k][2];
		E[3][0]+=2.0*p[k][0];	E[3][1]+=2.0*p[k][1];	E[3][2]+=2.0*p[k][2];	E[3][3]+=2.0;
	}

	//----------------------------------------------------------------------------------------
	//	using the non-linear optimization method to determine the equation
	//----------------------------------------------------------------------------------------
	for(int iter=0;iter<50;iter++) {
		for(i=0;i<4;i++) for(j=0;j<4;j++) T[i][j]=E[i][j];
		T[0][0]+=2.0*lumda;	T[1][1]+=2.0*lumda;	T[2][2]+=2.0*lumda;
		T[4][0]=2.0*A;	T[4][1]=2.0*B;	T[4][2]=2.0*C;	T[4][3]=0.0;
		T[0][4]=2.0*A;	T[1][4]=2.0*B;	T[2][4]=2.0*C;	T[3][4]=0.0;
		T[4][4]=0.0;
		//----------------------------------------------------------------------------------------
		bb[0]=-(E[0][0]*A+E[0][1]*B+E[0][2]*C+E[0][3]*D+2.0*lumda*A);
		bb[1]=-(E[1][0]*A+E[1][1]*B+E[1][2]*C+E[1][3]*D+2.0*lumda*B);
		bb[2]=-(E[2][0]*A+E[2][1]*B+E[2][2]*C+E[2][3]*D+2.0*lumda*C);
		bb[3]=-(E[3][0]*A+E[3][1]*B+E[3][2]*C+E[3][3]*D);
		bb[4]=-(A*A+B*B+C*C-1.0);
		ll=0.0;
		for(i=0;i<5;i++) ll+=bb[i]*bb[i];
		if (ll<1.0e-10) break;
		bRC=GLKMatrixLib::Inverse(T,5);
		if (!bRC) break;
		GLKMatrixLib::Mul(T,bb,5,5,delta);
		A+=delta[0];	B+=delta[1];	C+=delta[2];	D+=delta[3];	lumda+=delta[4];
	}
	GLKMatrixLib::DeleteMatrix(E,4,4);
	GLKMatrixLib::DeleteMatrix(T,5,5);

	if (!bRC) {
		if (CalPlaneEquation(p[0],p[1],p[2],A,B,C,D)) return true;
		return false;
	}

	ll=sqrt(A*A+B*B+C*C);	
	A=A/ll;	B=B/ll;	C=C/ll;	D=D/ll;
	return true;
*/
}

bool GLKGeometry::CalPlaneEquation( double p0[], double p1[], double p2[], double & A, double & B, double & C, double & D )
{
	double x[3],y[3],z[3];
	x[0]=p0[0];	x[1]=p1[0];	x[2]=p2[0];
	y[0]=p0[1];	y[1]=p1[1];	y[2]=p2[1];
	z[0]=p0[2];	z[1]=p1[2];	z[2]=p2[2];

	return CalPlaneEquation(A,B,C,D,x,y,z);
}

bool GLKGeometry::CalPlaneEquation( float p0[], float p1[], float p2[], double & A, double & B, double & C, double & D )
{
	double x[3],y[3],z[3];
	x[0]=p0[0];	x[1]=p1[0];	x[2]=p2[0];
	y[0]=p0[1];	y[1]=p1[1];	y[2]=p2[1];
	z[0]=p0[2];	z[1]=p1[2];	z[2]=p2[2];

	return CalPlaneEquation(A,B,C,D,x,y,z);
}

bool GLKGeometry::CalPlaneEquation( double & A, double & B, double & C, double & D, double x[], double y[], double z[])
{
	A =   y[0] * ( z[1] - z[2] )
		+ y[1] * ( z[2] - z[0] )
		+ y[2] * ( z[0] - z[1] );
	
	B =   z[0] * ( x[1] - x[2] )
		+ z[1] * ( x[2] - x[0] ) 
		+ z[2] * ( x[0] - x[1] );

	C =   x[0] * ( y[1] - y[2] )
		+ x[1] * ( y[2] - y[0] )
		+ x[2] * ( y[0] - y[1] );

	D = - x[0] * ( y[1]*z[2] - y[2]*z[1] )
		- x[1] * ( y[2]*z[0] - y[0]*z[2] )
		- x[2] * ( y[0]*z[1] - y[1]*z[0] );

	double  tt = A*A + B*B + C*C;
	tt = sqrt(tt);
	if(tt < EPS)    return false;
	A = A/tt;   B = B/tt;   C = C/tt;   D = D/tt;

	return true;
}

bool GLKGeometry::CalPlaneLineIntersection( double p[], double n[],
		double A, double B, double C, double D, double &mu)
{
	double denom;

	denom=A*n[0]+B*n[1]+C*n[2];
	if (fabs(denom)<EPS)	return false;

	mu=-(D+A*p[0]+B*p[1]+C*p[2])/denom;

	return true;
}

bool GLKGeometry::CalPlaneLineSegIntersection( double p1[], double p2[],
		double A, double B, double C, double D, double &mu)
{
	double denom;

	denom=A*(p2[0]-p1[0])+B*(p2[1]-p1[1])+C*(p2[2]-p1[2]);
	if (fabs(denom)<EPS)	return false;

	mu=-(D+A*p1[0]+B*p1[1]+C*p1[2])/denom;
	if ((mu<0.0) || (mu>1.0)) return false;

	return true;
}

void GLKGeometry::ClipPolygonByCube(double* &xp, double* &yp, double* &zp, int &pntNum,
									double xmin, double ymin, double zmin, double boxSize)
{
	double pp[3],normal[3];

	pp[0]=xmin;	pp[1]=ymin;	pp[2]=zmin;
	normal[0]=1.0;	normal[1]=0.0;	normal[2]=0.0;
	ClipPolygonByHalfSpace(xp,yp,zp,pntNum,pp,normal);
	normal[0]=0.0;	normal[1]=1.0;	normal[2]=0.0;
	ClipPolygonByHalfSpace(xp,yp,zp,pntNum,pp,normal);
	normal[0]=0.0;	normal[1]=0.0;	normal[2]=1.0;
	ClipPolygonByHalfSpace(xp,yp,zp,pntNum,pp,normal);

	pp[0]=xmin+boxSize;	pp[1]=ymin+boxSize;	pp[2]=zmin+boxSize;
	normal[0]=-1.0;	normal[1]=0.0;	normal[2]=0.0;
	ClipPolygonByHalfSpace(xp,yp,zp,pntNum,pp,normal);
	normal[0]=0.0;	normal[1]=-1.0;	normal[2]=0.0;
	ClipPolygonByHalfSpace(xp,yp,zp,pntNum,pp,normal);
	normal[0]=0.0;	normal[1]=0.0;	normal[2]=-1.0;
	ClipPolygonByHalfSpace(xp,yp,zp,pntNum,pp,normal);
}

void GLKGeometry::ClipPolygonByHalfSpace(double* &xp, double* &yp, double* &zp, int &pntNum,
									double planePnt[], double planeNormal[])
{
	GLKArray xc(50,50,3);
	GLKArray yc(50,50,3);
	GLKArray zc(50,50,3);
	int i,pntIndex,thisIndex,lastIndex;
	double dd;

	if (pntNum==0) return;

	pntIndex=-1;
	//----------------------------------------------------------------------------
	//	Step 1: determine the first positive point' index
	for(i=0;i<pntNum;i++) {
		dd=(xp[i]-planePnt[0])*planeNormal[0]
			+(yp[i]-planePnt[1])*planeNormal[1]
			+(zp[i]-planePnt[2])*planeNormal[2];
		if (dd>0.0) {pntIndex=i;break;}
	}

	//----------------------------------------------------------------------------
	//	Step 2: incrementally adding the point into the list
	bool lastFlag;
	if (pntIndex>=0) {
		thisIndex=pntIndex;
		xc.Add(xp[thisIndex]);	yc.Add(yp[thisIndex]);	zc.Add(zp[thisIndex]);
		lastFlag=true;
		for(i=1;i<=pntNum;i++) {
			lastIndex=thisIndex;
			thisIndex=(i+pntIndex)%pntNum;

			if (lastFlag) {
				dd=(xp[thisIndex]-planePnt[0])*planeNormal[0]
					+(yp[thisIndex]-planePnt[1])*planeNormal[1]
					+(zp[thisIndex]-planePnt[2])*planeNormal[2];
				if (dd>=0.0) {
					xc.Add(xp[thisIndex]);	yc.Add(yp[thisIndex]);	zc.Add(zp[thisIndex]);
					lastFlag=true;
				}
				else {
					double scale=dd/((xp[thisIndex]-xp[lastIndex])*planeNormal[0]
						+(yp[thisIndex]-yp[lastIndex])*planeNormal[1]
						+(zp[thisIndex]-zp[lastIndex])*planeNormal[2]);
					xc.Add(xp[thisIndex]*(1.0-scale)+xp[lastIndex]*scale);
					yc.Add(yp[thisIndex]*(1.0-scale)+yp[lastIndex]*scale);
					zc.Add(zp[thisIndex]*(1.0-scale)+zp[lastIndex]*scale);
					lastFlag=false;
				}
			}
			else {
				dd=(xp[thisIndex]-planePnt[0])*planeNormal[0]
					+(yp[thisIndex]-planePnt[1])*planeNormal[1]
					+(zp[thisIndex]-planePnt[2])*planeNormal[2];
				if (dd>=0.0) {
					double scale=dd/((xp[thisIndex]-xp[lastIndex])*planeNormal[0]
						+(yp[thisIndex]-yp[lastIndex])*planeNormal[1]
						+(zp[thisIndex]-zp[lastIndex])*planeNormal[2]);
					xc.Add(xp[thisIndex]*(1.0-scale)+xp[lastIndex]*scale);
					yc.Add(yp[thisIndex]*(1.0-scale)+yp[lastIndex]*scale);
					zc.Add(zp[thisIndex]*(1.0-scale)+zp[lastIndex]*scale);

					xc.Add(xp[thisIndex]);	yc.Add(yp[thisIndex]);	zc.Add(zp[thisIndex]);
					lastFlag=true;
				}
			}
		}
	}

	//----------------------------------------------------------------------------
	//	Step 3: memory allocation 
	delete xp;	delete yp; delete zp;	pntNum=xc.GetSize();
	if (pntNum>0) {
		pntNum--;
		xp=new double[pntNum];
		yp=new double[pntNum];
		zp=new double[pntNum];
		for(i=0;i<pntNum;i++) {
			xp[i]=xc.GetDoubleAt(i);
			yp[i]=yc.GetDoubleAt(i);
			zp[i]=zc.GetDoubleAt(i);
		}
	}
}

bool GLKGeometry::CalLineFacetIntersection(double p[], double n[], double &mu,
		double x[], double y[], double z[],
		double A, double B, double C, double D)
{
	double denom,sp[3],pa1[3],pa2[3],pa3[3],total;
	double a1,a2,a3,a;

	denom=A*n[0]+B*n[1]+C*n[2];
	if (fabs(denom)<EPS)	return false;
	mu=-(D+A*p[0]+B*p[1]+C*p[2])/denom;

	// Obtain the intersection point 
	for(int i=0;i<3;i++) sp[i]=p[i]+mu*n[i];

	// Determine whether or not the intersection point is bounded by x[],y[],z[]
	pa1[0]=x[0];	pa1[1]=y[0];	pa1[2]=z[0];
	pa2[0]=x[1];	pa2[1]=y[1];	pa2[2]=z[1];
	pa3[0]=x[2];	pa3[1]=y[2];	pa3[2]=z[2];

	a1=SpatialTriangleArea(sp,pa1,pa2);
	a2=SpatialTriangleArea(sp,pa2,pa3);
	a3=SpatialTriangleArea(sp,pa3,pa1);
	a=a1+a2+a3;
	total=SpatialTriangleArea(pa1,pa2,pa3);

	if (fabs(a-total)>1.0e-3) return false;

	return true;
}

#define EPSILON 0.00000001
bool GLKGeometry::CalLineFacetIntersection( double p[], double n[],
        double v0[], double v1[], double v2[],
        double& t, double& u, double& v)
{
    double edge1[3],edge2[3],tvec[3],pvec[3],qvec[3];
    double det,inv_det;
    double delta = 0.00000001;
    for(int i =0;i<3;i++)
    {
        edge1[i] = v1[i]-v0[i];
        edge2[i] = v2[i]-v0[i];
    }


    pvec[0] = n[1]*edge2[2]-n[2]*edge2[1];
    pvec[1] = n[2]*edge2[0]-n[0]*edge2[2];
    pvec[2] = n[0]*edge2[1]-n[1]*edge2[0];

    det = edge1[0]*pvec[0]+edge1[1]*pvec[1]+edge1[2]*pvec[2];


    if((det>-EPSILON)&&(det<EPSILON))
        return false;
    inv_det = 1.0/det;

    for(int j = 0;j<3;j++)
    {
       tvec[j] = p[j]-v0[j];
    }

    u = (tvec[0]*pvec[0]+tvec[1]*pvec[1]+tvec[2]*pvec[2])*inv_det;
    //if(u<0.0||u>1.0)
    if(u<((-1.0)*delta)||u>(1.0+delta))
        return false;

    qvec[0] = tvec[1]*edge1[2]-tvec[2]*edge1[1];
    qvec[1] = tvec[2]*edge1[0]-tvec[0]*edge1[2];
    qvec[2] = tvec[0]*edge1[1]-tvec[1]*edge1[0];

    v = (n[0]*qvec[0]+n[1]*qvec[1]+n[2]*qvec[2])*inv_det;

    //if(v<0.0||u+v>1.0)
      if(v<((-1.0)*delta)||u+v>(1.0+(2*delta)))
        return false;

    t = (edge2[0]*qvec[0]+edge2[1]*qvec[1]+edge2[2]*qvec[2])*inv_det;

    return true;
}

float GLKGeometry::Distance_to_Point(float p1[], float p2[])
{
	double dis=(p1[0]-p2[0])*(p1[0]-p2[0])
		+(p1[1]-p2[1])*(p1[1]-p2[1])
		+(p1[2]-p2[2])*(p1[2]-p2[2]);
	dis=sqrt(dis);

	return (float)dis;
}

double GLKGeometry::Distance_to_Point(double p1[], double p2[])
{
	double dis=(p1[0]-p2[0])*(p1[0]-p2[0])
		+(p1[1]-p2[1])*(p1[1]-p2[1])
		+(p1[2]-p2[2])*(p1[2]-p2[2]);
	dis=sqrt(dis);

	return dis;
}

double GLKGeometry::Distance_to_LineSegment(double p[], double p1[], double p2[])
{
	double vt[3],pt[3],proj,dist;

	vt[0]=p2[0]-p1[0];	vt[1]=p2[1]-p1[1];	vt[2]=p2[2]-p1[2];	Normalize(vt);
	pt[0]=p[0]-p1[0];	pt[1]=p[1]-p1[1];	pt[2]=p[2]-p1[2];
	proj=pt[0]*vt[0]+pt[1]*vt[1]+pt[2]*vt[2];
	dist=(p2[0]-p1[0])*vt[0]+(p2[1]-p1[1])*vt[1]+(p2[2]-p1[2])*vt[2];
	if (proj<=0.0) return Distance_to_Point(p,p1);
	if (proj>=dist) return Distance_to_Point(p,p2);

	pt[0]=pt[0]-vt[0]*proj;	pt[1]=pt[1]-vt[1]*proj;	pt[2]=pt[2]-vt[2]*proj;
	dist=sqrt(pt[0]*pt[0]+pt[1]*pt[1]+pt[2]*pt[2]);
	return dist;
}

double GLKGeometry::Distance_to_Triangle_Approx(double p[], double p1[], double p2[], double p3[])
{
	double dist,cp[3],temp;

	cp[0]=(p3[0]+p1[0]+p2[0])/3.0;
	cp[1]=(p3[1]+p1[1]+p2[1])/3.0;
	cp[2]=(p3[2]+p1[2]+p2[2])/3.0;
	dist=Distance_to_Point(cp,p);

	cp[0]=(p1[0]+p2[0])/2.0;
	cp[1]=(p1[1]+p2[1])/2.0;
	cp[2]=(p1[2]+p2[2])/2.0;
	temp=Distance_to_Point(cp,p);
	if (temp<dist) {dist=temp;}

	cp[0]=(p1[0]+p3[0])/2.0;
	cp[1]=(p1[1]+p3[1])/2.0;
	cp[2]=(p1[2]+p3[2])/2.0;
	temp=Distance_to_Point(cp,p);
	if (temp<dist) {dist=temp;}

	cp[0]=(p3[0]+p2[0])/2.0;
	cp[1]=(p3[1]+p2[1])/2.0;
	cp[2]=(p3[2]+p2[2])/2.0;
	temp=Distance_to_Point(cp,p);
	if (temp<dist) {dist=temp;}

	temp=Distance_to_Point(p1,p);
	if (temp<dist) {dist=temp;}

	temp=Distance_to_Point(p2,p);
	if (temp<dist) {dist=temp;}

	temp=Distance_to_Point(p3,p);
	if (temp<dist) {dist=temp;}

	return dist;
}

double GLKGeometry::Distance_to_Triangle_Approx(double p[], double p1[], double p2[], double p3[], 
												double closePnt[])
{
	double dist,cp[3],temp;

	cp[0]=(p3[0]+p1[0]+p2[0])/3.0;
	cp[1]=(p3[1]+p1[1]+p2[1])/3.0;
	cp[2]=(p3[2]+p1[2]+p2[2])/3.0;
	dist=Distance_to_Point(cp,p);

	cp[0]=(p1[0]+p2[0])/2.0;
	cp[1]=(p1[1]+p2[1])/2.0;
	cp[2]=(p1[2]+p2[2])/2.0;
	temp=Distance_to_Point(cp,p);
	if (temp<dist) {dist=temp; closePnt[0]=cp[0]; closePnt[1]=cp[1]; closePnt[2]=cp[2];}

	cp[0]=(p1[0]+p3[0])/2.0;
	cp[1]=(p1[1]+p3[1])/2.0;
	cp[2]=(p1[2]+p3[2])/2.0;
	temp=Distance_to_Point(cp,p);
	if (temp<dist) {dist=temp; closePnt[0]=cp[0]; closePnt[1]=cp[1]; closePnt[2]=cp[2];}

	cp[0]=(p3[0]+p2[0])/2.0;
	cp[1]=(p3[1]+p2[1])/2.0;
	cp[2]=(p3[2]+p2[2])/2.0;
	temp=Distance_to_Point(cp,p);
	if (temp<dist) {dist=temp; closePnt[0]=cp[0]; closePnt[1]=cp[1]; closePnt[2]=cp[2];}

	temp=Distance_to_Point(p1,p);
	if (temp<dist) {dist=temp; closePnt[0]=p1[0]; closePnt[1]=p1[1]; closePnt[2]=p1[2];}

	temp=Distance_to_Point(p2,p);
	if (temp<dist) {dist=temp; closePnt[0]=p2[0]; closePnt[1]=p2[1]; closePnt[2]=p2[2];}

	temp=Distance_to_Point(p3,p);
	if (temp<dist) {dist=temp; closePnt[0]=p3[0]; closePnt[1]=p3[1]; closePnt[2]=p3[2];}

	return dist;
}

double GLKGeometry::Distance_to_Triangle(double p[], double p1[], double p2[], double p3[], 
										 double closePnt[])
{
	double dist,temp,ll,proj,normal[3],v[3],v2[3],cp[3],normal2[3];
	bool bFlag=true;

	//-----------------------------------------------------------------------------
	//	distance to face is considerred
	CalPlaneEquation(p1,p2,p3,normal[0],normal[1],normal[2],dist);
	v[0]=p[0]-p1[0];	v[1]=p[1]-p1[1];	v[2]=p[2]-p1[2];
	dist=v[0]*normal[0]+v[1]*normal[1]+v[2]*normal[2];
	cp[0]=p[0]-dist*normal[0];	closePnt[0]=cp[0];
	cp[1]=p[1]-dist*normal[1];	closePnt[1]=cp[1];
	cp[2]=p[2]-dist*normal[2];	closePnt[2]=cp[2];

	v2[0]=p2[0]-p1[0];	v2[1]=p2[1]-p1[1];	v2[2]=p2[2]-p1[2];
	v[0]=cp[0]-p1[0];	v[1]=cp[1]-p1[1];	v[2]=cp[2]-p1[2];
	VectorProduct(v2,v,normal2);
	temp=normal2[0]*normal[0]+normal2[1]*normal[1]+normal2[2]*normal[2];
	if (temp<=0.0) bFlag=false;
	v2[0]=p3[0]-p1[0];	v2[1]=p3[1]-p1[1];	v2[2]=p3[2]-p1[2];
	VectorProduct(v,v2,normal2);
	temp=normal2[0]*normal[0]+normal2[1]*normal[1]+normal2[2]*normal[2];
	if (temp<=0.0) bFlag=false;

	if (bFlag) {
		v2[0]=p1[0]-p3[0];	v2[1]=p1[1]-p3[1];	v2[2]=p1[2]-p3[2];
		v[0]=cp[0]-p3[0];	v[1]=cp[1]-p3[1];	v[2]=cp[2]-p3[2];
		VectorProduct(v2,v,normal2);
		temp=normal2[0]*normal[0]+normal2[1]*normal[1]+normal2[2]*normal[2];
		if (temp<=0.0) bFlag=false;
		v2[0]=p2[0]-p3[0];	v2[1]=p2[1]-p3[1];	v2[2]=p2[2]-p3[2];
		VectorProduct(v,v2,normal2);
		temp=normal2[0]*normal[0]+normal2[1]*normal[1]+normal2[2]*normal[2];
		if (temp<=0.0) bFlag=false;
	}

	if (bFlag) {
		v2[0]=p3[0]-p2[0];	v2[1]=p3[1]-p2[1];	v2[2]=p3[2]-p2[2];
		v[0]=cp[0]-p2[0];	v[1]=cp[1]-p2[1];	v[2]=cp[2]-p2[2];
		VectorProduct(v2,v,normal2);
		temp=normal2[0]*normal[0]+normal2[1]*normal[1]+normal2[2]*normal[2];
		if (temp<=0.0) bFlag=false;
		v2[0]=p1[0]-p2[0];	v2[1]=p1[1]-p2[1];	v2[2]=p1[2]-p2[2];
		VectorProduct(v,v2,normal2);
		temp=normal2[0]*normal[0]+normal2[1]*normal[1]+normal2[2]*normal[2];
		if (temp<=0.0) bFlag=false;
	}

	if (bFlag) {return fabs(dist);}

	//-----------------------------------------------------------------------------
	//	distances to three points are considerred
	dist=Distance_to_Point(p,p1);
	closePnt[0]=p1[0]; closePnt[1]=p1[1]; closePnt[2]=p1[2];
	temp=Distance_to_Point(p,p2); 
	if (temp<dist) {
		dist=temp;	closePnt[0]=p2[0]; closePnt[1]=p2[1]; closePnt[2]=p2[2];
	}
	temp=Distance_to_Point(p,p3); 
	if (temp<dist) {
		dist=temp;	closePnt[0]=p3[0]; closePnt[1]=p3[1]; closePnt[2]=p3[2];
	}

	//-----------------------------------------------------------------------------
	//	distances to three segments are considerred
	ll=Distance_to_Point(p1,p2);
	v[0]=p[0]-p1[0];	v[1]=p[1]-p1[1];	v[2]=p[2]-p1[2];
	v2[0]=p2[0]-p1[0];	v2[1]=p2[1]-p1[1];	v2[2]=p2[2]-p1[2];
	Normalize(v2);
	proj=v[0]*v2[0]+v[1]*v2[1]+v[2]*v2[2];
	if ((proj>0.0) && (proj<ll)) {
		cp[0]=v[0]-proj*v2[0];	cp[1]=v[1]-proj*v2[1];	cp[2]=v[2]-proj*v2[2];
		temp=sqrt(cp[0]*cp[0]+cp[1]*cp[1]+cp[2]*cp[2]);
		if (temp<dist) {
			dist=temp;
			closePnt[0]=p[0]-cp[0];	closePnt[1]=p[1]-cp[1];	closePnt[2]=p[2]-cp[2];
		}
	}

	ll=Distance_to_Point(p2,p3);
	v[0]=p[0]-p2[0];	v[1]=p[1]-p2[1];	v[2]=p[2]-p2[2];
	v2[0]=p3[0]-p2[0];	v2[1]=p3[1]-p2[1];	v2[2]=p3[2]-p2[2];
	Normalize(v2);
	proj=v[0]*v2[0]+v[1]*v2[1]+v[2]*v2[2];
	if ((proj>0.0) && (proj<ll)) {
		cp[0]=v[0]-proj*v2[0];	cp[1]=v[1]-proj*v2[1];	cp[2]=v[2]-proj*v2[2];
		temp=sqrt(cp[0]*cp[0]+cp[1]*cp[1]+cp[2]*cp[2]);
		if (temp<dist) {
			dist=temp;
			closePnt[0]=p[0]-cp[0];	closePnt[1]=p[1]-cp[1];	closePnt[2]=p[2]-cp[2];
		}
	}

	ll=Distance_to_Point(p3,p1);
	v[0]=p[0]-p3[0];	v[1]=p[1]-p3[1];	v[2]=p[2]-p3[2];
	v2[0]=p1[0]-p3[0];	v2[1]=p1[1]-p3[1];	v2[2]=p1[2]-p3[2];
	Normalize(v2);
	proj=v[0]*v2[0]+v[1]*v2[1]+v[2]*v2[2];
	if ((proj>0.0) && (proj<ll)) {
		cp[0]=v[0]-proj*v2[0];	cp[1]=v[1]-proj*v2[1];	cp[2]=v[2]-proj*v2[2];
		temp=sqrt(cp[0]*cp[0]+cp[1]*cp[1]+cp[2]*cp[2]);
		if (temp<dist) {
			dist=temp;
			closePnt[0]=p[0]-cp[0];	closePnt[1]=p[1]-cp[1];	closePnt[2]=p[2]-cp[2];
		}
	}

	return dist;
}

double GLKGeometry::Distance_to_Triangle(double p[], double p1[], double p2[], double p3[])
{
	double dist,temp,ll,proj,normal[3],v[3],v2[3],cp[3],normal2[3];
	bool bFlag=true;

	//-----------------------------------------------------------------------------
	//	distance to face is considerred
	CalPlaneEquation(p1,p2,p3,normal[0],normal[1],normal[2],dist);
	v[0]=p[0]-p1[0];	v[1]=p[1]-p1[1];	v[2]=p[2]-p1[2];
	dist=v[0]*normal[0]+v[1]*normal[1]+v[2]*normal[2];
	cp[0]=v[0]-dist*normal[0]+p1[0];	
	cp[1]=v[1]-dist*normal[1]+p1[1];
	cp[2]=v[2]-dist*normal[2]+p1[2];

	v2[0]=p2[0]-p1[0];	v2[1]=p2[1]-p1[1];	v2[2]=p2[2]-p1[2];
	v[0]=cp[0]-p1[0];	v[1]=cp[1]-p1[1];	v[2]=cp[2]-p1[2];
	VectorProduct(v2,v,normal2);
	temp=normal2[0]*normal[0]+normal2[1]*normal[1]+normal2[2]*normal[2];
	if (temp<=0.0) bFlag=false;
	v2[0]=p3[0]-p1[0];	v2[1]=p3[1]-p1[1];	v2[2]=p3[2]-p1[2];
	VectorProduct(v,v2,normal2);
	temp=normal2[0]*normal[0]+normal2[1]*normal[1]+normal2[2]*normal[2];
	if (temp<=0.0) bFlag=false;

	if (bFlag) {
		v2[0]=p1[0]-p3[0];	v2[1]=p1[1]-p3[1];	v2[2]=p1[2]-p3[2];
		v[0]=cp[0]-p3[0];	v[1]=cp[1]-p3[1];	v[2]=cp[2]-p3[2];
		VectorProduct(v2,v,normal2);
		temp=normal2[0]*normal[0]+normal2[1]*normal[1]+normal2[2]*normal[2];
		if (temp<=0.0) bFlag=false;
		v2[0]=p2[0]-p3[0];	v2[1]=p2[1]-p3[1];	v2[2]=p2[2]-p3[2];
		VectorProduct(v,v2,normal2);
		temp=normal2[0]*normal[0]+normal2[1]*normal[1]+normal2[2]*normal[2];
		if (temp<=0.0) bFlag=false;
	}

	if (bFlag) {
		v2[0]=p3[0]-p2[0];	v2[1]=p3[1]-p2[1];	v2[2]=p3[2]-p2[2];
		v[0]=cp[0]-p2[0];	v[1]=cp[1]-p2[1];	v[2]=cp[2]-p2[2];
		VectorProduct(v2,v,normal2);
		temp=normal2[0]*normal[0]+normal2[1]*normal[1]+normal2[2]*normal[2];
		if (temp<=0.0) bFlag=false;
		v2[0]=p1[0]-p2[0];	v2[1]=p1[1]-p2[1];	v2[2]=p1[2]-p2[2];
		VectorProduct(v,v2,normal2);
		temp=normal2[0]*normal[0]+normal2[1]*normal[1]+normal2[2]*normal[2];
		if (temp<=0.0) bFlag=false;
	}

	if (bFlag) {return fabs(dist);}

	//-----------------------------------------------------------------------------
	//	distances to three points are considerred
	dist=Distance_to_Point(p,p1);
	temp=Distance_to_Point(p,p2); if (temp<dist) dist=temp;
	temp=Distance_to_Point(p,p3); if (temp<dist) dist=temp;

	//-----------------------------------------------------------------------------
	//	distances to three segments are considerred
	ll=Distance_to_Point(p1,p2);
	v[0]=p[0]-p1[0];	v[1]=p[1]-p1[1];	v[2]=p[2]-p1[2];
	v2[0]=p2[0]-p1[0];	v2[1]=p2[1]-p1[1];	v2[2]=p2[2]-p1[2];
	Normalize(v2);
	proj=v[0]*v2[0]+v[1]*v2[1]+v[2]*v2[2];
	if ((proj>0.0) && (proj<ll)) {
		cp[0]=v[0]-proj*v2[0];	cp[1]=v[1]-proj*v2[1];	cp[2]=v[2]-proj*v2[2];
		temp=sqrt(cp[0]*cp[0]+cp[1]*cp[1]+cp[2]*cp[2]);
		if (temp<dist) dist=temp;
	}

	ll=Distance_to_Point(p2,p3);
	v[0]=p[0]-p2[0];	v[1]=p[1]-p2[1];	v[2]=p[2]-p2[2];
	v2[0]=p3[0]-p2[0];	v2[1]=p3[1]-p2[1];	v2[2]=p3[2]-p2[2];
	Normalize(v2);
	proj=v[0]*v2[0]+v[1]*v2[1]+v[2]*v2[2];
	if ((proj>0.0) && (proj<ll)) {
		cp[0]=v[0]-proj*v2[0];	cp[1]=v[1]-proj*v2[1];	cp[2]=v[2]-proj*v2[2];
		temp=sqrt(cp[0]*cp[0]+cp[1]*cp[1]+cp[2]*cp[2]);
		if (temp<dist) dist=temp;
	}

	ll=Distance_to_Point(p3,p1);
	v[0]=p[0]-p3[0];	v[1]=p[1]-p3[1];	v[2]=p[2]-p3[2];
	v2[0]=p1[0]-p3[0];	v2[1]=p1[1]-p3[1];	v2[2]=p1[2]-p3[2];
	Normalize(v2);
	proj=v[0]*v2[0]+v[1]*v2[1]+v[2]*v2[2];
	if ((proj>0.0) && (proj<ll)) {
		cp[0]=v[0]-proj*v2[0];	cp[1]=v[1]-proj*v2[1];	cp[2]=v[2]-proj*v2[2];
		temp=sqrt(cp[0]*cp[0]+cp[1]*cp[1]+cp[2]*cp[2]);
		if (temp<dist) dist=temp;
	}

	return dist;
}

void GLKGeometry::SpatialPolygonCenter(double *xp, double *yp, double *zp, int pntNum, 
									   double centerPos[])
{
	centerPos[0]=0.0;	centerPos[1]=0.0;	centerPos[2]=0.0;
	for(int i=0;i<pntNum;i++) {
		centerPos[0]+=xp[i]; centerPos[1]+=yp[i]; centerPos[2]+=zp[i];
	}
	centerPos[0]=centerPos[0]/(double)pntNum;
	centerPos[1]=centerPos[1]/(double)pntNum;
	centerPos[2]=centerPos[2]/(double)pntNum;
}

double GLKGeometry::PolygonArea(double *xp, double *yp, int pntNum)
{
    double p0[2], p1[2], p2[2];
    double area = 0.0;
    p0[0]=xp[0]; p0[1]=yp[0];
    for (int i=0; i<pntNum-3; i++){
        p1[0]=xp[i+1]; p1[1]=yp[i+1];
        p2[0]=xp[i+2]; p2[1]=yp[i+2];
        area += 0.5*fabs((p1[0]-p0[0])*(p2[1]-p0[1])-(p2[0]-p0[0])*(p1[1]-p0[1]));
    }
    return area;
}

double GLKGeometry::SpatialPolygonArea(double *xp, double *yp, double *zp, int pntNum)
{
	double p0[3], p1[3], p2[3];
	double area;

	area=0.0;	
	p0[0]=xp[0]; p0[1]=yp[0]; p0[2]=zp[0];
	for(int i=0;i<(pntNum-3);i++) {
		p1[0]=xp[i+1];	p1[1]=yp[i+1];	p1[2]=zp[i+1];
		p2[0]=xp[i+2];	p2[1]=yp[i+2];	p2[2]=zp[i+2];

		area+=SpatialTriangleArea(p0, p1, p2);
	}

	return area;
}

double GLKGeometry::SpatialTriangleArea(double p0[], double p1[], double p2[])
{
	double x1,y1,z1,x2,y2,z2;
	double ii,jj,kk;
	double area;

	x1=p1[0]-p0[0];	y1=p1[1]-p0[1];	z1=p1[2]-p0[2];
	x2=p2[0]-p0[0];	y2=p2[1]-p0[1];	z2=p2[2]-p0[2];

	ii=y1*z2-z1*y2;
	jj=x2*z1-x1*z2;
	kk=x1*y2-x2*y1;

	area=sqrt(ii*ii+jj*jj+kk*kk)/2.0;

	return area;
}

void GLKGeometry::DiscretizationByLength(double* &x, double* &y, double* &z, int n, 
								 double Len, int &m)
{
    int *nIndex;

    nIndex=new int[n];
    m=0;

    int startNo,endNo;
    for(int j=0;j<n;j++)
    {
        if ((j==0) || (j==(n-1)))
        {
            nIndex[m++]=j;
            startNo=j;
            continue;
        }
        endNo=j;

        double point1[3],point2[3];
        point1[0]=x[startNo];	point1[1]=y[startNo];	point1[2]=z[startNo];
        point2[0]=x[endNo];		point2[1]=y[endNo];		point2[2]=z[endNo];

        double distance=Distance_to_Point(point1,point2);

        if (distance>Len)
        {
            nIndex[m++]=endNo;
            startNo=endNo;
        }
    }

    for(int j=0;j<m;j++)
    {
        x[j]=x[nIndex[j]];	y[j]=y[nIndex[j]];	z[j]=z[nIndex[j]];
    }

    delete []nIndex;

    GLPositionArray  posArray;
    posArray.Add(x[0],y[0],z[0]);
    for(int j=1;j<m;j++)
    {
        double p1[3],p2[3];
        p1[0]=x[j-1];	p1[1]=y[j-1];	p1[2]=z[j-1];
        p2[0]=x[j];		p2[1]=y[j];		p2[2]=z[j];
        double l=Distance_to_Point(p1,p2);
        if (l<=Len)
        {
            posArray.Add(x[j],y[j],z[j]);
        }
        else
        {
            double d[3];
            int num=(int)(l/Len+1);
            for(int i=0;i<3;i++) d[i]=(p2[i]-p1[i])/((double)num);
            for(int i=1;i<=num;i++)
                posArray.Add(p1[0]+d[0]*((double)i),
                             p1[1]+d[1]*((double)i),
                             p1[2]+d[2]*((double)i));
        }
    }

    delete []x;	delete []y;	delete []z;
    m=posArray.GetSize();
    x=new double[m];	y=new double[m];	z=new double[m];
    for(int j=0;j<m;j++) posArray.ElementAt(j,x[j],y[j],z[j]);
}

void GLKGeometry::DiscretizationByChordal(double *x, double *y, double *z, int n, 
								  double Chordal, int &m)
{
	int *nIndex;
	int j;

	nIndex=new int[n];
	m=0;

	int startNo,endNo;
	for(j=0;j<n;j++)
	{
		if ((j==0) || (j==(n-1)))
		{
			nIndex[m++]=j;
			startNo=j;
			continue;
		}
		endNo=j;

		double point1[3],point2[3];
		point1[0]=x[startNo];	point1[1]=y[startNo];	point1[2]=z[startNo];
		point2[0]=x[endNo];		point2[1]=y[endNo];		point2[2]=z[endNo];

		for(int k=startNo+1;k<endNo;k++)
		{
			double point[3];
			point[0]=x[k];	point[1]=y[k];	point[2]=z[k];

			double distance=Distance_to_LineSegment(point,point1,point2);;
			if (distance>Chordal)
			{
				nIndex[m++]=endNo-1;
				startNo=endNo-1;
				break;
			}
		}
	}

	for(j=0;j<m;j++)
	{
		x[j]=x[nIndex[j]];	y[j]=y[nIndex[j]];	z[j]=z[nIndex[j]];
	}

	delete []nIndex;
}

bool GLKGeometry::EdgeFlipDetection(double p1[], double p2[], double p3[], double p4[])
{
	double e[6],e2[6],a,minA[2];

	e[0]=Distance_to_Point(p1,p2);
	e[1]=Distance_to_Point(p2,p3);
	e[2]=Distance_to_Point(p3,p4);
	e[3]=Distance_to_Point(p4,p1);
	e[4]=Distance_to_Point(p1,p3);
	e[5]=Distance_to_Point(p2,p4);

	for(int i=0;i<6;i++) e2[i]=e[i]*e[i];

	a=acos((e2[0]+e2[1]-e2[4])/(2.0*e[0]*e[1]));
	minA[0]=a;
	a=acos((e2[1]+e2[4]-e2[0])/(2.0*e[1]*e[4]));
	if (a<minA[0]) minA[0]=a;
	a=acos((e2[0]+e2[4]-e2[1])/(2.0*e[0]*e[4]));
	if (a<minA[0]) minA[0]=a;
	a=acos((e2[3]+e2[4]-e2[2])/(2.0*e[3]*e[4]));
	if (a<minA[0]) minA[0]=a;
	a=acos((e2[2]+e2[4]-e2[3])/(2.0*e[2]*e[4]));
	if (a<minA[0]) minA[0]=a;
	a=acos((e2[2]+e2[3]-e2[4])/(2.0*e[2]*e[3]));
	if (a<minA[0]) minA[0]=a;

	a=acos((e2[0]+e2[5]-e2[3])/(2.0*e[0]*e[5]));
	minA[1]=a;
	a=acos((e2[5]+e2[3]-e2[0])/(2.0*e[5]*e[3]));
	if (a<minA[1]) minA[1]=a;
	a=acos((e2[0]+e2[3]-e2[5])/(2.0*e[0]*e[3]));
	if (a<minA[1]) minA[1]=a;
	a=acos((e2[1]+e2[5]-e2[2])/(2.0*e[1]*e[5]));
	if (a<minA[1]) minA[1]=a;
	a=acos((e2[1]+e2[2]-e2[5])/(2.0*e[1]*e[2]));
	if (a<minA[1]) minA[1]=a;
	a=acos((e2[2]+e2[5]-e2[1])/(2.0*e[2]*e[5]));
	if (a<minA[1]) minA[1]=a;
	
	if (minA[0]<minA[1]) return false;

	return true;
}

void GLKGeometry::Get3rdPointCoord(double x1,double y1,double r1,double x2,double y2,double r2,double &x3,double &y3)
{
	long double r0=sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
	long double cs,sn,nx,ny,mx,my,temp;

	cs=(r1*r1+r0*r0-r2*r2)/(2.0*r1*r0);
	temp=1.0-cs*cs;

	if ((fabs(cs)>1.0) || (fabs(cs)<0.0))
	{
		cs=1.0;
		temp=0.0;
		r1=r0/2.0;
	}

	sn=sqrt(temp);
	nx=(x2-x1)/r0;	ny=(y2-y1)/r0;
	mx=nx*cs-ny*sn;	my=nx*sn+ny*cs;
	x3=mx*r1+x1;	y3=my*r1+y1;
}

bool GLKGeometry::JugTwoLineSegmentsIntersectOrNot(double x1, double y1, double x2, double y2,
												   double x3, double y3, double x4, double y4)
{
	if ((x1==x3) && (y1==y3)) return false;
	if ((x1==x4) && (y1==y4)) return false;
	if ((x2==x3) && (y2==y3)) return false;
	if ((x2==x4) && (y2==y4)) return false;

	double ua1,ua2,ua,ub1,ub2,ub;
	ua1=(x4-x3)*(y1-y3)-(y4-y3)*(x1-x3);
	ua2=(y4-y3)*(x2-x1)-(x4-x3)*(y2-y1);
	ub1=(x2-x1)*(y1-y3)-(y2-y1)*(x1-x3);
	ub2=ua2;

	if (fabs(ua2)<EPS) return false;
	if (fabs(ub2)<EPS) return false;
	ua=ua1/ua2;
	ub=ub1/ub2;
	if ((ua<0.0) || (ua>1.0)) return false;
	if ((ub<0.0) || (ub>1.0)) return false;

	return true;
}

bool GLKGeometry::JugClockwiseOrNot(int pNum, double xp[], double yp[])
{
	double area=0.0;

	for(int i=1;i<pNum;i++)	area+=(xp[i-1]-xp[i])*(yp[i-1]+yp[i]);
	area+=(xp[pNum-1]-xp[0])*(yp[pNum-1]+yp[0]);

	if (area<0.0) return true;

	return false;
}

bool GLKGeometry::JugPointInsideOrNot(int pNum, double xp[], double yp[], double x, double y)
{
	int i, j;
	bool c=false;

	j=pNum-1;
	for(i=0;i<pNum;j=i++)
	{
		if ((((yp[i]<=y) && (y<yp[j])) ||
			((yp[j]<=y) && (y<yp[i]))) &&
			(x<(xp[j]-xp[i])*(y-yp[i])/(yp[j]-yp[i])+xp[i]))
			c=!c;
	}

	return c;
}

bool GLKGeometry::CalTwoLinesIntersection(double a1, double b1, double c1,
										   double a2, double b2, double c2,
										   double &xx, double &yy)
{
	double d=a1*b2-a2*b1;

	if (fabs(d)<EPS) return false;

	xx=-(c1*b2-c2*b1)/d;
	yy=(c1*a2-c2*a1)/d;

	return true;
}

bool GLKGeometry::CalTwoLineSegmentsIntersection(double x1, double y1, double x2, double y2,
										   double x3, double y3, double x4, double y4,
										   double &xx, double &yy)
{
	double a1,b1,c1,a2,b2,c2;

	CalLineEquation(a1,b1,c1,x1,y1,x2,y2);
	CalLineEquation(a2,b2,c2,x3,y3,x4,y4);
	
	if (!(CalTwoLinesIntersection(a1,b1,c1,a2,b2,c2,xx,yy))) return false;

	double u1;
	if (x3==x4)
		u1=(yy-y3)/(y4-y3);
	else
		u1=(xx-x3)/(x4-x3);

	double u2;
	if (x1==x2)
		u2=(yy-y1)/(y2-y1);
	else
		u2=(xx-x1)/(x2-x1);

	if ((u1>=0.0) && (u1<=1.0) && (u2>=0.0) && (u2<=1.0)) return true;

	return false;
}

double GLKGeometry::CalAngle(double p1[], double p[], double p2[])
{
	double angle;
	double a=Distance_to_Point(p,p1);
	double b=Distance_to_Point(p,p2);
	double c=Distance_to_Point(p1,p2);
	angle=acos((a*a+b*b-c*c)/(2.0*a*b));
		
	return ROTATE_TO_DEGREE(angle);
}

bool GLKGeometry::CalSphereEquation(double x[], double y[], double z[],
						   double& x0, double& y0, double& z0, double& R )
{
	double a[16];
	double t1,t2,t3,t4;
	double M11,M12,M13,M14,M15;

	t1=x[0]*x[0]+y[0]*y[0]+z[0]*z[0];	t2=x[1]*x[1]+y[1]*y[1]+z[1]*z[1];
	t3=x[2]*x[2]+y[2]*y[2]+z[2]*z[2];	t4=x[3]*x[3]+y[3]*y[3]+z[0]*z[3];

	a[0]=x[0];	a[1]=y[0];	a[2]=z[0];	a[3]=1.0;
	a[4]=x[1];	a[5]=y[1];	a[6]=z[1];	a[7]=1.0;
	a[8]=x[2];	a[9]=y[2];	a[10]=z[2];	a[11]=1.0;
	a[12]=x[3];	a[13]=y[3];	a[14]=z[3];	a[15]=1.0;
	M11=Determinant4(a);

	a[0]=t1;	a[1]=y[0];	a[2]=z[0];	a[3]=1.0;
	a[4]=t2;	a[5]=y[1];	a[6]=z[1];	a[7]=1.0;
	a[8]=t3;	a[9]=y[2];	a[10]=z[2];	a[11]=1.0;
	a[12]=t4;	a[13]=y[3];	a[14]=z[3];	a[15]=1.0;
	M12=Determinant4(a);

	a[0]=t1;	a[1]=x[0];	a[2]=z[0];	a[3]=1.0;
	a[4]=t2;	a[5]=x[1];	a[6]=z[1];	a[7]=1.0;
	a[8]=t3;	a[9]=x[2];	a[10]=z[2];	a[11]=1.0;
	a[12]=t4;	a[13]=x[3];	a[14]=z[3];	a[15]=1.0;
	M13=Determinant4(a);

	a[0]=t1;	a[1]=x[0];	a[2]=y[0];	a[3]=1.0;
	a[4]=t2;	a[5]=x[1];	a[6]=y[1];	a[7]=1.0;
	a[8]=t3;	a[9]=x[2];	a[10]=y[2];	a[11]=1.0;
	a[12]=t4;	a[13]=x[3];	a[14]=y[3];	a[15]=1.0;
	M14=Determinant4(a);

	a[0]=t1;	a[1]=x[0];	a[2]=y[0];	a[3]=z[0];
	a[4]=t2;	a[5]=x[1];	a[6]=y[1];	a[7]=z[1];
	a[8]=t3;	a[9]=x[2];	a[10]=y[2];	a[11]=z[2];
	a[12]=t4;	a[13]=x[3];	a[14]=y[3];	a[15]=z[3];
	M15=Determinant4(a);

	if (M11<EPS) return false;

	x0=M12/(2.0*M11);
	y0=-M13/(2.0*M11);
	z0=M14/(2.0*M11);
	R=sqrt((M12*M12+M13*M13+M14*M14-4.0*M15*M11)/(4.0*M11*M11));

	return true;
}

double GLKGeometry::Determinant3(double a[])
{
	double r;
	
	r=a[0]*a[4]*a[8]-a[0]*a[5]*a[7]
		+a[1]*a[5]*a[6]-a[1]*a[3]*a[8]
		+a[2]*a[3]*a[7]-a[2]*a[4]*a[6];

	return r;
}

double GLKGeometry::Determinant4(double a[])
{
	double r;
	double b[9];

	b[0]=a[5];	b[1]=a[6];	b[2]=a[7];
	b[3]=a[9];	b[4]=a[10];	b[5]=a[11];
	b[6]=a[13];	b[7]=a[14];	b[8]=a[15];
	r=a[0]*Determinant3(b);

	b[0]=a[4];	b[1]=a[6];	b[2]=a[7];
	b[3]=a[8];	b[4]=a[10];	b[5]=a[11];
	b[6]=a[12];	b[7]=a[14];	b[8]=a[15];
	r=r-a[1]*Determinant3(b);

	b[0]=a[4];	b[1]=a[5];	b[2]=a[7];
	b[3]=a[8];	b[4]=a[9];	b[5]=a[11];
	b[6]=a[12];	b[7]=a[13];	b[8]=a[15];
	r=r+a[2]*Determinant3(b);

	b[0]=a[4];	b[1]=a[5];	b[2]=a[6];
	b[3]=a[8];	b[4]=a[9];	b[5]=a[10];
	b[6]=a[12];	b[7]=a[13];	b[8]=a[14];
	r=r-a[3]*Determinant3(b);

	return r;
}

void GLKGeometry::VectorProduct(double n1[], double n2[], double n3[])
{
	n3[0]=n1[1]*n2[2]-n1[2]*n2[1];
	n3[1]=n1[2]*n2[0]-n1[0]*n2[2];
	n3[2]=n1[0]*n2[1]-n1[1]*n2[0];
}

double GLKGeometry::VectorProject(double n1[], double n2[])
{
	double r=n1[0]*n2[0]+n1[1]*n2[1]+n1[2]*n2[2];

	return r;
}

double GLKGeometry::VectorProject(double n1[], double n2[], const int n)
{
    double r = 0.0;
    for(int i=0;i<n;i++) r += n1[i]*n2[i];
    return r;
}

double GLKGeometry::TripleProduct(double va[], double vb[], double vc[])
{
	return (va[0]*vb[1]*vc[2]+va[1]*vb[2]*vc[0]+va[2]*vb[0]*vc[1]
		-va[0]*vb[2]*vc[1]-va[1]*vb[0]*vc[2]-va[2]*vb[1]*vc[0]);
}

void GLKGeometry::CoordinateTransf(double xA[], double yA[], double zA[], double p[], 
						  double &xx, double &yy, double &zz)
{
	xx=VectorProject(p,xA);
	yy=VectorProject(p,yA);
	zz=VectorProject(p,zA);
}

void GLKGeometry::InverseCoordinateTransf(double xA[], double yA[], double zA[], 
										   double p[], double &xx, double &yy, double &zz)
{
	xx=p[0]*xA[0]+p[1]*yA[0]+p[2]*zA[0];
	yy=p[0]*xA[1]+p[1]*yA[1]+p[2]*zA[1];
	zz=p[0]*xA[2]+p[1]*yA[2]+p[2]*zA[2];
}

void GLKGeometry::RotatePointAlongX(double px, double py, double pz, double angle, 
				double &px1, double &py1, double &pz1)
{
	double a=DEGREE_TO_ROTATE(angle);
	double ca=cos(a),sa=sin(a);
	px1=px;
	py1=py*ca-pz*sa;
	pz1=py*sa+pz*ca;
}

void GLKGeometry::RotatePointAlongY(double px, double py, double pz, double angle, 
				double &px1, double &py1, double &pz1)
{
	double a=DEGREE_TO_ROTATE(angle);
	double ca=cos(a),sa=sin(a);
	px1=pz*sa+px*ca;
	py1=py;
	pz1=pz*ca-px*sa;
}

void GLKGeometry::RotatePointAlongZ(double px, double py, double pz, double angle, 
				double &px1, double &py1, double &pz1)
{
	double a=DEGREE_TO_ROTATE(angle);
	double ca=cos(a),sa=sin(a);
	px1=px*ca-py*sa;
	py1=px*sa+py*ca;
	pz1=pz;
}

void GLKGeometry::RotatePointAlongVector(double px, double py, double pz, 
				double x1, double y1, double z1, double x2, double y2, double z2,
				double angle, double &px1, double &py1, double &pz1)
{
	double rx,ry,rz,rrrr;	double costheta,sintheta;

	angle=DEGREE_TO_ROTATE(angle);
	costheta=cos(angle);	sintheta=sin(angle);
	px1=0.0;	py1=0.0;	pz1=0.0;	px=px-x1;	py=py-y1;	pz=pz-z1;
	rx=x2-x1;	ry=y2-y1;	rz=z2-z1;	rrrr=sqrt(rx*rx+ry*ry+rz*rz);
	rx=rx/rrrr;	ry=ry/rrrr;	rz=rz/rrrr;

	px1 += (costheta + (1 - costheta) * rx * rx) * px;
	px1 += ((1 - costheta) * rx * ry - rz * sintheta) * py;
	px1 += ((1 - costheta) * rx * rz + ry * sintheta) * pz;

	py1 += ((1 - costheta) * rx * ry + rz * sintheta) * px;
	py1 += (costheta + (1 - costheta) * ry * ry) * py;
	py1 += ((1 - costheta) * ry * rz - rx * sintheta) * pz;

	pz1 += ((1 - costheta) * rx * rz - ry * sintheta) * px;
	pz1 += ((1 - costheta) * ry * rz + rx * sintheta) * py;
	pz1 += (costheta + (1 - costheta) * rz * rz) * pz;

	px1 += x1;	py1 += y1;	pz1 += z1;
}

void GLKGeometry::QuickSort(int pArr[], int d, int h, bool bAscending)
{
	int i,j;
	int str;

	i = h;
	j = d;

	str = pArr[((int) ((d+h) / 2))];

	do {
		if (bAscending) {
			while (pArr[j] < str) j++;
			while (pArr[i] > str) i--;
		} else {
			while (pArr[j] > str) j++;
			while (pArr[i] < str) i--;
		}
		if ( i >= j ) {
			if ( i != j ) {
				int zal;

				zal = pArr[i];
				pArr[i] = pArr[j];
				pArr[j] = zal;
			}
			i--;
			j++;
		}
	} while (j <= i);

	if (d < i) QuickSort(pArr,d,i,bAscending);
	if (j < h) QuickSort(pArr,j,h,bAscending);
}

void GLKGeometry::ConvexHull2D(Point2D *points, int n, std::vector<Point2D> &hull)
{
    // There must be at least 3 points
    if (n < 3) return;
    hull.clear();
    // Find the leftmost point
    int l = 0;
    for (int i = 1; i < n; i++)
        if (points[i].x <= points[l].x)
            l = i;

    // Start from leftmost point, keep moving counterclockwise
    // until reach the start point again.  This loop runs O(h)
    // times where h is number of points in result or output.
    int p = l, q;
    int number = 0;
    do
    {
        // Add current point to result
        hull.push_back(points[p]);

        // Search for a point 'q' such that orientation(p, x,
        // q) is counterclockwise for all points 'x'. The idea
        // is to keep track of last visited most counterclock-
        // wise point in q. If any point 'i' is more counterclock-
        // wise than q, then update q.
        q = (p+1)%n;
        for (int i = 0; i < n; i++)
        {
            // If i is more counterclockwise than current q, then
            // update q
            if (orientation(points[p], points[i], points[q]) == 2)
                q = i;
        }

        // Now q is the most counterclockwise with respect to p
        // Set p as q for next iteration, so that q is added to
        // result 'hull'
        p = q;
        number++;
        if (number > n)
            break;

    } while (p != l);  // While we don't come to first point
}
