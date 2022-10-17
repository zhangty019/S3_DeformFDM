// GLKGeometry.h: interface for the GLKGeometry class.
//
//////////////////////////////////////////////////////////////////////
#include <vector>
#include "GLKLib.h"

#ifndef _GLKGEOMETRY
#define _GLKGEOMETRY

#define EPS		1.0e-8
#define PI		3.141592654
#define DEGREE_TO_ROTATE(x)		(0.0174532922222*x)
#define ROTATE_TO_DEGREE(x)		(57.295780490443*x)
#define MIN(a,b)	(((a)<(b))?(a):(b))
#define MAX(a,b)	(((a)>(b))?(a):(b))

//////////////////////////////////////////////////////////////////////
//	This class defines all geometry calculation functions needed

struct Point2D{
    double x,y;
};

class GLKGeometry  
{
public:

	//////////////////////////////////////////////////////////////////////
	// To clip a convex polygon by half-space
	//	the polygon is defined in xp[],yp[],zp[] (with pntNum define the number of vertices
	//	the half-space is defined by a point "planePnt[]" and an unit direction vector "planeNormal[]"
	void ClipPolygonByHalfSpace(double* &xp, double* &yp, double* &zp, int &pntNum,
									double planePnt[], double planeNormal[]);
	void ClipPolygonByCube(double* &xp, double* &yp, double* &zp, int &pntNum,
									double xmin, double ymin, double zmin, double boxSize);


	//////////////////////////////////////////////////////////////////////
	// To transfer the point from wcl coordinate (p[0], p[1], p[2])
	//		to the given coordinate system with:
	//			X axis vector - (xA[0], xA[1], xA[2])	
	//			Y axis vector - (yA[0], yA[1], yA[2])
	//			Z axis vector - (zA[0], zA[1], zA[2])
	//			( they are unit vector )
	// Return value:
	//		new coordinate (xx, yy, zz)
	void CoordinateTransf(double xA[], double yA[], double zA[], double p[], 
						  double &xx, double &yy, double &zz);

	//////////////////////////////////////////////////////////////////////
	// To transfer the point from coordinate (p[0], p[1], p[2])
	//		in the given coordinate system with:
	//			X axis vector - (xA[0], xA[1], xA[2])	
	//			Y axis vector - (yA[0], yA[1], yA[2])
	//			Z axis vector - (zA[0], zA[1], zA[2])
	//			( they are unit vector )
	//		back to wcl coordinate system
	// Return value:
	//		wcl coordinate (xx, yy, zz)
	void InverseCoordinateTransf(double xA[], double yA[], double zA[], double p[], 
						  double &xx, double &yy, double &zz);

	//////////////////////////////////////////////////////////////////////
	// To normalize the vector (n[0],n[1],n[2])
	// Return value:
	//		true	--	Has been normalized.
	//		false	--	Length of the vector is zero
    bool Normalize(double n[]);

	//////////////////////////////////////////////////////////////////////
	// To calculate plane equation parameter by three points
	// Plane equation:  Ax + By + Cz + D = 0, and
	// Vector(A,B,C) is positive unit normal vector of this trangle plane
	// Three points (x[0],y[0],z[0]), (x[1],y[1],z[1]) & (x[2],y[2],z[2])
	//		are in anti-clockwise direction
	// Return value:
	//		true	--	3 points are not on the same line
	//		false	--	3 points are on the same line
	bool CalPlaneEquation( double & A, double & B, double & C, double & D, 
		double x[], double y[], double z[]);

	//////////////////////////////////////////////////////////////////////
	// To calculate plane equation parameter by three points
	// Plane equation:  Ax + By + Cz + D = 0, and
	// Vector(A,B,C) is positive unit normal vector of this trangle plane
	// Three points: p0, p1, p2
	//		are in anti-clockwise direction
	// Return value:
	//		true	--	3 points are not on the same line
	//		false	--	3 points are on the same line
	bool CalPlaneEquation( double p0[], double p1[], double p2[],
		double & A, double & B, double & C, double & D );
	bool CalPlaneEquation( float p0[], float p1[], float p2[],
		double & A, double & B, double & C, double & D );

	//////////////////////////////////////////////////////////////////////
	// To approximate plane equation parameter by points ( p[][0], p[][1], p[][2])
	// Plane equation:  Ax + By + Cz + D = 0, and
	// Points number is: n, index from 0 to n-1
	//	
	// Return value:
	//		true	--	has solution
	//		false	--	no solution
	//		The (A,B,C) has been normalized
	bool ApproximatePlaneEquation( int n, double** p,
		double & A, double & B, double & C, double & D );
	bool ApproximatePlaneEquation( int n, float** p,
		double & A, double & B, double & C, double & D );

	//////////////////////////////////////////////////////////////////////
	// To calculate line equation parameter by two points
	// Line equation:  Ax + By + C = 0 
	// Two points (x1,y1) & (x2,y2)
	//
	void CalLineEquation( double & A, double & B, double & C, double x1, double y1, double x2, double y2);

	//////////////////////////////////////////////////////////////////////
	// Line intersection test
	//
	// Line equation: a1 X + b1 Y + c1 = 0   &   a2 X + b2 Y + c2 = 0
	// Intersection point: (xx,yy)
	//
	// Return Value:
	//		true	- Have Intersection
	//		false	- No Intersection
	bool CalTwoLinesIntersection(double a1, double b1, double c1,
								 double a2, double b2, double c2,
								 double &xx, double &yy);

	//////////////////////////////////////////////////////////////////////
	// Line segment intersection test
	//
	// Line segment: (x1,y1)-(x2,y2)  &  (x3,y3)-(x4,y4)
	// Intersection point: (xx,yy)
	//
	// Return Value:
	//		true	- Have Intersection
	//		false	- No Intersection
	bool CalTwoLineSegmentsIntersection(double x1, double y1, double x2, double y2,
			double x3, double y3, double x4, double y4, double &xx, double &yy);

	//////////////////////////////////////////////////////////////////////
	// To calculate plane equation parameter by two points and one vector:
	//		Two points: (p1[0],p1[1],p1[2]) & (p2[0],p2[1],p2[2])
	//		Vector: (l,m,n)
	//		Plane equation:  Ax + By + Cz + D = 0
	bool CalPlaneEquation( double & A, double & B, double & C, double & D,
		double p1[], double p2[], double l, double m, double n);

	//////////////////////////////////////////////////////////////////////
	// To calculate the intersection point of a line and a facet
	// Facet:	(x[0],y[0],z[0]), (x[1],y[1],z[1]) & (x[2],y[2],z[2]) 
	//		in anti-clockwise direction with plane equation:  Ax + By + Cz + D = 0
	// Line:	Point=(p[0],p[1],p[2]) & Direction=(n[0],n[1],n[2])
	//
	// Return value:
	//		true	--	Has an intersection point (p[0]+mu*n[0],p[1]+mu*n[1],p[2]+mu*n[2])
	//		false	--	Has no intersection point
	bool CalLineFacetIntersection( double p[], double n[], double &mu,
		double x[], double y[], double z[],
		double A, double B, double C, double D);
    bool CalLineFacetIntersection( double p[], double n[],
            double v0[], double v1[], double v2[],
            double& t, double& u, double& v);

	//////////////////////////////////////////////////////////////////////
	// To calculate the intersection point of a line and a plane
	// Plane equation:  Ax + By + Cz + D = 0
	// Line:	Points (p1[0],p1[1],p1[2]) & Direction (n[0],n[1],n[2])
	//
	// Return value:
	//		true	--	Has an intersection point : 
	//									(p[0]+mu*n[0],p[1]+mu*n[1],p[2]+mu*n[2])
	//		false	--	Has no intersection point
	bool CalPlaneLineIntersection( double p[], double n[],
		double A, double B, double C, double D, double &mu);

	//////////////////////////////////////////////////////////////////////
	// To calculate the intersection point of a linesegment and a plane
	// Plane equation:  Ax + By + Cz + D = 0
	// Line Segment:	Points (p1[0],p1[1],p1[2]) & (p1[0],p1[1],p1[2])
	//
	// Return value:
	//		true	--	Has an intersection point : (p[0],p[1],p[2])
	//							p[0]=p1[0]+mu*(p2[0]-p1[0]);
	//							p[1]=p1[1]+mu*(p2[1]-p1[1]);
	//							p[2]=p1[2]+mu*(p2[2]-p1[2]);
	//		false	--	Has no intersection point
	bool CalPlaneLineSegIntersection( double p1[], double p2[],
		double A, double B, double C, double D, double &mu);

	//////////////////////////////////////////////////////////////////////
	// To calculate the Areal Coordinate of point pp[] in triangle
	//			(p1[],p2[],p3[])
	//
	// Return value:
	//		areal coordinate ( u, v, w )
	void CalArealCoordinate(double p1[], double p2[], double p3[], double pp[], 
		double &u, double &v, double &w);

	//////////////////////////////////////////////////////////////////////
	// To calculate distance between two points:
	//			(p1[0],p1[1],p1[2]) and (p2[0],p2[1],p2[2])
	//
	// Return value:
	//		The distance value
	double Distance_to_Point(double p1[], double p2[]);
	float Distance_to_Point(float p1[], float p2[]);

	//////////////////////////////////////////////////////////////////////
	// To calculate distance between Point (p[0],p[1],p[2])
	//			and Line (p1[0],p1[1],p1[2])-(p2[0],p2[1],p2[2])
	//
	// Return value:
	//		The distance value
	double Distance_to_LineSegment(double p[], double p1[], double p2[]);

	//////////////////////////////////////////////////////////////////////
	// To calculate distance between Point (p[0],p[1],p[2])
	//			and Triangle (p1[0],p1[1],p1[2]),(p2[0],p2[1],p2[2]),(p3[0],p3[1],p3[2])
	//
	// Return value:
	//		The distance value
	double Distance_to_Triangle(double p[], double p1[], double p2[], double p3[]);
	double Distance_to_Triangle(double p[], double p1[], double p2[], double p3[], double closePnt[]);
	double Distance_to_Triangle_Approx(double p[], double p1[], double p2[], double p3[]);
	double Distance_to_Triangle_Approx(double p[], double p1[], double p2[], double p3[], double closePnt[]);

	//////////////////////////////////////////////////////////////////////
	// To calculate area of triangle:
	//			(p0[0],p0[1],p0[2]), (p1[0],p1[1],p1[2]) & (p2[0],p2[1],p2[2])
	//
	// Return value:
	//		The area value,
	double SpatialTriangleArea(double p0[], double p1[], double p2[]);
	double SpatialPolygonArea(double *xp, double *yp, double *zp, int pntNum);
	void SpatialPolygonCenter(double *xp, double *yp, double *zp, int pntNum, double centerPos[]);

    double PolygonArea(double *xp, double *yp, int pntNum);

	//////////////////////////////////////////////////////////////////////
	// To do discretization of polyline:
	//			(x[0],y[0],z[0]), (x[0],y[0],z[0]) ... (x[n-1],y[n-1],z[n-1])
	//	with the Chordal Thickness = Chordal
	//
	// Return value:
	//		Polyline,
	//			(x[0],y[0],z[0]), (x[0],y[0],z[0]) ... (x[m-1],y[m-1],z[m-1])
	void DiscretizationByChordal(double *x, double *y, double *z, int n, 
								 double Chordal, int &m);

	//////////////////////////////////////////////////////////////////////
	// To do discretization of polyline:
	//			(x[0],y[0],z[0]), (x[0],y[0],z[0]) ... (x[n-1],y[n-1],z[n-1])
	//	with the edge length = Len
	//
	// Return value:
	//		Polyline,
	//			(x[0],y[0],z[0]), (x[0],y[0],z[0]) ... (x[m-1],y[m-1],z[m-1])
	void DiscretizationByLength(double* &x, double* &y, double* &z, int n, 
								 double Len, int &m);

	//////////////////////////////////////////////////////////////////////
	// To do edge flip by Thales's Theorem:
	//		Input four points: p1[],p2[],p3[],p4[]
	//
	//					P1------------p4
	//					 |				\
	//					 |				  \
	//					 |					\
	//					 |					  \
	//					p2---------------------p3
	//
	//		To decide whether should connect "p1[]-p3[]" or "p2[]-p4[]"
	//
	// Return value:
	//		true	- Should connect "p1[]-p3[]"
	//		false	- Should connect "p2[]-p4[]"
	bool EdgeFlipDetection(double p1[], double p2[], double p3[], double p4[]);

	//////////////////////////////////////////////////////////////////////
	// To calculate the 3rd point by points P1, P2 and radius R1, R2 as following:
	//
	//								P2
	//							   /|
	//						  R2 /	|
	//						   /	^
	//						 /  	|
	//					   P3-------P1
	//							R1
	//
	//		P3 should lie on the right side of edge "P1-P2", 
	//								the edge is pointing from P1 to P2 
	//
	void Get3rdPointCoord(double x1,double y1,double r1,double x2,double y2,double r2,double &x3,double &y3);

	//////////////////////////////////////////////////////////////////////
	// Inside/outside polygon test
	//
	// Polygon: xp[], yp[]	(the first point and the last point must be the same point)
	// Polygon point Number: pNum
	// Jug point: (x,y)
	//
	// Return Value:
	//		true	- Inside Polygon
	//		false	- Outside Polygon
	bool JugPointInsideOrNot(int pNum, double xp[], double yp[], double x, double y);

	//////////////////////////////////////////////////////////////////////
	// Clockwise/anti-clockwise polygon test
	//
	// Polygon: xp[], yp[]
	// Polygon point Number: pNum
	//
	// Return Value:
	//		true	- Clockwise Polygon
	//		false	- Anti-clockwise Polygon
	bool JugClockwiseOrNot(int pNum, double xp[], double yp[]);

	//////////////////////////////////////////////////////////////////////
	// Calculate the value of determinant:
	//			| a0  a1  a2  a3  |
	//			| a4  a5  a6  a7  |
	//			| a8  a9  a10 a11 |
	//			| a12 a13 a14 a15 |
	double Determinant4(double a[]);

	//////////////////////////////////////////////////////////////////////
	// Calculate the value of determinant:
	//			| a0  a1  a2 |
	//			| a3  a4  a5 |
	//			| a6  a7  a8 |
	double Determinant3(double a[]);

	//////////////////////////////////////////////////////////////////////
	// Calculate sphere equation of four points:
	//		(x[0],y[0],z[0]), (x[1],y[1],z[1]), (x[2],y[2],z[2]) & (x[3],y[3],z[3])
	//
	// Return Value:
	//		true:
	//			Center point - (x0,y0,z0)
	//			Radius - R
	//		FLASE: the four points are co-planar
	bool CalSphereEquation(double x[], double y[], double z[],
						   double& x0, double& y0, double& z0, double& R );

	//////////////////////////////////////////////////////////////////////
	// To calculate the angle detarmined by points P1, P, and P2
	//
	//								P1
	//							   /
	//						     /	
	//						   /	
	//						 /  	
	//					   P--------P2
	//
	// Return Value:
	//		Angle value
	double CalAngle(double p1[], double p[], double p2[]);

    //////////////////////////////////////////////////////////////////////
    // To calculate the vector product n3 = n1 X n2
    //
    //		where n1, n2 and n3 are three dimensional vectors
    //
    // Return Value:
    //		vector n3
    void VectorProduct(double n1[], double n2[], double n3[]);

    //////////////////////////////////////////////////////////////////////
    // To calculate the vector project n3 = n1 * n2
    //
    //		where n1, n2 are three dimensional vectors
    //
    // Return Value:
    //		project result
    double VectorProject(double n1[], double n2[]);

    //////////////////////////////////////////////////////////////////////
    // To calculate the vector project n3 = n1 * n2
    //
    //		where n1, n2 are n dimensional vectors
    //
    // Return Value:
    //		project result
    double VectorProject(double n1[], double n2[], const int n);

	//////////////////////////////////////////////////////////////////////
	// To calculate the triple product of vectors result = va * (vb X vc)
	//
	//		where n1, n2 are three dimensional vectors
	//
	// Return Value:
	//		product result
	static double TripleProduct(double va[], double vb[], double vc[]);

	//////////////////////////////////////////////////////////////////////
	// To calculate the new postion of point (px, py, pz) after rotating
	//		along X, Y, Z axis or arbitrary vector (x1, y1, z1)->(x2, y2, z2)
	//
	//		where angle is the rotate angle in degree
	//
	// Return Value:
	//		new point position (px1,py1,pz1)
	void RotatePointAlongX(double px, double py, double pz, double angle, 
				double &px1, double &py1, double &pz1);
	void RotatePointAlongY(double px, double py, double pz, double angle, 
				double &px1, double &py1, double &pz1);
	void RotatePointAlongZ(double px, double py, double pz, double angle, 
				double &px1, double &py1, double &pz1);
	void RotatePointAlongVector(double px, double py, double pz, 
				double x1, double y1, double z1, double x2, double y2, double z2,
				double angle, double &px1, double &py1, double &pz1);

	//////////////////////////////////////////////////////////////////////
	// To sort an array by the quick-sort algorithm 
	void QuickSort(int a[], int n) {QuickSort(a,0,n-1,true);}
	void QuickSort(int pArr[], int d, int h, bool bAscending);

	//////////////////////////////////////////////////////////////////////
	// Detect whether two line segments intersect
	//	
	//	Note that:	1) intersect at the endpoints will return false;
	//				2) two line segments have some part overlapped will return false.
	bool JugTwoLineSegmentsIntersectOrNot(double x1, double y1, double x2, double y2,
				double x3, double y3, double x4, double y4);

    // To find orientation of ordered triplet (p, q, r).
    // The function returns following values
    // 0 --> p, q and r are colinear
    // 1 --> Clockwise
    // 2 --> Counterclockwise
    inline int orientation(Point2D p, Point2D q, Point2D r)
    {
        double val = (q.y - p.y) * (r.x - q.x) -
                  (q.x - p.x) * (r.y - q.y);

        if (val == 0) return 0;  // colinear
        return (val > 0)? 1: 2; // clock or counterclock wise
    }
    // ConvexHull2D(): Jarvis’s Algorithm or Wrapping
    //     Input:  P[] = an array of 2D points
    //                  presorted by increasing x and y-coordinates
    //             n =  the number of points in P[]
    //     Output: H = an array of the convex hull vertices
    void ConvexHull2D( Point2D* P, int n, std::vector<Point2D> &H);
};

#endif

#ifndef CW_POSITIONARRAY
#define CW_POSITIONARRAY

class GLPosition : public GLKObject
{
public:
    GLPosition(double xi, double yi, double zi) {xx=xi;yy=yi;zz=zi;};
    ~GLPosition() {};

    double x() {return xx;}
    double y() {return yy;}
    double z() {return zz;}
    void set_x(double xi) {xx=xi;};
    void set_y(double yi) {yy=yi;};
    void set_z(double zi) {zz=zi;};

private:
    double xx,yy,zz;
};

class GLPositionArray: public GLKObject
{
public:
    GLPositionArray() {Empty();};
    virtual ~GLPositionArray() {Empty();};

    ////////////////////////////////////////////////////////////
    //	Add position into the array
    void Add(double xi, double yi, double zi)
    {
        x.push_back(xi);	y.push_back(yi);	z.push_back(zi);
    };

    ////////////////////////////////////////////////////////////
    //	Clear all positions in the array
    void Empty()
    {
        x.clear();	y.clear();	z.clear();
    };

    ////////////////////////////////////////////////////////////
    //	Get the size of the array
    unsigned int GetSize() {return x.size();};

    ////////////////////////////////////////////////////////////
    //	Get the element (xi,yi,zi) at index - nIndex (begin from 0)
    void ElementAt(unsigned int nIndex, double &xi, double &yi, double &zi)
    {
        xi = x.at(nIndex);	yi = y.at(nIndex);	zi = z.at(nIndex);
    };

    ////////////////////////////////////////////////////////////
    //	Remove the element (xi,yi,zi) at index - nIndex (begin from 0)
    void RemoveAt(unsigned int nIndex)	//	Begin from 0
    {
        //x.RemoveAt(nIndex);		y.RemoveAt(nIndex);		z.RemoveAt(nIndex);
        x.erase(x.begin()+nIndex);	y.erase(y.begin()+nIndex);	z.erase(z.begin()+nIndex);
    }

    ////////////////////////////////////////////////////////////
    //	Insert the element (xi,yi,zi) at index - nIndex (begin from 0)
    void InsertAt(unsigned int nIndex, double xi, double yi, double zi)	//	Begin from 0
    {
        x.insert(x.begin()+nIndex, xi);
        y.insert(y.begin()+nIndex, yi);
        z.insert(z.begin()+nIndex, zi);
    }

private:
    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> z;
};

#endif
