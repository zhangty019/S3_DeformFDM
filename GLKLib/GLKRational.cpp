#include "GLKRational.h"

GLKRational::~GLKRational(void)
{
}

long GLKRational::gcd1( long n, long m )
{
	if( n % m == 0 )
		return m;
	else
		return gcd1( m, n % m );
}

long GLKRational::gcd( long m, long n )
{
	if( m > 0 )
		return gcd1( n, m );
	else
		return gcd1( n, -m );
}

void GLKRational::fixSigns( )
{
    if( denom < 0 )
    {
        denom = -denom;
        numer = -numer;
    }
}

void GLKRational::reduce( )
{
    long d = 1;

    if( denom != 0 && numer != 0 )
        d = gcd( numer, denom );

    if( d > 1 )
    {
        numer /= d;
        denom /= d;
    }

	if (numer==0) denom = 1;

	fixSigns( );
}

#define TIMES_ESP	1.0/TRUNC_EPS

const GLKRational & GLKRational::operator=( const GLKRational & rhs )
{
	numer = rhs.numer;	denom = rhs.denom;
    reduce( );

    return *this;
}

const GLKRational & GLKRational::operator=( const double value )
{
	numer=(long)(value*TIMES_ESP);		denom=(long)(TIMES_ESP);
    reduce( );

    return *this;
}

const GLKRational & GLKRational::operator=( const int value )
{
    numer = value;	 denom = 1;

    return *this;
}

const GLKRational & GLKRational::operator+=( const GLKRational & rhs )
{
    numer = numer * rhs.denom + rhs.numer * denom;
    denom = denom * rhs.denom;
    reduce( );

    return *this;
}

const GLKRational & GLKRational::operator-=( const GLKRational & rhs )
{
    numer = numer * rhs.denom - rhs.numer * denom;
    denom = denom * rhs.denom;
    reduce( );

    return *this;
}


const GLKRational & GLKRational::operator*=( const GLKRational & rhs )
{
    int newNumer = numer * rhs.numer;
    int newDenom = denom * rhs.denom;

    numer = newNumer;
    denom = newDenom;
    reduce( );

    return *this;
}

const GLKRational & GLKRational::operator/=( const GLKRational & rhs )
{
    int newNumer = numer * rhs.denom;
    int newDenom = denom * rhs.numer;

    numer = newNumer;
    denom = newDenom;

    reduce( );

    return *this;
}

GLKRational operator+( const GLKRational & lhs, const GLKRational & rhs )
{
    GLKRational answer( lhs );   // Initialize answer with lhs
    answer += rhs;            // Add the second operand
    return answer;            // Return answer by copy
}


GLKRational operator-( const GLKRational & lhs, const GLKRational & rhs )
{
    GLKRational answer( lhs );   // Initialize answer with lhs
    answer -= rhs;            // Subtract the second operand
    return answer;            // Return answer by copy
}

GLKRational operator*( const GLKRational & lhs, const GLKRational & rhs )
{
    GLKRational answer( lhs );   // Initialize answer with lhs
    answer *= rhs;            // Multiply the second operand
    return answer;            // Return answer by copy
}

GLKRational operator/( const GLKRational & lhs, const GLKRational & rhs )
{
    GLKRational answer( lhs );   // Initialize answer with lhs
    answer /= rhs;            // Divide the second operand
    return answer;            // Return answer by copy
}


bool operator==( const GLKRational & lhs, const GLKRational & rhs )
{
    return (lhs - rhs).isZero( );
}


bool operator!=( const GLKRational & lhs, const GLKRational & rhs )
{
    return (!lhs - rhs).isZero( );
}

bool operator<( const GLKRational & lhs, const GLKRational & rhs )
{
    return (lhs - rhs).isNegative( );
}

bool operator>( const GLKRational & lhs, const GLKRational & rhs )
{
    return (lhs - rhs).isPositive( );
}

bool operator<=( const GLKRational & lhs, const GLKRational & rhs )
{
    return !(lhs - rhs).isPositive( );
}

bool operator>=( const GLKRational & lhs, const GLKRational & rhs )
{
    return (!lhs - rhs).isNegative( );
}

const GLKRational & GLKRational::operator++( )  // Prefix form
{
    numer += denom;
    return *this;
}

GLKRational GLKRational::operator++( int )      // Postfix form
{
    GLKRational tmp = *this;
    numer += denom;
    return tmp;
}

const GLKRational & GLKRational::operator--( )  // Prefix form
{
    numer -= denom;
    return *this;
}

GLKRational GLKRational::operator--( int )      // Postfix form
{
    GLKRational tmp = *this;
    numer -= denom;
    return tmp;
}

bool GLKRational::operator!( ) const
{
    return !numer;
}

const GLKRational & GLKRational::operator+( ) const
{
    return *this;
}

GLKRational GLKRational::operator-( ) const
{
    return GLKRational( -numer, denom );
}
