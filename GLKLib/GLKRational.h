#ifndef	_CCL_GLKRATIONAL
#define	_CCL_GLKRATIONAL

#define	TRUNC_EPS	1.0e-3

class GLKRational
{
public:
	~GLKRational(void);
	
	// Constructors
	GLKRational( int numerator = 0 )
		: numer( numerator ), denom( 1 ) { };
    GLKRational( int numerator, int denominator )
		: numer( numerator ), denom( denominator ) { fixSigns( ); reduce( ); };

	// Assignment Ops
	const GLKRational & operator=( const int value );
	const GLKRational & operator=( const double value);
    const GLKRational & operator=( const GLKRational & rhs );
    const GLKRational & operator+=( const GLKRational & rhs );
    const GLKRational & operator-=( const GLKRational & rhs );
    const GLKRational & operator/=( const GLKRational & rhs );
    const GLKRational & operator*=( const GLKRational & rhs );

      // Unary Operators
    const GLKRational & operator++( );      // Prefix
    GLKRational operator++( int );          // Postfix
    const GLKRational & operator--( );      // Prefix
    GLKRational operator--( int );          // Postfix
    const GLKRational & operator+( ) const;
    GLKRational operator-( ) const;
    bool operator!( ) const;

      // Named Member Functions
    double toDouble( ) const             // Do the division
      { return static_cast<double>( numer ) / denom; }
    int toInt( ) const                   // Do the division
      { return numer >= 0 ? numer / denom : - ( -numer / denom ); }
    bool isPositive( ) const
      { return numer > 0; }
    bool isNegative( ) const
      { return numer < 0; }
    bool isZero( ) const
      { return numer == 0; }

private:
      // A rational number is represented by a numerator and
      // denominator in reduced form
    long numer;                      // The numerator
    long denom;                      // The denominator

    void fixSigns( );               // Ensures denom >= 0
    void reduce( );                 // Ensures lowest form

	long gcd( long n, long m );
	long gcd1( long n, long m );
};

// Math Binary Ops
GLKRational operator+( const GLKRational & lhs, const GLKRational & rhs );
GLKRational operator-( const GLKRational & lhs, const GLKRational & rhs );
GLKRational operator/( const GLKRational & lhs, const GLKRational & rhs );
GLKRational operator*( const GLKRational & lhs, const GLKRational & rhs );

// Relational & Equality Ops
bool operator< ( const GLKRational & lhs, const GLKRational & rhs );
bool operator<=( const GLKRational & lhs, const GLKRational & rhs );
bool operator> ( const GLKRational & lhs, const GLKRational & rhs );
bool operator>=( const GLKRational & lhs, const GLKRational & rhs );
bool operator==( const GLKRational & lhs, const GLKRational & rhs );
bool operator!=( const GLKRational & lhs, const GLKRational & rhs );

#endif