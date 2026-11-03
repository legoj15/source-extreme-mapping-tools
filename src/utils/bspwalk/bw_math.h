//========= bspwalk: native BSP walker (docs/bsp_walker_rebuild.md) ==========//
//
// Purpose: small double-precision vector type for configuration-space geometry.
//          Source's Vector is float; near +-16384 a float ULP is ~0.002u, which
//          is too coarse for exact sliver gaps, so all C-space math uses dvec3.
//
//=============================================================================//

#ifndef BW_MATH_H
#define BW_MATH_H
#pragma once

#include <math.h>
#include "mathlib/vector.h"

struct dvec3
{
	double x, y, z;

	dvec3() : x( 0 ), y( 0 ), z( 0 ) {}
	dvec3( double ax, double ay, double az ) : x( ax ), y( ay ), z( az ) {}
	explicit dvec3( const Vector &v ) : x( v.x ), y( v.y ), z( v.z ) {}

	double operator[]( int i ) const { return ( &x )[i]; }
	double &operator[]( int i ) { return ( &x )[i]; }

	dvec3 operator+( const dvec3 &o ) const { return dvec3( x + o.x, y + o.y, z + o.z ); }
	dvec3 operator-( const dvec3 &o ) const { return dvec3( x - o.x, y - o.y, z - o.z ); }
	dvec3 operator-() const { return dvec3( -x, -y, -z ); }
	dvec3 operator*( double s ) const { return dvec3( x * s, y * s, z * s ); }
	dvec3 &operator+=( const dvec3 &o ) { x += o.x; y += o.y; z += o.z; return *this; }
	dvec3 &operator-=( const dvec3 &o ) { x -= o.x; y -= o.y; z -= o.z; return *this; }
};

inline double DDot( const dvec3 &a, const dvec3 &b ) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline dvec3 DCross( const dvec3 &a, const dvec3 &b )
{
	return dvec3( a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x );
}
inline double DLength( const dvec3 &a ) { return sqrt( DDot( a, a ) ); }
inline dvec3 DMin( const dvec3 &a, const dvec3 &b ) { return dvec3( a.x < b.x ? a.x : b.x, a.y < b.y ? a.y : b.y, a.z < b.z ? a.z : b.z ); }
inline dvec3 DMax( const dvec3 &a, const dvec3 &b ) { return dvec3( a.x > b.x ? a.x : b.x, a.y > b.y ? a.y : b.y, a.z > b.z ? a.z : b.z ); }

// Returns the length before normalizing; leaves v untouched if it is ~zero.
inline double DNormalize( dvec3 &v )
{
	double len = DLength( v );
	if ( len > 1e-300 )
	{
		v = v * ( 1.0 / len );
	}
	return len;
}

//-----------------------------------------------------------------------------
// Rigid transform with optional uniform scale: world = R * (scale * local) + origin.
// R is built exactly like mathlib's AngleMatrix (pitch = x, yaw = y, roll = z),
// but in double.
//-----------------------------------------------------------------------------
struct DTransform
{
	double m[3][3];
	dvec3 origin;
	double scale;

	DTransform() : scale( 1.0 )
	{
		for ( int i = 0; i < 3; i++ )
			for ( int j = 0; j < 3; j++ )
				m[i][j] = ( i == j ) ? 1.0 : 0.0;
	}

	void SetOrgAngles( const dvec3 &org, double pitchDeg, double yawDeg, double rollDeg, double flScale = 1.0 )
	{
		const double d2r = 3.14159265358979323846 / 180.0;
		double sy = sin( yawDeg * d2r ), cy = cos( yawDeg * d2r );
		double sp = sin( pitchDeg * d2r ), cp = cos( pitchDeg * d2r );
		double sr = sin( rollDeg * d2r ), cr = cos( rollDeg * d2r );

		m[0][0] = cp * cy;
		m[1][0] = cp * sy;
		m[2][0] = -sp;

		double crcy = cr * cy, crsy = cr * sy, srcy = sr * cy, srsy = sr * sy;
		m[0][1] = sp * srcy - crsy;
		m[1][1] = sp * srsy + crcy;
		m[2][1] = sr * cp;

		m[0][2] = sp * crcy + srsy;
		m[1][2] = sp * crsy - srcy;
		m[2][2] = cr * cp;

		origin = org;
		scale = flScale;
	}

	dvec3 Apply( const dvec3 &p ) const
	{
		dvec3 s = p * scale;
		return dvec3( m[0][0] * s.x + m[0][1] * s.y + m[0][2] * s.z + origin.x,
					  m[1][0] * s.x + m[1][1] * s.y + m[1][2] * s.z + origin.y,
					  m[2][0] * s.x + m[2][1] * s.y + m[2][2] * s.z + origin.z );
	}
};

#endif // BW_MATH_H
