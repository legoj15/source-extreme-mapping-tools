//========= bspwalk: native BSP walker (docs/bsp_walker_rebuild.md) ==========//
//
// Purpose: small helpers shared by the bspwalk modules.
//
//=============================================================================//

#include "cmdlib.h"
#include "bsplib.h"
#include "bw_util.h"

const char *EntValue( entity_t *ent, const char *pszKey )
{
	return ValueForKey( ent, (char *)pszKey );
}

void TransformShape( ConvexShape &shape, const DTransform &xf )
{
	for ( size_t i = 0; i < shape.verts.size(); i++ )
		shape.verts[i] = xf.Apply( shape.verts[i] );

	for ( size_t i = 0; i < shape.faceNormals.size(); i++ )
	{
		const dvec3 &n = shape.faceNormals[i];
		dvec3 r( xf.m[0][0] * n.x + xf.m[0][1] * n.y + xf.m[0][2] * n.z,
				 xf.m[1][0] * n.x + xf.m[1][1] * n.y + xf.m[1][2] * n.z,
				 xf.m[2][0] * n.x + xf.m[2][1] * n.y + xf.m[2][2] * n.z );
		DNormalize( r );
		shape.faceNormals[i] = r;
	}
	shape.Finish();
}
