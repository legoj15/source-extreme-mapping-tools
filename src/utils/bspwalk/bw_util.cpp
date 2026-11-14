//========= bspwalk: native BSP walker (docs/bsp_walker_rebuild.md) ==========//
//
// Purpose: small helpers shared by the bspwalk modules.
//
//=============================================================================//

#include "cmdlib.h"
#include "bsplib.h"
#include "bw_util.h"
#include <windows.h>
#include <psapi.h>
#include <new>

#pragma comment( lib, "psapi.lib" )

//-----------------------------------------------------------------------------
// Global new/delete on the Windows process heap (low-fragmentation, scales
// across threads). memoverride.cpp would route them to tier0's allocator,
// whose single CThreadFastMutex serialized every std::vector growth of the
// worker threads: GrowWorld took 0.7 s on 4 threads and 5.8 s on 32 on
// fightspace3 (a cdb sample had 31 of 32 workers in CThreadFastMutex::Lock).
// bspwalk.vpc defines NO_MEMOVERRIDE_NEW_DELETE so memoverride.cpp leaves
// these out; malloc/free stay on tier0, so the Valve code is unchanged.
// Each module pairs its own new and delete; the DLLs (vphysics, filesystem)
// hand their objects back through their own Destroy calls. memoverride.cpp
// itself must stay (#36): the filesystem DLL grows a caller's CUtlBuffer with
// tier0's malloc (IFileSystem::ReadFile in ReadVmtFile), so the exe has to free
// it there too; without memoverride.cpp the exe's CRT free got that block and
// RtlFreeHeap faulted in ReadVmt, Stage 1.
//-----------------------------------------------------------------------------
static void *BwHeapAlloc( size_t nSize )
{
	void *p = HeapAlloc( GetProcessHeap(), 0, nSize ? nSize : 1 );
	if ( !p )
		throw std::bad_alloc();
	return p;
}

static void BwHeapFree( void *p )
{
	if ( p )
		HeapFree( GetProcessHeap(), 0, p );
}

void *__cdecl operator new( size_t nSize )
{
	return BwHeapAlloc( nSize );
}

void *__cdecl operator new[]( size_t nSize )
{
	return BwHeapAlloc( nSize );
}

// The debug-CRT form that memdbgon.h maps new to.
void *__cdecl operator new( size_t nSize, int, const char *, int )
{
	return BwHeapAlloc( nSize );
}

void *__cdecl operator new[]( size_t nSize, int, const char *, int )
{
	return BwHeapAlloc( nSize );
}

void __cdecl operator delete( void *p ) noexcept
{
	BwHeapFree( p );
}

void __cdecl operator delete[]( void *p ) noexcept
{
	BwHeapFree( p );
}

void __cdecl operator delete( void *p, size_t ) noexcept
{
	BwHeapFree( p );
}

void __cdecl operator delete[]( void *p, size_t ) noexcept
{
	BwHeapFree( p );
}

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

bool EntNameMatches( const char *pszPattern, const char *pszName )
{
	if ( !pszPattern[0] || !pszName[0] )
		return false;
	int len = Q_strlen( pszPattern );
	// a trailing '*' is an I/O wildcard (CGlobalEntityList::FindEntityByName)
	if ( pszPattern[len - 1] == '*' )
		return !Q_strnicmp( pszPattern, pszName, len - 1 );
	return !Q_stricmp( pszPattern, pszName );
}

int FindEntityByTargetname( const char *pszName, int after )
{
	for ( int e = MAX( after + 1, 0 ); e < num_entities; e++ )
	{
		if ( EntNameMatches( pszName, EntValue( &entities[e], "targetname" ) ) )
			return e;
	}
	return -1;
}

dvec3 EntOrigin( entity_t *ent )
{
	Vector org( 0, 0, 0 );
	GetVectorForKey( ent, (char *)"origin", org );
	return dvec3( org );
}

double PeakWorkingSetBytes()
{
	PROCESS_MEMORY_COUNTERS pmc;
	if ( !GetProcessMemoryInfo( GetCurrentProcess(), &pmc, sizeof( pmc ) ) )
		return 0;
	return (double)pmc.PeakWorkingSetSize;
}
