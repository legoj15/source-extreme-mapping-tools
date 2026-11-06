//========= bspwalk: native BSP walker (docs/bsp_walker_rebuild.md) ==========//
//
// Purpose: entry point and CLI.
//
//   bspwalk <map.bsp> -game <dir> [-profile <name>] [-dumpobj <base>] [-checkprops]
//           [-verify] [-seed x y z]... [-nowalk] [-noarcs | -cpuarcs | -arccheck]
//           [-arcallnodes] [-novis | -notierb] [-viewpoint x y z]... [-selftest]
//           [-threads N] [-out <json>]
//
// Milestone 5 (#14): loads the BSP, builds the player collision world in
// configuration space (Stage 1), the exact walkable polygons (Stage 2), the
// CPU walk graph and the ballistic arcs (GPU via OptiX, or the CPU reference)
// with the flood from the seeds (Stage 3), the eye volume (Stage 4) and the
// tiered face visibility (Stage 5: tier A from the PVS, tier B from the CPU
// ray reference), and optionally dumps them as OBJ. -out writes run stats, the
// "reach" result and the "faces" list (the harness contract).
//
//=============================================================================//

#include "cmdlib.h"
#include "bsplib.h"
#include "threads.h"
#include "tier0/icommandline.h"
#include "tier0/platform.h"
#include "mathlib/mathlib.h"
#include "bw_grow.h"
#include "bw_world.h"
#include "bw_props.h"
#include "bw_disp.h"
#include "bw_obj.h"
#include "bw_walk.h"
#include "bw_arcs.h"
#include "bw_vis.h"

#define BSPWALK_MILESTONE 5

// lumps 22/23 (vbsp -emitsideids), loaded by LoadBSPFile (bsplib.cpp)
extern CUtlVector<dfacesideid_index_t> g_FaceSideIdIndex;
extern CUtlVector<int> g_FaceSideIdData;

struct StageTimes
{
	double init, load, brushes, props, disps, grow, verify, walk, eyes, vis, dumpobj, total;
	StageTimes() { memset( this, 0, sizeof( *this ) ); }
};

static void PrintUsage()
{
	Msg( "usage: bspwalk <map.bsp> -game <dir> [options]\n"
		 "  -profile <name>  game profile from bspwalk_profiles.txt (default: by game directory, else hl2)\n"
		 "  -dumpobj <base>  write <base>_raw.obj, <base>_stand.obj, <base>_crouch.obj,\n"
		 "                   and <base>_reached.obj / <base>_unreached.obj (walkable polygons)\n"
		 "  -checkprops      compare each prop model's collision AABB with its .mdl hull\n"
		 "  -verify          check every grown polytope against its exact Minkowski point set\n"
		 "  -verifydump <f>  with -verify: write every failing convex to <f> (for -growdebug / offline checks)\n"
		 "  -seed <x y z>    extra flood seed (feet point), repeatable; info_player_*,\n"
		 "                   info_ladder_dismount and prop_vehicle_* are always seeds\n"
		 "  -vehicle-free    from vehicle seeds, walk / jump / drop / slide edges count both ways\n"
		 "  -nowalk          stop after Stage 1 (no walkable polygons, graph or flood)\n"
		 "  -keepoffmap      keep walkable polygons outside the map (dropped by default: every probe in a cluster -1 leaf)\n"
		 "  -nosurf          no surfing: slides are left only downhill and launch walk-offs only (the #13 model)\n"
		 "  -reachslim       write reach.nodes without their vertices (counts and the unreached report stay)\n"
		 "  -noarcs          no ballistic arcs (walk / jump-up / drop edges only)\n"
		 "  -cpuarcs         trace arcs on the CPU reference path (default: GPU when available)\n"
		 "  -arccheck        trace every arc on both paths and report disagreements (flood follows the CPU)\n"
		 "  -arcallnodes     launch arcs from every reached node, not only rim and slope nodes (measures that rule)\n"
		 "  -novis           stop after the flood (no eye volume; the JSON lists no faces)\n"
		 "  -notierb         tier A (PVS) only; every face the PVS does not hide stays visible\n"
		 "  -cpuvis          tier B face and mirror rays on the CPU reference (default: GPU when available)\n"
		 "  -vischeck        tier B face and mirror rays on both paths; the CPU decides, disagreements reported\n"
		 "  -visgpusim       the GPU path's batching with every segment traced on the CPU (tests it without a GPU)\n"
		 "  -viewpoint <x y z> extra eye point (a camera the map renders from), repeatable\n"
		 "  -eyespacing <u>  tier B eye sample spacing (default 8), widened until about -eyemaxsamples <n> (default 24e6)\n"
		 "  -fanrays <n>     rays per fan eye (default 1024); -fanspacing <u> one fan eye per cell (default 64)\n"
		 "  -facespacing <u> tier B face sample spacing (default 4; above 4 weakens tier B); -eyebins <n> angular bins per axis (default 32)\n"
		 "  -eyebinlevels <n> a face tile with no clear ray retries at 2x the bins, up to n levels (default 2)\n"
		 "  -selftest        known-answer checks of the grow, polygon, edge, arc and visibility routines (no map needed)\n"
		 "  -threads <n>     worker threads (default: all cores)\n"
		 "  -out <json>      write run stats as JSON\n" );
}

// The first argument that is neither an option nor an option's value.
static const char *FindBspArg( int argc, char **argv )
{
	static const char *s_ValueOptions[] = { "-game", "-vproject", "-dumpobj", "-out", "-threads", "-growdebug", "-verifydump",
											"-profile", "-eyespacing", "-fanrays", "-fanspacing", "-facespacing", "-eyebins", "-eyebinlevels", "-eyemaxsamples" };
	for ( int i = 1; i < argc; i++ )
	{
		if ( argv[i][0] == '-' )
		{
			if ( !Q_stricmp( argv[i], "-seed" ) || !Q_stricmp( argv[i], "-viewpoint" ) )
			{
				i += 3;
				continue;
			}
			for ( int k = 0; k < ARRAYSIZE( s_ValueOptions ); k++ )
			{
				if ( !Q_stricmp( argv[i], s_ValueOptions[k] ) )
				{
					i++;
					break;
				}
			}
			continue;
		}
		return argv[i];
	}
	return NULL;
}

// Length of the well-formed UTF-8 sequence at s (RFC 3629: no overlong form, no
// surrogate, at most U+10FFFF), or 0 when s does not start one.
static int Utf8SequenceLength( const unsigned char *s )
{
	int n;
	unsigned int cp;
	if ( s[0] < 0x80 )
		return 1;
	if ( ( s[0] & 0xE0 ) == 0xC0 )
		n = 2, cp = s[0] & 0x1F;
	else if ( ( s[0] & 0xF0 ) == 0xE0 )
		n = 3, cp = s[0] & 0x0F;
	else if ( ( s[0] & 0xF8 ) == 0xF0 )
		n = 4, cp = s[0] & 0x07;
	else
		return 0;
	for ( int i = 1; i < n; i++ )
	{
		if ( ( s[i] & 0xC0 ) != 0x80 )	// also stops at the terminator
			return 0;
		cp = ( cp << 6 ) | ( s[i] & 0x3F );
	}
	static const unsigned int s_Min[5] = { 0, 0, 0x80, 0x800, 0x10000 };
	if ( cp < s_Min[n] || cp > 0x10FFFF || ( cp >= 0xD800 && cp <= 0xDFFF ) )
		return 0;
	return n;
}

// Windows-1252 code points of 0x80..0x9F (0: undefined there, kept as U+00XX the
// way Windows' own conversion keeps them); 0xA0..0xFF are Latin-1.
static const unsigned short s_Cp1252High[32] = {
	0x20AC, 0, 0x201A, 0x0192, 0x201E, 0x2026, 0x2020, 0x2021, 0x02C6, 0x2030, 0x0160, 0x2039, 0x0152, 0, 0x017D, 0,
	0, 0x2018, 0x2019, 0x201C, 0x201D, 0x2022, 0x2013, 0x2014, 0x02DC, 0x2122, 0x0161, 0x203A, 0x0153, 0, 0x017E, 0x0178 };

// The JSON is UTF-8, but the strings need not be: the BSP path comes from narrow
// argv (the ANSI code page) and entity strings are in Hammer's code page. A byte
// that does not start a valid UTF-8 sequence goes out as the \uXXXX escape of its
// Windows-1252 character; valid sequences go out as they are. That assumes the
// usual Western install: on another ANSI code page (a CJK one, say) such text
// comes out as the wrong characters, though still as valid JSON.
static void JsonString( FILE *fp, const char *str )
{
	const unsigned char *s = (const unsigned char *)str;
	fputc( '"', fp );
	while ( *s )
	{
		int n = 1;
		if ( *s == '"' || *s == '\\' )
			fprintf( fp, "\\%c", *s );
		else if ( *s < 0x20 )
			fprintf( fp, "\\u%04x", *s );
		else if ( *s < 0x80 )
			fputc( *s, fp );
		else if ( ( n = Utf8SequenceLength( s ) ) != 0 )
			fwrite( s, 1, n, fp );
		else
		{
			unsigned int cp = *s < 0xA0 && s_Cp1252High[*s - 0x80] ? s_Cp1252High[*s - 0x80] : *s;
			fprintf( fp, "\\u%04x", cp );
			n = 1;
		}
		s += n;
	}
	fputc( '"', fp );
}

// False when the category has no convexes (the bounds are then meaningless).
static bool WorldAABB( const CollisionWorld &world, int category, dvec3 &mins, dvec3 &maxs )
{
	mins = dvec3( 1e300, 1e300, 1e300 );
	maxs = dvec3( -1e300, -1e300, -1e300 );
	bool any = false;
	for ( size_t i = 0; i < world.convexes.size(); i++ )
	{
		if ( world.convexes[i].category != category )
			continue;
		mins = DMin( mins, world.convexes[i].shape.mins );
		maxs = DMax( maxs, world.convexes[i].shape.maxs );
		any = true;
	}
	return any;
}

static int CountGrown( const std::vector<GrownPolytope> &v )
{
	int n = 0;
	for ( size_t i = 0; i < v.size(); i++ )
		n += v[i].IsValid();
	return n;
}

static void JsonPoint( FILE *fp, const dvec3 &p )
{
	fprintf( fp, "[%.4f, %.4f, %.4f]", p.x, p.y, p.z );
}

static dvec3 PolyCentroid( const DWinding &w )
{
	dvec3 c( 0, 0, 0 );
	for ( size_t i = 0; i < w.size(); i++ )
		c += w[i];
	return w.empty() ? c : c * ( 1.0 / w.size() );
}

// "special" and "teleports" inside the reach object (#13B special movement).
static void WriteSpecialJson( FILE *fp, const WalkGraph &g )
{
	const SpecialGraph &sg = g.special;
	const SpecialStats &ss = sg.stats;
	fprintf( fp, "    \"special\": {\"counts\": {" );
	for ( int k = 0; k < SPECIAL_KIND_COUNT; k++ )
		fprintf( fp, "%s\"%s\": %d", k ? ", " : "", SpecialKindName( k ), ss.count[k] );
	fprintf( fp, "}, \"reached\": {" );
	for ( int k = 0; k < SPECIAL_KIND_COUNT; k++ )
		fprintf( fp, "%s\"%s\": %d", k ? ", " : "", SpecialKindName( k ), ss.reached[k] );
	fprintf( fp, "}, \"edges\": %d, \"ride_links\": %d, \"conveyor_nodes\": %d, \"vehicle_seeds\": %d, "
			 "\"vehicle_free\": %s, \"vehicle_nodes\": %d, \"skipped_non_player_triggers\": %d, "
			 "\"skipped_fake_ladders\": %d, \"launchers\": %d, \"launch_pairs\": %lld, \"launch_arcs\": %lld, "
			 "\"launch_free\": %lld, \"launch_joined\": %lld, \"targets\": %d, \"target_pairs\": %lld, "
			 "\"target_arcs\": %lld, \"target_free\": %lld, \"target_joined\": %lld, \"build_s\": %.3f, "
			 "\"launch_s\": %.3f, \"targets_s\": %.3f},\n",
			 ss.edges, ss.rideLinks, ss.conveyorNodes, ss.vehicleSeeds, g.vehicleFree ? "true" : "false", ss.vehicleNodes,
			 ss.skippedNoPlayers, ss.skippedFakeLadders, (int)sg.launchers.size(), ss.launchPairs, ss.launchArcs,
			 ss.launchFree, ss.launchJoined, (int)sg.targets.size(), ss.targetPairs, ss.targetArcs, ss.targetFree,
			 ss.targetJoined, ss.tBuild, ss.tLaunch, ss.tTargets );
	fprintf( fp, "    \"teleports\": [" );
	for ( size_t i = 0; i < sg.teleports.size(); i++ )
	{
		const TeleportReport &t = sg.teleports[i];
		bool triggered = t.vertex >= 0 && t.vertex < (int)sg.reached.size() && sg.reached[t.vertex];
		fprintf( fp, "%s\n      {\"entity\": %d, \"label\": ", i ? "," : "", t.entity );
		JsonString( fp, t.vertex >= 0 ? sg.verts[t.vertex].label : "" );
		fprintf( fp, ", \"target\": " );
		JsonString( fp, t.target );
		fprintf( fp, ", \"landmark\": " );
		JsonString( fp, t.landmark );
		fprintf( fp, ", \"start_disabled\": %s, \"players\": %s, \"triggered\": %s, \"dests\": [",
				 t.startDisabled ? "true" : "false", t.players ? "true" : "false", triggered ? "true" : "false" );
		for ( size_t d = 0; d < t.dests.size(); d++ )
		{
			fprintf( fp, "%s{\"entity\": %d, \"point\": ", d ? ", " : "", t.dests[d].entity );
			JsonPoint( fp, t.dests[d].point );
			if ( t.dests[d].node >= 0 )
				fprintf( fp, ", \"seed_node\": %d}", t.dests[d].node );
			else
				fprintf( fp, ", \"seed_node\": null}" );
		}
		fprintf( fp, "]}" );
	}
	fprintf( fp, "\n    ],\n" );
}

// -reachslim: the nodes go out without their vertices (#21).
static bool s_bReachSlim = false;

// The "reach" object (docs/bsp_walker_rebuild.md, bspwalk usage).
static void WriteReachJson( FILE *fp, const WalkGraph &g )
{
	const WalkStats &st = g.stats;
	fprintf( fp, "  \"reach\": {\n    \"seeds\": [" );
	for ( size_t i = 0; i < g.seeds.size(); i++ )
	{
		fprintf( fp, "%s\n      {\"point\": ", i ? "," : "" );
		JsonPoint( fp, g.seeds[i].point );
		fprintf( fp, ", \"source\": " );
		JsonString( fp, g.seeds[i].source );
		if ( g.seeds[i].vehicle )
			fprintf( fp, ", \"vehicle\": true" );
		if ( g.seeds[i].node >= 0 )
			fprintf( fp, ", \"node\": %d}", g.seeds[i].node );
		else
			fprintf( fp, ", \"node\": null}" );
	}
	fprintf( fp, "\n    ],\n" );
	fprintf( fp, "    \"edges\": {" );
	for ( int k = 0; k < EDGE_KIND_COUNT; k++ )
		fprintf( fp, "%s\"%s\": %d", k ? ", " : "", WalkEdgeKindName( k ), st.edgeCounts[k] );
	fprintf( fp, "},\n" );
	fprintf( fp, "    \"node_count\": %d, \"total_area\": %.3f, \"reached_count\": %d, \"reached_area\": %.3f,\n",
			 (int)g.nodes.size(), st.totalArea, st.reachedCount, st.reachedArea );
	fprintf( fp, "    \"up_faces\": %d, \"mover_entities\": %d, \"mover_poses\": %d, \"mover_grow_failed\": %d, \"slide_nodes\": %d,\n",
			 st.upFaces, st.moverEntities, st.moverPoses, st.moverGrowFailed, st.slideNodes );
	fprintf( fp, "    \"offmap_nodes\": %d, \"offmap_area\": %.3f, \"slim\": %s, \"surf\": %s, \"surf_nodes\": %d,\n",
			 st.offMapNodes, st.offMapArea, s_bReachSlim ? "true" : "false", SurfEnabled() ? "true" : "false", st.surfNodes );
	WriteSpecialJson( fp, g );
	const ArcStats &as = g.arcs;
	fprintf( fp, "    \"arcs\": {\"path\": \"%s\", \"reach\": %.3f, \"reach_max\": %.3f, \"launch\": \"%s\", \"rim_nodes\": %d, \"slope_nodes\": %d, \"segment_sagitta\": %g, \"rounds\": %d, \"pairs\": %lld, "
			 "\"arcs_traced\": %lld, \"arcs_free\": %lld, \"pairs_free\": %lld, \"mesh_tris\": [%d, %d], \"gas_mb\": [%.2f, %.2f],\n",
			 as.path, as.reach, as.reachMax, as.allNodes ? "all" : "rim", as.rimNodes, as.slopeNodes, ARC_SAGITTA, as.rounds, as.pairs, as.arcs, as.arcsFree, as.pairsFree, as.meshTris[0], as.meshTris[1],
			 as.gasMB[0], as.gasMB[1] );
	if ( as.checked )
		fprintf( fp, "      \"check\": {\"arcs\": %lld, \"gpu_only_blocked\": %lld, \"cpu_only_blocked\": %lld, "
				 "\"max_distance\": %.6g, \"float_bound\": %g, \"unexplained\": %lld, \"cpu_only_nodes\": %d, "
				 "\"gpu_only_nodes\": %d},\n",
				 as.checkArcs, as.gpuOnlyBlocked, as.cpuOnlyBlocked, as.checkMaxDist, ARC_FLOAT_BOUND, as.unexplained,
				 as.cpuOnlyNodes, as.gpuOnlyNodes );
	fprintf( fp, "      \"per_round\": [" );
	for ( size_t r = 0; r < as.perRound.size(); r++ )
	{
		const ArcRoundStats &rs = as.perRound[r];
		fprintf( fp, "%s\n        {\"frontier\": %d, \"launch\": %d, \"pairs\": %lld, \"arcs\": %lld, \"arcs_free\": %lld, \"pairs_free\": %lld, "
				 "\"reached\": %d, \"gen_s\": %.3f, \"trace_s\": %.3f}",
				 r ? "," : "", rs.frontier, rs.launch, rs.pairs, rs.arcs, rs.arcsFree, rs.pairsFree, rs.reached, rs.tGen, rs.tTrace );
	}
	fprintf( fp, "\n      ]},\n" );

	fprintf( fp, "    \"unreached_walkable\": [" );
	bool first = true;
	for ( size_t i = 0; i < g.nodes.size(); i++ )
	{
		if ( g.reached[i] )
			continue;
		const WalkNode &n = g.nodes[i];
		fprintf( fp, "%s\n      {\"id\": %d, \"centroid\": ", first ? "" : ",", (int)i );
		JsonPoint( fp, PolyCentroid( n.poly ) );
		fprintf( fp, ", \"area\": %.4f, \"category\": \"%s\"}", n.area, CategoryName( n.category ) );
		first = false;
	}
	fprintf( fp, "\n    ],\n" );

	// plane: n . x = d, so z = (d - nx x - ny y) / nz on the polygon
	fprintf( fp, "    \"nodes\": [" );
	for ( size_t i = 0; i < g.nodes.size(); i++ )
	{
		const WalkNode &n = g.nodes[i];
		fprintf( fp, "%s\n      {\"id\": %d, \"category\": \"%s\", \"convex\": %d, \"pose\": %d, \"reached\": %s, "
				 "\"area\": %.4f, \"plane\": [%.9f, %.9f, %.9f, %.6f], \"normal_z\": %.6f, "
				 "\"stand_count\": %d, \"stand_area\": %.4f",
				 i ? "," : "", (int)i, CategoryName( n.category ), n.convex, n.pose, g.reached[i] ? "true" : "false",
				 n.area, n.plane.n.x, n.plane.n.y, n.plane.n.z, n.plane.d, n.plane.n.z, (int)n.stand.size(), n.standArea );
		if ( s_bReachSlim )
		{
			fprintf( fp, "}" );
			continue;
		}
		fprintf( fp, ", \"verts\": [" );
		for ( size_t k = 0; k < n.poly.size(); k++ )
		{
			if ( k )
				fputs( ", ", fp );
			JsonPoint( fp, n.poly[k] );
		}
		fprintf( fp, "]}" );
	}
	fprintf( fp, "\n    ]\n  },\n" );
}

// "eyes" and "vis" (#14).
static void WriteVisJson( FILE *fp, const EyeVolume &e, const VisResult &v, const VisOptions &vo )
{
	fprintf( fp, "  \"eyes\": {\"pieces\": {" );
	for ( int s = 0; s < EYESRC_COUNT; s++ )
		fprintf( fp, "%s\"%s\": %d", s ? ", " : "", EyeSourceName( s ), e.pieces[s] );
	fprintf( fp, "}, \"mapped_pieces\": %d, \"sky\": ", e.mappedPieces );
	if ( e.sky )
		fprintf( fp, "{\"origin\": [%.4f, %.4f, %.4f], \"scale\": %g}", e.skyOrigin.x, e.skyOrigin.y, e.skyOrigin.z, e.skyScale );
	else
		fprintf( fp, "null" );
	fprintf( fp, ", \"clusters\": %d, \"eye_clusters\": %d, \"viewpoints\": %d, \"samples\": %d, \"spacing\": %g, "
			 "\"bare_clusters\": %d, \"build_s\": %.3f},\n",
			 (int)e.clusters.size(), e.eyeClusters, e.viewpoints, (int)e.samples.size(), e.spacing,
			 (int)e.bareClusters.size(), e.tBuild );
	const VisStats &s = v.stats;
	fprintf( fp, "  \"vis\": {\"pvs\": %s, \"tier_b\": %s, \"faces\": %d, \"world_faces\": %d, \"entity_faces\": %d, "
			 "\"disp_faces\": %d, \"tier_a\": %d, \"tier_b_hidden\": %d, \"visible\": %d, \"reflective\": %d, "
			 "\"reflect_clusters\": %d, \"occluders\": %d, \"occluders_skipped\": %d, \"faces_no_leaf\": %d, "
			 "\"fan_rays\": %lld, \"face_rays\": %lld, \"mirror_rays\": %lld, \"visible_fan\": %d, \"visible_ray\": %d, "
			 "\"visible_mirror\": %d, \"visible_noeye\": %d, \"noeye_clusters\": %d, \"solid_eyes\": %d, "
			 "\"fan_rays_per_eye\": %d, \"fan_spacing\": %g, \"face_spacing\": %g, \"face_spacing_used_max\": %g, "
			 "\"faces_spacing_widened\": %d, \"eye_spacing_requested\": %g, \"eye_spacing_used\": %g, "
			 "\"bins\": %d, \"bin_levels\": %d, "
			 "\"path\": \"%s\", \"mesh_tris\": %d, \"check\": {\"segments\": %lld, \"gpu_only_blocked\": %lld, "
			 "\"cpu_only_blocked\": %lld, \"tile_levels_differ\": %d}, "
			 "\"tier_a_s\": %.3f, \"occluders_s\": %.3f, \"fan_s\": %.3f, \"face_s\": %.3f, \"total_s\": %.3f},\n",
			 v.pvs ? "true" : "false", vo.tierB ? "true" : "false", s.faces, s.worldFaces, s.entityFaces, s.dispFaces,
			 s.tierA, s.tierB, s.visible, s.reflective, s.reflectClusters, s.occluders, s.occludersSkipped, s.faceNoLeaf,
			 s.fanRays, s.faceRays, s.mirrorRays, s.visibleFan, s.visibleFace, s.visibleMirror, s.visibleNoEye, s.noEyeClusters, s.solidEyes, vo.fanRays,
			 vo.fanSpacing, vo.faceSpacing, s.facesWidened ? s.faceSpacingMax : vo.faceSpacing, s.facesWidened,
			 e.spacingRequested, e.spacing, vo.bins, vo.binLevels, s.path, s.meshTris, s.checkSegments, s.gpuOnlyBlocked,
			 s.cpuOnlyBlocked, s.checkTileDiffer, s.tTierA, s.tOccluders, s.tFan, s.tFace, s.tTotal );
}

// "faces": every face with its side ids (lumps 22/23), tier and reason.
static void WriteFacesJson( FILE *fp, const VisResult *pVis )
{
	fprintf( fp, "  \"faces\": [" );
	if ( pVis )
	{
		static const char *s_Tier[3] = { "visible", "B", "A" };
		for ( int f = 0; f < numfaces && f < (int)pVis->tier.size(); f++ )
		{
			fprintf( fp, "%s\n    {\"face\": %d, \"sideids\": [", f ? "," : "", f );
			if ( f < g_FaceSideIdIndex.Count() )
			{
				const dfacesideid_index_t &ix = g_FaceSideIdIndex[f];
				for ( int k = 0; k < ix.numIds; k++ )
				{
					int at = ix.firstId + k;
					if ( at >= 0 && at < g_FaceSideIdData.Count() )
						fprintf( fp, "%s%d", k ? ", " : "", g_FaceSideIdData[at] );
				}
			}
			fprintf( fp, "], \"tier\": \"%s\", \"reason\": \"%s\"}", s_Tier[pVis->tier[f]], VisReasonName( pVis->why[f] ) );
		}
		fprintf( fp, "\n  " );
	}
	fprintf( fp, "]\n" );
}

static bool WriteJson( const char *pszPath, const char *pszBsp, const CollisionWorld &world, const PlayerProfile &prof,
					   const StageTimes &t, int verifyFailures, int propMismatches, const WalkGraph *pGraph,
					   const EyeVolume *pEyes, const VisResult *pVis, const VisOptions &vo )
{
	FILE *fp = fopen( pszPath, "w" );
	if ( !fp )
		return false;

	const CollectStats &s = world.stats;
	dvec3 wmin, wmax;
	bool bWorld = WorldAABB( world, CAT_WORLD, wmin, wmax );

	fprintf( fp, "{\n  \"walker\": \"bspwalk\",\n  \"milestone\": %d,\n  \"bsp\": ", BSPWALK_MILESTONE );
	JsonString( fp, pszBsp );
	fprintf( fp, ",\n  \"profile\": {\"name\": " );
	JsonString( fp, prof.name );
	fprintf( fp, ", \"source\": \"%s\", \"half_width\": %g, \"stand_height\": %g, \"crouch_height\": %g, "
			 "\"eye_stand\": %g, \"eye_crouch\": %g, \"step_height\": %g, \"gravity\": %g, \"jump_velocity\": %.10g, "
			 "\"crouch_lift\": %g, \"run_speed\": %g, \"max_ground_speed\": %g, \"max_air_speed\": %g, \"reach_margin\": %g, "
			 "\"air_jumps\": %d, \"jump_apex\": %.6f, \"max_jump_rise\": %.6f, \"arc_airtime\": %.6f, \"arc_reach\": %.3f, \"arc_max_drop\": %.6f, \"arc_reach_max\": %.3f},\n",
			 prof.source, prof.halfWidth, prof.standHeight, prof.crouchHeight, prof.eyeStand, prof.eyeCrouch, prof.stepHeight,
			 prof.gravity, prof.jumpVelocity, prof.crouchLift, prof.runSpeed, prof.maxGroundSpeed, prof.maxAirSpeed,
			 prof.reachMargin, prof.airJumps, prof.jumpApex, prof.maxJumpRise, prof.arcAirtime, prof.arcReach, prof.arcMaxDrop, prof.arcReachMax );

	fprintf( fp, "  \"counts\": {" );
	for ( int c = 0; c < CAT_COUNT; c++ )
		fprintf( fp, "\"%s\": %d, ", CategoryName( c ), world.CountCategory( c ) );
	fprintf( fp, "\"convexes\": %d, \"grown_stand\": %d, \"grown_crouch\": %d, \"prop_models\": %d},\n",
			 (int)world.convexes.size(), CountGrown( world.stand ), CountGrown( world.crouch ), Props_ModelCount() );

	fprintf( fp, "  \"skipped\": {\"brushes_wrong_contents\": %d, \"brushes_build_failed\": %d, "
			 "\"brush_entities_nonsolid_class\": %d, \"brush_entities_passable\": %d, \"brush_entities_never_solid\": %d, "
			 "\"brush_entities_no_solid_brush\": %d, \"static_props_nonsolid\": %d, \"static_props_no_collision\": %d, "
			 "\"entity_props_nonsolid\": %d, \"entity_props_no_collision\": %d, \"disps_no_hull_collision\": %d, "
			 "\"disp_tris_degenerate\": %d, \"prop_convexes_build_failed\": %d, \"grow_failed\": %d},\n",
			 s.brushesWrongContents, s.brushesBuildFailed, s.brushEntitiesNonSolidClass, s.brushEntitiesPassable,
			 s.brushEntitiesNeverSolid, s.brushEntitiesNoSolidBrush, s.staticPropsNonSolid, s.staticPropsNoCollision,
			 s.entityPropsNonSolid, s.entityPropsNoCollision, s.dispsNoHullCollision, s.dispTrisDegenerate,
			 s.convexesBuildFailed, s.growFailed );
	fprintf( fp, "  \"prop_instances_hull_fallback\": %d,\n", s.propsHullFallback );

	if ( bWorld )
		fprintf( fp, "  \"world_aabb\": [[%.3f, %.3f, %.3f], [%.3f, %.3f, %.3f]],\n", wmin.x, wmin.y, wmin.z, wmax.x, wmax.y, wmax.z );
	else
		fprintf( fp, "  \"world_aabb\": null,\n" );
	fprintf( fp, "  \"model0_aabb\": [[%.3f, %.3f, %.3f], [%.3f, %.3f, %.3f]],\n",
			 dmodels[0].mins.x, dmodels[0].mins.y, dmodels[0].mins.z, dmodels[0].maxs.x, dmodels[0].maxs.y, dmodels[0].maxs.z );

	if ( verifyFailures >= 0 )
		fprintf( fp, "  \"verify_failures\": %d,\n", verifyFailures );
	if ( propMismatches >= 0 )
		fprintf( fp, "  \"checkprops_mismatches\": %d,\n", propMismatches );

	WalkStats ws;
	if ( pGraph )
	{
		ws = pGraph->stats;
		WriteReachJson( fp, *pGraph );
	}
	if ( pEyes && pVis )
		WriteVisJson( fp, *pEyes, *pVis, vo );
	fprintf( fp, "  \"timings_s\": {\"init\": %.3f, \"load\": %.3f, \"brushes\": %.3f, \"props\": %.3f, \"disps\": %.3f, "
			 "\"grow\": %.3f, \"verify\": %.3f, \"walk_polygons\": %.3f, \"walk_stand\": %.3f, \"walk_edges\": %.3f, "
			 "\"walk_flood\": %.3f, \"walk_special\": %.3f, \"walk_arcs\": %.3f, \"walk_arcs_build\": %.3f, \"walk_arcs_gen\": %.3f, "
			 "\"walk_arcs_launch\": %.3f, \"walk_arcs_special\": %.3f, \"walk\": %.3f, \"eyes\": %.3f, \"vis\": %.3f, "
			 "\"dumpobj\": %.3f, \"total\": %.3f},\n",
			 t.init, t.load, t.brushes, t.props, t.disps, t.grow, t.verify, ws.tPolys, ws.tStand, ws.tEdges, ws.tFlood,
			 ws.tSpecial, pGraph ? pGraph->arcs.tTotal : 0.0, pGraph ? pGraph->arcs.tBuild : 0.0, pGraph ? pGraph->arcs.tGen : 0.0,
			 pGraph ? pGraph->arcs.tTrace : 0.0, pGraph ? pGraph->special.stats.tLaunch : 0.0, t.walk, t.eyes, t.vis,
			 t.dumpobj, t.total );
	WriteFacesJson( fp, pVis );
	fprintf( fp, "}\n" );

	bool ok = !ferror( fp );
	fclose( fp );
	return ok;
}

static int GrowDebugFile( const char *pszPath )
{
	FILE *fp = fopen( pszPath, "r" );
	if ( !fp )
		Error( "bspwalk: cannot read %s\n", pszPath );
	std::vector<dvec3> pts;
	char line[512];
	int blocks = 0;
	while ( fgets( line, sizeof( line ), fp ) )
	{
		// a -verifydump file holds many convexes; use the first
		if ( line[0] == '#' && ++blocks > 1 )
			break;
		dvec3 v;
		if ( sscanf( line, " v %lf %lf %lf", &v.x, &v.y, &v.z ) == 3 )
			pts.push_back( v );
	}
	fclose( fp );

	ConvexShape shape;
	if ( !ConvexFromPoints( pts.data(), (int)pts.size(), shape ) )
		Error( "bspwalk: %d points do not form a solid\n", (int)pts.size() );
	Msg( "hull: %d of %d points, %d faces, %d edges\n", (int)shape.verts.size(), (int)pts.size(),
		 (int)shape.faces.size(), (int)shape.edges.size() );

	PlayerProfile prof;
	int failures = 0;
	for ( int posture = 0; posture < 2; posture++ )
	{
		PlayerHull hull = posture ? prof.Crouch() : prof.Stand();
		Msg( "%s:\n", posture ? "crouch" : "stand" );
		GrownPolytope g;
		g_bGrowDebug = true;
		bool ok = GrowConvex( shape, hull, g );
		g_bGrowDebug = false;
		char why[256] = "";
		ok = ok && VerifyGrown( shape, hull, g, why, sizeof( why ) );
		Msg( "%s: %s %s\n", posture ? "crouch" : "stand", ok ? "verified" : "FAILED", why );
		failures += !ok;
	}
	return failures ? 1 : 0;
}

int main( int argc, char **argv )
{
	double tStart = Plat_FloatTime();

	CommandLine()->CreateCmdLine( argc, argv );
	MathLib_Init( 2.2f, 2.2f, 0.0f, 1.0f, false, false, false, false );
	InstallSpewFunction();

	int exitCode = 0;
	bool selftest = CommandLine()->FindParm( "-selftest" ) != 0;
	if ( selftest && RunGrowSelfTest() )
		exitCode = 1;

	// -growdebug <file>: grow the hull of the "v x y z" lines of a file (as
	// printed by -verify for a failing convex) with every decision printed.
	const char *pszGrowDebug = CommandLine()->ParmValue( "-growdebug", (const char *)NULL );
	if ( pszGrowDebug )
		return GrowDebugFile( pszGrowDebug ) ? 1 : exitCode;

	const char *pszBspArg = FindBspArg( argc, argv );
	if ( !pszBspArg )
	{
		if ( !selftest )
		{
			PrintUsage();
			return 1;
		}
		return exitCode;
	}

	// Without -game the filesystem falls back to the VPROJECT environment
	// variable, as every SDK tool does. Say which game that is, since a stale
	// VPROJECT shows up later as missing models or a vphysics.dll load failure.
	if ( !CommandLine()->FindParm( "-game" ) && !CommandLine()->FindParm( "-vproject" ) )
	{
		const char *pszEnv = getenv( "VPROJECT" );
		if ( !pszEnv || !pszEnv[0] )
		{
			Warning( "bspwalk: no game directory: pass -game <dir> (the directory holding gameinfo.txt) or set VPROJECT\n" );
			PrintUsage();
			return 1;
		}
		Msg( "bspwalk: no -game given, using VPROJECT=%s\n", pszEnv );
	}

	// a relative path is taken against the process cwd; made absolute here so
	// the filesystem (which searches the game's paths) cannot resolve it elsewhere
	char szBsp[MAX_PATH];
	V_MakeAbsolutePath( szBsp, sizeof( szBsp ), pszBspArg );
	const char *pszBsp = szBsp;

	// -1 = all cores. A zero, negative or non-numeric count would run no
	// workers at all (nothing grown, nothing verified), so it falls back.
	numthreads = -1;
	if ( CommandLine()->FindParm( "-threads" ) )
	{
		const char *pszThreads = CommandLine()->ParmValue( "-threads", "" );
		int n = atoi( pszThreads );
		if ( n > 0 )
			numthreads = n;
		else
			Warning( "bspwalk: -threads \"%s\" is not a positive count; using all cores\n", pszThreads );
	}
	ThreadSetDefault();

	StageTimes t;
	double t0 = Plat_FloatTime();
	CmdLib_InitFileSystem( pszBsp );
	Props_Init();
	t.init = Plat_FloatTime() - t0;

	t0 = Plat_FloatTime();
	LoadBSPFile( pszBsp );
	ParseEntities();
	t.load = Plat_FloatTime() - t0;
	Msg( "bspwalk: %s (%d brushes, %d models, %d entities, %d displacements), %d threads\n",
		 pszBsp, numbrushes, nummodels, num_entities, g_dispinfo.Count(), numthreads );

	CollisionWorld world;
	PlayerProfile profile;
	if ( !LoadPlayerProfile( CommandLine()->ParmValue( "-profile", (const char *)NULL ), gamedir, profile ) )
		return 1;

	t0 = Plat_FloatTime();
	CollectWorldBrushes( world );
	CollectBrushEntities( world );
	t.brushes = Plat_FloatTime() - t0;

	t0 = Plat_FloatTime();
	CollectStaticProps( world );
	CollectEntityProps( world );
	t.props = Plat_FloatTime() - t0;

	t0 = Plat_FloatTime();
	CollectDisplacements( world );
	t.disps = Plat_FloatTime() - t0;

	t0 = Plat_FloatTime();
	GrowWorld( world, profile );
	t.grow = Plat_FloatTime() - t0;
	if ( world.stats.growFailed )
		Warning( "bspwalk: %d convexes failed to grow (degenerate)\n", world.stats.growFailed );

	int verifyFailures = -1;
	if ( CommandLine()->FindParm( "-verify" ) )
	{
		t0 = Plat_FloatTime();
		verifyFailures = VerifyWorld( world, profile, CommandLine()->ParmValue( "-verifydump", (const char *)NULL ) );
		t.verify = Plat_FloatTime() - t0;
		Msg( "verify: %d polytope(s) differ from their exact Minkowski sum\n", verifyFailures );
		if ( verifyFailures )
			exitCode = 1;
	}

	//
	// Stages 2 and 3 (CPU): walkable polygons, walk graph, flood.
	//
	WalkGraph graph;
	bool bWalk = !CommandLine()->FindParm( "-nowalk" );
	bool bFlooded = false;
	if ( bWalk )
	{
		std::vector<dvec3> cliSeeds;
		for ( int i = 1; i < argc; i++ )
		{
			if ( Q_stricmp( argv[i], "-seed" ) )
				continue;
			dvec3 p;
			if ( i + 3 >= argc || sscanf( argv[i + 1], "%lf", &p.x ) != 1 || sscanf( argv[i + 2], "%lf", &p.y ) != 1 ||
				 sscanf( argv[i + 3], "%lf", &p.z ) != 1 )
			{
				Warning( "bspwalk: -seed needs three numbers: -seed <x> <y> <z>\n" );
				return 1;
			}
			cliSeeds.push_back( p );
			i += 3;
		}

		t0 = Plat_FloatTime();
		ArcOptions arcOpts;
		arcOpts.enabled = !CommandLine()->FindParm( "-noarcs" );
		arcOpts.cpuOnly = CommandLine()->FindParm( "-cpuarcs" ) != 0;
		arcOpts.check = CommandLine()->FindParm( "-arccheck" ) != 0;
		arcOpts.allNodes = CommandLine()->FindParm( "-arcallnodes" ) != 0;
		arcOpts.vehicleFree = CommandLine()->FindParm( "-vehicle-free" ) != 0;
		graph.vehicleFree = arcOpts.vehicleFree;
		SetCullOffMap( !CommandLine()->FindParm( "-keepoffmap" ) );
		SetSurf( !CommandLine()->FindParm( "-nosurf" ) );
		BuildWalkPolygons( world, profile, graph );
		BuildWalkEdges( world, profile, graph );
		bFlooded = FloodWalkGraph( world, profile, cliSeeds, arcOpts, graph );
		if ( arcOpts.check && graph.arcs.checked && graph.arcs.unexplained )
			exitCode = 1;
		t.walk = Plat_FloatTime() - t0;
		if ( !bFlooded )
		{
			// a flood from nothing would call every face unreachable
			Warning( "bspwalk: no usable seed; pass -seed <x y z> (a feet point) or add an info_player_start\n" );
			exitCode = 1;
		}
	}

	//
	// Stages 4 and 5: eye volume, tiered visibility.
	//
	const char *pszObj = CommandLine()->ParmValue( "-dumpobj", (const char *)NULL );
	EyeVolume eyes;
	VisResult vis;
	VisOptions visOpts;
	bool bVis = bFlooded && !CommandLine()->FindParm( "-novis" );
	if ( bVis )
	{
		EyeOptions eyeOpts;
		eyeOpts.keepPieces = pszObj != NULL;
		eyeOpts.spacing = CommandLine()->ParmValue( "-eyespacing", 8.0f );
		eyeOpts.maxSamples = CommandLine()->ParmValue( "-eyemaxsamples", 24e6f );
		visOpts.tierB = !CommandLine()->FindParm( "-notierb" );
		visOpts.fanRays = CommandLine()->ParmValue( "-fanrays", 1024 );
		visOpts.fanSpacing = CommandLine()->ParmValue( "-fanspacing", 64.0f );
		visOpts.faceSpacing = CommandLine()->ParmValue( "-facespacing", 4.0f );
		visOpts.bins = CommandLine()->ParmValue( "-eyebins", 32 );
		visOpts.binLevels = CommandLine()->ParmValue( "-eyebinlevels", 2 );
		visOpts.cpuOnly = CommandLine()->FindParm( "-cpuvis" ) != 0;
		visOpts.check = CommandLine()->FindParm( "-vischeck" ) != 0;
		visOpts.gpuSim = CommandLine()->FindParm( "-visgpusim" ) != 0;
		if ( !( eyeOpts.spacing > 0 ) || !( eyeOpts.maxSamples > 0 ) || visOpts.fanRays < 0 || !( visOpts.fanSpacing > 0 ) || !( visOpts.faceSpacing > 0 ) ||
			 visOpts.bins < 1 || visOpts.binLevels < 1 )
		{
			Warning( "bspwalk: -eyespacing, -eyemaxsamples, -fanspacing and -facespacing must be positive, -fanrays >= 0, -eyebins and -eyebinlevels >= 1\n" );
			return 1;
		}
		if ( visOpts.faceSpacing > 4 )
			Warning( "bspwalk: -facespacing %g is above 4: face samples that far apart can miss a visible sliver, so tier B may call visible faces hidden\n",
					 visOpts.faceSpacing );
		for ( int i = 1; i < argc; i++ )
		{
			if ( Q_stricmp( argv[i], "-viewpoint" ) )
				continue;
			dvec3 p;
			if ( i + 3 >= argc || sscanf( argv[i + 1], "%lf", &p.x ) != 1 || sscanf( argv[i + 2], "%lf", &p.y ) != 1 ||
				 sscanf( argv[i + 3], "%lf", &p.z ) != 1 )
			{
				Warning( "bspwalk: -viewpoint needs three numbers: -viewpoint <x> <y> <z>\n" );
				return 1;
			}
			eyeOpts.viewpoints.push_back( p );
			i += 3;
		}
		t0 = Plat_FloatTime();
		BuildEyeVolume( world, profile, graph, eyeOpts, eyes );
		t.eyes = Plat_FloatTime() - t0;
		t0 = Plat_FloatTime();
		RunVisibility( world, profile, graph, eyes, visOpts, vis );
		t.vis = Plat_FloatTime() - t0;
	}

	if ( pszObj )
	{
		t0 = Plat_FloatTime();
		if ( !DumpWorldObj( world, pszObj ) )
			exitCode = 1;
		if ( bFlooded && !DumpReachObj( world, graph, pszObj ) )
			exitCode = 1;
		if ( bVis && !DumpEyesObj( eyes, pszObj ) )
			exitCode = 1;
		t.dumpobj = Plat_FloatTime() - t0;
	}

	int propMismatches = -1;
	if ( CommandLine()->FindParm( "-checkprops" ) )
		propMismatches = CheckPropHulls();

	t.total = Plat_FloatTime() - tStart;
	s_bReachSlim = CommandLine()->FindParm( "-reachslim" ) != 0;

	//
	// Summary
	//
	const CollectStats &s = world.stats;
	Msg( "\nconvexes by category:\n" );
	for ( int c = 0; c < CAT_COUNT; c++ )
		Msg( "  %-18s %7d%s\n", CategoryName( c ), world.CountCategory( c ),
			 CategoryIsGrown( c ) ? ( CategoryIsObstacle( c ) ? "" : "  (grown, standable only)" ) : "  (not grown)" );
	Msg( "  grown: %d stand, %d crouch; %d prop models\n", CountGrown( world.stand ), CountGrown( world.crouch ), Props_ModelCount() );
	Msg( "skipped: brushes wrong contents %d, brush build failed %d; brush entities: non-solid class %d, passable %d, "
		 "never solid %d, no solid brush %d\n",
		 s.brushesWrongContents, s.brushesBuildFailed, s.brushEntitiesNonSolidClass, s.brushEntitiesPassable,
		 s.brushEntitiesNeverSolid, s.brushEntitiesNoSolidBrush );
	Msg( "skipped: static props non-solid %d, no collision %d; entity props non-solid %d, no collision %d; "
		 "disps no hull collision %d, degenerate disp tris %d, bad prop convexes %d\n",
		 s.staticPropsNonSolid, s.staticPropsNoCollision, s.entityPropsNonSolid, s.entityPropsNoCollision,
		 s.dispsNoHullCollision, s.dispTrisDegenerate, s.convexesBuildFailed );
	if ( s.propsHullFallback )
		Msg( "props without a usable .phy, kept as their .mdl hull box: %d instance(s)\n", s.propsHullFallback );

	dvec3 wmin, wmax;
	if ( WorldAABB( world, CAT_WORLD, wmin, wmax ) )
		Msg( "world brush AABB [%.2f %.2f %.2f]-[%.2f %.2f %.2f], model 0 [%.2f %.2f %.2f]-[%.2f %.2f %.2f]\n",
			 wmin.x, wmin.y, wmin.z, wmax.x, wmax.y, wmax.z,
			 dmodels[0].mins.x, dmodels[0].mins.y, dmodels[0].mins.z, dmodels[0].maxs.x, dmodels[0].maxs.y, dmodels[0].maxs.z );
	else
		Warning( "bspwalk: no player-solid world brushes (model 0 is empty or entirely non-solid)\n" );
	if ( bWalk )
	{
		const WalkStats &ws = graph.stats;
		const ArcStats &as = graph.arcs;
		Msg( "walk: %d up faces -> %d nodes (%d slides, %.0f u^2); edges walk %d, jump %d, drop %d, arc %d, slide %d; %d "
			 "seed(s) -> %d nodes reached (%.0f u^2); %d mover entities, %d poses%s\n",
			 ws.upFaces, (int)graph.nodes.size(), ws.slideNodes, ws.totalArea, ws.edgeCounts[EDGE_WALK], ws.edgeCounts[EDGE_JUMP],
			 ws.edgeCounts[EDGE_DROP], ws.edgeCounts[EDGE_ARC], ws.edgeCounts[EDGE_SLIDE], (int)graph.seeds.size(),
			 ws.reachedCount, ws.reachedArea, ws.moverEntities, ws.moverPoses,
			 ws.moverGrowFailed ? " (some mover poses failed to grow)" : "" );
		const SpecialStats &ss = graph.special.stats;
		Msg( "special reached:" );
		for ( int k = 0; k < SPECIAL_KIND_COUNT; k++ )
			Msg( " %s %d/%d", SpecialKindName( k ), ss.reached[k], ss.count[k] );
		int trig = 0;
		for ( size_t i = 0; i < graph.special.teleports.size(); i++ )
		{
			int v = graph.special.teleports[i].vertex;
			trig += v >= 0 && v < (int)graph.special.reached.size() && graph.special.reached[v];
		}
		Msg( "; teleports triggered %d of %d; vehicle seeds %d%s; launches %lld pairs, %lld arcs, %lld joined (%.2f s); "
			 "trigger targets %d: %lld pairs, %lld arcs, %lld joined (%.2f s)\n",
			 trig, (int)graph.special.teleports.size(), ss.vehicleSeeds, graph.vehicleFree ? " (-vehicle-free)" : "",
			 ss.launchPairs, ss.launchArcs, ss.launchJoined, ss.tLaunch, (int)graph.special.targets.size(), ss.targetPairs,
			 ss.targetArcs, ss.targetJoined, ss.tTargets );
		Msg( "arcs (%s, R %.1fu to %.1fu, launch from %s, %d rim nodes, %d slope launchers): %d round(s), %lld pairs, %lld "
			 "arcs traced, %lld free, %lld pairs joined\n", as.path, as.reach, as.reachMax,
			 as.allNodes ? "all nodes" : "rim and slope nodes", as.rimNodes, as.slopeNodes, as.rounds, as.pairs, as.arcs, as.arcsFree, as.pairsFree );
		Msg( "walk timings (s): polygons %.3f, stand %.3f, edges %.3f, flood %.3f (special %.3f), arcs %.3f (build %.3f, gen %.3f, "
			 "trace %.3f, special %.3f), total %.3f\n", ws.tPolys, ws.tStand, ws.tEdges, ws.tFlood, ws.tSpecial, as.tTotal,
			 as.tBuild, as.tGen, as.tTrace, ss.tLaunch, t.walk );
	}
	Msg( "timings (s): init %.3f, load %.3f, brushes %.3f, props %.3f, disps %.3f, grow %.3f, verify %.3f, walk %.3f, "
		 "eyes %.3f, vis %.3f, dumpobj %.3f, total %.3f\n",
		 t.init, t.load, t.brushes, t.props, t.disps, t.grow, t.verify, t.walk, t.eyes, t.vis, t.dumpobj, t.total );

	const char *pszOut = CommandLine()->ParmValue( "-out", (const char *)NULL );
	if ( pszOut && bWalk && !bFlooded )
		Warning( "bspwalk: %s not written: no flood result\n", pszOut );
	else if ( pszOut )
	{
		if ( WriteJson( pszOut, pszBsp, world, profile, t, verifyFailures, propMismatches, bWalk ? &graph : NULL,
						bVis ? &eyes : NULL, bVis ? &vis : NULL, visOpts ) )
			Msg( "wrote %s\n", pszOut );
		else
		{
			Warning( "bspwalk: cannot write %s\n", pszOut );
			exitCode = 1;
		}
	}
	return exitCode;
}
