//========= bspwalk: native BSP walker (docs/bsp_walker_rebuild.md) ==========//
//
// Purpose: The tier B eye store (#39). The eye samples are not kept as one
//          list: they are made one XY tile at a time (bw_eyes.cpp), on demand,
//          and held in a cache with a byte budget. A tile can be made again
//          any number of times and always gives the same samples, so what
//          bw_vis reads never depends on what the cache happened to hold.
//
//          A tile id is the tile's index in the canonical order (layer: real,
//          then sky camera 0, 1, ...; tile x; tile y). An eye id is
//          tile << 32 | the sample's index in its tile. Within a tile the
//          samples are grouped by cluster (ascending, -1 first), each group in
//          the tile's own (x, y, z) order, so a cluster's samples read tile by
//          tile in tile order are in the order the eager list had them.
//
//=============================================================================//

#ifndef BW_EYESTORE_H
#define BW_EYESTORE_H
#pragma once

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <vector>
#include "bw_vis.h"

// One generated tile.
struct EyeTileData
{
	std::vector<float> xyz;				// 3 per sample
	std::vector<unsigned char> source;	// EyeSource per sample
	std::vector<int> groupCl;			// per group: its cluster, ascending (-1 first)
	std::vector<int> groupOfs;			// per group: its first sample; groupCl.size() + 1 entries
	long long made;						// generator: samples made before the dedupe
	long long unrecorded;				// generator: samples given the cluster of their unrounded position
	long long unplaced;					// generator: samples in no eye cluster, rounded or not (group -1)

	EyeTileData() : made( 0 ), unrecorded( 0 ), unplaced( 0 ) {}
	int Count() const { return (int)source.size(); }
	size_t Bytes() const;
	// the cluster of sample i
	int ClusterOf( int i ) const;
};

// Fills 'out' with tile 'tile' (out arrives empty). Called from any thread, for
// different tiles at once; never under a lock of the store.
typedef void ( *EyeTileGenFn )( void *ctx, int tile, EyeTileData &out );

// A run of one cluster's samples in one tile: samples [start, start + count) of the tile.
struct EyeRun
{
	int tile, start, count;
};

// What a cluster's samples give, once computed.
struct EyeClusterFacts
{
	long long count;				// samples
	float box[6];					// their float mins xyz, maxs xyz (FLT_MAX / -FLT_MAX: none)
	EyeBound bound;					// their k-DOP bound
	std::vector<EyeRun> runs;		// tile by tile in tile order
	EyeBound region;				// the piece parts' bound merged with 'bound' (EyeVolume::region's rule)
	bool bare;						// a piece cluster with no sample
};

struct EyeStoreSetup
{
	int numTiles;
	std::vector<int> tileSky;					// per tile: -1 real, k sky camera k
	int numClusters;
	// per cluster: the sorted tile ids (CSR) that can hold one of its samples: every sample the
	// generator gives cluster c lies in a tile of c's list (the generator checks it for every
	// sample; bw_eyes.cpp, TileSampleCluster)
	std::vector<int> clTileOfs, clTiles;
	std::vector<unsigned char> pieceClusters;	// per cluster: the eye pieces touch it
	std::vector<EyeBound> partBound;			// per cluster: the bound of the piece parts in it
	double budgetBytes;							// the cache's target (pinned tiles may pass it)
	EyeTileGenFn gen;
	void *ctx;

	EyeStoreSetup() : numTiles( 0 ), numClusters( 0 ), budgetBytes( 0 ), gen( NULL ), ctx( NULL ) {}
};

// The eye ids fit a JSON number (a double) exactly: tile < 2^21, index < 2^32.
#define EYE_STORE_MAX_TILES		( 1 << 21 )

inline long long EyeId( int tile, int index )
{
	return ( (long long)tile << 32 ) | (unsigned int)index;
}
inline int EyeIdTile( long long id )
{
	return (int)( id >> 32 );
}
inline int EyeIdIndex( long long id )
{
	return (int)( id & 0xffffffffLL );
}

struct EyeStoreStats
{
	long long samples;			// samples in all, each tile counted once (its first generation)
	long long made;				// samples made before the dedupe, each tile once
	long long unrecorded;		// EyeTileData::unrecorded, each tile once
	long long unplaced;			// EyeTileData::unplaced, each tile once
	int tilesGenerated;			// tiles generated at least once
	long long regenerations;	// generations past a tile's first (it was evicted and needed again)
	double peakBytes;			// most bytes resident at once
	double genSeconds;			// time inside the generator, summed over threads
};

class EyeStore
{
public:
	explicit EyeStore( EyeStoreSetup &setup );	// takes the setup's vectors
	~EyeStore();

	int NumTiles() const { return m_numTiles; }
	int NumClusters() const { return m_numClusters; }
	int TileSky( int tile ) const { return m_tileSky[tile]; }
	bool TileInCluster( int tile, int c ) const;
	const int *ClusterTiles( int c, int &n ) const;
	bool PieceCluster( int c ) const { return m_pieceClusters[c] != 0; }

	// A pinned tile is never evicted; Pin waits while another thread generates the
	// tile, and generates it itself when it is absent. Any thread.
	const EyeTileData &Pin( int tile );
	void Unpin( int tile );
	// the data of a tile the caller holds pinned
	const EyeTileData &Pinned( int tile ) const { return *m_slots[tile].data; }

	// Facts of the listed clusters (#39): the tiles of their lists not made yet are made once (in
	// parallel), then each cluster's facts are merged from the group facts of the tiles in its
	// list, in tile order. A cluster's samples lie only in tiles of its list (EyeStoreSetup::clTiles),
	// so no other tile is made. A cluster computed already is left as it is. Main thread only
	// (RunThreadsBatched). If a tile turned out to hold a sample of a cluster whose list lacks it
	// (NoteListMiss), the lists cannot be relied on: every tile is made and every cluster's facts
	// are merged from every tile, as ComputeAll does. Slower, never a missed eye.
	void ComputeFor( const std::vector<int> &clusters );
	// The generator found a sample of a cluster in a tile the cluster's list lacks. True the first
	// time (the caller warns once). Any thread.
	bool NoteListMiss() { return !m_listMiss.exchange( true ); }
	bool ListMiss() const { return m_listMiss.load(); }
	// Every tile made once (in parallel) and every cluster's facts merged from every tile's group
	// facts, in tile order: what a face pass without vis data reads (its candidate buckets are
	// the tiles), and the reference the -selftest checks ComputeFor against. Main thread only.
	void ComputeAll();
	bool Computed( int c ) const { return c >= 0 && c < m_numClusters && m_computed[c] != 0; }
	// Piece clusters computed so far
	int NeededClusters() const { return m_neededClusters; }
	// Cluster c's facts: an Error unless it was computed (ComputeFor, ComputeAll). Any thread.
	const EyeClusterFacts &Facts( int c ) const;
	// The computed piece clusters with no sample, ascending, and their regions.
	const std::vector<int> &BareClusters() const { return m_bareClusters; }
	const std::vector<EyeBound> &BareRegions() const { return m_bareRegions; }
	// Recorded at a tile's first generation (0 and no box for a tile not made yet).
	int TileCount( int tile ) const { return m_tileCount[tile]; }
	const float *TileBox( int tile ) const { return &m_tileBox[(size_t)tile * 6]; }

	EyeStoreStats Stats() const;
	double BudgetBytes() const { return m_budget; }
	unsigned long long Id() const { return m_id; }

private:
	enum
	{
		ST_ABSENT = 0,
		ST_GENERATING = 1,
		ST_RESIDENT = 2,
		ST_SHIFT = 28,
		PIN_MASK = ( 1 << ST_SHIFT ) - 1
	};
	// One cluster group of a tile, recorded at the tile's first generation: the cluster
	// facts are merged from these, so they never need a tile made again (the bound's
	// Merge is a per-direction max, so merging the groups' bounds gives what adding every
	// sample would).
	struct GroupFacts
	{
		int cluster, start, count;
		float box[6];
		EyeBound bound;
	};
	struct Slot
	{
		std::atomic<unsigned int> word;		// state << ST_SHIFT | pins
		EyeTileData *data;					// set while resident
		std::atomic<long long> lastUse;
		std::atomic<bool> generated;		// generated at least once (set under m_mutex, after 'groups')
		std::vector<GroupFacts> groups;		// recorded at the first generation
	};
	static void TileGroupFacts( const EyeTileData &d, std::vector<GroupFacts> &out, float box[6] );
	static void WarmOne( int iThread, int item );
	void EvictLocked( std::vector<EyeTileData *> &freed );
	// Makes the tiles (not made yet; ascending, no repeats) in parallel
	void MakeTiles( const std::vector<int> &tiles );
	void BeginFacts( int c );					// cluster c's facts empty
	void AddGroup( int c, int tile, const GroupFacts &g );
	void FinishFacts( const std::vector<int> &fresh );	// the regions and bare flags of the clusters just merged

	int m_numTiles, m_numClusters;
	std::vector<int> m_tileSky;
	std::vector<int> m_clTileOfs, m_clTiles;
	std::vector<unsigned char> m_pieceClusters;
	std::vector<EyeBound> m_partBound;
	double m_budget;
	EyeTileGenFn m_gen;
	void *m_ctx;

	std::unique_ptr<Slot[]> m_slots;
	mutable std::mutex m_mutex;
	std::condition_variable m_cv;
	double m_bytes;						// under m_mutex
	std::atomic<long long> m_clock;
	std::vector<int> m_tileCount;		// recorded at first generation (under m_mutex)
	std::vector<float> m_tileBox;

	std::unique_ptr<EyeClusterFacts[]> m_facts;
	std::vector<unsigned char> m_computed;	// per cluster: its facts are in m_facts
	int m_neededClusters;
	unsigned long long m_id;
	std::vector<int> m_bareClusters;
	std::vector<EyeBound> m_bareRegions;

	EyeStoreStats m_stats;				// under m_mutex
	std::atomic<long long> m_genMicros;
	std::atomic<bool> m_listMiss;		// NoteListMiss
};

// A thread's pins (#39): each tile pinned once however often it is asked for,
// all released together (Release, or the destructor). One per thread. Not copyable
// (a copy would release the same pins twice); a move hands the pins over.
class EyePinSet
{
public:
	EyePinSet() : m_store( NULL ), m_storeId( 0 ), m_epoch( 1 ) {}
	~EyePinSet() { Release(); }
	EyePinSet( const EyePinSet & ) = delete;
	EyePinSet &operator=( const EyePinSet & ) = delete;
	EyePinSet( EyePinSet &&o ) noexcept : m_store( NULL ), m_storeId( 0 ), m_epoch( 1 ) { Take( o ); }
	EyePinSet &operator=( EyePinSet &&o ) noexcept
	{
		if ( this != &o )
		{
			Release();
			Take( o );
		}
		return *this;
	}
	const EyeTileData &Get( EyeStore &store, int tile );
	void Release();

private:
	// this set (holding no pin) takes o's pins and marks; o is left as new
	void Take( EyePinSet &o )
	{
		m_store = o.m_store;
		m_storeId = o.m_storeId;
		m_epoch = o.m_epoch;
		m_tiles.swap( o.m_tiles );
		m_mark.swap( o.m_mark );
		o.m_store = NULL;
		o.m_storeId = 0;
		o.m_epoch = 1;
		o.m_tiles.clear();
		o.m_mark.clear();
	}

	EyeStore *m_store;
	unsigned long long m_storeId;
	std::vector<int> m_tiles;
	std::vector<unsigned int> m_mark;
	unsigned int m_epoch;
};

#endif // BW_EYESTORE_H
