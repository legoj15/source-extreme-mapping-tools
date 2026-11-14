//========= bspwalk: native BSP walker (docs/bsp_walker_rebuild.md) ==========//
//
// Purpose: The tier B eye store (#39): see bw_eyestore.h.
//
//=============================================================================//

#include "cmdlib.h"
#include "tier0/platform.h"
#include "bw_eyestore.h"
#include "bw_hash.h"

#include <algorithm>
#include <float.h>

// Eviction stops once the resident bytes are down to this share of the budget, so
// a cache at its budget does not scan every tile for each one it makes.
static const double EYE_CACHE_LOW_WATER = 0.75;

static std::atomic<unsigned long long> s_StoreIds( 0 );

size_t EyeTileData::Bytes() const
{
	return sizeof( *this ) + xyz.capacity() * sizeof( float ) + source.capacity() + groupCl.capacity() * sizeof( int ) +
		   groupOfs.capacity() * sizeof( int );
}

int EyeTileData::ClusterOf( int i ) const
{
	// the last group starting at or before i
	int g = (int)( std::upper_bound( groupOfs.begin(), groupOfs.end() - 1, i ) - groupOfs.begin() ) - 1;
	return g >= 0 && g < (int)groupCl.size() ? groupCl[g] : -1;
}

EyeStore::EyeStore( EyeStoreSetup &setup )
	: m_numTiles( setup.numTiles ), m_numClusters( setup.numClusters ), m_budget( setup.budgetBytes ), m_gen( setup.gen ),
	  m_ctx( setup.ctx ), m_bytes( 0 ), m_clock( 0 ), m_allComputed( false ), m_genMicros( 0 )
{
	if ( m_numTiles > EYE_STORE_MAX_TILES )
		Error( "bspwalk: %d eye tiles, more than the %d an eye id holds; raise -eyespacing\n", m_numTiles, EYE_STORE_MAX_TILES );
	m_tileSky.swap( setup.tileSky );
	m_clTileOfs.swap( setup.clTileOfs );
	m_clTiles.swap( setup.clTiles );
	m_pieceClusters.swap( setup.pieceClusters );
	m_partBound.swap( setup.partBound );
	m_tileSky.resize( m_numTiles, -1 );
	m_clTileOfs.resize( m_numClusters + 1, m_clTileOfs.empty() ? 0 : m_clTileOfs.back() );
	m_pieceClusters.resize( m_numClusters, 0 );
	m_partBound.resize( m_numClusters );
	m_slots.reset( new Slot[m_numTiles > 0 ? m_numTiles : 1] );
	for ( int i = 0; i < m_numTiles; i++ )
	{
		m_slots[i].word.store( 0 );
		m_slots[i].data = NULL;
		m_slots[i].lastUse.store( 0 );
		m_slots[i].generated.store( false );
	}
	m_tileCount.assign( m_numTiles, 0 );
	m_tileBox.assign( (size_t)m_numTiles * 6, 0.0f );
	m_facts.reset( new EyeClusterFacts[m_numClusters > 0 ? m_numClusters : 1] );
	memset( &m_stats, 0, sizeof( m_stats ) );
	m_id = ++s_StoreIds;
}

EyeStore::~EyeStore()
{
	for ( int i = 0; i < m_numTiles; i++ )
		delete m_slots[i].data;
}

bool EyeStore::TileInCluster( int tile, int c ) const
{
	if ( c < 0 || c >= m_numClusters )
		return false;
	const int *a = m_clTiles.data() + m_clTileOfs[c], *b = m_clTiles.data() + m_clTileOfs[c + 1];
	return std::binary_search( a, b, tile );
}

const int *EyeStore::ClusterTiles( int c, int &n ) const
{
	n = m_clTileOfs[c + 1] - m_clTileOfs[c];
	return m_clTiles.data() + m_clTileOfs[c];
}

// The groups' facts and the tile's box, from a generated tile.
void EyeStore::TileGroupFacts( const EyeTileData &d, std::vector<GroupFacts> &out, float box[6] )
{
	box[0] = box[1] = box[2] = FLT_MAX;
	box[3] = box[4] = box[5] = -FLT_MAX;
	out.resize( d.groupCl.size() );
	for ( size_t g = 0; g < d.groupCl.size(); g++ )
	{
		GroupFacts &f = out[g];
		f.cluster = d.groupCl[g];
		f.start = d.groupOfs[g];
		f.count = d.groupOfs[g + 1] - d.groupOfs[g];
		f.box[0] = f.box[1] = f.box[2] = FLT_MAX;
		f.box[3] = f.box[4] = f.box[5] = -FLT_MAX;
		f.bound = EyeBound();
		for ( int i = f.start; i < f.start + f.count; i++ )
		{
			const float *p = &d.xyz[(size_t)i * 3];
			for ( int a = 0; a < 3; a++ )
			{
				f.box[a] = MIN( f.box[a], p[a] );
				f.box[3 + a] = MAX( f.box[3 + a], p[a] );
			}
			f.bound.Add( dvec3( p[0], p[1], p[2] ) );
		}
		for ( int a = 0; a < 3; a++ )
		{
			box[a] = MIN( box[a], f.box[a] );
			box[3 + a] = MAX( box[3 + a], f.box[3 + a] );
		}
	}
}

const EyeTileData &EyeStore::Pin( int tile )
{
	Slot &s = m_slots[tile];
	for ( ;; )
	{
		unsigned int w = s.word.load();
		unsigned int st = w >> ST_SHIFT;
		if ( st == ST_RESIDENT )
		{
			if ( s.word.compare_exchange_weak( w, w + 1 ) )
			{
				s.lastUse.store( m_clock.fetch_add( 1, std::memory_order_relaxed ) + 1, std::memory_order_relaxed );
				return *s.data;
			}
			continue;
		}
		if ( st == ST_ABSENT )
		{
			// this thread makes it: the others asking wait for it (below)
			if ( !s.word.compare_exchange_strong( w, ( (unsigned int)ST_GENERATING << ST_SHIFT ) | 1 ) )
				continue;
			EyeTileData *d = new EyeTileData;
			double t0 = Plat_FloatTime();
			m_gen( m_ctx, tile, *d );
			// only this thread makes the tile now, so 'generated' cannot change under it
			const bool first = !s.generated.load();
			float box[6];
			if ( first )
				TileGroupFacts( *d, s.groups, box );
			m_genMicros.fetch_add( (long long)( ( Plat_FloatTime() - t0 ) * 1e6 ), std::memory_order_relaxed );
			std::vector<EyeTileData *> freed;
			{
				std::lock_guard<std::mutex> lock( m_mutex );
				s.data = d;
				m_bytes += (double)d->Bytes();
				if ( first )
				{
					m_stats.tilesGenerated++;
					m_stats.samples += d->Count();
					m_stats.made += d->made;
					m_stats.offTile += d->offTile;
					m_stats.unrecorded += d->unrecorded;
					m_stats.unplaced += d->unplaced;
					m_tileCount[tile] = d->Count();
					memcpy( &m_tileBox[(size_t)tile * 6], box, sizeof( box ) );
					s.generated.store( true );
				}
				else
					m_stats.regenerations++;
				m_stats.peakBytes = MAX( m_stats.peakBytes, m_bytes );
				s.lastUse.store( m_clock.fetch_add( 1, std::memory_order_relaxed ) + 1, std::memory_order_relaxed );
				s.word.store( ( (unsigned int)ST_RESIDENT << ST_SHIFT ) | 1 );
				if ( m_bytes > m_budget )
					EvictLocked( freed );
			}
			m_cv.notify_all();
			for ( size_t i = 0; i < freed.size(); i++ )
				delete freed[i];
			return *d;
		}
		// another thread is making it
		std::unique_lock<std::mutex> lock( m_mutex );
		m_cv.wait( lock, [&s]() { return ( s.word.load() >> ST_SHIFT ) != ST_GENERATING; } );
	}
}

void EyeStore::Unpin( int tile )
{
	m_slots[tile].word.fetch_sub( 1 );
}

// Under m_mutex: the unpinned resident tiles, least recently pinned first, until the
// bytes are down to the low-water mark. A tile pinned meanwhile keeps its place (its
// word no longer matches).
void EyeStore::EvictLocked( std::vector<EyeTileData *> &freed )
{
	const unsigned int idle = (unsigned int)ST_RESIDENT << ST_SHIFT;
	std::vector< std::pair<long long, int> > lru;
	for ( int i = 0; i < m_numTiles; i++ )
	{
		if ( m_slots[i].word.load() == idle )
			lru.push_back( std::make_pair( m_slots[i].lastUse.load( std::memory_order_relaxed ), i ) );
	}
	std::sort( lru.begin(), lru.end() );
	const double low = m_budget * EYE_CACHE_LOW_WATER;
	for ( size_t k = 0; k < lru.size() && m_bytes > low; k++ )
	{
		Slot &s = m_slots[lru[k].second];
		unsigned int w = idle;
		if ( !s.word.compare_exchange_strong( w, (unsigned int)ST_ABSENT << ST_SHIFT ) )
			continue;
		m_bytes -= (double)s.data->Bytes();
		freed.push_back( s.data );
		s.data = NULL;
	}
}

static EyeStore *s_pBulkStore;

void EyeStore::WarmOne( int, int tile )
{
	if ( s_pBulkStore->m_slots[tile].generated.load() )
		return;
	s_pBulkStore->Pin( tile );
	s_pBulkStore->Unpin( tile );
}

void EyeStore::ComputeAll()
{
	if ( m_allComputed )
		return;
	s_pBulkStore = this;
	RunThreadsBatched( m_numTiles, WarmOne );
	s_pBulkStore = NULL;
	for ( int c = 0; c < m_numClusters; c++ )
	{
		EyeClusterFacts &f = m_facts[c];
		f.count = 0;
		f.box[0] = f.box[1] = f.box[2] = FLT_MAX;
		f.box[3] = f.box[4] = f.box[5] = -FLT_MAX;
		f.bound = EyeBound();
		f.runs.clear();
	}
	// tile by tile in tile order, so each cluster's runs are in tile order
	for ( int tile = 0; tile < m_numTiles; tile++ )
	{
		const std::vector<GroupFacts> &gs = m_slots[tile].groups;
		for ( size_t k = 0; k < gs.size(); k++ )
		{
			const GroupFacts &g = gs[k];
			if ( g.cluster < 0 || g.cluster >= m_numClusters )
				continue;
			EyeClusterFacts &f = m_facts[g.cluster];
			EyeRun r = { tile, g.start, g.count };
			f.runs.push_back( r );
			f.count += r.count;
			for ( int a = 0; a < 3; a++ )
			{
				f.box[a] = MIN( f.box[a], g.box[a] );
				f.box[3 + a] = MAX( f.box[3 + a], g.box[3 + a] );
			}
			f.bound.Merge( g.bound );
		}
	}
	// EyeVolume::region's rule (#39): a piece cluster's region is the bound of its piece parts
	// and of its samples (unbounded when no part was recorded); a cluster with samples and no
	// piece (none can have any: a sample takes only an eye cluster, which pieces touched) would
	// be unbounded too; any other cluster's is the point 0.
	m_bareClusters.clear();
	m_bareRegions.clear();
	for ( int c = 0; c < m_numClusters; c++ )
	{
		EyeClusterFacts &f = m_facts[c];
		f.region = EyeBound();
		if ( m_pieceClusters[c] )
		{
			f.region = m_partBound[c];
			if ( !f.region.any )
				f.region.Unbounded();
			else
				f.region.Merge( f.bound );
		}
		else if ( f.count )
			f.region.Unbounded();
		else
			f.region.AtOrigin();
		f.bare = m_pieceClusters[c] && !f.count;
		if ( f.bare )
		{
			m_bareClusters.push_back( c );
			m_bareRegions.push_back( f.region );
		}
	}
	m_allComputed = true;
}

const EyeClusterFacts &EyeStore::Facts( int c ) const
{
	if ( !m_allComputed )
		Error( "bspwalk: eye cluster facts read before every eye tile was made (EyeStore::ComputeAll)\n" );
	return m_facts[c];
}

EyeStoreStats EyeStore::Stats() const
{
	std::lock_guard<std::mutex> lock( m_mutex );
	EyeStoreStats s = m_stats;
	s.genSeconds = (double)m_genMicros.load() * 1e-6;
	return s;
}

const EyeTileData &EyePinSet::Get( EyeStore &store, int tile )
{
	if ( m_store != &store || m_storeId != store.Id() )
	{
		Release();
		m_store = &store;
		m_storeId = store.Id();
		m_mark.assign( store.NumTiles(), 0 );
		m_epoch = 1;
	}
	if ( m_mark[tile] != m_epoch )
	{
		store.Pin( tile );
		m_mark[tile] = m_epoch;
		m_tiles.push_back( tile );
	}
	return store.Pinned( tile );
}

void EyePinSet::Release()
{
	if ( m_tiles.empty() )
		return;
	for ( size_t i = 0; i < m_tiles.size(); i++ )
		m_store->Unpin( m_tiles[i] );
	m_tiles.clear();
	if ( ++m_epoch == 0 )
	{
		std::fill( m_mark.begin(), m_mark.end(), 0u );
		m_epoch = 1;
	}
}
