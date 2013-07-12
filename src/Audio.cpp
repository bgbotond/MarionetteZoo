#include <assert.h>

#include "Audio.h"

using namespace ci;
using namespace std;

Audio::~Audio()
{
	stopAll();

	mTracks.clear();
}

void Audio::setup( const fs::path &audioFolder )
{
	_loadTracks( audioFolder );
}

void Audio::play( string name, float volume /* = 1.0f */, bool loop /* = false */ )
{
	stop( name );

	unsigned track = _findTrack( name );

	if( ! track )
		return;

	mAudio.play( track, volume, loop );
}

void Audio::stop( string name )
{
	unsigned track = _findTrack( name );

	if( ! track )
		return;

	mAudio.stop( track );
}

void Audio::stopAll()
{
	for( map< string, unsigned >::iterator it = mTracks.begin(); it != mTracks.end(); ++it )
	{
		mAudio.stop( it->second );
	}
}

unsigned Audio::_findTrack( string name )
{
	std::map< string, unsigned >::iterator it = mTracks.lower_bound( name );

	if( it != mTracks.end() && !( mTracks.key_comp()( name, it->first )))
	{
		return it->second;
	}

	return 0;
}

void Audio::_loadTracks( const fs::path folder )
{
	for( fs::directory_iterator it( folder ); it != fs::directory_iterator(); ++it )
	{
		if( fs::is_regular_file( *it ))
		{
			string name = folder.string() + "/" + it->path().filename().string();

			if( it->path().extension().string() != ".ogg" )
				continue;

			unsigned track = mAudio.load( name );
			mTracks.insert( make_pair( it->path().stem().string(), track ));
		}
	}
}
