#pragma once

#include <map>
#include "cinder/Cinder.h"
#include "cinder/Filesystem.h"
#include "OpenALAudio.h"

class Audio
{
	public:
		~Audio();
		void setup( const ci::fs::path &audioFolder );

		void  play( std::string name, float volume = 1.0f, bool loop = false );
		void  stop( std::string name );
		void  stopAll();

	private:
		unsigned _findTrack( std::string name );
		void     _loadTracks( const ci::fs::path folder );

	private:
		mndl::openal::OpenALAudio         mAudio;
		std::map< std::string, unsigned > mTracks;
};

