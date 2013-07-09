#pragma once

#include <string>
#include <map>
#include <cinder/Filesystem.h>

namespace btd
{
	class ModelFileManager
	{
		struct ModelFileInfo
		{
			std::string   mType;
			ci::fs::path  mModelFile;
			ci::fs::path  mBulletFile;
			ci::fs::path  mActionFile;
		};

		typedef std::map< std::string, ModelFileInfo > ModelFileInfos;

		// singleton
		ModelFileManager();

	public:
		~ModelFileManager();

		static ModelFileManager& getSingleton();

		void registerModel( const std::string& type, const ci::fs::path& modelFile, const ci::fs::path& bulletFile, const ci::fs::path& actionFile );

		bool hasModel( const std::string& type );
		std::vector< std::string > getModelTypes();

		ci::fs::path getModelFile( const std::string& type );
		ci::fs::path getBulletFile( const std::string& type );
		ci::fs::path getActionFile( const std::string& type );

	protected:
		ModelFileInfos::iterator getModelFileInfo( const std::string& type );

	protected:
		ModelFileInfos mModelFileInfos;
	};

} // namespace btd
