#include "cinder/app/App.h"
#include "ModelFileManager.h"

namespace btd
{
	ModelFileManager::ModelFileManager()
	{
//		registerModel( "bird", "bird.dae", "bird.bullet", "bird.xml" );
		registerModel( "test_softbody"     , "softbody8_1kotel.dae"  , "softbody8_1kotel.bullet"  , "nothing.xml" );
		registerModel( "test_constraint"   , "testconstraint.dae"    , "testconstraint.bullet"    , "nothing.xml" );
		registerModel( "test_softbody_bone", "softbody10_csontos.dae", "softbody10_csontos.bullet", "nothing.xml" );
		registerModel( "simple_test", "simple_test2.dae", "simple_test2.bullet", "nothing.xml" );

		// here we can add more models
	}

	ModelFileManager::~ModelFileManager()
	{
	}

	ModelFileManager& ModelFileManager::getSingleton()
	{
		static ModelFileManager modelFileManager;

		return modelFileManager;
	}

	void ModelFileManager::registerModel( const std::string& type, const ci::fs::path& modelFile, const ci::fs::path& bulletFile, const ci::fs::path& actionFile )
	{
		if( hasModel( type ) )
			return;

		ModelFileInfo modelFileInfo;
		
		modelFileInfo.mType       = type;
		modelFileInfo.mModelFile  = modelFile;
		modelFileInfo.mBulletFile = bulletFile;
		modelFileInfo.mActionFile = actionFile;

		mModelFileInfos[ type ] = modelFileInfo;
	}

	bool ModelFileManager::hasModel( const std::string& type )
	{
		ModelFileInfos::iterator it = getModelFileInfo( type );

		if( it != mModelFileInfos.end() )
			return true;

		return false;
	}

	std::vector< std::string > ModelFileManager::getModelTypes()
	{
		std::vector< std::string > modelTypes;

		for( auto it = mModelFileInfos.begin(); it != mModelFileInfos.end(); ++it )
		{
			modelTypes.push_back( it->first );
		}

		return modelTypes;
	}

	ci::fs::path ModelFileManager::getModelFile( const std::string& type )
	{
		if( ! hasModel( type ) )
			return ci::fs::path();

		return ci::app::App::get()->getAssetPath( getModelFileInfo( type )->second.mModelFile );
	}

	ci::fs::path ModelFileManager::getBulletFile( const std::string& type )
	{
		if( ! hasModel( type ) )
			return ci::fs::path();

		return ci::app::App::get()->getAssetPath( getModelFileInfo( type )->second.mBulletFile );
	}

	ci::fs::path ModelFileManager::getActionFile( const std::string& type )
	{
		if( ! hasModel( type ) )
			return ci::fs::path();

		return ci::app::App::get()->getAssetPath( getModelFileInfo( type )->second.mActionFile );
	}

	ModelFileManager::ModelFileInfos::iterator ModelFileManager::getModelFileInfo( const std::string& type )
	{
		return mModelFileInfos.find( type );
	}

} // namespace btd
