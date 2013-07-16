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
		registerModel( "softbody8", "softbody8_2055.dae", "softbody8_2055.bullet", "nothing.xml" );

		// load all models automatically from the models folder
		ci::fs::path dataPath = ci::app::getAssetPath( "models" );
		for ( ci::fs::directory_iterator it( dataPath ); it != ci::fs::directory_iterator(); ++it )
		{
			if ( ci::fs::is_regular_file( *it ) && ( it->path().extension().string() == ".bullet" ) )
			{
				std::string name = it->path().stem().string();
				ci::fs::path daeFile( it->path().filename() );
				daeFile.replace_extension( "dae" );
				ci::fs::path bulletFile( it->path().filename() );

				ci::app::console() << "registering model " << name << " " << daeFile << " " << bulletFile << std::endl;
				registerModel( name, dataPath / daeFile, dataPath / bulletFile, "nothing.xml" );
			}
		}
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

		ci::fs::path path = getModelFileInfo( type )->second.mModelFile;
		if ( path.is_absolute() )
			return path;
		else
			return ci::app::getAssetPath( getModelFileInfo( type )->second.mModelFile );
	}

	ci::fs::path ModelFileManager::getBulletFile( const std::string& type )
	{
		if( ! hasModel( type ) )
			return ci::fs::path();

		ci::fs::path path = getModelFileInfo( type )->second.mBulletFile;
		if ( path.is_absolute() )
			return path;
		else
			return ci::app::getAssetPath( getModelFileInfo( type )->second.mBulletFile );
	}

	ci::fs::path ModelFileManager::getActionFile( const std::string& type )
	{
		if( ! hasModel( type ) )
			return ci::fs::path();

		ci::fs::path path = getModelFileInfo( type )->second.mActionFile;
		if ( path.is_absolute() )
			return path;
		else
			return ci::app::getAssetPath( getModelFileInfo( type )->second.mActionFile );
	}

	ModelFileManager::ModelFileInfos::iterator ModelFileManager::getModelFileInfo( const std::string& type )
	{
		return mModelFileInfos.find( type );
	}

} // namespace btd
