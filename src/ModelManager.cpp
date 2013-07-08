#include <map>
#include "ModelManager.h"
#include "BulletWorld.h"

namespace btd
{
	ModelManager::ModelManager( BulletWorldRef& bulletWorld )
		: mBulletWorld( bulletWorld )
		, mModels()
	{
	}

	ModelManager::~ModelManager()
	{
	}

	ModelRef ModelManager::createModel( const std::string& name, const ci::Vec3f& worldOffset, const std::string& type )
	{
		Models::iterator it = findModel( name );

		if( it != mModels.end() )
			return it->second;

		ModelRef model = ModelRef( new Model( mBulletWorld, worldOffset, type ) );

		mModels.insert( std::make_pair( name, model ) );

		return model;
	}

	void ModelManager::destroyModel( const std::string& name )
	{
		Models::iterator it = findModel( name );

		if( it == mModels.end() )
			return;

		mModels.erase( it );
	}

	ModelRef ModelManager::getModel( const std::string& name )
	{
		Models::iterator it = findModel( name );

		if( it != mModels.end() )
			return it->second;

		return ModelRef();
	}

	void ModelManager::update()
	{
		// TODO how update gets the current hand information?
	}

	void ModelManager::draw()
	{
		for( auto it = mModels.begin(); it != mModels.end(); ++it )
		{
			it->second->draw();
		}
	}

	ModelManager::Models::iterator ModelManager::findModel( const std::string& name )
	{
		return mModels.find( name );
	}

} // namespace btd
