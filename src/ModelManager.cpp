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

		mModels[ name ] = model;

		return model;
	}

	void ModelManager::destroyModel( const std::string& name )
	{
		Models::iterator it = findModel( name );

		if( it == mModels.end() )
			return;

		mModels.erase( it );
	}

	void ModelManager::destroyModelAll()
	{
		mModels.clear();
	}

	ModelRef ModelManager::getModel( const std::string& name )
	{
		Models::iterator it = findModel( name );

		if( it != mModels.end() )
			return it->second;

		return ModelRef();
	}
	
	// TODO this should be deleted and managed by the controller who connects the hand and ModelRef.
	void ModelManager::update()
	{
		for( auto it = mModels.begin(); it != mModels.end(); ++it )
		{
			it->second->update( ci::Vec3f::zero(), ci::Vec3f::zero(), ci::Vec3f::zero() );
		}
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
