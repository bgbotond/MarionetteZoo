#pragma once

#include <memory>
#include <vector>

#include "cinder/Vector.h"

#include "Model.h"

namespace btd
{
	typedef std::shared_ptr< class ModelManager > ModelManagerRef;

	class ModelManager
	{
	public:
		typedef std::map< std::string, ModelRef > Models;

	public:
		ModelManager( BulletWorldRef& bulletWorld );
		~ModelManager();

		ModelRef createModel( const std::string& name, const ci::Vec3f& worldOffset, const std::string& type );
		void     destroyModel( const std::string& name );
		ModelRef getModel( const std::string& name );

		// TODO how update gets the current hand information?
		void update();
		void draw();

	protected:
		Models::iterator findModel( const std::string& name );

	protected:
		BulletWorldRef mBulletWorld;
		Models         mModels;
	};

} // namespace btd
