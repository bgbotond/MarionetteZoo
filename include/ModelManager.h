#pragma once

#include <memory>
#include <vector>

#include "cinder/Vector.h"
#include "cinder/Camera.h"

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
		void     destroyModelAll();
		ModelRef getModel( const std::string& name );

		// TODO how update gets the current hand information?
		void update();
		void draw( const ci::CameraPersp &camera );

	protected:
		Models::iterator findModel( const std::string& name );

	protected:
		BulletWorldRef mBulletWorld;
		Models         mModels;
	};

} // namespace btd
