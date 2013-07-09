#include "cinder/app/App.h"
#include "cinder/gl/gl.h"

#include "BulletConverter.h"
#include "BulletDebugDrawer.h"

namespace btd
{
	BulletDebugDrawer::BulletDebugDrawer()
		: mDebugModes( 0 )
	{
	}

	BulletDebugDrawer::~BulletDebugDrawer()
	{
	}

	void BulletDebugDrawer::drawLine( const btVector3& from, const btVector3& to, const btVector3& color )
	{
		ci::ColorA colorA = ci::ColorA( color.getX(), color.getY(), color.getZ() );

		ci::gl::color( colorA );
		ci::gl::drawLine( fromBullet( from ), fromBullet( to ) );
	}

	void BulletDebugDrawer::drawContactPoint( const btVector3& pointOnB, const btVector3& normalOnB, btScalar distance, int lifeTime, const btVector3& color )
	{
		drawLine( pointOnB, pointOnB + normalOnB * distance, color );
	}

	void BulletDebugDrawer::reportErrorWarning( const char* warningString )
	{
		ci::app::App::get()->console() << warningString << std::endl;
	}

	void BulletDebugDrawer::draw3dText( const btVector3& location, const char* textString )
	{
		ci::gl::drawString( textString, ci::Vec2f( location.getX(), location.getY() ) );
	}

	void BulletDebugDrawer::drawSphere( btScalar radius, const btTransform& transform, const btVector3& color )
	{
		ci::ColorA colorA = ci::ColorA( color.getX(), color.getY(), color.getZ() );

		ci::gl::enableWireframe();
		ci::gl::color( colorA );
		ci::gl::pushMatrices();
		ci::gl::translate( fromBullet( transform.getOrigin() ) );
		ci::gl::rotate( fromBullet( transform.getRotation() ) );
		ci::gl::drawSphere( ci::Vec3f::zero(), radius, 20 );
		ci::gl::popMatrices();
		ci::gl::disableWireframe();
	}

	void BulletDebugDrawer::drawCylinder( btScalar radius, btScalar halfHeight, int upAxis, const btTransform& transform, const btVector3& color )
	{
		btIDebugDraw::drawCylinder( radius, halfHeight, upAxis, transform, color );
		return;
		ci::ColorA colorA = ci::ColorA( color.getX(), color.getY(), color.getZ() );

		ci::gl::enableWireframe();
		ci::gl::color( colorA );
		ci::gl::pushMatrices();
		ci::gl::translate( fromBullet( transform.getOrigin() ) );
		ci::gl::rotate( fromBullet( transform.getRotation() ) );
		ci::gl::drawCylinder( radius, radius, halfHeight, 20, 3 );
		ci::gl::popMatrices();
		ci::gl::disableWireframe();
	}

	void BulletDebugDrawer::setDebugMode( int debugMode )
	{
		mDebugModes = (DebugDrawModes) debugMode;
	}

	int BulletDebugDrawer::getDebugMode() const
	{
		return mDebugModes;
	}

	void BulletDebugDrawer::setDrawTransform( bool drawTransform )
	{
		mDrawTransform = drawTransform;
	}

	bool BulletDebugDrawer::getDrawTransform() const
	{
		return mDrawTransform;
	}

	void BulletDebugDrawer::drawTransform( const btTransform& transform, btScalar orthoLen )
	{
		if( ! mDrawTransform )
			return;

		btIDebugDraw::drawTransform( transform, orthoLen );
	}

} // namespace btd