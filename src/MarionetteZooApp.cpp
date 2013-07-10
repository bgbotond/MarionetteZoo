#include "cinder/app/AppNative.h"
#include "cinder/gl/gl.h"
#include "cinder/gl/Light.h"
#include "cinder/gl/Texture.h"
#include "cinder/ImageIo.h"
#include "cinder/MayaCamUI.h"
#include "cinder/Rand.h"
#include "cinder/Timeline.h"
#include "cinder/Utilities.h"

#include "BulletWorld.h"
#include "ModelManager.h"
#include "Model.h"

// for soft body test
#include "BulletExtension/btSoftWorldImporter.h"

#include "mndlkit/params/PParams.h"

using namespace ci;
using namespace ci::app;
using namespace std;
using namespace btd;

class MarionetteZooApp : public AppNative
{
public:
	void prepareSettings( Settings *settings );
	void setup();
	void mouseDown( MouseEvent event );
	void mouseDrag( MouseEvent event );
	void mouseUp( MouseEvent event );
	void keyDown( KeyEvent event );
	void update();
	void draw();
	void resize();
	void shutdown();

	void reload();

protected:
	void setupParams();

protected:

	mndl::params::PInterfaceGl mParams;
	float mFps;

	BulletWorldRef  mBulletWorld;
	ModelManagerRef mModelManager;

	// camera
	MayaCamUI mMayaCam;
	bool      mCameraLock;
	float     mCameraFov;
	Vec3f     mCameraEyePoint;
	Vec3f     mCameraCenterOfInterestPoint;
	static const int mStepKey = 3;

	gl::Light* mLight;
	Vec3f      mLightDirection;
};

void MarionetteZooApp::prepareSettings( Settings *settings )
{
	settings->setWindowSize( 1280, 800 );
}

void MarionetteZooApp::setup()
{
	gl::enableDepthRead();
	gl::enableDepthWrite();

	setupParams();

	CameraPersp cam;
	cam.setPerspective( mCameraFov, getWindowAspectRatio(), 0.1f, 1000.0f );
	cam.setEyePoint( mCameraEyePoint );
	cam.setCenterOfInterestPoint( mCameraCenterOfInterestPoint );
	mMayaCam.setCurrentCam( cam );

	{
		glEnable( GL_LIGHTING );
		glEnable( GL_DEPTH_TEST );
		glEnable( GL_RESCALE_NORMAL );

		//create light
		mLight = new gl::Light( gl::Light::DIRECTIONAL, 0 );
		mLight->setAmbient( Color( 1.0f, 1.0f, 1.0f ) );
		mLight->setDiffuse( Color( 1.0f, 1.0f, 1.0f ) );
		mLight->setSpecular( Color( 1.0f, 1.0f, 1.0f ) );
		mLight->setShadowParams( 100.0f, 1.0f, 20.0f );
//		mLight->update( cam );
//		mLight->enable();
	}

	mBulletWorld = BulletWorldRef( new BulletWorld() );
	mBulletWorld->setup();

	mModelManager = ModelManagerRef( new ModelManager( mBulletWorld ) );

//	ModelRef model = mModelManager->createModel( "madar", ci::Vec3f::zero(), "bird" );
}

void MarionetteZooApp::setupParams()
{
	mndl::params::PInterfaceGl::load( "params.xml" );

	mParams = mndl::params::PInterfaceGl( "Parameters", Vec2i( 230, 550 ), Vec2i( 50, 50 ) );
	mParams.addPersistentSizeAndPosition();

	mFps = 0;
	mParams.addParam( "Fps", &mFps, "", true );
	mParams.addSeparator();
	mParams.addText( "Camera" );
	mParams.addPersistentParam( "Lock camera (l)", &mCameraLock, false );
	mParams.addPersistentParam( "Fov", &mCameraFov, 45.f, "min=20 max=180 step=.1" );
	mParams.addPersistentParam( "Eye", &mCameraEyePoint, Vec3f( 0.0f, 10.0f, -40.0f ));
	mParams.addPersistentParam( "Center of Interest", &mCameraCenterOfInterestPoint, Vec3f( 0.0f, 10.0f, 0.0f ));

	mParams.addButton( "Reload", [ this ]()
								{
									reload();
								} );
}

void MarionetteZooApp::mouseDown( MouseEvent event )
{
	mMayaCam.mouseDown( event.getPos() );
}

void MarionetteZooApp::mouseDrag( MouseEvent event )
{
	mMayaCam.mouseDrag( event.getPos(), event.isLeftDown(), event.isMiddleDown(), event.isRightDown() );
}

void MarionetteZooApp::mouseUp( MouseEvent event )
{
}

void MarionetteZooApp::keyDown( KeyEvent event )
{
	switch( event.getCode() )
	{
	case KeyEvent::KEY_f:
		if ( ! isFullScreen() )
		{
			setFullScreen( true );
			if ( mParams.isVisible() )
				showCursor();
			else
				hideCursor();
		}
		else
		{
			setFullScreen( false );
			showCursor();
		}
		break;
	case KeyEvent::KEY_s:
		{
			mndl::params::PInterfaceGl::showAllParams( !mParams.isVisible() );
			if ( isFullScreen() )
			{
				if ( mParams.isVisible() )
					showCursor();
				else
					hideCursor();
			}
			break;
		}
	case KeyEvent::KEY_LEFT:
		{
			mMayaCam.mouseDown( Vec2i( mStepKey, 0 ));
			mMayaCam.mouseDrag( Vec2i( mStepKey, 0 ), true, false, false );
			mMayaCam.mouseDrag( Vec2i( 0       , 0 ), true, false, false );
		}
		break;
	case KeyEvent::KEY_RIGHT:
		{
			mMayaCam.mouseDown( Vec2i( 0       , 0 ));
			mMayaCam.mouseDrag( Vec2i( 0       , 0 ), true, false, false );
			mMayaCam.mouseDrag( Vec2i( mStepKey, 0 ), true, false, false );
		}
		break;
	case KeyEvent::KEY_UP:
		{
			mMayaCam.mouseDown( Vec2i( 0, mStepKey ));
			mMayaCam.mouseDrag( Vec2i( 0, mStepKey ), true, false, false );
			mMayaCam.mouseDrag( Vec2i( 0, 0        ), true, false, false );
		}
		break;
	case KeyEvent::KEY_DOWN:
		{
			mMayaCam.mouseDown( Vec2i( 0, 0        ));
			mMayaCam.mouseDrag( Vec2i( 0, 0        ), true, false, false );
			mMayaCam.mouseDrag( Vec2i( 0, mStepKey ), true, false, false );
		}
		break;
	case KeyEvent::KEY_ESCAPE:
		quit();
		break;
	}
}

void MarionetteZooApp::update()
{
	CameraPersp cam = mMayaCam.getCamera();

	mLight->setDirection( mLightDirection * Vec3f( 1.f, 1.f, -1.f ) );
	mLight->update( cam );

	mBulletWorld->update();
	mModelManager->update();
}

void MarionetteZooApp::draw()
{
	gl::clear( Colorf( 0.392, 0.392, 0.784 ));

	gl::setViewport( getWindowBounds() );
	gl::setMatrices( mMayaCam.getCamera() );

	gl::enableDepthRead();
	gl::enableDepthWrite();

	mBulletWorld->draw();
	mModelManager->draw();

	mParams.draw();
}

void MarionetteZooApp::resize()
{
	CameraPersp cam = mMayaCam.getCamera();
	cam.setAspectRatio( getWindowAspectRatio() );
	mMayaCam.setCurrentCam( cam );
}

void MarionetteZooApp::shutdown()
{
	mndl::params::PInterfaceGl::save();
}

void MarionetteZooApp::reload()
{
	mBulletWorld->clear();

	btSoftBulletWorldImporter worldImporter( (btSoftRigidDynamicsWorld*)mBulletWorld->getDynamicsWorld() );
	worldImporter.loadFile( app::getAssetPath( "softbodytest_small_rectangle.bullet" ).string().c_str() );

	for( int rigidBodyIdx = 0; rigidBodyIdx < worldImporter.getNumRigidBodies(); rigidBodyIdx++ )
	{
		btRigidBody* rigidBody = btRigidBody::upcast( worldImporter.getRigidBodyByIndex( rigidBodyIdx ) );
		mBulletWorld->setupRigidBody( rigidBody );
		mBulletWorld->addRigidBody( rigidBody, true );
	}

	for( int constraintIdx = 0; constraintIdx < worldImporter.getNumConstraints(); constraintIdx++ )
	{
		btTypedConstraint* constraint = worldImporter.getConstraintByIndex( constraintIdx );
		//btTypedConstraintType type = constraint->getConstraintType();
		mBulletWorld->setupConstraint( constraint );
		mBulletWorld->addConstraint( constraint, true, true );
	}

	for( int softBodyIdx = 0; softBodyIdx < worldImporter.getNumSoftBodies(); softBodyIdx++ )
	{
		btSoftBody* softBody = worldImporter.getSoftBodyByIndex( softBodyIdx );
		mBulletWorld->setupSoftBody( softBody );
		mBulletWorld->addSoftBody( softBody, true );
	}
}

CINDER_APP_NATIVE( MarionetteZooApp, RendererGl( RendererGl::AA_MSAA_4 ) )
