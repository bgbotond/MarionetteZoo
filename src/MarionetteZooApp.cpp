#include "cinder/app/AppNative.h"
#include "cinder/gl/gl.h"
#include "cinder/gl/Light.h"
#include "cinder/gl/Texture.h"
#include "cinder/ImageIo.h"
#include "cinder/MayaCamUI.h"
#include "cinder/Rand.h"
#include "cinder/Utilities.h"

#include "BulletWorld.h"
#include "Model.h"
#include "ModelManager.h"
#include "ModelFileManager.h"

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

	void testModel();

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

	vector< string > mModelTypes;
	int mModelTypeId;

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
// 		glEnable( GL_LIGHTING );
// 		glEnable( GL_DEPTH_TEST );
// 		glEnable( GL_RESCALE_NORMAL );
// 
// 		//create light
// 		mLight = new gl::Light( gl::Light::DIRECTIONAL, 0 );
// 		mLight->setAmbient( Color( 1.0f, 1.0f, 1.0f ) );
// 		mLight->setDiffuse( Color( 1.0f, 1.0f, 1.0f ) );
// 		mLight->setSpecular( Color( 1.0f, 1.0f, 1.0f ) );
// 		mLight->setShadowParams( 100.0f, 1.0f, 20.0f );
// //		mLight->update( cam );
// //		mLight->enable();
	}

	mBulletWorld = BulletWorldRef( new BulletWorld() );
	mBulletWorld->setup();

	mModelManager = ModelManagerRef( new ModelManager( mBulletWorld ) );
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
	mParams.addPersistentParam( "Eye", &mCameraEyePoint, Vec3f( 0.0f, -20.0f, 0.0f ) );
	mParams.addPersistentParam( "Center of Interest", &mCameraCenterOfInterestPoint, Vec3f( 0.0f, 0.0f, 0.1f ) );
	mParams.addButton( "Reset camera", [ & ]()
			{
				mCameraCenterOfInterestPoint = Vec3f( 0.0f, 0.0f, 0.1f );
				mCameraFov = 45.f;
				mCameraEyePoint = Vec3f( 0.0f, -20.0f, 0.0f );

				CameraPersp cam = mMayaCam.getCamera();
				cam.setPerspective( mCameraFov, getWindowAspectRatio(), 0.1f, 1000.0f );
				cam.setEyePoint( mCameraEyePoint );
				cam.setCenterOfInterestPoint( mCameraCenterOfInterestPoint );
				mMayaCam.setCurrentCam( cam );
			} );
	mParams.addSeparator();

	mModelTypes = ModelFileManager::getSingleton().getModelTypes();
	mParams.addPersistentParam( "Model", mModelTypes, &mModelTypeId, 0 );
	if ( mModelTypeId > mModelTypes.size() )
		mModelTypeId = 0;

	mParams.addButton( "Test model", std::bind( &MarionetteZooApp::testModel, this ) );
}

void MarionetteZooApp::mouseDown( MouseEvent event )
{
	if ( mCameraLock )
		mBulletWorld->mouseDown( event, mMayaCam.getCamera() );
	else
		mMayaCam.mouseDown( event.getPos() );
}

void MarionetteZooApp::mouseDrag( MouseEvent event )
{
	if ( mCameraLock )
		mBulletWorld->mouseDrag( event, mMayaCam.getCamera() );
	else
		mMayaCam.mouseDrag( event.getPos(), event.isLeftDown(), event.isMiddleDown(), event.isRightDown() );
}

void MarionetteZooApp::mouseUp( MouseEvent event )
{
	if ( mCameraLock )
		mBulletWorld->mouseUp( event, mMayaCam.getCamera() );
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
	case KeyEvent::KEY_l:
		{
			mCameraLock = ! mCameraLock;
		}
		break;
	case KeyEvent::KEY_LEFT:
		{
			mMayaCam.mouseDown( Vec2i( mStepKey, 0 ) );
			mMayaCam.mouseDrag( Vec2i( mStepKey, 0 ), true, false, false );
			mMayaCam.mouseDrag( Vec2i( 0       , 0 ), true, false, false );
		}
		break;
	case KeyEvent::KEY_RIGHT:
		{
			mMayaCam.mouseDown( Vec2i( 0       , 0 ) );
			mMayaCam.mouseDrag( Vec2i( 0       , 0 ), true, false, false );
			mMayaCam.mouseDrag( Vec2i( mStepKey, 0 ), true, false, false );
		}
		break;
	case KeyEvent::KEY_UP:
		{
			mMayaCam.mouseDown( Vec2i( 0, mStepKey ) );
			mMayaCam.mouseDrag( Vec2i( 0, mStepKey ), true, false, false );
			mMayaCam.mouseDrag( Vec2i( 0, 0        ), true, false, false );
		}
		break;
	case KeyEvent::KEY_DOWN:
		{
			mMayaCam.mouseDown( Vec2i( 0, 0        ) );
			mMayaCam.mouseDrag( Vec2i( 0, 0        ), true, false, false );
			mMayaCam.mouseDrag( Vec2i( 0, mStepKey ), true, false, false );
		}
		break;
	case KeyEvent::KEY_ESCAPE:
		quit();
		break;
	default:
		mBulletWorld->keyDown( event );
	}
}

void MarionetteZooApp::update()
{
	mFps = getAverageFps();

	CameraPersp cam = mMayaCam.getCamera();
	if ( cam.getFov() != mCameraFov )
	{
		cam.setPerspective( mCameraFov, getWindowAspectRatio(), 0.1f, 1000.0f );
		mMayaCam.setCurrentCam( cam );
	}
	if( mCameraLock )
	{
		if( mCameraEyePoint != cam.getEyePoint() )
		{
			cam.setEyePoint( mCameraEyePoint );
			mMayaCam.setCurrentCam( cam );
		}
		if( mCameraCenterOfInterestPoint != cam.getCenterOfInterestPoint() )
		{
			cam.setCenterOfInterestPoint( mCameraCenterOfInterestPoint );
			mMayaCam.setCurrentCam( cam );
		}
	}
	else
	{
		mCameraEyePoint              = cam.getEyePoint();
		mCameraCenterOfInterestPoint = cam.getCenterOfInterestPoint();
	}

// 	mLight->setDirection( mLightDirection * Vec3f( 1.f, 1.f, -1.f ) );
// 	mLight->update( cam );

	mBulletWorld->update();
	mModelManager->update();
}

void MarionetteZooApp::draw()
{
	// clear out the window with black
	gl::clear( Colorf( 0.392, 0.392, 0.784 ) );

	gl::setViewport( getWindowBounds() );
	gl::setMatrices( mMayaCam.getCamera() );

	gl::enableDepthRead();
	gl::enableDepthWrite();

	ci::gl::pushMatrices();
	mBulletWorld->draw();
	ci::gl::popMatrices();
	mModelManager->draw( mMayaCam.getCamera() );

	ci::gl::drawCoordinateFrame( 5.f );

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

void MarionetteZooApp::testModel()
{
	mModelManager->destroyModelAll();
	mModelManager->createModel( "1", ci::Vec3f::zero(), mModelTypes[ mModelTypeId ] );
}

CINDER_APP_NATIVE( MarionetteZooApp, RendererGl( RendererGl::AA_MSAA_4 ) )
