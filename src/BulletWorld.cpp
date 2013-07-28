#include "cinder/app/App.h"

#include "BulletConverter.h"
#include "BulletWorld.h"

namespace btd
{
	const float BulletWorld::CONSTRAINT_DEBUG_SIZE = 0.5f;

	BulletWorld::BulletWorld()
		: mCollisionConfiguration( NULL )
		, mDispatcher( NULL )
		, mBroadphase( NULL )
		, mSolver( NULL )
		, mSoftRigidDynamicsWorld( NULL )
		, mBulletDebugDrawer()
		, mBulletParameter()
		, mBulletPicker()
		, mShowDebugDrawRigidBody( false )
	{
	}

	BulletWorld::~BulletWorld()
	{
		donePhysics();
	}

	void BulletWorld::setup()
	{
		mTime = ci::app::App::get()->getElapsedSeconds();
		initPhysics();
	}

	void BulletWorld::update()
	{
		mBulletParameter->update();

		if( mBulletParameter->mGravity != fromBullet( mSoftRigidDynamicsWorld->getGravity() ) )
		{
			mSoftRigidDynamicsWorld->setGravity( toBullet( mBulletParameter->mGravity ) );
			btSoftBodyWorldInfo& worldInfo = mSoftRigidDynamicsWorld->getWorldInfo();
			worldInfo.m_gravity = toBullet( mBulletParameter->mGravity );
		}

		setShowDebugDrawRigidBody( mBulletParameter->mShowDebugDrawRigidBody );
		mBulletDebugDrawer->setDrawTransform( mBulletParameter->mDrawTransform );
		mBulletDebugDrawer->setDebugMode( mBulletParameter->mGeneralDebugDrawTypes );
		mSoftRigidDynamicsWorld->setDrawFlags( mBulletParameter->mSoftDebugDrawTypes );

		// simple dynamics world doesn't handle fixed-time-stepping
		double time = ci::app::App::get()->getElapsedSeconds();
		float  ellapsedTime = ( float )( time - mTime );
		time = mTime;

		if( mSoftRigidDynamicsWorld )
		{
			if( mBulletParameter->mSimulateOne || mBulletParameter->mSimulateAlways )
			{
//				const float timegranularityphysics = 1/60.f;
				const float timegranularityphysics = 1/10.f;
				float deltatimephy = ellapsedTime;
				if( deltatimephy > timegranularityphysics )
					deltatimephy = timegranularityphysics; // It is important to not simulate more than the granularity otherwise more than 1 round will be calculated that makes everything even slower that cause the frame time slower that cause even more round to calculate that makes it even more slower and so on until the system will stop, changing maxiteration to 1 is not a good solution, because the inner time lost would be accumulated and the system would want to get back if it can, made everything faster potentially a long period

				mSoftRigidDynamicsWorld->stepSimulation( deltatimephy, 10, timegranularityphysics );
				mBulletParameter->mSimulateOne = false;
			}
		}
	}

	void BulletWorld::draw()
	{
		if( mSoftRigidDynamicsWorld != NULL )
			mSoftRigidDynamicsWorld->debugDrawWorld();
	}

	void BulletWorld::initPhysics()
	{
		mCollisionConfiguration = new btDefaultCollisionConfiguration();
		mDispatcher             = new btCollisionDispatcher( mCollisionConfiguration );

		btVector3 worldAabbMin( -10000, -10000, -10000 );
		btVector3 worldAabbMax(  10000,  10000,  10000 );
		mBroadphase = new btAxisSweep3( worldAabbMin, worldAabbMax );

		mSolver = new btSequentialImpulseConstraintSolver;

		mSoftRigidDynamicsWorld = new btSoftRigidDynamicsWorld( mDispatcher, mBroadphase, mSolver, mCollisionConfiguration );
		mSoftRigidDynamicsWorld->setGravity( btVector3( 0, 0, -9.81 ) );

		btSoftBodyWorldInfo& worldInfo = mSoftRigidDynamicsWorld->getWorldInfo();
		worldInfo.m_dispatcher = mDispatcher;
		worldInfo.m_broadphase = mBroadphase;
		worldInfo.m_gravity.setValue( 0, 0, -9.81 );
		worldInfo.m_sparsesdf.Initialize();

		mSoftRigidDynamicsWorld->getSolverInfo().m_numIterations = 10;
		mSoftRigidDynamicsWorld->getDispatchInfo().m_enableSPU = true;

		mBulletParameter   = BulletParameterRef(   new BulletParameter()   );
		mBulletDebugDrawer = BulletDebugDrawerRef( new BulletDebugDrawer() );
		mSoftRigidDynamicsWorld->setDebugDrawer( mBulletDebugDrawer.get() );
		mBulletParameter->setup();
	}

	void BulletWorld::donePhysics()
	{
		mSoftRigidDynamicsWorld->setDebugDrawer( 0 );

		delete mSoftRigidDynamicsWorld;
		delete mSolver;
		delete mBroadphase;
		delete mDispatcher;
		delete mCollisionConfiguration;
	}

	btSoftRigidDynamicsWorld* BulletWorld::getDynamicsWorld()
	{
		return mSoftRigidDynamicsWorld;
	}

	void BulletWorld::addRigidBody( btRigidBody* rigidBody, bool add /* = false */ )
	{
		if( add )
			mSoftRigidDynamicsWorld->addRigidBody( rigidBody );

		mRigidBodies.push_back( rigidBody );
	}

	void BulletWorld::removeRigidBody( btRigidBody* rigidBody )
	{
		mSoftRigidDynamicsWorld->removeRigidBody( rigidBody );

		mRigidBodies.erase( std::remove( mRigidBodies.begin(), mRigidBodies.end(), rigidBody ), mRigidBodies.end() );
	}

	void BulletWorld::destroyRigidBodyAll()
	{
		for( RigidBodies::iterator it = mRigidBodies.begin(); it != mRigidBodies.end(); ++it )
		{
			btRigidBody* rigidBody = *it;

			if( rigidBody->getMotionState())
			{
				delete rigidBody->getMotionState();
			}

			if( rigidBody->getCollisionShape() )
			{
				delete rigidBody->getCollisionShape();
			}

			mSoftRigidDynamicsWorld->removeRigidBody( rigidBody );
			delete rigidBody;
		}

		mRigidBodies.clear();
	}

	void BulletWorld::addSoftBody( btSoftBody* softBody, bool add /* = false */ )
	{
		if( add )
			mSoftRigidDynamicsWorld->addSoftBody( softBody );

		mSoftBodies.push_back( softBody );
	}

	void BulletWorld::removeSoftBody( btSoftBody* softBody )
	{
		mSoftRigidDynamicsWorld->removeSoftBody( softBody );

		mSoftBodies.erase( std::remove( mSoftBodies.begin(), mSoftBodies.end(), softBody ), mSoftBodies.end() );
	}

	void BulletWorld::destroySoftBodyAll()
	{
		for( SoftBodies::iterator it = mSoftBodies.begin(); it != mSoftBodies.end(); ++it )
		{
			btSoftBody* softBody = *it;

			mSoftRigidDynamicsWorld->removeSoftBody( softBody );
			delete softBody;
		}

		mSoftBodies.clear();
	}

	void BulletWorld::addConstraint( btTypedConstraint* constraint, bool add /* = false */, bool disableCollisionsBetweenLinkedBodies /* = false */ )
	{
		if( add )
			mSoftRigidDynamicsWorld->addConstraint( constraint, disableCollisionsBetweenLinkedBodies );

		mConstraints.push_back( constraint );
	}

	void BulletWorld::removeConstraint( btTypedConstraint* constraint )
	{
		mSoftRigidDynamicsWorld->removeConstraint( constraint );

		mConstraints.erase( std::remove( mConstraints.begin(), mConstraints.end(), constraint ), mConstraints.end() );
	}

	void BulletWorld::destroyConstraintAll()
	{
		for( Constraints::iterator it = mConstraints.begin(); it != mConstraints.end(); ++it )
		{
			btTypedConstraint* constraint = *it;

			mSoftRigidDynamicsWorld->removeConstraint( constraint );
			delete constraint;
		}

		mConstraints.clear();
	}

	void BulletWorld::setupRigidBody( btRigidBody* rigidBody )
	{
		rigidBody->setDamping( mBulletParameter->mLinearDamping, mBulletParameter->mAngularDamping );
		rigidBody->setDeactivationTime( mBulletParameter->mDeactivationTime );
		rigidBody->setSleepingThresholds( mBulletParameter->mLinearSleepingThresholds, mBulletParameter->mAngularSleepingThresholds );

		if( mBulletParameter->mShowDebugDrawRigidBody )
			rigidBody->setCollisionFlags( rigidBody->getCollisionFlags() & ~( btCollisionObject::CF_DISABLE_VISUALIZE_OBJECT ) );
		else
			rigidBody->setCollisionFlags( rigidBody->getCollisionFlags() |    btCollisionObject::CF_DISABLE_VISUALIZE_OBJECT );
	}

	void BulletWorld::setupSoftBody( btSoftBody* softBody )
	{
		softBody->setTotalMass( mBulletParameter->mRopeMass );

		softBody->m_cfg.kVCF        = mBulletParameter->mKVCF;
		softBody->m_cfg.kDP         = mBulletParameter->mKDP;
		softBody->m_cfg.kDG         = mBulletParameter->mKDG;
		softBody->m_cfg.kLF         = mBulletParameter->mKLF;
		softBody->m_cfg.kPR         = mBulletParameter->mKPR;
		softBody->m_cfg.kVC         = mBulletParameter->mKVC;
		softBody->m_cfg.kDF         = mBulletParameter->mKDF;
		softBody->m_cfg.kMT         = mBulletParameter->mKMT;
		softBody->m_cfg.kCHR        = mBulletParameter->mKCHR;
		softBody->m_cfg.kKHR        = mBulletParameter->mKKHR;
		softBody->m_cfg.kSHR        = mBulletParameter->mKSHR;
		softBody->m_cfg.kAHR        = mBulletParameter->mKAHR;
		softBody->m_cfg.maxvolume   = mBulletParameter->mMaxvolume;
		softBody->m_cfg.timescale   = mBulletParameter->mTimescale;
		softBody->m_cfg.viterations = mBulletParameter->mViterations;
		softBody->m_cfg.piterations = mBulletParameter->mPiterations;
		softBody->m_cfg.diterations = mBulletParameter->mDiterations;
		softBody->m_cfg.citerations = mBulletParameter->mCiterations;
	}

	void BulletWorld::setupConstraint( btTypedConstraint* constraint )
	{
		constraint->setDbgDrawSize( CONSTRAINT_DEBUG_SIZE );

		if( constraint->getConstraintType() == CONETWIST_CONSTRAINT_TYPE )
		{
			static_cast< btConeTwistConstraint* >( constraint )->setDamping( mBulletParameter->mDamping );
		}

		for( int i = 0; i < 6; ++i )
		{
			constraint->setParam( BT_CONSTRAINT_STOP_CFM, mBulletParameter->mStopCMF, i );
			constraint->setParam( BT_CONSTRAINT_STOP_ERP, mBulletParameter->mStopERP, i );
		}
	}

	BulletParameterRef BulletWorld::getBulletParameter() const
	{
		return mBulletParameter;
	}

	void BulletWorld::clear()
	{
		destroyConstraintAll();
		destroyRigidBodyAll();
		destroySoftBodyAll();
	}

	void BulletWorld::mouseDown( ci::app::MouseEvent event, const ci::CameraPersp &cam )
	{
		// release picker if any
		mouseUp( event, cam );

		ci::Vec2f pos = ci::Vec2f( event.getPos() ) / ci::Vec2f( ci::app::App::get()->getWindowSize() );
		pos.y         = 1.0f - pos.y;
		ci::Ray ray   = cam.generateRay( pos.x, pos.y, ci::app::App::get()->getWindowAspectRatio() );
		checkIntersects( ray, cam.getFarClip() );
	}

	bool BulletWorld::checkIntersects( const ci::Ray &ray, float farClip )
	{
		btVector3 rayFrom = toBullet( ray.getOrigin() );
		btVector3 rayTo   = toBullet( ray.calcPosition( farClip ) );

		btCollisionWorld::ClosestRayResultCallback rayCallback( rayFrom, rayTo );
		mSoftRigidDynamicsWorld->rayTest( rayFrom, rayTo, rayCallback );

		if( rayCallback.hasHit() )
		{
			btRigidBody* rigidBody = const_cast< btRigidBody* >( btRigidBody::upcast( rayCallback.m_collisionObject ) );
			if( rigidBody )
			{
				btVector3 position = rayCallback.m_hitPointWorld;
				btVector3 pivot    = rigidBody->getCenterOfMassTransform().inverse() * position;

				mBulletPicker = BulletPickerRef( new BulletPicker( rigidBody, pivot, ( position - rayFrom ).length(), fromBullet( rayTo ) ) );

				mBulletPicker->getConstraint()->m_setting.m_impulseClamp = 0.0f;
				mBulletPicker->getConstraint()->m_setting.m_tau          = 0.1f;

				addConstraint( mBulletPicker->getConstraint(), true );
			}
		}
		return false;
	}

	void BulletWorld::mouseDrag( ci::app::MouseEvent event, const ci::CameraPersp &cam )
	{
		if( mBulletPicker )
		{
			ci::Vec2f pos = ci::Vec2f( event.getPos() ) / ci::Vec2f( ci::app::App::get()->getWindowSize() );
			pos.y         = 1.0f - pos.y;
			ci::Ray ray   = cam.generateRay( pos.x, pos.y, ci::app::App::get()->getWindowAspectRatio() );
			mBulletPicker->update( ray );
		}
	}

	void BulletWorld::mouseUp( ci::app::MouseEvent event, const ci::CameraPersp &cam )
	{
		if( mBulletPicker )
		{
			removeConstraint( mBulletPicker->getConstraint() );
			mBulletPicker = BulletPickerRef();
		}
	}

	void BulletWorld::keyDown( ci::app::KeyEvent event )
	{
		switch( event.getCode() )
		{
		case ci::app::KeyEvent::KEY_v:
			mBulletParameter->mSimulateOne    = ! mBulletParameter->mSimulateOne;
			break;
		case ci::app::KeyEvent::KEY_c:
			mBulletParameter->mSimulateAlways = ! mBulletParameter->mSimulateAlways;
			break;
		}
	}

	void BulletWorld::setShowDebugDrawRigidBody( bool showDebugDrawRigidBody )
	{
		if( mShowDebugDrawRigidBody == showDebugDrawRigidBody )
			return;

		mShowDebugDrawRigidBody = showDebugDrawRigidBody;

		for( RigidBodies::iterator it = mRigidBodies.begin(); it != mRigidBodies.end(); ++it )
		{
			btRigidBody* rigidBody = *it;

			if( mShowDebugDrawRigidBody )
				rigidBody->setCollisionFlags( rigidBody->getCollisionFlags() & ~( btCollisionObject::CF_DISABLE_VISUALIZE_OBJECT ) );
			else
				rigidBody->setCollisionFlags( rigidBody->getCollisionFlags() |    btCollisionObject::CF_DISABLE_VISUALIZE_OBJECT );
		}
	}

	bool BulletWorld::getShowDebugDrawRigidBody() const
	{
		return mShowDebugDrawRigidBody;
	}

} // namespace btd
