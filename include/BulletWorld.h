#pragma once

#include "cinder/app/App.h"
#include "cinder/app/MouseEvent.h"
#include "cinder/Camera.h"
#include "cinder/Vector.h"
#include "btBulletDynamicsCommon.h"
#include "BulletSoftBody/btSoftRigidDynamicsWorld.h"
#include "BulletSoftBody/btSoftBody.h"
#include "BulletDebugDrawer.h"
#include "BulletPicker.h"
#include "BulletParameter.h"
//#include "AssimpModel.h"
#include "mndlkit/params/PParams.h"

namespace btd
{
	typedef std::shared_ptr< class BulletWorld > BulletWorldRef;

	class BulletWorld
	{
	protected:
		typedef std::vector< btRigidBody*       > RigidBodies;
		typedef std::vector< btSoftBody*        > SoftBodies;
		typedef std::vector< btTypedConstraint* > Constraints;

	public:
		BulletWorld();
		~BulletWorld();

		void setup();
		void update();
		void draw();

		void mouseDown( ci::app::MouseEvent event, const ci::CameraPersp &cam );
		void mouseDrag( ci::app::MouseEvent event, const ci::CameraPersp &cam );
		void mouseUp( ci::app::MouseEvent event, const ci::CameraPersp &cam );
		void keyDown( ci::app::KeyEvent event );

		void initPhysics();
		void donePhysics();

		btDynamicsWorld* getDynamicsWorld();

		void addRigidBody( btRigidBody* rigidBody, bool load = false );
		void removeRigidBody( btRigidBody* rigidBody );
		void destroyRigidBodyAll();
		void addSoftBody( btSoftBody* softBody, bool load = false );
		void removeSoftBody( btSoftBody* softBody );
		void destroySoftBodyAll();
		void addConstraint( btTypedConstraint* constraint, bool disableCollisionsBetweenLinkedBodies, bool load = false );
		void removeConstraint( btTypedConstraint* constraint );
		void destroyConstraintAll();

		void setupRigidBody( btRigidBody* rigidBody );
		void setupSoftBody( btSoftBody* softBody );
		void setupConstraint( btTypedConstraint* constraint );

		BulletParameterRef getBulletParameter() const;

		void clear();

	protected:
		bool checkIntersects( const ci::Ray &ray, float farClip );

		void setShowDebugDrawRigidBody( bool showDebugDrawRigidBody );
		bool getShowDebugDrawRigidBody() const;

	protected:
		btDefaultCollisionConfiguration*           mCollisionConfiguration;
		btCollisionDispatcher*                     mDispatcher;
		btBroadphaseInterface*                     mBroadphase;
		btSequentialImpulseConstraintSolver*       mSolver;
		btSoftRigidDynamicsWorld*                  mSoftRigidDynamicsWorld;
		btSoftBodyWorldInfo                        mSoftBodyWorldInfo;

		RigidBodies                                mRigidBodies;
		SoftBodies                                 mSoftBodies;
		Constraints                                mConstraints;

		BulletDebugDrawerRef                       mBulletDebugDrawer;
		BulletParameterRef                         mBulletParameter;
		BulletPickerRef                            mBulletPicker;

		bool                                       mShowDebugDrawRigidBody;
		double                                     mTime;

		static const float                         CONSTRAINT_DEBUG_SIZE;
	};

} // namespace btd
