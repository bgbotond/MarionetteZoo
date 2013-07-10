#pragma warning (disable : 4996)

#include <map>
#include <assert.h>
#include "cinder/app/App.h"
#include "cinder/Vector.h"
#include "BulletConverter.h"
#include "Model.h"
#include "BulletWorld.h"
#include "ModelFileManager.h"

#include "Extras/Serialize/BulletFileLoader/bFile.h"
#include "Extras/Serialize/BulletWorldImporter/btBulletWorldImporter.h"
#include "BulletExtension/btSoftWorldImporter.h"

namespace btd
{
	Model::Model( const BulletWorldRef& bulletWorld, const ci::Vec3f& worldOffset, const std::string& type )
		: mBulletWorld( bulletWorld )
		, mAssimpLoader()
		, mBones()
		, mConstraints()
	{
		mAssimpLoader = mndl::assimp::AssimpLoaderRef( new mndl::assimp::AssimpLoader( ModelFileManager::getSingleton().getModelFile( type ) ) );
// 		mAssimpLoader->enableSkinning( true );
// 		mAssimpLoader->enableAnimation( true );

		// TODO apply the worldOffset to place it in special position

		loadBullet( ModelFileManager::getSingleton().getBulletFile( type ) );
	}

	Model::~Model()
	{
		// do nothing
	}

	void Model::update( const ci::Vec3f& pos, const ci::Vec3f& dir, const ci::Vec3f& norm )
	{
		for( auto it = mBones.begin(); it != mBones.end(); ++it )
		{
			BoneRef bone = *it;;
			bone->synchronize();
		}

		mAssimpLoader->update();

		// TODO update CrossBar
	}

	void Model::draw()
	{
		if( mBulletWorld->getBulletParameter()->mDrawSkin )
		{
			if ( mBulletWorld->getBulletParameter()->mEnableWireframe )
				ci::gl::enableWireframe();

			mAssimpLoader->draw();

			if ( mBulletWorld->getBulletParameter()->mEnableWireframe )
				ci::gl::disableWireframe();
		}

		// TODO draw strings (SoftBody)
		// how to calculate the positions of the string

	}

	BulletWorldRef& Model::getBulletWorld()
	{
		return mBulletWorld;
	}

	void Model::loadBullet( const ci::fs::path& bulletFile )
	{
		btSoftBulletWorldImporter worldImporter( (btSoftRigidDynamicsWorld*)mBulletWorld->getDynamicsWorld() );

		worldImporter.loadFile( bulletFile.string().c_str());

		handleLoadRigidBodies( &worldImporter );
		handleLoadConstraints( &worldImporter );
		handleLoadSoftBodies(  &worldImporter );
	}

	void Model::handleLoadRigidBodies( btSoftBulletWorldImporter* worldImporter )
	{
		for( int rigidBodyIdx = 0; rigidBodyIdx < worldImporter->getNumRigidBodies(); rigidBodyIdx++ )
		{
			btRigidBody* rigidBody = btRigidBody::upcast( worldImporter->getRigidBodyByIndex( rigidBodyIdx ) );
			std::string name = worldImporter->getNameForPointer( rigidBody );

			// all the rigid body starts with rb_ prefix.
			assert( name.substr( 0, 3 ) == "rb_" );
//			name = name.substr( 3, name.npos );

			mBulletWorld->setupRigidBody( rigidBody );
			mBulletWorld->addRigidBody( rigidBody, true );

			// All the rigid body name should match with the .dae file bones.
			mndl::assimp::AssimpNodeRef node = mAssimpLoader->getAssimpNode( name );

			BoneRef bone = BoneRef( new Bone( this, node, rigidBody ) );
			mBones.push_back( bone );
		}
	}

	void Model::handleLoadConstraints( btSoftBulletWorldImporter* worldImporter )
	{
		for( int constraintIdx = 0; constraintIdx < worldImporter->getNumConstraints(); constraintIdx++ )
		{
			btTypedConstraint* typedConstraint = worldImporter->getConstraintByIndex( constraintIdx );
			assert( typedConstraint->getConstraintType() == CONETWIST_CONSTRAINT_TYPE );

			mBulletWorld->setupConstraint( typedConstraint );
			mBulletWorld->addConstraint( typedConstraint, true, true );

			btRigidBody* rigidBodyA = &typedConstraint->getRigidBodyA();
			btRigidBody* rigidBodyB = &typedConstraint->getRigidBodyB();

			BoneRef boneA = getBone( rigidBodyA );
			BoneRef boneB = getBone( rigidBodyB );

			ConstraintRef constraint = ConstraintRef( new Constraint( this, boneA, boneB, typedConstraint ) );
			mConstraints.push_back( constraint );
		}
	}

	void Model::handleLoadSoftBodies( btSoftBulletWorldImporter* worldImporter )
	{
		for( int softBodyIdx = 0; softBodyIdx < worldImporter->getNumSoftBodies(); softBodyIdx++ )
		{
			btSoftBody* softBody = worldImporter->getSoftBodyByIndex( softBodyIdx );

			// the SoftBody names are not loaded by the worldImporter
// 			std::string name = worldImporter->getNameForPointer( softBody );
// 
// 			// all the soft body starts with sb_ prefix.
// 			assert( name.substr( 0, 3 ) == "sb_" );
// 			name = name.substr( 3, name.npos );

			mBulletWorld->setupSoftBody( softBody );
			mBulletWorld->addSoftBody( softBody, true );

			StringRef string = StringRef( new String( this, softBody ) );
			mStrings.push_back( string );
		}
	}

	BoneRef Model::getBone( btRigidBody* rigidBody )
	{
		for( auto it = mBones.begin(); it != mBones.end(); ++it )
		{
			BoneRef bone = *it;

			if( rigidBody == bone->getRigidBody() )
				return bone;
		}

		assert( 0 );

		return BoneRef();
	}


	Bone::Bone( Model* owner, const mndl::NodeRef& node, btRigidBody* rigidBody )
		: mOwner( owner )
		, mNode( node )
		, mRigidBody( rigidBody )
	{
	}

	Bone::~Bone()
	{
		btCollisionShape* shape = mRigidBody->getCollisionShape();

		mOwner->getBulletWorld()->removeRigidBody( mRigidBody );

		while( mRigidBody->getNumConstraintRefs() > 0 )
			mRigidBody->removeConstraintRef( mRigidBody->getConstraintRef( 0 ) );

		if( mRigidBody->getMotionState() )
			delete mRigidBody->getMotionState();

		delete shape;
		delete mRigidBody;
	}

	mndl::NodeRef Bone::getNode() const
	{
		return mNode;
	}

	btRigidBody* Bone::getRigidBody() const
	{
		return mRigidBody;
	}

	// synchronize the bullet world to graphics
	void Bone::synchronize()
	{
		const btTransform& centerOfMassTransform = getRigidBody()->getCenterOfMassTransform();

		ci::Vec3f pos = fromBullet( centerOfMassTransform.getOrigin()   );
		ci::Quatf rot = fromBullet( centerOfMassTransform.getRotation() );

		getNode()->setPosition(    getNode()->convertWorldToLocalPosition(    pos ) );
		getNode()->setOrientation( getNode()->convertWorldToLocalOrientation( rot ) );
	}

	Constraint::Constraint( Model* owner, const BoneRef& boneA, const BoneRef& boneB, btTypedConstraint* constraint )
		: mOwner( owner )
		, mBoneA( boneA )
		, mBoneB( boneB )
		, mConstraint( constraint )
	{
	}

	Constraint::~Constraint()
	{
		mOwner->getBulletWorld()->removeConstraint( mConstraint );

		delete mConstraint;
	}

	BoneRef Constraint::getBoneA() const
	{
		return mBoneA;
	}

	BoneRef Constraint::getBoneB() const
	{
		return mBoneB;
	}

	btTypedConstraint* Constraint::getConstraint() const
	{
		return mConstraint;
	}

	String::String( Model* owner, btSoftBody* softBody )
		: mOwner( owner )
		, mSoftBody( softBody )
	{
	}

	String::~String()
	{
		mOwner->getBulletWorld()->removeSoftBody( mSoftBody );

		delete mSoftBody;
	}

	btSoftBody* String::getSoftBody() const
	{
		return mSoftBody;
	}

} // namespace btd