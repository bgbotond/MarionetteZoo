#pragma warning (disable : 4996)

#include <assert.h>

#include "cinder/app/App.h"
#include "cinder/Vector.h"
#include "cinder/Xml.h"
#include "cinder/Rand.h"

#include "BulletConverter.h"
#include "Model.h"
#include "BulletWorld.h"
#include "ModelFileManager.h"
#include "GlobalData.h"

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
		loadActions( ModelFileManager::getSingleton().getActionFile( type ) );
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

		for( auto it = mStrings.begin(); it != mStrings.end(); ++it )
		{
			StringRef string = *it;
			string->synchronize();
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
		// position of the string should be calculated in String::synchrinize
	}

	BulletWorldRef& Model::getBulletWorld()
	{
		return mBulletWorld;
	}

	unsigned int Model::getNumActions() const
	{
		return mActions.size();
	}

	std::vector< std::string > Model::getNameActions()
	{
		std::vector< std::string > actionNames;

		for( auto it = mActions.begin(); it != mActions.end(); ++it )
		{
			ActionRef action = *it;
			actionNames.push_back( action->getName() );
		}

		return actionNames;
	}

	void Model::doAction( unsigned int pos )
	{
		if( pos >= getNumActions() )
			return;

		ActionRef action = mActions[ pos ];

		action->doAction();
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

			// all the rigid body should start with "rb_" prefix.
			assert( name.substr( 0, 3 ) == "rb_" );

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

			// it should link two RigidBodies
			assert( softBody->m_anchors.size() == 2 );

			BoneRef bone0 = getBone( softBody->m_anchors[ 0 ].m_body );
			BoneRef bone1 = getBone( softBody->m_anchors[ 1 ].m_body );

			// string name should be one of the RigidBody name it links but with "sb_" prefix instead of "rb_" prefix
			std::string name = bone0->getNode()->getName();
			mndl::assimp::AssimpNodeRef nodeString = mAssimpLoader->getAssimpNode( std::string( "sb_" ) + name.substr( 3, name.npos ) );
			if( ! nodeString )
			{
				name = bone1->getNode()->getName();
				nodeString = mAssimpLoader->getAssimpNode( std::string( "sb_" ) + name.substr( 3, name.npos ) );
			}

			assert( nodeString );
			assert( nodeString->mMeshes.size() == 1 ); // should contain only one mesh
			mndl::assimp::AssimpMeshRef mesh = *nodeString->mMeshes.begin();
			assert( softBody->m_nodes.size() == mesh->mAnimatedPos.size() ); // softBody vertex size should be the same as node mesh vertex size

			mBulletWorld->setupSoftBody( softBody );
			mBulletWorld->addSoftBody( softBody, true );

			StringRef string = StringRef( new String( this, nodeString, softBody ) );
			mStrings.push_back( string );
		}
	}

	void Model::loadActions( const ci::fs::path& actionFile )
	{
		ci::XmlTree doc( ci::loadFile( actionFile ) );

		if( ! doc.hasChild( "MarionettZoo" ) )
			return;

		ci::XmlTree& node = doc.getChild( "MarionettZoo" );

		if( node.hasChild( "Actions" ))
		{
			ci::XmlTree& nodeActions = doc.getChild( "Actions" );

			for( auto it = nodeActions.begin(); it != nodeActions.end(); ++it )
			{
				if( node.hasChild( "Action" ))
				{
					loadAction( doc.getChild( "Action" ) );
				}
			}
		}
	}

	void Model::loadAction( const ci::XmlTree& node )
	{
		std::string name = node.getAttributeValue< std::string >( "name", "" );

		ActionRef action = ActionRef( new Action( name ) );

		for( auto child = node.begin(); child != node.end(); ++child )
		{
			if( child->getTag() == "Bone" )
			{
				std::string nameBone = child->getAttributeValue<std::string>( "name", "" );
				float       rotateX  = child->getAttributeValue<float>( "rotateX", 0.0 );
				float       rotateY  = child->getAttributeValue<float>( "rotateY", 0.0 );
				float       rotateZ  = child->getAttributeValue<float>( "rotateZ", 0.0 );

				BoneRef   bone    = getBone( nameBone );
				ci::Vec3f impulse = ci::Vec3f( rotateX, rotateY, rotateZ );

				action->addAnimation( bone, impulse );
			}
			else if( child->getTag() == "sound")
			{
				action->addSound( child->getValue() );
			}
		}

		mActions.push_back( action );
	}

	BoneRef Model::getBone( const std::string& name )
	{
		for( auto it = mBones.begin(); it != mBones.end(); ++it )
		{
			BoneRef bone = *it;

			if( name == bone->getNode()->getName() )
				return bone;
		}

		assert( 0 );

		return BoneRef();
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

	Bone::Bone( Model* owner, const mndl::assimp::AssimpNodeRef& node, btRigidBody* rigidBody )
		: mOwner( owner )
		, mNode( node )
		, mRigidBody( rigidBody )
		, mTransform()
	{
		/*
		const btTransform& centerOfMassTransform = rigidBody->getCenterOfMassTransform();

		ci::Vec3f posBullet = fromBullet( centerOfMassTransform.getOrigin()   );
		ci::Quatf rotBullet = fromBullet( centerOfMassTransform.getRotation() );

		ci::Vec3f posAssimp = node->convertLocalToWorldPosition(    node->getPosition()    );
		ci::Quatf rotAssimp = node->convertLocalToWorldOrientation( node->getOrientation() );

		ci::Vec3f posAssimp2 = node->getDerivedPosition();
		ci::Quatf rotAssimp2 = node->getDerivedOrientation();
		*/

		// TODO get the transformation between AssimpNode and RigidBody calculate mTransform
// 		mTransform = ???
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

	mndl::assimp::AssimpNodeRef Bone::getNode() const
	{
		return mNode;
	}

	btRigidBody* Bone::getRigidBody() const
	{
		return mRigidBody;
	}

	ci::Matrix44f Bone::getTransform() const
	{
		return mTransform;
	}

	// synchronize the bullet world to graphics
	void Bone::synchronize()
	{
		const btTransform& centerOfMassTransform = getRigidBody()->getCenterOfMassTransform();

		ci::Vec3f pos = fromBullet( centerOfMassTransform.getOrigin()   );
		ci::Quatf rot = fromBullet( centerOfMassTransform.getRotation() );

		// TODO use mTransform in synchronize

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

	String::String( Model* owner, const mndl::assimp::AssimpNodeRef& node, btSoftBody* softBody )
		: mOwner( owner )
		, mNode( node )
		, mSoftBody( softBody )
	{
	}

	String::~String()
	{
		mOwner->getBulletWorld()->removeSoftBody( mSoftBody );

		delete mSoftBody;
	}

	mndl::assimp::AssimpNodeRef String::getNode() const
	{
		return mNode;
	}

	btSoftBody* String::getSoftBody() const
	{
		return mSoftBody;
	}

	// synchronize the bullet world to graphics
	void String::synchronize()
	{
// 		mndl::assimp::AssimpMeshRef mesh = *getNode()->mMeshes.begin();
// 		std::vector< ci::Vec3f >& vertices = mesh->mCachedTriMesh.getVertices();

#if 0
		int size = getSoftBody()->m_nodes.size();
		for( int pos = 0; pos < size; ++pos )
		{
			btSoftBody::Node& node = getSoftBody()->m_nodes.at( pos );

			// TODO do something with the SoftBody vertex positions. (update the mesh vertices and normals)
			ci::Vec3f vertexPosition = fromBullet( node.m_x );

//			vertexPosition = getNode()->convertWorldToLocalPosition( vertexPosition );
//			vertices[ pos ] = vertexPosition;
		}
#endif
	}

	Action::Action( const std::string& name )
		: mName( name )
		, mAnimations()
		, mSounds()
	{
	}

	Action::~Action()
	{
		// do nothing
	}

	const std::string& Action::getName() const
	{
		return mName;
	}

	void Action::doAction()
	{
		if( ! getNumSounds() )
			GlobalData::getSingleton().mAudio.play( mSounds[ ci::Rand::randInt( mSounds.size() ) ] );

		for( auto it = mAnimations.begin(); it != mAnimations.end(); ++it )
		{
			BoneRef   bone    = it->first;
			ci::Vec3f impulse = it->second;

			// TODO need to check when we have test data
			ci::Vec3f relPos = ci::Vec3f::yAxis() * bone->getNode()->getDerivedOrientation();
			ci::Vec3f force  = relPos.cross( impulse );
			bone->getRigidBody()->applyTorqueImpulse( toBullet( force ) );
		}
	}

	void Action::addAnimation( const BoneRef& bone, const ci::Vec3f& impulse )
	{
		mAnimations[ bone ] = impulse;
	}

	unsigned int Action::getNumAnimations() const
	{
		return mAnimations.size();
	}

	const Action::Animations& Action::getAnimations() const
	{
		return mAnimations;
	}

	void Action::addSound( const std::string& soundName )
	{
		mSounds.push_back( soundName );
	}

	unsigned int Action::getNumSounds() const
	{
		return mSounds.size();
	}

	const Action::Sounds& Action::getSounds() const
	{
		return mSounds;
	}

} // namespace btd
