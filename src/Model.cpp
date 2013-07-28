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

#include "BulletSoftBody/btSoftBodyHelpers.h"


namespace btd
{

Model::Model( const BulletWorldRef& bulletWorld, const ci::Vec3f& worldOffset, const std::string& type )
	: mBulletWorld( bulletWorld )
{
	mAssimpLoader = mndl::assimp::AssimpLoaderRef( new mndl::assimp::AssimpLoader( ModelFileManager::getSingleton().getModelFile( type ) ) );
	mAssimpLoader->enableSkinning( true );
	//mAssimpLoader->enableAnimation( true );

	// TODO apply the worldOffset to place it in special position

	loadBullet( ModelFileManager::getSingleton().getBulletFile( type ) );
	loadActions( ModelFileManager::getSingleton().getActionFile( type ) );

	printNodeInfo( mAssimpLoader->getRootNode() );
}

Model::~Model()
{
	// do nothing
}

void Model::printNodeInfo( const mndl::NodeRef &node, int level /* = 0 */ )
{
	std::string indent = std::string( level, '-' );
	indent += indent + " ";
	ci::app::console() << indent << node->getName() << std::endl;
	ci::app::console() << indent << node->getPosition() << std::endl;
	ci::app::console() << indent << node->getOrientation() << std::endl;
	auto children = node->getChildren();
	for ( auto chIt = children.begin(); chIt != children.end(); ++chIt )
	{
		printNodeInfo( *chIt, level + 1 );
	}
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

void Model::draw( const ci::CameraPersp &camera )
{
	if( mBulletWorld->getBulletParameter()->mDrawSkin )
	{
		if ( mBulletWorld->getBulletParameter()->mEnableWireframe )
			ci::gl::enableWireframe();

		mAssimpLoader->draw();

		if ( mBulletWorld->getBulletParameter()->mEnableWireframe )
			ci::gl::disableWireframe();
	}

	// draw soft body strings
	for ( auto it = mStrings.begin(); it != mStrings.end(); ++it )
	{
		(*it)->draw();
	}

	// skeleton
	if ( mBulletWorld->getBulletParameter()->mDrawSkeleton )
	{
		ci::gl::disableDepthRead();
		ci::gl::disableDepthWrite();

		drawSkeleton( camera );

		ci::gl::enableDepthRead();
		ci::gl::enableDepthWrite();
	}
}

void Model::drawSkeleton( const ci::CameraPersp &camera )
{
	// collect bones
	std::map< std::string, mndl::assimp::AssimpNodeRef > bones;
	const aiScene *scene = mAssimpLoader->getAiScene();
	for ( unsigned i = 0; i < scene->mNumMeshes; ++i )
	{
		const aiMesh *mesh = scene->mMeshes[ i ];
		for ( unsigned j = 0; j < mesh->mNumBones; ++j )
		{
			const aiBone *bone = mesh->mBones[ j ];

			std::string boneName = mndl::assimp::fromAssimp( bone->mName );
			mndl::assimp::AssimpNodeRef nodeRef = mAssimpLoader->getAssimpNode( boneName );
			bones[ boneName ] = nodeRef;
		}
	}

	std::vector< mndl::NodeRef > labelNodes;

	glLineWidth( 3.f );
	ci::gl::color( ci::Color( 1, .9, .1 ) );
	for ( auto it = bones.begin(); it != bones.end(); ++it )
	{
		ci::Vec3f p0 = it->second->getDerivedPosition();
		ci::gl::drawSphere( p0, .1f, 4 );
		mndl::NodeRef parent = it->second->getParent();
		if ( parent )
		{
			auto parentBoneIt = bones.find( parent->getName() );
			if ( parentBoneIt != bones.end() )
			{
				ci::Vec3f p1 = parentBoneIt->second->getDerivedPosition();
				ci::gl::drawLine( p0, p1 );
			}
		}
		auto children = it->second->getChildren();
		if ( children.size() == 1 )
		{
			mndl::NodeRef endNode = children.front();
			auto endBoneIt = bones.find( endNode->getName() );
			if ( endBoneIt == bones.end() ) // NULL bones are not listed as bones in assimp, only as nodes
			{
				ci::Vec3f p1 = children.front()->getDerivedPosition();
				ci::gl::drawLine( p0, p1 );

				labelNodes.push_back( endNode );
			}
		}

		labelNodes.push_back( it->second );
	}

	// draw labels
	ci::gl::pushMatrices();
	ci::Vec2i size = ci::app::getWindowSize();
	ci::gl::setMatricesWindow( size );
	for ( auto it = labelNodes.cbegin(); it != labelNodes.cend(); ++it )
	{
		ci::Vec2f labelPos = camera.worldToScreen(
				(*it)->getDerivedPosition(), size.x, size.y );
		ci::gl::drawString( (*it)->getName(), labelPos );
	}
	ci::gl::popMatrices();

	glLineWidth( 1.f );
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

	worldImporter.loadFile( bulletFile.string().c_str() );

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

		if ( rigidBody->getMotionState() == NULL )
		{
			btDefaultMotionState *motionState = new btDefaultMotionState();
			motionState->setWorldTransform( rigidBody->getCenterOfMassTransform() );
			rigidBody->setMotionState( motionState );
		}

		mBulletWorld->setupRigidBody( rigidBody );
		mBulletWorld->addRigidBody( rigidBody );

		// All rigid body names should match with the .dae file bones without the rb_ prefix
		std::string nodeName = name.replace( 0, 3, "" );
		mndl::assimp::AssimpNodeRef node = mAssimpLoader->getAssimpNode( nodeName );

		if ( !node )
		{
			ci::app::console() << "node not found for name" << nodeName << std::endl;
		}
		else
		{
			BoneRef bone = BoneRef( new Bone( this, node, rigidBody ) );
			mBones.push_back( bone );
		}
	}

	// sort bones according to level, parents should be processed first
	std::sort( mBones.begin(), mBones.end(), []( BoneRef b0, BoneRef b1 ) { return b0->getLevel() < b1->getLevel(); } );
	/*
	for( auto it = mBones.begin(); it != mBones.end(); ++it )
	{
		ci::app::console() << (*it)->getNode()->getName() << " " << (*it)->getLevel() << std::endl;
	}
	*/
}

void Model::handleLoadConstraints( btSoftBulletWorldImporter* worldImporter )
{
	for( int constraintIdx = 0; constraintIdx < worldImporter->getNumConstraints(); constraintIdx++ )
	{
		btTypedConstraint* typedConstraint = worldImporter->getConstraintByIndex( constraintIdx );

		mBulletWorld->setupConstraint( typedConstraint );
		mBulletWorld->addConstraint( typedConstraint );

		btRigidBody* rigidBodyA = &typedConstraint->getRigidBodyA();
		btRigidBody* rigidBodyB = &typedConstraint->getRigidBodyB();

		BoneRef boneA = getBone( rigidBodyA );
		BoneRef boneB = getBone( rigidBodyB );
		/* there are constraints that are connected to a rigid and a soft
		 * body in Blender, in which case one of the rigid body pointers is
		 * invalid. Skip these constraints */
		if ( boneA && boneB )
		{
			/* FIXME: why is this necessary, mConstraints is not used
			 * anywhere, since the rigid bodies control the connected
			 * bones, we are not dealing with constraints */
			ConstraintRef constraint = ConstraintRef( new Constraint( this, boneA, boneB, typedConstraint ) );
			mConstraints.push_back( constraint );
		}
	}
}

void Model::handleLoadSoftBodies( btSoftBulletWorldImporter* worldImporter )
{
	for( int softBodyIdx = 0; softBodyIdx < worldImporter->getNumSoftBodies(); softBodyIdx++ )
	{
		btSoftBody* softBody = worldImporter->getSoftBodyByIndex( softBodyIdx );

#if 0
		 /* this part is not needed, since we render the strings instead
		  * of controlling an assimp mesh */

		// the SoftBody names are not loaded by the worldImporter
		// TODO load the SoftBody names
		// std::string name = worldImporter->getNameForPointer( softBody );

		// it should link two RigidBodies
		assert( softBody->m_anchors.size() == 2 );

		// RigidBodies are rb_X and rb_Y
		btRigidBody *rb0 = softBody->m_anchors[ 0 ].m_body;
		btRigidBody *rb1 = softBody->m_anchors[ 1 ].m_body;
		std::string name0 = worldImporter->getNameForPointer( rb0 );
		std::string name1 = worldImporter->getNameForPointer( rb1 );

		// soft body name is sb_X-Y, in which X and Y are in alphabetical order
		std::string sbName;
		if ( name0 < name1 )
			sbName = "sb_" + name0 + name1;
		else
			sbName = "sb_" + name1 + name0;

		nodeString = mAssimpLoader->getAssimpNode( sbName );
		assert( nodeString );
		assert( nodeString->mMeshes.size() == 1 ); // should contain only one mesh
		mndl::assimp::AssimpMeshRef mesh = *nodeString->mMeshes.begin();
		//assert( softBody->m_nodes.size() == mesh->mAnimatedPos.size() ); // softBody vertex size should be the same as node mesh vertex size
#endif

		mBulletWorld->setupSoftBody( softBody );
		mBulletWorld->addSoftBody( softBody );
		StringRef string = StringRef( new String( this, softBody ) );
		mStrings.push_back( string );
	}

#if 0
	// test rope
	btSoftBodyWorldInfo &worldInfo = mBulletWorld->getDynamicsWorld()->getWorldInfo();
	btSoftBody *rope = btSoftBodyHelpers::CreateRope( worldInfo,
			btVector3( 4, 0, 0 ), btVector3( 4, 0, -38 ),
			mBulletWorld->getBulletParameter()->mRopePart, 2 );
	btRigidBody *rbFrom = worldImporter->getRigidBodyByName( "rb_cross" );
	btRigidBody *rbTo = worldImporter->getRigidBodyByName( "rb_head" );
	ci::app::console() << std::hex << rbFrom << " " << rbTo << std::endl;
	rope->appendAnchor( 0, rbFrom );
	rope->appendAnchor( rope->m_nodes.size() - 1, rbTo );

	mBulletWorld->setupSoftBody( rope );
	mBulletWorld->addSoftBody( rope, true );
#endif
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

			BoneRef bone = getBone( nameBone );
			ci::Vec3f impulse = ci::Vec3f( rotateX, rotateY, rotateZ );

			if ( bone )
			{
				action->addAnimation( bone, impulse );
			}
			else
			{
				ci::app::console() << "unknown bone in action: " << nameBone << std::endl;
			}

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

	return BoneRef();
}

Bone::Bone( Model* owner, const mndl::assimp::AssimpNodeRef& node, btRigidBody* rigidBody )
	: mOwner( owner )
	, mNode( node )
	, mRigidBody( rigidBody )
{
	// transformation from RigidBody center of mass to AssimpNode
	const btTransform& centerOfMassTransform = rigidBody->getCenterOfMassTransform();
	ci::Vec3f posBullet = fromBullet( centerOfMassTransform.getOrigin() );
	ci::Quatf rotBullet = fromBullet( centerOfMassTransform.getRotation() );

	ci::Vec3f posAssimp = node->getDerivedPosition();
	ci::Quatf rotAssimp = node->getDerivedOrientation();

	mBoneTranslation = ( posAssimp - posBullet );
	// NOTE: Quaternion::inverted() and invert() is wrong, because they use the
	// axis, radian setter with v, which is not the axis
	// Quaternion.inverse() seems to be fine
	mBoneOrientation = rotBullet.inverse() * rotAssimp;

	mLevel = 0;
	mndl::NodeRef n = node;
	while ( ( n = n->getParent() ) )
	{
		mLevel++;
	}

	/*
	ci::app::console() << node->getName() <<
		" bullet: \t" << posBullet << " \t" << rotBullet << std::endl <<
		node->getName() << " assimp: \t" << posAssimp << " \t" << rotAssimp << std::endl <<
		" level: " << mLevel << std::endl;
	ci::app::console() << mBoneTranslation << ", " << mBoneOrientation << std::endl;
	*/
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
	const btTransform& centerOfMassTransform = mRigidBody->getCenterOfMassTransform();

	ci::Vec3f pos = fromBullet( centerOfMassTransform.getOrigin() );
	ci::Quatf ori = fromBullet( centerOfMassTransform.getRotation() );

	// apply bullet to node offset and rotation
	pos += mBoneTranslation * mBoneOrientation * ori;

	mndl::NodeRef parent = mNode->getParent();
	ci::Quatf nodeOri;
	ci::Vec3f nodePos;
	if ( parent )
	{
		nodePos = parent->convertWorldToLocalPosition( pos );
		nodeOri = parent->convertWorldToLocalOrientation( ori );
	}
	else
	{
		nodePos = pos;
		nodeOri = ori;
	}

	mNode->setOrientation( nodeOri );
	mNode->setPosition( nodePos );
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

void String::draw() const
{
	ci::gl::color( ci::Color::white() );
	ci::gl::begin( GL_LINES );

	btSoftBody::tLinkArray &links = mSoftBody->m_links;

	for ( int i = 0; i < links.size(); i++ )
	{
		btSoftBody::Node *node0 = links[ i ].m_n[ 0 ];
		btSoftBody::Node *node1 = links[ i ].m_n[ 1 ];
		ci::gl::vertex( fromBullet( node0->m_x ) );
		ci::gl::vertex( fromBullet( node1->m_x ) );
	}
	ci::gl::end();
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
