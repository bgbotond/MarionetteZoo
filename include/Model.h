#pragma once

#include <vector>
#include <map>
#include "AssimpLoader.h"
#include "Node.h"
#include "BulletWorld.h"

#include "mndlkit/params/PParams.h"

class btSoftBulletWorldImporter;
class RigidBody;
class btTypedConstraint;

namespace btd
{
	typedef std::shared_ptr< class Model      > ModelRef;
	typedef std::shared_ptr< class Bone       > BoneRef;
	typedef std::shared_ptr< class Constraint > ConstraintRef;
	typedef std::shared_ptr< class String     > StringRef;
	typedef std::shared_ptr< class Action     > ActionRef;

	class Model
	{
		typedef std::vector< BoneRef       > Bones;
		typedef std::vector< ConstraintRef > Constraints;
		typedef std::vector< StringRef     > Strings;
		typedef std::vector< ActionRef     > Actions;

	public:
		Model( const BulletWorldRef& bulletWorld, const ci::Vec3f& worldOffset, const std::string& type );
		~Model();

		void update( const ci::Vec3f& pos, const ci::Vec3f& dir, const ci::Vec3f& norm );
		void draw();

		BulletWorldRef& getBulletWorld();

		unsigned int getNumActions() const;
		std::vector< std::string > getNameActions();
		void doAction( unsigned int pos );

	protected:
		void loadBullet( const ci::fs::path& bulletFile );
		void handleLoadRigidBodies( btSoftBulletWorldImporter* worldImporter );
		void handleLoadConstraints( btSoftBulletWorldImporter* worldImporter );
		void handleLoadSoftBodies(  btSoftBulletWorldImporter* worldImporter );

		void loadActions( const ci::fs::path& actionFile );
		void loadAction( const ci::XmlTree& node );

		BoneRef getBone( const std::string& name );
		BoneRef getBone( btRigidBody* rigidBody );

	protected:
		BulletWorldRef                mBulletWorld;

		mndl::assimp::AssimpLoaderRef mAssimpLoader;

		Bones                         mBones;
		Constraints                   mConstraints;
		Strings                       mStrings;
		Actions                       mActions;
	};

	// TODO make weak_ptr from Model*
	class Bone
	{
	public:
		Bone( Model* owner, const mndl::assimp::AssimpNodeRef& node, btRigidBody* rigidBody );
		~Bone();

		mndl::assimp::AssimpNodeRef getNode() const;
		btRigidBody*                getRigidBody() const;
		ci::Matrix44f               getTransform() const;

		void                        synchronize();

	protected:
		Model*                      mOwner;
		mndl::assimp::AssimpNodeRef mNode;
		btRigidBody*                mRigidBody;

		ci::Matrix44f               mTransform; // A transformation that converts the rigid body center of mass to the AssimpNode center position
	};

	// TODO make weak_ptr from Model*
	class Constraint
	{
	public:
		Constraint( Model* owner, const BoneRef& boneA, const BoneRef& boneB, btTypedConstraint* constraint );
		~Constraint();

		BoneRef            getBoneA() const;
		BoneRef            getBoneB() const;
		btTypedConstraint* getConstraint() const;

	protected:
		Model*             mOwner;
		BoneRef            mBoneA;
		BoneRef            mBoneB;
		btTypedConstraint* mConstraint;
	};

	// TODO make weak_ptr from Model*
	class String
	{
	public:
		String( Model* owner, btSoftBody* softBody );
		~String();

		btSoftBody* getSoftBody() const;

		void draw() const;

	protected:
		Model*                      mOwner;
		btSoftBody*                 mSoftBody;
	};

	class Action
	{
	public:
		typedef std::map< BoneRef, ci::Vec3f > Animations; // Bone and impulse pair
		typedef std::vector< std::string >     Sounds;

	public:
		Action( const std::string& name );
		~Action();

		const std::string& getName() const;

		void               doAction();

		void               addAnimation( const BoneRef& bone, const ci::Vec3f& impulse );
		unsigned int       getNumAnimations() const;
		const Animations&  getAnimations() const;

		void               addSound( const std::string& soundName );
		unsigned int       getNumSounds() const;
		const Sounds&      getSounds() const;

	protected:
		std::string     mName;
		Animations      mAnimations;
		Sounds          mSounds;
	};

	// TODO make other class for CrossBar (or can we use the Bone class for it?)

} // namespace btd
