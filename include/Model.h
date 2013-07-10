#pragma once

#include <vector>
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

	class Model
	{
		typedef std::vector< BoneRef       > Bones;
		typedef std::vector< ConstraintRef > Constraints;
		typedef std::vector< StringRef     > Strings;

	public:
		Model( const BulletWorldRef& bulletWorld, const ci::Vec3f& worldOffset, const std::string& type );
		~Model();

		void update( const ci::Vec3f& pos, const ci::Vec3f& dir, const ci::Vec3f& norm );
		void draw();

		BulletWorldRef& getBulletWorld();

	protected:
		void loadBullet( const ci::fs::path& bulletFile );
		void handleLoadRigidBodies( btSoftBulletWorldImporter* worldImporter );
		void handleLoadConstraints( btSoftBulletWorldImporter* worldImporter );
		void handleLoadSoftBodies(  btSoftBulletWorldImporter* worldImporter );

		BoneRef getBone( btRigidBody* rigidBody );

	protected:
		BulletWorldRef                mBulletWorld;

		mndl::assimp::AssimpLoaderRef mAssimpLoader;

		Bones                         mBones;
		Constraints                   mConstraints;
		Strings                       mStrings;
	};

	// TODO make weak_ptr from Model*
	class Bone
	{
	public:
		Bone( Model* owner, const mndl::NodeRef& node, btRigidBody* rigidBody );
		~Bone();

		mndl::NodeRef     getNode() const;
		btRigidBody*      getRigidBody() const;

		void              synchronize();

	protected:
		Model*            mOwner;
		mndl::NodeRef     mNode;
		btRigidBody*      mRigidBody;
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

//		mndl::NodeRef     getNode() const;
		btSoftBody*       getSoftBody() const;

		void              synchronize();

	protected:
		Model*            mOwner;
//		mndl::NodeRef     mNode;
		btSoftBody*       mSoftBody;
	};

	// TODO make other class for CrossBar

} // namespace btd
