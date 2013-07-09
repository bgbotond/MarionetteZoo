#pragma once

#include <vector>
#include "AssimpLoader.h"
#include "Node.h"
#include "BulletWorld.h"

#include "mndlkit/params/PParams.h"

class btBulletWorldImporter;
class RigidBody;
class btTypedConstraint;

namespace btd
{
	typedef std::shared_ptr< class Model      > ModelRef;
	typedef std::shared_ptr< class Bone       > BoneRef;
	typedef std::shared_ptr< class Constraint > ConstraintRef;

	class Model
	{
	public:
		Model( const BulletWorldRef& bulletWorld, const ci::Vec3f& worldOffset, const std::string& type );
		~Model();

		void update( const ci::Vec3f& pos, const ci::Vec3f& dir, const ci::Vec3f& norm );
		void draw();

		BulletWorldRef& getBulletWorld();

	protected:
		void loadBullet( const ci::fs::path& bulletFile );
		void handleLoadRigidBodies( btBulletWorldImporter* worldImporter );
		void handleLoadConstraints( btBulletWorldImporter* worldImporter );

		BoneRef getBone( btRigidBody* rigidBody );

	protected:
		BulletWorldRef                mBulletWorld;

		mndl::assimp::AssimpLoaderRef mAssimpLoader;

		std::vector< BoneRef >        mBones;
		std::vector< ConstraintRef >  mConstraints;
	};

	// TODO make class to subclass of Model
	class Bone
	{
	public:
		Bone( const ModelRef& owner, const mndl::NodeRef& node, btRigidBody* rigidBody );
		~Bone();

		mndl::NodeRef     getNode() const;
		btRigidBody*      getRigidBody() const;

		void              update();

	protected:
		ModelRef          mOwner;
		mndl::NodeRef     mNode;
		btRigidBody*      mRigidBody;
	};

	class Constraint
	{
	public:
		Constraint( const ModelRef& owner, const BoneRef& boneA, const BoneRef& boneB, btTypedConstraint* constraint );
		~Constraint();

		BoneRef            getBoneA() const;
		BoneRef            getBoneB() const;
		btTypedConstraint* getConstraint() const;

	protected:
		ModelRef           mOwner;
		BoneRef            mBoneA;
		BoneRef            mBoneB;
		btTypedConstraint* mConstraint;
	};

	// TODO make other class for String and CrossBar

} // namespace btd
