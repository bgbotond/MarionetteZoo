#include "cinder/Vector.h"

#include "BulletDynamics/Dynamics/btRigidBody.h"
#include "BulletDynamics/ConstraintSolver/btPoint2PointConstraint.h"
#include "BulletConverter.h"
#include "BulletPicker.h"

namespace btd
{
	BulletPicker::BulletPicker( btRigidBody* rigidBody, const btVector3& pivot, float distance, const ci::Vec3f& position )
		: mConstraint( 0 )
		, mDistance( distance )
		, mPosition( position )
	{
		mConstraint = new btPoint2PointConstraint( *rigidBody, pivot );
	}

	BulletPicker::~BulletPicker()
	{
		if( mConstraint != 0 )
		{
			delete mConstraint;
			mConstraint = 0;
		}

		mDistance = 0.0f;
		mPosition = ci::Vec3f::zero();
	}

	void BulletPicker::update( const ci::Ray &ray )
	{
		mPosition = ray.getOrigin() + ray.getDirection().normalized() * mDistance;
		mConstraint->setPivotB( toBullet( mPosition ) );
	}

	btPoint2PointConstraint* BulletPicker::getConstraint() const
	{
		return mConstraint;
	}

} // namespace btd
