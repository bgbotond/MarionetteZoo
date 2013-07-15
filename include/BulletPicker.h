#pragma once

#include <memory>
#include <vector>
#include "cinder/Ray.h"
#include "cinder/Vector.h"
#include "LinearMath/btVector3.h"

class btRigidBody;
class btPoint2PointConstraint;

namespace btd
{
	typedef std::shared_ptr< class BulletPicker > BulletPickerRef;

	class BulletPicker
	{
	public:
		BulletPicker( btRigidBody* rigidBody, const btVector3& pivot, float distance, const ci::Vec3f& position );
		~BulletPicker();

		void update( const ci::Ray& ray );

		btPoint2PointConstraint* getConstraint() const;

	protected:
		btPoint2PointConstraint* mConstraint;
		float                    mDistance;
		ci::Vec3f                mPosition;
	};

} // namespace btd
