#ifndef __BullePicker_H__
#define __BullePicker_H__

// TODO find out the correct header for shared_ptr
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
		BulletPicker( btRigidBody* rigidBody, btVector3& pivot, float distance, ci::Vec3f& position );
		~BulletPicker();

		void update( const ci::Ray& ray );

		btPoint2PointConstraint* getConstraint() const;

	protected:
		btPoint2PointConstraint* mConstraint;
		float                    mDistance;
		ci::Vec3f                mPosition;
	};

} // namespace btd

#endif // __BullePicker_H__
