#pragma once

#include "cinder/Vector.h"
#include "cinder/Quaternion.h"
#include "LinearMath/btVector3.h"
#include "LinearMath/btQuaternion.h"

namespace btd
{
	inline btVector3 toBullet( const ci::Vec3f& vector )
	{
		return btVector3( vector.x, vector.y, vector.z );
	}

	inline btQuaternion toBullet( const ci::Quatf& quaternion )
	{
		return btQuaternion( toBullet( quaternion.getAxis() ), quaternion.getAngle() );
	}

	inline ci::Vec3f fromBullet( const btVector3& vector )
	{
		return ci::Vec3f( vector.x(), vector.y(), vector.z() );
	}

	inline ci::Quatf fromBullet( const btQuaternion& quaternion )
	{
		return ci::Quatf( fromBullet( quaternion.getAxis() ), quaternion.getAngle() );
	}

} // namespace btd
