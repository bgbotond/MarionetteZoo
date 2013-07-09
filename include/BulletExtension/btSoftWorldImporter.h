/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2012 Erwin Coumans  http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/


#ifndef SOFT_BULLET_WORLD_IMPORTER_H
#define SOFT_BULLET_WORLD_IMPORTER_H

#include "btBulletDynamicsCommon.h"
#include "BulletSoftBody/btSoftBody.h"
#include "BulletSoftBody/btSoftBodyData.h"
#include "BulletSoftBody/btSoftRigidDynamicsWorld.h"
#include "Extras/Serialize/BulletFileLoader/bFile.h"
#include "Extras/Serialize/BulletFileLoader/btBulletFile.h"
#include "Extras/Serialize/BulletWorldImporter/btBulletWorldImporter.h"

///The btSoftBulletWorldImporter is a starting point to import .bullet files containing SoftBody objects.
class btSoftBulletWorldImporter : public btBulletWorldImporter
{
	btSoftRigidDynamicsWorld*                   m_softRigidWorld;

	btHashMap<btHashPtr,btSoftBody::Material*>  m_materialMap;

	btHashMap<btHashPtr,btSoftBody*>            m_clusterBodyMap;
	btHashMap<btHashPtr,btSoftBody*>            m_softBodyMap;

public:

	btSoftBulletWorldImporter( btSoftRigidDynamicsWorld* world );
	virtual ~btSoftBulletWorldImporter();

	virtual bool convertAllObjects( bParse::btBulletFile* bulletFile );

	int getNumSoftBodies() const;
	btSoftBody* getSoftBodyByIndex( int index ) const;
};


#endif //SOFT_BULLET_WORLD_IMPORTER_H
