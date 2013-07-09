#include "BulletExtension/btSoftWorldImporter.h"

btSoftBulletWorldImporter::btSoftBulletWorldImporter( btSoftRigidDynamicsWorld* world )
	: btBulletWorldImporter( world )
	, m_softRigidWorld( world )
{
}

btSoftBulletWorldImporter::~btSoftBulletWorldImporter()
{
}

bool btSoftBulletWorldImporter::convertAllObjects( bParse::btBulletFile* bulletFile )
{
	bool result = btBulletWorldImporter::convertAllObjects( bulletFile );
	int i;
	//now the soft bodies
	for (i=0;i<bulletFile->m_softBodies.size();i++)
	{
		if (bulletFile->getFlags() & bParse::FD_DOUBLE_PRECISION)
		{
			btAssert(0); //not yet
			//btSoftBodyFloatData* softBodyData = (btSoftBodyFloatData*)bulletFile->m_softBodies[i];
		} else
		{
			btSoftBodyFloatData* softBodyData = (btSoftBodyFloatData*)bulletFile->m_softBodies[i];
			int i;
			int numNodes = softBodyData->m_numNodes;
				
		
			btSoftBody*		psb=new btSoftBody(&m_softRigidWorld->getWorldInfo());
			m_softBodyMap.insert(softBodyData,psb);

			//materials
			for (i=0;i<softBodyData->m_numMaterials;i++)
			{
				SoftBodyMaterialData* matData = softBodyData->m_materials[i];
				btSoftBody::Material** matPtr = m_materialMap.find(matData);
				btSoftBody::Material* mat = 0;
				if (matPtr&& *matPtr)
				{
					mat = *matPtr;
				} else
				{
					mat = psb->appendMaterial();
					mat->m_flags = matData->m_flags;
					mat->m_kAST = matData->m_angularStiffness;
					mat->m_kLST = matData->m_linearStiffness;
					mat->m_kVST = matData->m_volumeStiffness;
					m_materialMap.insert(matData,mat);
				}
			}




			for (i=0;i<numNodes;i++)
			{
				SoftBodyNodeData& nodeData = softBodyData->m_nodes[i];
				btVector3 position;
				position.deSerializeFloat(nodeData.m_position);
				btScalar mass = nodeData.m_inverseMass? 1./nodeData.m_inverseMass : 0.f;
				psb->appendNode(position,mass);
				btSoftBody::Node* node = &psb->m_nodes[psb->m_nodes.size()-1];
				node->m_area = nodeData.m_area;
				node->m_battach = nodeData.m_attach;
				node->m_f.deSerializeFloat(nodeData.m_accumulatedForce);
				node->m_im = nodeData.m_inverseMass;

				btSoftBody::Material** matPtr = m_materialMap.find(nodeData.m_material);
				if (matPtr && *matPtr)
				{
					node->m_material = *matPtr;
				} else
				{
					printf("no mat?\n");
				}
					
				node->m_n.deSerializeFloat(nodeData.m_normal);
				node->m_q = node->m_x;
				node->m_v.deSerializeFloat(nodeData.m_velocity);
					
			}

			for (i=0;i<softBodyData->m_numLinks;i++)
			{
				SoftBodyLinkData& linkData = softBodyData->m_links[i];
				btSoftBody::Material** matPtr = m_materialMap.find(linkData.m_material);
				if (matPtr && *matPtr)
				{
					psb->appendLink(linkData.m_nodeIndices[0],linkData.m_nodeIndices[1],*matPtr);
				} else
				{
					psb->appendLink(linkData.m_nodeIndices[0],linkData.m_nodeIndices[1]);
				}
				btSoftBody::Link* link = &psb->m_links[psb->m_links.size()-1];
				link->m_bbending = linkData.m_bbending;
				link->m_rl = linkData.m_restLength;
			}

			for (i=0;i<softBodyData->m_numFaces;i++)
			{
				SoftBodyFaceData& faceData = softBodyData->m_faces[i];
				btSoftBody::Material** matPtr = m_materialMap.find(faceData.m_material);
				if (matPtr && *matPtr)
				{
					psb->appendFace(faceData.m_nodeIndices[0],faceData.m_nodeIndices[1],faceData.m_nodeIndices[2],*matPtr);
				} else
				{
					psb->appendFace(faceData.m_nodeIndices[0],faceData.m_nodeIndices[1],faceData.m_nodeIndices[2]);
				}
				btSoftBody::Face* face = &psb->m_faces[psb->m_faces.size()-1];
				face->m_normal.deSerializeFloat(faceData.m_normal);
				face->m_ra = faceData.m_restArea;
			}

			

			//anchors
			for (i=0;i<softBodyData->m_numAnchors;i++)
			{
				btCollisionObject** colAptr = m_bodyMap.find(softBodyData->m_anchors[i].m_rigidBody);
				if (colAptr && *colAptr)
				{
					btRigidBody* body = btRigidBody::upcast(*colAptr);
					if (body)
					{
						bool disableCollision = false;
						btVector3 localPivot;
						localPivot.deSerializeFloat(softBodyData->m_anchors[i].m_localFrame);
						psb->appendAnchor(softBodyData->m_anchors[i].m_nodeIndex,body,localPivot, disableCollision);
					}
				}
			}

			if (softBodyData->m_pose)
			{
				psb->m_pose.m_aqq.deSerializeFloat(  softBodyData->m_pose->m_aqq);
				psb->m_pose.m_bframe = (softBodyData->m_pose->m_bframe!=0);
				psb->m_pose.m_bvolume = (softBodyData->m_pose->m_bvolume!=0);
				psb->m_pose.m_com.deSerializeFloat(softBodyData->m_pose->m_com);
					
				psb->m_pose.m_pos.resize(softBodyData->m_pose->m_numPositions);
				for (i=0;i<softBodyData->m_pose->m_numPositions;i++)
				{
					psb->m_pose.m_pos[i].deSerializeFloat(softBodyData->m_pose->m_positions[i]);
				}
				psb->m_pose.m_rot.deSerializeFloat(softBodyData->m_pose->m_rot);
				psb->m_pose.m_scl.deSerializeFloat(softBodyData->m_pose->m_scale);
				psb->m_pose.m_wgh.resize(softBodyData->m_pose->m_numWeigts);
				for (i=0;i<softBodyData->m_pose->m_numWeigts;i++)
				{
					psb->m_pose.m_wgh[i] = softBodyData->m_pose->m_weights[i];
				}
				psb->m_pose.m_volume = softBodyData->m_pose->m_restVolume;
			}

#if 1
			psb->m_cfg.piterations=softBodyData->m_config.m_positionIterations;
			psb->m_cfg.diterations=softBodyData->m_config.m_driftIterations;
			psb->m_cfg.citerations=softBodyData->m_config.m_clusterIterations;
			psb->m_cfg.viterations=softBodyData->m_config.m_velocityIterations;
				
			//psb->setTotalMass(0.1);
			psb->m_cfg.aeromodel = (btSoftBody::eAeroModel::_)softBodyData->m_config.m_aeroModel;
			psb->m_cfg.kLF = softBodyData->m_config.m_lift;
			psb->m_cfg.kDG = softBodyData->m_config.m_drag;
			psb->m_cfg.kMT = softBodyData->m_config.m_poseMatch;
			psb->m_cfg.collisions = softBodyData->m_config.m_collisionFlags;
			psb->m_cfg.kDF = 1.f;//softBodyData->m_config.m_dynamicFriction;
			psb->m_cfg.kDP = softBodyData->m_config.m_damping;
			psb->m_cfg.kPR = softBodyData->m_config.m_pressure;
			psb->m_cfg.kVC = softBodyData->m_config.m_volume;
			psb->m_cfg.kAHR = softBodyData->m_config.m_anchorHardness;
			psb->m_cfg.kKHR = softBodyData->m_config.m_kineticContactHardness;
			psb->m_cfg.kSHR = softBodyData->m_config.m_softContactHardness;
			psb->m_cfg.kSRHR_CL = softBodyData->m_config.m_softRigidClusterHardness;
			psb->m_cfg.kSKHR_CL = softBodyData->m_config.m_softKineticClusterHardness;
			psb->m_cfg.kSSHR_CL = softBodyData->m_config.m_softSoftClusterHardness;
#endif

//				pm->m_kLST				=	1;

#if 1
			//clusters
			if (softBodyData->m_numClusters)
			{
				m_clusterBodyMap.insert(softBodyData->m_clusters,psb);
				int j;
				psb->m_clusters.resize(softBodyData->m_numClusters);
				for (i=0;i<softBodyData->m_numClusters;i++)
				{
					psb->m_clusters[i] = new(btAlignedAlloc(sizeof(btSoftBody::Cluster),16)) btSoftBody::Cluster();
					psb->m_clusters[i]->m_adamping = softBodyData->m_clusters[i].m_adamping;
					psb->m_clusters[i]->m_av.deSerializeFloat(softBodyData->m_clusters[i].m_av);
					psb->m_clusters[i]->m_clusterIndex = softBodyData->m_clusters[i].m_clusterIndex;
					psb->m_clusters[i]->m_collide = (softBodyData->m_clusters[i].m_collide!=0);
					psb->m_clusters[i]->m_com.deSerializeFloat(softBodyData->m_clusters[i].m_com);
					psb->m_clusters[i]->m_containsAnchor = (softBodyData->m_clusters[i].m_containsAnchor!=0);
					psb->m_clusters[i]->m_dimpulses[0].deSerializeFloat(softBodyData->m_clusters[i].m_dimpulses[0]);
					psb->m_clusters[i]->m_dimpulses[1].deSerializeFloat(softBodyData->m_clusters[i].m_dimpulses[1]);

					psb->m_clusters[i]->m_framerefs.resize(softBodyData->m_clusters[i].m_numFrameRefs);
					for (j=0;j<softBodyData->m_clusters[i].m_numFrameRefs;j++)
					{
						psb->m_clusters[i]->m_framerefs[j].deSerializeFloat(softBodyData->m_clusters[i].m_framerefs[j]);
					}
					psb->m_clusters[i]->m_nodes.resize(softBodyData->m_clusters[i].m_numNodes);
					for (j=0;j<softBodyData->m_clusters[i].m_numNodes;j++)
					{
						int nodeIndex = softBodyData->m_clusters[i].m_nodeIndices[j];
						psb->m_clusters[i]->m_nodes[j] = &psb->m_nodes[nodeIndex];
					}

					psb->m_clusters[i]->m_masses.resize(softBodyData->m_clusters[i].m_numMasses);
					for (j=0;j<softBodyData->m_clusters[i].m_numMasses;j++)
					{
						psb->m_clusters[i]->m_masses[j] = softBodyData->m_clusters[i].m_masses[j];
					}
					psb->m_clusters[i]->m_framexform.deSerializeFloat(softBodyData->m_clusters[i].m_framexform);
					psb->m_clusters[i]->m_idmass = softBodyData->m_clusters[i].m_idmass;
					psb->m_clusters[i]->m_imass = softBodyData->m_clusters[i].m_imass;
					psb->m_clusters[i]->m_invwi.deSerializeFloat(softBodyData->m_clusters[i].m_invwi);
					psb->m_clusters[i]->m_ldamping = softBodyData->m_clusters[i].m_ldamping;
					psb->m_clusters[i]->m_locii.deSerializeFloat(softBodyData->m_clusters[i].m_locii);
					psb->m_clusters[i]->m_lv.deSerializeFloat(softBodyData->m_clusters[i].m_lv);
					psb->m_clusters[i]->m_matching = softBodyData->m_clusters[i].m_matching;
					psb->m_clusters[i]->m_maxSelfCollisionImpulse = 0;//softBodyData->m_clusters[i].m_maxSelfCollisionImpulse;
					psb->m_clusters[i]->m_ndamping = softBodyData->m_clusters[i].m_ndamping;
					psb->m_clusters[i]->m_ndimpulses = softBodyData->m_clusters[i].m_ndimpulses;
					psb->m_clusters[i]->m_nvimpulses = softBodyData->m_clusters[i].m_nvimpulses;
					psb->m_clusters[i]->m_selfCollisionImpulseFactor = softBodyData->m_clusters[i].m_selfCollisionImpulseFactor;
					psb->m_clusters[i]->m_vimpulses[0].deSerializeFloat(softBodyData->m_clusters[i].m_vimpulses[0]);
					psb->m_clusters[i]->m_vimpulses[1].deSerializeFloat(softBodyData->m_clusters[i].m_vimpulses[1]);
						
				}
				//psb->initializeClusters();
				//psb->updateClusters();

			}
#else

			psb->m_cfg.piterations	=	2;
			psb->m_cfg.collisions	=	btSoftBody::fCollision::CL_SS+	btSoftBody::fCollision::CL_RS;
			//psb->setTotalMass(50,true);
			//psb->generateClusters(64);
			//psb->m_cfg.kDF=1;
			psb->generateClusters(8);


#endif //



			psb->updateConstants();
			m_softRigidWorld->getWorldInfo().m_dispatcher = m_softRigidWorld->getDispatcher();
				
			m_softRigidWorld->addSoftBody(psb);


		}
	}


	//now the soft body joints
	for (i=0;i<bulletFile->m_softBodies.size();i++)
	{
		if (bulletFile->getFlags() & bParse::FD_DOUBLE_PRECISION)
		{
			btAssert(0); //not yet
			//btSoftBodyFloatData* softBodyData = (btSoftBodyFloatData*)bulletFile->m_softBodies[i];
		} else
		{
			btSoftBodyFloatData* softBodyData = (btSoftBodyFloatData*)bulletFile->m_softBodies[i];
			btSoftBody** sbp = m_softBodyMap.find(softBodyData);
			if (sbp && *sbp)
			{
				btSoftBody* sb = *sbp;
				for (int i=0;i<softBodyData->m_numJoints;i++)
				{
					btSoftBodyJointData* sbjoint = &softBodyData->m_joints[i];


					btSoftBody::Body bdyB;

					btSoftBody* sbB = 0;
					btTransform transA;
					transA.setIdentity();
					transA = sb->m_clusters[0]->m_framexform;

					btCollisionObject** colBptr = m_bodyMap.find(sbjoint->m_bodyB);
					if (colBptr && *colBptr)
					{
						btRigidBody* rbB = btRigidBody::upcast(*colBptr);
						if (rbB)
						{
							bdyB = rbB;
						} else
						{
							bdyB = *colBptr;
						}
					}


					btSoftBody** bodyBptr = m_clusterBodyMap.find(sbjoint->m_bodyB);
					if (bodyBptr && *bodyBptr )
					{
						sbB = *bodyBptr;
						bdyB = sbB->m_clusters[0];
					}


					if (sbjoint->m_jointType==btSoftBody::Joint::eType::Linear)
					{
						btSoftBody::LJoint::Specs specs;
						specs.cfm = sbjoint->m_cfm;
						specs.erp = sbjoint->m_erp;
						specs.split = sbjoint->m_split;
						btVector3 relA;
						relA.deSerializeFloat(sbjoint->m_refs[0]);
						specs.position = transA*relA;
						sb->appendLinearJoint(specs,sb->m_clusters[0],bdyB);
					}

					if (sbjoint->m_jointType==btSoftBody::Joint::eType::Angular)
					{
						btSoftBody::AJoint::Specs specs;
						specs.cfm = sbjoint->m_cfm;
						specs.erp = sbjoint->m_erp;
						specs.split = sbjoint->m_split;
						btVector3 relA;
						relA.deSerializeFloat(sbjoint->m_refs[0]);
						specs.axis = transA.getBasis()*relA;
						sb->appendAngularJoint(specs,sb->m_clusters[0],bdyB);
					}
				}
			}

		}
	}
	return result;
}

int btSoftBulletWorldImporter::getNumSoftBodies() const
{
	return m_softBodyMap.size();
}

btSoftBody* btSoftBulletWorldImporter::getSoftBodyByIndex( int index ) const
{
	return *m_softBodyMap.getAtIndex( index );
}
