#pragma once

#include <memory>
#include "cinder/Vector.h"
#include "mndlkit/params/PParams.h"

namespace btd
{
	typedef std::shared_ptr< class BulletParameter > BulletParameterRef;

	class BulletParameter
	{
		friend class BulletWorld;
		friend class Model;

	public:
		enum GeneralDebugDrawType
		{
			GDDT_DrawWireframe        =  0,
			GDDT_DrawAabb             =  1,
			GDDT_DrawFeaturesText     =  2,
			GDDT_DrawContactPoints    =  3,
			GDDT_NoDeactivation       =  4,
			GDDT_NoHelpText           =  5,
			GDDT_DrawText             =  6,
			GDDT_ProfileTimings       =  7,
			GDDT_EnableSatComparison  =  8,
			GDDT_DisableBulletLCP     =  9,
			GDDT_EnableCCD            = 10,
			GDDT_DrawConstraints      = 11,
			GDDT_DrawConstraintLimits = 12,
			GDDT_FastWireframe        = 13,
			GDDT_DrawNormals          = 14,
			GDDT_DrawTransform        = 15,
		};

		enum SoftDebugDrawType
		{
			SDDT_Nodes                =  0,
			SDDT_Links                =  1,
			SDDT_Faces                =  2,
			SDDT_Tetras               =  3,
			SDDT_Normals              =  4,
			SDDT_Contacts             =  5,
			SDDT_Anchors              =  6,
			SDDT_Notes                =  7,
			SDDT_Clusters             =  8,
			SDDT_NodeTree             =  9,
			SDDT_FaceTree             = 10,
			SDDT_ClusterTree          = 11,
			SDDT_Joints               = 12,
		};

	public:
		BulletParameter();
		~BulletParameter();

		void setup();
		void update();

		void setGeneralDebugDrawEnable( GeneralDebugDrawType generalDebugDrawType, bool enable );
		bool getGeneralDebugDrawEnable( GeneralDebugDrawType generalDebugDrawType              ) const;

		void setSoftDebugDrawEnable( SoftDebugDrawType softDebugDrawType, bool enable );
		bool getSoftDebugDrawEnable( SoftDebugDrawType softDebugDrawType              ) const;

	protected:
		void setupParams();

	protected:
		mndl::params::PInterfaceGl  mParamsPhysics;

		// world
		ci::Vec3f                   mGravity;
		bool                        mSimulateOne;
		bool                        mSimulateAlways;

		// rigid body
		float                       mLinearDamping;  // [0-1]
		float                       mAngularDamping; // [0-1]
		float                       mDeactivationTime;
		float                       mLinearSleepingThresholds;
		float                       mAngularSleepingThresholds;

		// cone twist constraint
		float                       mDamping;
		float                       mStopCMF;
		float                       mStopERP;
// 		float                       mLinCFM;
// 		float                       mLinERP;
// 		float                       mAngCFM;

		// rope
		int                         mRopePart;
		float                       mRopeMass;
		float                       mKVCF;           // Velocities correction factor (Baumgarte)
		float                       mKDP;            // Damping coefficient [0,1]
		float                       mKDG;            // Drag coefficient [0,+inf]
		float                       mKLF;            // Lift coefficient [0,+inf]
		float                       mKPR;            // Pressure coefficient [-inf,+inf]
		float                       mKVC;            // Volume conversation coefficient [0,+inf]
		float                       mKDF;            // Dynamic friction coefficient [0,1]
		float                       mKMT;            // Pose matching coefficient [0,1]		
		float                       mKCHR;           // Rigid contacts hardness [0,1]
		float                       mKKHR;           // Kinetic contacts hardness [0,1]
		float                       mKSHR;           // Soft contacts hardness [0,1]
		float                       mKAHR;           // Anchors hardness [0,1]
		float                       mMaxvolume;      // Maximum volume ratio for pose
		float                       mTimescale;      // Time scale
		int                         mViterations;    // Velocities solver iterations
		int                         mPiterations;    // Positions solver iterations
		int                         mDiterations;    // Drift solver iterations
		int                         mCiterations;    // Cluster solver iterations

		mndl::params::PInterfaceGl  mParamsDebugDraw;
		bool                        mDrawSkin;
		bool                        mDrawSkeleton;
		bool                        mEnableWireframe;
		bool                        mShowDebugDrawRigidBody;
		static const int            GENERAL_DEBUG_DRAW_NUM = 16;
		bool                        mGeneralDebugDrawActive[ GENERAL_DEBUG_DRAW_NUM ];
		static const int            SOFT_DEBUG_DRAW_NUM = 13;
		bool                        mSoftDebugDrawActive[ SOFT_DEBUG_DRAW_NUM ];

		unsigned int                mGeneralDebugDrawTypes;
		unsigned int                mSoftDebugDrawTypes;
		bool                        mDrawTransform;
	};

} // namespace btd
