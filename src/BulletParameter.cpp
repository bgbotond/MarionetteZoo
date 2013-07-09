#include "BulletParameter.h"

#include "btBulletCollisionCommon.h"
#include "BulletSoftBody/btSoftBodyHelpers.h"

namespace btd
{
	BulletParameter::BulletParameter()
		: mGeneralDebugDrawTypes( 0 )
		, mSoftDebugDrawTypes( 0 )
		, mDrawTransform( false )
	{
		setGeneralDebugDrawEnable( GDDT_DrawWireframe       , true  );
		setGeneralDebugDrawEnable( GDDT_DrawAabb            , false );
		setGeneralDebugDrawEnable( GDDT_DrawFeaturesText    , false );
		setGeneralDebugDrawEnable( GDDT_DrawContactPoints   , false );
		setGeneralDebugDrawEnable( GDDT_NoDeactivation      , true  );
		setGeneralDebugDrawEnable( GDDT_NoHelpText          , false );
		setGeneralDebugDrawEnable( GDDT_DrawText            , false );
		setGeneralDebugDrawEnable( GDDT_ProfileTimings      , false );
		setGeneralDebugDrawEnable( GDDT_EnableSatComparison , false );
		setGeneralDebugDrawEnable( GDDT_DisableBulletLCP    , false );
		setGeneralDebugDrawEnable( GDDT_EnableCCD           , false );
		setGeneralDebugDrawEnable( GDDT_DrawConstraints     , false );
		setGeneralDebugDrawEnable( GDDT_DrawConstraintLimits, false );
		setGeneralDebugDrawEnable( GDDT_FastWireframe       , false );
		setGeneralDebugDrawEnable( GDDT_DrawNormals         , false );
		setGeneralDebugDrawEnable( GDDT_DrawTransform       , false );

		setSoftDebugDrawEnable( SDDT_Nodes       , false );
		setSoftDebugDrawEnable( SDDT_Links       , true  );
		setSoftDebugDrawEnable( SDDT_Faces       , false );
		setSoftDebugDrawEnable( SDDT_Tetras      , false );
		setSoftDebugDrawEnable( SDDT_Normals     , false );
		setSoftDebugDrawEnable( SDDT_Contacts    , false );
		setSoftDebugDrawEnable( SDDT_Anchors     , false );
		setSoftDebugDrawEnable( SDDT_Notes       , false );
		setSoftDebugDrawEnable( SDDT_Clusters    , false );
		setSoftDebugDrawEnable( SDDT_NodeTree    , false );
		setSoftDebugDrawEnable( SDDT_FaceTree    , false );
		setSoftDebugDrawEnable( SDDT_ClusterTree , false );
		setSoftDebugDrawEnable( SDDT_Joints      , false );
	}

	BulletParameter::~BulletParameter()
	{
	}

	void BulletParameter::setup()
	{
		setupParams();
	}

	void BulletParameter::update()
	{
		for( int i = 0; i < GENERAL_DEBUG_DRAW_NUM; ++i )
		{
			if( mGeneralDebugDrawActive[ i ] != getGeneralDebugDrawEnable( (GeneralDebugDrawType)i ) )
				setGeneralDebugDrawEnable( (GeneralDebugDrawType)i, mGeneralDebugDrawActive[ i ] );
		}

		for( int i = 0; i < SOFT_DEBUG_DRAW_NUM; ++i )
		{
			if( mSoftDebugDrawActive[ i ] != getSoftDebugDrawEnable( (SoftDebugDrawType)i ) )
				setSoftDebugDrawEnable( (SoftDebugDrawType)i, mSoftDebugDrawActive[ i ] );
		}
	}

	void BulletParameter::setupParams()
	{
		mParamsPhysics = mndl::params::PInterfaceGl( "Physics", ci::Vec2i( 250, 550 ), ci::Vec2i( 300, 50 ) );
		mParamsDebugDraw.addText( "World" );
		mParamsPhysics.addPersistentSizeAndPosition();

		mParamsPhysics.addPersistentParam( "Gravity", &mGravity, ci::Vec3f( 0.0f, -9.81f, 0.0f ) );

		mParamsPhysics.addPersistentParam( "SimulateOne (v)"   , &mSimulateOne   , false );
		mParamsPhysics.addPersistentParam( "SimulateAlways (c)", &mSimulateAlways, true  );

		mParamsPhysics.addText( "RigidBody" );
		mParamsPhysics.addPersistentParam( "Linear damping"             , &mLinearDamping            , 0.85 , "min=0.0 max=1.0 step=0.01" );
		mParamsPhysics.addPersistentParam( "Angular damping"            , &mAngularDamping           , 0.85 , "min=0.0 max=1.0 step=0.01" );
		mParamsPhysics.addPersistentParam( "Deactivation time"          , &mDeactivationTime         , 0.8  , "min=0.0         step=0.01" );
		mParamsPhysics.addPersistentParam( "Linear sleeping thresholds" , &mLinearSleepingThresholds , 1.6  , "min=0.0         step=0.01" );
		mParamsPhysics.addPersistentParam( "Angular sleeping thresholds", &mAngularSleepingThresholds, 2.5  , "min=0.0         step=0.01" );

		mParamsPhysics.addText( "Constraint" );
		mParamsPhysics.addPersistentParam( "Damping"                    , &mDamping                  , 0.60 , "min=0.0 max=1.0 step=0.01" );
		mParamsPhysics.addPersistentParam( "StopCMF"                    , &mStopCMF                  , 0.30 , "min=0.0 max=1.0 step=0.01" );
		mParamsPhysics.addPersistentParam( "StopERP"                    , &mStopERP                  , 0.80 , "min=0.0 max=1.0 step=0.01" );
// 		mParamsPhysics.addPersistentParam( "LinCFM"                     , &mLinCFM                   , 0.0  , "min=0.0 max=1.0 step=0.01" );
// 		mParamsPhysics.addPersistentParam( "LinERP"                     , &mLinERP                   , 0.7  , "min=0.0 max=1.0 step=0.01" );
// 		mParamsPhysics.addPersistentParam( "AngCFM"                     , &mAngCFM                   , 0.0f , "min=0.0 max=1.0 step=0.01" );

		mParamsPhysics.addText( "Rope" );
		mParamsPhysics.addPersistentParam( "Part"                                     , &mRopePart    , 16,   "min=4 max=50 step=1"         );
		mParamsPhysics.addPersistentParam( "Mass"                                     , &mRopeMass    , 5.0,  "min=0.01 max=20.0 step=0.01" );
		mParamsPhysics.addPersistentParam( "Velocities correction factor (Baumgarte)" , &mKVCF        , 1.0,  "min=0.0  max=20.0 step=0.1"  );
		mParamsPhysics.addPersistentParam( "Damping coefficient [0,1]"                , &mKDP         , 0.0,  "min=0.0  max=1.0  step=0.01" );
		mParamsPhysics.addPersistentParam( "Drag coefficient [0,+inf]"                , &mKDG         , 0.0,  "min=0.0           step=0.01" );
		mParamsPhysics.addPersistentParam( "Lift coefficient [0,+inf]"                , &mKLF         , 0.0,  "min=0.0           step=0.01" );
		mParamsPhysics.addPersistentParam( "Pressure coefficient [-inf,+inf]"         , &mKPR         , 0.0,  "                  step=0.01" );
		mParamsPhysics.addPersistentParam( "Volume conversation coefficient [0,+inf]" , &mKVC         , 0.0,  "min=0.0           step=0.01" );
		mParamsPhysics.addPersistentParam( "Dynamic friction coefficient [0,1]"       , &mKDF         , 0.2,  "min=0.0  max=1.0  step=0.01" );
		mParamsPhysics.addPersistentParam( "Pose matching coefficient [0,1]"          , &mKMT         , 0.0,  "min=0.0  max=1.0  step=0.01" );
		mParamsPhysics.addPersistentParam( "Rigid contacts hardness [0,1]"            , &mKCHR        , 1.0,  "min=0.0  max=1.0  step=0.01" );
		mParamsPhysics.addPersistentParam( "Kinetic contacts hardness [0,1]"          , &mKKHR        , 0.1,  "min=0.0  max=1.0  step=0.01" );
		mParamsPhysics.addPersistentParam( "Soft contacts hardness [0,1]"             , &mKSHR        , 1.0,  "min=0.0  max=1.0  step=0.01" );
		mParamsPhysics.addPersistentParam( "Anchors hardness [0,1]"                   , &mKAHR        , 1.0,  "min=0.0  max=1.0  step=0.01" );
		mParamsPhysics.addPersistentParam( "Maximum volume ratio for pose"            , &mMaxvolume   , 1.0   );
		mParamsPhysics.addPersistentParam( "Time scale"                               , &mTimescale   , 1.0   );
		mParamsPhysics.addPersistentParam( "Velocities solver iterations"             , &mViterations , 0     );
		mParamsPhysics.addPersistentParam( "Positions solver iterations"              , &mPiterations , 15    );
		mParamsPhysics.addPersistentParam( "Drift solver iterations"                  , &mDiterations , 0     );
		mParamsPhysics.addPersistentParam( "Cluster solver iterations"                , &mCiterations , 4     );

		mParamsDebugDraw = mndl::params::PInterfaceGl( "DebugDraw", ci::Vec2i( 250, 550 ), ci::Vec2i( 580, 50 ) );
		mParamsDebugDraw.addPersistentParam( "DrawSkin"           , &mDrawSkin, true );
		mParamsDebugDraw.addPersistentParam( "EnableWireframe"    , &mEnableWireframe, false );
		mParamsDebugDraw.addPersistentParam( "ShowRigidBody"      , &mShowDebugDrawRigidBody, false );

		mParamsDebugDraw.addText( "General" );
		const char *generalText[GENERAL_DEBUG_DRAW_NUM] = { "DrawWireframe", "DrawAabb", "DrawFeaturesText", "DrawContactPoints", "NoDeactivation", "NoHelpText", "DrawText", "ProfileTimings", "EnableSatComparison", "DisableBulletLCP", "EnableCCD", "DrawConstraints", "DrawConstraintLimits", "FastWireframe", "DrawNormals", "DrawTransform" };

		for( int i = 0; i < GENERAL_DEBUG_DRAW_NUM; ++i )
		{
			mParamsDebugDraw.addPersistentParam( generalText[i], &mGeneralDebugDrawActive[i], getGeneralDebugDrawEnable( (GeneralDebugDrawType)i ) );
		}

		mParamsDebugDraw.addText( "Soft" );
		const char *softText[SOFT_DEBUG_DRAW_NUM] = { "Nodes", "Links", "Faces", "Tetras", "Normals", "Contacts", "Anchors", "Notes", "Clusters", "NodeTree", "FaceTree", "ClusterTree", "Joints" };

		for( int i = 0; i < SOFT_DEBUG_DRAW_NUM; ++i )
		{
			mParamsDebugDraw.addPersistentParam( softText[i], &mSoftDebugDrawActive[i], getSoftDebugDrawEnable( (SoftDebugDrawType)i ) );
		}
	}

	void BulletParameter::setGeneralDebugDrawEnable( GeneralDebugDrawType generalDebugDrawType, bool enable )
	{
		btIDebugDraw::DebugDrawModes debugModel = btIDebugDraw::DBG_DrawWireframe;

		switch( generalDebugDrawType )
		{
		case GDDT_DrawWireframe        : debugModel = btIDebugDraw::DBG_DrawWireframe;        break;
		case GDDT_DrawAabb             : debugModel = btIDebugDraw::DBG_DrawAabb;             break;
		case GDDT_DrawFeaturesText     : debugModel = btIDebugDraw::DBG_DrawFeaturesText;     break;
		case GDDT_DrawContactPoints    : debugModel = btIDebugDraw::DBG_DrawContactPoints;    break;
		case GDDT_NoDeactivation       : debugModel = btIDebugDraw::DBG_NoDeactivation;       break;
		case GDDT_NoHelpText           : debugModel = btIDebugDraw::DBG_NoHelpText;           break;
		case GDDT_DrawText             : debugModel = btIDebugDraw::DBG_DrawText;             break;
		case GDDT_ProfileTimings       : debugModel = btIDebugDraw::DBG_ProfileTimings;       break;
		case GDDT_EnableSatComparison  : debugModel = btIDebugDraw::DBG_EnableSatComparison;  break;
		case GDDT_DisableBulletLCP     : debugModel = btIDebugDraw::DBG_DisableBulletLCP;     break;
		case GDDT_EnableCCD            : debugModel = btIDebugDraw::DBG_EnableCCD;            break;
		case GDDT_DrawConstraints      : debugModel = btIDebugDraw::DBG_DrawConstraints;      break;
		case GDDT_DrawConstraintLimits : debugModel = btIDebugDraw::DBG_DrawConstraintLimits; break;
		case GDDT_FastWireframe        : debugModel = btIDebugDraw::DBG_FastWireframe;        break;
		case GDDT_DrawNormals          : debugModel = btIDebugDraw::DBG_DrawNormals;          break;
		case GDDT_DrawTransform        : mDrawTransform = enable; return;
		}

		if( enable )
			mGeneralDebugDrawTypes |= debugModel;
		else
			mGeneralDebugDrawTypes &= ~debugModel;
	}

	bool BulletParameter::getGeneralDebugDrawEnable( GeneralDebugDrawType generalDebugDrawType ) const
	{
		btIDebugDraw::DebugDrawModes debugModel = btIDebugDraw::DBG_DrawWireframe;

		switch( generalDebugDrawType )
		{
		case GDDT_DrawWireframe        : debugModel = btIDebugDraw::DBG_DrawWireframe;        break;
		case GDDT_DrawAabb             : debugModel = btIDebugDraw::DBG_DrawAabb;             break;
		case GDDT_DrawFeaturesText     : debugModel = btIDebugDraw::DBG_DrawFeaturesText;     break;
		case GDDT_DrawContactPoints    : debugModel = btIDebugDraw::DBG_DrawContactPoints;    break;
		case GDDT_NoDeactivation       : debugModel = btIDebugDraw::DBG_NoDeactivation;       break;
		case GDDT_NoHelpText           : debugModel = btIDebugDraw::DBG_NoHelpText;           break;
		case GDDT_DrawText             : debugModel = btIDebugDraw::DBG_DrawText;             break;
		case GDDT_ProfileTimings       : debugModel = btIDebugDraw::DBG_ProfileTimings;       break;
		case GDDT_EnableSatComparison  : debugModel = btIDebugDraw::DBG_EnableSatComparison;  break;
		case GDDT_DisableBulletLCP     : debugModel = btIDebugDraw::DBG_DisableBulletLCP;     break;
		case GDDT_EnableCCD            : debugModel = btIDebugDraw::DBG_EnableCCD;            break;
		case GDDT_DrawConstraints      : debugModel = btIDebugDraw::DBG_DrawConstraints;      break;
		case GDDT_DrawConstraintLimits : debugModel = btIDebugDraw::DBG_DrawConstraintLimits; break;
		case GDDT_FastWireframe        : debugModel = btIDebugDraw::DBG_FastWireframe;        break;
		case GDDT_DrawNormals          : debugModel = btIDebugDraw::DBG_DrawNormals;          break;
		case GDDT_DrawTransform        : return mDrawTransform;
		}

		return( mGeneralDebugDrawTypes & debugModel ) != 0;
	}

	void BulletParameter::setSoftDebugDrawEnable( SoftDebugDrawType softDebugDrawType, bool enable )
	{
		fDrawFlags::_ softDebugModel = fDrawFlags::Nodes;

		switch( softDebugDrawType )
		{
		case SDDT_Nodes        : softDebugModel = fDrawFlags::Nodes;         break;
		case SDDT_Links        : softDebugModel = fDrawFlags::Links;         break;
		case SDDT_Faces        : softDebugModel = fDrawFlags::Faces;         break;
		case SDDT_Tetras       : softDebugModel = fDrawFlags::Tetras;        break;
		case SDDT_Normals      : softDebugModel = fDrawFlags::Normals;       break;
		case SDDT_Contacts     : softDebugModel = fDrawFlags::Contacts;      break;
		case SDDT_Anchors      : softDebugModel = fDrawFlags::Anchors;       break;
		case SDDT_Notes        : softDebugModel = fDrawFlags::Notes;         break;
		case SDDT_Clusters     : softDebugModel = fDrawFlags::Clusters;      break;
		case SDDT_NodeTree     : softDebugModel = fDrawFlags::NodeTree;      break;
		case SDDT_FaceTree     : softDebugModel = fDrawFlags::FaceTree;      break;
		case SDDT_ClusterTree  : softDebugModel = fDrawFlags::ClusterTree;   break;
		case SDDT_Joints       : softDebugModel = fDrawFlags::Joints;        break;
		}

		if( enable )
			mSoftDebugDrawTypes |= softDebugModel;
		else
			mSoftDebugDrawTypes &= ~softDebugModel;
	}

	bool BulletParameter::getSoftDebugDrawEnable( SoftDebugDrawType softDebugDrawType ) const
	{
		fDrawFlags::_ softDebugModel = fDrawFlags::Nodes;

		switch( softDebugDrawType )
		{
		case SDDT_Nodes        : softDebugModel = fDrawFlags::Nodes;         break;
		case SDDT_Links        : softDebugModel = fDrawFlags::Links;         break;
		case SDDT_Faces        : softDebugModel = fDrawFlags::Faces;         break;
		case SDDT_Tetras       : softDebugModel = fDrawFlags::Tetras;        break;
		case SDDT_Normals      : softDebugModel = fDrawFlags::Normals;       break;
		case SDDT_Contacts     : softDebugModel = fDrawFlags::Contacts;      break;
		case SDDT_Anchors      : softDebugModel = fDrawFlags::Anchors;       break;
		case SDDT_Notes        : softDebugModel = fDrawFlags::Notes;         break;
		case SDDT_Clusters     : softDebugModel = fDrawFlags::Clusters;      break;
		case SDDT_NodeTree     : softDebugModel = fDrawFlags::NodeTree;      break;
		case SDDT_FaceTree     : softDebugModel = fDrawFlags::FaceTree;      break;
		case SDDT_ClusterTree  : softDebugModel = fDrawFlags::ClusterTree;   break;
		case SDDT_Joints       : softDebugModel = fDrawFlags::Joints;        break;
		}

		return( mSoftDebugDrawTypes & softDebugModel ) != 0;
	}

} // namespace
