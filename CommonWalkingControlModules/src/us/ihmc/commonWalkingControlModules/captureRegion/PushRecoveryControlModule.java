package us.ihmc.commonWalkingControlModules.captureRegion;

import java.awt.Color;
import java.util.ArrayList;
import java.util.List;

import javax.media.j3d.Transform3D;
import javax.vecmath.Point2d;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.desiredFootStep.Footstep;
import us.ihmc.commonWalkingControlModules.desiredFootStep.FootstepUtils;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.ICPAndMomentumBasedController;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.commonWalkingControlModules.referenceFrames.CommonWalkingReferenceFrames;
import us.ihmc.commonWalkingControlModules.trajectories.SwingTimeCalculationProvider;
import us.ihmc.plotting.shapes.PointArtifact;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.math.geometry.FrameConvexPolygon2d;
import us.ihmc.utilities.math.geometry.FrameLine2d;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.PoseReferenceFrame;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

import com.yobotics.simulationconstructionset.BooleanYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.plotting.YoFrameLine2dArtifact;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameLine2d;
import com.yobotics.simulationconstructionset.util.statemachines.StateMachine;
import com.yobotics.simulationconstructionset.util.statemachines.StateTransitionCondition;

public class PushRecoveryControlModule
{
   private static final boolean ENABLE = false;

   private static final double MINIMUM_TIME_BEFORE_RECOVER_WITH_REDUCED_POLYGON = 6;
   private static final double DOUBLESUPPORT_SUPPORT_POLYGON_SCALE = 0.85;
   private static final double TRUST_TIME_SCALE = 0.9;
   private static final double MINIMUM_TIME_TO_REPLAN = 0.1;
   private static final double MINIMUM_SWING_TIME_FOR_DOUBLE_SUPPORT_RECOVERY = 0.35;
   private static final double MINIMUN_CAPTURE_REGION_PERCENTAGE_OF_FOOT_AREA = 2.5;
   private static final double REDUCE_SWING_TIME_MULTIPLIER = 0.95;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final boolean useICPProjection = true;

   private final BooleanYoVariable enablePushRecoveryFromDoubleSupport;
   private final ICPAndMomentumBasedController icpAndMomentumBasedController;
   private final MomentumBasedController momentumBasedController;
   private final OrientationStateVisualizer orientationStateVisualizer;
   private final FootstepAdjustor footstepAdjustor;
   private final OneStepCaptureRegionCalculator captureRegionCalculator;
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final BooleanYoVariable footstepWasProjectedInCaptureRegion;
   private final BooleanYoVariable enablePushRecovery = new BooleanYoVariable("enablePushRecovery", registry);
   private final CommonWalkingReferenceFrames referenceFrames;
   private final StateMachine<?> stateMachine;
   private final SwingTimeCalculationProvider swingTimeCalculationProvider;

   private boolean recoveringFromDoubleSupportFall;
   private boolean usingReducedSwingTime;
   private double reducedSwingTime, doubleSupportInitialSwingTime;
   private Footstep recoverFromDoubleSupportFallFootStep;
   private final BooleanYoVariable recovering;
   private BooleanYoVariable readyToGrabNextFootstep;
   private final DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry;
   private final SideDependentList<? extends ContactablePlaneBody> feet;
   private final List<FramePoint> tempContactPoints;
   private final FrameConvexPolygon2d tempFootPolygon;

   private final YoFrameLine2d capturePointTrajectoryLine;
   private final YoFrameLine2dArtifact capturePointTrajectoryLineArtifact;
   private final PointArtifact projectedCapturePointArtifact;
   private final Point2d projectedCapturePoint;

   public PushRecoveryControlModule(MomentumBasedController momentumBasedController, WalkingControllerParameters walkingControllerParameters,
         BooleanYoVariable readyToGrabNextFootstep, ICPAndMomentumBasedController icpAndMomentumBasedController, StateMachine<?> stateMachine,
         YoVariableRegistry parentRegistry, SwingTimeCalculationProvider swingTimeCalculationProvider, SideDependentList<? extends ContactablePlaneBody> feet)
   {
      this.momentumBasedController = momentumBasedController;
      this.readyToGrabNextFootstep = readyToGrabNextFootstep;
      this.icpAndMomentumBasedController = icpAndMomentumBasedController;
      this.referenceFrames = momentumBasedController.getReferenceFrames();
      this.stateMachine = stateMachine;
      this.swingTimeCalculationProvider = swingTimeCalculationProvider;
      this.feet = feet;

      enablePushRecovery.set(ENABLE);

      this.enablePushRecoveryFromDoubleSupport = new BooleanYoVariable("enablePushRecoveryFromDoubleSupport", registry);
      this.enablePushRecoveryFromDoubleSupport.set(false);
      this.usingReducedSwingTime = false;

      this.dynamicGraphicObjectsListRegistry = momentumBasedController.getDynamicGraphicObjectsListRegistry();
      captureRegionCalculator = new OneStepCaptureRegionCalculator(referenceFrames, walkingControllerParameters, registry, dynamicGraphicObjectsListRegistry);
      footstepAdjustor = new FootstepAdjustor(registry, dynamicGraphicObjectsListRegistry);
      orientationStateVisualizer = new OrientationStateVisualizer(dynamicGraphicObjectsListRegistry, registry);

      footstepWasProjectedInCaptureRegion = new BooleanYoVariable("footstepWasProjectedInCaptureRegion", registry);
      recovering = new BooleanYoVariable("recovering", registry);

      capturePointTrajectoryLine = new YoFrameLine2d(getClass().getSimpleName(), "CapturePointTrajectoryLine", ReferenceFrame.getWorldFrame(), registry);
      projectedCapturePoint = new Point2d();
      projectedCapturePointArtifact = new PointArtifact("ProjectedCapturePointArtifact", projectedCapturePoint);
      tempContactPoints = new ArrayList<FramePoint>();
      tempFootPolygon = new FrameConvexPolygon2d(worldFrame);

      this.capturePointTrajectoryLineArtifact = new YoFrameLine2dArtifact("CapturePointTrajectoryLineArtifact", capturePointTrajectoryLine, Color.red);
      dynamicGraphicObjectsListRegistry.registerArtifact("CapturePointTrajectoryArtifact", capturePointTrajectoryLineArtifact);
      dynamicGraphicObjectsListRegistry.registerArtifact("ProjectedCapturePointArtifact", projectedCapturePointArtifact);

      parentRegistry.addChild(registry);

      reset();
   }

   public boolean isEnabled()
   {
      return enablePushRecovery.getBooleanValue();
   }

   public void reset()
   {
      footstepWasProjectedInCaptureRegion.set(false);
      recovering.set(false);
      recoverFromDoubleSupportFallFootStep = null;
      captureRegionCalculator.hideCaptureRegion();

      if (recoveringFromDoubleSupportFall)
      {
         if (usingReducedSwingTime)
         {
            this.swingTimeCalculationProvider.updateSwingTime();
            usingReducedSwingTime = false;
         }
         recoveringFromDoubleSupportFall = false;
      }
   }

   public class IsFallingFromDoubleSupportCondition implements StateTransitionCondition
   {
      private final FramePoint2d capturePoint2d = new FramePoint2d();
      private RobotSide swingSide = null;
      private RobotSide transferToSide = null;

      private final Transform3D fromWorldToPelvis = new Transform3D();
      private final Transform3D scaleTransformation = new Transform3D();
      private final FrameConvexPolygon2d reducedSupportPolygon;
      private final ReferenceFrame midFeetZUp;
      private double capturePointYAxis, regularSwingTime;
      private FrameVector projectedCapturePoint;
      private Footstep currentFootstep = null;
      private boolean isICPOutside;

      public IsFallingFromDoubleSupportCondition(RobotSide robotSide)
      {
         this.transferToSide = robotSide;
         this.scaleTransformation.setScale(DOUBLESUPPORT_SUPPORT_POLYGON_SCALE);
         this.reducedSupportPolygon = new FrameConvexPolygon2d(icpAndMomentumBasedController.getBipedSupportPolygons().getSupportPolygonInMidFeetZUp());
         this.projectedCapturePoint = new FrameVector(worldFrame, 0, 0, 0);
         midFeetZUp = referenceFrames.getMidFeetZUpFrame();
      }

      @Override
      public boolean checkCondition()
      {
         if (enablePushRecovery.getBooleanValue())
         {
            if (getDoubleSupportEnableState())
            {
               FrameConvexPolygon2d supportPolygonInMidFeetZUp = icpAndMomentumBasedController.getBipedSupportPolygons().getSupportPolygonInMidFeetZUp();

               // get current robot status
               reducedSupportPolygon.changeFrame(midFeetZUp);
               reducedSupportPolygon.setAndUpdate(supportPolygonInMidFeetZUp);
               icpAndMomentumBasedController.getCapturePoint().getFrameTuple2dIncludingFrame(capturePoint2d);

               capturePoint2d.changeFrame(midFeetZUp);
               reducedSupportPolygon.applyTransform(scaleTransformation);

               // update the visualization
               momentumBasedController.getFullRobotModel().getPelvis().getBodyFixedFrame().getTransformToDesiredFrame(fromWorldToPelvis, worldFrame);
               orientationStateVisualizer.updatePelvisReferenceFrame(fromWorldToPelvis);
               orientationStateVisualizer.updateReducedSupportPolygon(reducedSupportPolygon);

               if (stateMachine.timeInCurrentState() < MINIMUM_TIME_BEFORE_RECOVER_WITH_REDUCED_POLYGON)
               {
                  isICPOutside = !supportPolygonInMidFeetZUp.isPointInside(capturePoint2d);
               }
               else
               {
                  isICPOutside = !reducedSupportPolygon.isPointInside(capturePoint2d);
               }

               if (isICPOutside && recoverFromDoubleSupportFallFootStep == null)
               {
                  System.out.println("Robot is falling from double support");
                  projectedCapturePoint.changeFrame(capturePoint2d.getReferenceFrame());
                  projectedCapturePoint.set(capturePoint2d.getX(), capturePoint2d.getY(), 0);
                  projectedCapturePoint.changeFrame(momentumBasedController.getFullRobotModel().getPelvis().getBodyFixedFrame());
                  capturePointYAxis = projectedCapturePoint.getY();
                  if (capturePointYAxis >= 0)
                  {
                     swingSide = RobotSide.LEFT;
                  }
                  else
                  {
                     swingSide = RobotSide.RIGHT;
                  }

                  if (transferToSide == swingSide)
                  {
                     return false;
                  }

                  currentFootstep = createFootstepAtCurrentLocation(swingSide);
                  capturePoint2d.changeFrame(worldFrame);
                  
                  regularSwingTime = swingTimeCalculationProvider.getValue();
                  
                  doubleSupportInitialSwingTime = computeInitialSwingTimeForDoubleSupportRecovery(swingSide, regularSwingTime, capturePoint2d);

                  if (doubleSupportInitialSwingTime < MINIMUM_SWING_TIME_FOR_DOUBLE_SUPPORT_RECOVERY)
                  {
                     // TODO prepare to fall or try to do 2-step recover
                     return false;
                  }
                  
                  if(doubleSupportInitialSwingTime < regularSwingTime)
                  {
                     swingTimeCalculationProvider.setSwingTime(doubleSupportInitialSwingTime);
                     usingReducedSwingTime = true;
                     readyToGrabNextFootstep.set(false);
                     momentumBasedController.getUpcomingSupportLeg().set(transferToSide.getOppositeSide());
                     recoverFromDoubleSupportFallFootStep = currentFootstep;
                     recoveringFromDoubleSupportFall = true;
                     reducedSwingTime = doubleSupportInitialSwingTime;
                  }
                  
                  return true;
               }

               // we need this to reset the reference frame 
               reducedSupportPolygon.changeFrame(worldFrame);
               return false;
            }
         }

         return false;
      }
   }

   public boolean checkAndUpdateFootstep(RobotSide swingSide, double swingTimeRemaining, FramePoint2d capturePoint2d, Footstep nextFootstep, double omega0,
         FrameConvexPolygon2d footPolygon)
   {
      if (enablePushRecovery.getBooleanValue())
      {
         captureRegionCalculator.calculateCaptureRegion(swingSide, swingTimeRemaining, capturePoint2d, omega0, footPolygon);

         if (swingTimeRemaining < MINIMUM_TIME_TO_REPLAN)
         {
            // do not re-plan if we are almost at touch-down
            return false;
         }

         footstepWasProjectedInCaptureRegion.set(footstepAdjustor.adjustFootstep(nextFootstep, footPolygon.getCentroid(),
               captureRegionCalculator.getCaptureRegion()));

         if (footstepWasProjectedInCaptureRegion.getBooleanValue())
         {
            recovering.set(true);
         }
         
         return footstepWasProjectedInCaptureRegion.getBooleanValue();
      }
      return false;
   }
   
   private double computeMinimumSwingTime(RobotSide swingSide, double swingTimeRemaining, FramePoint2d capturePoint2d, double omega0, FrameConvexPolygon2d footPolygon)
   {
      double reducedSwingTime = swingTimeRemaining;
      captureRegionCalculator.calculateCaptureRegion(swingSide, swingTimeRemaining, capturePoint2d, omega0, footPolygon);
      
      // If the capture region is too small we reduce the swing time. 
      while (captureRegionCalculator.getCaptureRegionArea() < PushRecoveryControlModule.MINIMUN_CAPTURE_REGION_PERCENTAGE_OF_FOOT_AREA * footPolygon.getArea() || 
                  Double.isNaN(captureRegionCalculator.getCaptureRegionArea()))
      {
         reducedSwingTime = reducedSwingTime * REDUCE_SWING_TIME_MULTIPLIER;
         captureRegionCalculator.calculateCaptureRegion(swingSide, reducedSwingTime, capturePoint2d, omega0, footPolygon);
      }
      
      return reducedSwingTime;
      
   }
   
   private double computeInitialSwingTimeForDoubleSupportRecovery(RobotSide swingSide, double swingTimeRemaining, FramePoint2d capturePoint2d)
   {
      momentumBasedController.getContactPoints(feet.get(swingSide.getOppositeSide()), tempContactPoints);
      tempFootPolygon.setIncludingFrameByProjectionOntoXYPlaneAndUpdate(momentumBasedController.getReferenceFrames().getAnkleZUpFrame(swingSide.getOppositeSide()), tempContactPoints);
      
      return computeMinimumSwingTime(swingSide, swingTimeRemaining, capturePoint2d, icpAndMomentumBasedController.getOmega0(), tempFootPolygon);
   }

   public double computeTimeToProjectDesiredICPToClosestPointOnTrajectoryToActualICP(FramePoint2d capturePoint2d, FramePoint2d constantCenterOfPressure,
         FramePoint2d initialICP, FramePoint2d finalDesiredICP, double omega0)
   {
      FrameLine2d capturePointTrajectoryLine = new FrameLine2d(finalDesiredICP, initialICP);
      this.capturePointTrajectoryLine.setFrameLine2d(capturePointTrajectoryLine);

      capturePointTrajectoryLine.orthogonalProjection(capturePoint2d);//project current capture point to desired capture point trajectory line

      projectedCapturePoint.set(capturePoint2d.getPointCopy());

      return computeOffsetTimeToMoveDesiredICPToTrajectoryLine(capturePoint2d, constantCenterOfPressure, initialICP, finalDesiredICP, omega0);
   }
   
   private Footstep createFootstepAtCurrentLocation(RobotSide robotSide)
   {
      ContactablePlaneBody foot = feet.get(robotSide);
      ReferenceFrame footReferenceFrame = foot.getRigidBody().getParentJoint().getFrameAfterJoint();
      FramePose framePose = new FramePose(footReferenceFrame);
      framePose.changeFrame(worldFrame);

      PoseReferenceFrame poseReferenceFrame = new PoseReferenceFrame("poseReferenceFrame", framePose);

      ReferenceFrame soleReferenceFrame = FootstepUtils.createSoleFrame(poseReferenceFrame, foot);
      List<FramePoint> expectedContactPoints = FootstepUtils.getContactPointsInFrame(foot, soleReferenceFrame);
      boolean trustHeight = true;

      Footstep footstep = new Footstep(foot, poseReferenceFrame, soleReferenceFrame, expectedContactPoints, trustHeight);

      return footstep;
   }

   private double computeOffsetTimeToMoveDesiredICPToTrajectoryLine(FramePoint2d capturePoint, FramePoint2d constantCenterOfPressure, FramePoint2d initialICP,
         FramePoint2d finalDesiredICP, double omega0)
   {
      // Given a desired capture point that is on the capture point trajectory, an initial capture point, a constant center of pressure, and omega, this 
      // method computes a time offset that will move the desired capture point to a desired point on that trajectory given by the argument capturePoint.
      FramePoint2d tmpNumerator = capturePoint;
      tmpNumerator.sub(constantCenterOfPressure);

      FramePoint2d tmpDenominator = initialICP;
      tmpDenominator.sub(constantCenterOfPressure);

      return (1 / omega0) * Math.log(Math.abs(tmpNumerator.getX()) / Math.abs(tmpDenominator.getX()));
   }

   //   private FrameConvexPolygon2d computeFootPolygon(RobotSide robotSide, ReferenceFrame referenceFrame)
   //   {
   //      final List<FramePoint> tempContactPoints = new ArrayList<FramePoint>();
   //      final FrameConvexPolygon2d tempFootPolygon = new FrameConvexPolygon2d(worldFrame);
   //
   //      momentumBasedController.getContactPoints(momentumBasedController.getContactablePlaneFeet().get(robotSide), tempContactPoints);
   //      tempFootPolygon.setIncludingFrameByProjectionOntoXYPlaneAndUpdate(referenceFrame, tempContactPoints);
   //
   //      return tempFootPolygon;
   //   }

   public boolean isRecoveringFromDoubleSupportFall()
   {
      return recoveringFromDoubleSupportFall;
   }

   public boolean getDoubleSupportEnableState()
   {
      return enablePushRecoveryFromDoubleSupport.getBooleanValue();
   }

   public Footstep getRecoverFromDoubleSupportFootStep()
   {
      return recoverFromDoubleSupportFallFootStep;
   }

   public double getTrustTimeToConsiderSwingFinished()
   {
      return this.reducedSwingTime * TRUST_TIME_SCALE;
   }

   public void setRecoveringFromDoubleSupportState(boolean value)
   {
      recoveringFromDoubleSupportFall = value;
   }

   public void setRecoverFromDoubleSupportFootStep(Footstep recoverFootStep)
   {
      recoverFromDoubleSupportFallFootStep = recoverFootStep;
   }

   public boolean useICPProjection()
   {
      return this.useICPProjection;
   }

   public boolean isRecovering()
   {
      return recovering.getBooleanValue();
   }
}
