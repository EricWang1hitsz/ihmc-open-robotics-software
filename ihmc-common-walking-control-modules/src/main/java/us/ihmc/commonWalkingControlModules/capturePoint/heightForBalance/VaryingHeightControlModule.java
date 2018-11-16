package us.ihmc.commonWalkingControlModules.capturePoint.heightForBalance;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition.GraphicType;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.ArtifactList;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPosition;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.FloatingInverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.trajectories.MinimumJerkTrajectory;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoFramePoint2D;

import java.awt.*;

import static java.lang.Math.*;

public class VaryingHeightControlModule implements VaryingHeightControlModuleInterface
{
   private final YoVariableRegistry registry = new YoVariableRegistry("HeightForBalance");
   private WalkingControllerParameters walkingControllerParameters;

   private FrameConvexPolygon2D supportPolygon = new FrameConvexPolygon2D();
   private FramePoint2D desiredCMPtoProject = new FramePoint2D();
   private FrameVector2D icpError = new FrameVector2D();
   private FramePoint2D com2DtoProject = new FramePoint2D();
   private FramePoint2D com2DtoProjectEndOfSwing = new FramePoint2D();
   private FramePoint3D com3D = new FramePoint3D();
   private FrameVector3D linearMomentumRateOfChangeFromLIP = new FrameVector3D();
   private FramePoint2D comEndOfStep2D = new FramePoint2D();
   private YoFramePoint2D yoCoMEndOfSTep2D;
   private YoFramePoint2D yoCoMEndOfSwing2DNotHacky;

   private FrameVector2D desiredICPVelocity = new FrameVector2D();

   private YoFramePoint2D yoProjectedDesiredCMP;
   private YoFramePoint2D yoProjectedCoM2D;
   private final YoEnum<VaryingHeightPrimaryConditionEnum> primaryConditionYoEnum;
   private final YoEnum<VaryingHeightSecondaryConditionEnum> secondaryConditionYoEnum;
   private boolean primaryConditionHasChanged;
   private boolean secondaryConditionHasChanged;
   private boolean calculateUseAngleForConditions;

   private double totalMass;
   private double stateClock;
   private YoDouble yoTimeInState;

   private double desiredHeightAcceleration = 0;
   private double desiredHeightAccelerationPreviousTick = 0;
   private YoDouble yoDesiredHeightAcceleration;

   private RobotSide supportSide = null;

   private FrameVector2D addedHorizontalAcceleration = new FrameVector2D();

   private FrameVector3D modifiedLinearMomentumRateOfChange = new FrameVector3D();

   private HighLevelHumanoidControllerToolbox controllerToolbox;

   private OneDoFJoint kneeJoint;
   private OneDoFJoint ankleJoint;

   private double kneeAngle;

   private boolean isInDoubleSupport;

   private YoDouble copCoMICPeAngle;
   private YoDouble copCoMICPeAngleFinal;

   private boolean useAngleForConditions;
   private YoBoolean yoUseAngleForConditions;

   private YoDouble yoTimeMinVelReached;
   private YoDouble yoTimeMaxVelReached;
   private YoDouble yoTimeMinPosReached;
   private YoDouble yoTimeMaxPosReached;
   private YoDouble yoTimeRemaining;
   private YoDouble yoTimeToSwitch;

   private double posAlignTresh;
   private double posAlignTreshFromStart;
   private double negAlignTresh;
   private double negAlignTreshFromStart;
   private double vMax;
   private double vMin;
   private double aMaxCtrl;
   private double aMinCtrl;
   private double aMaxCtrlAngle;
   private double aMinCtrlAngle;
   private double aMaxCtrlDistance;
   private double aMinCtrlDistance;
   private double jMax;
   private double dt;
   private double aMaxPredicted;
   private double aMinPredicted;
   private double fracOfAForPrediction;
   private double zMax;
   private double zMaxStartSwing;
   private double zMaxTouchDown;
   private double zMin;
   private double minKneeAngle;
   private double maxKneeAngle;
   private double tf;
   private double tForHalfWaySwing;
   private double tRemainingEndOfWalkingState;
   private double tRemainingConditionSwitch;
   private double smoothEpsilon;
   private double cmpFracOfMaxDistance;
   private boolean useThreshold;
   private double copCoMProjMinDistance;

   boolean walkingStateSwitch;
   private YoBoolean yoWalkingStateSwitch;
   private YoBoolean yoAngleGrows;

   private YoBoolean yoAngleImproves;
   private YoBoolean yoDistanceImproves;

   private YoDouble yoPosAlignThresh;
   private YoDouble yoCoMHeightVelocity;
   private YoDouble yozMax;
   private YoDouble yozMaxAlpha;

   boolean heightControlInThisWalkingState = false;
   boolean dsTrajectoryIsGenerated = false;
   MinimumJerkTrajectory minimumJerkTrajectory = new MinimumJerkTrajectory();

   VaryingHeightAngleAndDistanceEvaluator angleAndDistanceEvaluator;
   VaryingHeightPrimaryConditionEvaluator primaryConditionEvaluator;
   VaryingHeightTimeToConstraintsPredictor timeToConstraintsPredictor;
   VaryingHeightSecondaryConditionEvaluator secondaryConditionEvaluator;

   public VaryingHeightControlModule(double totalMass, HighLevelHumanoidControllerToolbox controllerToolbox, YoVariableRegistry parentRegistry,
                                     YoGraphicsListRegistry yoGraphicsListRegistry, WalkingControllerParameters walkingControllerParameters)
   {
      this.controllerToolbox = controllerToolbox;
      this.totalMass = totalMass;
      this.walkingControllerParameters = walkingControllerParameters;
      parentRegistry.addChild(registry);
      dt = controllerToolbox.getControlDT();
      yoDesiredHeightAcceleration = new YoDouble("DesiredHeightAccelerationHeightControl", registry);

      // error angle, error angle end of swing and error angle direction
      copCoMICPeAngle = new YoDouble("CoPCoMICPeAngle", registry);
      copCoMICPeAngleFinal = new YoDouble("CoPCoMICPeAngleFinal", registry);

      yoCoMEndOfSTep2D = new YoFramePoint2D("varyingHeightCoMEndOfSTep", ReferenceFrame.getWorldFrame(), registry);
      yoCoMEndOfSwing2DNotHacky = new YoFramePoint2D("varyingHeightCoMEndOfSwingFromPlanner", ReferenceFrame.getWorldFrame(), registry);
      yoCoMHeightVelocity = new YoDouble("heightForBalanceVelocity", registry);

      // angle and distance booleans
      yoAngleGrows = new YoBoolean("angleGrows", registry);
      yoAngleImproves = new YoBoolean("angleImproves", registry);
      yoDistanceImproves = new YoBoolean("distanceImproves", registry);
      yoUseAngleForConditions = new YoBoolean("useAngleForConditions", registry);

      yoWalkingStateSwitch = new YoBoolean("stateSwitch", registry);

      // estimated times
      yoTimeMinVelReached = new YoDouble("estTimeMinVel", registry);
      yoTimeMaxVelReached = new YoDouble("estTimeMaxVel", registry);
      yoTimeMinPosReached = new YoDouble("estTimeMinPos", registry);
      yoTimeMaxPosReached = new YoDouble("estTimeMaxPos", registry);
      yoTimeRemaining = new YoDouble("estTimeRemainingToEnd", registry);
      yoTimeToSwitch = new YoDouble("estTimeToSwitch", registry);

      // threshold for positive alignment, can be modifed
      yoPosAlignThresh = new YoDouble("posAlignmentThreshold", registry);
      yozMax = new YoDouble("zMaxHeightCtrl",registry);
      yozMaxAlpha = new YoDouble("zMaxAlphaHeightCtrl",registry);

      yoTimeToSwitch.set(walkingControllerParameters.getDefaultSwingTime());

      primaryConditionYoEnum = new YoEnum<>("varyingHeightCondition", registry, VaryingHeightPrimaryConditionEnum.class);
      secondaryConditionYoEnum = new YoEnum<>("varyingHeightSecondaryCondition", registry, VaryingHeightSecondaryConditionEnum.class);

      yoTimeInState = new YoDouble("varyingHeightTimeInState", registry);

      String label = getClass().getSimpleName();
      ArtifactList artifacts = new ArtifactList(label);

      yoProjectedDesiredCMP = new YoFramePoint2D(label + "projCMPd", ReferenceFrame.getWorldFrame(), registry);
      artifacts.add(new YoArtifactPosition("proj CMPd", yoProjectedDesiredCMP.getYoX(), yoProjectedDesiredCMP.getYoY(), GraphicType.BALL_WITH_CROSS, Color.RED,
                                           0.05));

      yoProjectedCoM2D = new YoFramePoint2D(label + "projCOM2D", ReferenceFrame.getWorldFrame(), registry);
      artifacts.add(new YoArtifactPosition("proj CoM2D", yoProjectedCoM2D.getYoX(), yoProjectedCoM2D.getYoY(), GraphicType.BALL_WITH_CROSS, Color.RED, 0.02));

      primaryConditionYoEnum.set(VaryingHeightPrimaryConditionEnum.DEFAULT);
      secondaryConditionYoEnum.set(VaryingHeightSecondaryConditionEnum.DEFAULT);
      artifacts.setVisible(true);
      yoGraphicsListRegistry.registerArtifactList(artifacts);

      // Parameters:
      vMax = walkingControllerParameters.getHeightForBalanceParameters().getMaxVelocityForPrediction();
      vMin = walkingControllerParameters.getHeightForBalanceParameters().getMinVelocityForPrediction();
      zMaxStartSwing = walkingControllerParameters.getHeightForBalanceParameters().getMaxHeightFirstPhaseOfSwing();
      zMaxTouchDown = zMaxStartSwing * walkingControllerParameters.getHeightForBalanceParameters().getMaxHeightFractionForSecondPhaseOfSwing();
      zMin = walkingControllerParameters.getHeightForBalanceParameters().getMinHeight();
      smoothEpsilon = walkingControllerParameters.getHeightForBalanceParameters().getSmoothEpsilon();
      minKneeAngle = walkingControllerParameters.getHeightForBalanceParameters().getMinKneeAngle();
      maxKneeAngle = walkingControllerParameters.getHeightForBalanceParameters().getMaxKneeAngle();
      cmpFracOfMaxDistance = walkingControllerParameters.getHeightForBalanceParameters().getFractionCMPOfMaxDistanceFromPolygonForHeightControl();
      aMaxCtrlAngle = walkingControllerParameters.getHeightForBalanceParameters().getMaxHeightAccelerationForAngleCase();
      aMinCtrlAngle = walkingControllerParameters.getHeightForBalanceParameters().getMinHeightAccelerationForAngleCase();
      aMaxCtrlDistance = walkingControllerParameters.getHeightForBalanceParameters().getMaxHeightAccelerationForDistanceCase();
      aMinCtrlDistance = walkingControllerParameters.getHeightForBalanceParameters().getMinHeightAccelerationForDistanceCase();
      jMax = walkingControllerParameters.getHeightForBalanceParameters().getMaximumJerk();
      posAlignTreshFromStart = walkingControllerParameters.getHeightForBalanceParameters().getAnglePositiveAlignmentThresholdFromStart();
      negAlignTreshFromStart = walkingControllerParameters.getHeightForBalanceParameters().getAngleNegativeAlignmentThreshold();
      tForHalfWaySwing = walkingControllerParameters.getHeightForBalanceParameters().getFractionOfSwingTimeToChangeMaxHeight() * walkingControllerParameters
            .getDefaultSwingTime();
      fracOfAForPrediction = walkingControllerParameters.getHeightForBalanceParameters().getFractionOfMaxHeightAccelerationToConsiderInPrediction();
      copCoMProjMinDistance = walkingControllerParameters.getHeightForBalanceParameters().getMinimumCoPCoMProjectedICPeDistanceToControl();

      posAlignTresh = posAlignTreshFromStart;
      negAlignTresh = negAlignTreshFromStart;

      angleAndDistanceEvaluator = new VaryingHeightAngleAndDistanceEvaluator();
      primaryConditionEvaluator = new VaryingHeightPrimaryConditionEvaluator(zMin, minKneeAngle, maxKneeAngle);
      timeToConstraintsPredictor = new VaryingHeightTimeToConstraintsPredictor(zMin, vMin, vMax);
      secondaryConditionEvaluator = new VaryingHeightSecondaryConditionEvaluator(zMin, tForHalfWaySwing, smoothEpsilon, timeToConstraintsPredictor);
   }

   public void compute()
   {
      stateClock = stateClock + dt;
      tf = walkingControllerParameters.getDefaultSwingTime();
      tRemainingEndOfWalkingState = tf - stateClock;
      yoTimeRemaining.set(tRemainingEndOfWalkingState);
      yoTimeInState.set(stateClock);

      VaryingHeightPrimaryConditionEnum primaryConditionPreviousTick = primaryConditionYoEnum.getEnumValue();
      VaryingHeightSecondaryConditionEnum secondaryConditionPreviousTick = secondaryConditionYoEnum.getEnumValue();
      desiredHeightAccelerationPreviousTick = desiredHeightAcceleration;

      kneeAngle = getKneeAngle();

      com2DtoProject.changeFrame(ReferenceFrame.getWorldFrame());
      com2DtoProjectEndOfSwing.changeFrame(ReferenceFrame.getWorldFrame());
      com3D.changeFrame(ReferenceFrame.getWorldFrame());
      desiredCMPtoProject.changeFrame(ReferenceFrame.getWorldFrame());
      icpError.changeFrame(ReferenceFrame.getWorldFrame());

      yoCoMEndOfSTep2D.set(comEndOfStep2D);
      double distance = supportPolygon.signedDistance(desiredCMPtoProject);
      boolean cmpOutsidePolygon = (!supportPolygon.isPointInside(desiredCMPtoProject));
      boolean isProjected = supportPolygon.orthogonalProjection(desiredCMPtoProject);

      // For standing push
      boolean nonDynamicCase = desiredICPVelocity.length() < 0.02;

      // Reset values if state switch occurs, or if standing
      if (walkingStateSwitch
            || nonDynamicCase && secondaryConditionPreviousTick == VaryingHeightSecondaryConditionEnum.HOLD && distance < 0.1 * walkingControllerParameters
            .getMaxAllowedDistanceCMPSupport())
      {
         heightControlInThisWalkingState = false;
         dsTrajectoryIsGenerated = false;
         posAlignTresh = posAlignTreshFromStart;
         negAlignTresh = negAlignTreshFromStart;
      }

      // Determines if height control or not
      boolean heightControlCondition = (
            icpError.length() > 0.05 && isProjected && cmpOutsidePolygon && (distance > cmpFracOfMaxDistance * walkingControllerParameters
                  .getMaxAllowedDistanceCMPSupport()) && (isInDoubleSupport == false || nonDynamicCase == true) || heightControlInThisWalkingState == true);

      // Some variables
      FrameVector3D centerOfMassVelocity = new FrameVector3D();
      controllerToolbox.getCenterOfMassJacobian().getCenterOfMassVelocity(centerOfMassVelocity);
      double z = com3D.getZ();
      double dz = centerOfMassVelocity.getZ();
      yoCoMHeightVelocity.set(dz);
      FrameVector2D centerOfMassVelocity2D = new FrameVector2D();
      centerOfMassVelocity2D.setIncludingFrame(centerOfMassVelocity);
      centerOfMassVelocity2D.changeFrame(ReferenceFrame.getWorldFrame());

      // Determines if the decision has to be made which control law has to be used.
      calculateUseAngleForConditions = (isProjected && cmpOutsidePolygon && heightControlInThisWalkingState == false);


      // HEIGHT CONTROL
      if (heightControlCondition)
      {
         // different max heights
         zMax = getzMax(nonDynamicCase,stateClock,tForHalfWaySwing);
         yozMax.set(zMax);
         // Min/max - control law set based on using angle or distance
         if (calculateUseAngleForConditions || walkingStateSwitch)
         {
            useAngleForConditions = angleAndDistanceEvaluator.getUseAngleForConditions(supportPolygon, desiredCMPtoProject, yoCoMEndOfSwing2DNotHacky);
            yoUseAngleForConditions.set(useAngleForConditions);
            if (!useAngleForConditions)
            {
               aMinCtrl = aMinCtrlDistance;
               aMaxCtrl = aMaxCtrlAngle;
               aMaxPredicted = fracOfAForPrediction * aMaxCtrl;
               aMinPredicted = fracOfAForPrediction * aMinCtrlAngle;
               useThreshold=false;
            }
            else if (useAngleForConditions)
            {
               aMinCtrl = aMinCtrlAngle;
               aMaxCtrl = aMaxCtrlAngle;
               aMaxPredicted = fracOfAForPrediction * aMaxCtrl;
               aMinPredicted = fracOfAForPrediction * aMinCtrl;
               useThreshold=true;
            }
         }
         // set
         heightControlInThisWalkingState = true;
         calculateUseAngleForConditions = false;

         // projected com positions, NOT USED
         FrameLine2D desiredPushDirectionFromCoP = new FrameLine2D(desiredCMPtoProject, icpError); // should be positive
         desiredPushDirectionFromCoP.orthogonalProjection(com2DtoProject);
         desiredPushDirectionFromCoP.orthogonalProjection(com2DtoProjectEndOfSwing);
         yoProjectedCoM2D.set(com2DtoProject);

         /**
          * Error angle current
          */
         double errorAngle = angleAndDistanceEvaluator.getErrorAngle(icpError, desiredCMPtoProject, com3D);
         copCoMICPeAngle.set(errorAngle);
         /**
          * Angle direction
          */
         boolean angleGrows = angleAndDistanceEvaluator.getAngleGrows(errorAngle, centerOfMassVelocity2D, desiredCMPtoProject, com3D);
         yoAngleGrows.set(angleGrows);

         /**
          * Error angle end of swing
          */
         double errorAngleEndOfSwing = angleAndDistanceEvaluator.getErrorAngleEndOfSwing(yoCoMEndOfSwing2DNotHacky, desiredCMPtoProject, icpError);
         copCoMICPeAngleFinal.set(errorAngleEndOfSwing);
         /**
          *  Distance current + end of swing + distance direction + boolean improves
          */
         double copCoMProjDistance = com2DtoProject.distance(desiredCMPtoProject);
         double copCoMProjDistanceEndOFSwing = com2DtoProjectEndOfSwing.distance(desiredCMPtoProject);
         boolean distanceImproves = angleAndDistanceEvaluator.getDistanceImproves(centerOfMassVelocity2D, icpError);
         boolean distancePosAlignment = angleAndDistanceEvaluator.getDistancePosAlignment(com3D,desiredCMPtoProject,icpError);
         yoDistanceImproves.set(distanceImproves);

         /**
          * Computation of time to velocity constraints
          */
         double tMinVelReachedPredicted = timeToConstraintsPredictor.getTMinVelReachedPredicted(dz, aMinCtrl);
         yoTimeMinVelReached.set(tMinVelReachedPredicted);
         double tMaxVelReachedPredicted = timeToConstraintsPredictor.getTMaxVelReachedPredicted(dz, aMaxCtrl);
         yoTimeMaxVelReached.set(tMaxVelReachedPredicted);

         /**
          * Computation of time to position constraints
          */
         double tMinPosReachedPredicted = timeToConstraintsPredictor.getTMinPosReachedPredicted(z, dz, aMinCtrl, aMaxPredicted);
         yoTimeMinPosReached.set(tMinPosReachedPredicted);
         double tMaxPosReachedPredicted = timeToConstraintsPredictor.getTMaxPosReachedPredicted(z, dz, zMaxTouchDown, aMinPredicted, aMaxCtrl);
         yoTimeMaxPosReached.set(tMaxPosReachedPredicted);

         /**
          * Evaluate primary conditions
          */
         VaryingHeightPrimaryConditionEnum primaryCondition = primaryConditionEvaluator
               .computeAndGetPrimaryConditionEnum(aMinPredicted, aMaxPredicted, z, dz, zMax, kneeAngle, errorAngle, errorAngleEndOfSwing, angleGrows,
                                                  negAlignTresh, posAlignTresh, primaryConditionPreviousTick, useAngleForConditions, distancePosAlignment,
                                                  copCoMProjDistance, copCoMProjDistanceEndOFSwing, nonDynamicCase, yoTimeToSwitch.getDoubleValue(),useThreshold, copCoMProjMinDistance);
         primaryConditionYoEnum.set(primaryCondition);
         primaryConditionHasChanged = primaryConditionEvaluator.getPrimaryConditionHasChanged();
         // In second phase of swing, always the most extreme minimal control if the leg is at MAX
         if (primaryCondition == VaryingHeightPrimaryConditionEnum.MAXZ)
         {
            aMinCtrl = aMinCtrlAngle;
         }

         /**
          * Parameters for control and secondary condition
          */
         VaryingHeightSecondaryConditionEnum secondaryCondition = secondaryConditionEvaluator
               .computeAndGetSecondaryConditionEnum(aMinCtrl, aMaxCtrl, z, dz, primaryCondition, primaryConditionHasChanged, secondaryConditionPreviousTick,
                                                    stateClock, tMinVelReachedPredicted, tMinPosReachedPredicted, tMaxVelReachedPredicted,
                                                    tMaxPosReachedPredicted, tRemainingEndOfWalkingState, errorAngle, errorAngleEndOfSwing, angleGrows,
                                                    posAlignTresh, negAlignTresh, zMaxTouchDown, nonDynamicCase);
         secondaryConditionYoEnum.set(secondaryCondition);
         posAlignTresh = secondaryConditionEvaluator.getModifiedPosAlignTresh();
         yoPosAlignThresh.set(posAlignTresh);
         tRemainingConditionSwitch = secondaryConditionEvaluator.getTimeToConditionSwitch();
         yoTimeToSwitch.set(tRemainingConditionSwitch);
         double aCtrl = secondaryConditionEvaluator.getControlBound();
         double aSmooth = secondaryConditionEvaluator.getControlSmooth();

         /**
          * Control
          */
         if (secondaryCondition == VaryingHeightSecondaryConditionEnum.SMOOTH)
         {
            desiredHeightAcceleration = aSmooth;
         }
         else if (secondaryCondition == VaryingHeightSecondaryConditionEnum.HOLD)
         {
            FullHumanoidRobotModel fullRobotModel = controllerToolbox.getFullRobotModel();
            ReferenceFrame hipPitchFrame = fullRobotModel.getLegJoint(RobotSide.LEFT, LegJointName.HIP_PITCH).getFrameAfterJoint();
            FramePoint3D hipPoint = new FramePoint3D(hipPitchFrame);
            FramePoint3D kneePoint = new FramePoint3D(fullRobotModel.getLegJoint(RobotSide.LEFT, LegJointName.KNEE_PITCH).getFrameBeforeJoint());
            kneePoint.changeFrame(hipPitchFrame);

            double thighLength = hipPoint.distance(kneePoint);

            ReferenceFrame kneePitchFrame = fullRobotModel.getLegJoint(RobotSide.LEFT, LegJointName.KNEE_PITCH).getFrameAfterJoint();
            kneePoint.setToZero(kneePitchFrame);
            FramePoint3D anklePoint = new FramePoint3D(fullRobotModel.getLegJoint(RobotSide.LEFT, LegJointName.ANKLE_PITCH).getFrameBeforeJoint());
            anklePoint.changeFrame(kneePitchFrame);

            double shinLength = kneePoint.distance(anklePoint);
            anklePoint.changeFrame(ReferenceFrame.getWorldFrame());

            double r = shinLength + thighLength;
            double comXToAnkle = com3D.getX() - anklePoint.getX();
            double x = com3D.getX();
            double dx = centerOfMassVelocity.getX();
            double legHeightDistance = r- shinLength*cos(getAnkleAngle())-thighLength*cos(getAnkleAngle()+getKneeAngle());

            double omega0 = controllerToolbox.getOmega0();
            double ddx = omega0*omega0*(com3D.getX()-desiredCMPtoProject.getX());
            double rsqminxsq = r*r-comXToAnkle*comXToAnkle;
            double ddzdHold = -(r*r/rsqminxsq-1)*2*dx*ddx - 2*r*r*x*dx*dx*dx/(rsqminxsq*rsqminxsq);

            if (nonDynamicCase)
            {
               double kp = walkingControllerParameters.getCoMHeightControlGains().getKp();
               double kd = walkingControllerParameters.getCoMHeightControlGains().getKd();
               desiredHeightAcceleration = kp * (zMax - z) - kd * dz;
            }
            else if (z>(zMaxStartSwing-legHeightDistance))
            {
               desiredHeightAcceleration = ddzdHold;
            }
            else
            {
               desiredHeightAcceleration = 0;
            }
         }
         else if (secondaryCondition == VaryingHeightSecondaryConditionEnum.DEFAULT)
         {
            desiredHeightAcceleration = aCtrl;
         }

         if(isInDoubleSupport&&yoTimeInState.getDoubleValue()>walkingControllerParameters.getDefaultTransferTime()&&!nonDynamicCase || primaryCondition ==VaryingHeightPrimaryConditionEnum.DEFAULT)
         {
            desiredHeightAcceleration=linearMomentumRateOfChangeFromLIP.getZ()/totalMass;
         }

         /**
          * Acceleration and jerk checks, respectively
          */
         desiredHeightAcceleration = MathTools.clamp(desiredHeightAcceleration, aMinCtrl, aMaxCtrl);                                         // Acceleration
         desiredHeightAcceleration = MathTools
               .clamp(desiredHeightAcceleration, desiredHeightAccelerationPreviousTick - jMax * dt, desiredHeightAccelerationPreviousTick + jMax * dt); // Jerk

         yoDesiredHeightAcceleration.set(desiredHeightAcceleration);

         /**
          * Added lin momentum rate based on CoP/ZMP
          */
         if (primaryCondition == VaryingHeightPrimaryConditionEnum.PREPARE_NEG)
         {
            useThreshold=false;
         }
         yoProjectedDesiredCMP.set(desiredCMPtoProject);
         addedHorizontalAcceleration.set(com3D);
         addedHorizontalAcceleration.sub(desiredCMPtoProject);
         addedHorizontalAcceleration.scale(yoDesiredHeightAcceleration.getDoubleValue() / com3D.getZ());
         modifiedLinearMomentumRateOfChange.set(linearMomentumRateOfChangeFromLIP.getX() + addedHorizontalAcceleration.getX() * totalMass,
                                                linearMomentumRateOfChangeFromLIP.getY() + addedHorizontalAcceleration.getY() * totalMass,
                                                yoDesiredHeightAcceleration.getDoubleValue() * totalMass);
      }
      else
      {
         /**
          * Smoothing out differences if varying height controller kicked in, horizontal linear momentum rate is here not affected by height acceleration
          * (as in normal control setting)
          */
         primaryConditionYoEnum.set(VaryingHeightPrimaryConditionEnum.DEFAULT);
         secondaryConditionYoEnum.set(VaryingHeightSecondaryConditionEnum.DEFAULT);
         desiredHeightAcceleration = linearMomentumRateOfChangeFromLIP.getZ() / totalMass;
         /*
         if(desiredHeightAcceleration>2.0 && !nonDynamicCase)
         {
            if (isInDoubleSupport && yoTimeInState.getDoubleValue() < 0.1)
            {
               desiredHeightAcceleration = 0.0;
            }
            else if (isInDoubleSupport)
            {
               double kp = walkingControllerParameters.getCoMHeightControlGains().getKp();
               double kd = walkingControllerParameters.getCoMHeightControlGains().getKd();
               if (!dsTrajectoryIsGenerated)
               {
                  minimumJerkTrajectory.setMoveParameters(z, dz, desiredHeightAccelerationPreviousTick, 1.0, 0.0, 0.0, 0.1);
                  dsTrajectoryIsGenerated = true;
               }
               minimumJerkTrajectory.computeTrajectory(yoTimeInState.getDoubleValue()-0.1);
               desiredHeightAcceleration =
                     kp * (minimumJerkTrajectory.getPosition() - z) + kd * (minimumJerkTrajectory.getVelocity() - dz) + minimumJerkTrajectory.getAcceleration();
            }
         }
         */
         desiredHeightAcceleration = MathTools.clamp(desiredHeightAcceleration,aMinCtrlAngle,aMaxCtrlAngle);
         desiredHeightAcceleration = MathTools
               .clamp(desiredHeightAcceleration, desiredHeightAccelerationPreviousTick - jMax * dt, desiredHeightAccelerationPreviousTick + jMax * dt); // Jerk
         yoDesiredHeightAcceleration.set(desiredHeightAcceleration); // HERE DOESNT AFFECT Ld!!
         modifiedLinearMomentumRateOfChange.setIncludingFrame(linearMomentumRateOfChangeFromLIP.getReferenceFrame(), linearMomentumRateOfChangeFromLIP.getX(),
                                                              linearMomentumRateOfChangeFromLIP.getY(), desiredHeightAcceleration * totalMass);
      }
   }

   private double getKneeAngle()
   {
      double angle;
      if (supportSide == null)
      {
         angle = 1.17;
      }
      else
      {
         kneeJoint = controllerToolbox.getFullRobotModel().getLegJoint(supportSide, LegJointName.KNEE_PITCH);
         angle = kneeJoint.getQ();
      }
      return angle;
   }

   private double getAnkleAngle()
   {
      double angle;
      if (supportSide == null)
      {
         angle = 0.0;
      }
      else
      {
         ankleJoint = controllerToolbox.getFullRobotModel().getLegJoint(supportSide, LegJointName.ANKLE_PITCH);
         angle = ankleJoint.getQ();
      }
      return angle;
   }

   private double getzMax(boolean nonDynamicCase, double stateClock, double timeHalfWaySwing)
   {
      if (nonDynamicCase)
      {
         zMax = zMaxStartSwing;
         yozMaxAlpha.set(0.0);
      }
      else if (stateClock > timeHalfWaySwing)
      {
         double alpha = (stateClock-tForHalfWaySwing)/(walkingControllerParameters.getDefaultSwingTime()-tForHalfWaySwing);
         yozMaxAlpha.set(alpha);
         zMax = zMaxStartSwing - alpha*(zMaxStartSwing-zMaxTouchDown);
      }
      else
      {
         yozMaxAlpha.set(0.0);
         zMax = zMaxStartSwing;
      }
      return zMax;
   }
   public FrameVector3D getModifiedLinearMomentumRateOfChange()
   {
      return modifiedLinearMomentumRateOfChange;
   }

   public void setSupportPolygon(FrameConvexPolygon2D supportPolygon)
   {
      this.supportPolygon.setIncludingFrame(supportPolygon);
      this.supportPolygon.changeFrameAndProjectToXYPlane(ReferenceFrame.getWorldFrame());
   }

   public void setDesiredCMP(FramePoint2D desiredCMP)
   {
      this.desiredCMPtoProject.setIncludingFrame(desiredCMP);
   }

   public void setICPError(FrameVector2D icpError)
   {
      this.icpError.setIncludingFrame(icpError);
   }

   public void setCoM(FramePoint3D CoM)
   {
      this.com2DtoProject.setIncludingFrame(CoM);
      this.com3D.setIncludingFrame(CoM);
   }

   public void setCoMEndOfSTep(FramePoint3D coMEndOfSTep)
   {
      yoCoMEndOfSwing2DNotHacky.set(coMEndOfSTep);
      com2DtoProjectEndOfSwing.setIncludingFrame(coMEndOfSTep);

   }

   public void setLinearMomentumRateOfChangeFromLIP(FrameVector3D linearMomentumRateOfChangeFromLIP)
   {
      this.linearMomentumRateOfChangeFromLIP.setIncludingFrame(linearMomentumRateOfChangeFromLIP);
   }

   public void setSupportSide(RobotSide supportSide)
   {
      this.supportSide = supportSide;
   }

   public void setIsInDoubleSupport(boolean isInDoubleSupport)
   {
      if (this.isInDoubleSupport == isInDoubleSupport)
      {
         walkingStateSwitch = false;
         this.isInDoubleSupport = isInDoubleSupport;
      }
      else
      {
         walkingStateSwitch = true;
         stateClock = 0;
         this.isInDoubleSupport = isInDoubleSupport;
      }
      yoWalkingStateSwitch.set(walkingStateSwitch);
   }

   public void setDesiredCapturePointVelocity(FrameVector2D desiredCapturePointVelocity)
   {
      desiredICPVelocity.setIncludingFrame(desiredCapturePointVelocity);
      desiredICPVelocity.changeFrame(ReferenceFrame.getWorldFrame());
   }

   public VaryingHeightPrimaryConditionEnum getPrimaryCondition()
   {
      return primaryConditionYoEnum.getEnumValue();
   }

   public double getTimeToSwitch()
   {
      return yoTimeToSwitch.getDoubleValue();
   }

   public double getTimeRemaining()
   {
      return yoTimeRemaining.getDoubleValue();
   }

   public double getErrorAngleEndOfSwing()
   {
      return copCoMICPeAngleFinal.getDoubleValue();
   }
}
