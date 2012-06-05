package us.ihmc.commonWalkingControlModules.controlModules.swingLegTorqueControl;

import javax.media.j3d.Transform3D;
import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

import com.yobotics.simulationconstructionset.BooleanYoVariable;
import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;

import us.ihmc.commonWalkingControlModules.controlModuleInterfaces.SwingLegTorqueControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.LegJointPositionControlModule;
import us.ihmc.commonWalkingControlModules.couplingRegistry.CouplingRegistry;
import us.ihmc.commonWalkingControlModules.dynamics.FullRobotModel;
import us.ihmc.commonWalkingControlModules.kinematics.DesiredJointAccelerationCalculatorInWorldFrame;
import us.ihmc.commonWalkingControlModules.kinematics.DesiredJointVelocityCalculator;
import us.ihmc.commonWalkingControlModules.kinematics.InverseKinematicsException;
import us.ihmc.commonWalkingControlModules.kinematics.LegInverseKinematicsCalculator;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegJointName;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegJointPositions;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegJointVelocities;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegTorques;
import us.ihmc.commonWalkingControlModules.referenceFrames.CommonWalkingReferenceFrames;
import us.ihmc.commonWalkingControlModules.sensors.ProcessedSensorsInterface;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.Orientation;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.geometry.RotationFunctions;
import us.ihmc.utilities.screwTheory.InverseDynamicsCalculator;
import us.ihmc.utilities.screwTheory.SpatialAccelerationVector;
import us.ihmc.utilities.screwTheory.Twist;
import us.ihmc.utilities.screwTheory.Wrench;

public class PDPlusIDSwingLegTorqueControlModule implements SwingLegTorqueControlModule
{
   private final YoVariableRegistry registry = new YoVariableRegistry("PDPlusIDSwingLegTorqueControlModule");
   private final LegJointName[] legJointNames;

   private final ProcessedSensorsInterface processedSensors;
   
   private final CommonWalkingReferenceFrames referenceFrames;
   private final FullRobotModel fullRobotModel;
   private final CouplingRegistry couplingRegistry;
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   
   private final SideDependentList<LegJointPositions> desiredLegJointPositions = new SideDependentList<LegJointPositions>();
   private final SideDependentList<LegJointVelocities> desiredLegJointVelocities = new SideDependentList<LegJointVelocities>();
   
   private final LegInverseKinematicsCalculator inverseKinematicsCalculator;
   private final SideDependentList<DesiredJointVelocityCalculator> desiredJointVelocityCalculators;
   private final SideDependentList<DesiredJointAccelerationCalculatorInWorldFrame> desiredJointAccelerationCalculators;
   
   private final SideDependentList<LegJointPositionControlModule> legJointPositionControlModules;
   private final SideDependentList<InverseDynamicsCalculator> inverseDynamicsCalculators;
   
   private final BooleanYoVariable inverseKinematicsExceptionHasBeenThrown = new BooleanYoVariable("kinematicException", registry);
   private final DoubleYoVariable jacobianDeterminant = new DoubleYoVariable("jacobianDeterminant", registry);
   private final DoubleYoVariable dampedLeastSquaresAlpha = new DoubleYoVariable("dampedLeastSquaresAlpha", registry);
   private boolean useBodyAcceleration;

   
   public PDPlusIDSwingLegTorqueControlModule(LegJointName[] legJointNames, ProcessedSensorsInterface processedSensors,
         CommonWalkingReferenceFrames referenceFrames, FullRobotModel fullRobotModel, CouplingRegistry couplingRegistry,
         LegInverseKinematicsCalculator inverseKinematicsCalculator,
         SideDependentList<DesiredJointVelocityCalculator> desiredJointVelocityCalculators,
         SideDependentList<DesiredJointAccelerationCalculatorInWorldFrame> desiredJointAccelerationCalculators,
         SideDependentList<LegJointPositionControlModule> legJointPositionControlModules, SideDependentList<InverseDynamicsCalculator> inverseDynamicsCalculators, YoVariableRegistry parentRegistry)
   {
      this.legJointNames = legJointNames;
      this.processedSensors = processedSensors;
      this.referenceFrames = referenceFrames;
      this.fullRobotModel = fullRobotModel;
      this.couplingRegistry = couplingRegistry;
      this.inverseKinematicsCalculator = inverseKinematicsCalculator;
      this.desiredJointVelocityCalculators = desiredJointVelocityCalculators;
      this.desiredJointAccelerationCalculators = desiredJointAccelerationCalculators;
      this.legJointPositionControlModules = legJointPositionControlModules;
      this.inverseDynamicsCalculators = inverseDynamicsCalculators;
      
      for (RobotSide robotSide : RobotSide.values())
      {
         this.desiredLegJointPositions.put(robotSide, new LegJointPositions(robotSide));
         this.desiredLegJointVelocities.put(robotSide, new LegJointVelocities(legJointNames, robotSide));
      }
      dampedLeastSquaresAlpha.set(0.07);
      
      parentRegistry.addChild(registry);
   }

   public void compute(LegTorques legTorquesToPackForSwingLeg, FramePoint desiredFootPosition, Orientation desiredFootOrientation, FrameVector desiredFootVelocity, FrameVector desiredFootAngularVelocity, FrameVector desiredFootAcceleration, FrameVector desiredFootAngularAcceleration)
   {
      // robotSides
      RobotSide swingSide = legTorquesToPackForSwingLeg.getRobotSide();

      // reference frames
      ReferenceFrame pelvisFrame = referenceFrames.getPelvisFrame();
      ReferenceFrame footFrame = referenceFrames.getFootFrame(swingSide);
      ReferenceFrame footCoMFrame = fullRobotModel.getFoot(swingSide).getBodyFixedFrame();
      ReferenceFrame elevatorFrame = fullRobotModel.getElevatorFrame();

      // Desired positions
      Transform3D footToPelvis = computeDesiredTransform(pelvisFrame, desiredFootPosition, desiredFootOrientation);
      Twist desiredTwistOfSwingFootWithRespectToWorld = computeDesiredTwist(worldFrame, footFrame, desiredFootVelocity, desiredFootAngularVelocity);
      desiredTwistOfSwingFootWithRespectToWorld.changeFrame(footCoMFrame);
      desiredTwistOfSwingFootWithRespectToWorld.changeBodyFrameNoRelativeTwist(footCoMFrame);

      Matrix3d footToPelvisOrientation = new Matrix3d();
      footToPelvis.get(footToPelvisOrientation);
      double desiredHipYaw = RotationFunctions.getYaw(footToPelvisOrientation);    // TODO: wrong and not necessary for R2, but ok for now.
      try
      {
         inverseKinematicsCalculator.solve(desiredLegJointPositions.get(swingSide), footToPelvis, swingSide, desiredHipYaw);
         inverseKinematicsExceptionHasBeenThrown.set(false);
      }
      catch (InverseKinematicsException e)
      {
         inverseKinematicsExceptionHasBeenThrown.set(true);
      }

      // Desired velocities
      DesiredJointVelocityCalculator desiredJointVelocityCalculator = desiredJointVelocityCalculators.get(swingSide);
      desiredJointVelocityCalculator.packDesiredJointVelocities(desiredLegJointVelocities.get(swingSide), desiredTwistOfSwingFootWithRespectToWorld, dampedLeastSquaresAlpha.getDoubleValue());

      // set body acceleration
      if (useBodyAcceleration)
      {
         SpatialAccelerationVector bodyAcceleration = processedSensors.getAccelerationOfPelvisWithRespectToWorld();    // FIXME: set to LIPM-based predicted body acceleration
         bodyAcceleration.setAngularPart(new Vector3d());    // zero desired angular acceleration
         bodyAcceleration.setLinearPart(new Vector3d());    // zero linear acceleration as well for now
         fullRobotModel.getRootJoint().setDesiredAcceleration(bodyAcceleration);
      }

      // Desired acceleration
      SpatialAccelerationVector desiredAccelerationOfSwingFootWithRespectToWorld = computeDesiredSwingFootSpatialAcceleration(elevatorFrame, footFrame, desiredFootAcceleration, desiredFootAngularAcceleration);
      desiredAccelerationOfSwingFootWithRespectToWorld.changeFrameNoRelativeMotion(footCoMFrame);
      desiredAccelerationOfSwingFootWithRespectToWorld.changeBodyFrameNoRelativeAcceleration(footCoMFrame);
      jacobianDeterminant.set(desiredJointVelocityCalculator.swingFullLegJacobianDeterminant());
      desiredJointAccelerationCalculators.get(swingSide).compute(desiredAccelerationOfSwingFootWithRespectToWorld, dampedLeastSquaresAlpha.getDoubleValue());

      double percentScaling = 1.0; //getPercentScalingBasedOnJacobianDeterminant(jacobianDeterminant.getDoubleValue());

//      LegJointName[] legJointNames = fullRobotModel.getRobotSpecificJointNames().getLegJointNames();
//      for (LegJointName legJointName : legJointNames)
//      {
//         // this is better than not using the torques from the ID calculator at all, because at least gravity and Coriolis forces are compensated for
//         RevoluteJoint revoluteJoint = fullRobotModel.getLegJoint(swingSide, legJointName);
//         double qddDesired = revoluteJoint.getQddDesired();
//         revoluteJoint.setQddDesired(qddDesired * percentScaling);
//      }

      // control
      legJointPositionControlModules.get(swingSide).packTorquesForLegJointsPositionControl(percentScaling, legTorquesToPackForSwingLeg,
                                         desiredLegJointPositions.get(swingSide), desiredLegJointVelocities.get(swingSide));

      inverseDynamicsCalculators.get(swingSide).compute();

      for (LegJointName legJointName : legTorquesToPackForSwingLeg.getLegJointNames())
      {
         double tauInverseDynamics = fullRobotModel.getLegJoint(swingSide, legJointName).getTau();
         legTorquesToPackForSwingLeg.addTorque(legJointName, tauInverseDynamics);
      }

      setUpperBodyWrench();      
   }
   
   public void computePreSwing(RobotSide swingSide)
   {
      fullRobotModel.getRootJoint().setDesiredAccelerationToZero();

      for (LegJointName legJointName : legJointNames)
      {
         fullRobotModel.getLegJoint(swingSide, legJointName).setQddDesired(0.0);
      }

      inverseDynamicsCalculators.get(swingSide).compute();
      setUpperBodyWrench();      
   }
   
   public void setAnkleGainsSoft(RobotSide swingSide)
   {
      legJointPositionControlModules.get(swingSide).setAnkleGainsSoft();      
   }
   
   public void setAnkleGainsDefault(RobotSide swingSide)
   {
      legJointPositionControlModules.get(swingSide).resetScalesToDefault();
   }
   
   private Transform3D computeDesiredTransform(ReferenceFrame pelvisFrame, FramePoint desiredFootPosition, Orientation desiredFootOrientation)
   {
      desiredFootOrientation.changeFrame(pelvisFrame);
      desiredFootPosition.changeFrame(pelvisFrame);
      Transform3D footToPelvis = createTransform(desiredFootOrientation, desiredFootPosition);

      return footToPelvis;
   }

   private Twist computeDesiredTwist(ReferenceFrame worldFrame, ReferenceFrame footFrame, FrameVector desiredFootVelocity, FrameVector desiredFootAngularVelocity)
   {
      desiredFootVelocity.changeFrame(footFrame);
      desiredFootAngularVelocity.changeFrame(footFrame);
      Twist desiredTwistOfSwingFootWithRespectToStanceFoot = new Twist(footFrame, worldFrame, footFrame, desiredFootVelocity.getVector(),
                                                                desiredFootAngularVelocity.getVector());

      return desiredTwistOfSwingFootWithRespectToStanceFoot;
   }

   private SpatialAccelerationVector computeDesiredSwingFootSpatialAcceleration(ReferenceFrame elevatorFrame, ReferenceFrame footFrame, FrameVector desiredSwingFootAcceleration, FrameVector desiredSwingFootAngularAcceleration)
   {
      desiredSwingFootAcceleration.changeFrame(footFrame);
      desiredSwingFootAngularAcceleration.changeFrame(footFrame);
      SpatialAccelerationVector desiredAccelerationOfSwingFootWithRespectToWorld = new SpatialAccelerationVector(footFrame, elevatorFrame, footFrame,
                                                                                      desiredSwingFootAcceleration.getVector(),
                                                                                      desiredSwingFootAngularAcceleration.getVector());

      return desiredAccelerationOfSwingFootWithRespectToWorld;
   }

   private static Transform3D createTransform(Orientation orientation, FramePoint framePoint)
   {
      orientation.checkReferenceFrameMatch(framePoint);
      Matrix3d rotationMatrix = orientation.getMatrix3d();
      Transform3D ret = new Transform3D(rotationMatrix, new Vector3d(framePoint.getPoint()), 1.0);

      return ret;
   }
   
//   private double getPercentScalingBasedOnJacobianDeterminant(double jacobianDeterminant)
//   {
//      double determinantThresholdOne = 0.06;    // 0.05;    // 0.025;
//      double determinantThresholdTwo = 0.03;    // 0.02; //0.01;
//
//      double percent = (Math.abs(jacobianDeterminant) - determinantThresholdTwo) / (determinantThresholdOne - determinantThresholdTwo);
//      percent = MathTools.clipToMinMax(percent, 0.0, 1.0);
//
//      return percent;
//   }
   
   private void setUpperBodyWrench()
   {
      Wrench upperBodyWrench = new Wrench();
      fullRobotModel.getRootJoint().packWrench(upperBodyWrench);
      upperBodyWrench.changeBodyFrameAttachedToSameBody(referenceFrames.getPelvisFrame());
      upperBodyWrench.changeFrame(referenceFrames.getPelvisFrame());
      couplingRegistry.setDesiredUpperBodyWrench(upperBodyWrench);
   }
   
   public void setParametersForR2()
   {
      useBodyAcceleration = false;
   }
   
   public void setParametersForM2V2()
   {
      useBodyAcceleration = true;
   }
}
