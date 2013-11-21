package us.ihmc.vrcGraveYard.highLevelHumanoidControl.manipulation.states.fingerToroidManipulation.states;

import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.statemachines.State;

import us.ihmc.commonWalkingControlModules.controlModules.RigidBodySpatialAccelerationControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.SE3PDGains;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.IndividualHandControlModule;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.commonWalkingControlModules.trajectories.SE3ConfigurationProvider;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.GeometricJacobian;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.vrcGraveYard.highLevelHumanoidControl.manipulation.states.fingerToroidManipulation.FingerToroidManipulationState;

/**
 * @author twan
 *         Date: 5/21/13
 */
public class StraightLinePositionControlState extends State<FingerToroidManipulationState>
{
   private final SideDependentList<IndividualHandControlModule> individualHandControlModules;
   private final SideDependentList<SE3ConfigurationProvider> finalConfigurationProviders;
   private final SideDependentList<ReferenceFrame> handPositionControlFrames;

   private final FrameOrientation orientation = new FrameOrientation(ReferenceFrame.getWorldFrame());
   private final FramePoint position = new FramePoint(ReferenceFrame.getWorldFrame());
   private final double trajectoryTime;
   private final RigidBody base;
   private final ReferenceFrame trajectoryFrame = ReferenceFrame.getWorldFrame();
   private final FramePose finalDesiredPose = new FramePose(trajectoryFrame);
   private final SE3PDGains gains;

   public StraightLinePositionControlState(FingerToroidManipulationState stateEnum,
                                           SideDependentList<IndividualHandControlModule> individualHandControlModules, RigidBody rootBody,
                                           SideDependentList<SE3ConfigurationProvider> finalConfigurationProviders, SideDependentList<ReferenceFrame> handPositionControlFrames,
                                           double trajectoryTime, SE3PDGains gains)
   {
      super(stateEnum);
      this.individualHandControlModules = individualHandControlModules;
      this.finalConfigurationProviders = finalConfigurationProviders;
      this.trajectoryTime = trajectoryTime;
      this.base = rootBody;
      this.handPositionControlFrames = handPositionControlFrames;
      this.gains = gains;
   }

   @Override
   public void doAction()
   {
   }

   @Override
   public void doTransitionIntoAction()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         finalConfigurationProviders.get(robotSide).get(orientation);
         orientation.changeFrame(trajectoryFrame);

         finalConfigurationProviders.get(robotSide).get(position);
         position.changeFrame(trajectoryFrame);

         finalDesiredPose.set(orientation.getReferenceFrame());
         finalDesiredPose.setOrientation(orientation);
         finalDesiredPose.setPosition(position);
         finalDesiredPose.changeFrame(trajectoryFrame);

         ReferenceFrame handPositionControlFrame = handPositionControlFrames.get(robotSide);
         individualHandControlModules.get(robotSide).moveInStraightLine(finalDesiredPose, trajectoryTime, base, handPositionControlFrame,
               trajectoryFrame, false, gains);
      }
   }

   @Override
   public void doTransitionOutOfAction()
   {
      // TODO: automatically generated code
   }

   @Override
   public boolean isDone()
   {
      for (IndividualHandControlModule individualHandControlModule : individualHandControlModules)
      {
         if (!individualHandControlModule.isDone())
            return false;
      }
      return true;
   }
}
