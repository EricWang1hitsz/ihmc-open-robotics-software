package us.ihmc.commonWalkingControlModules.kinematics;

import us.ihmc.commonWalkingControlModules.dynamics.FullRobotModel;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegJointName;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegJointVelocities;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegTorques;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.screwTheory.GeometricJacobian;
import us.ihmc.utilities.screwTheory.InverseDynamicsJoint;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.ScrewTools;
import us.ihmc.utilities.screwTheory.SpatialAccelerationVector;
import us.ihmc.utilities.screwTheory.Twist;
import us.ihmc.utilities.screwTheory.Wrench;

import com.mathworks.jama.Matrix;

public class SwingFullLegJacobian
{
   private final RobotSide robotSide;
   private final GeometricJacobian geometricJacobian;
   
   /**
    * Constructs a new SwingFullLegJacobian, for the given side of the robot
    */
   public SwingFullLegJacobian(RobotSide robotSide, FullRobotModel fullRobotModel)
   {
      this.robotSide = robotSide;
      RigidBody pelvis = fullRobotModel.getPelvis();
      RigidBody foot = fullRobotModel.getFoot(robotSide);
      geometricJacobian = new GeometricJacobian(pelvis, foot, foot.getBodyFixedFrame());
   }

   /**
    * Computes the underlying openChainJacobian and the vtpJacobian
    */
   public void computeJacobian()
   {
      geometricJacobian.compute();
   }

   /**
    * @return the determinant of the Jacobian matrix
    */
   public double det()
   {
      return geometricJacobian.det();
   }

   /**
    * Returns the twist of the ankle pitch frame with respect to the pelvis frame, expressed in the ankle pitch frame,
    * corresponding to the given joint velocities.
    */
   public Twist getTwistOfFootWithRespectToPelvisInFootFrame(LegJointVelocities jointVelocities)
   {
      Matrix jointVelocitiesVector = jointVelocities.toMatrix();
      return geometricJacobian.getTwist(jointVelocitiesVector);
   }
   
   /**
    * Packs the joint velocities corresponding to the twist of the foot, with respect to the pelvis, expressed in ankle pitch frame
    * @param anklePitchTwistInAnklePitchFrame
    * @return corresponding joint velocities
    */
   public void packJointVelocitiesGivenTwist(LegJointVelocities legJointVelocitiesToPack, Twist anklePitchTwistInAnklePitchFrame, double alpha)
   {
      Matrix jointVelocities = geometricJacobian.computeJointVelocities(anklePitchTwistInAnklePitchFrame, alpha);
      int i = 0;
      for (LegJointName legJointName : legJointVelocitiesToPack.getLegJointNames())
      {
         legJointVelocitiesToPack.setJointVelocity(legJointName, jointVelocities.get(i++, 0));
      }
   }

   /**
    * Packs a LegTorques object with the torques corresponding to the given wrench on the foot.
    */
   public void packLegTorques(LegTorques legTorquesToPack, Wrench wrenchOnFootInFootFrame)
   {
      // check that the LegTorques object we're packing has the correct RobotSide.
      if (this.robotSide != legTorquesToPack.getRobotSide())
      {
         throw new RuntimeException("legTorques object has the wrong RobotSide");
      }

      // the actual computation
      Matrix jointTorques = geometricJacobian.computeJointTorques(wrenchOnFootInFootFrame);

      int i = 0;
      for (LegJointName legJointName : legTorquesToPack.getLegJointNames())
      {
         legTorquesToPack.setTorque(legJointName, jointTorques.get(i++, 0));
      }
   }

   public RobotSide getRobotSide()
   {
      return robotSide;
   }

   public Matrix computeJointAccelerations(SpatialAccelerationVector accelerationOfFootWithRespectToBody, SpatialAccelerationVector jacobianDerivativeTerm, double alpha)
   {
      Matrix biasedAccelerations = accelerationOfFootWithRespectToBody.toMatrix();    // unbiased at this point
      Matrix bias = jacobianDerivativeTerm.toMatrix();
      biasedAccelerations.minusEquals(bias);
      Matrix ret = geometricJacobian.solveUsingDampedLeastSquares(biasedAccelerations, alpha);

      return ret;
   }

   /**
    * For testing purposes only.
    */
   public Matrix getJacobian()
   {
      return geometricJacobian.getJacobianMatrix().copy();
   }
   
   public String toString()
   {
      return geometricJacobian.toString();
   }

   public GeometricJacobian getGeometricJacobian()
   {
      return geometricJacobian;
   }
}
