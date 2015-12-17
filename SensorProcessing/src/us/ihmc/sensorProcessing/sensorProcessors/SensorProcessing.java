package us.ihmc.sensorProcessing.sensorProcessors;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import javax.vecmath.Matrix3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.LongYoVariable;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.filters.AlphaFilteredYoFrameQuaternion;
import us.ihmc.robotics.math.filters.AlphaFilteredYoFrameVector;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.robotics.math.filters.BacklashProcessingYoVariable;
import us.ihmc.robotics.math.filters.FilteredVelocityYoVariable;
import us.ihmc.robotics.math.filters.ProcessingYoVariable;
import us.ihmc.robotics.math.filters.RevisedBacklashCompensatingVelocityYoVariable;
import us.ihmc.robotics.math.frames.YoFrameQuaternion;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.Wrench;
import us.ihmc.robotics.sensors.ForceSensorDataHolder;
import us.ihmc.robotics.sensors.ForceSensorDataHolderReadOnly;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.robotics.sensors.IMUDefinition;
import us.ihmc.sensorProcessing.communication.packets.dataobjects.AuxiliaryRobotData;
import us.ihmc.sensorProcessing.imu.IMUSensor;
import us.ihmc.sensorProcessing.simulatedSensors.SensorNoiseParameters;
import us.ihmc.sensorProcessing.simulatedSensors.StateEstimatorSensorDefinitions;
import us.ihmc.sensorProcessing.stateEstimation.IMUSensorReadOnly;
import us.ihmc.sensorProcessing.stateEstimation.SensorProcessingConfiguration;

public class SensorProcessing implements SensorOutputMapReadOnly, SensorRawOutputMapReadOnly
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   public enum SensorType
   {
      JOINT_POSITION, JOINT_VELOCITY, JOINT_ACCELERATION, JOINT_TAU, TORQUE_SENSOR, FORCE_SENSOR, IMU_ORIENTATION, IMU_ANGULAR_VELOCITY, IMU_LINEAR_ACCELERATION;

      public boolean isJointSensor()
      {
         switch (this)
         {
         case JOINT_POSITION:
         case JOINT_VELOCITY:
         case JOINT_ACCELERATION:
         case JOINT_TAU:
            return true;
         default:
            return false;
         }
      }

      public boolean isWrenchSensor()
      {
         return this == SensorType.TORQUE_SENSOR || this == FORCE_SENSOR;
      }

      public boolean isIMUSensor()
      {
         switch (this)
         {
         case IMU_ORIENTATION:
         case IMU_ANGULAR_VELOCITY:
         case IMU_LINEAR_ACCELERATION:
            return true;
         default:
            return false;
         }
      }

      public String getPrefixForProcessorName(String filterNameLowerCaseNoTrailingUnderscore)
      {
         switch (this)
         {
         case JOINT_POSITION:
         case IMU_ORIENTATION:
            return filterNameLowerCaseNoTrailingUnderscore + "_q_";
         case JOINT_VELOCITY:
            return filterNameLowerCaseNoTrailingUnderscore + "_qd_";
         case JOINT_ACCELERATION:
         case IMU_LINEAR_ACCELERATION:
            return filterNameLowerCaseNoTrailingUnderscore + "_qdd_";
         case JOINT_TAU:
            return filterNameLowerCaseNoTrailingUnderscore + "_tau_";
         case IMU_ANGULAR_VELOCITY:
            return filterNameLowerCaseNoTrailingUnderscore + "_qd_w";
         case FORCE_SENSOR:
            return filterNameLowerCaseNoTrailingUnderscore + "_force_";
         case TORQUE_SENSOR:
            return filterNameLowerCaseNoTrailingUnderscore + "_torque_";
         default:
            throw new RuntimeException("Should not get there.");
         }
      }
   };

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final LongYoVariable timestamp = new LongYoVariable("timestamp", registry);
   private final LongYoVariable visionSensorTimestamp = new LongYoVariable("visionSensorTimestamp", registry);
   private final LongYoVariable sensorHeadPPSTimetamp = new LongYoVariable("sensorHeadPPSTimetamp", registry);

   private final LinkedHashMap<OneDoFJoint, DoubleYoVariable> inputJointPositions = new LinkedHashMap<>();
   private final LinkedHashMap<OneDoFJoint, DoubleYoVariable> inputJointVelocities = new LinkedHashMap<>();
   private final LinkedHashMap<OneDoFJoint, DoubleYoVariable> inputJointAccelerations = new LinkedHashMap<>();
   private final LinkedHashMap<OneDoFJoint, DoubleYoVariable> inputJointTaus = new LinkedHashMap<>();

   private final LinkedHashMap<IMUDefinition, YoFrameQuaternion> inputOrientations = new LinkedHashMap<>();
   private final LinkedHashMap<IMUDefinition, YoFrameVector> inputAngularVelocities = new LinkedHashMap<>();
   private final LinkedHashMap<IMUDefinition, YoFrameVector> inputLinearAccelerations = new LinkedHashMap<>();

   private final LinkedHashMap<ForceSensorDefinition, YoFrameVector> inputForces = new LinkedHashMap<>();
   private final LinkedHashMap<ForceSensorDefinition, YoFrameVector> inputTorques = new LinkedHashMap<>();

   private final LinkedHashMap<IMUDefinition, YoFrameQuaternion> intermediateOrientations = new LinkedHashMap<>();
   private final LinkedHashMap<IMUDefinition, YoFrameVector> intermediateAngularVelocities = new LinkedHashMap<>();
   private final LinkedHashMap<IMUDefinition, YoFrameVector> intermediateLinearAccelerations = new LinkedHashMap<>();

   private final LinkedHashMap<ForceSensorDefinition, YoFrameVector> intermediateForces = new LinkedHashMap<>();
   private final LinkedHashMap<ForceSensorDefinition, YoFrameVector> intermediateTorques = new LinkedHashMap<>();

   private final LinkedHashMap<OneDoFJoint, List<ProcessingYoVariable>> processedJointPositions = new LinkedHashMap<>();
   private final LinkedHashMap<OneDoFJoint, List<ProcessingYoVariable>> processedJointVelocities = new LinkedHashMap<>();
   private final LinkedHashMap<OneDoFJoint, List<ProcessingYoVariable>> processedJointAccelerations = new LinkedHashMap<>();
   private final LinkedHashMap<OneDoFJoint, List<ProcessingYoVariable>> processedJointTaus = new LinkedHashMap<>();

   private final LinkedHashMap<IMUDefinition, List<ProcessingYoVariable>> processedOrientations = new LinkedHashMap<>();
   private final LinkedHashMap<IMUDefinition, List<ProcessingYoVariable>> processedAngularVelocities = new LinkedHashMap<>();
   private final LinkedHashMap<IMUDefinition, List<ProcessingYoVariable>> processedLinearAccelerations = new LinkedHashMap<>();

   private final LinkedHashMap<ForceSensorDefinition, List<ProcessingYoVariable>> processedForces = new LinkedHashMap<>();
   private final LinkedHashMap<ForceSensorDefinition, List<ProcessingYoVariable>> processedTorques = new LinkedHashMap<>();

   private final LinkedHashMap<OneDoFJoint, DoubleYoVariable> outputJointPositions = new LinkedHashMap<>();
   private final LinkedHashMap<OneDoFJoint, DoubleYoVariable> outputJointVelocities = new LinkedHashMap<>();
   private final LinkedHashMap<OneDoFJoint, DoubleYoVariable> outputJointAccelerations = new LinkedHashMap<>();
   private final LinkedHashMap<OneDoFJoint, DoubleYoVariable> outputJointTaus = new LinkedHashMap<>();

   private final ArrayList<IMUSensor> inputIMUs = new ArrayList<IMUSensor>();
   private final ArrayList<IMUSensor> outputIMUs = new ArrayList<IMUSensor>();

   private final ForceSensorDataHolder inputForceSensors;
   private final ForceSensorDataHolder outputForceSensors;

   private final List<OneDoFJoint> jointSensorDefinitions;
   private final List<IMUDefinition> imuSensorDefinitions;
   private final List<ForceSensorDefinition> forceSensorDefinitions;

   private final LinkedHashMap<OneDoFJoint, BooleanYoVariable> jointEnabledIndicators = new LinkedHashMap<>();

   private final double updateDT;

   private final Matrix3d tempOrientation = new Matrix3d();
   private final Vector3d tempAngularVelocity = new Vector3d();
   private final Vector3d tempLinearAcceleration = new Vector3d();

   private final FrameVector tempForce = new FrameVector();
   private final FrameVector tempTorque = new FrameVector();
   private final Wrench tempWrench = new Wrench();

   private AuxiliaryRobotData auxiliaryRobotData;

   public SensorProcessing(StateEstimatorSensorDefinitions stateEstimatorSensorDefinitions, SensorProcessingConfiguration sensorProcessingConfiguration,
         YoVariableRegistry parentRegistry)
   {
      this.updateDT = sensorProcessingConfiguration.getEstimatorDT();

      jointSensorDefinitions = stateEstimatorSensorDefinitions.getJointSensorDefinitions();
      imuSensorDefinitions = stateEstimatorSensorDefinitions.getIMUSensorDefinitions();
      forceSensorDefinitions = stateEstimatorSensorDefinitions.getForceSensorDefinitions();
      this.auxiliaryRobotData = null;

      for (int i = 0; i < jointSensorDefinitions.size(); i++)
      {
         OneDoFJoint oneDoFJoint = jointSensorDefinitions.get(i);
         String jointName = oneDoFJoint.getName();
         
         DoubleYoVariable rawJointPosition = new DoubleYoVariable("raw_q_" + jointName, registry);
         inputJointPositions.put(oneDoFJoint, rawJointPosition);
         outputJointPositions.put(oneDoFJoint, rawJointPosition);
         processedJointPositions.put(oneDoFJoint, new ArrayList<ProcessingYoVariable>());

         DoubleYoVariable rawJointVelocity = new DoubleYoVariable("raw_qd_" + jointName, registry);
         inputJointVelocities.put(oneDoFJoint, rawJointVelocity);
         outputJointVelocities.put(oneDoFJoint, rawJointVelocity);
         processedJointVelocities.put(oneDoFJoint, new ArrayList<ProcessingYoVariable>());
         
         DoubleYoVariable rawJointAcceleration = new DoubleYoVariable("raw_qdd_" + jointName, registry);
         inputJointAccelerations.put(oneDoFJoint, rawJointAcceleration);
         outputJointAccelerations.put(oneDoFJoint, rawJointAcceleration);
         processedJointAccelerations.put(oneDoFJoint, new ArrayList<ProcessingYoVariable>());

         DoubleYoVariable rawJointTau = new DoubleYoVariable("raw_tau_" + jointName, registry);
         inputJointTaus.put(oneDoFJoint, rawJointTau);
         outputJointTaus.put(oneDoFJoint, rawJointTau);
         processedJointTaus.put(oneDoFJoint, new ArrayList<ProcessingYoVariable>());

         BooleanYoVariable jointEnabledIndicator = new BooleanYoVariable("joint_enabled_" + jointName, registry);
         jointEnabledIndicator.set(true);
         jointEnabledIndicators.put(oneDoFJoint, jointEnabledIndicator);
      }

      SensorNoiseParameters sensorNoiseParameters = sensorProcessingConfiguration.getSensorNoiseParameters();

      for (int i = 0; i < imuSensorDefinitions.size(); i++)
      {
         IMUDefinition imuDefinition = imuSensorDefinitions.get(i);
         String imuName = imuDefinition.getName();

         YoFrameQuaternion rawOrientation = new YoFrameQuaternion("raw_q_", imuName, worldFrame, registry);
         inputOrientations.put(imuDefinition, rawOrientation);
         intermediateOrientations.put(imuDefinition, rawOrientation);
         processedOrientations.put(imuDefinition, new ArrayList<ProcessingYoVariable>());

         YoFrameVector rawAngularVelocity = new YoFrameVector("raw_qd_w", imuName, worldFrame, registry);
         inputAngularVelocities.put(imuDefinition, rawAngularVelocity);
         intermediateAngularVelocities.put(imuDefinition, rawAngularVelocity);
         processedAngularVelocities.put(imuDefinition, new ArrayList<ProcessingYoVariable>());
         
         YoFrameVector rawLinearAcceleration = new YoFrameVector("raw_qdd_", imuName, worldFrame, registry);
         inputLinearAccelerations.put(imuDefinition, rawLinearAcceleration);
         intermediateLinearAccelerations.put(imuDefinition, rawLinearAcceleration);
         processedLinearAccelerations.put(imuDefinition, new ArrayList<ProcessingYoVariable>());
         
         inputIMUs.add(new IMUSensor(imuDefinition, sensorNoiseParameters));
         outputIMUs.add(new IMUSensor(imuDefinition, sensorNoiseParameters));
      }

      for (int i = 0; i < forceSensorDefinitions.size(); i++)
      {
         ForceSensorDefinition forceSensorDefinition = forceSensorDefinitions.get(i);
         String sensorName = forceSensorDefinition.getSensorName();
         ReferenceFrame sensorFrame = forceSensorDefinition.getSensorFrame();

         YoFrameVector rawForce = new YoFrameVector("raw_" + sensorName + "_force", sensorFrame, registry);
         inputForces.put(forceSensorDefinition, rawForce);
         intermediateForces.put(forceSensorDefinition, rawForce);
         processedForces.put(forceSensorDefinition, new ArrayList<ProcessingYoVariable>());

         YoFrameVector rawTorque = new YoFrameVector("raw_" + sensorName + "_torque", sensorFrame, registry);
         inputTorques.put(forceSensorDefinition, rawTorque);
         intermediateTorques.put(forceSensorDefinition, rawTorque);
         processedTorques.put(forceSensorDefinition, new ArrayList<ProcessingYoVariable>());
      }

      inputForceSensors = new ForceSensorDataHolder(forceSensorDefinitions);
      outputForceSensors = new ForceSensorDataHolder(forceSensorDefinitions);

      sensorProcessingConfiguration.configureSensorProcessing(this);
      parentRegistry.addChild(registry);
   }

   public void initialize()
   {
      startComputation(0, 0, -1);
   }

   public void startComputation(long timestamp, long visionSensorTimestamp, long sensorHeadPPSTimestamp)
   {
      this.timestamp.set(timestamp);
      this.visionSensorTimestamp.set(visionSensorTimestamp);
      this.sensorHeadPPSTimetamp.set(sensorHeadPPSTimestamp);

      for (int i = 0; i < jointSensorDefinitions.size(); i++)
      {
         OneDoFJoint oneDoFJoint = jointSensorDefinitions.get(i);

         updateProcessors(processedJointPositions.get(oneDoFJoint));
         updateProcessors(processedJointVelocities.get(oneDoFJoint));
         updateProcessors(processedJointAccelerations.get(oneDoFJoint));
         updateProcessors(processedJointTaus.get(oneDoFJoint));
      }
      
      for (int i = 0; i < imuSensorDefinitions.size(); i++)
      {
         IMUDefinition imuDefinition = imuSensorDefinitions.get(i);

         IMUSensor inputIMU = inputIMUs.get(i);
         inputOrientations.get(imuDefinition).get(tempOrientation);
         inputAngularVelocities.get(imuDefinition).get(tempAngularVelocity);
         inputLinearAccelerations.get(imuDefinition).get(tempLinearAcceleration);
         inputIMU.setOrientationMeasurement(tempOrientation);
         inputIMU.setAngularVelocityMeasurement(tempAngularVelocity);
         inputIMU.setLinearAccelerationMeasurement(tempLinearAcceleration);

         updateProcessors(processedOrientations.get(imuDefinition));
         updateProcessors(processedAngularVelocities.get(imuDefinition));
         updateProcessors(processedLinearAccelerations.get(imuDefinition));
         
         IMUSensor outputIMU = outputIMUs.get(i);
         intermediateOrientations.get(imuDefinition).get(tempOrientation);
         intermediateAngularVelocities.get(imuDefinition).get(tempAngularVelocity);
         intermediateLinearAccelerations.get(imuDefinition).get(tempLinearAcceleration);
         outputIMU.setOrientationMeasurement(tempOrientation);
         outputIMU.setAngularVelocityMeasurement(tempAngularVelocity);
         outputIMU.setLinearAccelerationMeasurement(tempLinearAcceleration);
      }

      for (int i = 0; i < forceSensorDefinitions.size(); i++)
      {
         ForceSensorDefinition forceSensorDefinition = forceSensorDefinitions.get(i);

         inputForceSensors.getForceSensorValue(forceSensorDefinition, tempWrench);
         tempWrench.packLinearPartIncludingFrame(tempForce); 
         tempWrench.packAngularPartIncludingFrame(tempTorque); 
         inputForces.get(forceSensorDefinition).set(tempForce);
         inputTorques.get(forceSensorDefinition).set(tempTorque);
         
         updateProcessors(processedForces.get(forceSensorDefinition));
         updateProcessors(processedTorques.get(forceSensorDefinition));

         intermediateForces.get(forceSensorDefinition).getFrameTupleIncludingFrame(tempForce);
         intermediateTorques.get(forceSensorDefinition).getFrameTupleIncludingFrame(tempTorque);
         tempWrench.set(tempForce, tempTorque);
         outputForceSensors.setForceSensorValue(forceSensorDefinition, tempWrench);
      }
   }

   private void updateProcessors(List<ProcessingYoVariable> processors)
   {
      for (int j = 0; j < processors.size(); j++)
      {
         processors.get(j).update();
      }
   }

   public void addAlphaFilter(DoubleYoVariable alphaFilter, boolean forVizOnly, SensorType sensorType)
   {
      
   }

   /**
    * Add a low-pass filter stage on the joint positions.
    * This is cumulative, by calling this method twice for instance, you will obtain a two pole low-pass filter.
    * @param alphaFilter low-pass filter parameter.
    * @param forVizOnly if set to true, the result will not be used as the input of the next processing stage, nor as the output of the sensor processing.
    */
   public void addJointPositionAlphaFilter(DoubleYoVariable alphaFilter, boolean forVizOnly)
   {
      for (int i = 0; i < jointSensorDefinitions.size(); i++)
      {
         OneDoFJoint oneDoFJoint = jointSensorDefinitions.get(i);
         String jointName = oneDoFJoint.getName();
         DoubleYoVariable intermediateJointPosition = outputJointPositions.get(oneDoFJoint);
         List<ProcessingYoVariable> processors = processedJointPositions.get(oneDoFJoint);
         String suffix = "_sp" + processors.size();
         AlphaFilteredYoVariable filteredJointPosition = new AlphaFilteredYoVariable("filt_q_" + jointName + suffix, registry, alphaFilter, intermediateJointPosition);
         processedJointPositions.get(oneDoFJoint).add(filteredJointPosition);
         
         if (!forVizOnly)
            outputJointPositions.put(oneDoFJoint, filteredJointPosition);
      }
   }

   /**
    * Apply an elasticity compensator to correct the joint positions according their torque and a given stiffness.
    * Useful when the robot has a non negligible elasticity in the links or joints.
    * Implemented as a cumulative processor but should probably be called only once.
    * @param stiffnesses estimated stiffness for each joint.
    * @param forVizOnly if set to true, the result will not be used as the input of the next processing stage, nor as the output of the sensor processing.
    */
   public void addJointPositionElasticyCompensator(Map<OneDoFJoint, DoubleYoVariable> stiffnesses, boolean forVizOnly)
   {
      addJointPositionElasticyCompensatorWithJointsToIgnore(stiffnesses, forVizOnly);
   }

   public void addJointPositionElasticyCompensatorWithJointsToIgnore(Map<OneDoFJoint, DoubleYoVariable> stiffnesses, boolean forVizOnly, String... jointsToIgnore)
   {
      List<String> jointToIgnoreList = new ArrayList<>();
      if (jointsToIgnore != null && jointsToIgnore.length > 0)
         jointToIgnoreList.addAll(Arrays.asList(jointsToIgnore));

      for (int i = 0; i < jointSensorDefinitions.size(); i++)
      {
         OneDoFJoint oneDoFJoint = jointSensorDefinitions.get(i);
         String jointName = oneDoFJoint.getName();

         if (jointToIgnoreList.contains(jointName))
            continue;

         DoubleYoVariable stiffness = stiffnesses.get(oneDoFJoint);
         DoubleYoVariable intermediateJointPosition = outputJointPositions.get(oneDoFJoint);
         DoubleYoVariable intermediateJointTau = outputJointTaus.get(oneDoFJoint);
         List<ProcessingYoVariable> processors = processedJointPositions.get(oneDoFJoint);
         String suffix = "_sp" + processors.size();
         ElasticityCompensatorYoVariable filteredJointPosition = new ElasticityCompensatorYoVariable("stiff_q_" + jointName + suffix, stiffness, intermediateJointPosition, intermediateJointTau, registry);
         processors.add(filteredJointPosition);
         
         if (!forVizOnly)
            outputJointPositions.put(oneDoFJoint, filteredJointPosition);
      }
   }

   /**
    * Compute the joint velocities by calculating finite-difference on joint positions using {@link FilteredVelocityYoVariable}. It is then automatically low-pass filtered.
    * This is not cumulative and has the effect of ignoring the velocity signal provided by the robot.
    * @param alphaFilter low-pass filter parameter.
    * @param forVizOnly if set to true, the result will not be used as the input of the next processing stage, nor as the output of the sensor processing.
    */
   public void computeJointVelocityFromFiniteDifference(DoubleYoVariable alphaFilter, boolean forVizOnly)
   {
      computeJointVelocityFromFiniteDifferenceWithJointsToIgnore(alphaFilter, forVizOnly);
   }

   /**
    * Compute the joint velocities (for a specific subset of joints) by calculating finite-difference on joint positions using {@link FilteredVelocityYoVariable}. It is then automatically low-pass filtered.
    * This is not cumulative and has the effect of ignoring the velocity signal provided by the robot.
    * @param alphaFilter low-pass filter parameter.
    * @param forVizOnly if set to true, the result will not be used as the input of the next processing stage, nor as the output of the sensor processing.
    * @param jointsToBeProcessed list of the names of the joints that need to be processed.
    */
   public void computeJointVelocityFromFiniteDifferenceOnlyForSpecifiedJoints(DoubleYoVariable alphaFilter, boolean forVizOnly, String... jointsToBeProcessed)
   {
      computeJointVelocityFromFiniteDifferenceWithJointsToIgnore(alphaFilter, forVizOnly, invertJointSelection(jointsToBeProcessed));
   }

   /**
    * Compute the joint velocities (for a specific subset of joints) by calculating finite-difference on joint positions using {@link FilteredVelocityYoVariable}. It is then automatically low-pass filtered.
    * This is not cumulative and has the effect of ignoring the velocity signal provided by the robot.
    * @param alphaFilter low-pass filter parameter.
    * @param forVizOnly if set to true, the result will not be used as the input of the next processing stage, nor as the output of the sensor processing.
    * @param jointsToIgnore list of the names of the joints to ignore.
    */
   public void computeJointVelocityFromFiniteDifferenceWithJointsToIgnore(DoubleYoVariable alphaFilter, boolean forVizOnly, String... jointsToIgnore)
   {
      List<String> jointToIgnoreList = new ArrayList<>();
      if (jointsToIgnore != null && jointsToIgnore.length > 0)
         jointToIgnoreList.addAll(Arrays.asList(jointsToIgnore));

      for (int i = 0; i < jointSensorDefinitions.size(); i++)
      {
         OneDoFJoint oneDoFJoint = jointSensorDefinitions.get(i);
         String jointName = oneDoFJoint.getName();

         if (jointToIgnoreList.contains(jointName))
            continue;

         DoubleYoVariable intermediateJointPosition = outputJointPositions.get(oneDoFJoint);
         List<ProcessingYoVariable> processors = processedJointVelocities.get(oneDoFJoint);
         String suffix = "_sp" + processors.size();
         FilteredVelocityYoVariable jointVelocity = new FilteredVelocityYoVariable("fd_qd_" + jointName + suffix, "", alphaFilter, intermediateJointPosition, updateDT, registry);
         processors.add(jointVelocity);
         
         if (!forVizOnly)
            outputJointVelocities.put(oneDoFJoint, jointVelocity);
      }
   }

   
   /**
    * Compute the joint velocities by calculating finite-difference on joint positions and applying a backlash compensator (see {@link RevisedBacklashCompensatingVelocityYoVariable}). It is then automatically low-pass filtered.
    * This is not cumulative and has the effect of ignoring the velocity signal provided by the robot.
    * @param alphaFilter low-pass filter parameter.
    * @param forVizOnly if set to true, the result will not be used as the input of the next processing stage, nor as the output of the sensor processing.
    */
   public void computeJointVelocityWithBacklashCompensator(DoubleYoVariable alphaFilter, DoubleYoVariable slopTime, boolean forVizOnly)
   {
      computeJointVelocityWithBacklashCompensatorWithJointsToIgnore(alphaFilter, slopTime, forVizOnly);
   }

   /**
    * Compute the joint velocities (for a specific subset of joints) by calculating finite-difference on joint positions and applying a backlash compensator (see {@link RevisedBacklashCompensatingVelocityYoVariable}). It is then automatically low-pass filtered.
    * This is not cumulative and has the effect of ignoring the velocity signal provided by the robot.
    * @param alphaFilter low-pass filter parameter.
    * @param forVizOnly if set to true, the result will not be used as the input of the next processing stage, nor as the output of the sensor processing.
    * @param jointsToBeProcessed list of the names of the joints that need to be processed.
    */
   public void computeJointVelocityWithBacklashCompensatorOnlyForSpecifiedJoints(DoubleYoVariable alphaFilter, DoubleYoVariable slopTime, boolean forVizOnly, String... jointsToBeProcessed)
   {
      computeJointVelocityWithBacklashCompensatorWithJointsToIgnore(alphaFilter, slopTime, forVizOnly, invertJointSelection(jointsToBeProcessed));
   }

   /**
    * Compute the joint velocities (for a specific subset of joints) by calculating finite-difference on joint positions and applying a backlash compensator (see {@link RevisedBacklashCompensatingVelocityYoVariable}). It is then automatically low-pass filtered.
    * This is not cumulative and has the effect of ignoring the velocity signal provided by the robot.
    * @param alphaFilter low-pass filter parameter.
    * @param forVizOnly if set to true, the result will not be used as the input of the next processing stage, nor as the output of the sensor processing.
    * @param jointsToIgnore list of the names of the joints to ignore.
    */
   public void computeJointVelocityWithBacklashCompensatorWithJointsToIgnore(DoubleYoVariable alphaFilter, DoubleYoVariable slopTime, boolean forVizOnly, String... jointsToIgnore)
   {
      List<String> jointToIgnoreList = new ArrayList<>();
      if (jointsToIgnore != null && jointsToIgnore.length > 0)
         jointToIgnoreList.addAll(Arrays.asList(jointsToIgnore));

      for (int i = 0; i < jointSensorDefinitions.size(); i++)
      {
         OneDoFJoint oneDoFJoint = jointSensorDefinitions.get(i);
         String jointName = oneDoFJoint.getName();

         if (jointToIgnoreList.contains(jointName))
            continue;

         DoubleYoVariable intermediateJointPosition = outputJointPositions.get(oneDoFJoint);

         List<ProcessingYoVariable> processors = processedJointVelocities.get(oneDoFJoint);
         String suffix = "_sp" + processors.size();
         RevisedBacklashCompensatingVelocityYoVariable jointVelocity = new RevisedBacklashCompensatingVelocityYoVariable("bl_qd_" + jointName + suffix, "", alphaFilter, intermediateJointPosition, updateDT, slopTime, registry);
         processors.add(jointVelocity);

         if (!forVizOnly)
            outputJointVelocities.put(oneDoFJoint, jointVelocity);
      }
      
   }

   /**
    * Add a low-pass filter stage on the joint velocities.
    * This is cumulative, by calling this method twice for instance, you will obtain a two pole low-pass filter.
    * @param alphaFilter low-pass filter parameter.
    * @param forVizOnly if set to true, the result will not be used as the input of the next processing stage, nor as the output of the sensor processing.
    */
   public void addJointVelocityAlphaFilter(DoubleYoVariable alphaFilter, boolean forVizOnly)
   {
      addJointVelocityAlphaFilterWithJointsToIgnore(alphaFilter, forVizOnly);
   }

   /**
    * Add a low-pass filter stage on the joint velocities for a specific subset of joints.
    * This is cumulative, by calling this method twice for instance, you will obtain a two pole low-pass filter.
    * @param alphaFilter low-pass filter parameter.
    * @param forVizOnly if set to true, the result will not be used as the input of the next processing stage, nor as the output of the sensor processing.
    * @param jointsToBeProcessed list of the names of the joints that need to be filtered.
    */
   public void addJointVelocityAlphaFilterOnlyForSpecifiedJoints(DoubleYoVariable alphaFilter, boolean forVizOnly, String... jointsToBeProcessed)
   {
      addJointVelocityAlphaFilterWithJointsToIgnore(alphaFilter, forVizOnly, invertJointSelection(jointsToBeProcessed));
   }

   /**
    * Add a low-pass filter stage on the joint velocities for a specific subset of joints.
    * This is cumulative, by calling this method twice for instance, you will obtain a two pole low-pass filter.
    * @param alphaFilter low-pass filter parameter.
    * @param forVizOnly if set to true, the result will not be used as the input of the next processing stage, nor as the output of the sensor processing.
    * @param jointsToIgnore list of the names of the joints to ignore.
    */
   public void addJointVelocityAlphaFilterWithJointsToIgnore(DoubleYoVariable alphaFilter, boolean forVizOnly, String... jointsToIgnore)
   {
      List<String> jointToIgnoreList = new ArrayList<>();
      if (jointsToIgnore != null && jointsToIgnore.length > 0)
         jointToIgnoreList.addAll(Arrays.asList(jointsToIgnore));

      for (int i = 0; i < jointSensorDefinitions.size(); i++)
      {
         OneDoFJoint oneDoFJoint = jointSensorDefinitions.get(i);
         String jointName = oneDoFJoint.getName();

         if (jointToIgnoreList.contains(jointName))
            continue;

         DoubleYoVariable intermediateJointVelocity = outputJointVelocities.get(oneDoFJoint);
         List<ProcessingYoVariable> processors = processedJointVelocities.get(oneDoFJoint);
         String suffix = "_sp" + processors.size();
         AlphaFilteredYoVariable filteredJointVelocity = new AlphaFilteredYoVariable("filt_qd_" + jointName + suffix, registry, alphaFilter, intermediateJointVelocity);
         processors.add(filteredJointVelocity);

         if (!forVizOnly)
            outputJointVelocities.put(oneDoFJoint, filteredJointVelocity);
      }
      
   }

   /**
    * Apply a backlash compensator (see {@link BacklashProcessingYoVariable}) to the joint velocity.
    * Useful when the robot has backlash in its joints or simply to calm down small shakies when the robot is at rest.
    * Implemented as a cumulative processor but should probably be called only once.
    * @param slopTime every time the velocity changes sign, a slop is engaged during which a confidence factor is ramped up from 0 to 1.
    * @param forVizOnly if set to true, the result will not be used as the input of the next processing stage, nor as the output of the sensor processing.
    */
   public void addJointVelocityBacklashFilter(DoubleYoVariable slopTime, boolean forVizOnly)
   {
      addJointVelocityAlphaFilterWithJointsToIgnore(slopTime, forVizOnly);
   }

   /**
    * Apply a backlash compensator (see {@link BacklashProcessingYoVariable}) to the joint velocity.
    * Useful when the robot has backlash in its joints or simply to calm down small shakies when the robot is at rest.
    * Implemented as a cumulative processor but should probably be called only once.
    * @param slopTime every time the velocity changes sign, a slop is engaged during which a confidence factor is ramped up from 0 to 1.
    * @param forVizOnly if set to true, the result will not be used as the input of the next processing stage, nor as the output of the sensor processing.
    * @param jointsToBeProcessed list of the names of the joints that need to be filtered.
    */
   public void addJointVelocityBacklashFilterOnlyForSpecifiedJoints(DoubleYoVariable slopTime, boolean forVizOnly, String... jointsToBeProcessed)
   {
      addJointVelocityAlphaFilterWithJointsToIgnore(slopTime, forVizOnly, invertJointSelection(jointsToBeProcessed));
   }

   /**
    * Apply a backlash compensator (see {@link BacklashProcessingYoVariable}) to the joint velocity.
    * Useful when the robot has backlash in its joints or simply to calm down small shakies when the robot is at rest.
    * Implemented as a cumulative processor but should probably be called only once.
    * @param slopTime every time the velocity changes sign, a slop is engaged during which a confidence factor is ramped up from 0 to 1.
    * @param forVizOnly if set to true, the result will not be used as the input of the next processing stage, nor as the output of the sensor processing.
    * @param jointsToIgnore list of the names of the joints to ignore.
    */
   public void addJointVelocityBacklashFilterWithJointsToIgnore(DoubleYoVariable slopTime, boolean forVizOnly, String... jointsToIgnore)
   {
      List<String> jointToIgnoreList = new ArrayList<>();
      if (jointsToIgnore != null && jointsToIgnore.length > 0)
         jointToIgnoreList.addAll(Arrays.asList(jointsToIgnore));

      for (int i = 0; i < jointSensorDefinitions.size(); i++)
      {
         OneDoFJoint oneDoFJoint = jointSensorDefinitions.get(i);
         String jointName = oneDoFJoint.getName();

         if (jointToIgnoreList.contains(jointName))
            continue;

         DoubleYoVariable intermediateJointVelocity = outputJointVelocities.get(oneDoFJoint);
         List<ProcessingYoVariable> processors = processedJointVelocities.get(oneDoFJoint);
         String suffix = "_sp" + processors.size();
         BacklashProcessingYoVariable filteredJointVelocity = new BacklashProcessingYoVariable("bl_qd_" + jointName + suffix, "", intermediateJointVelocity, updateDT, slopTime, registry);
         processors.add(filteredJointVelocity);

         if (!forVizOnly)
            outputJointVelocities.put(oneDoFJoint, filteredJointVelocity);
      }
   }

   /**
    * Compute the joint accelerations by calculating finite-difference on joint velocities using {@link FilteredVelocityYoVariable}. It is then automatically low-pass filtered.
    * @param alphaFilter low-pass filter parameter.
    * @param forVizOnly if set to true, the result will not be used as the input of the next processing stage, nor as the output of the sensor processing.
    */
   public void computeJointAccelerationFromFiniteDifference(DoubleYoVariable alphaFilter, boolean forVizOnly)
   {
      computeJointAccelerationFromFiniteDifferenceWithJointsToIgnore(alphaFilter, forVizOnly);
   }

   /**
    * Compute the joint accelerations (for a specific subset of joints) by calculating finite-difference on joint velocities using {@link FilteredVelocityYoVariable}. It is then automatically low-pass filtered.
    * @param alphaFilter low-pass filter parameter.
    * @param forVizOnly if set to true, the result will not be used as the input of the next processing stage, nor as the output of the sensor processing.
    * @param jointsToIgnore list of the names of the joints to ignore.
    */
   public void computeJointAccelerationFromFiniteDifferenceWithJointsToIgnore(DoubleYoVariable alphaFilter, boolean forVizOnly, String... jointsToIgnore)
   {
      List<String> jointToIgnoreList = new ArrayList<>();
      if (jointsToIgnore != null && jointsToIgnore.length > 0)
         jointToIgnoreList.addAll(Arrays.asList(jointsToIgnore));

      for (int i = 0; i < jointSensorDefinitions.size(); i++)
      {
         OneDoFJoint oneDoFJoint = jointSensorDefinitions.get(i);
         String jointName = oneDoFJoint.getName();

         if (jointToIgnoreList.contains(jointName))
            continue;

         DoubleYoVariable intermediateJointVelocity = outputJointVelocities.get(oneDoFJoint);
         List<ProcessingYoVariable> processors = processedJointAccelerations.get(oneDoFJoint);
         String suffix = "_sp" + processors.size();
         FilteredVelocityYoVariable jointAcceleration = new FilteredVelocityYoVariable("filt_qdd_" + jointName + suffix, "", alphaFilter, intermediateJointVelocity, updateDT, registry);
         processors.add(jointAcceleration);

         if (!forVizOnly)
            outputJointAccelerations.put(oneDoFJoint, jointAcceleration);
      }
   }

   /**
    * Add a low-pass filter stage on the orientations provided by the IMU sensors.
    * This is cumulative, by calling this method twice for instance, you will obtain a two pole low-pass filter.
    * @param alphaFilter low-pass filter parameter.
    * @param forVizOnly if set to true, the result will not be used as the input of the next processing stage, nor as the output of the sensor processing.
    */
   public void addIMUOrientationAlphaFilter(DoubleYoVariable alphaFilter, boolean forVizOnly)
   {
      for (int i = 0; i < imuSensorDefinitions.size(); i++)
      {
         IMUDefinition imuDefinition = imuSensorDefinitions.get(i);
         String imuName = imuDefinition.getName();
         YoFrameQuaternion intermediateOrientation = intermediateOrientations.get(imuDefinition);
         List<ProcessingYoVariable> processors = processedOrientations.get(imuDefinition);
         String suffix = "_sp" + processors.size();
         AlphaFilteredYoFrameQuaternion filteredOrientation = new AlphaFilteredYoFrameQuaternion("filt_q_", imuName + suffix, intermediateOrientation, alphaFilter, registry);
         processors.add(filteredOrientation);
         
         if (!forVizOnly)
            intermediateOrientations.put(imuDefinition, filteredOrientation);
      }
   }

   /**
    * Add a low-pass filter stage on the angular velocities provided by the IMU sensors.
    * This is cumulative, by calling this method twice for instance, you will obtain a two pole low-pass filter.
    * @param alphaFilter low-pass filter parameter.
    * @param forVizOnly if set to true, the result will not be used as the input of the next processing stage, nor as the output of the sensor processing.
    */
   public void addIMUAngularVelocityAlphaFilter(DoubleYoVariable alphaFilter, boolean forVizOnly)
   {
      for (int i = 0; i < imuSensorDefinitions.size(); i++)
      {
         IMUDefinition imuDefinition = imuSensorDefinitions.get(i);
         String imuName = imuDefinition.getName();
         YoFrameVector intermediateAngularVelocity = intermediateAngularVelocities.get(imuDefinition);
         List<ProcessingYoVariable> processors = processedAngularVelocities.get(imuDefinition);
         String suffix = "_sp" + processors.size();
         AlphaFilteredYoFrameVector filteredAngularVelocity = AlphaFilteredYoFrameVector.createAlphaFilteredYoFrameVector("filt_qd_w", imuName + suffix, registry, alphaFilter, intermediateAngularVelocity);
         processors.add(filteredAngularVelocity);
         
         if (!forVizOnly)
            intermediateAngularVelocities.put(imuDefinition, filteredAngularVelocity);
      }
   }

   /**
    * Add a low-pass filter stage on the linear accelerations provided by the IMU sensors.
    * This is cumulative, by calling this method twice for instance, you will obtain a two pole low-pass filter.
    * @param alphaFilter low-pass filter parameter.
    * @param forVizOnly if set to true, the result will not be used as the input of the next processing stage, nor as the output of the sensor processing.
    */
   public void addIMULinearAccelerationAlphaFilter(DoubleYoVariable alphaFilter, boolean forVizOnly)
   {
      for (int i = 0; i < imuSensorDefinitions.size(); i++)
      {
         IMUDefinition imuDefinition = imuSensorDefinitions.get(i);
         String imuName = imuDefinition.getName();
         YoFrameVector intermediateLinearAcceleration = intermediateLinearAccelerations.get(imuDefinition);
         List<ProcessingYoVariable> processors = processedLinearAccelerations.get(imuDefinition);
         String suffix = "_sp" + processors.size();
         AlphaFilteredYoFrameVector filteredLinearAcceleration = AlphaFilteredYoFrameVector.createAlphaFilteredYoFrameVector("filt_qdd_", imuName + suffix, registry, alphaFilter, intermediateLinearAcceleration);
         processors.add(filteredLinearAcceleration);
         
         if (!forVizOnly)
            intermediateLinearAccelerations.put(imuDefinition, filteredLinearAcceleration);
      }
   }

   public void addForceSensorAlphaFilter(DoubleYoVariable alphaFilter, boolean forVizOnly)
   {
      addForceSensorAlphaFilterWithSensorsToIgnore(alphaFilter, forVizOnly);
   }

   public void addForceSensorAlphaFilterOnlyForSpecifiedSensors(DoubleYoVariable alphaFilter, boolean forVizOnly, String... sensorsToBeProcessed)
   {
      addForceSensorAlphaFilterWithSensorsToIgnore(alphaFilter, forVizOnly, invertForceSensorsSelection(sensorsToBeProcessed));
   }

   public void addForceSensorAlphaFilterWithSensorsToIgnore(DoubleYoVariable alphaFilter, boolean forVizOnly, String... sensorsToIgnore)
   {
      List<String> sensorToIgnoreList = new ArrayList<>();
      if (sensorsToIgnore != null && sensorsToIgnore.length > 0)
         sensorToIgnoreList.addAll(Arrays.asList(sensorsToIgnore));

      for (int i = 0; i < forceSensorDefinitions.size(); i++)
      {
         ForceSensorDefinition forceSensorDefinition = forceSensorDefinitions.get(i);
         String sensorName = forceSensorDefinition.getSensorName();

         if (sensorToIgnoreList.contains(sensorName))
            continue;

         YoFrameVector intermediateForce = intermediateForces.get(forceSensorDefinition);
         List<ProcessingYoVariable> forceProcessors = processedForces.get(forceSensorDefinition);
         String forceSuffix = "_sp" + forceProcessors.size();
         AlphaFilteredYoFrameVector filteredForce = AlphaFilteredYoFrameVector.createAlphaFilteredYoFrameVector("filt_" + sensorName + "_force", forceSuffix, registry, alphaFilter, intermediateForce);
         forceProcessors.add(filteredForce);

         YoFrameVector intermediateTorque = intermediateTorques.get(forceSensorDefinition);
         List<ProcessingYoVariable> torqueProcessors = processedTorques.get(forceSensorDefinition);
         String torqueSuffix = "_sp" + torqueProcessors.size();
         AlphaFilteredYoFrameVector filteredTorque = AlphaFilteredYoFrameVector.createAlphaFilteredYoFrameVector("filt_" + sensorName + "_torque", torqueSuffix, registry, alphaFilter, intermediateTorque);
         torqueProcessors.add(filteredTorque);
         
         if (!forVizOnly)
         {
            intermediateForces.put(forceSensorDefinition, filteredForce);
            intermediateTorques.put(forceSensorDefinition, filteredTorque);
         }
      }
   }

   /**
    * Add a low-pass filter stage on the joint torques.
    * This is cumulative, by calling this method twice for instance, you will obtain a two pole low-pass filter.
    * @param alphaFilter low-pass filter parameter.
    * @param forVizOnly if set to true, the result will not be used as the input of the next processing stage, nor as the output of the sensor processing.
    */
   public void addJointTauAlphaFilter(DoubleYoVariable alphaFilter, boolean forVizOnly)
   {
      addJointTauAlphaFilterWithJointsToIgnore(alphaFilter, forVizOnly);
   }

   /**
    * Add a low-pass filter stage on the joint torques for a specific subset of joints.
    * This is cumulative, by calling this method twice for instance, you will obtain a two pole low-pass filter.
    * @param alphaFilter low-pass filter parameter.
    * @param forVizOnly if set to true, the result will not be used as the input of the next processing stage, nor as the output of the sensor processing.
    * @param jointsToBeProcessed list of the names of the joints that need to be filtered.
    */
   public void addJointTauAlphaFilterOnlyForSpecifiedJoints(DoubleYoVariable alphaFilter, boolean forVizOnly, String... jointsToBeProcessed)
   {
      addJointTauAlphaFilterWithJointsToIgnore(alphaFilter, forVizOnly, invertJointSelection(jointsToBeProcessed));
   }

   /**
    * Add a low-pass filter stage on the joint torques for a specific subset of joints.
    * This is cumulative, by calling this method twice for instance, you will obtain a two pole low-pass filter.
    * @param alphaFilter low-pass filter parameter.
    * @param forVizOnly if set to true, the result will not be used as the input of the next processing stage, nor as the output of the sensor processing.
    * @param jointsToIgnore list of the names of the joints to ignore.
    */
   public void addJointTauAlphaFilterWithJointsToIgnore(DoubleYoVariable alphaFilter, boolean forVizOnly, String... jointsToIgnore)
   {
      List<String> jointToIgnoreList = new ArrayList<>();
      if (jointsToIgnore != null && jointsToIgnore.length > 0)
         jointToIgnoreList.addAll(Arrays.asList(jointsToIgnore));

      for (int i = 0; i < jointSensorDefinitions.size(); i++)
      {
         OneDoFJoint oneDoFJoint = jointSensorDefinitions.get(i);
         String jointName = oneDoFJoint.getName();

         if (jointToIgnoreList.contains(jointName))
            continue;

         DoubleYoVariable intermediateJointTaus = outputJointTaus.get(oneDoFJoint);
         List<ProcessingYoVariable> processors = processedJointTaus.get(oneDoFJoint);
         String suffix = "_sp" + processors.size();
         AlphaFilteredYoVariable filteredJointTaus = new AlphaFilteredYoVariable("filt_tau_" + jointName + suffix, registry, alphaFilter, intermediateJointTaus);
         processors.add(filteredJointTaus);

         if (!forVizOnly)
            outputJointTaus.put(oneDoFJoint, filteredJointTaus);
      }
   }

   /**
    * Create an alpha filter given a name and a break frequency (in Hertz) that will be registered in the {@code SensorProcessing}'s {@code YoVariableRegistry}.
    * @param name name of the variable.
    * @param breakFrequency break frequency in Hertz
    * @return a {@code DoubleYoVariable} to be used when adding a low-pass filter stage using the methods in this class such as {@link SensorProcessing#addJointVelocityAlphaFilter(DoubleYoVariable, boolean)}.
    */
   public DoubleYoVariable createAlphaFilter(String name, double breakFrequency)
   {
      DoubleYoVariable alphaFilter = new DoubleYoVariable(name, registry);
      alphaFilter.set(AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(breakFrequency, updateDT));
      return alphaFilter;
   }

   /**
    * Helper to create easily a {@code Map<OneDoFJoint, DoubleYoVariable>} referring to the stiffness for each joint.
    * @param nameSuffix suffix to be used in the variables' name.
    * @param defaultStiffness default value of stiffness to use when not referred in the jointSpecificStiffness.
    * @param jointSpecificStiffness {@code Map<String, Double>} referring the specific stiffness value to be used for each joint. Does not need to be exhaustive, can also be empty or null in which the defaultStiffness is used for every joint.
    * @return {@code Map<OneDoFJoint, DoubleYoVariable>} to be used when calling {@link SensorProcessing#addJointPositionElasticyCompensator(Map, boolean)}.
    */
   public Map<OneDoFJoint, DoubleYoVariable> createStiffness(String nameSuffix, double defaultStiffness, Map<String, Double> jointSpecificStiffness)
   {
      return createStiffnessWithJointsToIgnore(nameSuffix, defaultStiffness, jointSpecificStiffness);
   }

   public Map<OneDoFJoint, DoubleYoVariable> createStiffnessWithJointsToIgnore(String nameSuffix, double defaultStiffness, Map<String, Double> jointSpecificStiffness, String... jointsToIgnore)
   {
      List<String> jointToIgnoreList = new ArrayList<>();
      if (jointsToIgnore != null && jointsToIgnore.length > 0)
         jointToIgnoreList.addAll(Arrays.asList(jointsToIgnore));

      LinkedHashMap<OneDoFJoint, DoubleYoVariable> stiffesses = new LinkedHashMap<>();
      for (int i = 0; i < jointSensorDefinitions.size(); i++)
      {
         OneDoFJoint oneDoFJoint = jointSensorDefinitions.get(i);
         String jointName = oneDoFJoint.getName();
         
         if (jointToIgnoreList.contains(jointName))
            continue;

         DoubleYoVariable stiffness = new DoubleYoVariable(jointName + nameSuffix, registry);

         if (jointSpecificStiffness != null && jointSpecificStiffness.containsKey(jointName))
            stiffness.set(jointSpecificStiffness.get(jointName));
         else
            stiffness.set(defaultStiffness);
         
         stiffesses.put(oneDoFJoint, stiffness);
      }
      
      return stiffesses;
   }

   private String[] invertJointSelection(String... subSelection)
   {
      List<String> invertSelection = new ArrayList<>();
      List<String> originalJointSensorSelectionList = new ArrayList<>();
      if (subSelection != null && subSelection.length > 0)
         originalJointSensorSelectionList.addAll(Arrays.asList(subSelection));

      for (int i = 0; i < jointSensorDefinitions.size(); i++)
      {
         String jointName = jointSensorDefinitions.get(i).getName();
         if (!originalJointSensorSelectionList.contains(jointName))
            invertSelection.add(jointName);
      }
      return invertSelection.toArray(new String[0]);
   }

   private String[] invertForceSensorsSelection(String... subSelection)
   {
      List<String> invertSelection = new ArrayList<>();
      List<String> originalForceSensorSelectionList = new ArrayList<>();
      if (subSelection != null && subSelection.length > 0)
         originalForceSensorSelectionList.addAll(Arrays.asList(subSelection));

      for (int i = 0; i < forceSensorDefinitions.size(); i++)
      {
         String forceSensorName = forceSensorDefinitions.get(i).getSensorName();
         if (!originalForceSensorSelectionList.contains(forceSensorName))
            invertSelection.add(forceSensorName);
      }
      return invertSelection.toArray(new String[0]);
   }

   @Override
   public long getTimestamp()
   {
      return timestamp.getLongValue();
   }

   @Override
   public long getVisionSensorTimestamp()
   {
      return visionSensorTimestamp.getLongValue();
   }
   
   @Override
   public long getSensorHeadPPSTimestamp()
   {
      return sensorHeadPPSTimetamp.getLongValue();
   }

   public void setJointEnabled(OneDoFJoint oneDoFJoint, boolean enabled)
   {
      jointEnabledIndicators.get(oneDoFJoint).set(enabled);
   }

   public void setJointPositionSensorValue(OneDoFJoint oneDoFJoint, double value)
   {
      inputJointPositions.get(oneDoFJoint).set(value);
   }

   public void setJointVelocitySensorValue(OneDoFJoint oneDoFJoint, double value)
   {
      inputJointVelocities.get(oneDoFJoint).set(value);
   }
   
   public void setJointAccelerationSensorValue(OneDoFJoint oneDoFJoint, double value)
   {
      inputJointAccelerations.get(oneDoFJoint).set(value);
   }

   public void setJointTauSensorValue(OneDoFJoint oneDoFJoint, double value)
   {
      inputJointTaus.get(oneDoFJoint).set(value);
   }

   public void setOrientationSensorValue(IMUDefinition imuDefinition, Quat4d value)
   {
      inputOrientations.get(imuDefinition).set(value);
   }

   public void setOrientationSensorValue(IMUDefinition imuDefinition, Matrix3d value)
   {
      inputOrientations.get(imuDefinition).set(value);
   }

   public void setAngularVelocitySensorValue(IMUDefinition imuDefinition, Vector3d value)
   {
      inputAngularVelocities.get(imuDefinition).set(value);
   }

   public void setLinearAccelerationSensorValue(IMUDefinition imuDefinition, Vector3d value)
   {
      inputLinearAccelerations.get(imuDefinition).set(value);
   }

   public void setForceSensorValue(ForceSensorDefinition forceSensorDefinition, DenseMatrix64F value)
   {
      if (value.getNumRows() != Wrench.SIZE || value.getNumCols() != 1)
         throw new RuntimeException("Unexpected size");

      inputForceSensors.setForceSensorValue(forceSensorDefinition, value);
   }

   @Override
   public double getJointPositionProcessedOutput(OneDoFJoint oneDoFJoint)
   {
      return outputJointPositions.get(oneDoFJoint).getDoubleValue();
   }

   @Override
   public double getJointVelocityProcessedOutput(OneDoFJoint oneDoFJoint)
   {
      return outputJointVelocities.get(oneDoFJoint).getDoubleValue();
   }
   
   @Override
   public double getJointAccelerationProcessedOutput(OneDoFJoint oneDoFJoint)
   {
      return outputJointAccelerations.get(oneDoFJoint).getDoubleValue();
   }

   @Override
   public double getJointTauProcessedOutput(OneDoFJoint oneDoFJoint)
   {
      return outputJointTaus.get(oneDoFJoint).getDoubleValue();
   }

   @Override
   public List<? extends IMUSensorReadOnly> getIMUProcessedOutputs()
   {
      return outputIMUs;
   }

   @Override
   public ForceSensorDataHolderReadOnly getForceSensorProcessedOutputs()
   {
      return outputForceSensors;
   }

   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   @Override
   public double getJointPositionRawOutput(OneDoFJoint oneDoFJoint)
   {
      return inputJointPositions.get(oneDoFJoint).getDoubleValue();
   }

   @Override
   public double getJointVelocityRawOutput(OneDoFJoint oneDoFJoint)
   {
      return inputJointVelocities.get(oneDoFJoint).getDoubleValue();
   }

   @Override
   public double getJointAccelerationRawOutput(OneDoFJoint oneDoFJoint)
   {
      return inputJointAccelerations.get(oneDoFJoint).getDoubleValue();
   }

   @Override
   public double getJointTauRawOutput(OneDoFJoint oneDoFJoint)
   {
      return inputJointTaus.get(oneDoFJoint).getDoubleValue();
   }

   @Override
   public boolean isJointEnabled(OneDoFJoint oneDoFJoint)
   {
      return jointEnabledIndicators.get(oneDoFJoint).getBooleanValue();
   }

   @Override
   public List<? extends IMUSensorReadOnly> getIMURawOutputs()
   {
      return inputIMUs;
   }

   @Override
   public ForceSensorDataHolderReadOnly getForceSensorRawOutputs()
   {
      return inputForceSensors;
   }

   @Override
   public AuxiliaryRobotData getAuxiliaryRobotData()
   {
      return this.auxiliaryRobotData;
   }

   public void setAuxiliaryRobotData(AuxiliaryRobotData auxiliaryRobotData)
   {
      this.auxiliaryRobotData = auxiliaryRobotData;
   }
}
