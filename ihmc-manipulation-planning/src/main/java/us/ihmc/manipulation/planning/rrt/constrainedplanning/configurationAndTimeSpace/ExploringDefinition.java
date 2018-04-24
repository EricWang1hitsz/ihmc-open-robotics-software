package us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

import controller_msgs.msg.dds.KinematicsToolboxRigidBodyMessage;
import us.ihmc.commons.PrintTools;
import us.ihmc.humanoidRobotics.communication.wholeBodyTrajectoryToolboxAPI.RigidBodyExplorationConfigurationCommand;
import us.ihmc.humanoidRobotics.communication.wholeBodyTrajectoryToolboxAPI.WaypointBasedTrajectoryCommand;
import us.ihmc.robotics.screwTheory.RigidBody;

public class ExploringDefinition
{
   private static List<ExploringRigidBody> allExploringRigidBodies = new ArrayList<>();

   public ExploringDefinition(List<WaypointBasedTrajectoryCommand> endEffectorTrajectories,
                              List<RigidBodyExplorationConfigurationCommand> explorationConfigurations)
   {
      Map<RigidBody, WaypointBasedTrajectoryCommand> rigidBodyToTrajectoryMap = new HashMap<>();
      Map<RigidBody, RigidBodyExplorationConfigurationCommand> rigidBodyToExploringMap = new HashMap<>();

      for (int i = 0; i < endEffectorTrajectories.size(); i++)
      {
         rigidBodyToTrajectoryMap.put(endEffectorTrajectories.get(i).getEndEffector(), endEffectorTrajectories.get(i));
         PrintTools.info("" + endEffectorTrajectories.get(i).getEndEffector());
      }

      for (int i = 0; i < explorationConfigurations.size(); i++)
      {
         rigidBodyToExploringMap.put(explorationConfigurations.get(i).getRigidBody(), explorationConfigurations.get(i));
         PrintTools.info("" + explorationConfigurations.get(i).getRigidBody());
      }

      Set<RigidBody> rigidBodySet = new HashSet<>();
      rigidBodySet.addAll(rigidBodyToTrajectoryMap.keySet());
      rigidBodySet.addAll(rigidBodyToExploringMap.keySet());

      List<RigidBody> allRigidBodies = new ArrayList<>(rigidBodySet);

      for (int i = 0; i < allRigidBodies.size(); i++)
      {
         PrintTools.info("" + allRigidBodies.get(i).getName());

         ExploringRigidBody exploringRigidBody = new ExploringRigidBody(allRigidBodies.get(i), rigidBodyToTrajectoryMap.get(allRigidBodies.get(i)),
                                                                        rigidBodyToExploringMap.get(allRigidBodies.get(i)));
         allExploringRigidBodies.add(exploringRigidBody);
      }
   }

   public SpatialData getRandomSpatialData()
   {
      SpatialData randomSpatialData = new SpatialData();
      for (int i = 0; i < allExploringRigidBodies.size(); i++)
      {
         allExploringRigidBodies.get(i).appendRandomSpatialData(randomSpatialData);
      }

      return randomSpatialData;
   }

   // TODO
   public List<KinematicsToolboxRigidBodyMessage> createMessages(SpatialNode node)
   {
      List<KinematicsToolboxRigidBodyMessage> messages = new ArrayList<>();
      double timeInTrajectory = node.getTime();
      for (int i = 0; i < node.getSize(); i++)
      {
         //         RigidBody rigidBody = nameToRigidBodyMap.get(node.getName(i));
         //
         //         Pose3D poseToAppend = node.getSpatialData(i);
         //
         //         KinematicsToolboxRigidBodyMessage message = rigidBodyDataMap.get(rigidBody).createMessage(timeInTrajectory, poseToAppend);
         //         messages.add(message);
      }

      return messages;
   }
}