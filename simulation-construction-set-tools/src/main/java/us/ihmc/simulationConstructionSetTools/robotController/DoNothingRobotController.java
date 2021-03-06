package us.ihmc.simulationConstructionSetTools.robotController;

import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class DoNothingRobotController implements RobotController
{
   private final YoVariableRegistry registry = new YoVariableRegistry("DoNothing");

   @Override
   public void initialize()
   {
   }

   @Override
   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   @Override
   public String getName()
   {
      return "DoNothing";
   }

   @Override
   public String getDescription()
   {
      return getName();
   }

   @Override
   public void doControl()
   {
   }

}
