package us.ihmc.footstepPlanning;

import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.pathPlanning.DataSet;
import us.ihmc.pathPlanning.DataSetIOTools;
import us.ihmc.pathPlanning.DataSetName;
import us.ihmc.pathPlanning.PlannerInput;
import us.ihmc.robotics.robotSide.RobotSide;

import java.text.SimpleDateFormat;
import java.util.Date;

public class FootstepPlanningModuleTimingTest
{
   public FootstepPlanningModuleTimingTest()
   {
      FootstepPlanningModule planningModule = new FootstepPlanningModule(getClass().getSimpleName());
      DataSet dataSet = DataSetIOTools.loadDataSet(DataSetName._20190219_182005_Random);
      PlannerInput plannerInput = dataSet.getPlannerInput();

      FootstepPlannerRequest request = new FootstepPlannerRequest();
      request.setTimeout(Double.MAX_VALUE);
      Pose3D initialMidFootPose = new Pose3D(plannerInput.getStartPosition(), new Quaternion(plannerInput.getStartYaw(), 0.0, 0.0));
      Pose3D goalMidFootPose = new Pose3D(plannerInput.getGoalPosition(), new Quaternion(plannerInput.getGoalYaw(), 0.0, 0.0));
      request.setStartFootPoses(planningModule.getFootstepPlannerParameters().getIdealFootstepWidth(), initialMidFootPose);
      request.setGoalFootPoses(planningModule.getFootstepPlannerParameters().getIdealFootstepWidth(), goalMidFootPose);
      request.setRequestedInitialStanceSide(RobotSide.LEFT);
      request.setPlanarRegionsList(dataSet.getPlanarRegionsList());
      request.setPlanBodyPath(false);
      request.setAbortIfBodyPathPlannerFails(false);

      int warmups = 3;
      for (int i = 0; i < warmups; i++)
      {
         planningModule.handleRequest(request);
      }

      FootstepPlannerOutput output = planningModule.handleRequest(request);
      long iterations = output.getPlannerTimings().getStepPlanningIterations();
      double timePlanningStepsSeconds = output.getPlannerTimings().getTimePlanningStepsSeconds();

      double timePerIteration = timePlanningStepsSeconds / iterations;

      String date = new SimpleDateFormat("yyyyMMdd_HHmmss").format(new Date());
      System.out.println("time per iteration:");
      System.out.println(date + "\t\t" + timePerIteration);
   }

   //
   // Date                 Time per iteration      Note
   // 20200409_141222		0.022284476169492553    Before reducing branch factor
   //
   //
   //

   public static void main(String[] args)
   {
      new FootstepPlanningModuleTimingTest();
   }
}
