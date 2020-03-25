package us.ihmc.footstepPlanning.ui.components;

import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.footstepPlanning.*;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParameters;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersReadOnly;
import us.ihmc.log.LogTools;
import us.ihmc.messager.Messager;
import us.ihmc.messager.SharedMemoryMessager;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.DefaultVisibilityGraphParameters;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.VisibilityGraphsParametersReadOnly;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.ArrayList;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.atomic.AtomicReference;

import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.*;

public class FootstepPathCalculatorModule
{
   private static final boolean VERBOSE = true;

   private final ExecutorService executorService = Executors.newSingleThreadExecutor(ThreadTools.getNamedThreadFactory(getClass().getSimpleName()));

   private final AtomicReference<PlanarRegionsList> planarRegionsReference;
   private final AtomicReference<RobotSide> initialStanceSideReference;
   private final AtomicReference<Pose3DReadOnly> leftFootStartPose;
   private final AtomicReference<Pose3DReadOnly> rightFootStartPose;
   private final AtomicReference<Pose3DReadOnly> leftFootGoalPose;
   private final AtomicReference<Pose3DReadOnly> rightFootGoalPose;
   private final AtomicReference<Boolean> performAStarSearch;
   private final AtomicReference<Boolean> planBodyPath;

   private final AtomicReference<Double> plannerTimeoutReference;
   private final AtomicReference<Integer> plannerMaxIterationsReference;
   private final AtomicReference<Double> plannerHorizonLengthReference;

   private final AtomicReference<Boolean> snapGoalSteps;
   private final AtomicReference<Boolean> abortIfGoalStepSnapFails;

   private final AtomicReference<FootstepPlannerParametersReadOnly> parameters;
   private final AtomicReference<VisibilityGraphsParametersReadOnly> visibilityGraphsParameters;

   private final Messager messager;
   private final FootstepPlanningModule planningModule = new FootstepPlanningModule(getClass().getSimpleName());

   public FootstepPathCalculatorModule(Messager messager)
   {
      this.messager = messager;

      planarRegionsReference = messager.createInput(PlanarRegionData);
      initialStanceSideReference = messager.createInput(InitialSupportSide, RobotSide.LEFT);
      leftFootStartPose = messager.createInput(LeftFootPose);
      rightFootStartPose = messager.createInput(RightFootPose);
      leftFootGoalPose = messager.createInput(LeftFootGoalPose);
      rightFootGoalPose = messager.createInput(RightFootGoalPose);

      parameters = messager.createInput(PlannerParameters, new DefaultFootstepPlannerParameters());
      visibilityGraphsParameters = messager.createInput(VisibilityGraphsParameters, new DefaultVisibilityGraphParameters());
      performAStarSearch = messager.createInput(PerformAStarSearch, false);
      planBodyPath = messager.createInput(PlanBodyPath, true);
      plannerTimeoutReference = messager.createInput(PlannerTimeout, 5.0);
      plannerHorizonLengthReference = messager.createInput(PlannerHorizonLength, 1.0);

      plannerMaxIterationsReference = messager.createInput(MaxIterations, -1);
      snapGoalSteps = messager.createInput(SnapGoalSteps, false);
      abortIfGoalStepSnapFails = messager.createInput(AbortIfGoalStepSnapFails, false);

      messager.registerTopicListener(ComputePath, request -> computePathOnThread());
      new FootPoseFromMidFootUpdater(messager).start();
   }

   public void clear()
   {
      planarRegionsReference.set(null);
      initialStanceSideReference.set(null);
      leftFootStartPose.set(null);
      rightFootStartPose.set(null);
      leftFootGoalPose.set(null);
      rightFootGoalPose.set(null);
      plannerTimeoutReference.set(null);
      plannerMaxIterationsReference.set(null);
      plannerHorizonLengthReference.set(null);
      snapGoalSteps.set(null);
      abortIfGoalStepSnapFails.set(null);
   }

   public void start()
   {
   }

   public void stop()
   {
      executorService.shutdownNow();
   }

   private void computePathOnThread()
   {
      executorService.submit(this::computePath);
   }

   private void computePath()
   {
      if (VERBOSE)
      {
         LogTools.info("Starting to compute path...");
      }

      PlanarRegionsList planarRegionsList = planarRegionsReference.get();

      if (planarRegionsList == null)
         return;

      if (leftFootStartPose.get() == null || rightFootStartPose.get() == null)
         return;

      if (leftFootGoalPose.get() == null || rightFootGoalPose.get() == null)
         return;

      if (VERBOSE)
         LogTools.info("Computing footstep path.");

      try
      {
         FootstepPlannerRequest request = new FootstepPlannerRequest();
         request.setPlanarRegionsList(planarRegionsList);
         request.setTimeout(plannerTimeoutReference.get());
         request.setMaximumIterations(plannerMaxIterationsReference.get());
         request.setHorizonLength(plannerHorizonLengthReference.get());
         request.setRequestedInitialStanceSide(initialStanceSideReference.get());
         request.setStartFootPoses(leftFootStartPose.get(), rightFootStartPose.get());
         request.setGoalFootPoses(leftFootGoalPose.get(), rightFootGoalPose.get());
         request.setPlanBodyPath(planBodyPath.get());
         request.setPerformAStarSearch(performAStarSearch.get());
         request.setSnapGoalSteps(snapGoalSteps.get());
         request.setAbortIfGoalStepSnappingFails(abortIfGoalStepSnapFails.get());

         planningModule.getFootstepPlannerParameters().set(parameters.get());
         planningModule.getVisibilityGraphParameters().set(visibilityGraphsParameters.get());

         messager.submitMessage(PlannerStatus, FootstepPlannerStatus.PLANNING_PATH);

         planningModule.addBodyPathPlanCallback(bodyPathMessage ->
                                                {
                                                   if (FootstepPlanningResult.fromByte(bodyPathMessage.getFootstepPlanningResult()).validForExecution())
                                                   {
                                                      messager.submitMessage(PlannerStatus, FootstepPlannerStatus.PLANNING_STEPS);
                                                      messager.submitMessage(BodyPathData, new ArrayList<>(bodyPathMessage.getBodyPath()));
                                                   }
                                                });
         planningModule.addStatusCallback(status -> messager.submitMessage(PlanningResult, status.getResult()));

         FootstepPlannerOutput output = planningModule.handleRequest(request);

         messager.submitMessage(PlanningResult, output.getResult());
         messager.submitMessage(PlannerStatus, FootstepPlannerStatus.IDLE);

         if (output.getResult().validForExecution())
         {
            messager.submitMessage(FootstepPlanResponse, FootstepDataMessageConverter.createFootstepDataListFromPlan(output.getFootstepPlan(), -1.0, -1.0, ExecutionMode.OVERRIDE));
            if (!output.getLowLevelGoal().containsNaN())
            {
               messager.submitMessage(LowLevelGoalPosition, new Point3D(output.getLowLevelGoal().getPosition()));
               messager.submitMessage(LowLevelGoalOrientation, new Quaternion(output.getLowLevelGoal().getOrientation()));
            }
         }
      }
      catch (Exception e)
      {
         LogTools.error(e.getMessage());
         e.printStackTrace();
      }
   }

   public FootstepPlanningModule getPlanningModule()
   {
      return planningModule;
   }

   public static FootstepPathCalculatorModule createMessagerModule(SharedMemoryMessager messager)
   {
      return new FootstepPathCalculatorModule(messager);
   }
}