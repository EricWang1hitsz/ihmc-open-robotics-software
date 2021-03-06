package us.ihmc.humanoidBehaviors.patrol;

import static us.ihmc.humanoidBehaviors.patrol.PatrolBehavior.PatrolBehaviorState.*;
import static us.ihmc.humanoidBehaviors.patrol.PatrolBehaviorAPI.*;

import java.util.concurrent.atomic.AtomicReference;

import com.google.common.collect.Lists;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import controller_msgs.msg.dds.PlanarRegionsListMessage;
import controller_msgs.msg.dds.WalkingStatusMessage;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.thread.Notification;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.footstepPlanning.FootstepDataMessageConverter;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParameters;
import us.ihmc.humanoidBehaviors.BehaviorInterface;
import us.ihmc.communication.RemoteREAInterface;
import us.ihmc.humanoidBehaviors.BehaviorDefinition;
import us.ihmc.humanoidBehaviors.tools.BehaviorHelper;
import us.ihmc.humanoidBehaviors.tools.RemoteHumanoidRobotInterface;
import us.ihmc.humanoidBehaviors.tools.footstepPlanner.RemoteFootstepPlannerInterface;
import us.ihmc.humanoidBehaviors.tools.footstepPlanner.RemoteFootstepPlannerResult;
import us.ihmc.humanoidBehaviors.upDownExploration.UpDownExplorer;
import us.ihmc.humanoidBehaviors.waypoints.WaypointManager;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.robotics.stateMachine.core.StateMachine;
import us.ihmc.robotics.stateMachine.extra.EnumBasedStateMachineFactory;
import us.ihmc.tools.UnitConversions;
import us.ihmc.tools.thread.PausablePeriodicThread;
import us.ihmc.tools.thread.TypedNotification;

/**
 * Walk through a list of waypoints in order, looping forever.
 */
public class PatrolBehavior implements BehaviorInterface
{
   public static final BehaviorDefinition DEFINITION = new BehaviorDefinition("Patrol", PatrolBehavior::new, PatrolBehaviorAPI.create());

   public enum PatrolBehaviorState
   {
      /** Stop state that waits for or is triggered by a GoToWaypoint message */
      STOP,
      /** Decide on waypoints. */
      NAVIGATE,
      /** Request and wait for footstep planner result */
      PLAN,
      /** Optional user review of footstep plan. */
      REVIEW,
      /** Walking towards goal waypoint */
      WALK,
      /** Wait to gather perception data */
      PERCEIVE
   }

   public enum OperatorPlanReviewResult
   {
      REPLAN, WALK
   }

   private final BehaviorHelper helper;
   private final RemoteHumanoidRobotInterface robot;
   private final RemoteFootstepPlannerInterface footstepPlannerToolbox;
   private final RemoteREAInterface rea;

   private final StateMachine<PatrolBehaviorState, State> stateMachine;
   private final PausablePeriodicThread mainThread;

   private final UpDownExplorer upDownExplorer;

   private final Notification stopNotification = new Notification();
   private final Notification goNotification = new Notification();
   private final TypedNotification<OperatorPlanReviewResult> planReviewResult = new TypedNotification<>();
   private final Notification cancelPlanning = new Notification();
   private final Notification skipPerceive = new Notification();

   private TypedNotification<RemoteFootstepPlannerResult> footstepPlanResultNotification;
   private TypedNotification<WalkingStatusMessage> walkingCompleted;

   private final WaypointManager waypointManager;

   private final AtomicReference<Boolean> loop;
   private final AtomicReference<Boolean> swingOvers;
   private final AtomicReference<Boolean> planReviewEnabled;
   private final AtomicReference<Boolean> upDownExplorationEnabled;
   private final AtomicReference<Double> perceiveDuration;

   public PatrolBehavior(BehaviorHelper helper)
   {
      this.helper = helper;
      robot = helper.getOrCreateRobotInterface();
      footstepPlannerToolbox = helper.getOrCreateFootstepPlannerToolboxInterface();
      rea = helper.getOrCreateREAInterface();

      LogTools.debug("Initializing patrol behavior");

      EnumBasedStateMachineFactory<PatrolBehaviorState> factory = new EnumBasedStateMachineFactory<>(PatrolBehaviorState.class);
      factory.setOnEntry(STOP, this::onStopStateEntry);
      factory.setDoAction(STOP, this::doStopStateAction);
      factory.addTransition(STOP, NAVIGATE, this::transitionFromStop);
      factory.setOnEntry(NAVIGATE, this::onNavigateStateEntry);
      factory.setDoAction(NAVIGATE, this::doNavigateStateAction);
      factory.addTransition(NAVIGATE, Lists.newArrayList(PLAN, NAVIGATE, STOP), this::transitionFromNavigate);
      factory.setOnEntry(PLAN, this::onPlanStateEntry);
      factory.setDoAction(PLAN, this::doPlanStateAction);
      factory.addTransition(PLAN, Lists.newArrayList(REVIEW, NAVIGATE, STOP), this::transitionFromPlan);
      factory.setOnEntry(REVIEW, this::onReviewStateEntry);
      factory.setDoAction(REVIEW, this::onReviewStateAction);
      factory.addTransition(REVIEW, Lists.newArrayList(WALK, PLAN, STOP), this::transitionFromReview);
      factory.setOnEntry(WALK, this::onWalkStateEntry);
      factory.setDoAction(WALK, this::doWalkStateAction);
      factory.setOnExit(WALK, this::onWalkStateExit);
      factory.addTransition(WALK, Lists.newArrayList(PERCEIVE, NAVIGATE, STOP), this::transitionFromWalk);
      factory.setOnEntry(PERCEIVE, this::onPerceiveStateEntry);
      factory.setDoAction(PERCEIVE, this::doPerceiveStateAction);
      factory.addTransition(PERCEIVE, Lists.newArrayList(NAVIGATE, STOP), this::transitionFromPerceive);

      factory.getFactory().addStateChangedListener((from, to) ->
      {
         helper.publishToUI(CurrentState, to);
         LogTools.debug("{} -> {}", from == null ? null : from.name(), to == null ? null : to.name());
      });
      factory.getFactory().buildClock(() -> Conversions.nanosecondsToSeconds(System.nanoTime()));
      stateMachine = factory.getFactory().build(STOP);

      waypointManager = WaypointManager.createForModule(helper.getManagedMessager(),
                                                        WaypointsToModule,
                                                        WaypointsToUI,
                                                        GoToWaypoint,
                                                        CurrentWaypointIndexStatus,
                                                        goNotification);

      // TODO: Use helper
      helper.createUICallback(Stop, object -> stopNotification.set());
      helper.createUICallback(PlanReviewResult, message ->
      {
         planReviewResult.add(message);
      });
      helper.createUICallback(SkipPerceive, object -> skipPerceive.set());

      // TODO: Use helper
      loop = helper.createUIInput(Loop, false);
      swingOvers = helper.createUIInput(SwingOvers, false);
      planReviewEnabled = helper.createUIInput(PlanReviewEnabled, false);
      upDownExplorationEnabled = helper.createUIInput(UpDownExplorationEnabled, false);
      perceiveDuration = helper.createUIInput(PerceiveDuration, RemoteFootstepPlannerInterface.DEFAULT_PERCEIVE_TIME_REQUIRED);
      helper.createUICallback(UpDownExplorationEnabled, enabled -> { if (enabled) goNotification.set(); });

      upDownExplorer = new UpDownExplorer(helper, rea);
      helper.createUICallback(CancelPlanning, object ->
      {
         cancelPlanning.set();
         upDownExplorer.abortPlanning();
      });

      mainThread = helper.createPausablePeriodicThread(getClass(), UnitConversions.hertzToSeconds(250), 5, this::patrolThread);
   }

   private void patrolThread()   // update the active state
   {
      stateMachine.doActionAndTransition();
   }

   @Override
   public void setEnabled(boolean enabled)
   {
      LogTools.info("Patrol behavior selected = {}", enabled);

      mainThread.setRunning(enabled);
      helper.setCommunicationCallbacksEnabled(enabled);
   }

   private void onStopStateEntry()
   {
      robot.pauseWalking();
   }

   private void doStopStateAction(double timeInState)
   {
      pollInterrupts();
   }

   private boolean transitionFromStop(double timeInState)
   {
      boolean transition = goNotification.read() && (waypointManager.hasWaypoints() || upDownExplorationEnabled.get());
      if (transition)
      {
         LogTools.debug("STOP -> PLAN");
      }
      return transition;
   }

   private void onNavigateStateEntry()
   {
      if (upDownExplorationEnabled.get()) // find up-down if. setup the waypoint
      {
         upDownExplorer.onNavigateEntry(robot.pollHumanoidRobotState());
      }
   }

   private void doNavigateStateAction(double timeInState)
   {
      pollInterrupts();
      upDownExplorer.poll();
   }

   private PatrolBehaviorState transitionFromNavigate(double timeInState)
   {
      if (stopNotification.read())
      {
         return STOP;
      }
      else if (cancelPlanning.read())
      {
         return NAVIGATE;
      }
      else if (!upDownExplorationEnabled.get())
      {
         return PLAN;
      }
      else if (upDownExplorer.shouldTransitionToPlan())
      {
         return PLAN;
      }

      return null;
   }

   private void onPlanStateEntry()
   {
      // update waypoints if UI modified them
      waypointManager.updateToMostRecentData();

      footstepPlannerToolbox.abortPlanning();

      FramePose3DReadOnly midFeetZUpPose = robot.quickPollPoseReadOnly(HumanoidReferenceFrames::getMidFeetZUpFrame);

      if (upDownExplorationEnabled.get()) // TODO need this?? && upDownExplorer.getUpDownSearchNotification().hasNext())
      {
         upDownExplorer.onPlanEntry(midFeetZUpPose, waypointManager);
      }

      PlanarRegionsListMessage latestPlanarRegionList = rea.getLatestPlanarRegionsListMessage();

      FramePose3DReadOnly start = midFeetZUpPose;
      FramePose3D goal = new FramePose3D(waypointManager.peekNextPose());
      DefaultFootstepPlannerParameters defaultFootstepPlannerParameters = new DefaultFootstepPlannerParameters();
      if (decidePlanType(start, goal) == PlanTravelDistance.CLOSE)
      {
         defaultFootstepPlannerParameters.setMaximumStepYaw(1.1); // enable quick turn arounds
      }
      footstepPlanResultNotification = footstepPlannerToolbox.requestPlan(start, goal, latestPlanarRegionList, defaultFootstepPlannerParameters);
   }

   private static PlanTravelDistance decidePlanType(Pose3DReadOnly start, Pose3DReadOnly goal)
   {
      return start.getPosition().distance(goal.getPosition()) < PlanTravelDistance.CLOSE_PLAN_RADIUS ? PlanTravelDistance.CLOSE : PlanTravelDistance.FAR;
   }

   private void doPlanStateAction(double timeInState)
   {
      pollInterrupts();
      footstepPlanResultNotification.poll();

      if (footstepPlanResultNotification.hasNext())
      {
         upDownExplorer.onPlanFinished(footstepPlanResultNotification.peek());
      }
   }

   private PatrolBehaviorState transitionFromPlan(double timeInState)
   {
      if (stopNotification.read())
      {
         return STOP;
      }
      else if (cancelPlanning.read())
      {
         return NAVIGATE;
      }
      else if (footstepPlanResultNotification.hasNext())
      {
         if (footstepPlanResultNotification.peek().isValidForExecution())
         {
            return REVIEW;
         }
         else
         {
            return NAVIGATE;
         }
      }

      return null;
   }

   private void onReviewStateEntry()
   {
      helper.publishToUI(CurrentFootstepPlan,
                         FootstepDataMessageConverter.reduceFootstepPlanForUIMessager(footstepPlanResultNotification.peek().getFootstepPlan()));
   }

   private void onReviewStateAction(double timeInState)
   {
      pollInterrupts();
   }

   private PatrolBehaviorState transitionFromReview(double timeInState)
   {
      if (stopNotification.read())
      {
         return STOP;
      }
      else if (!planReviewEnabled.get()) // if review is disabled, proceed
      {
         return WALK;
      }
      else if (planReviewResult.hasNext())
      {
         if (planReviewResult.peek() == OperatorPlanReviewResult.REPLAN)
         {
            return PLAN;
         }
         else if (planReviewResult.peek() == OperatorPlanReviewResult.WALK)
         {
            return WALK;
         }
      }

      return null;
   }

   private void onWalkStateEntry()
   {
      PlanarRegionsList planarRegionsList = footstepPlanResultNotification.peek().getPlanarRegionsList();
      FootstepDataListMessage footstepDataListMessage = footstepPlanResultNotification.peek().getFootstepDataListMessage();
      Boolean swingOverPlanarRegions = swingOvers.get();

      HumanoidReferenceFrames humanoidReferenceFrames = robot.pollHumanoidRobotState();

//      swingOverPlanarRegions &= decidePlanDistance(footstepDataListMessage, humanoidReferenceFrames) == PlanTravelDistance.FAR;

      walkingCompleted = robot.requestWalk(footstepDataListMessage, humanoidReferenceFrames, planarRegionsList);

      helper.publishToUI(CurrentFootstepPlan, FootstepDataMessageConverter.reduceFootstepPlanForUIMessager(footstepDataListMessage));
   }

   private PlanTravelDistance decidePlanDistance(FootstepDataListMessage footstepPlan, HumanoidReferenceFrames humanoidReferenceFrames)
   {
      FramePose3D midFeetZUpPose = new FramePose3D();
      midFeetZUpPose.setFromReferenceFrame(humanoidReferenceFrames.getMidFeetZUpFrame());

      FootstepDataMessage footstep = footstepPlan.getFootstepDataList().get(footstepPlan.getFootstepDataList().size() - 1);

      double distance = midFeetZUpPose.getPosition().distance(footstep.getLocation());

      return distance < PlanTravelDistance.CLOSE_PLAN_RADIUS ? PlanTravelDistance.CLOSE : PlanTravelDistance.FAR;
   }

   private void doWalkStateAction(double timeInState)
   {
      pollInterrupts();
      walkingCompleted.poll();
      waypointManager.updateToMostRecentData();
   }

   private PatrolBehaviorState transitionFromWalk(double timeInState)
   {
      if (stopNotification.read())
      {
         return STOP;
      }
      else if (walkingCompleted.hasNext()) // TODO handle robot fell and more
      {
         if (!upDownExplorationEnabled.get() && !loop.get() && waypointManager.incrementingWillLoop()) // stop in the acute case of not exploring and
         {                                                                                             // looping disabled and it's going to loop
            return STOP;
         }
         else
         {
            // next waypoint is far, gather more data to increase robustness
            FramePose3DReadOnly midFeetZUpPose = robot.quickPollPoseReadOnly(HumanoidReferenceFrames::getMidFeetZUpFrame);

            PlanTravelDistance planType = decidePlanType(midFeetZUpPose, waypointManager.peekAfterNextPose());

         // perceive everytime when updownenabled
            if (upDownExplorationEnabled.get() || (planType == PlanTravelDistance.FAR))
            {
               return PERCEIVE;
            }
            else // next waypoint is close
            {
               return NAVIGATE;
            }
         }
      }

      return null;
   }

   private void onWalkStateExit()
   {
      if (!stopNotification.read()) // only increment in normal operation (WALK -> PLAN or end with loop disabled)
      {
         waypointManager.increment();
      }
   }

   private void onPerceiveStateEntry()
   {
      rea.clearREA();
   }

   private void doPerceiveStateAction(double timeInState)
   {
      pollInterrupts();
      upDownExplorer.setMidFeetZUpPose(robot.quickPollPoseReadOnly(HumanoidReferenceFrames::getMidFeetZUpFrame));
   }

   private PatrolBehaviorState transitionFromPerceive(double timeInState)
   {
      if (stopNotification.read())
      {
         return STOP;
      }
      else if (skipPerceive.read())
      {
         return NAVIGATE;
      }
      else if (upDownExplorationEnabled.get() && upDownExplorer.shouldTransitionFromPerceive())
      {
         return NAVIGATE;
      }
      else if (timeInState > perceiveDuration.get())
      {
         return NAVIGATE;
      }

      return null;
   }

   private void pollInterrupts()
   {
      HighLevelControllerName controllerState = robot.getLatestControllerState();
      boolean isWalking = robot.isRobotWalking();

      if (!stateMachine.getCurrentStateKey().equals(STOP) && !isWalking) // STOP if robot falls
      {
         LogTools.debug("Stopping from robot state: {}", controllerState.name());
         stopNotification.set();
      }

      stopNotification.poll();         // poll both at the same time to handle race condition

      // poll these everytime to throw away ill-timed inputs
      goNotification.poll();
      planReviewResult.poll();
      cancelPlanning.poll();
      skipPerceive.poll();
   }
}
