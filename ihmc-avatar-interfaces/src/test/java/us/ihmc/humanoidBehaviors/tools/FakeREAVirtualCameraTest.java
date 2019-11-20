package us.ihmc.humanoidBehaviors.tools;

import javafx.application.Platform;
import javafx.stage.Stage;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.humanoidBehaviors.ui.graphics.CameraViewportGraphic;
import us.ihmc.humanoidBehaviors.ui.simulation.BehaviorPlanarRegionEnvironments;
import us.ihmc.javaFXToolkit.cameraControllers.FocusBasedCameraMouseEventHandler;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;
import us.ihmc.javafx.applicationCreator.JavaFXApplicationCreator;
import us.ihmc.log.LogTools;
import us.ihmc.pathPlanning.PlannerTestEnvironments;
import us.ihmc.pathPlanning.visibilityGraphs.ui.graphics.PlanarRegionsGraphic;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.tools.gui.AWTTools;

import java.awt.*;

// TODO: Measure speed
public class FakeREAVirtualCameraTest
{
   public static final double VERTICAL_FOV = 90.0;
   public static final double HORIZONTAL_FOV = 90.0;
   private static boolean VISUALIZE = Boolean.parseBoolean(System.getProperty("visualize")); // To visualize, pass -Dvisualize=true

   @BeforeAll
   static public void beforeAll()
   {
      if (VISUALIZE) JavaFXApplicationCreator.createAJavaFXApplication();
   }

   @AfterEach
   public void afterEach()
   {
      if (VISUALIZE) ThreadTools.sleepForever();
   }

   @Test
   public void testFakeREACutsComplexField()
   {
      Pose3D pose = new Pose3D();
      pose.setZ(1.8);
      compareMapToSimulatedView(PlannerTestEnvironments.getTrickCorridor(), pose, 1, 2);
   }

   @Test
   public void testFakeREA1()
   {
      Pose3D pose = new Pose3D();
      pose.setZ(1.8);
      compareMapToSimulatedView(PlannerTestEnvironments.getTrickCorridorWCutFloor(), pose, 3, 4);
   }

   @Test
   public void testFakeREA2()
   {
      compareMapToSimulatedView(BehaviorPlanarRegionEnvironments.realDataFromAtlasSLAMDataset20190710(), new Pose3D(), 5, 6);
      Pose3D pose = new Pose3D();
      pose.setZ(1.8);
      pose.appendYawRotation(Math.toRadians(-135.0));
      compareMapToSimulatedView(BehaviorPlanarRegionEnvironments.realDataFromAtlasSLAMDataset20190710(), pose, 7, 8);
      pose = new Pose3D();
      pose.setZ(1.8);
      pose.appendYawRotation(Math.toRadians(180.0));
      compareMapToSimulatedView(BehaviorPlanarRegionEnvironments.realDataFromAtlasSLAMDataset20190710(), pose, 9, 10);
   }

   private void compareMapToSimulatedView(PlanarRegionsList map, Pose3DReadOnly pose3D, int mapWindowNumber, int resultWindowNumber)
   {
      PlanarRegionsList trickCorridor = map;
      PoseReferenceFrame cameraFrame = new PoseReferenceFrame("camera", ReferenceFrame.getWorldFrame());
      cameraFrame.setPoseAndUpdate(pose3D);
      FakeREAVirtualCamera fakeREAVirtualCamera = new FakeREAVirtualCamera(VERTICAL_FOV, HORIZONTAL_FOV, cameraFrame);
      Stopwatch stopwatch = new Stopwatch().start();
      PlanarRegionsList virtualCamera = fakeREAVirtualCamera.filterMapToVisible(trickCorridor);
      LogTools.info("Time taken: {}", stopwatch.lapElapsed());
      createAndShowPlanarRegionWindow(trickCorridor, cameraFrame, mapWindowNumber - 1);
      createAndShowPlanarRegionWindow(virtualCamera, cameraFrame, resultWindowNumber - 1);
   }

   private void createAndShowPlanarRegionWindow(PlanarRegionsList planarRegionsList, PoseReferenceFrame cameraFrame, int windowCount)
   {
      if (!VISUALIZE) return;

      Platform.runLater(() ->
      {
         int windowWidth = 400;
         int windowHeight = 300;
         View3DFactory view3dFactory = new View3DFactory(windowWidth, windowHeight);
         FocusBasedCameraMouseEventHandler camera = view3dFactory.addCameraController(0.05, 2000.0, true);
         double isoZoomOut = 15.0;
         camera.changeCameraPosition(-isoZoomOut, -isoZoomOut, isoZoomOut);
         view3dFactory.addWorldCoordinateSystem(0.3);
         view3dFactory.addDefaultLighting();

         PlanarRegionsGraphic regionsGraphic = new PlanarRegionsGraphic();
         regionsGraphic.generateMeshes(planarRegionsList);
         regionsGraphic.update();

         CameraViewportGraphic cameraViewportGraphic = new CameraViewportGraphic(cameraFrame, VERTICAL_FOV, HORIZONTAL_FOV);
         cameraViewportGraphic.update();

         view3dFactory.addNodeToView(regionsGraphic);
         view3dFactory.addNodeToView(cameraViewportGraphic);

         Stage primaryStage = new Stage();
         primaryStage.setTitle(getClass().getSimpleName());
         primaryStage.setMaximized(false);
         primaryStage.setScene(view3dFactory.getScene());

         Dimension dimensionForSmallestScreen = AWTTools.getDimensionOfSmallestScreen();
         int numberOfColumns = (int) Math.floor(dimensionForSmallestScreen.getWidth() / windowWidth);
         primaryStage.setX((windowCount % numberOfColumns) * windowWidth);
         primaryStage.setY((windowCount / numberOfColumns) * windowHeight);
         primaryStage.show();
         primaryStage.toFront();
      });
   }
}
