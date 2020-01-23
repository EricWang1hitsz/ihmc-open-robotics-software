package us.ihmc.humanoidBehaviors.ui.mapping.ihmcSlam;

import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.humanoidBehaviors.ui.mapping.IhmcSLAMTools;

public class IhmcSLAMFrame
{
   protected final IhmcSLAMFrame previousFrame;

   // from message.
   private final RigidBodyTransformReadOnly originalSensorPoseToWorld;

   // fixedDiff(parent.originalSensorPoseToWorld vs this.originalSensorPoseToWorld).
   private final RigidBodyTransformReadOnly transformFromPreviousFrame;

   // parent.optimizedSensorPoseToWorld * transformFromPreviousFrame.
   protected final RigidBodyTransformReadOnly sensorPoseToWorld;

   // SLAM result.
   private final RigidBodyTransform slamTransformer = new RigidBodyTransform();

   // this.sensorPoseToWorld * this.slamTransformer.
   protected final RigidBodyTransform optimizedSensorPoseToWorld = new RigidBodyTransform();

   protected final Point3DReadOnly[] originalPointCloudToWorld; // For comparison after mapping.
   protected final Point3DReadOnly[] pointCloudToSensorFrame;
   protected final Point3D[] optimizedPointCloudToWorld;

   public IhmcSLAMFrame(StereoVisionPointCloudMessage message)
   {
      previousFrame = null;

      originalSensorPoseToWorld = IhmcSLAMTools.extractSensorPoseFromMessage(message);

      transformFromPreviousFrame = new RigidBodyTransform(originalSensorPoseToWorld);
      sensorPoseToWorld = new RigidBodyTransform(originalSensorPoseToWorld);
      optimizedSensorPoseToWorld.set(originalSensorPoseToWorld);

      originalPointCloudToWorld = IhmcSLAMTools.extractPointsFromMessage(message);
      pointCloudToSensorFrame = IhmcSLAMTools.createConvertedPointsToSensorPose(originalSensorPoseToWorld, originalPointCloudToWorld);
      optimizedPointCloudToWorld = new Point3D[pointCloudToSensorFrame.length];
      for (int i = 0; i < optimizedPointCloudToWorld.length; i++)
         optimizedPointCloudToWorld[i] = new Point3D(pointCloudToSensorFrame[i]);

      updateOptimizedPointCloudAndSensorPose();
   }

   public IhmcSLAMFrame(IhmcSLAMFrame frame, StereoVisionPointCloudMessage message)
   {
      previousFrame = frame;

      originalSensorPoseToWorld = IhmcSLAMTools.extractSensorPoseFromMessage(message);

      RigidBodyTransform transformDiff = new RigidBodyTransform(originalSensorPoseToWorld);
      transformDiff.preMultiplyInvertOther(frame.originalSensorPoseToWorld);
      transformFromPreviousFrame = new RigidBodyTransform(transformDiff);

      RigidBodyTransform transformToWorld = new RigidBodyTransform(frame.optimizedSensorPoseToWorld);
      transformToWorld.multiply(transformFromPreviousFrame);
      sensorPoseToWorld = new RigidBodyTransform(transformToWorld);

      originalPointCloudToWorld = IhmcSLAMTools.extractPointsFromMessage(message);
      pointCloudToSensorFrame = IhmcSLAMTools.createConvertedPointsToSensorPose(originalSensorPoseToWorld, originalPointCloudToWorld);
      optimizedPointCloudToWorld = new Point3D[pointCloudToSensorFrame.length];
      for (int i = 0; i < optimizedPointCloudToWorld.length; i++)
         optimizedPointCloudToWorld[i] = new Point3D(pointCloudToSensorFrame[i]);

      updateOptimizedPointCloudAndSensorPose();
   }
   
   public void updateOptimizedCorrection(RigidBodyTransformReadOnly driftCorrectionTransform)
   {
      slamTransformer.set(driftCorrectionTransform);
      updateOptimizedPointCloudAndSensorPose();
   }

   private void updateOptimizedPointCloudAndSensorPose()
   {
      optimizedSensorPoseToWorld.set(sensorPoseToWorld);
      optimizedSensorPoseToWorld.multiply(slamTransformer);

      for (int i = 0; i < optimizedPointCloudToWorld.length; i++)
      {
         optimizedPointCloudToWorld[i].set(pointCloudToSensorFrame[i]);
         optimizedSensorPoseToWorld.transform(optimizedPointCloudToWorld[i]);
      }
   }

   public Point3DReadOnly[] getOriginalPointCloud()
   {
      return originalPointCloudToWorld;
   }

   public Point3DReadOnly[] getOriginalPointCloudToSensorPose()
   {
      return pointCloudToSensorFrame;
   }

   public RigidBodyTransformReadOnly getOriginalSensorPose()
   {
      return originalSensorPoseToWorld;
   }

   public RigidBodyTransformReadOnly getInitialSensorPoseToWorld()
   {
      return sensorPoseToWorld;
   }

   public Point3DReadOnly[] getPointCloud()
   {
      return optimizedPointCloudToWorld;
   }

   public RigidBodyTransformReadOnly getSensorPose()
   {
      return optimizedSensorPoseToWorld;
   }

   public boolean isFirstFrame()
   {
      if (previousFrame == null)
         return true;
      else
         return false;
   }
   
   public IhmcSLAMFrame getPreviousFrame()
   {
      return previousFrame;
   }
}
