package us.ihmc.robotics.geometry.shapes;

import static us.ihmc.euclid.tools.TransformationTools.*;

import us.ihmc.commons.Epsilons;
import us.ihmc.euclid.axisAngle.interfaces.AxisAngleBasics;
import us.ihmc.euclid.axisAngle.interfaces.AxisAngleReadOnly;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.interfaces.Transformable;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.transform.QuaternionBasedTransform;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.Vector4DBasics;
import us.ihmc.euclid.tuple4D.interfaces.Vector4DReadOnly;

public abstract class Shape3d<S extends Shape3d<S>> implements GeometryObject<S>
{
   private final RigidBodyTransform shapePose = new RigidBodyTransform();

   public Shape3d()
   {
   }

   @Override
   public boolean containsNaN()
   {
      return shapePose.containsNaN();
   }

   @Override
   public void setToNaN()
   {
      shapePose.setToNaN();
   }

   @Override
   public void setToZero()
   {
      shapePose.setToZero();
   }

   /**
    * Find the distance from the closest point on this shape to the given point. Returns 0.0 if the
    * point is inside.
    *
    * @param point
    * @return distance from the point to this Shape3d.
    */
   public final double distance(Point3DReadOnly point)
   {
      double xLocal = computeTransformedX(shapePose, true, point);
      double yLocal = computeTransformedY(shapePose, true, point);
      double zLocal = computeTransformedZ(shapePose, true, point);

      return evaluateQuery(xLocal, yLocal, zLocal, null, null);
   }

   /**
    * Determine whether the given point is on or inside the surface of this shape.
    *
    * @param pointToCheck
    * @return true if the point is inside or on the surface, false otherwise.
    */
   public final boolean isInsideOrOnSurface(Point3DReadOnly pointToCheck)
   {
      return isInsideOrOnSurface(pointToCheck, Epsilons.ONE_TRILLIONTH);
   }

   /**
    * Determine whether the given point is on or inside the surface of this shape, within a given
    * tolerance or error level. If epsilonToGrowObject is positive, then the object will be checked
    * as being larger. If negative, then the object will be shrunk.
    *
    * @param pointToCheck
    * @param epsilon
    * @return
    */
   public final boolean isInsideOrOnSurface(Point3DReadOnly pointToCheck, double epsilonToGrowObject)
   {
      double xLocal = computeTransformedX(shapePose, true, pointToCheck);
      double yLocal = computeTransformedY(shapePose, true, pointToCheck);
      double zLocal = computeTransformedZ(shapePose, true, pointToCheck);

      return isInsideOrOnSurfaceShapeFrame(xLocal, yLocal, zLocal, epsilonToGrowObject);
   }

   protected abstract boolean isInsideOrOnSurfaceShapeFrame(double x, double y, double z, double epsilon);

   /**
    * Find the closest point on the surface of this shape to the given point. If the given point is
    * on or inside the shape, then it is not changed. If you wish to know the surface normal, then
    * subtract the projected point from the original point.
    *
    * @param pointToCheckAndPack both an input parameter (the point to check), and an output
    *           parameter (packed with the resulting orthogonal point).
    */
   public final void orthogonalProjection(Point3DBasics pointToCheckAndPack)
   {
      double xOriginal = pointToCheckAndPack.getX();
      double yOriginal = pointToCheckAndPack.getY();
      double zOriginal = pointToCheckAndPack.getZ();

      double xLocal = computeTransformedX(shapePose, true, pointToCheckAndPack);
      double yLocal = computeTransformedY(shapePose, true, pointToCheckAndPack);
      double zLocal = computeTransformedZ(shapePose, true, pointToCheckAndPack);

      boolean isInside = evaluateQuery(xLocal, yLocal, zLocal, pointToCheckAndPack, null) <= 0.0;

      if (isInside)
         pointToCheckAndPack.set(xOriginal, yOriginal, zOriginal);
      else
         transformToWorld(pointToCheckAndPack);
   }

   /**
    * Returns true if inside the Shape3d. If inside, must pack the intersection and normal. If not
    * inside, packing those is optional. But if they are not packed when outside, then they should
    * be set to NaN. If they are set to NaN and you really do wish to see where they would project
    * to, then call orthogonalProjection.
    *
    * @param pointToCheck
    * @param intersectionToPack
    * @param normalToPack
    * @return true if the point is inside, false otherwise.
    */
   public final boolean checkIfInside(Point3DReadOnly pointToCheck, Point3DBasics closestPointOnSurfaceToPack, Vector3DBasics normalToPack)
   {
      double xLocal = computeTransformedX(shapePose, true, pointToCheck);
      double yLocal = computeTransformedY(shapePose, true, pointToCheck);
      double zLocal = computeTransformedZ(shapePose, true, pointToCheck);

      boolean isInside = evaluateQuery(xLocal, yLocal, zLocal, closestPointOnSurfaceToPack, normalToPack) <= 0.0;

      if (closestPointOnSurfaceToPack != null)
         transformToWorld(closestPointOnSurfaceToPack);

      if (normalToPack != null)
         transformToWorld(normalToPack);

      return isInside;
   }

   /**
    * Internal generic method used for the public API of any {@code Shape3d}.
    * 
    * @param x the x-coordinate of the query expressed in the local frame of this shape.
    * @param y the y-coordinate of the query expressed in the local frame of this shape.
    * @param z the z-coordinate of the query expressed in the local frame of this shape.
    * @param closestPointOnSurfaceToPack closest point to the query expressed in the local frame of
    *           this shape. Modified. Can be {@code null}.
    * @param normalAtClosestPointToPack normal of the shape surface at the closest point. Modified.
    *           Can be {@code null}.
    * @return the distance from the query to the closest point on the shape surface. The returned
    *         value is expected to be negative when the query is inside the shape.
    */
   protected abstract double evaluateQuery(double x, double y, double z, Point3DBasics closestPointOnSurfaceToPack, Vector3DBasics normalAtClosestPointToPack);

   @Override
   public final void applyTransform(Transform transform)
   {
      transform.transform(shapePose);
   }

   @Override
   public final void applyInverseTransform(Transform transform)
   {
      transform.inverseTransform(shapePose);
   }

   public boolean epsilonEqualsPose(Shape3d<S> other, double epsilon)
   {
      return shapePose.epsilonEquals(other.shapePose, epsilon);
   }

   public void setPose(Shape3d<S> other)
   {
      shapePose.set(other.shapePose);
   }

   // Pose API

   public final void setX(double x)
   {
      shapePose.setTranslationX(x);
   }

   public final void setY(double y)
   {
      shapePose.setTranslationY(y);
   }

   public final void setZ(double z)
   {
      shapePose.setTranslationZ(z);
   }

   public final double getX()
   {
      return shapePose.getTranslationX();
   }

   public final double getY()
   {
      return shapePose.getTranslationY();
   }

   public final double getZ()
   {
      return shapePose.getTranslationZ();
   }

   public final void setPosition(double x, double y, double z)
   {
      shapePose.setTranslation(x, y, z);
   }

   public final void translate(Tuple3DReadOnly translation)
   {
      shapePose.prependTranslation(translation);
   }

   public final void translate(double x, double y, double z)
   {
      shapePose.prependTranslation(x, y, z);
   }

   public final void appendTransform(RigidBodyTransform transform)
   {
      shapePose.multiply(transform);
   }

   public final void appendTransform(QuaternionBasedTransform transform)
   {
      shapePose.multiply(transform);
   }

   public String getPoseString()
   {
      return shapePose.toString();
   }

   public final void setPose(Pose3D pose)
   {
      pose.get(shapePose);
   }

   public final void setPose(RigidBodyTransform transform)
   {
      shapePose.set(transform);
   }

   public final void setPose(Tuple3DReadOnly position, QuaternionReadOnly orientation)
   {
      shapePose.set(orientation, position);
   }

   public final void setPose(Tuple3DReadOnly position, AxisAngleReadOnly orientation)
   {
      shapePose.set(orientation, position);
   }

   public final void setPosition(Tuple3DReadOnly position)
   {
      shapePose.setTranslation(position);
   }

   public final void setXY(Point2DReadOnly point)
   {
      shapePose.setTranslationX(point.getX());
      shapePose.setTranslationY(point.getY());
   }

   public final void getPose(RigidBodyTransform transformToPack)
   {
      transformToPack.set(shapePose);
   }

   public final void getPose(Pose3D poseToPack)
   {
      poseToPack.set(shapePose);
   }

   public final void getPosition(Tuple3DBasics tupleToPack)
   {
      shapePose.getTranslation(tupleToPack);
   }

   public final void setOrientation(QuaternionReadOnly quaternion)
   {
      shapePose.setRotation(quaternion);
   }

   public final void setOrientation(RotationMatrixReadOnly matrix3d)
   {
      shapePose.setRotation(matrix3d);
   }

   public final void setOrientation(AxisAngleReadOnly axisAngle4d)
   {
      shapePose.setRotation(axisAngle4d);
   }

   public final void getOrientation(RotationMatrix matrixToPack)
   {
      shapePose.getRotation(matrixToPack);
   }

   public final void getOrientation(QuaternionBasics quaternionToPack)
   {
      shapePose.getRotation(quaternionToPack);
   }

   public final void getOrientation(AxisAngleBasics axisAngleToPack)
   {
      shapePose.getRotation(axisAngleToPack);
   }

   public final void setYawPitchRoll(double[] yawPitchRoll)
   {
      shapePose.setRotationYawPitchRoll(yawPitchRoll);
   }

   public final void setYawPitchRoll(double yaw, double pitch, double roll)
   {
      shapePose.setRotationYawPitchRoll(yaw, pitch, roll);
   }

   public final void getYawPitchRoll(double[] yawPitchRollToPack)
   {
      shapePose.getRotationYawPitchRoll(yawPitchRollToPack);
   }

   public final double getYaw()
   {
      return shapePose.getRotationMatrix().getYaw();
   }

   public final double getPitch()
   {
      return shapePose.getRotationMatrix().getPitch();
   }

   public final double getRoll()
   {
      return shapePose.getRotationMatrix().getRoll();
   }

   public RotationMatrixReadOnly getOrientation()
   {
      return shapePose.getRotationMatrix();
   }

   public Tuple3DReadOnly getPosition()
   {
      return shapePose.getTranslationVector();
   }

   // Pose transform

   public final void transformToWorld(Transformable transformable)
   {
      transformable.applyTransform(shapePose);
   }

   public final void transformToWorld(Point3DReadOnly pointOriginal, Point3DBasics pointTransformed)
   {
      shapePose.transform(pointOriginal, pointTransformed);
   }

   public final void transformToWorld(Vector3DReadOnly vectorOriginal, Vector3DBasics vectorTransformed)
   {
      shapePose.transform(vectorOriginal, vectorTransformed);
   }

   public final void transformToWorld(QuaternionReadOnly quaternionOriginal, QuaternionBasics quaternionTransformed)
   {
      shapePose.transform(quaternionOriginal, quaternionTransformed);
   }

   public final void transformToWorld(Vector4DReadOnly vectorOriginal, Vector4DBasics vectorTransformed)
   {
      shapePose.transform(vectorOriginal, vectorTransformed);
   }

   public final void transformToWorld(Point2DReadOnly pointOriginal, Point2DBasics pointTransformed, boolean checkIfTransformInXYPlane)
   {
      shapePose.transform(pointOriginal, pointTransformed, checkIfTransformInXYPlane);
   }

   public final void transformToWorld(Vector2DReadOnly vectorOriginal, Vector2DBasics vectorTransformed, boolean checkIfTransformInXYPlane)
   {
      shapePose.transform(vectorOriginal, vectorTransformed, checkIfTransformInXYPlane);
   }

   public final void transformToWorld(Matrix3DReadOnly matrixOriginal, Matrix3D matrixTransformed)
   {
      shapePose.transform(matrixOriginal, matrixTransformed);
   }

   public final void transformToWorld(RotationMatrixReadOnly matrixOriginal, RotationMatrix matrixTransformed)
   {
      shapePose.transform(matrixOriginal, matrixTransformed);
   }

   public final void transformToWorld(RigidBodyTransform original, RigidBodyTransform transformed)
   {
      shapePose.transform(original, transformed);
   }

   public final void transformToWorld(QuaternionBasedTransform original, QuaternionBasedTransform transformed)
   {
      shapePose.transform(original, transformed);
   }

   public final void transformToLocal(Transformable transformable)
   {
      transformable.applyInverseTransform(shapePose);
   }

   public final void transformToLocal(Point3DReadOnly pointOriginal, Point3DBasics pointTransformed)
   {
      shapePose.inverseTransform(pointOriginal, pointTransformed);
   }

   public final void transformToLocal(Vector3DReadOnly vectorOriginal, Vector3DBasics vectorTransformed)
   {
      shapePose.inverseTransform(vectorOriginal, vectorTransformed);
   }

   public final void transformToLocal(QuaternionReadOnly quaternionOriginal, QuaternionBasics quaternionTransformed)
   {
      shapePose.inverseTransform(quaternionOriginal, quaternionTransformed);
   }

   public final void transformToLocal(Vector4DReadOnly vectorOriginal, Vector4DBasics vectorTransformed)
   {
      shapePose.inverseTransform(vectorOriginal, vectorTransformed);
   }

   public final void transformToLocal(Point2DReadOnly pointOriginal, Point2DBasics pointTransformed, boolean checkIfTransformInXYPlane)
   {
      shapePose.inverseTransform(pointOriginal, pointTransformed, checkIfTransformInXYPlane);
   }

   public final void transformToLocal(Vector2DReadOnly vectorOriginal, Vector2DBasics vectorTransformed, boolean checkIfTransformInXYPlane)
   {
      shapePose.inverseTransform(vectorOriginal, vectorTransformed, checkIfTransformInXYPlane);
   }

   public final void transformToLocal(Matrix3DReadOnly matrixOriginal, Matrix3D matrixTransformed)
   {
      shapePose.inverseTransform(matrixOriginal, matrixTransformed);
   }

   public final void transformToLocal(RotationMatrixReadOnly matrixOriginal, RotationMatrix matrixTransformed)
   {
      shapePose.inverseTransform(matrixOriginal, matrixTransformed);
   }

   public final void transformToLocal(RigidBodyTransform original, RigidBodyTransform transformed)
   {
      shapePose.inverseTransform(original, transformed);
   }

   public final void transformToLocal(QuaternionBasedTransform original, QuaternionBasedTransform transformed)
   {
      shapePose.inverseTransform(original, transformed);
   }
}
