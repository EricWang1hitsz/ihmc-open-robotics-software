package us.ihmc.robotics.math.frames;

import static us.ihmc.robotics.math.frames.YoFrameVariableNameTools.createQsName;
import static us.ihmc.robotics.math.frames.YoFrameVariableNameTools.createQxName;
import static us.ihmc.robotics.math.frames.YoFrameVariableNameTools.createQyName;
import static us.ihmc.robotics.math.frames.YoFrameVariableNameTools.createQzName;

import org.apache.commons.lang3.StringUtils;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.axisAngle.interfaces.AxisAngleReadOnly;
import us.ihmc.euclid.interfaces.Clearable;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.ReferenceFrameHolder;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.yoVariables.listener.VariableChangedListener;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

// Note: You should only make these once at the initialization of a controller. You shouldn't make
// any on the fly since they contain YoVariables.
public class YoFrameQuaternion implements ReferenceFrameHolder, Clearable, FrameQuaternionReadOnly
{
   private final String namePrefix;
   private final String nameSuffix;

   private final YoDouble qx, qy, qz, qs;
   private final FrameQuaternion frameOrientation = new FrameQuaternion();
   /**
    * Never use this reference frame directly, use {@link #getReferenceFrame()} instead so the
    * multiple frames version of this {@link YoFrameQuaternion} will work properly.
    */
   private final ReferenceFrame referenceFrame;

   public YoFrameQuaternion(String namePrefix, ReferenceFrame referenceFrame, YoVariableRegistry registry)
   {
      this(namePrefix, "", referenceFrame, registry);
   }

   public YoFrameQuaternion(String namePrefix, String nameSuffix, ReferenceFrame referenceFrame, YoVariableRegistry registry)
   {
      this.namePrefix = namePrefix;
      this.nameSuffix = nameSuffix;

      this.qx = new YoDouble(createQxName(namePrefix, nameSuffix), registry);
      this.qy = new YoDouble(createQyName(namePrefix, nameSuffix), registry);
      this.qz = new YoDouble(createQzName(namePrefix, nameSuffix), registry);
      this.qs = new YoDouble(createQsName(namePrefix, nameSuffix), registry);
      this.referenceFrame = referenceFrame;

      qs.set(1.0);
   }

   public YoFrameQuaternion(YoDouble qx, YoDouble qy, YoDouble qz, YoDouble qs, ReferenceFrame referenceFrame)
   {
      this.namePrefix = StringUtils.getCommonPrefix(qx.getName(), qy.getName(), qz.getName(), qs.getName());
      this.nameSuffix = YoFrameVariableNameTools.getCommonSuffix(qx.getName(), qy.getName(), qz.getName(), qs.getName());

      this.qx = qx;
      this.qy = qy;
      this.qz = qz;
      this.qs = qs;
      this.referenceFrame = referenceFrame;
   }

   public final FrameQuaternion getFrameOrientation()
   {
      putYoValuesIntoFrameOrientation();
      return frameOrientation;
   }

   public void set(QuaternionReadOnly quaternion)
   {
      frameOrientation.set(quaternion);
      getYoValuesFromFrameOrientation();
   }

   public void set(RotationMatrixReadOnly matrix)
   {
      frameOrientation.set(matrix);
      getYoValuesFromFrameOrientation();
   }

   public void set(AxisAngleReadOnly axisAngle)
   {
      frameOrientation.set(axisAngle);
      getYoValuesFromFrameOrientation();
   }

   public void set(double[] yawPitchRoll)
   {
      frameOrientation.setYawPitchRoll(yawPitchRoll);
      getYoValuesFromFrameOrientation();
   }

   public void setYawPitchRoll(double yaw, double pitch, double roll)
   {
      frameOrientation.setYawPitchRoll(yaw, pitch, roll);
      getYoValuesFromFrameOrientation();
   }

   public void set(double qx, double qy, double qz, double qs)
   {
      this.qx.set(qx);
      this.qy.set(qy);
      this.qz.set(qz);
      this.qs.set(qs);
   }

   public void set(FrameQuaternionReadOnly frameOrientation)
   {
      checkReferenceFrameMatch(frameOrientation);
      this.frameOrientation.setIncludingFrame(frameOrientation);
      getYoValuesFromFrameOrientation(true);
   }

   public void setAndMatchFrame(FrameQuaternionReadOnly frameOrientation)
   {
      this.frameOrientation.setIncludingFrame(frameOrientation);
      this.frameOrientation.changeFrame(getReferenceFrame());
      getYoValuesFromFrameOrientation(true);
   }

   /**
    * Sets the orientation of this to the origin of the passed in ReferenceFrame.
    *
    * @param referenceFrame
    */
   public void setFromReferenceFrame(ReferenceFrame referenceFrame)
   {
      frameOrientation.setToZero(referenceFrame);
      frameOrientation.changeFrame(getReferenceFrame());
      getYoValuesFromFrameOrientation(true);
   }

   /**
    * Sets this quaternion to the same orientation described by the given rotation vector
    * {@code rotationVector}.
    * <p>
    * WARNING: a rotation vector is different from a yaw-pitch-roll or Euler angles representation.
    * A rotation vector is equivalent to the axis of an axis-angle that is multiplied by the angle
    * of the same axis-angle.
    * </p>
    *
    * @param rotation vector the rotation vector used to set this {@code YoFrameQuaternion}. Not
    *           modified.
    */
   public void set(Vector3DReadOnly rotationVector)
   {
      frameOrientation.setToZero(getReferenceFrame());
      frameOrientation.set(rotationVector);
      getYoValuesFromFrameOrientation();
   }

   /**
    * Sets this quaternion to the same orientation described by the given rotation vector
    * {@code rotationVector}.
    * <p>
    * WARNING: a rotation vector is different from a yaw-pitch-roll or Euler angles representation.
    * A rotation vector is equivalent to the axis of an axis-angle that is multiplied by the angle
    * of the same axis-angle.
    * </p>
    *
    * @param rotation vector the rotation vector used to set this {@code YoFrameQuaternion}. Not
    *           modified.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in
    *            {@code this.referenceFrame}.
    */
   public void set(FrameVector3DReadOnly rotationVector)
   {
      frameOrientation.setToZero(getReferenceFrame());
      frameOrientation.set(rotationVector);
      getYoValuesFromFrameOrientation();
   }

   public YoDouble getYoQx()
   {
      return qx;
   }

   public YoDouble getYoQy()
   {
      return qy;
   }

   public YoDouble getYoQz()
   {
      return qz;
   }

   public YoDouble getYoQs()
   {
      return qs;
   }

   @Override
   public double getX()
   {
      return qx.getDoubleValue();
   }

   @Override
   public double getY()
   {
      return qy.getDoubleValue();
   }

   @Override
   public double getZ()
   {
      return qz.getDoubleValue();
   }

   @Override
   public double getS()
   {
      return qs.getDoubleValue();
   }

   public void interpolate(FrameQuaternionReadOnly frameOrientation1, FrameQuaternionReadOnly frameOrientation2, double alpha)
   {
      checkReferenceFrameMatch(frameOrientation1);
      checkReferenceFrameMatch(frameOrientation2);

      frameOrientation.interpolate(frameOrientation1, frameOrientation2, alpha);
      frameOrientation.checkIfUnitary();
      getYoValuesFromFrameOrientation();
   }

   public void interpolate(QuaternionReadOnly quaternion1, QuaternionReadOnly quaternion2, double alpha)
   {
      alpha = MathTools.clamp(alpha, 0.0, 1.0);

      frameOrientation.interpolate(quaternion1, quaternion2, alpha);
      frameOrientation.checkIfUnitary();
      getYoValuesFromFrameOrientation();
   }

   /**
    * Multiplies this quaternion by {@code quaternion}.
    * <p>
    * this = this * quaternion
    * </p>
    *
    * @param quaternion the other quaternion to multiply this. Not modified.
    */
   public void multiply(QuaternionReadOnly quaternion)
   {
      putYoValuesIntoFrameOrientation();
      frameOrientation.multiply(quaternion);
      getYoValuesFromFrameOrientation();
   }

   /**
    * Multiplies this quaternion by {@code frameOrientation}.
    * <p>
    * this = this * frameOrientation.quaternion
    * </p>
    *
    * @param frameOrientation the frame orientation to multiply this. Not modified.
    */
   public void multiply(FrameQuaternionReadOnly frameOrientation)
   {
      putYoValuesIntoFrameOrientation();
      this.frameOrientation.multiply(frameOrientation);
      getYoValuesFromFrameOrientation();
   }

   public void conjugate()
   {
      putYoValuesIntoFrameOrientation();
      frameOrientation.conjugate();
      getYoValuesFromFrameOrientation();
   }

   /**
    * Compute the dot product between this quaternion and the other quaternion: this . other = qx *
    * other.qx + qy * other.qy + qz * other.qz + qs * other.qs.
    * 
    * @param other
    * @return
    */
   public double dot(YoFrameQuaternion other)
   {
      putYoValuesIntoFrameOrientation();
      return frameOrientation.dot(other.frameOrientation);
   }

   public void negate()
   {
      qx.set(-qx.getDoubleValue());
      qy.set(-qy.getDoubleValue());
      qz.set(-qz.getDoubleValue());
      qs.set(-qs.getDoubleValue());
   }

   public void checkQuaternionIsUnitMagnitude()
   {
      putYoValuesIntoFrameOrientation();
      frameOrientation.checkIfUnitary();
   }

   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return referenceFrame;
   }

   private void getYoValuesFromFrameOrientation()
   {
      getYoValuesFromFrameOrientation(true);
   }

   private void getYoValuesFromFrameOrientation(boolean notifyListeners)
   {
      qx.set(frameOrientation.getX(), notifyListeners);
      qy.set(frameOrientation.getY(), notifyListeners);
      qz.set(frameOrientation.getZ(), notifyListeners);
      qs.set(frameOrientation.getS(), notifyListeners);
   }

   @Override
   public void setToNaN()
   {
      frameOrientation.setToNaN();
      getYoValuesFromFrameOrientation();
   }

   @Override
   public void setToZero()
   {
      frameOrientation.setToZero(getReferenceFrame());
      getYoValuesFromFrameOrientation();
   }

   @Override
   public boolean containsNaN()
   {
      return qx.isNaN() || qy.isNaN() || qz.isNaN() || qs.isNaN();
   }

   private void putYoValuesIntoFrameOrientation()
   {
      frameOrientation.setIncludingFrame(getReferenceFrame(), qx.getDoubleValue(), qy.getDoubleValue(), qz.getDoubleValue(), qs.getDoubleValue());
   }

   public void attachVariableChangedListener(VariableChangedListener variableChangedListener)
   {
      qx.addVariableChangedListener(variableChangedListener);
      qy.addVariableChangedListener(variableChangedListener);
      qz.addVariableChangedListener(variableChangedListener);
      qs.addVariableChangedListener(variableChangedListener);
   }

   /**
    * toString
    *
    * String representation of a FrameVector (qx, qy, qz, qs)-reference frame name
    *
    * @return String
    */
   @Override
   public String toString()
   {
      return "(" + qx.getDoubleValue() + ", " + qy.getDoubleValue() + ", " + qz.getDoubleValue() + ", " + qs.getDoubleValue() + ")-" + getReferenceFrame();
   }

   public String getNamePrefix()
   {
      return namePrefix;
   }

   public String getNameSuffix()
   {
      return nameSuffix;
   }

   public boolean epsilonEquals(YoFrameQuaternion other, double epsilon)
   {
      putYoValuesIntoFrameOrientation();
      return frameOrientation.epsilonEquals(other.frameOrientation, epsilon);
   }
}
