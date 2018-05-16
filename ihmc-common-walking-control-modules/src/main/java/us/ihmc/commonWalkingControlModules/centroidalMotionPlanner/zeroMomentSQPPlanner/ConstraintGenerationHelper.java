package us.ihmc.commonWalkingControlModules.centroidalMotionPlanner.zeroMomentSQPPlanner;

import java.util.List;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;

/**
 * 
 * @author Apoorv S
 *
 */
public class ConstraintGenerationHelper
{
   private final DenseMatrix64F xPowers = new DenseMatrix64F(0, 1);

   public ConstraintGenerationHelper()
   {

   }

   private void computePowers(DenseMatrix64F powersMatrix, double x, int polynomialOrder)
   {
      powersMatrix.reshape(polynomialOrder + 1, 1);
      powersMatrix.zero();
      powersMatrix.set(0, 0, 1.0);
      for (int i = 1; i <= polynomialOrder; i++)
         powersMatrix.set(i, 0, powersMatrix.get(i - 1, 0) * x);
   }

   public void generateCoefficients(DenseMatrix64F constraintCoefficentToSet, int polynomialOrder, int derivatiesOrder, double x)
   {
      constraintCoefficentToSet.reshape(derivatiesOrder + 1, polynomialOrder + 1);
      constraintCoefficentToSet.zero();
      computePowers(xPowers, x, polynomialOrder);
      for (int i = 0; i <= polynomialOrder; i++)
      {
         double fact = 1.0;
         for (int j = 0; j <= derivatiesOrder && j <= i; j++)
         {
            constraintCoefficentToSet.set(j, i, fact * xPowers.get(i - j, 0));
            fact *= (i - j);
         }
      }
   }

   /**
    * 
    * @param constraintCoefficentsToSet modified.
    * @param constraintBiasToSet modified
    * @param polynomialCoefficient column vector of polynomial coefficients
    * @param polynomialOrder order of the polynomial 
    * @param derivatiesOrder order of the derivatives to be determined
    */
   public void generateDerivativeCoefficientsAndBiasMatrix(DenseMatrix64F constraintCoefficentsToSet, DenseMatrix64F constraintBiasToSet,
                                                           DenseMatrix64F polynomialCoefficient, int polynomialOrder, int derivatiesOrder, double x)
   {
      generateCoefficients(constraintCoefficentsToSet, polynomialOrder, derivatiesOrder, x);
      constraintBiasToSet.reshape(derivatiesOrder + 1, 1);
      CommonOps.mult(constraintCoefficentsToSet, polynomialCoefficient, constraintBiasToSet);
   }

   public void generateLocationConstraintMatrix()
   {

   }

   public void generateScalarConstraintMatrix(DenseMatrix64F coefficientMatrixToSet, DenseMatrix64F biasMatrixToSet, DenseMatrix64F scalarCoefficientMatrix,
                                              int polynomialOrder, List<Double> nodeTimes)
   {
      coefficientMatrixToSet.reshape(nodeTimes.size(), polynomialOrder + 1);
      biasMatrixToSet.reshape(nodeTimes.size(), 1);

      for (int i = 0; i < nodeTimes.size(); i++)
      {
         generateDerivativeCoefficientsAndBiasMatrix(tempCoeffMatrixForScalar, tempBiasMatrixForScalar, scalarCoefficientMatrix, polynomialOrder, 0,
                                                     nodeTimes.get(i));
         CommonOps.insert(tempCoeffMatrixForScalar, coefficientMatrixToSet, i, 0);
         CommonOps.insert(tempBiasMatrixForScalar, biasMatrixToSet, i, 0);
      }
   }

   private final DenseMatrix64F tempCoeffMatrix2 = new DenseMatrix64F(0, 1);
   private final DenseMatrix64F tempCoeffMatrix1 = new DenseMatrix64F(0, 1);
   private final DenseMatrix64F tempCoeffMatrixForScalar = new DenseMatrix64F(0, 1);
   private final DenseMatrix64F tempBiasMatrix2 = new DenseMatrix64F(0, 1);
   private final DenseMatrix64F tempBiasMatrix1 = new DenseMatrix64F(0, 1);
   private final DenseMatrix64F tempBiasMatrixForScalar = new DenseMatrix64F(0, 1);

   public void generateDynamicsCollocationConstraints(DenseMatrix64F comCoefficientMatrixToSet, DenseMatrix64F copCoefficientMatrixToSet,
                                                      DenseMatrix64F scalarCoefficientMatrixToSet, DenseMatrix64F biasMatrixToSet,
                                                      List<Double> timeNodesForConstraints, DenseMatrix64F comCoefficients, DenseMatrix64F copCoefficients,
                                                      DenseMatrix64F scalarCoefficients, int comPolynomialOrder, int copPolynomialOrder,
                                                      int scalarPolynomialOrder, double gravity)
   {
      comCoefficientMatrixToSet.reshape(timeNodesForConstraints.size(), comPolynomialOrder + 1);
      copCoefficientMatrixToSet.reshape(timeNodesForConstraints.size(), copPolynomialOrder + 1);
      scalarCoefficientMatrixToSet.reshape(timeNodesForConstraints.size(), scalarPolynomialOrder + 1);

      for (int i = 0; i < timeNodesForConstraints.size(); i++)
      {
         double nodeTime = timeNodesForConstraints.get(i);
         generateDerivativeCoefficientsAndBiasMatrix(tempCoeffMatrix2, tempBiasMatrix2, comCoefficients, comPolynomialOrder, 0, nodeTime);
         generateDerivativeCoefficientsAndBiasMatrix(tempCoeffMatrix1, tempBiasMatrix1, copCoefficients, copPolynomialOrder, 0, nodeTime);
         generateDerivativeCoefficientsAndBiasMatrix(tempCoeffMatrixForScalar, tempBiasMatrixForScalar, scalarCoefficients, scalarPolynomialOrder, 0, nodeTime);
         double u = tempBiasMatrixForScalar.get(0, 0);
         double x = tempBiasMatrix2.get(0, 0);
         double v = tempBiasMatrix1.get(0, 0);
         for (int j = comPolynomialOrder; j >= 2; j--)
            tempCoeffMatrix2.set(0, j, tempCoeffMatrix2.get(0, j - 2) * j * (j - 1) - u * tempCoeffMatrix2.get(0, j));
         for (int j = 1; j >= 0; j--)
            tempCoeffMatrix2.set(0, j, -u * tempCoeffMatrix2.get(0, j));
         CommonOps.scale(u, tempCoeffMatrix1);
         CommonOps.scale(v - x, tempCoeffMatrixForScalar);

         CommonOps.insert(tempCoeffMatrix2, comCoefficientMatrixToSet, i, 0);
         CommonOps.insert(tempCoeffMatrix1, copCoefficientMatrixToSet, i, 0);
         CommonOps.insert(tempCoeffMatrixForScalar, scalarCoefficientMatrixToSet, i, 0);
         biasMatrixToSet.set(i, 0, u * (x - v) + gravity);
      }
   }

   private final DenseMatrix64F tempConstraint1 = new DenseMatrix64F(0, 1);
   private final DenseMatrix64F tempConstraint2 = new DenseMatrix64F(0, 1);

   public void generateSupportPolygonConstraint(DenseMatrix64F xAxisConstraintCoefficientToSet, DenseMatrix64F yAxisConstraintCoefficientToSet,
                                                DenseMatrix64F biasMatrixToSet, DenseMatrix64F xAxisTrajectoryCoefficients,
                                                DenseMatrix64F yAxisTrajectoryCoefficients, ConvexPolygon2D supportPolygon, List<Double> nodeTimes,
                                                int copTrajectoryOrder)
   {
      int numberOfConstraintPerTime = supportPolygon.getNumberOfVertices();
      xAxisConstraintCoefficientToSet.reshape(nodeTimes.size() * numberOfConstraintPerTime, copTrajectoryOrder + 1);
      yAxisConstraintCoefficientToSet.reshape(nodeTimes.size() * numberOfConstraintPerTime, copTrajectoryOrder + 1);
      biasMatrixToSet.reshape(nodeTimes.size() * numberOfConstraintPerTime, 1);
      for (int i = 0; i < nodeTimes.size(); i++)
      {
         Point2DReadOnly initialPoint = supportPolygon.getVertex(numberOfConstraintPerTime - 1);
         generateDerivativeCoefficientsAndBiasMatrix(tempCoeffMatrix1, tempBiasMatrix1, xAxisTrajectoryCoefficients, copTrajectoryOrder, 0,
                                                     nodeTimes.get(i));
         generateDerivativeCoefficientsAndBiasMatrix(tempCoeffMatrix2, tempBiasMatrix2, yAxisTrajectoryCoefficients, copTrajectoryOrder, 0,
                                                     nodeTimes.get(i));
         for (int j = 0; j < numberOfConstraintPerTime; j++)
         {
            Point2DReadOnly finalPoint = supportPolygon.getVertex(j);
            double xCoefficient = initialPoint.getY() - finalPoint.getY();
            double yCoefficient = finalPoint.getX() - initialPoint.getX();
            double bias = initialPoint.getY() * finalPoint.getX() - initialPoint.getX() * finalPoint.getY();

            tempConstraint1.reshape(tempCoeffMatrix1.getNumRows(), tempCoeffMatrix1.getNumCols());
            tempConstraint2.reshape(tempCoeffMatrix2.getNumRows(), tempCoeffMatrix2.getNumCols());
            CommonOps.scale(xCoefficient, tempCoeffMatrix1, tempConstraint1);
            CommonOps.scale(yCoefficient, tempCoeffMatrix2, tempConstraint2);
            
            CommonOps.insert(tempConstraint1, xAxisConstraintCoefficientToSet, i * numberOfConstraintPerTime + j, 0);
            CommonOps.insert(tempConstraint2, yAxisConstraintCoefficientToSet, i * numberOfConstraintPerTime + j, 0);
            biasMatrixToSet.set(i * numberOfConstraintPerTime + j, 0, bias - tempBiasMatrix1.get(0, 0) * xCoefficient - tempBiasMatrix2.get(0, 0) * yCoefficient);
            initialPoint = finalPoint;
         }
      }
   }
}