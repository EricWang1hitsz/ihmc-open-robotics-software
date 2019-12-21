package us.ihmc.robotics.linearAlgebra.careSolvers;

import org.ejml.data.DenseMatrix64F;
import org.ejml.data.Matrix;
import org.ejml.factory.DecompositionFactory;
import org.ejml.factory.LinearSolverFactory;
import org.ejml.interfaces.decomposition.EigenDecomposition;
import org.ejml.interfaces.decomposition.SingularValueDecomposition;
import org.ejml.interfaces.linsol.LinearSolver;
import org.ejml.ops.CommonOps;
import us.ihmc.commons.MathTools;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.matrixlib.NativeCommonOps;

/**
 * This solver computes the solution to the algebraic Riccati equation
 *
 * <p>
 * A' P + P A - P B R^-1 B' P + Q = 0
 * </p>
 * <p> which can also be written as</p>
 * <p>A' P + P A - P M P + Q = 0</p>*
 * <p>where P is the unknown to be solved for, R is symmetric positive definite, Q is symmetric positive semi-definite, A is the state transition matrix,
 * and B is the control matrix.</p>
 *
 * <p>
 *    The solution is found by computing the Hamiltonian and performing an ordered eigen value decomposition, as outlined in
 *    https://en.wikipedia.org/wiki/Algebraic_Riccati_equation and https://stanford.edu/class/ee363/lectures/clqr.pdf. This assumes that the Hamiltonian
 *    has only real eigenvalues, with no complex conjugate pairs.
 * </p>
 */
public class HamiltonianIterativeSchurCARESolver implements CARESolver
{
   /** The solution of the algebraic Riccati equation. */
   private final DenseMatrix64F P = new DenseMatrix64F(0, 0);

   private final SingularValueDecomposition<DenseMatrix64F> svd = DecompositionFactory.svd(0, 0, false, false, false);
   private final SchurDecomposition<DenseMatrix64F> schur = SchurDecompositionFactory.qrBased(0);

   private final DenseMatrix64F BTranspose = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F ATranspose = new DenseMatrix64F(0, 0);

   private final DenseMatrix64F A = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F Q = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F Rinv = new DenseMatrix64F(0, 0);

   /** Hamiltonian of the Riccati equation. */
   private final DenseMatrix64F H = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F M = new DenseMatrix64F(0, 0);

   private final DenseMatrix64F u = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F u1 = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F u2 = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F u1Inv = new DenseMatrix64F(0, 0);

   private boolean isUpToDate = false;
   private int n;

   /** {@inheritDoc} */
   public void setMatrices(DenseMatrix64F A, DenseMatrix64F B, DenseMatrix64F Q, DenseMatrix64F R, boolean checkMatrices)
   {
      isUpToDate = false;

      // checking A
      MatrixChecking.assertIsSquare(A);
      MatrixChecking.assertMultiplicationCompatible(A, B);
      MatrixChecking.assertMultiplicationCompatible(B, R);
      MatrixChecking.assertMultiplicationCompatible(A, Q);

      if (checkMatrices)
      {
         // checking R
         svd.decompose(R);
         if (MathTools.min(svd.getSingularValues()) == 0.0)
            throw new IllegalArgumentException("R Matrix is singular.");
      }

      this.n = A.getNumRows();

      BTranspose.set(B);
      CommonOps.transpose(BTranspose);

      Rinv.reshape(n, n);
      NativeCommonOps.invert(R, Rinv);
      M.reshape(n, n);
      NativeCommonOps.multQuad(BTranspose, Rinv, M);

      this.n = A.getNumRows();

      this.A.set(A);
      this.Q.set(Q);
      this.M.set(M);
   }

   /** {@inheritDoc} */
   public DenseMatrix64F computeP()
   {
      // defining Hamiltonian
      assembleHamiltonian(H);

      boolean converged = false;
      int iterations = 0;

      schur.decompose(H);
      DenseMatrix64F U = schur.getU(null);
      DenseMatrix64F T = schur.getT(null);

      DenseMatrix64F U11 = new DenseMatrix64F(n, n);
      DenseMatrix64F U11Inv = new DenseMatrix64F(n, n);
      DenseMatrix64F U21 = new DenseMatrix64F(n, n);

      MatrixTools.setMatrixBlock(U11, 0 ,0, U, 0, 0, n, n, 1.0);
      MatrixTools.setMatrixBlock(U21, 0 ,0, U, n, 0, n, n, 1.0);

      NativeCommonOps.invert(U11, U11Inv);

      CommonOps.mult(U21, U11Inv, P);

      isUpToDate = true;

      return P;
   }

   private static final int maxIterations = Integer.MAX_VALUE;
   private static final double epsilon = 1e-12;


   /** {inheritDoc} */
   public DenseMatrix64F getP()
   {
      return isUpToDate ? P : computeP();
   }

   private void assembleHamiltonian(DenseMatrix64F hamiltonianToPack)
   {
      ATranspose.reshape(n, n);
      CommonOps.transpose(A, ATranspose);

      hamiltonianToPack.reshape(2 * n, 2 * n);

      MatrixTools.setMatrixBlock(hamiltonianToPack, 0, 0, A, 0, 0, n, n, 1.0);
      MatrixTools.setMatrixBlock(hamiltonianToPack, n, 0, Q, 0, 0, n, n, -1.0);
      MatrixTools.setMatrixBlock(hamiltonianToPack, 0, n, M, 0, 0, n, n, -1.0);
      MatrixTools.setMatrixBlock(hamiltonianToPack, n, n, ATranspose, 0, 0, n, n, -1.0);
   }


}
