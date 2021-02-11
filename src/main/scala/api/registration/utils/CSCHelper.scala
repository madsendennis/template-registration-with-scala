package api.registration.utils

import breeze.linalg.{CSCMatrix, DenseMatrix}

object CSCHelper {
  def eye(N: Int): CSCMatrix[Double] = {
    val mat = CSCMatrix.zeros[Double](N, N)
    (0 until N).foreach(i => mat(i, i) = 1.0)
    mat
  }

  def DenseMatrix2CSCMatrix(mat: DenseMatrix[Double]): CSCMatrix[Double] = {
    CSCMatrix.tabulate(mat.rows, mat.cols)(mat(_, _))
  }

  def CSCMatrixMultipliedWithDouble(mat: CSCMatrix[Double], scale: Double): CSCMatrix[Double] = {
    mat.mapValues(_ * scale)
  }

  def vertcat(matrices: CSCMatrix[Double]*): CSCMatrix[Double] = {
    require(matrices.forall(m => m.cols == matrices(0).cols), "Not all matrices have the same number of columns")
    val numRows = matrices.foldLeft(0)(_ + _.rows)
    val numCols = matrices(0).cols
    val res = CSCMatrix.zeros[Double](numRows, numCols)
    var offset = 0
    for (m <- matrices) {
      res((offset) until (offset + m.rows), 0 until numCols) := m
      offset += m.rows
    }
    res
  }

  // Update with: https://stackoverflow.com/questions/44461658/efficient-kronecker-product-with-identity-matrix-and-regular-matrix-numpy-pyt
  def kroneckerProduct(matrix1: CSCMatrix[Double], matrix2: CSCMatrix[Double]): CSCMatrix[Double] = {
    val r1 = matrix1.rows
    val c1 = matrix1.cols
    val r2 = matrix2.rows
    val c2 = matrix2.cols

    val res = CSCMatrix.zeros[Double](r1 * r2, c1 * c2)

    for (
      i <- 0 until r1;
      j <- 0 until c1;
      k <- 0 until r2;
      l <- 0 until c2
    ) {
      res(r2 * i + k, c2 * j + l) = matrix1(i, j) * matrix2(k, l)
    }
    res
  }
}
