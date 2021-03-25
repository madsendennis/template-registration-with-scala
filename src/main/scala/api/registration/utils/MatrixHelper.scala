package api.registration.utils

import breeze.linalg.{DenseMatrix, diag, svd}

object MatrixHelper {
  def pinv(m: DenseMatrix[Double], precision: Double = 0.00001): DenseMatrix[Double] = {
    val Decomp = svd(m)
    Decomp.U * diag(Decomp.∑.map(d=>if(d>precision) 1.0/d else 0.0)) * Decomp.Vt
  }
}
