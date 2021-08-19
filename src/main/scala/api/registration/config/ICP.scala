package api.registration.config

import api.registration.utils.NonRigidClosestPointRegistrator.ClosestPointTriangleMesh3D
import api.{CorrespondencePairs, DefaultRegistrationPars, GiNGRConfig, UncertaintyComputationPars}
import breeze.linalg.{DenseMatrix, DenseVector}
import scalismo.common.PointId
import scalismo.geometry._3D
import scalismo.mesh.TriangleMesh
import scalismo.statisticalmodel.MultivariateNormalDistribution

class icpUncertaintyComp extends UncertaintyComputationPars {
  val initSigma = 100
  val endSigma = 1.0
}

class ICP extends GiNGRConfig {
  val icpUncert = new icpUncertaintyComp
  var sigma2 = 1.0
  var step = 1.0

  override def Initialize(reference: TriangleMesh[_3D], target: TriangleMesh[_3D], default: DefaultRegistrationPars): Unit = {
    sigma2 = icpUncert.initSigma
    val delta = icpUncert.initSigma - icpUncert.endSigma
    step = delta.toDouble / default.max_iterations.toDouble
    println(s"Step: ${step}, delta: ${delta}")
  }

  override def GetCorrespondence(reference: TriangleMesh[_3D], target: TriangleMesh[_3D]): CorrespondencePairs = {
    val corr = ClosestPointTriangleMesh3D.closestPointCorrespondence(reference, target)
    CorrespondencePairs(pairs = corr._1.filter(_._3 == 1.0).map(f => (f._1, f._2)).toIndexedSeq)
  }

  override def UpdateUncertainty(meanUpdate: TriangleMesh[_3D]): Unit = {
    sigma2 = sigma2 - step
    println(s"New uncertainty: ${sigma2}")
  }

  override def PointIdUncertainty(id: PointId): MultivariateNormalDistribution = {
    MultivariateNormalDistribution(DenseVector.zeros[Double](3), DenseMatrix.eye[Double](3) * sigma2)
  }
}