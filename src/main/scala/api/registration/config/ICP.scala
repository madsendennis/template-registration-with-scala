package api.registration.config

import api.registration.utils.NonRigidClosestPointRegistrator.ClosestPointTriangleMesh3D
import api.{CorrespondencePairs, GingrAlgorithm, GingrConfig, GingrRegistrationState}
import breeze.linalg.{DenseMatrix, DenseVector}
import scalismo.common.PointId
import scalismo.geometry.{Point, _3D}
import scalismo.mesh.TriangleMesh
import scalismo.statisticalmodel.{MultivariateNormalDistribution, PointDistributionModel}
import scalismo.transformations.{RigidTransformation, TranslationAfterRotationSpace3D}

object ICPCorrespondence {
  def estimate[T](state: GingrRegistrationState[T]): CorrespondencePairs = {
    val source = state.fit()
    val target = state.target()
    val corr = ClosestPointTriangleMesh3D.closestPointCorrespondence(source, target)
    CorrespondencePairs(pairs = corr._1.filter(_._3 == 1.0).map(f => (f._1, f._2)).toIndexedSeq)
  }
}

case class IcpRegistrationState(
                                 override val model: PointDistributionModel[_3D, TriangleMesh],
                                 override val modelParameters: DenseVector[Double],
                                 override val target: TriangleMesh[_3D],
                                 override val fit: TriangleMesh[_3D],
                                 override val rigidAlignment: RigidTransformation[_3D],
                                 override val scaling: Double = 1.0,
                                 override val converged: Boolean,
                                 sigma2: Double = 1.0,
                                 override val iteration: Int = 0
                               ) extends GingrRegistrationState[IcpRegistrationState] {
  override def updateFit(next: TriangleMesh[_3D]): IcpRegistrationState = this.copy(fit = next)
}

case class IcpConfiguration(
                             override val maxIterations: Int = 100,
                             override val converged: (IcpRegistrationState, IcpRegistrationState) => Boolean = (last: IcpRegistrationState, current: IcpRegistrationState) => false,
                             initialSigma: Double = 100.0,
                             endSigma: Double = 1.0
                           ) extends GingrConfig[IcpRegistrationState] {}

class IcpRegistration(
                       val target: TriangleMesh[_3D],
                       val config: IcpConfiguration,
                       val pdm: PointDistributionModel[_3D, TriangleMesh],
                       override val getCorrespondence: IcpRegistrationState => CorrespondencePairs = (state: IcpRegistrationState) => ICPCorrespondence.estimate(state),
                       override val getUncertainty: (PointId, IcpRegistrationState) => MultivariateNormalDistribution = (id: PointId, state: IcpRegistrationState) =>
                         MultivariateNormalDistribution(DenseVector.zeros[Double](3), DenseMatrix.eye[Double](3) * state.sigma2)
                     ) extends GingrAlgorithm[IcpRegistrationState] {

  override def initialize(): IcpRegistrationState = {
    val initial =
      IcpRegistrationState(
        pdm,
        DenseVector.zeros[Double](pdm.rank),
        target,
        pdm.mean,
        TranslationAfterRotationSpace3D(Point(0, 0, 0)).identityTransformation,
        1.0,
        false,
        config.initialSigma,
        config.maxIterations
      )
    initial
  }

  val sigmaStep = (config.initialSigma - config.endSigma) / config.maxIterations.toDouble

  def updateSigma(current: Double): Double = {
    current - sigmaStep;
  }

  // possibility to override the update function, or just use the base class method?
  override def update(current: IcpRegistrationState): IcpRegistrationState = {
    val correspondences = getCorrespondence(current)
    val uncertainObservations = correspondences.pairs.map { pair =>
      val (pid, point) = pair
      val uncertainty = getUncertainty(pid, current)
      (pid, point, uncertainty)
    }
    val mean = current.model.posterior(uncertainObservations).mean
    val alpha = current.model.coefficients(mean)
    current.copy(
      modelParameters = alpha,
      fit = mean,
      sigma2 = updateSigma(current.sigma2),
      iteration = current.iteration - 1
    )
  }
}

