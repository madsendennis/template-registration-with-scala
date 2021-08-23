package apps

import api.CorrespondencePairs
import breeze.linalg.{DenseMatrix, DenseVector}
import scalismo.common.PointId
import scalismo.geometry.{Landmark, Point, _3D}
import scalismo.mesh.TriangleMesh3D
import scalismo.statisticalmodel.{MultivariateNormalDistribution, PointDistributionModel, StatisticalMeshModel}
import scalismo.transformations.{RigidTransformation, TranslationAfterRotationSpace3D}
import api.registration.utils.NonRigidClosestPointRegistrator.ClosestPointTriangleMesh3D

trait GingrConfig[T <: GingrRegistrationState[T]] {
  def maxIterations(): Int
  def converged: (T, T) => Boolean
}

trait GingrRegistrationState[T <: GingrRegistrationState[T]] {
  val iteration: Int

  /** initial prior model */
  def model(): PointDistributionModel[_3D, TriangleMesh3D]

  /** parameters of the current fitting state in the initial prior model */
  def modelParameters(): DenseVector[Double]

  def target(): TriangleMesh3D
  def fit(): TriangleMesh3D
  def rigidAlignment(): RigidTransformation[_3D]
  def scaling(): Double = 1.0
  def converged(): Boolean

  /**
    * Updates the current state with the new fit.
    * @param next The newly calculated shape / fit.
    */
  private[apps] def updateFit(next: TriangleMesh3D): T
}

trait GingrAlgorithm[State <: GingrRegistrationState[State]] {
  def initialize(): State
  val getCorrespondence: (State) => CorrespondencePairs
  val getUncertainty: (PointId, State) => MultivariateNormalDistribution

  def update(current: State): State = {
    val currentFit = current.fit()
    val correspondences = getCorrespondence(current)
    val uncertainObservations = correspondences.pairs.map { pair =>
      val (pid, point) = pair
      val uncertainty = getUncertainty(pid, current)
      (pid, point, uncertainty)
    }
    val posterior = current.model.posterior(uncertainObservations)
    current.updateFit(posterior.mean)
  }

  def run(
      target: TriangleMesh3D,
      targetLandmarks: Option[Seq[Landmark[_3D]]],
      model: StatisticalMeshModel,
      modelLandmarks: Option[Seq[Landmark[_3D]]]
  ): State = {
    val initialState: State = initialize()
    runFromState(target, targetLandmarks, model, modelLandmarks, initialState)
  }

  def runFromState(
      target: TriangleMesh3D,
      targetLandmarks: Option[Seq[Landmark[_3D]]],
      model: StatisticalMeshModel,
      modelLandmarks: Option[Seq[Landmark[_3D]]],
      initialState: State
  ): State = {

    val registration: Iterator[State] = Iterator.iterate(initialState) { current =>
      val next = update(current)
      next
    }

    val states: Iterator[State] = registration.take(100)
    val fit: State = states.dropWhile(state => !state.converged()).next()
    fit
  }
}

object Correspondence {
  def function1[T](state: GingrRegistrationState[T]) = {
    val source = state.fit
    val target = state.target
    val corr = ClosestPointTriangleMesh3D.closestPointCorrespondence(source, target)
    CorrespondencePairs(pairs = corr._1.filter(_._3 == 1.0).map(f => (f._1, f._2)).toIndexedSeq)
  }
}

case class IcpRegistrationState(
    override val model: PointDistributionModel[_3D, TriangleMesh3D],
    override val modelParameters: DenseVector[Double],
    override val target: TriangleMesh3D,
    override val fit: TriangleMesh3D,
    override val rigidAlignment: RigidTransformation[_3D],
    override val scaling: Double = 1.0,
    override val converged: Boolean,
    sigma2: Double = 1.0,
    override val iteration: Int = 0
) extends GingrRegistrationState[IcpRegistrationState] {
  override def updateFit(next: TriangleMesh3D): IcpRegistrationState = this.copy(fit = next)
}

case class IcpConfiguration(
    override val maxIterations: Int = 100,
    override val converged: (IcpRegistrationState, IcpRegistrationState) => Boolean = (_: IcpRegistrationState, _: IcpRegistrationState) => false,
    initialSigma: Double = 100.0,
    endSigma: Double = 1.0
) extends GingrConfig[IcpRegistrationState] {}

class IcpRegistration(
    val target: TriangleMesh3D,
    val config: IcpConfiguration,
    val pdm: PointDistributionModel[_3D, TriangleMesh3D],
    override val getCorrespondence: IcpRegistrationState => CorrespondencePairs = (state: IcpRegistrationState) => Correspondence.function1(state),
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
        DenseVector.zeros[Double](pdm.rank),
        TranslationAfterRotationSpace3D(Point(0, 0, 0)).identityTransformation,
        false
      )
    initial
  }

  val sigmaStep = (config.initialSigma - config.endSigma).toDouble / config.maxIterations
  def updateSigma(current: Double): Double = {
    current - sigmaStep;
  }

  // possibility to override the update function, or just use the base class method?
  override def update(current: IcpRegistrationState): IcpRegistrationState = {
    current.copy(
      fit = ???,
      sigma2 = updateSigma(current.sigma2),
      iteration = current.iteration + 1
    )
  }
}

object IcpUserApplication extends App {
  // data
  val model: StatisticalMeshModel = ???
  val target: TriangleMesh3D = ???

  // parameters for algorithm
  val config: IcpConfiguration = IcpConfiguration(maxIterations = 7)

  val registration: IcpRegistration = new IcpRegistration(target, config, PointDistributionModel[_3D, TriangleMesh3D](model.gp))

  val fit: IcpRegistrationState = registration.run(target, None, model, None)
}
