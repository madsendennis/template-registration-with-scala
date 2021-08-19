package api

import api.registration.utils.{AlignmentTransforms, GlobalTranformationType, NoTransforms, SimilarityTransformParameters}
import breeze.linalg.{DenseMatrix, DenseVector}
import scalismo.common.{DiscreteDomain, PointId}
import scalismo.geometry.{EuclideanVector, Landmark, NDSpace, Point, _3D}
import scalismo.mesh.TriangleMesh
import scalismo.statisticalmodel.{MultivariateNormalDistribution, PointDistributionModel}
import scalismo.transformations.{RigidTransformation, Rotation, Scaling, Translation, TranslationAfterRotation, TranslationAfterRotation3D}

case class IterationPars(model: PointDistributionModel[_3D, TriangleMesh], alpha: DenseVector[Double], transform: SimilarityTransformParameters[_3D])

//trait CorrespondenceConfig
case class CorrespondencePairs(pairs: IndexedSeq[(PointId, Point[_3D])])


case class DefaultRegistrationPars(max_iterations: Int, tolerance: Double, landmarkAlignTarget: Boolean)

trait UncertaintyComputationPars

trait GiNGRConfig {
  def Initialize(reference: TriangleMesh[_3D], target: TriangleMesh[_3D], default: DefaultRegistrationPars): Unit

  def GetCorrespondence(reference: TriangleMesh[_3D], target: TriangleMesh[_3D]): CorrespondencePairs

  def UpdateUncertainty(meanUpdate: TriangleMesh[_3D]): Unit

  def PointIdUncertainty(id: PointId): MultivariateNormalDistribution
}

trait GiNGRbase {
  def Registration(target: TriangleMesh[_3D], targetLandmarks: Option[Seq[Landmark[_3D]]] = None, regConfig: DefaultRegistrationPars): (PointDistributionModel[_3D, TriangleMesh], DenseVector[Double], SimilarityTransformParameters[_3D])

  def Iteration(pars: IterationPars, target: TriangleMesh[_3D]): IterationPars
}


class GiNGR(model: PointDistributionModel[_3D, TriangleMesh],
            registrationConfig: GiNGRConfig,
            transformationType: GlobalTranformationType = NoTransforms,
            modelLandmarks: Option[Seq[Landmark[_3D]]] = None
           ) extends GiNGRbase {

  override def Registration(target: TriangleMesh[_3D], targetLandmarks: Option[Seq[Landmark[_3D]]], regConfig: DefaultRegistrationPars): (PointDistributionModel[_3D, TriangleMesh], DenseVector[Double], SimilarityTransformParameters[_3D]) = {
    val initGlobalTransform: TranslationAfterRotation[_3D] = if (regConfig.landmarkAlignTarget) { // Maybe align model instead and return alignment as globalTrans ???
      AlignmentTransforms.computeTransform(modelLandmarks.get, targetLandmarks.get, Point(0, 0, 0))
    } else TranslationAfterRotation[_3D](Translation(EuclideanVector(0, 0, 0)), Rotation(0, 0, 0, Point(0, 0, 0)))

    val modelAligned = model.transform(initGlobalTransform)

    registrationConfig.Initialize(modelAligned.mean, target, regConfig) // Initialize method specific parameters
    val initGlobalTrans = SimilarityTransformParameters[_3D](Scaling(1.0), Translation(EuclideanVector(0, 0, 0)), Rotation(0, 0, 0, Point(0, 0, 0))) // Input initial Global model transform (needed if calling Register multiple times with different detail levels)
    val initAlpha = DenseVector.zeros[Double](modelAligned.rank)
    val initPars = IterationPars(model = modelAligned, alpha = initAlpha, transform = initGlobalTrans) // Also possible to input from user as with GlobalTrans

    val fit = (0 until regConfig.max_iterations).foldLeft(initPars) { (pars, i) =>
      val iterationPars = Iteration(pars, target)
      println(s"Iteration: ${i}/${regConfig.max_iterations}") // Update with logger.info instead
      iterationPars
    }
    val globalTrans = SimilarityTransformParameters[_3D](Scaling(1.0), initGlobalTransform.translation, initGlobalTransform.rotation) // Update to be the iterated model update
    (modelAligned, fit.alpha, globalTrans)
  }

  override def Iteration(pars: IterationPars, target: TriangleMesh[_3D]): IterationPars = {
    //    val globalTrans = pars.transform.
    val instance = pars.model.instance(pars.alpha) // Apply possible global transformation
    val corrPairs = registrationConfig.GetCorrespondence(instance, target)
    //    val posteriorMean = gpmm.newReference(instance, NearestNeighborInterpolator()).posterior(cp, sigma2).mean
    val corrWithUncertainty = corrPairs.pairs.map { f => (f._1, f._2, registrationConfig.PointIdUncertainty(f._1)) } // Add uncertainty pr/landmark
    val posteriorMean = pars.model.posterior(corrWithUncertainty).mean // Update uncertainty to: Map[PointId, MultivariateNormalDistribution]

    registrationConfig.UpdateUncertainty(posteriorMean)

    pars.copy(alpha = pars.model.coefficients(posteriorMean))
  }
}


//class PGiNGR() extends GiNGR{
//
//}
