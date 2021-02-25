package api.registration

import api.registration.cpd.{BCPDwithGPMM, NonRigidCPDwithGPMM}
import api.registration.utils.{PointSequenceConverter, SimilarityTransformParameters, TransformationHelper}
import breeze.linalg.DenseVector
import scalismo.common.{DiscreteDomain, DiscreteField, DomainWarp, Vectorizer}
import scalismo.geometry.{EuclideanVector, NDSpace, Point}
import scalismo.statisticalmodel.PointDistributionModel
import scalismo.transformations.{Scaling, Translation}

import scala.language.higherKinds

class GpmmCpdRegistration[D: NDSpace, DDomain[A] <: DiscreteDomain[A]](
                                                                        gpmm: PointDistributionModel[D, DDomain],
                                                                        lambda: Double = 2,
                                                                        w: Double = 0,
                                                                        max_iterations: Int = 100)(implicit warper: DomainWarp[D, DDomain], vectorizer: Vectorizer[Point[D]], pointSequenceConverter: PointSequenceConverter[D]) {
  val cpd = new NonRigidCPDwithGPMM(gpmm, lambda, w, max_iterations)

  def registrationMethod(target: DDomain[D], tolerance: Double): DenseVector[Double] = cpd.Registration(target.pointSet.points.toSeq, tolerance)

  def registerAndWarp(target: DDomain[D], tolerance: Double = 0.001): DDomain[D] = {
    val template = gpmm.reference
    val registrationPars = registrationMethod(target, tolerance)
    val registration = gpmm.instance(registrationPars)
    val warpField = DiscreteField(template, template.pointSet.points.toIndexedSeq.zip(registration.pointSet.points.toIndexedSeq).map { case (a, b) => b - a })
    warper.transformWithField(template, warpField)
  }

  def register(target: DDomain[D], tolerance: Double = 0.001): DenseVector[Double] = {
    registrationMethod(target, tolerance)
  }
}

class GpmmBcpdRegistration[D: NDSpace, DDomain[A] <: DiscreteDomain[A]](
                                                                         gpmm: PointDistributionModel[D, DDomain],
                                                                         targetPoints: DDomain[D],
                                                                         w: Double = 0, // Outlier, [0,1]
                                                                         lambda: Double = 2.0, // Noise scaling, R+
                                                                         gamma: Double = 1.0, // Initial noise scaling, R+
                                                                         k: Double = 1.0,
                                                                         max_iterations: Int = 100
                                                                       )(implicit warper: DomainWarp[D, DDomain], vectorizer: Vectorizer[Point[D]], pointSequenceConverter: PointSequenceConverter[D], simTrans: TransformationHelper[D]) {
  val bcpd = new BCPDwithGPMM(gpmm, targetPoints.pointSet.points.toSeq, w, lambda, gamma, k, max_iterations)
  private val defaultInitialGPMMPars = DenseVector.zeros[Double](gpmm.rank)

  private val defaultTransformationPars = SimilarityTransformParameters[D](
    s = Scaling[D](1.0),
    t = Translation[D](EuclideanVector.fromBreezeVector(DenseVector.zeros[Double](vectorizer.dim))),
    R = simTrans.zeroRotationInitialization
  )

  def registrationMethod(tolerance: Double, initialGPMM: DenseVector[Double], initialTrans: SimilarityTransformParameters[D]): (DenseVector[Double], SimilarityTransformParameters[D]) = {
    bcpd.Registration(tolerance, initialGPMMpars = initialGPMM, initialTransformation = initialTrans)
  }

  //  def registerAndWarp(tolerance: Double = 0.001, initialPars: DenseVector[Double] = defaultInitialGPMMPars, initialTranform: SimilarityTransformation[D] = defaultInitialTranform): DDomain[D] = {
  //    val template = gpmm.reference
  //    val (registrationPars, registrationTrans) = registrationMethod(tolerance, initialPars, initialTranform)
  //    val registration = gpmm.transform(registrationTrans) //gpmm.instance(registrationPars)
  //    val warpField = DiscreteField(template, template.pointSet.points.toIndexedSeq.zip(registration.pointSet.points.toIndexedSeq).map { case (a, b) => b - a })
  //    warper.transformWithField(template, warpField)
  //  }

  def register(tolerance: Double = 0.001, initialGPMM: DenseVector[Double] = defaultInitialGPMMPars, initialTrans: SimilarityTransformParameters[D] = defaultTransformationPars): (DenseVector[Double], SimilarityTransformParameters[D]) = {
    registrationMethod(tolerance, initialGPMM, initialTrans)
  }
}