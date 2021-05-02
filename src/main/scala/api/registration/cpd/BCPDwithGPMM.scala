package api.registration.cpd

import api.registration.utils._
import breeze.linalg.{Axis, DenseMatrix, DenseVector, diag, pinv, sum, tile, trace}
import breeze.numerics.digamma
import scalismo.common._
import scalismo.common.interpolation.NearestNeighborInterpolator
import scalismo.geometry._
import scalismo.mesh.TriangleMesh
import scalismo.statisticalmodel.{MultivariateNormalDistribution, PointDistributionModel}
import scalismo.transformations.TranslationAfterScalingAfterRotation

// Similarity transformation parameters
case class BCPDParameters[D](alpha: DenseVector[Double], sigma2: Double, simTrans: SimilarityTransformParameters[D])

abstract class GPMMfitting[D: NDSpace, DDomain[D] <: DiscreteDomain[D]](
                                                                         gpmm: PointDistributionModel[D, DDomain],
                                                                         targetDomain: DDomain[D],
                                                                         w: Double, // Outlier, [0,1]
                                                                         lambda: Double, // Noise scaling, R+
                                                                         gamma: Double, // Initial noise scaling, R+
                                                                         k: Double,
                                                                         max_iterations: Int,
                                                                         modelView: Option[modelViewer]
                                                                       )(
                                                                         implicit vectorizer: Vectorizer[Point[D]],
                                                                         domainWarper: DomainWarp[D, DDomain],
                                                                         dataConverter: PointSequenceConverter[D],
                                                                         simTrans: TransformationHelper[D],
                                                                         viewer: ModelViewerHelper[D]
                                                                       ) {
  println("New version using GPMM and BCPD update method")
  require(lambda > 0)
  require(gamma > 0)
  require(k > 0)
  require(0 <= w && w < 1.0)

  val targetPoints: Seq[Point[D]] =  targetDomain.pointSet.points.toSeq

  val dim: Int = vectorizer.dim
  val referencePoints: Seq[Point[D]] = gpmm.reference.pointSet.points.toSeq
  val M: Int = referencePoints.length
  val N: Int = targetPoints.length

  val Xmat: DenseMatrix[Double] = dataConverter.toMatrix(targetPoints)
  val Yvec: DenseVector[Double] = dataConverter.toVector(referencePoints)

  def Registration(tolerance: Double, transformationType: GlobalTranformationType, initialGPMMpars: DenseVector[Double], initialTransformation: SimilarityTransformParameters[D]): (DenseVector[Double], SimilarityTransformParameters[D], Double) = {
    val instance = gpmm.instance(initialGPMMpars) //TODO: Add similarityTransform
    val sigma2Init = gamma * computeSigma2init(instance.pointSet.points.toSeq, targetPoints)

    val parsInit = BCPDParameters[D](
      alpha = DenseVector.ones[Double](M) / M.toDouble,
      sigma2 = sigma2Init,
      simTrans = initialTransformation
    )

    val fit = (0 until max_iterations).foldLeft((initialGPMMpars, parsInit)) { (it, i) =>
      val gpmmParsInit = it._1
      val pars = it._2
      val sigma2 = pars.sigma2
      println(s"BCPD, iteration: ${i}/${max_iterations}, sigma2: ${sigma2}")
      val iter = Iteration(gpmmParsInit, pars, transformationType)
      if (modelView.nonEmpty) {
        if (i % modelView.get.updateFrequency == 0) {
          modelView.get.modelView.shapeTransformationView.coefficients = iter._1
          modelView.get.modelView.poseTransformationView.transformation = viewer.getRigidTransformation(iter._2.simTrans)
        }
      }

      val sigmaDiff = math.abs(iter._2.sigma2 - sigma2)
      if (sigmaDiff < tolerance) {
        println(s"Converged")
        return (iter._1, iter._2.simTrans, iter._2.sigma2)
      } else {
        iter
      }
    }
    (fit._1, fit._2.simTrans, fit._2.sigma2)
  }

  private def computeSigma2init(reference: Seq[Point[D]], target: Seq[Point[D]]): Double = {
    val M = reference.length
    val N = target.length
    val dim = vectorizer.dim
    val s: Double = (0 until M).flatMap { m =>
      (0 until N).map { n =>
        (reference(m) - target(n)).norm2
      }
    }.sum
    s / (dim * N * M)
  }

  def computeCorrespondenceProbability(movingRefPoints: Seq[Point[D]], sigma2: Double, s: Double, alpha: DenseVector[Double]): DenseMatrix[Double] = {
    val Phi = DenseMatrix.zeros[Double](M, N)
    (0 until M).map { m =>
      val mvnd = MultivariateNormalDistribution(vectorizer.vectorize(movingRefPoints(m)), DenseMatrix.eye[Double](dim) * sigma2)
      val e = math.exp(-s / (2 * sigma2))
      (0 until N).map { n =>
        Phi(m, n) = mvnd.pdf(vectorizer.vectorize(targetPoints(n))) * e * alpha(m)
      }
    }
    val Pinit = Phi.copy * (1 - w)
    // TODO: Fix outlier distribution (section 4.3.4)
    val c = w * 1.0 / N.toDouble // * pout(x_n) see 4.3.4 (1/V)
    val denRow = DenseMatrix(sum(Pinit, Axis._0).t) * (1 - w) + c
    val den = tile(denRow, M, 1)

    Pinit /:/ den
  }

  def Iteration(gpmmPars: DenseVector[Double], pars: BCPDParameters[D], transformationType: GlobalTranformationType): (DenseVector[Double], BCPDParameters[D])
}

class BCPDwithGPMM[D: NDSpace, DDomain[D] <: DiscreteDomain[D]](
                                                                 gpmm: PointDistributionModel[D, DDomain],
                                                                 targetDomain: DDomain[D],
                                                                 w: Double, // Outlier, [0,1]
                                                                 lambda: Double, // Noise scaling, R+
                                                                 gamma: Double, // Initial noise scaling, R+
                                                                 k: Double,
                                                                 max_iterations: Int,
                                                                 modelView: Option[modelViewer]
                                                               )(
                                                                 implicit vectorizer: Vectorizer[Point[D]],
                                                                 domainWarper: DomainWarp[D, DDomain],
                                                                 dataConverter: PointSequenceConverter[D],
                                                                 simTrans: TransformationHelper[D],
                                                                 viewer: ModelViewerHelper[D],
                                                                 closestPointRegistrator: ClosestPointRegistrator[D, DDomain]
                                                               ) extends GPMMfitting(gpmm, targetDomain, w, lambda, gamma, k, max_iterations, modelView) {

  override def Iteration(gpmmPars: DenseVector[Double], pars: BCPDParameters[D], transformationType: GlobalTranformationType): (DenseVector[Double], BCPDParameters[D]) = {
    val instance = gpmm.instance(gpmmPars)

    val (cpInfo, _) = closestPointRegistrator.closestPointCorrespondence(instance, targetDomain)
    val idFilter = cpInfo.map(_._3.toInt)

    val instancePoints = pars.simTrans.transform(instance.pointSet.points.toSeq)
    val P = computeCorrespondenceProbability(instancePoints, pars.sigma2, pars.simTrans.s.s, pars.alpha)
    val v = sum(P, Axis._1) // R^M Estimated number of target points matched with each source point
    val v_ = sum(P, Axis._0).t.copy // R^N Posterior probability that x_n is a non-outlier
    val Nhat = sum(v_)

    val xhat = dataConverter.toPointSequence(pinv(diag(v)) * P * Xmat)
    val xhatTinv = pars.simTrans.invTransform(xhat)

    // Update Local deformations
    val sigma2DIVs2 = pars.sigma2 / math.pow(pars.simTrans.s.s, 2)

    val gpTrainingData = (0 until M).map { i =>
      (PointId(i), xhatTinv(i), MultivariateNormalDistribution(DenseVector.zeros[Double](dim), DenseMatrix.eye[Double](dim) * lambda * sigma2DIVs2 / v(i)))
    }.zip(idFilter).filter(_._2==1).map(_._1)

    val gpmmRegression = gpmm.newReference(instance, NearestNeighborInterpolator()).posterior(gpTrainingData)
    val myVhat = gpmmRegression.gp.meanVector

    val uhat = Yvec + myVhat

    val alpha = v.map(f => math.exp(digamma(k + f) - digamma(k * M + Nhat)))

    // Update Similarity transform
    val sigma2bar = sum((0 until M).map(m => v(m))) / Nhat
    val uhatPoints = dataConverter.toPointSequence(uhat)

    val newTransform = transformationType match {
      case SimilarityTransforms => simTrans.getSimilarityTransform(uhatPoints, xhat)
      case RigidTransforms => simTrans.getRigidTransform(uhatPoints.zip(idFilter).filter(_._2==1).map(_._1), xhat.zip(idFilter).filter(_._2==1).map(_._1))
      case _ => pars.simTrans
    }

    val newYhatPoints = dataConverter.toPointSequence(Yvec + myVhat)
    val newYhat = dataConverter.toMatrix(pars.simTrans.transform(newYhatPoints))

    val sXX = trace(Xmat.t * diag(v_) * Xmat)
    val sXY = trace(Xmat.t * P.t * newYhat)
    val sYY = trace(newYhat.t * diag(v) * newYhat)
    val sC = pars.sigma2 * sigma2bar

    val newSigma2 = (sXX - 2 * sXY + sYY + sC) / (Nhat * dim) // TODO: Should sC be added to all or included in the parenthesis as currently?

    val warpField = DiscreteField(gpmm.reference, referencePoints.toIndexedSeq.zip(newYhatPoints).map { case (a, b) => b - a })
    val newGpmmPars = gpmm.gp.coefficients(warpField)

    val newPars = BCPDParameters[D](
      alpha = alpha,
      sigma2 = newSigma2,
      simTrans = newTransform
    )

    (newGpmmPars, newPars)
  }
}


class SpecialICPwithGPMM(
                          gpmm: PointDistributionModel[_3D, TriangleMesh],
                          target: TriangleMesh[_3D],
                          w: Double, // Outlier, [0,1]
                          lambda: Double, // Noise scaling, R+
                          gamma: Double, // Initial noise scaling, R+
                          k: Double,
                          max_iterations: Int,
                          modelView: Option[modelViewer]
                        )(
                          implicit vectorizer: Vectorizer[Point[_3D]],
                          domainWarper: DomainWarp[_3D, TriangleMesh],
                          dataConverter: PointSequenceConverter[_3D],
                          simTrans: TransformationHelper[_3D],
                          viewer: ModelViewerHelper[_3D],
                          closestPointRegistrator: ClosestPointRegistrator[_3D, TriangleMesh]
                        ) extends GPMMfitting(gpmm, target, w, lambda, gamma, k, max_iterations, modelView) {

  override def Iteration(gpmmPars: DenseVector[Double], pars: BCPDParameters[_3D], transformationType: GlobalTranformationType): (DenseVector[Double], BCPDParameters[_3D]) = {
    val initialTransformation = TranslationAfterScalingAfterRotation(pars.simTrans.t, pars.simTrans.s, pars.simTrans.R)
    val instance = gpmm.instance(gpmmPars)
    val instanceTransform = instance.transform(initialTransformation)

    val closestPoints = ClosestPointRegistrator.ClosestPointTriangleMesh3D.closestPointCorrespondence(instanceTransform, target)._1
    val closestPointsFiltered = closestPoints.filter(_._3 == 1.0)

    val vSim = DenseVector(closestPoints.map(_._3).toArray)
    val numberOfMatchingPoints = sum(vSim)

    val xhat = closestPoints.map(_._2)
    val xhatFiltered = closestPointsFiltered.map(_._2)
    val xhatFilteredTinv = pars.simTrans.invTransform(xhatFiltered)

    // Update Local deformations
    val sigma2DIVs2 = pars.sigma2 / math.pow(pars.simTrans.s.s, 2)

    val gpTrainingData = (closestPointsFiltered zip xhatFilteredTinv).map { case (pInfo, cp) =>
      (pInfo._1, cp)
    }.toIndexedSeq
    val gpmmRegression = gpmm.newReference(instance, NearestNeighborInterpolator()).posterior(gpTrainingData, lambda * sigma2DIVs2)
    val myVhat = gpmmRegression.gp.meanVector

    val uhat = Yvec + myVhat

    val alpha = vSim.map(f => math.exp(digamma(k + f) - digamma(k * M + N)))

    // Update Similarity transform
    val sigma2bar = numberOfMatchingPoints / N
    val uhatPoints = dataConverter.toPointSequence(uhat)

    val newTransform = transformationType match {
      case SimilarityTransforms => simTrans.getSimilarityTransform(uhatPoints, xhat)
      case RigidTransforms => simTrans.getRigidTransform(uhatPoints, xhat)
      case _ => pars.simTrans
    }

    val newYhatPoints = dataConverter.toPointSequence(Yvec + myVhat)
    val newYhat = dataConverter.toMatrix(pars.simTrans.transform(newYhatPoints))
    val XmatClosest = dataConverter.toMatrix(xhat)
    val sXX = trace(Xmat.t * DenseMatrix.eye[Double](Xmat.rows) * Xmat)
    val sXY = trace(XmatClosest.t * DenseMatrix.eye[Double](XmatClosest.rows) * newYhat)
    val sYY = trace(newYhat.t * DenseMatrix.eye[Double](newYhat.rows) * newYhat)
    val sC = pars.sigma2 * sigma2bar

    val newSigma2 = (sXX - 2 * sXY + sYY + sC) / (N * dim) // TODO: Should sC be added to all or included in the parenthesis as currently?

    val warpField = DiscreteField(gpmm.reference, referencePoints.toIndexedSeq.zip(newYhatPoints).map { case (a, b) => b - a })
    val newGpmmPars = gpmm.gp.coefficients(warpField)

    val newPars = BCPDParameters[_3D](
      alpha = alpha,
      sigma2 = newSigma2,
      simTrans = newTransform
    )

    (newGpmmPars, newPars)
  }
}
