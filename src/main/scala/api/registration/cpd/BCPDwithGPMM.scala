package api.registration.cpd

import api.registration.utils.{PointSequenceConverter, SimilarityTransformParameters, TransformationHelper}
import breeze.linalg.{Axis, DenseMatrix, DenseVector, diag, kron, pinv, sum, tile, trace}
import breeze.numerics.digamma
import scalismo.common._
import scalismo.geometry._
import scalismo.statisticalmodel.{MultivariateNormalDistribution, PointDistributionModel}

// Similarity transformation parameters
case class BCPDParameters[D](Sigma: DenseMatrix[Double], alpha: DenseVector[Double], simTrans: SimilarityTransformParameters[D])

class BCPDwithGPMM[D: NDSpace, DDomain[D] <: DiscreteDomain[D]](
                                                                 val gpmm: PointDistributionModel[D, DDomain],
                                                                 val targetPoints: Seq[Point[D]],
                                                                 val w: Double, // Outlier, [0,1]
                                                                 val lambda: Double, // Noise scaling, R+
                                                                 val gamma: Double, // Initial noise scaling, R+
                                                                 val k: Double,
                                                                 val max_iterations: Int
                                                               )(
                                                                 implicit val vectorizer: Vectorizer[Point[D]],
                                                                 domainWarper: DomainWarp[D, DDomain],
                                                                 dataConverter: PointSequenceConverter[D],
                                                                 simTrans: TransformationHelper[D]
                                                               ) {
  println("New version using GPMM and BCPD update method")
  require(lambda > 0)
  require(gamma > 0)
  require(k > 0)
  require(0 <= w && w < 1.0)

  val dim: Int = vectorizer.dim
  val referencePoints: Seq[Point[D]] = gpmm.reference.pointSet.points.toSeq
  val M: Int = referencePoints.length
  val N: Int = targetPoints.length

  def Registration(tolerance: Double, initialGPMMpars: DenseVector[Double], initialTransformation: SimilarityTransformParameters[D]): (DenseVector[Double], SimilarityTransformParameters[D]) = {
    val instance = gpmm.instance(initialGPMMpars) //TODO: Add similarityTransform
    val sigma2Init = gamma * computeSigma2init(instance.pointSet.points.toSeq, targetPoints)

    val parsInit = BCPDParameters[D](
      Sigma = DenseMatrix.eye[Double](M),
      alpha = DenseVector.ones[Double](M) / M.toDouble,
      simTrans = initialTransformation
    )

    val fit = (0 until max_iterations).foldLeft((initialGPMMpars, sigma2Init, parsInit)) { (it, i) =>
      val gpmmParsInit = it._1
      val sigma2 = it._2
      val pars = it._3
      println(s"BCPD, iteration: ${i}/${max_iterations}, sigma2: ${sigma2}")
      val iter = Iteration(gpmmParsInit, sigma2, pars)
      val sigmaDiff = math.abs(iter._2 - sigma2)
      if (sigmaDiff < tolerance) {
        println(s"Converged")
        return (iter._1, iter._3.simTrans)
      } else {
        iter
      }
    }
    (fit._1, fit._3.simTrans)
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


  def computeCorrespondenceProbability(movingRefPoints: Seq[Point[D]], sigma2: Double, s: Double, Sigma: DenseMatrix[Double], alpha: DenseVector[Double]): DenseMatrix[Double] = {
    val Phi = DenseMatrix.zeros[Double](M, N)
    (0 until M).map { m =>
      val mvnd = MultivariateNormalDistribution(vectorizer.vectorize(movingRefPoints(m)), DenseMatrix.eye[Double](dim) * sigma2)
      val e = math.exp(-s / (2 * sigma2) * trace(Sigma(m, m) * DenseMatrix.eye[Double](dim)))
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

  // RotationAfterScalingAfterTranslation
  private def vectorInvTransform(v: DenseVector[Double], pars: SimilarityTransformParameters[D]): DenseVector[Double] = {
    val sinv = 1.0 / pars.s.s
    (kron(DenseMatrix.eye[Double](M), pinv(pars.R.rotationMatrix.toBreezeMatrix))) *
      (sinv * (v + (kron(DenseVector.ones[Double](M).toDenseMatrix, pars.t.t.toBreezeVector.toDenseMatrix * (-1.0)).toDenseVector)))
  }

  def Iteration(gpmmPars: DenseVector[Double], sigma2: Double, pars: BCPDParameters[D]): (DenseVector[Double], Double, BCPDParameters[D]) = {
    val instancePoints = pars.simTrans.transform(gpmm.instance(gpmmPars).pointSet.points.toSeq)
    val P = computeCorrespondenceProbability(instancePoints, sigma2, pars.simTrans.s.s, pars.Sigma, pars.alpha)

    // ********** ********** ********** ********** ********** ********** ********** ********** ********** **********
    // ********** ********** ********** ********** ********** ********** ********** ********** ********** **********
    // ********** ********** ********** ********** ********** ********** ********** ********** ********** **********
    val X: DenseVector[Double] = dataConverter.toVector(targetPoints)
    val Xmat = dataConverter.toMatrix(targetPoints)
    val Y: DenseVector[Double] = dataConverter.toVector(referencePoints)
    // Helper unit matrices
    val D1Vec: DenseMatrix[Double] = DenseVector.ones[Double](dim).toDenseMatrix
    val Dmat: DenseMatrix[Double] = DenseMatrix.eye[Double](dim)

    val v = sum(P, Axis._1) // R^M Estimated number of target points matched with each source point
    val v_ = sum(P, Axis._0).t.copy // R^N Posterior probability that x_n is a non-outlier
    val Nhat = sum(v_)

    val Pkron = kron(P, Dmat)
    val xhat = pinv(diag(v)) * P * Xmat
    val xhatPoints = dataConverter.toPointSequence(xhat)

    val xhatTinv = pars.simTrans.invTransform(xhatPoints)

    // Update Local deformations
    val sigma2DIVs2 = sigma2 / math.pow(pars.simTrans.s.s, 2)

    val gpTrainingData = (0 until M).map { i =>
      (PointId(i), xhatTinv(i), MultivariateNormalDistribution(DenseVector.zeros[Double](dim), DenseMatrix.eye[Double](dim) * lambda * sigma2DIVs2 / v(i)))
    }
    val gpmmRegression = gpmm.posterior(gpTrainingData)
    val myVhat = gpmmRegression.gp.meanVector

    val uhat = Y + myVhat

    val alpha = v.map(f => math.exp(digamma(k + f) - digamma(k * M + Nhat)))

    // Update Similarity transform
    val sigma2bar = sum((0 until M).map(m => v(m))) / Nhat

    val uhatPoints = dataConverter.toPointSequence(uhat)
    val newTransform = simTrans.getSimilarityTransform(uhatPoints, xhatPoints)

    val newYhatPoints = dataConverter.toPointSequence(Y + myVhat)
    val newYhat = dataConverter.toVector(pars.simTrans.transform(newYhatPoints))

    val vkron = kron(v.toDenseMatrix, D1Vec).toDenseVector
    val v_kron = kron(v_.toDenseMatrix, D1Vec).toDenseVector
    val sXX = X.t * diag(v_kron) * X
    val sXY = X.t * Pkron.t * newYhat
    val sYY = newYhat.t * diag(vkron) * newYhat
    val sC = sigma2 * sigma2bar

    val newSigma2 = (sXX - 2 * sXY + sYY + sC) / (Nhat * dim) // TODO: Should sC be added to all or included in the parenthesis as currently?

    // ********** ********** ********** ********** ********** ********** ********** ********** ********** **********
    // ********** ********** ********** ********** ********** ********** ********** ********** ********** **********
    // ********** ********** ********** ********** ********** ********** ********** ********** ********** **********
    val warpField = DiscreteField(gpmm.reference, referencePoints.toIndexedSeq.zip(newYhatPoints).map { case (a, b) => b - a })
    val newPointsAsDomain = domainWarper.transformWithField(gpmm.reference, warpField)
    val newGpmmPars = gpmm.coefficients(newPointsAsDomain)

    val newPars = BCPDParameters[D](
      Sigma = pars.Sigma,
      alpha = alpha,
      simTrans = newTransform
    )

    (newGpmmPars, newSigma2, newPars)
  }
}