package api.registration.cpd

import api.registration.utils.PointSequenceConverter
import breeze.linalg.{*, Axis, DenseMatrix, DenseVector, diag, norm, sum, tile}
import breeze.numerics.pow
import scalismo.common.interpolation.NearestNeighborInterpolator
import scalismo.common.{DiscreteDomain, DomainWarp, Vectorizer}
import scalismo.geometry._
import scalismo.statisticalmodel.{MultivariateNormalDistribution, PointDistributionModel}

class NonRigidCPDwithGPMMmatrix[D: NDSpace, DDomain[D] <: DiscreteDomain[D]](
                                                                        val gpmm: PointDistributionModel[D, DDomain],
                                                                        val lambda: Double,
                                                                        val w: Double,
                                                                        val max_iteration: Int,
                                                                      )(
                                                                        implicit val vectorizer: Vectorizer[Point[D]],
                                                                        domainWarper: DomainWarp[D, DDomain],
                                                                        dataConverter: PointSequenceConverter[D]
                                                                      ) {
  println("New version using GPMM as G and optimizing loops")
  require(lambda > 0)
  require(0 <= w && w < 1.0)
  private val initialPars = DenseVector.zeros[Double](gpmm.rank)

  def Registration(target: Seq[Point[D]], tolerance: Double, initialGPMMpars: DenseVector[Double] = initialPars): DenseVector[Double] = {
    val instance = gpmm.instance(initialGPMMpars)
    val sigma2Init = computeInitialSigma2(instance.pointSet.points.toSeq, target)
    val X = dataConverter.toMatrix(target)

    val fit = (0 until max_iteration).foldLeft((initialGPMMpars, sigma2Init)) { (it, i) =>
      val parsInit = it._1
      val sigma2 = it._2
      println(s"CPD, iteration: ${i}/${max_iteration}, sigma2: ${sigma2}")
      val iter = Iteration(parsInit, X, sigma2)
      val pars = iter._1
      val sigmaDiff = sigma2- iter._2
      if (sigmaDiff < tolerance) {
        println(s"Converged")
        return pars
      } else {
        iter
      }
    }
    fit._1
  }

  private def computeInitialSigma2(template: Seq[Point[D]], target: Seq[Point[D]]): Double = {
    val N = target.length
    val M = template.length
    val sumDist = template.flatMap { pm =>
      target.map { pn =>
        (pn - pm).norm2
      }
    }.sum
    sumDist / (vectorizer.dim * N * M)
  }

  private def computeSigma2(X: DenseMatrix[Double], TY: DenseMatrix[Double], P: DenseMatrix[Double]): Double = {
    val P1 = sum(P, Axis._1)
    val Pt1 = sum(P, Axis._0)
    val Np = sum(P1)

    val xPx: Double = Pt1.t dot sum(TY *:* TY, Axis._1)
    val yPy: Double = P1.t * sum(X *:* X, Axis._1)
    val trPXY: Double = sum(X *:* (P * TY))
    (xPx - 2 * trPXY + yPy) / (Np * vectorizer.dim)
  }

  private def gaussKernel(x: DenseVector[Double], y: DenseVector[Double], sigma2: Double): Double = {
    math.exp(- math.pow(norm(x-y),2) / (2.0 * sigma2))
  }

  private def Expectation(template: DenseMatrix[Double], target: DenseMatrix[Double], sigma2: Double): DenseMatrix[Double] = {
    val M = template.rows
    val N = target.rows
    val dim = vectorizer.dim
    // TODO: Approximate using nyström
    // TODO: Do matrix substraction with broadcasting instead
    val P: DenseMatrix[Double] = template(*, ::).map { x =>
      val vec = target(*, ::).map { y =>
        gaussKernel(x, y, sigma2)
      }
      vec
    }
    val c = w / (1 - w) * math.pow((2.0 * math.Pi * sigma2), dim.toDouble / 2.0) * (M.toDouble / N.toDouble)
    val denRow = DenseMatrix(sum(P, Axis._0).t)
    val den = tile(denRow, M, 1) + c

    P /:/ den
  }

  def getCorrespondence(Y: DenseMatrix[Double], X: DenseMatrix[Double], sigma2: Double): (Seq[(Point[D],MultivariateNormalDistribution)], DenseMatrix[Double]) = {
    val D = vectorizer.dim

    val P = Expectation(Y, X, sigma2)

    val P1 = sum(P, Axis._1)
    val P1inv = 1.0/P1

    def d2MVND(d: Double): MultivariateNormalDistribution = {
      MultivariateNormalDistribution(DenseVector.zeros[Double](D), DenseMatrix.eye[Double](D)*d)
    }

    val P1invPXY = diag(P1inv)*P*X-Y
//    val deform = (0 until defo.cols).map(i => defo(::,i)).map(vectorizer.unvectorize).map(_.toVector)
    val deform = Y+P1invPXY

    val deformPoints = deform(*,::).map(d => vectorizer.unvectorize(d)).toArray.toSeq
    val deformNoise = (P1inv*lambda*sigma2).toArray.toSeq.map(d => d2MVND(d))
    val out = deformPoints zip deformNoise
    (out, P)
  }

  def Iteration(pars: DenseVector[Double], target: DenseMatrix[Double], sigma2: Double): (DenseVector[Double], Double) = {
    val instance = gpmm.instance(pars)

    val template = dataConverter.toMatrix(instance.pointSet.points.toSeq)

    val (closestPoints, pmat: DenseMatrix[Double]) = getCorrespondence(template, target, sigma2)
    val cp = instance.pointSet.pointIds.toSeq.zip(closestPoints).map(t => (t._1,t._2._1,t._2._2)).toIndexedSeq

    val posteriorMean = gpmm.newReference(instance, NearestNeighborInterpolator()).posterior(cp).mean
//    val posteriorMean = gpmm.posterior(cp, sigma2 * lambda).mean

    val TY = dataConverter.toMatrix(posteriorMean.pointSet.points.toSeq)
    val newSigma2 = computeSigma2(TY, target, pmat)

    (gpmm.coefficients(posteriorMean), newSigma2)
  }
}