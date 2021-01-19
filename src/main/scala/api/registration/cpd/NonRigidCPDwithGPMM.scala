package api.registration.cpd

import api.registration.utils.PointSequenceConverter
import breeze.linalg.{Axis, DenseMatrix, DenseVector, diag, sum, tile}
import scalismo.common.interpolation.NearestNeighborInterpolator
import scalismo.common.{DiscreteDomain, DomainWarp, Vectorizer}
import scalismo.geometry._
import scalismo.statisticalmodel.{MultivariateNormalDistribution, PointDistributionModel}

class NonRigidCPDwithGPMM[D: NDSpace, DDomain[D] <: DiscreteDomain[D]](
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

    val fit = (0 until max_iteration).foldLeft((initialGPMMpars, sigma2Init)) { (it, i) =>
      val parsInit = it._1
      val sigma2 = it._2
      println(s"CPD, iteration: ${i}/${max_iteration}, sigma2: ${sigma2}")
      val iter = Iteration(parsInit, target, sigma2)
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

  private def gaussKernel(x: Point[D], y: Point[D], sigma2: Double): Double = {
    math.exp(-(x - y).norm2 / (2.0 * sigma2))
  }

  private def Expectation(template: Seq[Point[D]], target: Seq[Point[D]], sigma2: Double): DenseMatrix[Double] = {
    val M = template.length
    val N = target.length
    // TODO: Approximate using nyström
    val P: DenseMatrix[Double] = DenseMatrix.zeros[Double](M,N)
    template.zipWithIndex.par.foreach { case (y, i) =>
      target.zipWithIndex.foreach { case (x, j) =>
        P(i, j) = gaussKernel(x, y, sigma2)
      }
    }
    val c = w / (1 - w) * math.pow((2.0 * math.Pi * sigma2), vectorizer.dim.toDouble / 2.0) * (M.toDouble / N.toDouble)
    val denRow = DenseMatrix(sum(P, Axis._0).t)
    val den = tile(denRow, M, 1) + c

    P /:/ den
  }



  def getCorrespondence(template: Seq[Point[D]], target: Seq[Point[D]], sigma2: Double): (Seq[(Point[D],MultivariateNormalDistribution)], DenseMatrix[Double]) = {
    val D = vectorizer.dim
    val P = Expectation(template, target, sigma2)
    val P1inv = 1.0/sum(P, Axis._1)

    def d2MVND(d: Double): MultivariateNormalDistribution = {
      MultivariateNormalDistribution(DenseVector.zeros[Double](D), DenseMatrix.eye[Double](D)*d)
    }

    val deform = template.zipWithIndex.par.map{case (y, i) =>
      val xscale = target.zipWithIndex.map{case (x, j) =>
        P1inv(i)*P(i,j)*x.toBreezeVector
      }
      vectorizer.unvectorize(sum(xscale)-y.toBreezeVector).toVector
    }

//    val X = DenseMatrix(target.map(p => p.toArray).toArray:_*)
//    val Yhat = DenseMatrix(template.map(p => p.toArray).toArray:_*)
//    val defo = (diag(P1.map(d=>1.0/d))*P*X-Yhat).t //.t for simpler vector creation below


//    val deform = (0 until defo.cols).map(i => defo(::,i)).map(vectorizer.unvectorize).map(_.toVector)
    val td = template.zip(deform).map{case (p,d) => p+d}
    val out = td.zipWithIndex.map{case (p,i) => (p, d2MVND(lambda*sigma2*P1inv(i)))}
    (out, P)
  }

  def Iteration(pars: DenseVector[Double], target: Seq[Point[D]], sigma2: Double): (DenseVector[Double], Double) = {
    val instance = gpmm.instance(pars)
    val templatePoints = instance.pointSet.points.toSeq

    val (closestPoints, pmat: DenseMatrix[Double]) = getCorrespondence(templatePoints, target, sigma2)

    val cp = instance.pointSet.pointIds.toSeq.zip(closestPoints).map(t => (t._1,t._2._1,t._2._2)).toIndexedSeq

    val posteriorMean = gpmm.newReference(instance, NearestNeighborInterpolator()).posterior(cp).mean
//    val posteriorMean = gpmm.posterior(cp, sigma2 * lambda).mean

    val TY = dataConverter.toMatrix(posteriorMean.pointSet.points.toSeq)
    val X = dataConverter.toMatrix(target)
    val newSigma2 = computeSigma2(TY, X, pmat)

    (gpmm.coefficients(posteriorMean), newSigma2)
  }
}