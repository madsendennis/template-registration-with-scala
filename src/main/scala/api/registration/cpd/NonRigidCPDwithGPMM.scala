package api.registration.cpd

import api.registration.utils.{ClosestPointRegistrator, ModelViewerHelper, PointSequenceConverter, modelViewer}
import breeze.linalg.{Axis, DenseMatrix, DenseVector, sum, tile}
import scalismo.common.interpolation.NearestNeighborInterpolator
import scalismo.common.{DiscreteDomain, DomainWarp, PointId, Vectorizer}
import scalismo.geometry._
import scalismo.statisticalmodel.{MultivariateNormalDistribution, PointDistributionModel}

class NonRigidCPDwithGPMM[D: NDSpace, DDomain[D] <: DiscreteDomain[D]](
                                                                     val gpmm: PointDistributionModel[D, DDomain],
                                                                     val targetMesh: DDomain[D],
                                                                     val gpmmLMs: Seq[Landmark[D]],
                                                                     val targetLMs: Seq[Landmark[D]],
                                                                     val lambda: Double,
                                                                     val w: Double,
                                                                     val max_iteration: Int,
                                                                     val modelView: Option[modelViewer]
                                                                   )(
                                                                        implicit val vectorizer: Vectorizer[Point[D]],
                                                                        domainWarper: DomainWarp[D, DDomain],
                                                                        dataConverter: PointSequenceConverter[D],
                                                                        closestPointRegistrator: ClosestPointRegistrator[D, DDomain]
                                                                   ) {
  require(lambda > 0)
  require(0 <= w && w < 1.0)


  private val commonLmNames = gpmmLMs.map(_.id) intersect targetLMs.map(_.id)
  val lmIdsOnReference: Seq[(PointId, MultivariateNormalDistribution)] = commonLmNames.map(name => gpmmLMs.find(_.id == name).get).map(lm => (gpmm.reference.pointSet.findClosestPoint(lm.point).id, lm.uncertainty.get))
  val lmPointsOnTarget: Seq[Point[D]] = commonLmNames.map(name => targetLMs.find(_.id == name).get).map(lm => targetMesh.pointSet.findClosestPoint(lm.point).point)

  val lmCP = lmIdsOnReference.zip(lmPointsOnTarget).map{case(id, p) => (id._1, p, id._2)}


  def Registration(tolerance: Double, initialGPMMpars: DenseVector[Double], initialSigma2: Double): DenseVector[Double] = {
    val target = targetMesh.pointSet.points.toSeq
    val instance = gpmm.instance(initialGPMMpars)

    val sigma2Init = if(initialSigma2 == Double.PositiveInfinity) computeInitialSigma2(instance.pointSet.points.toSeq, target) else initialSigma2

    val fit = (0 until max_iteration).foldLeft((initialGPMMpars, sigma2Init)) { (it, i) =>
      val parsInit = it._1
      val sigma2 = it._2
      println(s"CPD, iteration: ${i}/${max_iteration}, sigma2: ${sigma2}")
      val iter = Iteration(parsInit, target, sigma2)
      if (modelView.nonEmpty) {
        if (i % modelView.get.updateFrequency == 0) {
          modelView.get.modelView.shapeTransformationView.coefficients = iter._1
        }
      }
      val pars = iter._1
      val sigmaDiff = sigma2 - iter._2
      if (sigmaDiff < tolerance) {
        println(s"Converged")
        return pars
      } else {
        iter
      }
    }
    fit._1
  }

  def computeInitialSigma2(reference: Seq[Point[D]], target: Seq[Point[D]]): Double = {
    val N = target.length
    val M = reference.length
    val sumDist = reference.flatMap { pm =>
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

  private def Expectation(reference: Seq[Point[D]], target: Seq[Point[D]], sigma2: Double): DenseMatrix[Double] = {
    val M = reference.length
    val N = target.length
    // TODO: Approximate using nyström
    val P: DenseMatrix[Double] = DenseMatrix.zeros[Double](M, N)
    reference.zipWithIndex.par.foreach { case (y, i) =>
      target.zipWithIndex.foreach { case (x, j) =>
        P(i, j) = gaussKernel(x, y, sigma2)
      }
    }
    val c = w / (1 - w) * math.pow((2.0 * math.Pi * sigma2), vectorizer.dim.toDouble / 2.0) * (M.toDouble / N.toDouble)
    val denRow = DenseMatrix(sum(P, Axis._0).t)
    val den = tile(denRow, M, 1) + c

    P /:/ den
  }


  def getCorrespondence(reference: Seq[Point[D]], target: Seq[Point[D]], sigma2: Double): (Seq[(Point[D], MultivariateNormalDistribution)], DenseMatrix[Double]) = {
    val D = vectorizer.dim
    val P = Expectation(reference, target, sigma2)
    val P1inv = 1.0 / sum(P, Axis._1)

    def d2MVND(d: Double): MultivariateNormalDistribution = {
      MultivariateNormalDistribution(DenseVector.zeros[Double](D), DenseMatrix.eye[Double](D) * d)
    }

    val deform = reference.zipWithIndex.par.map { case (y, i) =>
      val xscale = target.zipWithIndex.map { case (x, j) =>
        P1inv(i) * P(i, j) * x.toBreezeVector
      }
      vectorizer.unvectorize(sum(xscale) - y.toBreezeVector).toVector
    }
    val td = reference.zip(deform).map { case (p, d) => p + d }
    val out = td.zipWithIndex.map { case (p, i) => (p, d2MVND(lambda * sigma2 * P1inv(i))) }
    (out, P)
  }


  def Iteration(pars: DenseVector[Double], target: Seq[Point[D]], sigma2: Double): (DenseVector[Double], Double) = {
    val instance = gpmm.instance(pars)
    val referencePoints = instance.pointSet.points.toSeq

    val (closestPoints, pmat: DenseMatrix[Double]) = getCorrespondence(referencePoints, target, sigma2)
    val corr = instance.pointSet.pointIds.toSeq.zip(closestPoints).map(t => (t._1, t._2._1, t._2._2)).toIndexedSeq ++ lmCP

    val cpInfo = closestPointRegistrator.closestPointCorrespondence(instance, targetMesh)._1.map(_._3)
    val corrFiltered = corr.zip(cpInfo).filter(f => f._2==1.0).map(_._1)  // Remove points from training data where nearest point is on boundary

    val posteriorMean = gpmm.posterior(corrFiltered).mean

    //    val posteriorMean = gpmm.posterior(cp, sigma2 * lambda).mean

    val TY = dataConverter.toMatrix(posteriorMean.pointSet.points.toSeq)
    val X = dataConverter.toMatrix(target)
    val newSigma2 = computeSigma2(TY, X, pmat)

    (gpmm.coefficients(posteriorMean), newSigma2)
  }
}