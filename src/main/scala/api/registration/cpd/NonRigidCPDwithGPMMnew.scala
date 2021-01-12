package api.registration.cpd

import breeze.linalg.{Axis, DenseMatrix, DenseVector, InjectNumericOps, diag, sum}
import breeze.numerics.pow
import scalismo.common.{DiscreteDomain, PointId, Vectorizer}
import scalismo.geometry._
import scalismo.statisticalmodel.{MultivariateNormalDistribution, PointDistributionModel}

class NonRigidCPDwithGPMMnew[D: NDSpace, DDomain[D] <: DiscreteDomain[D]](
                                                                        val gpmm: PointDistributionModel[D, DDomain],
                                                                        val target: DDomain[D],
                                                                        val lambda: Double,
                                                                        val w: Double,
                                                                        val max_iteration: Int,
                                                                      )(
                                                                        implicit val vectorizer: Vectorizer[Point[D]]
                                                                      ) {
  println("New version...")
  require(lambda > 0)
  require(0 <= w && w < 1.0)
  private val N = target.pointSet.numberOfPoints
  private val M = gpmm.reference.pointSet.numberOfPoints
  private val D = vectorizer.dim

  private val tolerance = 0.001

  private val initialPars = DenseVector.zeros[Double](gpmm.rank)

  def Registration(): DDomain[D] = {
    val instance = gpmm.instance(initialPars)
    val sigma2Init = computeInitialSigma2(instance, target)

    val fit = (0 until max_iteration).foldLeft((initialPars, sigma2Init)) { (it, i) =>
      val parsInit = it._1
      val sigma2 = it._2
      println(s"CPD, iteration: ${i}/${max_iteration}, sigma2: ${sigma2}")
      val iter = Iteration(parsInit, target, sigma2)
      val pars = iter._1
      if (sigma2 < tolerance) {
        println(s"Converged")
        return gpmm.instance(pars)
      } else {
        iter
      }
    }
    gpmm.instance(fit._1)
  }

  def computeInitialSigma2(template: DDomain[D], target: DDomain[D]): Double = {
    val sumDist = template.pointSet.points.toIndexedSeq.flatMap { pm =>
      target.pointSet.points.toIndexedSeq.map { pn =>
        (pn - pm).norm2
      }
    }.sum
    sumDist / (D * N * M)
  }



  private def probability(x: Point[D], y: Point[D], sigma2: Double): Double = {
    math.exp(-(x - y).norm2 / (2.0 * sigma2))
  }

  private def variance(data: Seq[Point[D]], p: DenseVector[Double]): DenseVector[Double] = {
    require(data.length == p.length)
    //    sum(a.zip(b).map{case (x,y) => x.toBreezeVector *:* y.toBreezeVector})
    sum(data.indices.map{ i => p(i)* (data(i).toBreezeVector *:* data(i).toBreezeVector)})
  }

  private def varianceMix(a: Seq[Point[D]], b: Seq[Point[D]], P: DenseMatrix[Double]): DenseVector[Double] = {
    require(a.length == P.rows)
    require(b.length == P.cols)
    sum(a.indices.map{m =>
      val inner = sum(b.indices.map{n =>
        b(n).toBreezeVector * P(m,n)
      })
      a(m).toBreezeVector *:* inner
    })
  }

  def getCorrespondence(template: DDomain[D], target: DDomain[D], sigma2: Double): (Seq[(PointId, Point[D], Double)], DenseMatrix[Double]) = {
    // Need to make an estimation of W = ((G+lambda*sigma2*d(P1)^-1)^-1)*(d(P1)^-1*PX-Y)

    // TODO: avoid spanning the full p matrix
    val tmpPoints = template.pointSet.points.toSeq
    val tarPoints = target.pointSet.points.toSeq
    val P: DenseMatrix[Double] = DenseMatrix.zeros[Double](M,N)
    tmpPoints.indices.foreach { m =>
      val pm = tmpPoints(m)
      tarPoints.indices.foreach { n =>
        val pn = tarPoints(n)
        val div = template.pointSet.points.toSeq.map { minner =>
          probability(minner, pn, sigma2)
        }.sum
        P(m,n) = probability(pm, pn, sigma2) / div
      }
    }
    val P1 = sum(P, Axis._1)
    val P1mP = diag(1.0 /:/ P1)*P

    val W: Seq[DenseVector[Double]] = (0 until template.pointSet.numberOfPoints).map { m =>
      val vec = (0 until target.pointSet.numberOfPoints).map { n =>
        ((target.pointSet.points.toIndexedSeq(n).toBreezeVector - template.pointSet.points.toIndexedSeq(m).toBreezeVector).*(P(m, n)))
      }
      sum(vec)/(sigma2*lambda)
    }

    val Wspec: Seq[DenseVector[Double]] = (0 until template.pointSet.numberOfPoints).map { m =>
      val vec = (0 until target.pointSet.numberOfPoints).map { n =>
        P1mP(m, n)*:*target.pointSet.points.toIndexedSeq(n).toBreezeVector
      }
      sum(vec)-template.pointSet.points.toIndexedSeq(m).toBreezeVector
    }

    val deform: Seq[Point[D]] = template.pointSet.points.toSeq.zip(W).map{case (p,d) => p+vectorizer.unvectorize(d).toVector}

    val out = template.pointSet.pointIds.toSeq.zip(deform).map{case (id, p) => (id, p, 1.0)}
    (out, P)
  }

  def Iteration(pars: DenseVector[Double], target: DDomain[D], sigma2: Double): (DenseVector[Double], Double) = {
    def d2MVND(d: Double): MultivariateNormalDistribution = {
      MultivariateNormalDistribution(DenseVector.zeros[Double](D), DenseMatrix.eye[Double](D)*d)
    }

    val instance = gpmm.instance(pars)

    val (cpinfo, prob) = getCorrespondence(instance, target, sigma2)

    val P1 = sum(prob, Axis._1)
    val Pt1 = sum(prob, Axis._0).t

    val cp = cpinfo.zipWithIndex.filter(f => f._1._3 == 1.0).map(f => (f._1._1, f._1._2, d2MVND(1/(P1(f._2))))).toIndexedSeq

    val posteriorMean = gpmm.posterior(cp).mean

    val targetPoints = target.pointSet.points.toSeq
    val deformPoints = posteriorMean.pointSet.points.toSeq
    val varX = sum(variance(targetPoints, Pt1))
    val varY = sum(variance(deformPoints, P1))
    val varMix = sum(varianceMix(deformPoints, targetPoints, prob))

    val Np = sum(P1)

    val newSigma2 = (varX-2*varMix+varY)/(Np*D)

    (gpmm.coefficients(posteriorMean), newSigma2)
  }
}


//class NonRigidCPDwithGPMMUnstructuredPointsDomain3D(gpmm: PointDistributionModel[_3D, UnstructuredPointsDomain],
//                                                    target: UnstructuredPointsDomain[_3D]) extends NonRigidCPDwithGPMM[_3D, UnstructuredPointsDomain](gpmm, target) {
//  override def getCorrespondence(template: UnstructuredPointsDomain[_3D], target: UnstructuredPointsDomain[_3D]): (Seq[(PointId, Point[_3D], Double)], Double) = {
//    ClosestPointUnstructuredPointsDomain3D.closestPointCorrespondence(template, target)
//  }
//}