package api.registration.cpd

import breeze.linalg.{Axis, DenseMatrix, DenseVector, sum, tile}
import breeze.numerics.pow
import scalismo.common.interpolation.NearestNeighborInterpolator
import scalismo.common.{DiscreteDomain, DomainWarp, PointId, Vectorizer}
import scalismo.geometry._
import scalismo.statisticalmodel.PointDistributionModel

// Direct implementation of CPD with loops over points instead of matrix multiplications + GPMM instead of using the G matrix

class NonRigidCPDwithGPMM[D: NDSpace, DDomain[D] <: DiscreteDomain[D]](
                                                                        val gpmm: PointDistributionModel[D, DDomain],
                                                                        val target: DDomain[D],
                                                                        val lambda: Double,
                                                                        val w: Double,
                                                                        val max_iteration: Int,
                                                                      )(
                                                                        implicit val vectorizer: Vectorizer[Point[D]],
                                                                        domainWarper: DomainWarp[D, DDomain]
                                                                      ) {
  println("Clean version")
  require(lambda > 0)
  require(0 <= w && w < 1.0)
  private val N = target.pointSet.numberOfPoints
  private val M = gpmm.reference.pointSet.numberOfPoints
  private val D = vectorizer.dim

  private val tolerance = 0.001

  private val initialPars = DenseVector.zeros[Double](gpmm.rank)

  val G: DenseMatrix[Double] = initializeKernelMatrixG(gpmm.reference.pointSet.points.toIndexedSeq, 50)


  private def initializeKernelMatrixG(
                                       points: Seq[Point[D]],
                                       beta: Double
                                     ): DenseMatrix[Double] = {
    val M = points.length
    val G: DenseMatrix[Double] = DenseMatrix.zeros[Double](M, M)
    (0 until M).map { i =>
      (0 until M).map { j =>
        G(i, j) = math.exp(-1 / (math.pow(beta, 2)) * (points(i) - points(j)).norm2)
      }
    }
    G
  }

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

  def getCorrespondence(template: DDomain[D], target: DDomain[D], sigma2: Double): (Seq[(PointId, Point[D], Double)], Double) = {
    // TODO: avoid spanning the full p matrix
    val tmpPoints = template.pointSet.points.toSeq
    val tarPoints = target.pointSet.points.toSeq

    val Punscaled = DenseMatrix.zeros[Double](M,N)
    tmpPoints.indices.foreach { m =>
      val pm = tmpPoints(m)
      tarPoints.indices.foreach { n =>
        val pn = tarPoints(n)
        Punscaled(m,n) = probability(pm, pn, sigma2)
      }
    }
    val denRow = DenseMatrix(sum(Punscaled, Axis._0).t)
    val den = tile(denRow, M, 1) //+ c

    val P = Punscaled /:/ den

    val P1 = sum(P, Axis._1)
    val Pt1 = sum(P, Axis._0).t

    val W: Seq[DenseVector[Double]] = (0 until template.pointSet.numberOfPoints).map { m =>
      val vec = (0 until target.pointSet.numberOfPoints).map { n =>
        ((target.pointSet.points.toIndexedSeq(n).toBreezeVector - template.pointSet.points.toIndexedSeq(m).toBreezeVector).*(P(m, n)))
      }
      sum(vec)/(sigma2*lambda)
    }

    // Using approximated GPMM
    val GW: Seq[DenseVector[Double]] = (0 until template.pointSet.numberOfPoints).map{ m=>
      val res = (0 until template.pointSet.numberOfPoints).map { minner =>
        gpmm.gp.cov(PointId(m), PointId(minner))*W(minner)
      }
      sum(res)
    }
    // Using G matrix
//    val GW: Seq[DenseVector[Double]] = (0 until template.pointSet.numberOfPoints).map{ m=>
//      val res = (0 until template.pointSet.numberOfPoints).map { minner =>
//        G(m, minner) * W(minner)
//      }
//      sum(res)
//    }

    val deform: Seq[Point[D]] = template.pointSet.points.toSeq.zip(GW).map{case (p,d) => p+vectorizer.unvectorize(d).toVector}

    val targetPoints = target.pointSet.points.toSeq
    val varX = sum(variance(targetPoints, Pt1))
    val varY = sum(variance(deform, P1))
    val varMix = sum(varianceMix(deform, targetPoints, P))

    val Np = sum(P1)

    val newSigma2 = (varX-2*varMix+varY)/(Np*D)

    val out = template.pointSet.pointIds.toSeq.zip(deform).map{case (id, p) => (id, p, 1.0)}
    (out, newSigma2)
  }

  def Iteration(pars: DenseVector[Double], target: DDomain[D], sigma2: Double): (DenseVector[Double], Double) = {
    val instance = gpmm.instance(pars)

    val (cpinfo, newSigma2) = getCorrespondence(instance, target, sigma2)
    val cp = cpinfo.filter(_._3 == 1.0).map(f => (f._1, f._2)).toIndexedSeq


    val posteriorLMnoise = 1e-15

    val posteriorMean = gpmm.newReference(instance, NearestNeighborInterpolator()).posterior(cp, posteriorLMnoise).mean
//    val posteriorMean = gpmm.posterior(cp, posteriorLMnoise).mean
    (gpmm.coefficients(posteriorMean), newSigma2)
  }
}