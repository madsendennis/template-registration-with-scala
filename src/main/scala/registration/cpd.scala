package registration

import java.awt.Color
import java.io.File

import breeze.linalg.{Axis, DenseMatrix, DenseVector, diag, sum, tile}
import breeze.numerics.abs
import scalismo.common._
import scalismo.geometry.{Point, Point3D, _3D}
import scalismo.io.MeshIO
import scalismo.mesh.TriangleMesh3D
import scalismo.ui.api.ScalismoUI

/*
 Implementation of Point Set Registration: Coherent Point Drift
 In this script, only the non-rigid algorithm is implemented. Paper: https://arxiv.org/pdf/0905.2635.pdf
 A python implementation already exists from where parts of the implementation is from: https://github.com/siavashk/pycpd
 */


// Dummy case class to easily convert between point representation and matrix representation.
// TODO: Find a better way to easily convert between representations
case class myMesh(pd: DiscreteDomain[_3D]) {
  var points: UnstructuredPoints3D = UnstructuredPoints.Create.CreateUnstructuredPoints3D.create(pd.pointSet.points.toIndexedSeq)
  var mat: DenseMatrix[Double] = PointCloud3DtoMatrix(points)

  def PointCloud3DtoMatrix(pd: UnstructuredPoints3D): DenseMatrix[Double] = {
    val m = DenseMatrix.zeros[Double](pd.numberOfPoints, 3)
    pd.points.zipWithIndex.foreach { case (p, i) =>
      m(i, 0) = p.x
      m(i, 1) = p.y
      m(i, 2) = p.z
    }
    m
  }

  def denseMatrixToPointDomain(dm: DenseMatrix[Double]): UnstructuredPointsDomain[_3D] = {
    val p: IndexedSeq[Point[_3D]] = 0 until dm.rows map { r => Point3D(x = dm(r, 0), y = dm(r, 1), z = dm(r, 2)) }
    UnstructuredPointsDomain.Create.CreateUnstructuredPointsDomain3D.create(p)
  }

  def getPointsDomain(): UnstructuredPointsDomain[_3D] = {
    UnstructuredPointsDomain.Create.CreateUnstructuredPointsDomain3D.create(points.points.toIndexedSeq)
  }
}


case class cpd(targetPoints: DiscreteDomain[_3D], sourcePoints: DiscreteDomain[_3D],
               lamdba: Double = 2, beta: Double = 2, w: Double = 0) {
  // Paper: sigma2=3, lambda=1, beta=1
  // w: 0 until 1 - uniform distribution to account for outliers
  val M: Int = sourcePoints.pointSet.numberOfPoints // num of source points
  val N: Int = targetPoints.pointSet.numberOfPoints // num of target points
  val D: Int = 3 // dimension

  // Matrix representation of data points
  val source: myMesh = myMesh(sourcePoints) // Y as in paper
  val target: myMesh = myMesh(targetPoints) // X as in paper

  val G: DenseMatrix[Double] = initializeKernelMatrixG(sourcePoints, beta)
  var sigma2: Double = initializeGaussianKernel(sourcePoints, targetPoints)
  var W: DenseMatrix[Double] = DenseMatrix.zeros[Double](M, N)
  var P: DenseMatrix[Double] = DenseMatrix.zeros[Double](M, N)


  /*
  Initialize G matrix - formula in paper fig. 4
 */
  private def initializeKernelMatrixG(Y: DiscreteDomain[_3D], beta: Double): DenseMatrix[Double] = {
    val G: DenseMatrix[Double] = DenseMatrix.zeros[Double](M, M)
    (0 until M).map { i =>
      (0 until M).map { j =>
        G(i, j) = math.exp(-1 / (2 * math.pow(beta, 2)) * (Y.pointSet.point(PointId(i)) - Y.pointSet.point(PointId(j))).norm2)
      }
    }
    G
  }

  /*
  Initialize sigma2 - formula in paper fig. 4
   */
  private def initializeGaussianKernel(Y: DiscreteDomain[_3D], X: DiscreteDomain[_3D]): Double = {
    val s: Double = (0 until M).flatMap { m =>
      (0 until N).map { n =>
        (Y.pointSet.point(PointId(m)) - X.pointSet.point(PointId(n))).norm2
      }
    }.sum
    s / (D * N * M)
  }

  def registration(max_iteration: Int = 10, tolerance: Double = 0.001): myMesh = {

    (0 until max_iteration).foldLeft(source) { (Y, i) =>
      val currentSigma2 = sigma2
      println(s"CPD, iteration: ${i}, variance: ${sigma2}")
      val TY = iteration(target, Y)
      val diff = abs(sigma2 - currentSigma2)
      if (diff < tolerance) {
        println("Converged")
        return TY
      }
      else {
        TY
      }
    }
  }

  /*
  1 iteration of the EM algorithm
  */
  def iteration(X: myMesh, Y: myMesh): myMesh = {
    val P = expectation(X, Y)
    val TY = maximization(X: myMesh, Y: myMesh, P)
    TY
  }

  /*
  Expectation computation
   - Compute the P matrix which contains correspondence probabilities for each point pair
   - formula in paper fig. 4. Matrix implementation help from pycpd
  */
  def expectation(X: myMesh, Y: myMesh): DenseMatrix[Double] = {
    (0 until M).map { m =>
      (0 until N).map { n =>
        P(m, n) = math.exp(-1 / (2 * sigma2) * (X.points.point(PointId(n)) - Y.points.point(PointId(m))).norm2)
      }
    }
    val c = w / (1 - w) * math.pow((2 * math.Pi * sigma2), D / 2) * (M / N)
    val denRow = DenseMatrix(sum(P, Axis._0).t)
    val den = tile(denRow, M, 1) + c

    P = P /:/ den
    P
  }

  /*
  Maximize the expectation by updating the source point cloud
   - The general steps are mentioned in paper fig. 4. Section 6 gives explanation on the exact implementation of e.g. P1.
   - The update of sigma2 "variance update" is from the pycpd librarie
  */
  def maximization(X: myMesh, Y: myMesh, P: DenseMatrix[Double]): myMesh = {
    // Update transform
    val P1: DenseVector[Double] = sum(P, Axis._1)
    val Pt1 = sum(P, Axis._0)
    val Np = sum(P1)

    val A: DenseMatrix[Double] = diag(P1) * G + lamdba * sigma2 * DenseMatrix.eye[Double](M)
    val B: DenseMatrix[Double] = P * X.mat - diag(P1) * Y.mat

    W = A \ B
    // Update Point Cloud
    val TY = Y.mat + G * W

    // Update variance
    /*
        Update the variance of the mixture model using the new estimate of the deformable transformation.
        See the update rule for sigma2 in Eq. 23 of of https://arxiv.org/pdf/0905.2635.pdf.
    */
    // The original CPD paper does not explicitly calculate the objective functional.
    // This functional will include terms from both the negative log-likelihood and
    // the Gaussian kernel used for regularization.
    val xPx: Double = Pt1.t dot sum(X.mat *:* X.mat, Axis._1)
    val yPy: Double = P1.t * sum(TY *:* TY, Axis._1)

    val trPXY: Double = sum(TY *:* (P * X.mat))

    sigma2 = (xPx - 2 * trPXY + yPy) / (Np * D)

    myMesh(Y.denseMatrixToPointDomain(TY))
  }
}


object cpd {

  def fixTriangulation(source: TriangleMesh3D, fit: UnstructuredPoints3D) = {
    TriangleMesh3D(fit, source.triangulation)
  }

  def fishFun() = {
    val target = MeshIO.readMesh(new File("data/fish0.ply")).get
    val targetPoints = UnstructuredPointsDomain.Create.CreateUnstructuredPointsDomain3D.create(target.pointSet.points.toIndexedSeq)
    val source = MeshIO.readMesh(new File("data/fish1.ply")).get
    val sourcePoints = UnstructuredPointsDomain.Create.CreateUnstructuredPointsDomain3D.create(source.pointSet.points.toIndexedSeq)
    val cp = cpd(target, sourcePoints, lamdba = 2, beta = 2, w = 0.0) // target, source ... Source is moving!!!
    val finalReg = cp.registration(max_iteration = 20)

    val ui = ScalismoUI()
    val showTarget = ui.show(targetPoints, "target")
    val showSource = ui.show(sourcePoints, "source")
    val showFinal = ui.show(finalReg.getPointsDomain(), "final")
    showTarget.radius = 0.1
    showSource.radius = 0.1
    showFinal.radius = 0.1
    showTarget.color = Color.GREEN
    showSource.color = Color.RED
  }

  def femurFun() = {
    // low resolution aligned femur meshes ~100 vertices
    val source = MeshIO.readMesh(new File("data/femur0_coarse.stl")).get
    val target = MeshIO.readMesh(new File("data/femur1_coarse.stl")).get

    // "high" resolution femur meshes ~1600 vertices
    //    val source = MeshIO.readMesh(new File("data/femur_reference.stl")).get
    //    val target = MeshIO.readMesh(new File("data/femur_target.stl")).get
    //    val target = MeshIO.readMesh(new File("data/femur_target_partial.stl")).get
    //    val target = MeshIO.readMesh(new File("data/femur_target_rigid.stl")).get
    //    val target = MeshIO.readMesh(new File("data/femur_target_rigid_rotate.stl")).get


    println(s"CPD Femur fun - Source points: ${source.pointSet.numberOfPoints}, target points: ${target.pointSet.numberOfPoints}")

    // lambda and beta both reflect the amount of smoothness regularization
    // beta = width of gaussian filter
    // lambda = trade-off between goodness of fit and regularization
    val cp = cpd(target, source, lamdba = 10, beta = 100, w = 0.0) // target, source ... Source is moving!!!

    val finalReg = cp.registration(max_iteration = 1000, tolerance = 0.001)
    val finalMesh = fixTriangulation(source, finalReg.points)
    val ui = ScalismoUI()
    val showTarget = ui.show(target, "target")
    val showSource = ui.show(source, "source")
    val showFinal = ui.show(finalMesh, "final")
    showTarget.color = Color.GREEN
    showSource.color = Color.RED
  }


  def main(args: Array[String]): Unit = {
    scalismo.initialize()

    fishFun()
    femurFun()
  }
}
