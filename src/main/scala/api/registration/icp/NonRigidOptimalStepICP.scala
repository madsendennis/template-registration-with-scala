package api.registration.icp

import api.registration.utils.{CSCHelper, PointSequenceConverter}
import breeze.linalg.{*, Axis, CSCMatrix, DenseMatrix, DenseVector, SparseVector, diag, inv, kron, pinv, sum, svd}
import breeze.numerics.sqrt
import breeze.stats.mean
import scalismo.common.{DiscreteField, DomainWarp, EuclideanSpace, Field, PointId, RealSpace, UnstructuredPoints, UnstructuredPoints3D, Vectorizer}
import scalismo.geometry._
import scalismo.kernels.{DiagonalKernel, GaussianKernel, MatrixValuedPDKernel, PDKernel}
import scalismo.mesh.{TriangleCell, TriangleMesh}
import scalismo.statisticalmodel.{GaussianProcess, MultivariateNormalDistribution}

/*
 Implementation of the paper: "Optimal Step Nonrigid ICP Algorithms for Surface Registration"
 */

case class myPDKernel[D](tmp: TriangleMesh[D], cov: DenseMatrix[Double]) extends PDKernel[D] {

  override def domain = EuclideanSpace[D]

  override def k(x: Point[D], y: Point[D]): Double = {
    val pId1 = tmp.pointSet.findClosestPoint(x).id
    val pId2 = tmp.pointSet.findClosestPoint(y).id

    val adjacent1  =  tmp.triangulation.adjacentPointsForPoint(pId1)
    val numOfNeigh = adjacent1.length
    if(pId1 == pId2) numOfNeigh else if(adjacent1.contains(pId2)) -1.0 else 0.0
  }
}

case class myPDKernelMatrix[D: NDSpace](tmp: TriangleMesh[D], preComp: DenseMatrix[Double], kernel: PDKernel[D], override val outputDim: Int)
  extends MatrixValuedPDKernel[D] {

  def k(x: Point[D], y: Point[D]): DenseMatrix[Double] = {
    val m = tmp.pointSet.findClosestPoint(x).id.id
    val n = tmp.pointSet.findClosestPoint(y).id.id
    val out = preComp(m until m+3, n until n+3).copy
    out
  }
  // k is scalar valued
  override def domain = kernel.domain
}

class NonRigidOptimalStepICP(templateMesh: TriangleMesh[_3D],
                             targetMesh: TriangleMesh[_3D],
                             templateLandmarks: Seq[Landmark[_3D]],
                             targetLandmarks: Seq[Landmark[_3D]],
                             gamma: Double = 1.0)(
                              implicit vectorizer: Vectorizer[Point[_3D]],
                              dataConverter: PointSequenceConverter[_3D],
                              warper: DomainWarp[_3D, TriangleMesh]
                            ) {
  require(gamma >= 0)
  private val m: Int = templateMesh.triangulation.triangles.length * 3 // Number of edges (3 per triangle)
  private val n: Int = templateMesh.pointSet.numberOfPoints // Number of nodes
  private val dim = vectorizer.dim

  private val edges = trianglesToEdges(templateMesh.triangulation.triangles)
  private val numOfEdges = edges.length

  private val commonLmNames = templateLandmarks.map(_.id) intersect targetLandmarks.map(_.id)
  private val lmIdsOnTemplate: Seq[PointId] = commonLmNames.map(name => templateLandmarks.find(_.id == name).get).map(lm => templateMesh.pointSet.findClosestPoint(lm.point).id)
  private val lmPointsOnTarget: Seq[Point[_3D]] = commonLmNames.map(name => targetLandmarks.find(_.id == name).get).map(lm => targetMesh.pointSet.findClosestPoint(lm.point).point)

  private val UL: CSCMatrix[Double] = CSCHelper.DenseMatrix2CSCMatrix(dataConverter.toMatrix(lmPointsOnTarget))

  private val M = InitializeMatrixM(edges)
//  private val M = InitializeMatrixMDoubles(templateMesh.triangulation.triangles.toIndexedSeq)

  private val G = diag(SparseVector(1, 1, 1, gamma))
  private val kronMG: CSCMatrix[Double] = CSCHelper.kroneckerProduct(M, G) // TODO: Need optimized version of kron
  val B1: CSCMatrix[Double] = CSCMatrix.zeros[Double](4 * numOfEdges, dim)
//  val B1: CSCMatrix[Double] = CSCMatrix.zeros[Double](4 * m, dim)

//  private val defaultAlpha: Seq[Double] = (1 to 10).scanLeft(1)((a, _) => (a * 2)).map(_.toDouble / 2).reverse // ...,8,4,2,1,0.5
  private val defaultAlpha: Seq[Double] = Seq(5.0)

  private val defaultBeta: Seq[Double] = defaultAlpha.indices.map(_ => 0.0)


  private def trianglesToEdges(triangles: IndexedSeq[TriangleCell]): IndexedSeq[(PointId, PointId)] = {
    triangles.flatMap{triangle =>
      val t = triangle.pointIds.sortBy(_.id)
      Seq((t(0), t(1)), (t(0), t(2)), (t(1), t(2)))
    }.toSet.toIndexedSeq
  }

  private def InitializeMatrixM(edges: IndexedSeq[(PointId, PointId)]): CSCMatrix[Double] = {
    val M = CSCMatrix.zeros[Double](edges.length, n)
    edges.zipWithIndex.foreach { case (t, i) =>
      val p1 = t._1.id
      val p2 = t._2.id
      M(i, p1) = 1.0
      M(i, p2) = -1.0
    }
    M
  }

  private def InitializeMatrixMDoubles(triangles: IndexedSeq[TriangleCell]): CSCMatrix[Double] = {
    val M = CSCMatrix.zeros[Double](m, n)

    templateMesh.triangulation.triangles.toIndexedSeq.zipWithIndex.foreach { case (t, i) =>
      val e11 = t.ptId1.id
      val e12 = t.ptId2.id
      M(i * 3 + 0, e11) = -1
      M(i * 3 + 0, e12) = 1
      val e21 = t.ptId2.id
      val e22 = t.ptId3.id
      M(i * 3 + 1, e21) = -1
      M(i * 3 + 1, e22) = 1
      val e31 = t.ptId3.id
      val e32 = t.ptId1.id
      M(i * 3 + 2, e31) = -1
      M(i * 3 + 2, e32) = 1
    }
    M
  }

  private def ComputeMatrixD(points: Seq[Point[_3D]]): CSCMatrix[Double] = {
    val locn = points.length
    val D: CSCMatrix[Double] = CSCMatrix.zeros[Double](locn, 4 * n) // Reference points each entry in D is a matrix (4nx3)
    (0 until locn).foreach { i =>
      (0 until dim + 1).foreach { j =>
        val index = i * 4 + j
        if (j == dim) D(i, index) = 1.0
        else D(i, index) = points(i)(j)
      }
    }
    D
  }

  def Registration(max_iteration: Int, tolerance: Double = 0.001, alpha: Seq[Double] = defaultAlpha, beta: Seq[Double] = defaultBeta): TriangleMesh[_3D] = {
    require(alpha.length == beta.length)

    val fit = (alpha zip beta).zipWithIndex.foldLeft(templateMesh) { (temp, config) =>
      val (a, b) = config._1
      val j = config._2

      val innerFit = (0 until max_iteration).foldLeft((temp, Double.PositiveInfinity)) { (it, i) =>
        val iter = Iteration(it._1, targetMesh, a, b)
        val TY = iter._1
        val newDist = iter._2
        println(s"ICP, iteration: ${j * max_iteration + i}/${max_iteration * alpha.length}, alpha: ${a}, beta: ${b}, average distance to target: ${newDist}")
        if (newDist < tolerance) {
          println("Converged")
          return TY
        } else {
          iter
        }
      }
      innerFit._1
    }
    fit
  }

  private def Iteration(template: TriangleMesh[_3D], target: TriangleMesh[_3D], alpha: Double, beta: Double): (TriangleMesh[_3D], Double) = {
    require(alpha >= 0.0)
    require(beta >= 0.0)
    val w = SparseVector.zeros[Double](n) // Weight matrix (0.0 when no correspondence, 1.0 when correspondence is found)
    val cpinfo = template.pointSet.pointIds.toIndexedSeq.zipWithIndex.map { case (id, i) =>
      val p = template.pointSet.point(id)
      val cpOnSurface = target.operations.closestPointOnSurface(p)
      val cp = target.pointSet.findClosestPoint(cpOnSurface.point)
      val intersectingPoints = template.operations.getIntersectionPoints(p, p - cpOnSurface.point).filter(f => f != p) // All intersecting points with the closest point vector
      val closestIntersectingPoint = if (intersectingPoints.nonEmpty) intersectingPoints.map(ip => (p - ip).norm).min else Double.PositiveInfinity // Distance to closest intersecting point on template
      // TODO: Restructure if/else setup
      // Use closest point for boundary and normal check as a heuristic as it is faster to check
      // Use closest point on surface for everything else
      if (target.operations.pointIsOnBoundary(cp.id)) w(i) = 1.0
      else if ((target.vertexNormals.atPoint(cp.id) dot template.vertexNormals.atPoint(id)) <= 0) w(i) = 1.0 // TODO: Introduce "angle" hyperparameter
      else if (closestIntersectingPoint < (p - cpOnSurface.point).norm) w(i) = 1.0 // TODO: Validate if check works
      else w(i) = 1.0
      cpOnSurface
    }
    val cp = cpinfo.map(_.point)
    val cpDist = sqrt(cpinfo.map(_.distanceSquared).sum / cpinfo.length)

    val W: CSCMatrix[Double] = diag(w)
    val D = ComputeMatrixD(template.pointSet.points.toSeq)

    val lmPointsOnTemplate = lmIdsOnTemplate.map(id => template.pointSet.point(id))
    val DL = ComputeMatrixD(lmPointsOnTemplate)
    val U = CSCHelper.DenseMatrix2CSCMatrix(dataConverter.toMatrix(cp))

//    // N-ICP-A
//    val A1: CSCMatrix[Double] = kronMG * alpha
//    val A2: CSCMatrix[Double] = W * D
//    val A3: CSCMatrix[Double] = DL * beta
//
//    val B2: CSCMatrix[Double] = W * U
//    val B3: CSCMatrix[Double] = UL * beta // TODO: Should beta also be multiplied to B3 ???
//
//    val A = CSCHelper.vertcat(A1, A2, A3)
//    val B = CSCHelper.vertcat(B1, B2, B3)
//    val X = (A \ B).toDenseMatrix
//
//    val updatedPoints = template.pointSet.points.toIndexedSeq.zipWithIndex.map { case (p, i) =>
//      val x: DenseMatrix[Double] = X(i * 4 until i * 4 + 4, ::).t
//      val v: DenseVector[Double] = DenseVector[Double](p(0), p(1), p(2), 1.0)
//      val xv = (x * v)
//      vectorizer.unvectorize(xv)
//    }

    // N-ICP-T
    val V = CSCHelper.DenseMatrix2CSCMatrix(dataConverter.toMatrix(template.pointSet.points.toSeq))
    val A1: CSCMatrix[Double] = M * alpha
    val A2: CSCMatrix[Double] = W*diag(SparseVector.fill(template.pointSet.numberOfPoints)(1.0))

    val B1: CSCMatrix[Double] = CSCMatrix.zeros[Double](numOfEdges, dim)
    val B2: CSCMatrix[Double] = W * (U-V)

    val A = CSCHelper.vertcat(A1, A2)
    val B = CSCHelper.vertcat(B1, B2)
    val X = (A \ B).toDenseMatrix

    val updatedPoints = dataConverter.toPointSequence(dataConverter.toMatrix(template.pointSet.points.toSeq) + X).toIndexedSeq

    // N-ICP-T GPregression
//    val A1d = A1.toDenseMatrix
//    val A2d = A2.toDenseMatrix
//    val B1d = B1.toDenseMatrix
//    val B2d = B2.toDenseMatrix
//
//    val Xgpreg = pinv(A1d.t*A1d+A2d*A2d)*A2*B2

    def myPinv(mat: DenseMatrix[Double]): DenseMatrix[Double] = {
      val Decomp = svd(mat)
      Decomp.rightVectors.t * diag(Decomp.singularValues.map(d=>if(d>0.00001) 1.0/d else 0.0)) * Decomp.leftVectors.t
    }

    val gLambda = M.toDenseMatrix.t*M.toDenseMatrix
    //  private val K = pinv(gLambda/*+0.01*DenseMatrix.eye[Double](gLambda.rows)*/)
    val K = myPinv(gLambda)

    // Experiment:
    val In = DenseMatrix.eye[Double](K.rows)
//    val invK = pinv(K)

    val y = W*(U-V)
    val ymean = y.t*DenseVector.ones[Double](y.rows)/sum(w) //y.rows.toDouble
    val gpMean = K*myPinv(K+alpha*alpha*In)*y
    val gpMeanAdd = gpMean(*,::).map{r => r+ymean}

    val validation = gpMeanAdd-X
    val sumvalidation = sum(validation)
    val gpMeanK = (-K+K*myPinv(K+In))*y
//    val part1 = K-K*pinv(In+K)*K
//    val part2 = pinv(pinv(K)+In)


//    val noise = A2.toDenseMatrix
//    val Lnoise = Ln+noise
//    val LnoiseInv = inv(Lnoise)
//    val Wd = W.toDenseMatrix
//    val ydef = A2.toDenseMatrix*(U.toDenseMatrix-V.toDenseMatrix)
//
//    val Xgpreg = LnoiseInv*ydef

    (template.copy(pointSet = UnstructuredPoints3D(updatedPoints)), cpDist)
  }
}