package api.registration.icp

import api.registration.utils.{CSCHelper, PointSequenceConverter}
import breeze.linalg.{CSCMatrix, DenseMatrix, DenseVector, SparseVector, diag}
import breeze.numerics.sqrt
import scalismo.common.{PointId, UnstructuredPoints3D, Vectorizer}
import scalismo.geometry._
import scalismo.mesh.{TriangleCell, TriangleMesh}

/*
 Implementation of the paper: "Optimal Step Nonrigid ICP Algorithms for Surface Registration"
 */
class NonRigidOptimalStepICP(templateMesh: TriangleMesh[_3D],
                             targetMesh: TriangleMesh[_3D],
                             templateLandmarks: Seq[Landmark[_3D]],
                             targetLandmarks: Seq[Landmark[_3D]],
                             gamma: Double = 1.0)(
                              implicit vectorizer: Vectorizer[Point[_3D]],
                              dataConverter: PointSequenceConverter[_3D]
                            ) {
  require(gamma >= 0)
  private val m: Int = templateMesh.triangulation.triangles.length * 3 // Number of edges (3 per triangle)
  private val n: Int = templateMesh.pointSet.numberOfPoints // Number of nodes
  private val dim = vectorizer.dim

  private val commonLmNames = templateLandmarks.map(_.id) intersect targetLandmarks.map(_.id)
  private val lmIdsOnTemplate: Seq[PointId] = commonLmNames.map(name => templateLandmarks.find(_.id == name).get).map(lm => templateMesh.pointSet.findClosestPoint(lm.point).id)
  private val lmPointsOnTarget: Seq[Point[_3D]] = commonLmNames.map(name => targetLandmarks.find(_.id == name).get).map(lm => targetMesh.pointSet.findClosestPoint(lm.point).point)

  private val UL: CSCMatrix[Double] = CSCHelper.DenseMatrix2CSCMatrix(dataConverter.toMatrix(lmPointsOnTarget))

  private val M = InitializeMatrixM(templateMesh.triangulation.triangles.toIndexedSeq)
  private val G = diag(SparseVector(1, 1, 1, gamma))
  private val kronMG: CSCMatrix[Double] = CSCHelper.kroneckerProduct(M, G) // TODO: Need optimized version of kron
  val B1: CSCMatrix[Double] = CSCMatrix.zeros[Double](4 * m, dim)

  private val defaultAlpha: Seq[Double] = (1 to 10).scanLeft(1)((a, _) => (a * 2)).map(_.toDouble / 2).reverse // ...,8,4,2,1,0.5
  private val defaultBeta: Seq[Double] = defaultAlpha.indices.map(_ => 0.0)

  private def InitializeMatrixM(triangles: IndexedSeq[TriangleCell]): CSCMatrix[Double] = {
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
      // TODO: Restructure if/else setup
      if (target.operations.pointIsOnBoundary(cp.id)) w(i) = 0.0
      else if ((target.vertexNormals.atPoint(cp.id) dot template.vertexNormals.atPoint(id)) <= 0) w(i) = 0.0 // TODO: Introduce "angle" hyperparameter
      //      val ll = template.operations.getIntersectionPoints(p, p-cp.point).minBy(intersect => norm(p-intersect) )
      //      else if() // TODO: Check if closest point intersect with surface
      else w(i) = 1.0 // Set to 1 if a robust closest point is found
      cpOnSurface
    }
    val cp = cpinfo.map(_.point)
    val cpDist = sqrt(cpinfo.map(_.distanceSquared).sum / cpinfo.length)

    val W: CSCMatrix[Double] = diag(w)
    val D = ComputeMatrixD(template.pointSet.points.toSeq)

    val lmPointsOnTemplate = lmIdsOnTemplate.map(id => template.pointSet.point(id))
    val DL = ComputeMatrixD(lmPointsOnTemplate)
    val U = CSCHelper.DenseMatrix2CSCMatrix(dataConverter.toMatrix(cp))

    val A1: CSCMatrix[Double] = kronMG * alpha
    val A2: CSCMatrix[Double] = W * D
    val A3: CSCMatrix[Double] = DL * beta

    val B2: CSCMatrix[Double] = W * U
    val B3: CSCMatrix[Double] = UL * beta // TODO: Should beta also be multiplied to B3 ???

    val A = CSCHelper.vertcat(A1, A2, A3)
    val B = CSCHelper.vertcat(B1, B2, B3)

    val X = (A \ B).toDenseMatrix

    val updatedPoints = template.pointSet.points.toIndexedSeq.zipWithIndex.map { case (p, i) =>
      val x: DenseMatrix[Double] = X(i * 4 until i * 4 + 4, ::).t
      val v: DenseVector[Double] = DenseVector[Double](p(0), p(1), p(2), 1.0)
      val xv = (x * v)
      vectorizer.unvectorize(xv)
    }

    (template.copy(pointSet = UnstructuredPoints3D(updatedPoints)), cpDist)
  }
}