package api.registration.icp

import api.registration.utils.{CSCHelper, NonRigidClosestPointRegistrator, PointSequenceConverter}
import breeze.linalg.{CSCMatrix, DenseVector, SparseVector, diag}
import scalismo.common.{DomainWarp, PointId, UnstructuredPoints3D, Vectorizer}
import scalismo.geometry._
import scalismo.mesh.{TriangleCell, TriangleMesh}

/*
 Implementation of the paper: "Optimal Step Nonrigid ICP Algorithms for Surface Registration"
 */

abstract class NonRigidOptimalStepICP(templateMesh: TriangleMesh[_3D],
                                      targetMesh: TriangleMesh[_3D],
                                      templateLandmarks: Seq[Landmark[_3D]],
                                      targetLandmarks: Seq[Landmark[_3D]],
                                      gamma: Double = 1.0)(
                                       implicit vectorizer: Vectorizer[Point[_3D]],
                                       dataConverter: PointSequenceConverter[_3D],
                                       warper: DomainWarp[_3D, TriangleMesh]
                                     ) {
  require(gamma >= 0)
  val n: Int = templateMesh.pointSet.numberOfPoints // Number of nodes
  val dim: Int = vectorizer.dim

  private val commonLmNames = templateLandmarks.map(_.id) intersect targetLandmarks.map(_.id)
  val lmIdsOnTemplate: Seq[PointId] = commonLmNames.map(name => templateLandmarks.find(_.id == name).get).map(lm => templateMesh.pointSet.findClosestPoint(lm.point).id)
  private val lmPointsOnTarget: Seq[Point[_3D]] = commonLmNames.map(name => targetLandmarks.find(_.id == name).get).map(lm => targetMesh.pointSet.findClosestPoint(lm.point).point)
  val UL: CSCMatrix[Double] = CSCHelper.DenseMatrix2CSCMatrix(dataConverter.toMatrix(lmPointsOnTarget))

  private val edges = trianglesToEdges(templateMesh.triangulation.triangles)
  val numOfEdges: Int = edges.length

  val M: CSCMatrix[Double] = InitializeMatrixM(edges)

  private val defaultAlpha: Seq[Double] = (1 to 10).scanLeft(1)((a, _) => (a * 2)).map(_.toDouble / 2).reverse // ...,8,4,2,1,0.5
  private val defaultBeta: Seq[Double] = defaultAlpha.indices.map(_ => 1.0)

  private def trianglesToEdges(triangles: IndexedSeq[TriangleCell]): IndexedSeq[(PointId, PointId)] = {
    triangles.flatMap { triangle =>
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

  def Registration(max_iteration: Int, tolerance: Double = 0.001, alpha: Seq[Double] = defaultAlpha, beta: Seq[Double] = defaultBeta): (TriangleMesh[_3D], Seq[Double])= {
    require(alpha.length == beta.length)

    val initseq = Seq(0.0,0.0,0.0)

    val fit = (alpha zip beta).zipWithIndex.foldLeft((templateMesh, initseq)) { (temp, config) =>
      val (a, b) = config._1
      val j = config._2
      val innerFit = (0 until max_iteration).foldLeft((temp._1, Double.PositiveInfinity,temp._2)) { (it, i) =>
        val iter = Iteration(it._1, targetMesh, a, b)
        val TY = iter._1
        val newDist = iter._2
        val w = iter._3
        println(s"ICP, iteration: ${j * max_iteration + i}/${max_iteration * alpha.length}, alpha: ${a}, beta: ${b}, average distance to target: ${newDist}")
        if (newDist < tolerance) {
          println("Converged")
          return (TY,w)
        } else {
          iter
        }
      }
      (innerFit._1,innerFit._3)
    }
    fit
  }

  def getClosestPoints(template: TriangleMesh[_3D], target: TriangleMesh[_3D]): (Seq[Point[_3D]], Seq[Double], Double) = {
    val (corr, dist) = NonRigidClosestPointRegistrator.ClosestPointTriangleMesh3D.closestPointCorrespondence(template, target)
    val cp = corr.map(_._2)
    val w = corr.map(_._3)
    (cp, w, dist)
  }

  def Iteration(template: TriangleMesh[_3D], target: TriangleMesh[_3D], alpha: Double, beta: Double): (TriangleMesh[_3D], Double, Seq[Double])
}


/*
 Implementation of "N-ICP-T" from the paper: "Optimal Step Nonrigid ICP Algorithms for Surface Registration"
 */
class NonRigidOptimalStepICP_T(templateMesh: TriangleMesh[_3D],
                               targetMesh: TriangleMesh[_3D],
                               templateLandmarks: Seq[Landmark[_3D]],
                               targetLandmarks: Seq[Landmark[_3D]],
                               gamma: Double = 1.0)(
                                implicit vectorizer: Vectorizer[Point[_3D]],
                                dataConverter: PointSequenceConverter[_3D],
                                warper: DomainWarp[_3D, TriangleMesh]
                              ) extends NonRigidOptimalStepICP(templateMesh, targetMesh, templateLandmarks, targetLandmarks, gamma) {
  val B1: CSCMatrix[Double] = CSCMatrix.zeros[Double](numOfEdges, dim)

  override def Iteration(template: TriangleMesh[_3D], target: TriangleMesh[_3D], alpha: Double, beta: Double): (TriangleMesh[_3D], Double, Seq[Double]) = {
    require(alpha >= 0.0)
    require(beta >= 0.0)

    val (cp, w, dist) = getClosestPoints(template, target)

    val W = diag(SparseVector(w: _*))

    val lmPointsOnTemplate = lmIdsOnTemplate.map(id => template.pointSet.point(id))
    val VL = CSCHelper.DenseMatrix2CSCMatrix(dataConverter.toMatrix(lmPointsOnTemplate))

    val U = CSCHelper.DenseMatrix2CSCMatrix(dataConverter.toMatrix(cp))
    val WL = CSCHelper.eye(lmPointsOnTemplate.length)
    // N-ICP-T
    val V = CSCHelper.DenseMatrix2CSCMatrix(dataConverter.toMatrix(template.pointSet.points.toSeq))
    val A1: CSCMatrix[Double] = M * alpha
    val A2: CSCMatrix[Double] = W * CSCHelper.eye(template.pointSet.numberOfPoints)
    val A3: CSCMatrix[Double] = CSCMatrix.zeros[Double](lmPointsOnTemplate.length, M.cols)
    lmPointsOnTemplate.indices.foreach(i => A3(i, i) = 1.0)

    val B2: CSCMatrix[Double] = W * (U - V)
    val B3: CSCMatrix[Double] = (UL - VL) * beta

    val A = CSCHelper.vertcat(A1, A2, A3)
    val B = CSCHelper.vertcat(B1, B2, B3)
    val X = (A \ B).toDenseMatrix

    val updatedPoints = dataConverter.toPointSequence(dataConverter.toMatrix(template.pointSet.points.toSeq) + X).toIndexedSeq

    (template.copy(pointSet = UnstructuredPoints3D(updatedPoints)), dist, w)
  }
}

/*
 Implementation of "N-ICP-A" from the paper: "Optimal Step Nonrigid ICP Algorithms for Surface Registration"
 */

class NonRigidOptimalStepICP_A(templateMesh: TriangleMesh[_3D],
                               targetMesh: TriangleMesh[_3D],
                               templateLandmarks: Seq[Landmark[_3D]],
                               targetLandmarks: Seq[Landmark[_3D]],
                               gamma: Double = 1.0)(
                                implicit vectorizer: Vectorizer[Point[_3D]],
                                dataConverter: PointSequenceConverter[_3D],
                                warper: DomainWarp[_3D, TriangleMesh]
                              ) extends NonRigidOptimalStepICP(templateMesh, targetMesh, templateLandmarks, targetLandmarks, gamma) {

  private val G = diag(SparseVector(1, 1, 1, gamma))
  private val kronMG: CSCMatrix[Double] = CSCHelper.kroneckerProduct(M, G)

  val B1: CSCMatrix[Double] = CSCMatrix.zeros[Double](4 * numOfEdges, dim)

  private def ComputeMatrixD(points: Seq[Point[_3D]]): CSCMatrix[Double] = {
    val locn = points.length
    val D: CSCMatrix[Double] = CSCMatrix.zeros[Double](locn, 4 * n) // Reference points each entry in D is a matrix (4D)
    (0 until locn).foreach { i =>
      (0 until dim + 1).foreach { j =>
        val index = i * 4 + j
        if (j == dim) D(i, index) = 1.0
        else D(i, index) = points(i)(j)
      }
    }
    D
  }

  override def Iteration(template: TriangleMesh[_3D], target: TriangleMesh[_3D], alpha: Double, beta: Double): (TriangleMesh[_3D], Double, Seq[Double]) = {
    require(alpha >= 0.0)
    require(beta >= 0.0)

    val (cp, w, dist) = getClosestPoints(template, target)

    val W = diag(SparseVector(w: _*))

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

    val updatedPoints = dataConverter.toPointSequence(D * X).toIndexedSeq

    (template.copy(pointSet = UnstructuredPoints3D(updatedPoints)), dist,w)
  }
}