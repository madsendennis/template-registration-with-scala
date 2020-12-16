package api.registration.icp

import java.awt.Color
import java.io.File

import api.registration.utils.{CSCHelper, PointSequenceConverter}
import breeze.linalg.{CSCMatrix, DenseMatrix, DenseVector, SparseVector, diag}
import breeze.numerics.sqrt
import scalismo.common.{PointId, PointSet, UnstructuredPoints3D, Vectorizer}
import scalismo.geometry._
import scalismo.io.{LandmarkIO, MeshIO}
import scalismo.mesh.TriangleMesh
import scalismo.ui.api.ScalismoUI

private[icp] class NonRigidICP[D: NDSpace](
                                            override val targetPoints: PointSet[D],
                                            override val icp: ICPFactory[D]
                                          )(
                                            implicit vectorizer: Vectorizer[Point[D]],
                                            dataConverter: PointSequenceConverter[D]
                                          ) extends RigidICP[D](targetPoints, icp) {
}

/*
 Implementation of the paper: "Optimal Step Nonrigid ICP Algorithms for Surface Registration"
 */
case class NonRigidICPoptimalStep(templateMesh: TriangleMesh[_3D],
                                  targetMesh: TriangleMesh[_3D],
                                  templateLandmarks: Seq[Landmark[_3D]],
                                  targetLandmarks: Seq[Landmark[_3D]],
                                  gamma: Double = 1.0)(
                                   implicit vectorizer: Vectorizer[Point[_3D]],
                                   dataConverter: PointSequenceConverter[_3D]
                                 ) {
  require(gamma >= 0)
  val m: Int = templateMesh.triangulation.triangles.length * 3
  val n: Int = templateMesh.pointSet.numberOfPoints
  val dim = vectorizer.dim


  private val commonLmNames = templateLandmarks.map(_.id) intersect targetLandmarks.map(_.id)
  private val lmIdsOnTemplate: Seq[PointId] = commonLmNames.map(name => templateLandmarks.find(_.id == name).get).map(lm => templateMesh.pointSet.findClosestPoint(lm.point).id)
  private val lmPointsOnTarget: Seq[Point[_3D]] = commonLmNames.map(name => targetLandmarks.find(_.id == name).get).map(lm => targetMesh.pointSet.findClosestPoint(lm.point).point)

  private val UL: CSCMatrix[Double] = CSCHelper.DenseMatrix2CSCMatrix(dataConverter.toMatrix(lmPointsOnTarget))

  private val M: CSCMatrix[Double] = CSCMatrix.zeros[Double](m, n)
  templateMesh.triangulation.triangles.toIndexedSeq.zipWithIndex.map { case (t, i) =>
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
  private val G: CSCMatrix[Double] = diag(SparseVector(1, 1, 1, gamma))
  private val kronMG: CSCMatrix[Double] = CSCHelper.kroneckerProduct(M, G) // TODO: Need optimized version of kron

  //  private val defaultAlpha: Seq[Double] = Seq(100,50,25,15,10,5,3,2,1)
  //  private val defaultBeta: Seq[Double] = Seq(0,0,0,0,0,0,0,0,0)

  private val defaultAlpha: Seq[Double] = (1 to 10).scanLeft(1)((a, _) => (a * 2)).map(_.toDouble / 2).reverse // ...,8,4,2,1,0.5
  private val defaultBeta: Seq[Double] = defaultAlpha.indices.map(_ => 0.0)

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

  def Iteration(template: TriangleMesh[_3D], target: TriangleMesh[_3D], alpha: Double, beta: Double): (TriangleMesh[_3D], Double) = {
    require(alpha >= 0.0)
    require(beta >= 0.0)
    val w = SparseVector.zeros[Double](n)
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
    val D: CSCMatrix[Double] = CSCMatrix.zeros[Double](n, 4 * n) // Reference points each entry in D is a matrix (4nx3)
    (0 until n).foreach { i =>
      (0 until dim + 1).foreach { j =>
        val index = i * 4 + j
        if (j == dim) D(i, index) = 1.0
        else D(i, index) = template.pointSet.point(PointId(i))(j)
      }
    }

    val lmPointsOnTemplate = lmIdsOnTemplate.map(id => template.pointSet.point(id))
    val nLM = lmPointsOnTemplate.length
    val DL: CSCMatrix[Double] = CSCMatrix.zeros[Double](nLM, 4 * n)
    (0 until nLM).foreach { i =>
      (0 until dim + 1).foreach { j =>
        val index = i * 4 + j
        if (j == dim) DL(i, index) = 1.0
        else DL(i, index) = lmPointsOnTemplate(i)(j)
      }
    }
    val U = CSCHelper.DenseMatrix2CSCMatrix(dataConverter.toMatrix(cp))

    val A1: CSCMatrix[Double] = kronMG * alpha
    val A2: CSCMatrix[Double] = W * D
    val A3: CSCMatrix[Double] = DL * beta

    val B1: CSCMatrix[Double] = CSCMatrix.zeros[Double](4 * m, dim)
    val B2: CSCMatrix[Double] = W * U
    val B3: CSCMatrix[Double] = UL * beta // TODO: Should beta also be multiplied to B3 ???


    val A = CSCHelper.vertcat(A1, A2, A3)
    val B = CSCHelper.vertcat(B1, B2, B3)

    val X = (A \ B).toDenseMatrix

    val updatedPoints = template.pointSet.points.toIndexedSeq.zipWithIndex.map { case (p, i) =>
      val x: DenseMatrix[Double] = X(i * 4 until i * 4 + 4, ::).t
      val v: DenseVector[Double] = DenseVector[Double](p(0), p(1), p(2), 1.0)
      val xv = (x * v)
      Point3D(xv(0), xv(1), xv(2))
    }

    (template.copy(pointSet = UnstructuredPoints3D(updatedPoints)), cpDist)
  }

}


object FemurRegistrationICPnonRigid extends App {
  scalismo.initialize()

  val vertexDecimater = 500
  val template = MeshIO.readMesh(new File("data/femur_reference.stl")).get.operations.decimate(vertexDecimater)
  val target = MeshIO.readMesh(new File("data/femur_target.stl")).get
  val templateLms = LandmarkIO.readLandmarksJson3D(new File("data/femur_reference.json")).get
  val targetLms = LandmarkIO.readLandmarksJson3D(new File("data/femur_target.json")).get

  println(s"Template points: ${template.pointSet.numberOfPoints}, triangles: ${template.triangles.length}")
  println(s"Target points: ${target.pointSet.numberOfPoints}, triangles: ${target.triangles.length}")

  //  val nicp = NonRigidICPoptimalStep(template, target, templateLms, targetLms) // With landmarks
  val t00 = System.currentTimeMillis()
  val nicp = NonRigidICPoptimalStep(template, target, Seq(), Seq()) // Without landmarks
  val t01 = System.currentTimeMillis()
  println(s"Config time: ${(t01 - t00) / 1000} sec.")

  val t10 = System.currentTimeMillis()
  val fit = nicp.Registration(5, 0.0000001)
  val t11 = System.currentTimeMillis()
  println(s"Fitting time: ${(t11 - t10) / 1000.0} sec.")


  println(s"Fit points: ${fit.pointSet.numberOfPoints}, triangles: ${fit.triangles.length}")

  val ui = ScalismoUI()
  val dataGroup = ui.createGroup("data")
  ui.show(dataGroup, template, "template").color = Color.RED
  ui.show(dataGroup, target, "target").color = Color.GREEN
  ui.show(dataGroup, fit, "fit")
}