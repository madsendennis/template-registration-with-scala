package api.registration

import breeze.linalg.DenseMatrix
import scalismo.common._
import scalismo.geometry._
import scalismo.mesh._

trait DiscreteDomainConverter[D, DDomain[D] <: DiscreteDomain[D]] {
  def denseMatrixToDomain(mat: DenseMatrix[Double], reference: DDomain[D]): DDomain[D]

  def PointCloudToMatrix(dc: DDomain[D]): DenseMatrix[Double] = {
    val dim: Int = dc.pointSet.points.toIndexedSeq.head.dimensionality
    val mat = DenseMatrix.zeros[Double](dc.pointSet.numberOfPoints, dim)
    dc.pointSet.points.zipWithIndex.foreach { case (p, i) =>
      (0 until dim).foreach { j =>
        mat(i, j) = p(j)
      }
    }
    mat
  }
}

object DiscreteDomainConverter {

  private def matrixTo1Dpoints(mat: DenseMatrix[Double]): IndexedSeq[Point[_1D]] = {
    0 until mat.rows map { r => Point1D(x = mat(r, 0)) }
  }

  private def matrixTo2Dpoints(mat: DenseMatrix[Double]): IndexedSeq[Point[_2D]] = {
    0 until mat.rows map { r => Point2D(x = mat(r, 0), y = mat(r, 1)) }
  }

  private def matrixTo3Dpoints(mat: DenseMatrix[Double]): IndexedSeq[Point[_3D]] = {
    0 until mat.rows map { r => Point3D(x = mat(r, 0), y = mat(r, 1), z = mat(r, 2)) }
  }

  implicit object denseMatrixToPointDomain1D extends DiscreteDomainConverter[_1D, UnstructuredPointsDomain] {
    override def denseMatrixToDomain(mat: DenseMatrix[Double], reference: UnstructuredPointsDomain[_1D]): UnstructuredPointsDomain[_1D] = {
      val p: IndexedSeq[Point[_1D]] = matrixTo1Dpoints(mat)
      UnstructuredPointsDomain.Create.CreateUnstructuredPointsDomain1D.create(p)
    }
  }

  implicit object denseMatrixToPointDomain2D extends DiscreteDomainConverter[_2D, UnstructuredPointsDomain] {
    override def denseMatrixToDomain(mat: DenseMatrix[Double], reference: UnstructuredPointsDomain[_2D]): UnstructuredPointsDomain[_2D] = {
      val p: IndexedSeq[Point[_2D]] = matrixTo2Dpoints(mat)
      UnstructuredPointsDomain.Create.CreateUnstructuredPointsDomain2D.create(p)
    }
  }

  implicit object denseMatrixToPointDomain3D extends DiscreteDomainConverter[_3D, UnstructuredPointsDomain] {
    override def denseMatrixToDomain(mat: DenseMatrix[Double], reference: UnstructuredPointsDomain[_3D]): UnstructuredPointsDomain[_3D] = {
      val p: IndexedSeq[Point[_3D]] = matrixTo3Dpoints(mat)
      UnstructuredPointsDomain.Create.CreateUnstructuredPointsDomain3D.create(p)
    }
  }

  implicit object denseMatrixToTriangleMesh2D extends DiscreteDomainConverter[_2D, TriangleMesh] {
    override def denseMatrixToDomain(mat: DenseMatrix[Double], reference: TriangleMesh[_2D]): TriangleMesh[_2D] = {
      val p: IndexedSeq[Point[_2D]] = matrixTo2Dpoints(mat)
      TriangleMesh2D(UnstructuredPoints2D(p), reference.triangulation)
    }
  }

  implicit object denseMatrixToTriangleMesh3D extends DiscreteDomainConverter[_3D, TriangleMesh] {
    override def denseMatrixToDomain(mat: DenseMatrix[Double], reference: TriangleMesh[_3D]): TriangleMesh[_3D] = {
      val p: IndexedSeq[Point[_3D]] = matrixTo3Dpoints(mat)
      TriangleMesh3D(p, reference.triangulation)
    }
  }

  implicit object denseMatrixToTetrahedralMesh3D extends DiscreteDomainConverter[_3D, TetrahedralMesh] {
    override def denseMatrixToDomain(mat: DenseMatrix[Double], reference: TetrahedralMesh[_3D]): TetrahedralMesh[_3D] = {
      val p: IndexedSeq[Point[_3D]] = matrixTo3Dpoints(mat)
      TetrahedralMesh3D(p, reference.tetrahedralization)
    }
  }

}