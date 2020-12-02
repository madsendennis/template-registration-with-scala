package api.registration.utils

import breeze.linalg.DenseMatrix
import scalismo.common.Vectorizer
import scalismo.geometry.Point.{Point1DVectorizer, Point2DVectorizer, Point3DVectorizer}
import scalismo.geometry._

trait PointSequenceConverter[D] {
  def toPointSequence(
      mat: DenseMatrix[Double]
  )(implicit vectorizer: Vectorizer[Point[D]]): Seq[Point[D]]

  def toMatrix(points: Seq[Point[D]])(implicit vectorizer: Vectorizer[Point[D]]): DenseMatrix[Double] = {
    val dim: Int = vectorizer.dim
    val mat = DenseMatrix.zeros[Double](points.length, dim)
    points.zipWithIndex.foreach {
      case (p, i) =>
        mat(i, ::) := vectorizer.vectorize(p).t
    }
    mat
  }
}

object PointSequenceConverter {

  def matrixTo1Dpoints(mat: DenseMatrix[Double]): IndexedSeq[Point[_1D]] = {
    0 until mat.rows map { r =>
      Point1D(x = mat(r, 0))
    }
  }

  def matrixTo2Dpoints(mat: DenseMatrix[Double]): IndexedSeq[Point[_2D]] = {
    0 until mat.rows map { r =>
      Point2D(x = mat(r, 0), y = mat(r, 1))
    }
  }

  def matrixTo3Dpoints(mat: DenseMatrix[Double]): IndexedSeq[Point[_3D]] = {
    0 until mat.rows map { r =>
      Point3D(x = mat(r, 0), y = mat(r, 1), z = mat(r, 2))
    }
  }

  implicit object denseMatrixToPoint1DSequence extends PointSequenceConverter[_1D] {
    implicit val vectorizer = Point1DVectorizer
    override def toPointSequence(mat: DenseMatrix[Double])(implicit vectorizer: Vectorizer[Point[_1D]]): Seq[Point[_1D]] = {
      matrixTo1Dpoints(mat)
    }
  }

  implicit object denseMatrixToPoint2DSequence extends PointSequenceConverter[_2D] {
    implicit val vectorizer = Point2DVectorizer
    override def toPointSequence(mat: DenseMatrix[Double])(implicit vectorizer: Vectorizer[Point[_2D]]): Seq[Point[_2D]] = {
      matrixTo2Dpoints(mat)
    }
  }

  implicit object denseMatrixToPoint3DSequence extends PointSequenceConverter[_3D] {
    implicit val vectorizer = Point3DVectorizer
    override def toPointSequence(mat: DenseMatrix[Double])(implicit vectorizer: Vectorizer[Point[_3D]]): Seq[Point[_3D]] = {
      matrixTo3Dpoints(mat)
    }
  }
}
