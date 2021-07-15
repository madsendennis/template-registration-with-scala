package apps.animations

import java.io.{File, IOException}

import scalismo.common.{DiscreteField, UnstructuredPointsDomain}
import scalismo.geometry.{EuclideanVector, _3D}
import vtk.{vtkCellArray, vtkDoubleArray, vtkFloatArray, vtkPoints, vtkPolyData, vtkPolyDataWriter, vtkVertex}

import scala.util.{Failure, Success, Try}

object ioStuff {

  def writeVTKPdasVTK(vtkPd: vtkPolyData, file: File): Try[Unit] = {
    val writer = new vtkPolyDataWriter()
    writer.SetFileName(file.getAbsolutePath)
    writer.SetInputData(vtkPd)
    //      writer.SetFileTypeToBinary()
    writer.Update()
    val succOrFailure = if (writer.GetErrorCode() != 0) {
      Failure(
        new IOException(s"could not write file ${file.getAbsolutePath} (received error code ${writer.GetErrorCode})")
      )
    } else {
      Success(())
    }
    writer.Delete()
    succOrFailure
  }

  def convertDiscreteFieldToVtkPolyData(df: DiscreteField[_3D, UnstructuredPointsDomain, EuclideanVector[_3D]]): vtkPolyData = {
    //      lazy val vectors: immutable.IndexedSeq[EuclideanVector[_3D]] = source.values.toIndexedSeq
    val n = df.data.length
    def setupPolyData(): vtkPolyData = {
      val points = new vtkPoints
      points.SetNumberOfPoints(n)
      val vertices = new vtkCellArray()
      vertices.SetNumberOfCells(n)

      for ((point, i) <- df.domain.pointSet.points.toIndexedSeq.zipWithIndex) {
        points.SetPoint(i, point(0), point(1), point(2))
        val vertex = new vtkVertex
        vertex.GetPointIds().SetId(0,i)
        vertices.InsertNextCell(vertex)
      }

      val vectors = new vtkDoubleArray
      vectors.SetName("u")
      vectors.SetNumberOfComponents(3)
      vectors.SetNumberOfTuples(n)
      for ((vector, i) <- df.data.toIndexedSeq.zipWithIndex) {
        vectors.SetTuple3(i, vector(0), vector(1), vector(2))
      }

      new vtkPolyData {
        SetPoints(points)
        SetVerts(vertices)
        GetPointData().SetVectors(vectors)
      }
    }
    val polydata: vtkPolyData = setupPolyData()
    polydata
  }
}
