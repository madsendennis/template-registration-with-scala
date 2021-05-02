package apps.femurCourse

import java.io.File

import apps.util.FileUtils
import scalismo.common.PointWithId
import scalismo.geometry.{Point, _3D}
import scalismo.io.MeshIO
import scalismo.mesh.{MeshMetrics, TriangleMesh}
import scalismo.utils.MeshConversion
import vtk.{vtkPolyData, vtkPolyDataConnectivityFilter}

case class MeshDistances(avgDistance: Double, hausdorffDistance: Double)

object xxxMagicEvaluator extends App{
  scalismo.initialize()
  val completedPath = new File(data.completed, "preg_pca2_2mm").listFiles(_.getName.endsWith(".stl")).sorted

  val targetPath = data.step3.listFiles(_.getName.endsWith(".stl"))
  val groundTruth = data.groundTruth.listFiles(_.getName.endsWith(".stl"))

  val res = completedPath.map{f =>
    val res = evaluate(f.getName)
    println(s"${res._1}, avg: ${res._2.avgDistance}, haus: ${res._2.hausdorffDistance}")
    res
  }
  val avgDist = res.map(f => f._2.avgDistance).sum/res.length.toDouble
  val hausDist = res.map(f => f._2.hausdorffDistance).sum/res.length.toDouble

  println(s"Total - avg: ${avgDist}, haus: ${hausDist}")

  def evaluate(id: String): (String, MeshDistances) = {
        val reconstruction = MeshIO.readMesh(completedPath.find(_.getName.equals(id)).get).get
        val groundTruthFullBone = MeshIO.readMesh(groundTruth.find(_.getName.equals(id)).get).get
        val groundTruthPartialBone = MeshIO.readMesh(targetPath.find(_.getName.equals(id)).get).get

        // identify the point identifiers for the patches that need to be reconstructed
        // the ptIds are here the points of the patches that are given, as these
        // are later clipped away
        val ptIDsGivemPartGroundTruth = groundTruthFullBone.pointSet.pointsWithId.filter { case (pt, id) =>
          val distanceToPartial = (groundTruthPartialBone.pointSet.findClosestPoint(pt).point - pt).norm
          distanceToPartial < 1.0
        }.map(_._2).toIndexedSeq

        // we get the ptIds for the reconstruction using forward and back projection. If we end up on the
        // same point, we assume that it was the given part. If our closest point turns out to be a different
        // point, we will
        val ptIdsReconstructions = reconstruction.pointSet.pointsWithId.filter { case (ptOnRec, id) =>
          val PointWithId(pointOnPartial, partialGTId) = groundTruthPartialBone.pointSet.findClosestPoint(ptOnRec)
          val backProjectedId = reconstruction.pointSet.findClosestPoint(pointOnPartial).id
          //groundTruthPartialBone.operations.pointIsOnBoundary(partialGTId)
          backProjectedId == id
        }.map(_._2).toIndexedSeq


        // cut the reconstructed patches from the ground truth and the reconstruction
        val clippedPartGroundTruth = groundTruthFullBone.operations.clip((pt: Point[_3D]) => ptIDsGivemPartGroundTruth.contains(groundTruthFullBone.pointSet.findClosestPoint(pt).id))
        val truePatch = getBiggestConnectedComponent(clippedPartGroundTruth)
        val clippedReconstruction = reconstruction.operations.clip((pt: Point[_3D]) => ptIdsReconstructions.contains(reconstruction.pointSet.findClosestPoint(pt).id))
        val reconstructedPatch = getBiggestConnectedComponent(clippedReconstruction)

        //        MeshIO.writeMesh(reconstructedPatch, new java.io.File(s"c:/users/Luethi/tmp/debug/rec-${id.value}.stl")).get
        //        MeshIO.writeMesh(truePatch, new java.io.File(s"c:/users/Luethi/tmp/debug/true-${id.value}.stl")).get
        // compute average mesh distance in both directions
        val averageOneWay = MeshMetrics.avgDistance(truePatch, reconstructedPatch)
        val averageOtherWay = MeshMetrics.avgDistance(reconstructedPatch, truePatch)

        // Hausdorff distance. We compute that on the full mesh, rather than the reconstruction,
        // to avoid problems in case that we don't cut the boundary perfectly.
        val hausdorff = MeshMetrics.hausdorffDistance(reconstruction, groundTruthFullBone)

        (id, MeshDistances((averageOneWay + averageOtherWay) * 0.5, hausdorff))
     }

    /**
      * Extracts the biggest connected component in a mesh.
      *
      * We use this to be safe and guarantee that no disconnected points or cells are used when comparing the 2 patches
      **/
    private def getBiggestConnectedComponent(mesh: TriangleMesh[_3D]): TriangleMesh[_3D] = {
      val connectivity = new vtkPolyDataConnectivityFilter()

      val polydata = MeshConversion.meshToVtkPolyData(mesh)

      connectivity.SetExtractionModeToSpecifiedRegions()
      connectivity.SetInputData(polydata)
      connectivity.Update()
      val count = connectivity.GetNumberOfExtractedRegions()

      val (largestSize, largestId) = {
        val vtk = connectivity.GetRegionSizes()
        val out = for (i <- -0 until count) yield vtk.GetValue(i)
        vtk.Delete()
        out
      }.zipWithIndex.sortBy(_._1).reverse.head

      connectivity.InitializeSpecifiedRegionList()
      connectivity.AddSpecifiedRegion(largestId)
      connectivity.Update()

      val out = new vtkPolyData()
      out.DeepCopy(connectivity.GetOutput())

      val outAsScalismoMesh = MeshConversion.vtkPolyDataToCorrectedTriangleMesh(out).get
      out.Delete()
      outAsScalismoMesh
    }

}
