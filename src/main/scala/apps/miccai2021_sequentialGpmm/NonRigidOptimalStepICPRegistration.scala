package apps.miccai2021_sequentialGpmm

import java.awt.Color
import java.io.File
import api.registration.icp.NonRigidOptimalStepICP_T
import apps.miccai2021_sequentialGpmm.SyntheticData.{gtLabelMaps, inputLocation, modelFile, namesToTest, syntheticDir}
import scalismo.common.ScalarMeshField
import scalismo.geometry._3D
import scalismo.io.{LandmarkIO, MeshIO, StatismoIO}
import scalismo.mesh.TriangleMesh
import scalismo.ui.api.ScalismoUI

object NonRigidOptimalStepICPRegistration extends App {

  scalismo.initialize()

  val outputDir = new File(syntheticDir, "outputNRICP")
  outputDir.mkdirs()

  val model = StatismoIO.readStatismoPDM[_3D, TriangleMesh](new File(modelFile, "augmentedModel100.h5")).get
  val template = model.reference

  namesToTest.sorted.foreach { name =>
    println(name)

    val meshoutputdir = new File(outputDir, s"mesh$name")
    meshoutputdir.mkdirs()

    val target = MeshIO.readMesh(new File(inputLocation, s"$name.stl")).get
    val gtTarget = MeshIO.readScalarMeshField[Int](gtLabelMaps.filter(_.getName.contains(name)).head).get

    println(s"Template points: ${template.pointSet.numberOfPoints}, triangles: ${template.triangles.length}")
    println(s"Target points: ${target.pointSet.numberOfPoints}, triangles: ${target.triangles.length}")

    // Choose between ICP-A and ICP-T
    //  val nicp = new NonRigidOptimalStepICP_A(template, target, Seq(), Seq())
    val nicp = new NonRigidOptimalStepICP_T(template,
      target,
      Seq(),
      Seq())
    val t0 = System.currentTimeMillis()
    val fitAll = nicp.Registration(10, 0.0000001)
    val fit = fitAll._1
    val w = fitAll._2.toIndexedSeq
    val t1 = System.currentTimeMillis()
    println(s"Fitting time: ${(t1 - t0) / 1000.0} sec.")

    println(s"Fit points: ${fit.pointSet.numberOfPoints}, triangles: ${fit.triangles.length}")

    val labelMap = ScalarMeshField[Double](fit,w)
    val labelMapREc = ScalarMeshField[Double](model.reference,w)
    MeshIO.writeScalarMeshField[Double](labelMap, new File (meshoutputdir, s"labelMapOnFit.vtk"))
    MeshIO.writeScalarMeshField[Double](labelMapREc, new File (meshoutputdir, s"labelMapOnRef.vtk"))
    MeshIO.writeMesh(fit,new File (meshoutputdir, "fit.vtk"))
  }
}