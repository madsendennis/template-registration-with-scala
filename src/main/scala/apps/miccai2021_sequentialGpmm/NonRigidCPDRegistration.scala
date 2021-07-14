package apps.miccai2021_sequentialGpmm

import java.awt.Color
import java.io.File
import api.registration.NonRigidCPDRegistration
import apps.miccai2021_sequentialGpmm.SyntheticData.{gtLabelMaps, inputLocation, namesToTest, syntheticDir}
import breeze.numerics.pow
import scalismo.common.ScalarMeshField
import scalismo.geometry._3D
import scalismo.io.{MeshIO, StatismoIO}
import scalismo.mesh.TriangleMesh
import scalismo.ui.api.ScalismoUI

object NonRigidCPDRegistration extends App {
  scalismo.initialize()

  val outputDir = new File(syntheticDir, "outputCPD")
  outputDir.mkdirs()

  val model = StatismoIO.readStatismoPDM[_3D, TriangleMesh](SyntheticData.modelFile).get
  val template = model.reference

  namesToTest.sorted.foreach { name =>

    println(name)

    val meshoutputdir = new File (outputDir, s"mesh$name")
    meshoutputdir.mkdirs()

    val target = MeshIO.readMesh(new File(inputLocation, s"$name.stl")).get
    val gtTarget = MeshIO.readScalarMeshField[Int](gtLabelMaps.filter(_.getName.contains(name)).head).get

    println(s"Template points: ${template.pointSet.numberOfPoints}, triangles: ${template.triangles.length}")
    println(s"Target points: ${target.pointSet.numberOfPoints}, triangles: ${target.triangles.length}")

    //note set w to expected pathology size 0.05,0.1,0.2,0.33
    val cpd = new NonRigidCPDRegistration(template, lambda = 1, beta = 50, w = 0.10, max_iterations = 30)

    val t10 = System.currentTimeMillis()
    val fitAll = cpd.register(target)
    val fit = fitAll._1
    val P1 = fitAll._2
    val t11 = System.currentTimeMillis()
    println(s"Fitting time: ${(t11 - t10) / 1000.0} sec.")

    println(s"Fit points: ${fit.pointSet.numberOfPoints}, triangles: ${fit.triangles.length}")

    val data = P1.data.toIndexedSeq.map{v=>
      if (v<0.001) 1.0 else 0.0
    }.map(_.toInt)
    val labelMap = ScalarMeshField[Int](fit,data)
    val labelMapOnRef = ScalarMeshField[Int](model.reference,data)
    MeshIO.writeScalarMeshField[Int](labelMap, new File (meshoutputdir, s"labelMapOnFit.vtk"))
    MeshIO.writeMesh(fit, new File (meshoutputdir, s"fit.vtk"))
    MeshIO.writeScalarMeshField[Int](labelMapOnRef, new File (meshoutputdir, s"labelMapOnRef.vtk"))
  }
}
