package api.registration.utils

import scalismo.ui.api.{ScalismoUI, _3DLeft, _3DRight}
import scalismo.ui.view.perspective.ThreeDTwicePerspective

object ScalismoUiHelper {

  def SetDualView(ui: ScalismoUI){
    val fields = ui.getClass.getDeclaredFields.filter(f => f.getName.contains("frame"))
    fields.head.setAccessible(true)
    val frame = fields.head.get(ui).asInstanceOf[scalismo.ui.view.ScalismoFrame]
    frame.perspective.perspective = ThreeDTwicePerspective
  }

}

//object UIPlayground extends App {
//  scalismo.initialize()
//  implicit val rng = Random(1024l)
//  val ui = ScalismoUI()
//  val grp = ui.createGroup("default")
//  val points: IndexedSeq[Point[_3D]] = IndexedSeq(Point3D(0, 0, 1), Point3D(1, 0, 1), Point3D(0, 1, 1))
//  val points2: IndexedSeq[Point[_3D]] = IndexedSeq(Point3D(0, 0, 10), Point3D(10, 0, 10), Point3D(0, 10, 10))
//  val fields = ui.getClass.getDeclaredFields.filter(f => f.getName.contains("frame"))
//  fields.head.setAccessible(true)
//  val frame = fields.head.get(ui).asInstanceOf[scalismo.ui.view.ScalismoFrame]
//  frame.perspective.perspective = ThreeDTwicePerspective
//  val pv = ui.show(grp, points, "points")
//  val pv2 = ui.show(grp, points2, "points2")
//  ui.setVisibility(pv, List(_3DLeft))
//  ui.setVisibility(pv2, List(_3DRight))
//}