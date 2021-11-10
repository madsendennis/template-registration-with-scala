package api.sampling.loggers

import api.GingrRegistrationState
import spray.json.DefaultJsonProtocol.jsonFormat10
import spray.json.RootJsonFormat
import spray.json.DefaultJsonProtocol._
import spray.json._
import scalismo.sampling.{DistributionEvaluator, ProposalGenerator}
import scalismo.sampling.loggers.AcceptRejectLogger
import java.io.{BufferedWriter, File, FileOutputStream, IOException, OutputStreamWriter}
import java.text.SimpleDateFormat
import java.util.Calendar

import api.sampling.Evaluator

import scala.collection.SortedSet
import scala.collection.mutable.ListBuffer
import scala.io.Source

case class jsonLogFormat(
  index: Int,
  name: String,
  logvalue: Map[String, Double],
  status: Boolean,
  modelParameters: Seq[Double],
  translation: Seq[Double],
  rotation: Seq[Double],
  rotationCenter: Seq[Double],
  scaling: Double,
  datetime: String
)

object JsonLoggerProtocol {
  implicit val myJsonFormatLogger: RootJsonFormat[jsonLogFormat] = jsonFormat10(jsonLogFormat.apply)
}

case class JSONStateLogger[State <: GingrRegistrationState[State]](evaluators: Evaluator[State], filePath: Option[File] = None) extends AcceptRejectLogger[State] {
  import JsonLoggerProtocol._

  private var numOfRejected: Int = 0
  private var numOfAccepted: Int = 0
  private var generatedBy: SortedSet[String] = SortedSet()
  private val datetimeFormat = new SimpleDateFormat("yyyy-MM-dd HH:mm:ss")
  private val productEvaluator = evaluators.productEvaluator()

  filePath.map { f =>
    if (f.getParentFile != null) {
      if (!f.getParentFile.exists()) {
        throw new IOException(s"JSON log path does not exist: ${f.getParentFile.toString}!")
      } else if (f.exists() && !f.canWrite) {
        throw new IOException(s"JSON file exist and cannot be overwritten: ${f.toString}!")
      } else if (!f.exists()) {
        try {
          f.createNewFile()
          f.delete()
        } catch {
          case e: Exception => throw new IOException(s"JSON file path cannot be written to: ${f.toString}!")
        }
      }
    }
    f.setReadable(true, false)
    f.setExecutable(true, false)
    f.setWritable(true, false)
    f
  }

  def totalSamples: Int = numOfRejected + numOfAccepted

  val log: ListBuffer[jsonLogFormat] = new ListBuffer[jsonLogFormat]

  private def identifier(sample: State): String = {
    val name = sample.general.generatedBy
    generatedBy += name
    name
  }

  private def mapEvaluators(sample: State): Map[String, Double] = {
    val evals: Map[String, Double] = evaluators.evaluator.map(f => (f.name, f.evaluator.logValue(sample))).toMap
    evals + ("product" -> productEvaluator.logValue(sample))
  }

  override def accept(current: State, sample: State, generator: ProposalGenerator[State], evaluator: DistributionEvaluator[State]): Unit = {
    numOfAccepted += 1
    val evalValue = mapEvaluators(sample)
    log += jsonLogFormat(
      totalSamples,
      identifier(sample),
      evalValue,
      status = true,
      sample.general.modelParameters.shape.parameters.toArray.toSeq,
      sample.general.modelParameters.pose.translation.toArray.toSeq,
      sample.general.modelParameters.pose.rotation.angles.parameters.toArray.toSeq,
      sample.general.modelParameters.pose.rotation.center.toBreezeVector.toArray.toSeq,
      sample.general.modelParameters.scale.s,
      datetimeFormat.format(Calendar.getInstance().getTime)
    )
  }

  // The rejected state will contain the same parameters as the previous accepted state, so no need to double store all the information
  override def reject(current: State, sample: State, generator: ProposalGenerator[State], evaluator: DistributionEvaluator[State]): Unit = {
    numOfRejected += 1
    val evalValue = mapEvaluators(sample)
    log += jsonLogFormat(
      totalSamples,
      identifier(sample),
      evalValue,
      status = false,
      Seq(),
      Seq(),
      Seq(),
      Seq(),
      sample.general.modelParameters.scale.s,
      datetimeFormat.format(Calendar.getInstance().getTime)
    )
  }

  def percentRejected: Double = BigDecimal(numOfRejected.toDouble / totalSamples.toDouble).setScale(2, BigDecimal.RoundingMode.HALF_UP).toDouble

  def percentAccepted: Double = 1.0 - percentRejected

  def writeLog(): Unit = {
    if (filePath.isEmpty) {
      throw new IOException(s"JSON log path does not exist: ${filePath.getOrElse(new File("")).getParentFile.toString}!")
    } else {
      val content = log.toIndexedSeq
      try {
        val writer = new BufferedWriter(new OutputStreamWriter(new FileOutputStream(filePath.toString)))
        writer.write(content.toList.toJson.prettyPrint)
        writer.close()
      } catch {
        case e: Exception => throw new IOException("Writing JSON log file failed!")
      }
      println("Log written to: " + filePath.toString)
    }
  }

  def printAcceptInfo(id: String = ""): Unit = {
    val lastX = 100
    println(s"${id} Total accepted (${totalSamples}): ${percentAccepted}")
    generatedBy.foreach { name =>
      println(s"${id} ${name}: ${percentAcceptedOfType(name)}")
    }
    if (log.length > lastX) {
      val logLastX = log.takeRight(lastX)
      println(s"${id} Last ${lastX} samples accepted (${lastX}): ${logLastX.map(f => if (f.status) 1.0 else .0).sum / lastX.toDouble}")
      generatedBy.foreach { name =>
        println(s"${id} ${name}: ${percentAcceptedOfTypeLocal(name, logLastX)}")
      }
    }
  }

  def prettyPrint: String = {
    log.toIndexedSeq.toList.toJson.prettyPrint
  }

  def percentAcceptedOfType(id: String): Double = {
    percentAcceptedOfTypeLocal(id, log)
  }

  def percentAcceptedOfTypeLocal(id: String, localLog: ListBuffer[jsonLogFormat]): Double = {
    val filtered = localLog.filter(f => f.name == id)
    val accepted = filtered.filter(f => f.status)
    accepted.length.toDouble / filtered.length.toDouble
  }

  override def toString: String = {
    "# of Accepted: " + numOfAccepted + " = " + percentAccepted + "%\n" +
      "# of Rejected: " + numOfRejected + " = " + percentRejected + "%"
  }

}

object JSONStateLogger {
  import JsonLoggerProtocol._

  def loadLog(filePath: File): IndexedSeq[jsonLogFormat] = {
    println(s"Loading JSON log file: ${filePath.toString}")
    val src = Source.fromFile(filePath.toString)
    val data = src.mkString.parseJson.convertTo[IndexedSeq[jsonLogFormat]]
    src.close()
    data
  }

  // TODO: Convert JSON entry to State format
//  def getBestStateFromJSON[State <: GingrRegistrationState[State]](filePath: File): State = {
//    val loggerSeq = loadLog(filePath).filter(_.status)
//    val bestSample = loggerSeq.sortBy(f => f.logvalue("product")).reverse.head
//    bestSample
//  }
}
