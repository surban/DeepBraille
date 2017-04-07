module TactileCurve

open System.Diagnostics
open System

open Basics
open Datasets
open BRML.Drivers
open SampleRecorder


type XY = float * float

/// Drive point.
type DrivePoint = {
    Time:               float
    XPos:               float
    YPos:               float
}

/// Curve to record.
type DriveCurve = {
    IndentorPos:        float
    StartPos:           XY
    Accel:              float
    StartXVel:          float
    Points:             DrivePoint list
}

/// A tactile point.
type TactilePoint = {
    Time:               float
    Pos:                XY
    Biotac:             float []
}

/// A tactile data curve.
type TactileCurve = {
    IndentorPos:        float
    Accel:              float
    Points:             TactilePoint list
}


/// Records a tactile curve given a drive curve.
let record (curve: DriveCurve) =

    let gotoStart withDown = 
        async {
            do! Devices.Linmot.DriveTo Devices.LinmotUpPos
            do! Devices.XYTable.DriveTo curve.StartPos
            if withDown then
                do! Devices.Linmot.DriveTo curve.IndentorPos
        } 
    gotoStart true |> Async.RunSynchronously   

    Devices.XYTable.PosReportInterval <- 2
    let sensors = [Devices.XYTable :> ISensor; Devices.Biotac :> ISensor]
    let recorder = Recorder<TactilePoint> sensors

    let sw = Stopwatch()
    let xPidCntrl = PID.Controller XYTableSim.pidCfg
    let yPidCntrl = PID.Controller XYTableSim.pidCfg
    let rec pidControl (points: DrivePoint list) =
        let t = (float sw.ElapsedMilliseconds) / 1000.
        let x, y = Devices.XYTable.CurrentPos 
        
        printf "t=%.3f s     x=%.3f mm     y=%.3f mm       \r" t x y
        if Console.KeyAvailable && Console.ReadKey().KeyChar = 'q' then
            Devices.XYTable.Stop()
            exit 0

        match points with
        | _ when x > 142. -> ()
        | {Time=ct; XPos=cx; YPos=cy} :: {Time=nct; XPos=ncx; YPos=ncy} ::_  when ct <= t && t < nct ->
            let fac = (nct - t) * (nct - ct)
            let xTrgt = fac * cx + (1.-fac) * ncx
            let yTrgt = fac * cy + (1.-fac) * ncy
            let xVel = xPidCntrl.Control xTrgt x
            let yVel = yPidCntrl.Control yTrgt y 
            Devices.XYTable.DriveWithVel ((xVel, yVel))
            pidControl points
        | {Time=ct} :: _ when t < ct ->
            Devices.XYTable.DriveWithVel ((curve.StartXVel, 0.0))
            pidControl points
        | _::rCurve ->
            pidControl rCurve
        | [] -> ()

    recorder.Start ()
    sw.Start()
    pidControl curve.Points
    recorder.Stop ()

    Devices.XYTable.Stop ()
    gotoStart false |> Async.Start
    
    printf "                                                            \r"
    recorder.PrintStatistics ()

    {
        IndentorPos = curve.IndentorPos
        Accel       = curve.Accel
        Points      = recorder.GetSamples None |> List.ofSeq
    }
    


