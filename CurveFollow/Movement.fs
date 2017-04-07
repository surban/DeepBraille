module Movement

open System
open System.IO
open RProvider
open RProvider.graphics
open RProvider.grDevices
open Nessos.FsPickler
open Nessos.FsPickler.Json
open MathNet.Numerics

open Basics
open ArrayNDNS
open RTools



type XY = float * float
type TV = float * float

/// Braille coordinate system.
module BrailleCS =

    let dot_radius = 0.72       // mm
    let dot_height = 0.48       // mm
    let dot_dist = 2.34         // mm
    let char_dist = 6.2         // mm
    let line_dist = 10.0        // mm
    let x_offset = 22.67        // mm
    let y_offset = 18.5         // mm
    let hole1_x = 6.5           // mm
    let hole1_y = 31.0          // mm
    let hole1_radius = 3.2      // mm
    let hole2_x = 7.5           // mm
    let hole2_y = 111.5         // mm
    let hole2_radius = 3.3      // mm
    let label_x = 4.0           // mm
    let label_y = 42.0          // mm
    let label_thickness = 1.0   // mm
    let page_width = 206.0      // mm
    let page_height = 145.0     // mm
    let page_thickness = 0.4    // mm
    let clip_width = 10.0       // mm
    let clip_height = 1.0       // mm
    let clip_thickness = 6.0    // mm
    //let col_offset = 3.0        // chars
    let col_offset = -3.0        // chars
    let row_offset = 0.0        // lines

    /// Relative position of dot (0..5) to character in mm.
    let dotRelMM (dot: int) =
        assert (0 <= dot && dot < 6)
        let x = dot_dist * float (dot / 3) - dot_dist / 2.
        let y = dot_dist * float (dot % 3) - dot_dist
        x, y

    /// Braille character index to table position in mm.
    let charToMM (col: float, row: float) = 
        let x = char_dist * (col + col_offset) + x_offset
        let y = line_dist * (row + row_offset) + y_offset
        x, y

    /// Braille character index and dot number to table position in mm.
    let dotToMM (col, row) dot =
        let xc, yc = charToMM (col, row)
        let xd, yd = dotRelMM dot
        xc + xd, yc + yd


type DistortionCfg = {
    DistortionsPerSec:      float
    MaxOffset:              float
    MinHold:                float
    MaxHold:                float
    NotAgainFor:            float
}

type private DistortionState = 
    | InactiveUntil of float
    | GotoPos of float
    | HoldUntil of float


type RandomVelocityCfg = {
    HoldLength:         float
    MaxOffset:          float
    MaxYVel:            float
}

type private RandomVelocityState = {
    YVel:               float
    HoldUntil:          float
}

type Mode =
    | FixedOffset of float
    | Distortions of DistortionCfg
    | RandomVelocities of RandomVelocityCfg

type MovementCfg = {
    Dt:                 float
    Accel:              float
    VelX:               float
    MaxVel:             float
    MaxControlVel:      float
    Mode:               Mode
    IndentorPos:        float
    SkipFirstAndLast:   bool
}

type MovementPoint = {
    Time:           float
    Pos:            XY
    ControlVel:     XY
    CurveY:         float
    Distorted:      bool
}

type Movement = {
    StartPos:       XY
    IndentorPos:    float
    Accel:          float
    StartVelX:      float
    Points:         MovementPoint list
}

type RecordedMovementPoint = {
    Time:           float
    SimPos:         XY
    DrivenPos:      XY
    ControlVel:     XY
    YDist:          float
    Distorted:      bool
    Biotac:         float []
}

type RecordedMovement = {
    IndentorPos:    float
    Accel:          float
    Points:         RecordedMovementPoint list
}

let rng = Random()
let uniform lower upper = 
    lower + rng.NextDouble() * (upper - lower)
    


let generateMovement (cfg: MovementCfg) (rnd: System.Random) (curve: XY list) (xVels: TV list) = 
    let tblCfg = {
        XYTableSim.Accel  = cfg.Accel, cfg.Accel 
        XYTableSim.Dt     = cfg.Dt
        XYTableSim.MaxVel = cfg.MaxVel, cfg.MaxVel
    }

    let _, baseY = curve.[0]
    let startPos = 
        match cfg.Mode with
        | FixedOffset offset -> let x, y = curve.[0] in x, y + offset
        | _ -> curve.[0]   

    let controlPid = PID.Controller {
        PID.PFactor     = 4.0
        PID.IFactor     = 2.0
        PID.DFactor     = 1.0
        PID.ITime       = 0.05
        PID.DTime       = 0.05
    }
    let maxOffset = 9.6

    let handleFixedOffset ofst x y t cy slope () =
        let trgt = cy + ofst
        let trgt = min trgt (baseY + maxOffset)
        let trgt = max trgt (baseY - maxOffset)
        let yVel = controlPid.Simulate trgt y t
        yVel, abs ofst < 1e-3, ()

    let handleDistortions dc x y t cy slope distState =
        let cPos, nextState =
            match distState with
            | InactiveUntil iu ->               
                let prob = dc.DistortionsPerSec * cfg.Dt
                if x >= iu && rnd.NextDouble() < prob then
                    let trgt = cy + (2.0 * rnd.NextDouble() - 1.0) * dc.MaxOffset
                    let trgt = min trgt (baseY + maxOffset)
                    let trgt = max trgt (baseY - maxOffset)
                    cy, GotoPos trgt
                else cy, distState
            | GotoPos trgt ->
                if abs (y - trgt) < 0.05 then
                    let hu = x + dc.MinHold + rnd.NextDouble() * (dc.MaxHold - dc.MinHold)
                    trgt, HoldUntil hu
                else trgt, distState
            | HoldUntil hu ->
                if x >= hu then cy, InactiveUntil (x + dc.NotAgainFor)
                else y, distState

        let yVel = controlPid.Simulate cPos y t      
        let distorted = match distState with InactiveUntil _ -> false | _ -> true
        yVel, distorted, nextState

    let handleRandomVelocities rvCfg x y t cy slope rvState =
        let nextState =
            if x > rvState.HoldUntil || 
                    (abs (cy - y) > rvCfg.MaxOffset && sign rvState.YVel <> sign (cy-y)) then
                // Calculate allowable y velocities so that offset after HoldLength
                // is within MaxOffset. This is done using linear extrapolation
                // of the curve using the slope at current curve point.
                let vy delta =
                    (slope + (delta + cy - y) / rvCfg.HoldLength) * cfg.VelX
                let vyMin = min (vy rvCfg.MaxOffset) (vy -rvCfg.MaxOffset)
                let vyMax = max (vy rvCfg.MaxOffset) (vy -rvCfg.MaxOffset)
                let vy = uniform vyMin vyMax |> min rvCfg.MaxYVel |> max -rvCfg.MaxYVel
                //printfn "slope: %.3f  vyMin: %.3f  vyMax: %.3f  vy: %.3f" slope vyMin vyMax vy

                // Sample new y speed from allowable range
                {
                    YVel      = vy
                    HoldUntil = x + rvCfg.HoldLength
                }
            else rvState

        let yVel = nextState.YVel
        yVel, true, nextState

    let rec getXVel xVels t =
        match xVels with
        | (t1,v1) :: (t2,v2) :: _ when t1 <= t && t < t2 ->
            let fac = (t2 - t) / (t2 - t1)
            fac * v1 + (1. - fac) * v2, xVels
        | (t1,v1) :: _ when t < t1 -> cfg.VelX, xVels
        | _ :: rVels -> getXVel rVels t
        | [] -> cfg.VelX, xVels

    let rec generate curve xVels (state: XYTableSim.State) handler (handlerState: 'HandlerState) = seq {
        let x, y = state.Pos
        let t = state.Time
        let xVel, xVels = getXVel xVels t
        //printfn "time: %f x: %f y: %f  XVel: %f" t x y xVel
        match curve with
        | [] -> ()
        | (x1,y1) :: (x2,y2) :: _ when x1 <= x && x < x2 ->
            // interpolate curve points
            let fac = (x2 - x) / (x2 - x1)
            let cy = fac * y1 + (1. - fac) * y2
            let slope = (y2 - y1) / (x2 - x1)

            let yVel, distorted, nextHandlerState = handler x y t cy slope handlerState
            let cVel = xVel, yVel
            yield {
                Time        = state.Time
                Pos         = state.Pos
                ControlVel  = cVel
                CurveY      = cy
                Distorted   = distorted 
            }
            yield! generate curve xVels (XYTableSim.step cVel tblCfg state) handler nextHandlerState               

        | (x1,y1) :: _ when x < x1 ->
            // x position is left of curve start
            let cVel = xVel, 0.
            yield {
                Time        = state.Time
                Pos         = state.Pos
                ControlVel  = cVel
                CurveY      = y1
                Distorted   = false
            }            
            yield! generate curve xVels (XYTableSim.step cVel tblCfg state) handler handlerState

        | _ :: rCurve ->
            // move forward on curve
            yield! generate rCurve xVels state handler handlerState
    }

    let state = {XYTableSim.Time=0.; XYTableSim.Pos=startPos; XYTableSim.Vel=0., 0. }
    let movement = 
        match cfg.Mode with
        | FixedOffset ofst -> generate curve xVels state (handleFixedOffset ofst) ()
        | Distortions dc -> generate curve xVels state (handleDistortions dc) (InactiveUntil 0.3)
        | RandomVelocities rvCfg -> generate curve xVels state (handleRandomVelocities rvCfg) {YVel=0.; HoldUntil=0.}

    {
        StartPos    = startPos
        IndentorPos = cfg.IndentorPos
        Accel       = cfg.Accel
        StartVelX   = cfg.VelX
        Points      = movement |> Seq.toList
    }


let toDriveCurve (movement: Movement) = 
    {
        TactileCurve.IndentorPos = movement.IndentorPos
        TactileCurve.StartPos    = movement.StartPos
        TactileCurve.Accel       = movement.Accel
        TactileCurve.StartXVel   = movement.StartVelX     
        TactileCurve.Points      = [ for mp in movement.Points -> 
                                     {
                                        TactileCurve.Time = mp.Time
                                        TactileCurve.XPos = fst mp.Pos
                                        TactileCurve.YPos = snd mp.Pos
                                     } ]            
    }


let syncTactileCurve (tc: TactileCurve.TactileCurve) (m: Movement) =
    let rec syncPoints (tcPoints: TactileCurve.TactilePoint list) (mPoints: MovementPoint list) synced = 
        //printfn "tcPoints: %A   mPoints: %A" (List.head tcPoints) (List.head mPoints) 
        match tcPoints, mPoints with
        | [], _  
        | _, [] -> List.rev synced
        | ({Time=tct} as t)::tcRest, ({Time=mt} as m)::({Time=mtNext} as mNext)::_ when mt <= tct && tct < mtNext ->
            let fac = float (mtNext - tct) / float (mtNext - mt)
            let interp a b = 
                let xa, ya = a
                let xb, yb = b
                let x = fac * xa + (1.0 - fac) * xb
                let y = fac * ya + (1.0 - fac) * yb
                x, y
            let rp = {
                Time       = tct
                SimPos     = interp m.Pos mNext.Pos
                DrivenPos  = t.Pos
                ControlVel = interp m.ControlVel mNext.ControlVel
                YDist      = m.CurveY - snd t.Pos
                Distorted  = m.Distorted
                Biotac     = t.Biotac
            }
            syncPoints tcRest mPoints (rp::synced)
        | {Time=tct}::tcRest, {Time=mt}::_ when tct < mt ->
            syncPoints tcRest mPoints synced
        | _, _::mRest ->
            syncPoints tcPoints mRest synced
       
    let synced = syncPoints tc.Points m.Points []

    {
        IndentorPos = tc.IndentorPos
        Accel       = tc.Accel
        Points      = synced
    }

let loadCurves path =
    use file = NPZFile.Open path
    let pos: ArrayNDHostT<float> = file.Get "pos" // pos[dim, idx, smpl]
    seq { for smpl in 0L .. pos.Shape.[2] - 1L do
              yield [for idx in 0L .. pos.Shape.[1] - 1L do
                         // convert to mm
                         let col, row = pos.[[0L; idx; smpl]], pos.[[1L; idx; smpl]]
                         yield BrailleCS.charToMM (col, row) ] }  
    |> List.ofSeq
    
let toArray extract points = 
    let xAry = Array.zeroCreate (List.length points)
    let yAry = Array.zeroCreate (List.length points)
    for idx, pnt in List.indexed points do
        let x, y = extract pnt
        xAry.[idx] <- x
        yAry.[idx] <- y
    xAry, yAry
    

let plotMovement (path: string) (curve: XY list) (movement: Movement) =
    let curveX, curveY = toArray id curve
    let posX, posY = toArray (fun (p: MovementPoint) -> p.Pos) movement.Points
    let times =  movement.Points |> List.map (fun p -> p.Time) |> List.toArray
    let velY = (Array.pairwise times, Array.pairwise posY)
               ||> Array.map2 (fun (tb, ta) (b, a) -> (b - a) / (tb - ta))
               |> Array.append [| 0. |]
    let posX, posY = toArray (fun (p: MovementPoint) -> p.Pos) movement.Points
    let controlVelX, controlVelY = toArray (fun (p: MovementPoint) -> p.ControlVel) movement.Points
    let distorted = movement.Points |> List.map (fun p -> p.Distorted) |> Array.ofList

    R.pdf (path) |> ignore
    R.par2 ("oma", [0; 0; 0; 0])
    R.par2 ("mar", [3.2; 2.6; 1.0; 0.5])
    R.par2 ("mgp", [1.7; 0.7; 0.0])
    R.par2 ("mfrow", [6; 1])

    R.plot2 ([0; 150], [curveY.[0] - 10.; curveY.[0] + 10.], "curve", "x", "y")
    R.abline(h=curveY.[0]) |> ignore
    R.lines2 (curveX, curveY, "black")
    R.lines2 (posX, posY, "blue")
    R.legend (115., curveY.[0] + 10., ["curve"; "movement"], col=["black"; "blue"], lty=[1;1]) |> ignore

    R.plot2 ([0; 25], [0; 150], "position", "t", "x")
    R.lines2 (times, posX, "black")

    R.plot2 ([0; 25], [posY.[0] - 10.; posY.[0] + 10.], "position", "t", "y")
    R.lines2 (times, posY, "black")

    R.plot2 ([0; 150], [-20; 20], "velocity", "x", "y velocity")
    R.abline(h=0) |> ignore
    R.lines2 (posX, controlVelY, "blue")
    R.lines2 (posX, velY, "red")
    R.legend (125., 20, ["control"; "table"], col=["blue"; "red"], lty=[1;1]) |> ignore

    R.plot2 ([0; 25], [0; 20], "velocity", "t", "x")
    R.lines2 (times, controlVelX, "black")

    R.plot2 ([0; 25], [-20; 20], "velocity", "t", "y")
    R.lines2 (times, controlVelY, "black")

    R.dev_off() |> ignore


let plotTactile (path: string) (curve: XY list) (tactile: TactileCurve.TactileCurve) =
    let curveX, curveY = toArray id curve
    let drivenPosX, drivenPosY = tactile.Points |> toArray (fun p -> p.Pos) 
    let biotac = tactile.Points |> List.map (fun p -> p.Biotac)

    let dt = tactile.Points.[1].Time - tactile.Points.[0].Time
    let drivenVelY =
        drivenPosY
        |> Array.toSeq
        |> Seq.pairwise
        |> Seq.map (fun (a, b) -> (b - a) / dt)
        |> Seq.append (Seq.singleton 0.)
        |> Array.ofSeq

    R.pdf (path) |> ignore
    R.par2 ("oma", [0; 0; 0; 0])
    R.par2 ("mar", [3.2; 2.6; 1.0; 0.5])
    R.par2 ("mgp", [1.7; 0.7; 0.0])
    R.par2 ("mfrow", [2; 1])

    R.plot2 ([0; 150], [curveY.[0] - 6.; curveY.[0] + 6.], "position", "x", "y")
    R.abline(h=curveY.[0]) |> ignore
    R.lines2 (curveX, curveY, "black")
    R.lines2 (drivenPosX, drivenPosY, "yellow")
    R.legend (115., curveY.[0] + 6., ["curve"; "driven"], col=["black"; "yellow"], lty=[1;1]) |> ignore

    R.plot2 ([0; 150], [-20; 20], "velocity", "x", "y velocity")
    R.abline(h=0) |> ignore
    R.lines2 (drivenPosX, drivenVelY, "yellow")
    R.legend (125., 20, ["driven"], col=["yellow"], lty=[1;1]) |> ignore

    R.dev_off() |> ignore


let plotRecordedMovement (path: string) (curve: XY list) (recMovement: RecordedMovement) (predDistY: float list option) =
    let curveX, curveY = toArray id curve
    let times = recMovement.Points |> List.map (fun p -> p.Time) |> List.toArray
    let simPosX, simPosY = recMovement.Points |> toArray (fun p -> p.SimPos) 
    let drivenPosX, drivenPosY = recMovement.Points |> toArray (fun p -> p.DrivenPos) 
    let controlVelX, controlVelY = recMovement.Points |> toArray (fun p -> p.ControlVel) 
    let distY = recMovement.Points |> List.map (fun p -> p.YDist) |> Array.ofList
    let distorted = recMovement.Points |> List.map (fun p -> p.Distorted) |> Array.ofList
    let biotac = recMovement.Points |> List.map (fun p -> p.Biotac)

    let dt = times.[1] - times.[0]
    let left, right = Array.min drivenPosX, Array.max drivenPosX
    let tLeft, tRight = Array.min times, Array.max times

    let posToVel pos =
        pos
        |> Array.toSeq
        |> Seq.pairwise
        |> Seq.map (fun (a, b) -> (b - a) / dt)
        |> Seq.append (Seq.singleton 0.)
        |> Array.ofSeq    
    let drivenVelX, drivenVelY = posToVel drivenPosX, posToVel drivenPosY

    R.pdf (path) |> ignore
    R.par2 ("oma", [0; 0; 0; 0])
    R.par2 ("mar", [3.2; 2.6; 1.0; 0.5])
    R.par2 ("mgp", [1.7; 0.7; 0.0])
    R.par2 ("mfrow", [4; 1])

    R.plot2 ([left; right], [curveY.[0] - 10.; curveY.[0] + 10.], "position", "x", "y")
    R.abline(h=curveY.[0]) |> ignore
    R.lines2 (curveX, curveY, "black")
    R.lines2 (simPosX, simPosY, "blue")
    R.lines2 (drivenPosX, drivenPosY, "yellow")
    R.legend (125., curveY.[0] + 10., ["curve"; "movement"; "driven"], col=["black"; "blue"; "yellow"], lty=[1;1]) |> ignore
    
    R.plot2 ([left; right], [-15; 15], "velocity", "x", "y velocity")
    R.abline(h=0) |> ignore
    R.lines2 (drivenPosX, controlVelY, "blue")
    R.lines2 (drivenPosX, drivenVelY, "yellow")
    R.legend (125., 15, ["control"; "driven"], col=["blue"; "yellow"], lty=[1;1]) |> ignore

    R.plot2 ([left; right], [-8; 8], "distance to curve", "x", "y distance")
    R.abline(h=0) |> ignore
    R.lines2 (drivenPosX, distY, "blue")
    match predDistY with
    | Some predDistY -> 
        R.lines2 (drivenPosX, predDistY, "red")
        R.legend (125., 8, ["true"; "predicted"], col=["blue"; "red"], lty=[1;1]) |> ignore
    | None -> ()

    // plot biotac
    let biotacImg = array2D biotac |> ArrayNDHost.ofArray2D |> ArrayND.transpose  // [chnl, smpl]
    let minVal, maxVal = ArrayND.minAxis 1 biotacImg, ArrayND.maxAxis 1 biotacImg
    let scaledImg = (biotacImg - minVal.[*, NewAxis]) / (maxVal - minVal).[*, NewAxis]
    R.image2 (ArrayNDHost.toArray2D scaledImg, lim=(0.0, 1.0),
              xlim=(left, right), colormap=Gray, title="biotac", xlabel="x", ylabel="channel")

    R.dev_off() |> ignore

let plotRecordedMovement2 (path: string) (curve: XY list) (recMovement: RecordedMovement) (predDistY: float list option) =
    let curveX, curveY = toArray id curve
    let times = recMovement.Points |> List.map (fun p -> p.Time) |> List.toArray
    let simPosX, simPosY = recMovement.Points |> toArray (fun p -> p.SimPos) 
    let drivenPosX, drivenPosY = recMovement.Points |> toArray (fun p -> p.DrivenPos) 
    let controlVelX, controlVelY = recMovement.Points |> toArray (fun p -> p.ControlVel) 
    let distY = recMovement.Points |> List.map (fun p -> p.YDist) |> Array.ofList
    let distorted = recMovement.Points |> List.map (fun p -> p.Distorted) |> Array.ofList
    let biotac = recMovement.Points |> List.map (fun p -> p.Biotac)

    let dt = times.[1] - times.[0]
    let left, right = Array.min drivenPosX, Array.max drivenPosX
    let tLeft, tRight = Array.min times, Array.max times

    let posToVel pos =
        pos
        |> Array.toSeq
        |> Seq.pairwise
        |> Seq.map (fun (a, b) -> (b - a) / dt)
        |> Seq.append (Seq.singleton 0.)
        |> Array.ofSeq    
    let drivenVelX, drivenVelY = posToVel drivenPosX, posToVel drivenPosY

    R.pdf (path) |> ignore
    R.par2 ("oma", [0; 0; 0; 0])
    R.par2 ("mar", [3.2; 2.6; 1.0; 0.5])
    R.par2 ("mgp", [1.7; 0.7; 0.0])
    R.par2 ("mfrow", [3; 1])

    R.plot2 ([tLeft; tRight], [left; right], "position", "t", "x")
    R.lines2 (times, drivenPosX, "black")

    R.plot2 ([tLeft; tRight], [0; 25], "velocity", "t", "vx")
    R.lines2 (times, drivenVelX, "black")
    R.lines2 (times, controlVelX, "yellow")

    // plot biotac
    let biotacImg = array2D biotac |> ArrayNDHost.ofArray2D |> ArrayND.transpose  // [chnl, smpl]
    let minVal, maxVal = ArrayND.minAxis 1 biotacImg, ArrayND.maxAxis 1 biotacImg
    let scaledImg = (biotacImg - minVal.[*, NewAxis]) / (maxVal - minVal).[*, NewAxis]
    R.image2 (ArrayNDHost.toArray2D scaledImg, lim=(0.0, 1.0),
              xlim=(left, right), colormap=Gray, title="biotac", xlabel="x", ylabel="channel")

    R.dev_off() |> ignore


let loadXVels dir = 
    seq {
        for path in Directory.EnumerateFiles(dir, "*.h5") do
            let velName = Path.GetFileNameWithoutExtension path
            use velHdf = HDF5.OpenRead path
            let velData : ArrayNDHostT<float> = ArrayNDHDF.read velHdf "vels"
            let ts = velData.[*, 0L] |> ArrayNDHost.toList
            let vs = velData.[*, 1L] |> ArrayNDHost.toList
            let tv = List.zip ts vs
            yield velName, tv
    } |> Map.ofSeq

let generateMovementForFile cfgs path outDir xVelDir =
    let rnd = Random ()
    let baseName = Path.Combine(Path.GetDirectoryName path, Path.GetFileNameWithoutExtension path)
    let curves = loadCurves path
    use curveHdf = HDF5.OpenWrite (baseName + ".h5")

    for curveIdx, curve in List.indexed curves do      
        // write curve to HDF5 file
        let ary = ArrayNDHost.zeros [List.length curve |> int64; 2L]
        for idx, (x, y) in List.indexed curve do
            ary.[[int64 idx; 0L]] <- x
            ary.[[int64 idx; 1L]] <- y
        ArrayNDHDF.write curveHdf (sprintf "curve%d" curveIdx) ary

        // load x velocities
        let xVels = 
            match xVelDir with
            | Some xVelDir -> loadXVels (Path.Combine(xVelDir, sprintf "curve%d" curveIdx))
            | None -> Map ["", []]
                
        for KeyValue(xVelName, xVel) in xVels do
            // generate movements
            for cfgIdx, cfg in List.indexed cfgs do
                let dir = Path.Combine(outDir, sprintf "Curve%d%sCfg%d" curveIdx xVelName cfgIdx)
                Directory.CreateDirectory dir |> ignore

                let movement = generateMovement cfg rnd curve xVel
                plotMovement (Path.Combine (dir, "movement.pdf")) curve movement

                if cfg.SkipFirstAndLast && (curveIdx = 0 || curveIdx = 6) then
                    ()
                else
                    let p = FsPickler.CreateBinarySerializer()
                    use tw = File.OpenWrite(Path.Combine (dir, "movement.dat"))
                    p.Serialize(tw, movement)
                    use tw = File.OpenWrite(Path.Combine (dir, "curve.dat"))
                    p.Serialize(tw, curve)
            
type GenCfg = {
    CurveDir:           string
    XVelDir:            string option
    MovementDir:        string
    MovementCfgs:       MovementCfg list
}

let generateMovementUsingCfg cfg  =
    for file in Directory.EnumerateFiles(cfg.CurveDir, "*.cur.npz") do
        printfn "%s" (Path.GetFullPath file)
        let baseName = Path.GetFileNameWithoutExtension file
        let outDir = Path.Combine(cfg.MovementDir, baseName)
        let xVelDir = cfg.XVelDir |> Option.map (fun xVelDir -> Path.Combine(xVelDir, baseName))
        generateMovementForFile cfg.MovementCfgs file outDir xVelDir


/// Records data for all */movement.json files in the given directory.
let recordMovements dir =
    let p = FsPickler.CreateJsonSerializer(indent=true)
    let bp = FsPickler.CreateBinarySerializer()
    
    for subDir in Directory.EnumerateDirectories dir do
        let movementFile = Path.Combine (subDir, "movement.dat")
        let recordFile = Path.Combine (subDir, "recorded.dat")
        if File.Exists movementFile && 
                (not (File.Exists recordFile) || File.GetLastWriteTime movementFile > File.GetLastWriteTime recordFile) then
            printfn "%s" movementFile
            use tr = File.OpenRead movementFile
            let movement : Movement = bp.Deserialize tr
            use tr = File.OpenRead (Path.Combine (subDir, "curve.dat"))
            let curve : XY list = bp.Deserialize tr

            let driveCurve = toDriveCurve movement
            let tactileCurve = TactileCurve.record driveCurve
            use tw = File.OpenWrite (Path.Combine (subDir, "tactile.json"))
            p.Serialize (tw, tactileCurve)
            plotTactile (Path.Combine (subDir, "tactile.pdf")) curve tactileCurve

            let recMovement = syncTactileCurve tactileCurve movement
            use tw = File.Create recordFile
            bp.Serialize (tw, recMovement)
            plotRecordedMovement (Path.Combine (subDir, "recorded.pdf")) curve recMovement None
            plotRecordedMovement2 (Path.Combine (subDir, "recorded2.pdf")) curve recMovement None


/// Linearly interpolates the samples at the given times.
/// The interpolation function ipFunc must linearly interpolate according to
/// let ipFunc fac a b = fac * b + (1. - fac) * a.
let rec interpolate ipFunc times samples = 
    match times, samples with
    | [], _ -> []
    | t::rTimes, (ta, a)::(tb, b)::_ when ta <= t && t < tb ->
        let fac = (t - ta) / (tb - ta) 
        let s = ipFunc fac a b
        (t, s)::(interpolate ipFunc rTimes samples)
    | t::_, [] -> failwith "cannot extrapolate"
    | t::_, (ta, _)::_ when t < ta -> failwith "cannot extrapolate"
    | _, _::rSamples -> interpolate ipFunc times rSamples


type CNNDatasetCfg = {

    /// directory of recorded movements of a Braille pages (contains *.cur/*/recorded.dat and *.dot)
    MovementDir:     string
    /// partitions of the data set
    Partitions:      Map<string, string list>
    /// output file (HDF5)
    OutFile:         string

    /// start column of valid tactile data in mm
    StartCol:           float 
    /// end column of valid tactile data in mm
    EndCol:             float
    /// column step in mm (is 0.02 mm in recorded data)
    ColRes:             float
    /// Number of steps per sample.
    /// If specified, one row will be cut into multiple, overlapping samples
    /// with the given number of steps.
    NSteps:             int option

    /// number of tactile data steps to cut from the left for the target dot values 
    TargetLeftCut:      int
    /// number of tactile data steps to cut from the right for the target dot values 
    TargetRightCut:     int
    /// down-sampling factor for the target dot values
    TargetDownsample:   int

    /// distance between lines in mm (is 10.0)
    LineDist:           float
    /// x position of center of first character in mm 
    XOffset:            float
    /// y position of first line in mm (is 18.5)
    YOffset:            float
    /// radius of Braille dot in mm (is 0.72)
    DotRadius:          float
    /// distance between centers of Braille dots in mm (is 2.34)
    DotDist:            float
    /// distance between corresponding dots of chars in mm (is 6.2)
    CharDist:           float

    
}

/// reads a .dot file
let readDots filename = [
    let lines = File.ReadAllLines filename
    for line in lines do
        yield [
            for char in line.Split(' ') do
                let value = char |> Seq.map (fun c -> c = '1') |> Array.ofSeq
                yield value
        ]
]     

/// Renders a line from dot data.
let renderDotLine (cfg: CNNDatasetCfg) nSteps (dotLine: bool[] list)  =

    // calculate dot pixels in same resolution as tactile data 
    let dotPos = ResizeArray()
    let pixels = ArrayNDHost.zeros<float> [nSteps; 3L]         // [x, y]
    let xStepWidth = (cfg.EndCol - cfg.StartCol) / float nSteps

    for charIdx, dots in List.indexed dotLine do
        let charX = float charIdx * cfg.CharDist + cfg.XOffset

        for dotIdx, isOn in Seq.indexed dots do
            let dotX = charX + float (dotIdx / 3) * cfg.DotDist - cfg.DotDist / 2.
            let dotY = dotIdx % 3

            let dotXStart = dotX - cfg.DotRadius
            let dotXEnd = dotX + cfg.DotRadius

            if (cfg.StartCol <= dotXStart && dotXStart < cfg.EndCol) && 
                (cfg.StartCol <= dotXEnd && dotXEnd < cfg.EndCol) then

                let dotXMinIdx = (dotXStart - cfg.StartCol) / xStepWidth |> round |> int
                let dotXMaxIdx = (dotXEnd - cfg.StartCol) / xStepWidth |> round |> int
                dotPos.Add((charIdx, dotIdx, dotXMinIdx, dotXMaxIdx, dotY))

                if isOn then
                    pixels.[dotXMinIdx .. dotXMaxIdx, dotY] <- ArrayNDHost.scalar 1.0

    // then cutoff from left and right and down-sample using averaging
    let cutPixels = pixels.[cfg.TargetLeftCut .. nSteps - 1L - (int64 cfg.TargetRightCut), Fill]
    let dsPixels = 
        ArrayNDHost.initIndexed 
            [cutPixels.Shape.[0] / (int64 cfg.TargetDownsample); 3L]
            (function 
             | [dsCol; dsRow] -> 
                [for i=0 to cfg.TargetDownsample-1 do
                    yield cutPixels.[[dsCol * (int64 cfg.TargetDownsample) + (int64 i); int64 dsRow]]]
                |> List.average
             | _ -> failwith "impossible")

    dsPixels, Seq.toList dotPos


/// Builds a dataset from multiple directories using a builder and a saver function.
let buildMultidirData builder saver movementDir partitions =
    for KeyValue(partition, pages) in partitions do
        // get directories and dot files
        let dirs = seq {
            for page in pages do
                let dotPath = Path.Combine (movementDir, page + ".dot")
                let curPath = Path.Combine (movementDir, page + ".cur")
                if Directory.Exists curPath then
                    yield curPath, dotPath
        }

        // acquire data for each directory and dot file
        let dirDatas =
            dirs
            |> Seq.collect (fun (curPath, dotPath) -> builder curPath dotPath)
            |> List.ofSeq

        // save data
        if not (List.isEmpty dirDatas) then
            saver partition dirDatas


/// CNN data sample(s)
type CNNData = {
    Biotac:         ArrayNDHostT<float>
    XPos:           ArrayNDHostT<float>
    Dots:           ArrayNDHostT<float>
    Cut:            int
    DotPos:         (int * int * int * int * int) list
}


/// Extracts data from recorded movements suitable for learning braille characters with CNNs.
let buildCNNData (cfg: CNNDatasetCfg)  =
    use hdf = HDF5.OpenWrite cfg.OutFile

    let builder recMovementDir dotFile =
        let bp = FsPickler.CreateBinarySerializer()

        // read dot file
        let allDots = readDots dotFile
  
        // read recorded data and interpolate
        let xPos = [cfg.StartCol .. cfg.ColRes .. cfg.EndCol]
        let nSteps = List.length xPos |> int64
        let allCurveSamples = [
            for subDir in Directory.EnumerateDirectories recMovementDir do
                let recordFile = Path.Combine (subDir, "recorded.dat")
                if File.Exists recordFile then
                    use fr = File.OpenRead recordFile
                    let recMovement : RecordedMovement = bp.Deserialize fr

                    let timeSamples = 
                        recMovement.Points
                        |> List.map (fun rmp -> fst rmp.DrivenPos, rmp.Biotac)
                    let ipFunc fac = Array.map2 (fun aEl bEl -> fac * bEl + (1. - fac) * aEl)
                    let xSamples = interpolate ipFunc xPos timeSamples

                    // get associated dot line
                    let _, y = recMovement.Points.Head.DrivenPos
                    let row = (y - cfg.YOffset) / cfg.LineDist |> round |> int

                    yield xSamples, allDots.[row]
        ]

        if List.isEmpty allCurveSamples then
            Seq.empty
        else
            // build dataset arrays
            let nSamples = List.length allCurveSamples |> int64
            let nPos = List.length xPos |> int64
            let nChannels = allCurveSamples |> List.head |> fst |> List.head |> snd |> Array.length |> int64
            let dotPixelsShape = allCurveSamples |> List.head |> snd |> renderDotLine cfg nSteps |> fst |> ArrayND.shape
            let nDotX, nDotY = dotPixelsShape.[0], dotPixelsShape.[1]

            // [smpl, xpos, channel]
            let biotac = ArrayNDHost.zeros [nSamples; nPos; nChannels]
            // [smpl, xpos]
            let xpos = ArrayNDHost.zeros [nSamples; nPos]
            // [smpl, dot_xpos, dot_ypos]
            let dots = ArrayNDHost.zeros [nSamples; nDotX; nDotY]

            let fstDotLine = allCurveSamples |> Seq.head |> snd
            // dotPos: (col, dot, startXIdx, endXIdx) list
            let _, dotPosInfo = renderDotLine cfg nSteps fstDotLine

            for smplIdx, (smpl, dotLine) in Seq.indexed allCurveSamples do
                dots.[smplIdx, Fill, Fill] <- renderDotLine cfg nSteps dotLine |> fst
                for xPosIdx, (xPos, channels) in Seq.indexed smpl do
                    xpos.[[int64 smplIdx; int64 xPosIdx]] <- xPos
                    biotac.[smplIdx, xPosIdx, Fill] <- channels |> ArrayNDHost.ofArray           


            match cfg.NSteps with
            | None ->
                Seq.singleton {
                    Biotac = biotac
                    XPos   = xpos
                    Dots   = dots
                    Cut    = 0
                    DotPos = dotPosInfo
                }
            | Some nSteps ->
                // now one sample was generated for the whole line
                // it needs to be cut into samples with configured number of steps

                let cutNSteps = nSteps - cfg.TargetLeftCut - cfg.TargetRightCut
                if cutNSteps % cfg.TargetDownsample <> 0 then
                    printfn "(NSteps - TargetLeftCut - TargetRightCut) = %d is not \
                               a multiple of TargetDownsample" cutNSteps
                let dotsPerCutSample = cutNSteps / cfg.TargetDownsample

                seq {
                    let mutable biotacPos = 0L
                    let mutable dotPos = 0L
                    let mutable nCuts = 0
                    for smpl in  0L .. nSamples - 1L do
                        biotacPos <- 0L
                        dotPos <- 0L
                        nCuts <- 0         
                        while biotacPos + int64 nSteps <= int64 nPos do
                            yield {
                                Biotac = biotac.[smpl .. smpl, biotacPos .. biotacPos + int64 nSteps - 1L, *]
                                XPos   = xpos.[smpl .. smpl, biotacPos .. biotacPos + int64 nSteps - 1L]
                                Dots   = dots.[smpl .. smpl, dotPos .. dotPos + int64 dotsPerCutSample - 1L, *]
                                Cut    = nCuts
                                DotPos = dotPosInfo
                            }

                            biotacPos <- biotacPos + int64 cutNSteps
                            dotPos <- dotPos + int64 dotsPerCutSample
                            nCuts <- nCuts + 1

                    let usedSteps = int biotacPos + cfg.TargetLeftCut + cfg.TargetRightCut
                    let lostSteps = int nPos - usedSteps
                    let lostDots = nDotX - dotPos
                    printfn "Each Braille line of %d steps (used: %d) was cut in %d samples of %d steps each."
                        biotac.Shape.[1] usedSteps nCuts nSteps
                    if lostSteps > 0 then
                        printfn "Lost %d biotac steps (%.2f %%) and %d dot columns (%.2f %%) \
                                 per Braile line due to cutting."
                            lostSteps (float lostSteps / float nPos * 100.) 
                            lostDots (float lostDots / float nDotX * 100.)                
                }       

    let saver partition dirDatas =
        // calculate number of cuts
        let nCuts =
            dirDatas
            |> List.map (fun d -> d.Cut + 1)
            |> List.max

        // concatenate data
        let concat extractor = dirDatas |> List.map extractor |> ArrayND.concat 0 
        let data = {
            Biotac = concat (fun d -> d.Biotac)
            XPos   = concat (fun d -> d.XPos)
            Dots   = concat (fun d -> d.Dots)
            Cut    = 0
            DotPos = dirDatas.Head.DotPos
        }

        // convert dot position list to array
        let dotPosAry = ArrayNDHost.zeros<int> [int64 data.DotPos.Length; 5L]
        for idx, (col, dot, startIdx, stopIdx, row) in Seq.indexed data.DotPos do
            dotPosAry.[idx, *] <- [col; dot; startIdx; stopIdx; row] |> ArrayNDHost.ofList

        // save as HDF5
        ArrayNDHDF.write hdf (partition + "/biotac") data.Biotac
        ArrayNDHDF.write hdf (partition + "/xpos") data.XPos
        ArrayNDHDF.write hdf (partition + "/dots") data.Dots
        ArrayNDHDF.write hdf (partition + "/ncuts") (ArrayNDHost.scalar nCuts)
        ArrayNDHDF.write hdf (partition + "/dotpos") dotPosAry

    buildMultidirData builder saver cfg.MovementDir cfg.Partitions       


type WorldRNNDatasetCfg = {

    /// directory of recorded movements of a Braille pages (contains *.cur/*/recorded.dat and *.dot)
    MovementDir:     string
    /// partitions of the data set
    Partitions:      Map<string, string list>
    /// output file (HDF5)
    OutFile:         string

    /// start column of valid tactile data in mm
    StartCol:           float 
    /// end column of valid tactile data in mm
    EndCol:             float
    /// column step in mm (is 0.02 mm in recorded data)
    ColRes:             float

    /// length of the history part of a sample in mm
    HistoryCols:        float
    /// length of the prediction part of a sample in mm
    PredCols:           float
    /// offset from the beginning of one sample to the beginning of the next in mm
    StrideCols:         float    
}

type WorldRNNData = {
    Biotac:         ArrayNDHostT<float>    // [smpl, step, ch]     for cfg.HistoryCols / cfg.ColRes steps
    Vel:            ArrayNDHostT<float>    // [smpl, step, dim]    for [(cfg.HistoryCols + cfg.PredCols) / cfg.ColRes - 1] steps
    Offset:         ArrayNDHostT<float>    // [smpl, step, dim=0]  for (cfg.HistoryCols + cfg.PredCols) / cfg.ColRes steps
    Pos:            ArrayNDHostT<float>    // [smpl, step, dim]    for (cfg.HistoryCols + cfg.PredCols) / cfg.ColRes steps
    Time:           ArrayNDHostT<float>    // [smpl, step]
}

let buildWorldRNNData (cfg: WorldRNNDatasetCfg) =
    use hdf = HDF5.OpenWrite cfg.OutFile
    
    let builder recMovementDir dotFile =
        let bp = FsPickler.CreateBinarySerializer()
 
        // read recorded data and interpolate by position
        let xPos = [cfg.StartCol .. cfg.ColRes .. cfg.EndCol]
        let nSteps = List.length xPos
        let allCurveSamples = [
            for subDir in Directory.EnumerateDirectories recMovementDir do
                let recordFile = Path.Combine (subDir, "recorded.dat")
                if File.Exists recordFile then
                    use fr = File.OpenRead recordFile
                    let recMovement : RecordedMovement = bp.Deserialize fr

                    let samplesByPos = 
                        recMovement.Points |> List.map (fun rmp -> fst rmp.DrivenPos, rmp)
                    let ipFunc fac (a: RecordedMovementPoint) (b: RecordedMovementPoint) = 
                        let ipValue aEl bEl = fac * bEl + (1. - fac) * aEl 
                        let ipTuple aTup bTup = ipValue (fst aTup) (fst bTup), ipValue (snd aTup) (snd bTup)
                        {
                            RecordedMovementPoint.Biotac = Array.map2 ipValue a.Biotac b.Biotac
                            RecordedMovementPoint.ControlVel = ipTuple a.ControlVel b.ControlVel
                            RecordedMovementPoint.Distorted = a.Distorted
                            RecordedMovementPoint.DrivenPos = ipTuple a.DrivenPos b.DrivenPos
                            RecordedMovementPoint.SimPos = ipTuple a.SimPos b.SimPos
                            RecordedMovementPoint.Time = ipValue a.Time b.Time
                            RecordedMovementPoint.YDist = ipValue a.YDist b.YDist
                        }                   
                    let xSamples = interpolate ipFunc xPos samplesByPos
                    yield xSamples
        ]

        // calculate dimensions
        let nChannels = allCurveSamples |> List.head |> List.head |> snd |> fun rmp -> rmp.Biotac.Length
        let stride = int (cfg.StrideCols / cfg.ColRes)
        let historySteps = int (cfg.HistoryCols / cfg.ColRes)
        let predSteps = int (cfg.PredCols / cfg.ColRes)
        let wholeSteps = historySteps + predSteps

        // extract samples
        seq {
            for line in allCurveSamples do
                let line = List.toArray line
                for pos in [0 .. stride .. line.Length-1]  do
                    if pos + historySteps + predSteps <= line.Length then

                        let history = line.[pos .. pos+historySteps-1]
                        let whole = line.[pos .. pos+historySteps+predSteps-1]

                        yield {
                            WorldRNNData.Biotac = 
                                history
                                |> Seq.map (fun (_, rmp) -> ArrayNDHost.ofArray rmp.Biotac)
                                |> Seq.map (fun ary -> ary |> ArrayND.reshape [1L; int64 nChannels])
                                |> Seq.toList
                                |> ArrayND.concat 0
                                |> ArrayND.reshape [1L; int64 historySteps; int64 nChannels]
                            WorldRNNData.Vel =
                                whole
                                |> Seq.pairwise
                                |> Seq.map (fun ((_, rmp2), (_, rmp1)) ->
                                    (fst rmp2.DrivenPos - fst rmp1.DrivenPos) / (rmp2.Time - rmp1.Time),
                                    (snd rmp2.DrivenPos - snd rmp1.DrivenPos) / (rmp2.Time - rmp1.Time))      
                                |> Seq.map (fun (vx, vy) ->  ArrayNDHost.ofList [vx; vy])
                                |> Seq.map (fun ary -> ary |> ArrayND.reshape [1L; 2L])
                                |> Seq.toList
                                |> ArrayND.concat 0
                                |> ArrayND.reshape [1L; int64 (wholeSteps-1); 2L]
                            WorldRNNData.Offset = 
                                whole
                                |> Seq.map (fun (_, rmp) -> ArrayNDHost.scalar rmp.YDist)
                                |> Seq.map (fun ary -> ary |> ArrayND.reshape [1L; 1L])
                                |> Seq.toList
                                |> ArrayND.concat 0
                                |> ArrayND.reshape [1L; int64 wholeSteps; 1L]
                            WorldRNNData.Pos = 
                                whole
                                |> Seq.map (fun (_, rmp) -> 
                                    ArrayNDHost.ofList [fst rmp.DrivenPos; snd rmp.DrivenPos])
                                |> Seq.map (fun ary -> ary |> ArrayND.reshape [1L; 2L])
                                |> Seq.toList
                                |> ArrayND.concat 0
                                |> ArrayND.reshape [1L; int64 wholeSteps; 2L]
                            WorldRNNData.Time =
                                whole
                                |> Seq.map (fun (_, rmp) -> ArrayNDHost.scalar rmp.Time)
                                |> Seq.map (fun ary -> ary |> ArrayND.reshape [1L; 1L])
                                |> Seq.toList
                                |> ArrayND.concat 0
                                |> ArrayND.reshape [1L; int64 wholeSteps]
                        }        
        }

    let saver partition dirDatas =
        // concatenate data and save as HDF5
        let concat extractor = dirDatas |> List.map extractor |> ArrayND.concat 0 
        concat (fun d -> d.Biotac) |> ArrayNDHDF.write hdf (partition + "/biotac") 
        concat (fun d -> d.Vel)    |> ArrayNDHDF.write hdf (partition + "/vel") 
        concat (fun d -> d.Offset) |> ArrayNDHDF.write hdf (partition + "/offset") 
        concat (fun d -> d.Pos)    |> ArrayNDHDF.write hdf (partition + "/pos") 
        concat (fun d -> d.Time)   |> ArrayNDHDF.write hdf (partition + "/time") 

    buildMultidirData builder saver cfg.MovementDir cfg.Partitions


let plotRecordedMovements dir =
    let bp = FsPickler.CreateBinarySerializer()
    
    for subDir in Directory.EnumerateDirectories dir do
        let recordedFile = Path.Combine (subDir, "recorded.dat")
        if File.Exists recordedFile then
            printfn "%s" recordedFile
            use tr = File.OpenRead recordedFile
            let recMovement : RecordedMovement = bp.Deserialize tr
            use tr = File.OpenRead (Path.Combine (subDir, "curve.dat"))
            let curve : XY list = bp.Deserialize tr

            plotRecordedMovement (Path.Combine (subDir, "recorded.pdf")) curve recMovement None
            plotRecordedMovement2 (Path.Combine (subDir, "recorded2.pdf")) curve recMovement None

            
