#if !CONFIG
#I "../../CurveFollow/bin/Debug"
#endif 

#r "MLModels.dll"
#r "CurveFollow.exe"

open Movement
open Models


let baseMovement = {
    Movement.Dt             = 0.01
    Movement.IndentorPos    = -43.4      
    Movement.Accel          = 40.0 
    Movement.VelX           = 6.0 
    Movement.MaxVel         = 40.0
    Movement.MaxControlVel  = 15.0

    Movement.Mode           = Movement.FixedOffset 0.
    Movement.SkipFirstAndLast = true
}

let distortionMode = 
    Movement.RandomVelocities {
        RandomVelocityCfg.HoldLength        = 3.0
        RandomVelocityCfg.MaxOffset         = 4.0
        RandomVelocityCfg.MaxYVel           = 10.0
    }         



let cfg = {
    CurveDir        = Config.baseDir + "/Data/DeepBraille/Curves/curv2"
    MovementDir     = Config.baseDir + "/Data/DeepBraille/Movements/Curv2-RV1"
    MovementCfgs    = 
        [
            {baseMovement with Mode = distortionMode}
            {baseMovement with Mode = distortionMode}
            {baseMovement with Mode = distortionMode}
            {baseMovement with Mode = distortionMode}
            {baseMovement with Mode = distortionMode}
            {baseMovement with Mode = distortionMode}
            {baseMovement with Mode = distortionMode}
            {baseMovement with Mode = distortionMode}
            {baseMovement with Mode = distortionMode}
        ]
}
