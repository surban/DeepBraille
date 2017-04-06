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
    Movement.SkipFirstAndLast = false
}


let cfg = {
    CurveDir        = Config.baseDir + "/Data/DeepBraille/Curves/even1"
    MovementDir     = Config.baseDir + "/Data/DeepBraille/Movements/RandomVelocity-Even1-Set"
    XVelDir         = Some (Config.baseDir + "/Data/DeepBraille/XVels/Set1")
    MovementCfgs    = 
        [
            baseMovement
        ]
}
