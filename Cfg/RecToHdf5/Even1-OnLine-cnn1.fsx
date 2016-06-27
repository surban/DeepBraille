#if !CONFIG
#I "../../CurveFollow/bin/Debug"
#endif 

#r "MLModels.dll"
#r "CurveFollow.exe"

open Movement
open Models


let cfg : RecToHdf5Cfg = {
    MovementDir         = @"Z:\dev\Deep\DeepBraille\Data\DeepBraille\Movements\Even1-OnLine"
    Partitions          = ["trn", ["00"; "01"; "02"; "03"; "04"; "05"; "06"; "07"]
                           "val", ["00"]
                           "tst", ["00"]]
                          |> Map.ofSeq
    OutFile             = @"Z:\dev\deep-braille\datasets\cnn\Even1-OnLine\cnn1\data.h5"

    StartCol            = 10.
    EndCol              = 130.
    ColRes              = 0.05
    NSteps              = Some 533

    TargetLeftCut       = 76
    TargetRightCut      = 77
    TargetDownsample    = 4

    LineDist            = 10.0
    YOffset             = 18.5
    DotRadius           = 0.72
    DotDist             = 2.34
    CharDist            = 6.2
}
