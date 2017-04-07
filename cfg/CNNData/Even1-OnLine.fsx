#if !CONFIG
#I "../../CurveFollow/bin/Debug"
#endif 

#r "MLModels.dll"
#r "CurveFollow.exe"
open Movement
open Models

let cfg : CNNDatasetCfg = {
    MovementDir         = Config.baseDir + @"\Data\DeepBraille\Movements\Even1-OnLine"
    Partitions          = ["trn", ["00"; "01"; "02"; "03"; "04"; "05"; "06"; "07"]
                           "val", ["08"]
                           "tst", ["09"]]
                          |> Map.ofSeq
    OutFile             = Config.baseDir + @"\Data\DeepBraille\CNNData\Even1-OnLine.h5"

    StartCol            = 10.
    EndCol              = 130.
    ColRes              = 0.05
    NSteps              = None

    StartTime           = 0.5
    EndTime             = 22.5 // 24.5
    TimeRes             = 0.01

    TargetLeftCut       = 0
    TargetRightCut      = 0
    TargetDownsample    = 1

    LineDist            = 10.0
    XOffset             = 3.0
    YOffset             = 18.5
    DotRadius           = 0.72
    DotDist             = 2.34
    CharDist            = 6.2
}
