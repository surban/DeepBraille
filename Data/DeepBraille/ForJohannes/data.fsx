
#r "MLModels.dll"
#r "CurveFollow.exe"
open Movement
open Models
let cfg : CNNDatasetCfg = {
    MovementDir         = @"\\srv-file\nthome\surban\dev\Deep\DeepBraille\Data\DeepBraille\Movements\Even1-OnLine"
    Partitions          = ["trn", ["00"; "01"; "02"; "03"; "04"; "05"; "06"; "07"]
                           "val", ["08"]
                           "tst", ["09"]]
                          |> Map.ofSeq
    OutFile             = @"\\srv-file.brml.tum.de\nthome\surban\dev\Deep\DeepBraille\Data\DeepBraille\ForJohannes\data.h5"

    StartCol            = 10.
    EndCol              = 130.
    ColRes              = 0.05
    NSteps              = None

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
