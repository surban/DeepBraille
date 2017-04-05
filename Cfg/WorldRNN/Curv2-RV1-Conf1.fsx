#if !CONFIG
#I "../../CurveFollow/bin/Debug"
#endif 

#r "MLModels.dll"
#r "CurveFollow.exe"

open Movement
open Models


let cfg : WorldRNNDatasetCfg = {
    MovementDir         = @"Z:\dev\Deep\DeepBraille\Data\DeepBraille\Movements\Curv2-RV1"
    Partitions          = ["trn", ["00"; "01"; "02"; "03"; "04"; "05"; "06"; "07"]
                           "val", ["08"]
                           "tst", ["09"]]
                          |> Map.ofSeq
    OutFile             = @"Z:\dev\deep-braille\datasets\world_rnn\Curv2-RV1\conf1.h5"

    StartCol            = 10.
    EndCol              = 130.
    ColRes              = 0.05

    HistoryCols         = 3. * 6.2
    PredCols            = 1.5 // 6.  // 3. * 6.2
    StrideCols          = 1. * 6.2
}
