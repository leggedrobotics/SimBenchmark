classdef plotspec
    %PLOTSPEC plot spec of each simulator and solvers
    %   plot line marker, color etc.
    
    properties (Constant)
        RAIRAIRAI =                     {'-d', 'Rai',           [137, 165, 78] ./ 255};
        BULLETSEQUENCEIMPULSEBULLET =   {'-s', 'BtSeqImp',     [170, 70, 67] ./ 255};
        BULLETNNCGBULLET =              {'-*', 'BtNNCG',       [170, 70, 67] ./ 255};
        BULLETMLCPDANTZIGBULLET =       {'-o', 'BtDantzig',    [170, 70, 67] ./ 255};
        BULLETMLCPLEMKEBULLET =         {'-p', 'BtLemke',      [170, 70, 67] ./ 255};
        BULLETMLCPPGSBULLET =           {'-h', 'BtPGS',        [170, 70, 67] ./ 255};
        BULLETMULTIBODYBULLET =         {'-d', 'BtMultibody',  [170, 70, 67] ./ 255};
        ODESTANDARDODE =                {'-o', 'OdeStd',       [219, 132, 61] ./ 255};
        ODEQUICKODE =                   {'-*', 'OdeQuick',     [219, 132, 61] ./ 255};
        MUJOCOPGSEULER =                {'-s', 'MjcPGS',       [69, 114, 167] ./ 255};
        MUJOCOCGEULER =                 {'-*', 'MjcCG',        [69, 114, 167] ./ 255};
        MUJOCONEWTONEULER =             {'-o', 'MjcNewton',    [69, 114, 167] ./ 255};
        MUJOCOPGSRK4 =                  {'-d', 'MjcPGS-RK4',       [69, 114, 167] ./ 255};
        MUJOCOCGRK4 =                   {'-p', 'MjcCG-RK4',        [69, 114, 167] ./ 255};
        MUJOCONEWTONRK4 =               {'-h', 'MjcNewton-RK4',    [69, 114, 167] ./ 255};
        DARTDANTZIGDART =               {'-o', 'DartDantzig',  [128, 105, 155] ./ 255};
        DARTPGSDART =                   {'-*', 'DartPGS',      [128, 105, 155] ./ 255};
    end
    
%     methods
%         function obj = plotspec(inputArg1,inputArg2)
%             %PLOTSPEC Construct an instance of this class
%             %   Detailed explanation goes here
%             obj.Property1 = inputArg1 + inputArg2;
%         end
%         
%         function outputArg = method1(obj,inputArg)
%             %METHOD1 Summary of this method goes here
%             %   Detailed explanation goes here
%             outputArg = obj.Property1 + inputArg;
%         end
%     end
end

