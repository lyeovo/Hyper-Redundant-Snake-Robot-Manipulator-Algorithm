function plan = buildfile
%buildfile MATLAB Build Tool 配置（CI 入口）
%   用法：
%     matlab -batch "buildtool"             运行默认任务（check + test）
%     matlab -batch "buildtool check"       仅静态检查
%     matlab -batch "buildtool test"        仅全量回归（含 check 前置）
%     matlab -batch "buildtool clean"       清理
%
%   说明：本文件用 buildplan() 建空计划后手动注册任务（而非 buildplan(localfunctions)），
%   这样下方的 checkAction / testAction 只是内部辅助函数，不会各自变成同名 task。
import matlab.buildtool.tasks.CleanTask

plan = buildplan;

plan("clean") = CleanTask;

plan("check") = matlab.buildtool.Task( ...
    Description = "静态代码检查（排除 archive / vision 等归档与外仓目录）", ...
    Action = @checkAction);

plan("test") = matlab.buildtool.Task( ...
    Description = "运行全量回归 test_all（10 项，失败即红灯）", ...
    Action = @testAction);
plan("test").Dependencies = "check";

plan.DefaultTasks = ["check" "test"];
end

%% ---------- 任务实现 ----------
function checkAction(~)
%checkAction 静态检查：只分析本项目源码，跳过归档/外仓/数据目录
%   archive/   已归档死代码（含文件名非法的历史 QA 脚本，MATLAB 本就不认）
%   vision/    独立 git 仓库，参考不改
%   data/ 电控文件/ rl_pipeline/  数据与二进制产物，非 MATLAB 源码
    ex = {'archive', 'vision', '.git', 'data', '电控文件', 'rl_pipeline', ...
          '.workbuddy', '.reasonix'};
    d = dir('**/*.m');
    files = {};
    for k = 1:numel(d)
        p    = fullfile(d(k).folder, d(k).name);
        rel  = strrep(p, [pwd filesep], '');
        part = strsplit(rel, filesep);
        if ismember(part{1}, ex), continue; end
        files{end+1} = p; %#ok<AGROW>
    end
    fprintf('[check] 分析 %d 个 .m 文件（已排除 %s）\n', numel(files), strjoin(ex, ' / '));

    issues = codeIssues(files);
    sev    = string(issues.Issues.Severity);
    nErr   = sum(sev == "error");
    fprintf('[check] 错误 %d / 警告 %d / 提示 %d\n', ...
        nErr, sum(sev == "warning"), sum(sev == "info"));

    rows = issues.Issues(sev == "error", :);
    for i = 1:height(rows)
        fprintf('  [error] %s (行 %d): %s\n', ...
            rows.Location(i), rows.LineStart(i), rows.Description(i));
    end
    if nErr > 0
        error('buildfile:check', '静态检查发现 %d 个错误', nErr);
    end
    fprintf('[check] 通过\n');
end

function testAction(~)
%testAction 全量回归：test_all 内部已聚合失败数并在非 0 时抛错
    addpath('ArmSimulator2D');
    addpath('test');
    addpath('.');
    nfail = test_all();
    if nfail > 0
        error('buildfile:test', '%d 项测试失败', nfail);
    end
end
