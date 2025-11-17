function results = run_all_tests()
%RUN_ALL_TESTS Add project paths and execute all MATLAB unit tests
    projectRoot = fileparts(fileparts(mfilename('fullpath')));
    addpath(fullfile(projectRoot, 'matlab'));
    addpath(fullfile(projectRoot, 'test'));

    results = runtests(fullfile(projectRoot, 'test'));
    disp(table(results));
end
