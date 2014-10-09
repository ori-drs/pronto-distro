function flags = mex_pkg_config(packages)
% flags = mex_pkg_config(packages)
% packages: space separated string of pods/cmake package config files
% flags: compile flags to be used with mex -
%  >> mex(mexfilename,mex_pkg_config(packages))

setenv('PKG_CONFIG_PATH',pods_get_pkgconfig_path);
[status,cflags]=system(['pkg-config --cflags ' packages]);
if (status==1)
    error(cflags)
end
cflags(end)=[];
cflags=regexprep(cflags,'-pthread','-lpthread');
cflags=regexprep(cflags,' m |^m',' -lm ');
cflags_str = ['"CFLAGS=\$CFLAGS' cflags '"'];


[status,lflags]=system(['pkg-config --libs --static ' packages]);
lflags(end)=[];
if (status==1)
    error(lflags)
end
lflags=regexprep(lflags,'-pthread','-lpthread');
lflags=regexprep(lflags,' m |^m ',' -lm ');
lflags = [lflags ' -Wl,-rpath,' pods_get_r_path];
lflags_str = [' "LDFLAGS=\$LDFLAGS ' lflags '"'];

flags = [cflags_str,lflags_str];


